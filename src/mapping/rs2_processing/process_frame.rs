use anyhow::bail;
use image::{Rgb, RgbImage};
///Low level processing for data from a realsense camera
use realsense_rust::frame::{ColorFrame, DepthFrame, FrameEx, PixelKind, PointsFrame};
use realsense_rust::stream_profile::DataError;
use realsense_sys::{
    rs2_create_frame_queue, rs2_create_pointcloud, rs2_create_temporal_filter_block,
    rs2_delete_frame_queue, rs2_delete_processing_block, rs2_error, rs2_frame, rs2_frame_queue,
    rs2_poll_for_frame, rs2_process_frame, rs2_processing_block, rs2_start_processing_queue,
    rs2_wait_for_frame,
};
use std::ptr::NonNull;
use std::time::Duration;

///Realsense frame processing
///Architecture modelled from the example decimation block - https://gitlab.com/tangram-vision/oss/realsense-rust/-/merge_requests/55/diffs?commit_id=a145ce0262f79f667b00ed29e3b081e00e258444#4243a5602f85e44e06d290e709da9d0217c9a4ee
#[derive(Debug)]
pub struct FrameProcBlock<T> {
    ///The processing block for the "Pointcloud" method
    processing_block: NonNull<rs2_processing_block>,

    ///The queue where processed frames are deposited
    processing_queue: NonNull<rs2_frame_queue>,

    ///Placeholder
    output: Option<T>,
}

pub trait FrameProc<T> {
    ///Wait for a frame to be processed
    fn wait(&mut self, timeout: Duration) -> Result<T, DataError>;
    ///Check to see if a frame has been processed
    fn poll(&mut self) -> Result<T, DataError>;
}

impl<T> Drop for FrameProcBlock<T> {
    ///Drop ('delete') the memory being used for the pointcloud block
    fn drop(&mut self) {
        unsafe {
            rs2_delete_frame_queue(self.processing_queue.as_ptr());
            rs2_delete_processing_block(self.processing_block.as_ptr());
        }
    }
}

impl FrameProcBlock<PointsFrame> {
    ///Create a new frame processing block
    pub fn make_raw_points_block(processing_queue_size: i32) -> Result<Self, DataError> {
        //Create the memory for the processing block and the queue
        let (processing_block, processing_queue) = unsafe {
            //Error pointer
            let mut err = std::ptr::null_mut::<rs2_error>();

            let ptr = rs2_create_pointcloud(&mut err);

            let queue_ptr = rs2_create_frame_queue(processing_queue_size, &mut err);

            rs2_start_processing_queue(ptr, queue_ptr, &mut err);

            (NonNull::new(ptr).unwrap(), NonNull::new(queue_ptr).unwrap())
        };

        Ok(Self {
            processing_block,
            processing_queue,
            output: None,
        })
    }

    pub fn make_tempflt_points_block(processing_queue_size: i32) -> Result<Self, DataError> {
        //Create the memory for the processing block and the queue
        let (processing_block, processing_queue) = unsafe {
            //Error pointer
            let mut err = std::ptr::null_mut::<rs2_error>();

            let ptr = rs2_create_temporal_filter_block(&mut err);

            let queue_ptr = rs2_create_frame_queue(processing_queue_size, &mut err);

            rs2_start_processing_queue(ptr, queue_ptr, &mut err);

            (NonNull::new(ptr).unwrap(), NonNull::new(queue_ptr).unwrap())
        };

        Ok(Self {
            processing_block,
            processing_queue,
            output: None,
        })
    }

    ///Process a given depth frame by adding it the queue (frame transfers ownership)
    pub fn queue(&mut self, frame: DepthFrame) -> Result<(), DataError> {
        unsafe {
            let mut err = std::ptr::null_mut::<rs2_error>();

            rs2_process_frame(
                self.processing_block.as_ptr(),
                frame.get_owned_raw().as_ptr(),
                &mut err,
            );
            Ok(())
        }
    }
}

//Note - no error checking as of yet!
impl FrameProc<PointsFrame> for FrameProcBlock<PointsFrame> {
    ///Wait until a frame is processed and return the frame on completion
    fn wait(&mut self, timeout: Duration) -> Result<PointsFrame, DataError> {
        unsafe {
            let mut err = std::ptr::null_mut::<rs2_error>();
            let timeout_millis = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);
            let point_frame =
                rs2_wait_for_frame(self.processing_queue.as_ptr(), timeout_millis, &mut err);

            Ok(PointsFrame::try_from(NonNull::new(point_frame).unwrap()).unwrap())
        }
    }

    ///Poll to see if the queue is ready to provide a processed frame
    fn poll(&mut self) -> Result<PointsFrame, DataError> {
        unsafe {
            let mut err = std::ptr::null_mut::<rs2_error>();
            let point_frame: *mut *mut rs2_frame = std::ptr::null_mut::<*mut rs2_frame>();
            //Poll the processing queue to see if it's ready - if not ready return a null pointer
            rs2_poll_for_frame(self.processing_queue.as_ptr(), point_frame, &mut err);
            Ok(PointsFrame::try_from(NonNull::new(*point_frame).unwrap()).unwrap())
        }
    }
}

///Creates an image from a given color frame
pub fn create_color_image(frame: &ColorFrame) -> Result<RgbImage, anyhow::Error> {
    let width = frame.width();
    let height = frame.height();

    //Create image
    let mut image = RgbImage::new(width as u32, height as u32);

    for i in 0..height {
        for j in 0..width {
            let pixel_rgb: [u8; 3];
            let curr_pixel = frame.get(j, i).unwrap();

            //Check the type of the pixel
            match curr_pixel {
                PixelKind::Bgr8 { b, g, r } => pixel_rgb = [*r, *g, *b],
                _ => {
                    bail!("Unexpected pixel type!")
                }
            }

            image.put_pixel(j as u32, i as u32, Rgb(pixel_rgb));
        }
    }

    Ok(image)
}

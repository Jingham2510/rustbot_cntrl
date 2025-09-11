use realsense_rust::frame::{DepthFrame, FrameEx, PointsFrame};
use realsense_rust::stream_profile::DataError;
use realsense_sys::{
    rs2_create_frame_queue, rs2_create_pointcloud, rs2_delete_frame_queue,
    rs2_delete_processing_block, rs2_error, rs2_frame_queue, rs2_process_frame,
    rs2_processing_block, rs2_start_processing_queue, rs2_wait_for_frame,
};
use std::ptr::NonNull;
use std::time::Duration;

//Creates a pointcloud frame from a provided depth cloud frame
//Architecture modelled from the example decimation block - https://gitlab.com/tangram-vision/oss/realsense-rust/-/merge_requests/55/diffs?commit_id=a145ce0262f79f667b00ed29e3b081e00e258444#4243a5602f85e44e06d290e709da9d0217c9a4ee
#[derive(Debug)]
pub struct PointCloudProcBlock {
    //The processing block for the "Pointcloud" method
    processing_block: NonNull<rs2_processing_block>,

    //The queue where processed frames are deposited
    processing_queue: NonNull<rs2_frame_queue>,
}

impl Drop for PointCloudProcBlock {
    //Dro ('delete') the memory being used for the pointcloud block
    fn drop(&mut self) {
        unsafe {
            rs2_delete_frame_queue(self.processing_queue.as_ptr());
            rs2_delete_processing_block(self.processing_block.as_ptr());
        }
    }
}

//Note - no error checking as of yet!
impl PointCloudProcBlock {
    //Create a new pointcloud processing block
    pub fn new(processing_queue_size: i32) -> Result<Self, DataError> {
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
        })
    }

    //Process a given frame by adding it the queue (frame transfers ownership)
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

    //Wait until a frame is processed
    pub fn wait(&mut self, timeout: Duration) -> Result<PointsFrame, DataError> {
        unsafe {
            let mut err = std::ptr::null_mut::<rs2_error>();
            let timeout_millis = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);
            let point_frame =
                rs2_wait_for_frame(self.processing_queue.as_ptr(), timeout_millis, &mut err);

            Ok(PointsFrame::try_from(NonNull::new(point_frame).unwrap()).unwrap())
        }
    }

    //Poll to see if the queue is ready to provide a processed frame
    //TODO - add POLLING using poll for frame
}

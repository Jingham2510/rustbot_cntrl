//! Configure and stream a 435i sensor.
//!
//! Notice that the streaming configuration changes based on the USB speed of the sensor.
//! If one attemps to set a streaming configuration that is too much for the current USB
//! speed, RealSense will return with an error. However, that error is non-descript and will
//! not help identify the underlying problem, i.e. the bandwidth of the connection.

use anyhow::{bail, ensure, Result};
use realsense_rust::{check_rs2_error, config::Config, context::Context, frame::{DepthFrame, GyroFrame}, kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind}, pipeline::InactivePipeline};
use std::{
    collections::HashSet,
    convert::TryFrom,
    io::{self, Write},
    time::Duration,
};
use std::io::Error;
use std::ptr::NonNull;
use realsense_rust::frame::{FrameEx, PointsFrame};
use realsense_rust::pipeline::ActivePipeline;
use realsense_rust;
use realsense_sys::{rs2_create_error, rs2_create_pointcloud, rs2_delete_processing_block, rs2_error, rs2_exception_type_RS2_EXCEPTION_TYPE_BACKEND, rs2_frame_callback, rs2_frame_queue, rs2_process_frame, rs2_processing_block};


mod rs2_processing{
    pub mod pointcloud;
}
use crate::terr_map_sense::rs2_processing::pointcloud;





//The realsense camera
//Contains the info about the camera
pub struct RealsenseCam {
    pipeline : ActivePipeline,
    pcl_block : pointcloud::PointCloudBlock
}

pub struct Point{
    x: f64,
    y: f64,
    z: f64
}

//The pointcloud object


impl RealsenseCam{

    //Initialise the camera - pipeline and config
    pub fn initialise() -> Result<Self>{

        // Check for depth or color-compatible devices.
        let mut queried_devices = HashSet::new();
        queried_devices.insert(Rs2ProductLine::D400);
        let context = Context::new()?;
        let devices = context.query_devices(queried_devices);
        ensure!(!devices.is_empty(), "No devices found");


        //Try and create the inactive pipeline
        let pipeline = InactivePipeline::try_from(&Context::new()?)?;

        //Create a new config
        let mut config = Config::new();


        //Only enable the  depth stream (not bothered about gyro/infra/colour atm)
        config
            .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
            .disable_all_streams()?
            //.enable_all_streams()?;
            //Height of 0 indicates to realsense that it should select the most appropriate height itself
            .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
            .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Rgb8, 30)?;



        // Check the USB speed of our connection - for now dont check because we only care about depth stream
        // CStr => str => f32
        /*
        let usb_cstr = devices[0].info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
        let usb_val: f32 = usb_cstr.to_str()?.parse()?;
        if usb_val >= 3.0 {
            config
                .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
                .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Rgb8, 30)?
                // RealSense doesn't seem to like index zero for the IR cameras on D435i
                //
                // Really not sure why? This seems like an implementation issue, but in practice most
                // won't be after the IR image directly.
                .enable_stream(Rs2StreamKind::Infrared, Some(1), 640, 0, Rs2Format::Y8, 30)?
                .enable_stream(Rs2StreamKind::Infrared, Some(2), 640, 0, Rs2Format::Y8, 30)?
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
        } else {
            config
                .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
                .disable_all_streams()?
                .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
                .enable_stream(Rs2StreamKind::Infrared, Some(1), 640, 0, Rs2Format::Y8, 30)?
                .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
        }
        */




        Ok(Self{
            //Start the pipeline in the camera object
            //(This is done to get around the pipeline being consumed
            pipeline: pipeline.start(Some(config))?,

        })

    }

    //Return a raw pointcloud - unsafe as it uses realsense-sys basecode
    pub fn get_depth_pnts(&mut self) -> Result<Vec<(f64, f64, f64)>>{

        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;
        //Extract the depth frame
        let mut depth_frame = frame.frames_of_type::<DepthFrame>();

        //Check the depth frame isnt empty
        if depth_frame.is_empty(){
            bail!("No depth frame to read!");
        }


        //UNSAFE HERE - hopefully a future realsense-rust patchwill fix this
        //to make this safe - add a bunch of error checks that look at the errors
        unsafe {

            //extract the frame from the vector
            let depth_frame_ptr =  depth_frame.pop().unwrap().get_owned_raw().as_ptr();


            //Create an error for the processing block to alert
            let mut err =  std::ptr::null_mut::<realsense_sys::rs2_error>();

            //Create a processing block to generate the point cloud from the depth frame
            let pcl_proc_block = rs2_create_pointcloud(&mut err);

            //Create the point frame from the depth frame using the processing block
            rs2_process_frame(pcl_proc_block, depth_frame_ptr, &mut err);

            
            


            //println!("{:?}", depth_frame_ptr);


            let nn_frame = NonNull::new(depth_frame_ptr);


            /*
            let dep_frame_test = DepthFrame::try_from(nn_frame.unwrap())?;
            println!("Pointsframe - {:?}", dep_frame_test);
            */
            //CRASHES IN THE TRY FROM! seems like its not actually being converted!



            let point_frame : PointsFrame = PointsFrame::try_from(nn_frame.unwrap())?;

            println!("Pointsframe - {:?}", point_frame);





            //Iterate and extract through every point in the depth frame
            for point in point_frame.vertices(){
                println!("{:?}", point);
            }


            rs2_delete_processing_block(pcl_proc_block);


        }


        //Return the unordered tuple


        Ok(vec!((1.0, 1.0, 1.0)))
    }


    //Save a raw pointlcoud (use the time stamp in the frame)


    //Return a filtered pointcloud




}



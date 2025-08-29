//! Configure and stream a 435i sensor.
//!
//! Notice that the streaming configuration changes based on the USB speed of the sensor.
//! If one attemps to set a streaming configuration that is too much for the current USB
//! speed, RealSense will return with an error. However, that error is non-descript and will
//! not help identify the underlying problem, i.e. the bandwidth of the connection.

use anyhow::{bail, ensure, Result};
use realsense_rust;
use realsense_rust::frame::{PointsFrame};
use realsense_rust::pipeline::ActivePipeline;
use realsense_rust::{config::Config, context::Context, frame::DepthFrame, kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind}, pipeline::InactivePipeline};
use std::{
    collections::HashSet,
    convert::TryFrom,
    time::Duration,
};

use crate::mapping::rs2_processing::pointcloud::PointCloudProcBlock;
use crate::mapping::terr_map_tools::PointCloud;

//The realsense camera
//Contains the info about the camera
pub struct RealsenseCam {
    pipeline : ActivePipeline,
    pcl_block : PointCloudProcBlock
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
            pcl_block: PointCloudProcBlock::new(3)?

        })

    }

    //Return a raw pointcloud - unsafe as it uses realsense-sys basecode
    pub fn get_depth_pnts(&mut self) -> Result<PointCloud>{

        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;
        //Extract the depth frame
        let mut depth_frame = frame.frames_of_type::<DepthFrame>();

        //Check the depth frame isnt empty
        if depth_frame.is_empty(){
            bail!("No depth frame to read!");
        }

        //Add the frame to the queue to extract the point cloud
        self.pcl_block.queue(depth_frame.pop().unwrap()).expect("FAILED TO QUEUE DEPTH FRAME");

        //Wait a maximum of 1000ms for the frame to be processed
        let point_frame : PointsFrame = self.pcl_block.wait(Duration::from_millis(1000))?;

        Ok(PointCloud::create_from_iter(point_frame.vertices()))


    }


    //Save a raw pointlcoud (use the time stamp in the frame)


    //Return a filtered pointcloud




}



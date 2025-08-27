//! Configure and stream a 435i sensor.
//!
//! Notice that the streaming configuration changes based on the USB speed of the sensor.
//! If one attemps to set a streaming configuration that is too much for the current USB
//! speed, RealSense will return with an error. However, that error is non-descript and will
//! not help identify the underlying problem, i.e. the bandwidth of the connection.

use anyhow::{ensure, Result};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::{DepthFrame, GyroFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{
    collections::HashSet,
    convert::TryFrom,
    io::{self, Write},
    time::Duration,
};

use realsense_rust::pipeline::ActivePipeline;




//The realsense camera
//Contains the info about the camera
pub struct RealsenseCam {
    pipeline : ActivePipeline
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
            //Height of 0 indicates to realsense that it should select the most appropriate height itself
            .enable_stream(Rs2StreamKind::Depth, None, 1280, 0, Rs2Format::Z16, 30)?;


        // Check the USB speed of our connection - for now dont check because we only care about depth stream
        //For now disable gyro and infrared stream - not too fussed
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

    //Return a raw pointcloud
    pub fn get_depth_pnts(&mut self) -> Result<Vec<(f64, f64, f64)>>{

        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;

        //Extract the points from the frame
        let mut depth_frames = frame.frames_of_type::<DepthFrame>();
        
       


        println!("{:?}", depth_frames.pop().unwrap().depth_units());

        //Return the unordered tuple


        Ok(vec!((1.0, 1.0, 1.0)))


    }

    //Return a filtered pointcloud



}





pub fn example() -> Result<()> {
    // Check for depth or color-compatible devices.
    let mut queried_devices = HashSet::new();
    queried_devices.insert(Rs2ProductLine::D400);
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No devices found");

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();

    // Check the USB speed of our connection
    // CStr => str => f32
    let usb_cstr = devices[0].info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
    let usb_val: f32 = usb_cstr.to_str().unwrap().parse().unwrap();
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

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;
    let mut distance = 0.0;
    let mut motion = [0.0, 0.0, 0.0];

    // process frames
    for i in 0..1000 {
        let timeout = Duration::from_millis(5000);
        let frames = pipeline.wait(Some(timeout))?;

        if i % 5 == 0 {
            // Get depth
            let mut depth_frames = frames.frames_of_type::<DepthFrame>();
            if !depth_frames.is_empty() {
                let depth_frame = depth_frames.pop().unwrap();
                let tmp_distance =
                    depth_frame.distance(depth_frame.width() / 2, depth_frame.height() / 2)?;
                if tmp_distance != 0.0 {
                    distance = tmp_distance;
                }
            }

            // Get gyro
            let motion_frames = frames.frames_of_type::<GyroFrame>();
            if !motion_frames.is_empty() {
                motion = *motion_frames[0].rotational_velocity();
            }
        }

        // Print our results
        print!(
            "\rDistance of center pixel: {:<10} m | Gyro reading: {:<15}, {:<15}, {:<15}",
            distance, motion[0], motion[1], motion[2]
        );
        io::stdout().flush().unwrap();
    }

    Ok(())
}

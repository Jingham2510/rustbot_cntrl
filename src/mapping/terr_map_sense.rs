//! Configure and stream a 435i sensor.
//!
//! Notice that the streaming configuration changes based on the USB speed of the sensor.
//! If one attemps to set a streaming configuration that is too much for the current USB
//! speed, RealSense will return with an error. However, that error is non-descript and will
//! not help identify the underlying problem, i.e. the bandwidth of the connection.

use anyhow::{Result, bail, ensure};
use realsense_rust;
use realsense_rust::frame::{FrameEx, PointsFrame};
use realsense_rust::pipeline::ActivePipeline;
use realsense_rust::{
    config::Config,
    context::Context,
    frame::DepthFrame,
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom, time::Duration};

use crate::mapping::rs2_processing::pointcloud::PointCloudProcBlock;
use crate::mapping::terr_map_tools::PointCloud;

use raylib;
use raylib::drawing::{RaylibDraw, RaylibDraw3D, RaylibMode3DExt};
use raylib::prelude::{Camera3D, Color};

use raylib::math::Vector3;

//The realsense camera
//Contains the info about the camera
pub struct RealsenseCam {
    pipeline: ActivePipeline,
    pcl_block: PointCloudProcBlock,
}

//The pointcloud object

impl RealsenseCam {
    //Initialise the camera - pipeline and config
    pub fn initialise() -> Result<Self> {
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

        Ok(Self {
            //Start the pipeline in the camera object
            //(This is done to get around the pipeline being consumed
            pipeline: pipeline.start(Some(config))?,
            pcl_block: PointCloudProcBlock::new(100)?,
        })
    }

    //Return a raw pointcloud - unsafe as it uses realsense-sys basecode
    pub fn get_depth_pnts(&mut self) -> Result<PointCloud> {
        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;
        //Extract the depth frame
        let mut depth_frame = frame.frames_of_type::<DepthFrame>();

        //Check the depth frame isnt empty
        if depth_frame.is_empty() {
            bail!("No depth frame to read!");
        }

        //Add the frame to the queue to extract the point cloud
        self.pcl_block
            .queue(depth_frame.pop().unwrap())
            .expect("FAILED TO QUEUE DEPTH FRAME");

        //Wait a maximum of 1000ms for the frame to be processed
        let point_frame: PointsFrame = self.pcl_block.wait(Duration::from_millis(1000))?;

        Ok(PointCloud::create_from_iter(
            point_frame.vertices(),
            point_frame.timestamp(),
        ))
    }

    //Return a filtered pointcloud

    //Visualise the pointcloud stream - FOR DEBUGGING
    pub fn debug_vis(&mut self) {
        //Create the window
        let (mut rl, thread) = raylib::init()
            .size(1280, 720)
            .title("REALSENSE - DEBUG")
            .build();

        //Create a camera for the viewpoint
        let cam = Camera3D::perspective(
            //Position
            Vector3::new(0.0, 3.0, -10.0),
            //Target
            Vector3::new(0.0, 2.5, 1.0),
            //Up
            Vector3::new(0.0, 1.0, 0.0),
            //FOV
            45.0,
        );

        //Constantly stream the pointcloud data to a raylib 3d container
        while !rl.window_should_close() {
            rl.draw(&thread, |mut d| {
                //Set the background
                d.clear_background(Color::BLACK);

                //Wait for a frame to arrive
                let frame = self.pipeline.wait(None).expect("NO FRAME");
                //Extract the depth frame
                let mut depth_frame = frame.frames_of_type::<DepthFrame>();

                //Add the frame to the queue to extract the point cloud
                self.pcl_block
                    .queue(depth_frame.pop().unwrap())
                    .expect("FAILED TO QUEUE DEPTH FRAME");

                //Wait a maximum of 1000ms for the frame to be processed
                let point_frame: PointsFrame = self
                    .pcl_block
                    .wait(Duration::from_millis(1000))
                    .expect("COULDNT GENERATE POINTS");

                //Read a frame - get a pointcloud and draw all the points
                for vert in point_frame.vertices().iter() {
                    let pnt = vert.xyz;

                    //Skip invalid points and points too far away
                    if pnt == [0.0, 0.0, 0.0] || pnt[2] > 5.0 {
                        continue;
                    }

                    d.draw_mode3D(cam, |mut d2, cam| {
                        d2.draw_sphere(Vector3::new(pnt[0], pnt[1], pnt[2]), 0.01, Color::WHITE);
                    });
                }
            });

            rl.wait_time(0.0);
        }
    }
}

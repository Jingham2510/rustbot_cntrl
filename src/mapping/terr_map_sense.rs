/// Configure and stream a 435i sensor.

/// Notice that the streaming configuration changes based on the USB speed of the sensor.
/// If one attempts to set a streaming configuration that is too much for the current USB
/// speed, RealSense will return with an error. However, that error is non-descript and will
/// not help identify the underlying problem, i.e. the bandwidth of the connection.
use anyhow::{Result, bail, ensure};
use realsense_rust;
use realsense_rust::frame::{ColorFrame, FrameEx, PointsFrame};
use realsense_rust::pipeline::ActivePipeline;
use realsense_rust::{
    config::Config,
    context::Context,
    frame::DepthFrame,
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};

use std::process::{Command, Stdio};
use std::{collections::HashSet, convert::TryFrom, time::Duration};

use crate::mapping::rs2_processing::process_frame::{
    FrameProc, FrameProcBlock, create_color_image,
};
use crate::mapping::terr_map_tools::PointCloud;

use raylib;
use raylib::drawing::{RaylibDraw, RaylibDraw3D, RaylibMode3DExt};
use raylib::prelude::{Camera3D, Color};

use raylib::math::Vector3;

///The realsense camera
pub struct RealsenseCam {
    pub cam_no: usize,
    pipeline: ActivePipeline,
    pcl_block: FrameProcBlock<PointsFrame>,
}

impl RealsenseCam {
    ///Connect and Initialise a camera
    /// When using multiple cameras, cam_no is used to specify which one to connect to
    pub fn initialise_raw(cam_no: usize) -> Result<Self, anyhow::Error> {
        let (pipeline, config) = Self::setup(cam_no)?;

        Ok(Self {
            cam_no,
            //Start the pipeline in the camera object
            //(This is done to get around the pipeline being consumed
            pipeline: pipeline.start(Some(config))?,
            pcl_block: FrameProcBlock::make_raw_points_block(1)?,
        })
    }

    fn setup(cam_no: usize) -> Result<(InactivePipeline, Config), anyhow::Error> {
        // Check for depth or color-compatible devices.
        let mut queried_devices = HashSet::new();
        queried_devices.insert(Rs2ProductLine::D400);
        let context = Context::new()?;
        let devices = context.query_devices(queried_devices);
        ensure!(!devices.is_empty(), "No devices found");

        //If the provided camera number is too high - bail
        if cam_no > devices.len() {
            bail!("Invalid camera selection");
        }

        /*
        println!("devs: {:?}", devices);
        println!(
            "Selected: {:?}",
            devices[cam_no].info(Rs2CameraInfo::SerialNumber)
        );
        */

        //Try and create the inactive pipeline
        let pipeline = InactivePipeline::try_from(&Context::new()?)?;

        //Create a new config
        let mut config = Config::new();

        //Only enable the  depth stream (not bothered about gyro/infra/colour atm)
        config
            .enable_device_from_serial(devices[cam_no].info(Rs2CameraInfo::SerialNumber).unwrap())?
            .disable_all_streams()?
            //.enable_all_streams()?;
            //Height of 0 indicates to realsense that it should select the most appropriate height itself
            .enable_stream(Rs2StreamKind::Depth, None, 1280, 0, Rs2Format::Z16, 30)?
            .enable_stream(Rs2StreamKind::Color, None, 1280, 0, Rs2Format::Rgb8, 30)?;

        Ok((pipeline, config))
    }

    ///Return a raw xyz pointcloud from the camera
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

    ///Get a colour image from the realsense camera
    pub fn get_image(&mut self, filename: &str) -> Result<(), anyhow::Error> {
        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;
        //Extract the first colour frame
        let color_frame = frame.frames_of_type::<ColorFrame>();

        //Check the depth frame isnt empty
        if color_frame.is_empty() {
            bail!("No color frame to read!");
        }

        //Process the image
        let image = create_color_image(color_frame.get(0).unwrap())?;

        //Save the image
        let fp = format!("appdata\\{}.png", filename);
        image.save(fp)?;

        println!("Image saved!");

        Ok(())
    }

    ///Return all aruco tags visible to the camera
    pub fn get_aruco_tags(&mut self) -> Result<Vec<(usize, [(i32, i32); 4])>, anyhow::Error> {
        //Take an image and save it in the application data
        let tag_img_fp = format!("aruco_detect_{}", self.cam_no);
        self.get_image(&tag_img_fp)?;

        //Run the python script - only works on windows machines
        let py_cmd = Command::new(
            //,
            "cmd",
        )
        .args([
            "/C",
            &format!(
                "py src\\aruco_detection\\detect_aruco_id.py appdata\\{}.png",
                tag_img_fp
            ),
        ])
        .stdout(Stdio::piped())
        .output()
        .unwrap();

        //Get the output string from the python file and split it line by line
        let out_string = String::from_utf8(py_cmd.stdout).unwrap();
        let line_split: Vec<&str> = out_string.split("\n").collect();

        //Get the number of IDs detected
        let no_of_ids: usize = line_split[1]
            .trim()
            .replace("ID_COUNT:", "")
            .parse()
            .unwrap();

        println!("Number of IDs detected: {}", no_of_ids);

        if no_of_ids == 0 {
            bail!("No ids detected")
        }

        let mut id_info: Vec<(usize, [(i32, i32); 4])> = vec![];

        //Extract the id information
        let lines_per_id = 4;
        for i in 0..no_of_ids {
            let start_line = i + 2 + (i * lines_per_id);
            println!("{}", line_split[start_line]);

            //Get the id number
            let id_no: usize = line_split[start_line]
                .trim()
                .replace("MARK:[[", "")
                .replace("]]", "")
                .parse()
                .unwrap();

            //Get the corner information
            let mut corners: [(i32, i32); 4] = [(0, 0), (0, 0), (0, 0), (0, 0)];
            for j in 1..=4 {
                let corner_str: &str = line_split[start_line + j];

                let corner_split = corner_str
                    .trim()
                    .replace("CORN:", "")
                    .replace("[", "")
                    .replace("]", "");

                let corner_split: Vec<&str> = corner_split.split(".").collect();

                corners[j - 1] = (
                    corner_split[0].trim().parse().unwrap(),
                    corner_split[1].trim().parse().unwrap(),
                );
            }
            id_info.push((id_no, corners));
        }

        Ok(id_info)
    }

    ///Visualise the pointcloud stream - FOR DEBUGGING
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

                    d.draw_mode3D(cam, |mut d2, _cam| {
                        d2.draw_sphere(Vector3::new(pnt[0], pnt[1], pnt[2]), 0.01, Color::WHITE);
                    });
                }
            });

            rl.wait_time(0.0);
        }
    }

    /*
    ///Load a json file into the device
    fn load_json(device: &Device, json_fp: String) -> Result<(), anyhow::Error> {
        //Check that the device is in advanced mode
        println!("{:?}", device.info(Rs2CameraInfo::AdvancedMode));

        if device.info(Rs2CameraInfo::AdvancedMode).unwrap()
            != CStr::from_bytes_until_nul("true".as_bytes()).unwrap()
        {
            bail!("Not in advanced mode");
        };

        //Load the json file into raw bytes
        let json = File::open(json_fp)?;

        let file_bytes = json.bytes();
        let mut bytes_vec: Vec<u8> = vec![];
        for byte in file_bytes {
            if byte.is_ok() {
                bytes_vec.push(byte.unwrap())
            }
        }

        //Turn the bytes into a c_void reference
        let json_void: *const c_void = bytes_vec.as_ref() as *const Vec<u8> as *const c_void;
        //Error pointer
        let mut err = std::ptr::null_mut::<rs2_error>();

        //Load the json to the file
        unsafe {
            rs2_load_json(
                device.get_raw().as_mut(),
                json_void,
                bytes_vec.len() as u32,
                &mut err,
            );
        }

        Ok(())
    }
    */
}

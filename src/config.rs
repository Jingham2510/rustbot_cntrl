#![allow(dead_code)]
///Configuraiton setup for the program
use anyhow::bail;
use nalgebra::Matrix4;
use std::fmt::Debug;
use std::fs::File;
use std::io::{BufRead, BufReader};

//TODO: Fix config again...
//Config structs and setup

#[derive(Debug)]
///The programme configuration information
pub struct Config {
    ///The filepath of a given test
    test_fp: String,
    ///The first camera config info
    pub cam_infor: CamInfo,
    ///The second camerca config info
    pub cam_infol: CamInfo,
    ///The robot information
    pub rob_info: RobInfo,
    ///Geo-test phase 2 controller settings
    pub phase2_cntrl_settings: String,
    ///Geo-test phase 3 controller setting
    pub phase3_cntrl_settings: String,

    ///Indicator for test processing (in case of erroneous analyses)
    default: bool,
}

#[derive(Debug)]
///Camera config info
pub struct CamInfo {
    ///Homogeneous transform to world 0,0,0 with z as vertical depth
    tmat: Matrix4<f64>,
    ///The x scale relative to the real-world
    x_scale: f64,
    ///The y scale relative to the real world
    y_scale: f64,
}

#[derive(Debug)]
///Robot config info
pub struct RobInfo {
    ///The name of the robot
    rob_name: String,
    ///Position relative to the zero position
    pos_for_zero: [f64; 3],
    ///Position relative to the zero orientation
    ori_for_zero: [f64; 3],
    ///Height that the robot registers where the end effector sits minimally above the soil
    min_embed_height: f64,
}

const CONFIG_FP: &str = "configs/";

impl Default for Config {
    ///Create a default configuration
    fn default() -> Config {
        //Create the default config
        Config {
            test_fp: "C:/Users/User/Documents/Results/DEPTH_TESTS"
                .parse()
                .unwrap(),
            cam_infor: CamInfo::default(),
            cam_infol: CamInfo::default(),
            rob_info: RobInfo::default(),
            phase2_cntrl_settings: "NONE".parse().unwrap(),
            phase3_cntrl_settings: "NONE".parse().unwrap(),
            default: true,
        }
    }
}

impl Default for CamInfo {
    ///Create a default camera configuartion
    fn default() -> CamInfo {
        CamInfo {
            tmat: Matrix4::identity(),
            //Scale from mm to m
            x_scale: 0.001,
            y_scale: 0.001,
        }
    }
}

impl Default for RobInfo {
    ///Create a default robot configuration
    fn default() -> RobInfo {
        RobInfo {
            rob_name: "ABB-IRB6400".parse().unwrap(),
            pos_for_zero: [2400.0, 1300.0, 1250.0],
            ori_for_zero: [60.0, 60.0, 40.0],
            min_embed_height: 161.0,
        }
    }
}

impl Config {
    ///Create the programme configuration
    pub fn setup_config() -> Result<Self, anyhow::Error> {
        //Get the test filepath
        let test_fp = Self::extract_test_fp()?;

        //Get the Caminfo (from the file)
        Ok(Self {
            test_fp,
            cam_infor: CamInfo::read_cam_info_from_file(0)?,
            cam_infol: CamInfo::read_cam_info_from_file(1)?,
            rob_info: RobInfo::read_rob_info_from_file()?,
            phase2_cntrl_settings: "NONE".parse()?,
            phase3_cntrl_settings: "NONE".parse()?,
            default: false,
        })
    }

    ///Create the test filepath
    fn extract_test_fp() -> Result<String, anyhow::Error> {
        //Construct the filepath
        const FP_FILENAME: &str = "filepaths.txt";
        let fp = format!("{}/{}", CONFIG_FP, FP_FILENAME);

        //Open the file
        let fp_config_fp = File::open(fp)?;

        //Read the line
        let mut fp_full_string = String::new();
        BufReader::new(fp_config_fp).read_line(&mut fp_full_string)?;

        //Extract the filepath from the line
        let data_fp: Vec<&str> = fp_full_string.split("\"").collect();

        //Auto extract the string
        Ok(data_fp[1].parse()?)
    }

    ///Get the test filepath
    pub fn test_fp(&self) -> String {
        self.test_fp.clone()
    }

    ///Set the phase 2 controller settings
    pub fn set_phase2_cntrl(&mut self, config_string: String) {
        self.phase2_cntrl_settings = config_string;
    }
    ///Set the phase 3 controller settings
    pub fn set_phase3_cntrl(&mut self, config_string: String) {
        self.phase3_cntrl_settings = config_string;
    }
    ///Get whether the config is at default
    pub fn is_default(&self) -> bool {
        self.default
    }
}

impl CamInfo {
    ///Create the camera info
    pub fn create_cam_info(tmat: Matrix4<f64>, x_scale: f64, y_scale: f64) -> Self {
        Self {
            tmat,
            x_scale,
            y_scale,
        }
    }

    ///Create a caminfo struct from a line with format CAM: POS:[X,Y,Z] ORI:[X,Y,Z] X_SC:[X] Y_SC[Y]
    pub fn create_cam_info_from_line(cam_info_line: String) -> Result<Self, anyhow::Error> {
        let split = cam_info_line.replace(" ", "");
        let split = split.split("SC");

        let mut tmat: Matrix4<f64> = Matrix4::identity();
        let mut x_scale: f64 = 1.0;
        let mut y_scale: f64 = 1.0;

        for (i, token) in split.into_iter().enumerate() {
            match i {
                //The extrinsic matrix is saved as columns due to nalgebra debug setup
                0 => {
                    //Cover both camera cases
                    let mut tmat_token = token.replace("CAMR:EXT_MAT:", "").replace("X_", "");
                    tmat_token = tmat_token.replace("CAML:EXT_MAT:", "").replace("X_", "");

                    let tmat_split = tmat_token.split("],");

                    for (i, token) in tmat_split.into_iter().enumerate() {
                        let token = token.replace("[", "").replace("]", "");

                        for (j, val) in token.split(",").into_iter().enumerate() {
                            tmat[(j, i)] = val.parse()?;
                        }
                    }
                }
                1 => {
                    let x_sc_str = token.replace(":[", "").replace("]Y_", "");
                    x_scale = x_sc_str.parse()?;
                }
                2 => {
                    let y_sc_str = token.replace(":[", "").replace("]\n", "");
                    y_scale = y_sc_str.parse()?;
                }
                _ => {
                    bail!("Cam line too long!");
                }
            }
        }

        let cam_info = CamInfo::create_cam_info(tmat, x_scale, y_scale);
        Ok(cam_info)
    }

    ///Create cmarea info from camera preset camera config
    fn read_cam_info_from_file(cam_no: usize) -> Result<Self, anyhow::Error> {
        let cam_side = if cam_no == 0 { "r" } else { "l" };

        //Construct the filepath
        let cam_config_filename = format!("caminfo_{}.txt", cam_side);
        let fp = format!("{}/{}", CONFIG_FP, cam_config_filename);

        //Open the cam config file
        let cam_config_file = File::open(fp)?;

        //Have the default values initialised - incase they aren't overwritten
        let mut tmat: Matrix4<f64> = Matrix4::identity();
        let mut x_scale: f64 = 0.001;
        let mut y_scale: f64 = 0.001;

        //Go through each line and parse the info
        for line in BufReader::new(cam_config_file).lines() {
            let mut curr_line = line?;

            if curr_line.starts_with("EXT_MAT") {
                //Get rid of all blank space
                curr_line = curr_line.replace(" ", "");
                //Remove opening and ending brackets
                curr_line = curr_line.replace("EXT_MAT=[", "");
                curr_line = curr_line.replace("]", "");

                //Split into lines
                let row_split = curr_line.split(";");
                for (i, token) in row_split.into_iter().enumerate() {
                    let val_split = token.split(",");
                    for (j, val) in val_split.into_iter().enumerate() {
                        tmat[(i, j)] = val.parse()?;
                    }
                }
            } else if curr_line.starts_with("X_SCALE") {
                x_scale = Self::extract_scale(curr_line)?;
            } else if curr_line.starts_with("Y_SCALE") {
                y_scale = Self::extract_scale(curr_line)?;
            } else {
                bail!("Invalid cam config! - unknown line!")
            }
        }

        Ok(Self {
            tmat,
            x_scale,
            y_scale,
        })
    }

    ///Get the scale from a string
    fn extract_scale(line: String) -> Result<f64, anyhow::Error> {
        //Access the value
        let line_split: Vec<&str> = line.split("[").collect();
        let val = line_split[1].replace("]", "");

        //Attempt to parse it
        Ok(val.parse()?)
    }

    ///Get the relative position
    pub fn tmat(&self) -> Matrix4<f64> {
        self.tmat
    }

    ///Get the x axis scaling factor
    pub fn x_scale(&self) -> f64 {
        self.x_scale
    }
    ///Get the y axis scaling factor
    pub fn y_scale(&self) -> f64 {
        self.y_scale
    }
}

impl RobInfo {
    ///Create a rob info struct from the info file
    pub fn read_rob_info_from_file() -> Result<Self, anyhow::Error> {
        const ROB_CONFIG_FILENAME: &str = "robinfo.txt";

        let fp = format!("{}/{}", CONFIG_FP, ROB_CONFIG_FILENAME);

        let mut rob_name = String::new();
        let mut pos_for_zero = [0.0, 0.0, 0.0];
        let mut ori_for_zero = [0.0, 0.0, 0.0];
        let mut min_embed_height = 0.0;

        //Open the file and iterate line by line
        let rob_config_file = File::open(fp)?;

        for line in BufReader::new(rob_config_file).lines() {
            let curr_line = line?;

            //Check which line your on
            if curr_line.starts_with("ROB_NAME") {
                //Split the current line to extract the name
                let split: Vec<&str> = curr_line.split("\"").collect();
                rob_name = split[1].parse()?;
            } else if curr_line.starts_with("POS_TO_ZERO") {
                pos_for_zero = pos_ori_parser(curr_line)?;
            } else if curr_line.starts_with("ORI_TO_ZERO") {
                ori_for_zero = pos_ori_parser(curr_line)?;
            } else if curr_line.starts_with("EMBED_HEIGHT") {
                let split: Vec<&str> = curr_line.split("\"").collect();
                min_embed_height = split[1].parse()?;
            } else {
                //Panic if it encounters a line that it cannot interpret!
                bail!("Invalid line in robot config!")
            }
        }

        println!("Got rob info");

        Ok(Self {
            rob_name,
            pos_for_zero,
            ori_for_zero,
            min_embed_height,
        })
    }

    ///Create robot info from a singular config line (test data)
    pub fn create_rob_info_from_line(line: String) -> Result<RobInfo, anyhow::Error> {
        //Access the robot name
        let name_split: Vec<&str> = line.split("\"").collect();
        let rob_name = name_split[1];

        //Split the string by [
        let para_split: Vec<&str> = name_split[2].split("[").collect();

        //Access the pos
        let mut pos_for_zero = [f64::NAN, f64::NAN, f64::NAN];
        let poses = para_split[1].replace("] ORI:", "");
        let poses: Vec<&str> = poses.split(",").collect();
        for (pos_cnt, pos) in poses.into_iter().enumerate() {
            pos_for_zero[pos_cnt] = pos.parse()?;
        }

        //Backwards compatability
        let ori_rep = if para_split.len() > 3 { "] EMB:" } else { "]" };

        //Access the orientation
        let mut ori_for_zero = [f64::NAN, f64::NAN, f64::NAN];
        let oris = para_split[2].replace(ori_rep, "");
        let oris: Vec<&str> = oris.trim().split(",").collect();
        for (ori_cnt, ori) in oris.into_iter().enumerate() {
            ori_for_zero[ori_cnt] = ori.parse()?;
        }

        //Backwards compatability - if embed height doesn't exist replace with a default
        let min_embed_height: f64 = if para_split.len() > 3 {
            let embed_height = para_split[3].replace("]", "");
            let parsed: f64 = f64::NAN;

            if Ok(parsed) == embed_height.trim().parse() {
                parsed
            } else {
                160.0
            }
        } else {
            160.0
        };

        Ok(RobInfo {
            rob_name: rob_name.to_string(),
            pos_for_zero,
            ori_for_zero,
            min_embed_height,
        })
    }

    ///Get the robot name
    pub fn rob_name(&self) -> String {
        self.rob_name.clone()
    }
    ///Get the relative position to zero
    pub fn pos_to_zero(&self) -> [f64; 3] {
        self.pos_for_zero
    }
    ///Get the relative orientation to zero
    pub fn ori_to_zero(&self) -> [f64; 3] {
        self.ori_for_zero
    }
    ///Get the minimum embedded height
    pub fn min_embed_height(&self) -> f64 {
        self.min_embed_height
    }
}

///Helper function for both cam and rob config to extract xyz coords/rotations from a given string surrounded by "[]" and delimited by ","
fn pos_ori_parser(line: String) -> Result<[f64; 3], anyhow::Error> {
    //Access the string array
    let line_split: Vec<&str> = line.split("[").collect();

    //Index to the correct part of the array and remove the final bracket
    let vals = line_split[1].replace("]", "");

    //Place each value in an actual array
    let mut out: [f64; 3] = [-0.1, -0.1, -0.1];

    for (cnt, token) in vals.split(",").enumerate() {
        //Check the count is correct
        if cnt > 2 {
            bail!("Too many values in pos/ori array")
        }

        out[cnt] = token.parse()?;
    }
    Ok(out)
}

//Configuraiton setup for hte program

use anyhow::bail;
use std::fmt::Debug;
use std::fs::File;
use std::io::{BufRead, BufReader};

//Config structs and setup

#[derive(Debug)]
pub struct Config {
    test_fp: String,
    pub cam_info: CamInfo,
    pub rob_info: RobInfo,
    pub phase2_cntrl_settings: String,
    pub phase3_cntrl_settings: String,

    //Indicator for test processing (in case of erroneous analyses)
    default: bool,
}

#[derive(Debug)]
pub struct CamInfo {
    rel_pos: [f64; 3],
    rel_ori: [f64; 3],
    x_scale: f64,
    y_scale: f64,
}

#[derive(Debug)]
pub struct RobInfo {
    rob_name: String,
    //Position and orientation information required for transformation to global world frame (0,0 in terrian box)
    pos_for_zero: [f64; 3],
    ori_for_zero: [f64; 3],
    //Height that the robot registers where the end effector sits minimally above the soil
    min_embed_height: f64,
}

impl RobInfo {}

const CONFIG_FP: &str = "configs/";

impl Default for Config {
    fn default() -> Config {
        //Create the default config
        Config {
            test_fp: "C:/Users/User/Documents/Results/DEPTH_TESTS"
                .parse()
                .unwrap(),
            cam_info: CamInfo::default(),
            rob_info: RobInfo::default(),
            phase2_cntrl_settings: "NONE".parse().unwrap(),
            phase3_cntrl_settings: "NONE".parse().unwrap(),
            default: true,
        }
    }
}

impl Default for CamInfo {
    fn default() -> CamInfo {
        CamInfo {
            rel_pos: [250.0, 250.0, 250.0],
            //around 45 degrees facing downward
            rel_ori: [0.785, std::f64::consts::PI, 0.0],
            //Scale from mm to m
            x_scale: 0.001,
            y_scale: 0.001,
        }
    }
}

impl Default for RobInfo {
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
    pub fn setup_config() -> Result<Self, anyhow::Error> {
        //Get the test filepath
        let test_fp = Self::extract_test_fp()?;



        //Get the Caminfo (from the file)
        Ok(Self {
            test_fp,
            cam_info: CamInfo::read_cam_info_from_file()?,
            rob_info: RobInfo::read_rob_info_from_file()?,
            phase2_cntrl_settings: "NONE".parse()?,
            phase3_cntrl_settings: "NONE".parse()?,
            default: false,
        })
    }

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

    pub fn test_fp(&self) -> String {
        self.test_fp.clone()
    }

    pub fn set_phase2_cntrl(&mut self, config_string: String) {
        self.phase2_cntrl_settings = config_string;
    }
    pub fn set_phase3_cntrl(&mut self, config_string: String) {
        self.phase3_cntrl_settings = config_string;
    }

    pub fn is_default(&self) -> bool {
        self.default
    }
}

impl CamInfo {
    pub fn create_cam_info(
        rel_pos: [f64; 3],
        rel_ori: [f64; 3],
        x_scale: f64,
        y_scale: f64,
    ) -> Self {
        Self {
            rel_pos,
            rel_ori,
            x_scale,
            y_scale,
        }
    }

    //Create a caminfo struct from a line with format CAM: POS:[X,Y,Z] ORI:[X,Y,Z] X_SC:[X] Y_SC[Y]
    pub fn create_cam_info_from_line(cam_info_line: String) -> Result<Self, anyhow::Error> {
        //Split the line up
        let cam_inf_split = cam_info_line.split("[");
        let mut ind_cnt = 0;

        let mut rel_pos = [f64::NAN, f64::NAN, f64::NAN];
        let mut rel_ori = [f64::NAN, f64::NAN, f64::NAN];
        let mut x_scale = f64::NAN;
        let mut y_scale = f64::NAN;

        for split in cam_inf_split {
            //Split again to isolate the data
            for token in split.split("]") {
                match ind_cnt {
                    1 => {
                        for (val_cnt, val) in token.split(",").enumerate() {
                            rel_pos[val_cnt] = val.trim().parse()?;
                        }
                    }
                    3 => {
                        for (val_cnt, val) in token.split(",").enumerate() {
                            rel_ori[val_cnt] = val.trim().parse()?;
                        }
                    }
                    5 => {
                        x_scale = token.parse()?;
                    }

                    7 => {
                        y_scale = token.parse()?;
                    }

                    _ => {
                        //Do nothing
                    }
                }
                ind_cnt += 1;
            }
        }
        let cam_info = CamInfo::create_cam_info(rel_pos, rel_ori, x_scale, y_scale);

        Ok(cam_info)
    }

    fn read_cam_info_from_file() -> Result<Self, anyhow::Error> {
        //Construct the filepath
        const CAM_CONFIG_FILENAME: &str = "caminfo.txt";
        let fp = format!("{}/{}", CONFIG_FP, CAM_CONFIG_FILENAME);

        //Open the cam config file
        let cam_config_file = File::open(fp)?;

        //Have the default values initialised - incase they aren't overwritten
        let mut rel_pos: [f64; 3] = [250.0, 250.0, 250.0];
        let mut rel_ori: [f64; 3] = [0.785, std::f64::consts::PI, 0.0];
        let mut x_scale: f64 = 0.001;
        let mut y_scale: f64 = 0.001;

        //Go through each line and parse the info
        for line in BufReader::new(cam_config_file).lines() {
            let curr_line = line?;

            if curr_line.starts_with("REL_POS") {
                rel_pos = pos_ori_parser(curr_line)?;
            } else if curr_line.starts_with("REL_ORI") {
                rel_ori = pos_ori_parser(curr_line)?;
            } else if curr_line.starts_with("X_SCALE") {
                x_scale = Self::extract_scale(curr_line)?;
            } else if curr_line.starts_with("Y_SCALE") {
                y_scale = Self::extract_scale(curr_line)?;
            } else {
                bail!("Invalid cam config! - unknown line!")
            }
        }


        Ok(Self {
            rel_pos,
            rel_ori,
            x_scale,
            y_scale,
        })
    }

    fn extract_scale(line: String) -> Result<f64, anyhow::Error> {
        //Access the value
        let line_split: Vec<&str> = line.split("[").collect();
        let val = line_split[1].replace("]", "");

        //Attempt to parse it
        Ok(val.parse()?)
    }

    pub fn rel_pos(&self) -> [f64; 3] {
        self.rel_pos
    }

    pub fn rel_ori(&self) -> [f64; 3] {
        self.rel_ori
    }

    pub fn x_scale(&self) -> f64 {
        self.x_scale
    }
    pub fn y_scale(&self) -> f64 {
        self.y_scale
    }
}

impl RobInfo {
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

        //Access the orientation
        let mut ori_for_zero = [f64::NAN, f64::NAN, f64::NAN];
        let oris = para_split[2].replace("]", "");
        let oris: Vec<&str> = oris.trim().split(",").collect();
        for (ori_cnt, ori) in oris.into_iter().enumerate() {
            ori_for_zero[ori_cnt] = ori.parse()?;
        }

        Ok(RobInfo {
            rob_name: rob_name.to_string(),
            pos_for_zero,
            ori_for_zero,
            //TODO: read properly - for now just manual
            min_embed_height: 161.0,
        })
    }

    //getters for config info
    pub fn rob_name(&self) -> String {
        self.rob_name.clone()
    }
    pub fn pos_to_zero(&self) -> [f64; 3] {
        self.pos_for_zero
    }
    pub fn ori_to_zero(&self) -> [f64; 3] {
        self.ori_for_zero
    }
    pub fn min_embed_height(&self) -> f64 {
        self.min_embed_height
    }
}

//Helper function for both cam and rob config to extract xyz coords/rotations from a given string surrounded by "[]" and delimited by ","
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

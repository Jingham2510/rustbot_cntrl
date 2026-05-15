#![allow(dead_code)]
///Configuraiton setup for the program
use anyhow::bail;
use nalgebra::Matrix4;
use std::fmt::Debug;
use std::fs::File;
use std::io::{BufRead, BufReader};

//TODO: Fix config again...
//Config structs and setup

#[derive(Debug, Clone)]
///The programme configuration information
pub struct Config {
    ///The filepath of a given test
    test_fp: String,
    ///The robot information
    pub rob_info: RobInfo,
    ///Geo-test phase 2 controller settings
    pub phase2_cntrl_settings: String,
    ///Geo-test phase 3 controller setting
    pub phase3_cntrl_settings: String,

    ///Indicator for test processing (in case of erroneous analyses)
    default: bool,
}

#[derive(Debug, Clone)]
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
            rob_info: RobInfo::default(),
            phase2_cntrl_settings: "NONE".parse().unwrap(),
            phase3_cntrl_settings: "NONE".parse().unwrap(),
            default: true,
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

impl RobInfo {
    ///Create a rob info struct from the info file
    pub fn read_rob_info_from_file() -> Result<Self, anyhow::Error> {
        const ROB_CONFIG_FILENAME: &str = "robinfo.txt";

        let fp = format!("{}/{}", CONFIG_FP, ROB_CONFIG_FILENAME);

        //println!("{}", fp);

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

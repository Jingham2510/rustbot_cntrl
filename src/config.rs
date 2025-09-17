//Configuraiton setup for hte program


use std::fmt::Debug;
use std::fs::File;
use std::io::{BufRead, BufReader};
use anyhow::bail;

//Config structs and setup

#[derive(Debug)]
pub struct Config{
    test_fp : String,
    pub cam_info : CamInfo,
    //Indicator for test processing (in case of erroneous analyses)
    default : bool
}

#[derive(Debug)]
pub struct CamInfo{
    rel_pos : [f32; 3],
    rel_ori : [f32; 3],
    x_scale : f32,
    y_scale : f32
}



const CONFIG_FP : &str = "configs/";

impl Default for Config{
    fn default() -> Config{

        //Create the default config
        Config{
            test_fp : "C:/Users/User/Documents/Results/DEPTH_TESTS".parse().unwrap(),
            cam_info : CamInfo::default(),
            default : true
        }
    }
}



impl Default for CamInfo{

    fn default() -> CamInfo{

        CamInfo{
            rel_pos : [250.0, 250.0, 250.0],
            //around 45 degrees facing downward
            rel_ori : [0.785, std::f32::consts::PI, 0.0],
            //Scale from mm to m
            x_scale : 0.001,
            y_scale : 0.001
        }
    }
}



impl Config{
    pub fn setup_config() -> Result<Self, anyhow::Error>{
        //Get the test filepath
        let test_fp = Self::extract_test_fp()?;

        //Get the Caminfo (from the file)
        Ok(Self{
            test_fp,
            cam_info : CamInfo::read_cam_info_from_file()?,
            default : false
        })
    }

    fn extract_test_fp() -> Result<String, anyhow::Error>{

        //Construct the filepath
        const FP_FILENAME : &str = "filepaths.txt";
        let fp = format!("{}/{}", CONFIG_FP, FP_FILENAME);


        //Open the file
        let fp_config_fp = File::open(fp)?;

        //Read the line
        let mut fp_full_string = String::new();
        BufReader::new(fp_config_fp).read_line(&mut fp_full_string)?;

        //Extract the filepath from the line
         let data_fp : Vec<&str> = fp_full_string.split("\"").collect();

        //Auto extract the string
        Ok(data_fp[1].parse()?)
    }


    pub fn test_fp(&self) -> String{
        self.test_fp.clone()
    }



    pub fn is_default(&self) -> bool{
        self.default
    }

}

impl CamInfo{


    pub fn create_cam_info(rel_pos : [f32;3], rel_ori : [f32;3], x_scale  :f32, y_scale : f32 ) -> Self{

        Self{
            rel_pos,
            rel_ori,
            x_scale,
            y_scale
        }
    }

    fn read_cam_info_from_file() -> Result<Self, anyhow::Error>{

        //Construct the filepath
        const CAM_CONFIG_FILENAME: &str = "caminfo.txt";
        let fp = format!("{}/{}", CONFIG_FP, CAM_CONFIG_FILENAME);

        println!("{:?}", fp);

        //Open the cam config file
        let cam_config_file = File::open(fp)?;



        //Have the default values initialised - incase they aren't overwritten
        let mut rel_pos : [f32;3] = [250.0, 250.0, 250.0];
        let mut rel_ori : [f32;3] = [0.785, std::f32::consts::PI, 0.0];
        let mut x_scale : f32 = 0.001;
        let mut y_scale : f32 = 0.001;

        //Go through each line and parse the info
        for line in BufReader::new(cam_config_file).lines(){

            let curr_line = line?;

                if curr_line.starts_with("REL_POS"){
                    rel_pos = Self::extract_pos_ori(curr_line)?;
                }else if curr_line.starts_with("REL_ORI"){
                    rel_ori = Self::extract_pos_ori(curr_line)?;
                }else if curr_line.starts_with("X_SCALE"){
                    x_scale = Self::extract_scale(curr_line)?;
                }else if curr_line.starts_with("Y_SCALE"){
                    y_scale = Self::extract_scale(curr_line)?;
                }

                else {
                    bail!("Invalid cam config! - unknown line!")
                }
        }

        Ok(Self{
            rel_pos,
            rel_ori,
            x_scale,
            y_scale

        })
    }


    fn extract_pos_ori(line: String) -> Result<[f32;3], anyhow::Error>{

        //Access the string array
        let line_split : Vec<&str> = line.split("[").collect();

        //Index to the correct part of the array and remove the final bracket
        let vals = line_split[1].replace("]", "");

        //Place each value in an actual array
        let mut out : [f32; 3] = [-0.1, -0.1, -0.1];
        let mut cnt = 0;

        for token in vals.split(","){

            //Check the count is correct
            if cnt > 2{
                bail!("Too many values in pos/ori array")
            }

            out[cnt] = token.parse()?;
            cnt = cnt + 1
        }



        Ok(out)


    }


    fn extract_scale(line : String) -> Result<f32, anyhow::Error>{

        //Access the value
        let line_split : Vec<&str> = line.split("[").collect();
        let val = line_split[1].replace("]", "");

        //Attempt to parse it
        Ok(val.parse()?)

    }

    pub fn rel_pos(&self) -> [f32;3]{
        self.rel_pos
    }

    pub fn rel_ori(&self) -> [f32;3]{
        self.rel_ori
    }

    pub fn x_scale(&self) -> f32{
        self.x_scale
    }

    pub fn y_scale(&self) -> f32{
        self.y_scale
    }



}
use std::fs::File;
use std::io::{BufRead, BufReader};

//Config structs and setup
pub struct Config{
    test_fp : String,
    cam_info : CamInfo
}

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
            cam_info : CamInfo::default()
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

    pub fn create_config() -> Result<Self, anyhow::Error>{

        //Get the test filepath
        let test_fp = Self::extract_test_fp()?;
        
        
        //Get the Caminfo (from the file)

        Ok(Self{
            test_fp,
            cam_info : CamInfo::default()
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
         let fp : Vec<&str> = fp_full_string.split("\"").collect();

        println!("{}", fp[1]);

        //Auto extract the string
        Ok(fp[1].parse()?)
    }


}
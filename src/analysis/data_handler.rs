//Data handler object - reads a data file for an associated test

use std::fs::File;
use std::io::{BufRead, BufReader};
use crate::config::CamInfo;

//Object associated with completing data handling tasks
pub struct DataHandler {
    file: File,
}


impl DataHandler {
    //Create a data handler by associating a data file with it
    //Do not store all the data in the object for now - can just read it when necessary
    pub fn read_data_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        let file = File::open(filepath)?;

        Ok(Self { file })
    }

    //Function which gets the rectangular bounds of a trajectory
    //Z pos currently unused
    pub fn get_traj_rect_bnds(&mut self) -> [f32; 4] {
        //First create a line reader
        let line_reader = BufReader::new(self.file.try_clone().expect("FAILED TO CLONE FILE"));

        let mut pos_list: Vec<[f32; 3]> = vec![];

        //Read every line
        for line in line_reader.lines() {
            let line = line.unwrap();

            //We know the data starts with ',[' so we can jump right to it
            let line_split = line.split(",[");

            //Extract the pos information (index - 1)
            //Remove the final square bracket
            let curr_pos_string = line_split.collect::<Vec<&str>>()[1].replace("]", "");

            let mut curr_pos: [f32; 3] = [f32::NAN, f32::NAN, f32::NAN];
            let mut cnt = 0;

            //Extract the individual numbers from the pos string and feed them into a pos array
            for token in curr_pos_string.split(",") {
                curr_pos[cnt] = token.parse().unwrap();
                cnt = cnt + 1;
            }

            //Push the pos array to the pos list
            pos_list.push(curr_pos);
        }

        let mut min_x = 999.0;
        let mut max_x = -999.0;
        let mut min_y = 999.0;
        let mut max_y = -999.0;

        //Go through each piece of trajectory information
        for pos in pos_list.iter() {
            if pos[0] < min_x {
                min_x = pos[0];
            } else if pos[0] > max_x {
                max_x = pos[0];
            }

            if pos[1] < min_y {
                min_y = pos[1];
            } else if pos[1] > max_y {
                max_y = pos[1];
            }
        }

        //Return the bounds
        [min_x, max_x, min_y, max_y]
    }


    //Gets the camera info from the first line of the data - parses it
    pub(crate) fn get_cam_info(&mut self) -> Result<CamInfo, anyhow::Error> {

       
        //Get the first line of the data file
        let mut cam_info_line = String::new();
        BufReader::new(&self.file).read_line(&mut cam_info_line)?;

        //Split the line up
        let cam_inf_split = cam_info_line.split("[");
        let mut ind_cnt = 0;
        
        let mut rel_pos = [f32::NAN, f32::NAN, f32::NAN];
        let mut rel_ori = [f32::NAN, f32::NAN, f32::NAN];
        let mut x_scale = f32::NAN;
        let mut y_scale = f32::NAN;
        
        for split in cam_inf_split{
            
            //Split again to isolate the data
            for token in split.split("]"){


                //We only want the first token
                //println!("{ind_cnt} - {token}");


                match ind_cnt {

                    1 => {
                        let mut val_cnt = 0;
                        for val in token.split(","){
                            rel_pos[val_cnt] = val.trim().parse()?;
                            val_cnt = val_cnt + 1;
                        }

                    }

                    3 =>{
                        let mut val_cnt = 0;
                        for val in token.split(","){
                            rel_ori[val_cnt] = val.trim().parse()?;
                            val_cnt = val_cnt + 1;
                        }

                    }

                    5 => {
                        x_scale = token.parse()?;
                    }

                    7 => {
                        y_scale = token.parse()?;
                    }

                    _ =>{
                        println!("Skipping line")
                    }
                }
                ind_cnt = ind_cnt + 1;
            }

        }


        
        let cam_info = CamInfo::create_cam_info(rel_pos, rel_ori, x_scale, y_scale);


        Ok(cam_info)
    }



}

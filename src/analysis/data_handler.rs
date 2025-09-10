//Data handler object - reads a data file for an associated test

use std::fs::File;
use std::io::{BufRead, BufReader};

//Object associated with completing data handling tasks
struct DataHandler{
    file : File
}


impl DataHandler{

    //Create a data handler by associating a data file with it
    //Do not store all the data in the object for now - can just read it when necessary
    pub fn read_data_from_file(filepath : String) -> Result<Self, anyhow::Error>{

        let file = File::open(filepath)?;

        Ok(Self{
            file
        })

    }


    //Function which gets the rectangular bounds of a trajectory
    pub fn get_traj_rect_bnds(&mut self) -> [f32; 4]{

        //First create a line reader
        let line_reader = BufReader::new(self.file.try_clone().expect("FAILED TO CLONE FILE"));


        let mut pos_list : Vec<[f32; 3]> = vec![];

        //Read every line
        for line in line_reader.lines(){

            if line.is_ok(){

                let line_split = line.unwrap().split(",");

                //Extract the pos information (index - 2)
                let curr_pos_string = line_split.collect::<Vec<&str>>()[2];

                //Extract the individual numbers from the pos string and feed them into a pos array
                todo!()

                //Push the pos array to the pos list

            }else{
                println!("Invalid data line!");

            }

        }

        let mut min_x = 999.0;
        let mut max_x = -999.0;
        let mut min_y = 999.0;
        let mut max_y = -999.0;

        //Go through each piece of trajectory information
        for pos in pos_list.iter(){

            todo!()

        }


        //Return the bounds
        [min_x, max_x, min_y, max_y]
    }






}
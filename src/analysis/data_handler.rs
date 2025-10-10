//Data handler object - reads a data file for an associated test

use std::fs::File;
use std::io::{BufRead, BufReader, Seek};
use anyhow::bail;
use crate::control::misc_tools::string_tools;

//Stores a datafile and the corresponding test data
pub struct DataHandler {
    file: File,
    timestamps : Vec<f32>,
    trajectory: Vec<[f32;3]>,
    ori : Vec<[f32; 3]>,
    forces : Vec<[f32;6]>
}

impl DataHandler {
    //Create a data handler by associating a data file with it
    //Do not store all the data in the object for now - can just read it when necessary
    pub fn read_data_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        let mut file = File::open(filepath)?;

        //Create the empty vectors
        let mut timestamps : Vec<f32> = vec![];
        let mut trajectory : Vec<[f32;3]> = vec![];
        let mut ori : Vec<[f32;3]> = vec![];
        let mut forces : Vec<[f32;6]> = vec![];

        //Go line by line through the file
        for line in BufReader::new(file.try_clone()?).lines(){

            //Split each line
            let line_split  = line?;
            let line_split : Vec<&str>= line_split.split(",").collect();


            //Extract the info into the vectors
            timestamps.push(line_split[1].parse()?);

            //BIT HACKY! -------------- reconstruct the strings based on the splits just to dissaemble again
            let mut str_to_parse = format!("{},{},{}", line_split[2], line_split[3], line_split[4]);

            trajectory.push(str_to_traj_ori(&*str_to_parse)?);

            str_to_parse = format!("{},{},{}", line_split[5], line_split[6], line_split[7]);

            ori.push(str_to_traj_ori(&*str_to_parse)?);

            str_to_parse = format!("{},{},{},{},{},{}", line_split[8], line_split[9], line_split[10], line_split[11], line_split[12], line_split[13]);

            forces.push(str_to_force(&*str_to_parse)?);

        }

        //Rewind the file
        file.rewind()?;

        //Create the object
        Ok(Self {
            file,
            timestamps,
            trajectory,
            ori,
            forces
        })
    }

    //Function which gets the rectangular bounds of a trajectory
    //Z pos currently unused
    pub fn get_traj_rect_bnds(&mut self) -> Result<[f32; 4], anyhow::Error> {


        let mut min_x = 9999.0;
        let mut max_x = -9999.0;
        let mut min_y = 9999.0;
        let mut max_y = -9999.0;

        //Go through each piece of trajectory information
        for pos in self.trajectory.iter() {
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
        Ok([min_x, max_x, min_y, max_y])
    }


    //Extracts every trajectory point from the datafile
    pub fn get_traj(&self) -> Vec<[f32;3]>{
        //Return the trajectory
        self.trajectory.clone()
   }


    //Returns the trajectory and force data for the file - in an vector comprised of a tuple trajectory/force vector pairs
    pub fn get_force(&self) -> Vec<[f32;6]>{
        self.forces.clone()
    }


    //Returns a tuple which pairs trajectory and force data together (essentially a zip)
    pub fn get_traj_force_pairs(&mut self) -> Vec<([f32;3], [f32;6])>{


        let mut traj_force_pairs:Vec<([f32;3], [f32;6])> = vec![];


        //Go through every point in the trajectory, extract it and pair with the force data
        for (i, pnt) in self.trajectory.iter_mut().enumerate(){

            traj_force_pairs.push((*pnt, self.forces[i]));

        }

        traj_force_pairs
    }


}

//turns a string in the format "[x,y,z]" into a vector
fn str_to_traj_ori(x : &str) -> Result<[f32;3], anyhow::Error>{


    let mut curr_vec: [f32;3] = [f32::NAN, f32::NAN, f32::NAN];

    //Strip the "[" and "]"
    let x_strip = string_tools::rem_first_and_last(x);

    let mut cnt = 0;

    //Extract the individual numbers from the pos string and feed them into a pos array
    for token in x_strip.split(",") {
        curr_vec[cnt] = token.parse()?;
        cnt = cnt + 1;
    }

    //Check to make sure that the values have been filled
    if curr_vec.iter().any(|vec| vec.is_nan()) {
        bail!("Failed to parse vector from string!")
    }

    Ok(curr_vec)
}

//Parses the force string into the force vector
fn str_to_force(x : &str) -> Result<[f32;6], anyhow::Error>{

    let mut curr_vec: [f32;6] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN];

    //Strip the "[" and "]"
    let x_strip = string_tools::rem_first_and_last(x);

    let mut cnt = 0;

    //Extract the individual numbers from the pos string and feed them into a pos array
    for token in x_strip.split(",") {
        curr_vec[cnt] = token.parse()?;
        cnt = cnt + 1;
    }

    //Check to make sure that the values have been filled
    if curr_vec.iter().any(|vec| vec.is_nan()) {
        bail!("Failed to parse vector from string!")
    }

    Ok(curr_vec)
}




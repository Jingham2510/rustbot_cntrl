//Data handler object - reads a data file for an associated test

use std::fs::File;
use std::io::{BufRead, BufReader};

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
    pub fn get_traj_rect_bnds(&mut self) -> Result<[f32; 4], anyhow::Error> {
     
        let mut pos_list: Vec<[f32; 3]> = self.get_trajectory()?;

        let mut min_x = 9999.0;
        let mut max_x = -9999.0;
        let mut min_y = 9999.0;
        let mut max_y = -9999.0;

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
        Ok([min_x, max_x, min_y, max_y])
    }


    //Extracts every trajectory point from the datafile
    pub fn get_trajectory(&self) -> Result<Vec<[f32;3]>, anyhow::Error>{

        let mut traj_list : Vec<[f32;3]> = vec![];

        //Create a line reader
        let line_reader = BufReader::new(self.file.try_clone().expect("FAILED TO CLONE FILE"));


        //Read every line
        for line in line_reader.lines() {
            let line = line?;

            //We know the data starts with ',[' so we can jump right to it
            let line_split = line.split(",[");

            //Extract the pos information (index - 1)
            //Remove the final square bracket
            let curr_pos_string = line_split.collect::<Vec<&str>>()[1].replace("]", "");

            let mut curr_pos: [f32; 3] = [f32::NAN, f32::NAN, f32::NAN];
            let mut cnt = 0;

            //Extract the individual numbers from the pos string and feed them into a pos array
            for token in curr_pos_string.split(",") {
                curr_pos[cnt] = token.parse()?;
                cnt = cnt + 1;
            }

            //Push the pos array to the pos list
            traj_list.push(curr_pos);

        }

        //Return the trajectory
        Ok(traj_list)
   }



    //Returns the trajectory and force data for the file - in an vector comprised of a tuple trajectory/force vector pairs
    pub fn get_traj_and_force(&self) -> Result<Vec<([f32;3],[f32;6])>, anyhow::Error>{

        let mut force_traj_data: Vec<([f32;3], [f32;6])> = vec![];

        //Create a line reader
        let line_reader = BufReader::new(self.file.try_clone().expect("FAILED TO CLONE FILE"));


        //Read every line
        for line in line_reader.lines() {
            let line = line?;

            //We know the data starts with ',[' so we can jump right to it
            let line_split = line.split(",[");
            let line_split = line_split.collect::<Vec<&str>>();

            //Extract the pos information (index - 1)
            //Remove the final square bracket
            let curr_pos_string = line_split[1].replace("]", "");

            let mut curr_pos: [f32; 3] = [f32::NAN, f32::NAN, f32::NAN];
            let mut cnt = 0;

            //Extract the individual numbers from the pos string and feed them into a pos array
            for token in curr_pos_string.split(",") {
                curr_pos[cnt] = token.parse()?;
                cnt = cnt + 1;
            }


            let mut curr_force : [f32;6] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN];
            cnt = 0;

            for token in line_split[3].replace("]", "").split(","){
               
                curr_force[cnt] = token.parse()?;
                cnt = cnt + 1;

            }

            //Push the pos array to the pos list
            force_traj_data.push((curr_pos, curr_force));

        }



        Ok(force_traj_data)

    }


}




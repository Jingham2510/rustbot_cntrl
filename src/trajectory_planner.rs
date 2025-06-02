//Generates trajectories for tests


use std::f32::consts::PI;
use std::{fs, io};
use std::fs::{File};
use std::io::{stdin, BufRead};

const IMPL_TRAJS : [&str; 4] = ["line", "circle", "slidedown", "custom"];


//Selects a trajectory bsaed on string input from user
pub fn traj_gen(traj: &str) -> Option<Vec<(f32, f32, f32)>>{

    //Define the trajcetory - it will be of unknown size so have to store on the heap
    let mut trajectory: Vec<(f32, f32, f32)> = Vec::new();



    match traj.to_lowercase().as_str() {
        //Line trajectory
        "line" => {
            //Define all the starting points etc
            let line_x= 262.0;
            let line_z = 125.0;
            let start_y = 1650.0;
            let end_y = 2550.0;
            let start_pos  = (line_x, start_y, line_z);
            let end_pos = (line_x, end_y, line_z);

            trajectory = vec![start_pos, end_pos];
        },

        "circle" =>{
            //Define all characteristics of the circle
            let centre = (200.0, 2160.0, 125.0);
            //Number of times the circle goes round
            let loops = 3;
            //"size" of circle
            let radius = 350.0;

            //Create the circle trajectory
            for i in 1..(360*loops){
                trajectory.push((centre.0 + ((i as f32 * (PI/180.0) ).sin() * radius), centre.1 + ((i as f32 * (PI/180.0) ).cos() * radius), centre.2));
            }
        }

        //A straight line that descends in height (i.e. slides down)
        "slidedown" =>{

            let line_x = 262.0;
            let start_z = 150.0;
            let end_z = 100.0;
            let start_y = 1650.0;
            let end_y = 2100.0;
            let start_pos  = (line_x, start_y, start_z);
            let end_pos = (line_x, end_y, end_z);

            //Ensure the trajectory starts in the right position
            trajectory= vec![start_pos, end_pos];
        }

        //Draw a trajectory from a custom file
        "custom" =>{
            if let Some(traj) = cust_traj_handler(){
                trajectory = traj;
            }else{
                println!("Trajectory not selected!");
                return None;
            }

        }


        _ => {
            //Dont provide a trajectory 
            println!("Unknown trajectory");
            println!("Available trajectories:");
            for traj in IMPL_TRAJS{
                println!("\t {traj}");
            }
            println!("Trajectory not selected!");
            return None            
        }
    }


    Option::from(trajectory)
}

//Reads a custom trajectory and returns it
fn cust_traj_handler() -> Option<Vec<(f32, f32, f32)>>{
    
  

    //Stored within the local directory for the project! (updated manually with custom trajectories)
    const CUST_TRAJ_LOC : &str = "./cust_trajs/";


    let mut dir_list = Vec::new();

    //Generate a list of the trajectories available in the pre-determined location - 
    //shouldn't crash unless file doesn't exist - at which point custom trajectories wont exist anyways
    for file in fs::read_dir(CUST_TRAJ_LOC).unwrap(){
        //Really really gross unwrapping and type changing...
        dir_list.push(file.unwrap().path().file_stem().unwrap().to_str().unwrap().to_lowercase());
    }


    println!("Available custom trajectories:");
    //Display all the trajectories (inefficient - but doing in one loop is tricky)
    for filestem in dir_list.iter(){
        //Print the file and add it to the vector
        println!("\t {:?}",filestem);
    }


    //Print the list of possible trajectories

    println!("Enter the chosen trajectory:");

    //Get user input
    let mut user_inp = String::new();
    stdin()
        .read_line(&mut user_inp)
        .expect("Failed to read line");




    //Attempt to match to a trajectory in the directory
    if let Ok(file) = File::open(format!("{}{}.traj", CUST_TRAJ_LOC, user_inp.trim())){

        //Create an empty vector
        let mut traj: Vec<(f32, f32, f32)> = vec![];

        //Extract the line containing the trajectory info
        let mut traj_str = String::new();
        
        for line in io::BufReader::new(file).lines(){
            traj_str = line.unwrap();
        }

        //Parse each trajectory coordinate via comma
        let split_traj = traj_str.split(",");

        //Place each trajectory coordinate into the trajectory vector (with a default height)
        for coord in split_traj{
            
            if coord.is_empty(){
                println!("Empty line");
            }else {
                //trim the parantheses (using a pattern)
                let coord = coord.replace(&['(', ')'][..], "");

                println!("{coord}");

                //split via space
                let coord_split: Vec<_> = coord.split(" ").collect();

                //Place in a tuple and add to the trajectory
                let default_height: f32 = 125.0;
                traj.push((coord_split[0].parse().unwrap(), coord_split[1].parse().unwrap(), default_height))
            }
        }

        Option::from(traj)

    }else{
        println!("Custom trajectory not found!");
        None
    }


}

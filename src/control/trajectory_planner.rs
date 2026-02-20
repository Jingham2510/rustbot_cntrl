//Generates trajectories for tests

use anyhow::bail;
use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufRead, stdin};
use std::{fs, io};

const IMPL_TRAJS: [&str; 4] = ["line", "circle", "slidedown", "custom"];

//120
const DEFAULT_Z: f64 = 161.0;

//Selects a trajectory bsaed on string input from user
pub fn traj_gen(traj: &str) -> Result<Vec<(f64, f64, f64)>, anyhow::Error> {
    //Define the trajcetory - it will be of unknown size so have to store on the heap
    let mut trajectory: Vec<(f64, f64, f64)> = Vec::new();

    match traj.to_lowercase().as_str() {
        //Line trajectory
        "line" => {
            //Define all the starting points etc
            let line_x = 400.0;
            let line_z = DEFAULT_Z;
            let start_y = 1600.0;
            let end_y = 2200.0;
            let start_pos = (line_x, start_y, line_z);
            let end_pos = (line_x, end_y, line_z);

            trajectory = vec![start_pos, end_pos];
        }

        //Straight line trajectory - discretised into multiple points
        "dline" => {
            //Define all the starting points etc
            let line_x = 400.0;
            let line_z = DEFAULT_Z;
            let start_y = 1800.0;
            let end_y = 2600.0;
            let start_pos = (line_x, start_y, line_z);
            let end_pos = (line_x, end_y, line_z);

            let num_of_points = 1000;

            let dist_per_pnt = (end_y - start_y) / num_of_points as f64;

            trajectory = vec![start_pos];

            //Add all the middle points
            for i in 1..num_of_points {
                trajectory.push((line_x, start_y + (i as f64 * dist_per_pnt), DEFAULT_Z));
            }

            trajectory.push(end_pos);
        }

        "circle" => {
            //Define all characteristics of the circle
            let centre = (200.0, 2160.0, DEFAULT_Z);
            //Number of times the circle goes round
            let loops = 1;
            //"size" of circle
            let radius = 350.0;

            //Create the circle trajectory
            for i in 1..(360 * loops) {
                trajectory.push((
                    centre.0 + ((i as f64 * (PI / 180.0)).sin() * radius),
                    centre.1 + ((i as f64 * (PI / 180.0)).cos() * radius),
                    centre.2,
                ));
            }
        }

        //A straight line that descends in height (i.e. slides down)
        "slidedown" => {
            let line_x = 262.0;
            let start_z = DEFAULT_Z;
            let end_z = start_z - 50.0;
            let start_y = 1650.0;
            let end_y = 2100.0;
            let start_pos = (line_x, start_y, start_z);
            let end_pos = (line_x, end_y, end_z);

            //Ensure the trajectory starts in the right position
            trajectory = vec![start_pos, end_pos];
        }

        //Small vibrational movements
        "wiggle" => {
            const WIGGLE_MOVE: f64 = 0.25;

            let start_x = 200.0;
            let start_y = 2160.0;

            //NORMALLY 300
            for i in 1..300 {
                if i % 2 == 0 {
                    trajectory.push((start_x + WIGGLE_MOVE, start_y, DEFAULT_Z));
                }
                if i % 3 == 0 {
                    trajectory.push((start_x, start_y, DEFAULT_Z));
                }
                if i % 4 == 0 {
                    trajectory.push((start_x - WIGGLE_MOVE, start_y, DEFAULT_Z));
                }
                if i % 5 == 0 {
                    trajectory.push((start_x, start_y + WIGGLE_MOVE, DEFAULT_Z));
                }
                if i % 6 == 0 {
                    trajectory.push((start_x, start_y - WIGGLE_MOVE, DEFAULT_Z));
                }
            }
        }

        //Draw a trajectory from a custom file
        "custom" => {
            if let Some(traj) = cust_traj_handler() {
                trajectory = traj;
            } else {
                bail!("Trajectory not selected!")
            }
        }

        _ => {
            //Dont provide a trajectory
            println!("Unknown trajectory");
            println!("Available trajectories:");
            for traj in IMPL_TRAJS {
                println!("\t {traj}");
            }
            anyhow::bail!("Invalid Trajectory");
        }
    }
    Ok(trajectory)
}

//Reads a custom trajectory and returns it
fn cust_traj_handler() -> Option<Vec<(f64, f64, f64)>> {
    //Stored within the local directory for the project! (updated manually with custom trajectories)
    const CUST_TRAJ_LOC: &str = "./cust_trajs/";

    let mut dir_list = Vec::new();

    //Generate a list of the trajectories available in the pre-determined location -
    //shouldn't crash unless file doesn't exist - at which point custom trajectories wont exist anyways
    for file in fs::read_dir(CUST_TRAJ_LOC).unwrap() {
        //Really really gross unwrapping and type changing...
        dir_list.push(
            file.unwrap()
                .path()
                .file_stem()
                .unwrap()
                .to_str()
                .unwrap()
                .to_lowercase(),
        );
    }

    println!("Available custom trajectories:");
    //Display all the trajectories (inefficient - but doing in one loop is tricky)
    for filestem in dir_list.iter() {
        //Print the file and add it to the vector
        println!("\t {:?}", filestem);
    }

    //Print the list of possible trajectories

    println!("Enter the chosen trajectory:");

    //Get user input
    let mut user_inp = String::new();
    stdin()
        .read_line(&mut user_inp)
        .expect("Failed to read line");

    //Attempt to match to a trajectory in the directory
    if let Ok(file) = File::open(format!("{}{}.traj", CUST_TRAJ_LOC, user_inp.trim())) {
        //Create an empty vector
        let mut traj: Vec<(f64, f64, f64)> = vec![];

        //Extract the line containing the trajectory info
        let mut traj_str = String::new();

        for line in io::BufReader::new(file).lines() {
            traj_str = line.unwrap();
        }

        //Parse each trajectory coordinate via comma
        let split_traj = traj_str.split(",");

        //Place each trajectory coordinate into the trajectory vector (with a default height)
        for coord in split_traj {
            if coord.is_empty() {
                println!("Empty line");
            } else {
                //trim the parantheses (using a pattern)
                let coord = coord.replace(&['(', ')'][..], "");

                println!("{coord}");

                //split via space
                let coord_split: Vec<_> = coord.split(" ").collect();

                //Place in a tuple and add to the trajectory
                let default_height: f64 = 125.0;
                traj.push((
                    coord_split[0].parse().unwrap(),
                    coord_split[1].parse().unwrap(),
                    default_height,
                ))
            }
        }

        Option::from(traj)
    } else {
        println!("Custom trajectory not found!");
        None
    }
}

//Calculates the relative distance between points of a desired trajectory
pub fn relative_traj_gen(traj: &str) -> Result<Vec<(f64, f64, f64)>, anyhow::Error> {
    //Check that the trajectory is valid
    if let Ok(desired_traj) = traj_gen(traj) {
        //The first value in the vector is the start position
        let mut rel_traj: Vec<(f64, f64, f64)> = vec![desired_traj[0]];

        //Calculate the relative difference between each point
        for i in 1..desired_traj.len() {
            rel_traj.push((
                desired_traj[i].0 - desired_traj[i - 1].0,
                desired_traj[i].1 - desired_traj[i - 1].1,
                desired_traj[i].2 - desired_traj[i - 1].2,
            ))
        }

        //println!("{:?}", rel_traj);
        //Return the relative trajectory
        Ok(rel_traj)
    } else {
        bail!("Invalid trajectory")
    }
}

//Calculates the required xy speeds to achieve a desired trajectory
//Return format (time of speed (s), (X speed (mm/s), Y speed (mm/s))
pub fn calc_xy_timing(
    mut traj: Vec<(f64, f64, f64)>,
    des_lat_speed: f64,
) -> Vec<(f64, (f64, f64))> {
    let mut timing_instructions: Vec<(f64, (f64, f64))> = vec![];

    let mut last_pnt = traj[0];

    //Iterate through the trajectory of the robot
    for (i, pnt) in traj.iter_mut().enumerate() {
        //Skip the first point
        if i == 0 {
            continue;
        }

        let xy_distances: (f64, f64) = ((pnt.0 - last_pnt.0), (pnt.1 - last_pnt.1));

        //Calculate distance between points
        let lat_distance = (xy_distances.0.powi(2) + xy_distances.1.powi(2)).sqrt();

        let mut req_x_speed;
        let mut req_y_speed;

        //If either X or Y are not changing
        if xy_distances.0 == 0.0 {
            req_x_speed = 0.0;
            req_y_speed = des_lat_speed
        } else if xy_distances.1 == 0.0 {
            req_x_speed = des_lat_speed;
            req_y_speed = 0.0;
        } else {
            //Determine the end-eff speed as a combination of XY based on the XY ratio of the distance
            let xy_ratio = (xy_distances.0 / xy_distances.1).abs();

            req_y_speed = (des_lat_speed.powi(2) / (xy_ratio.powi(2) + 1.0)).sqrt();
            req_x_speed = xy_ratio * req_y_speed;
        }

        //If the robot needs to move in a negative direction
        if pnt.0 < last_pnt.0{
            req_x_speed = -req_x_speed;
        }
        if pnt.1 < last_pnt.1{
            req_y_speed = - req_y_speed;
        }


        //Create the timing instruction
        timing_instructions.push((lat_distance / des_lat_speed, (req_x_speed, req_y_speed)));

        last_pnt = *pnt;
    }

    timing_instructions
}

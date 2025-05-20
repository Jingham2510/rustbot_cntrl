//Generates trajectories for tests

use std::f32::consts::PI;

const IMPL_TRAJS : [&str; 2] = ["line", "circle"];


//Selects a trajectory bsaed on string input from user
pub fn traj_gen(traj: &str) -> Option<Vec<(f32, f32, f32)>>{

    //Define the trajcetory - it will be of unknown size so have to store on the heap
    let mut trajectory: Vec<(f32, f32, f32)> = Vec::new();
    



    match traj.to_lowercase().as_str() {
        //Line trajectory
        "line" => {
            //Define all the starting points etc
            let line_x: f32 = 262.0;
            let line_z: f32 = 125.0;
            let start_y: f32 = 1650.0;
            let end_y: f32 = 2550.0;
            let start_pos : (f32, f32, f32) = (line_x, start_y, line_z);
            let end_pos: (f32, f32, f32) = (line_x, end_y, line_z);

            trajectory = vec![start_pos, end_pos];
        },

        "circle" =>{
            //Define all characteristics of the circle
            let centre: (f32, f32, f32) = (200.0, 2160.0, 125.0);
            //Number of times the circle goes round
            let loops = 3;
            //"size" of circle
            let radius : f32 = 350.0;

            //Create the circle trajectory 
            for i in 1..(360*loops){
                trajectory.push((centre.0 + (i as f32 * (PI/180.0) * radius).sin(), centre.1 + (i as f32 * (PI/180.0) * radius).cos(), centre.2));
            }            
        }


        _ => {
            //Dont provide a trajectory 
            println!("Unknown trajectory");
            println!("Available trajectories:");
            for traj in IMPL_TRAJS{
                println!("\t {traj}");
            }
            return None            
        }
    }


    Option::from(trajectory)
}

//Generates trajectories for tests

use std::f32::consts::PI;


const IMPL_TRAJS : [&str; 3] = ["line", "circle", "slidedown"];


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
            let start_z = 75.0;
            let end_z = 125.0;
            let start_y = 1650.0;
            let end_y = 2550.0;
            let start_pos  = (line_x, start_y, start_z);
            let steps = 100.0;

            let y_step = (end_y - start_y) / steps;
            let z_step = (end_z - start_z) / steps;

            //Ensure the trajectory starts in the right position
            trajectory.push(start_pos);

            //Create the trajectory
            for i in 0..steps as i32{
                trajectory.push((line_x, start_y + (y_step * i as f32), start_z + (z_step * i as f32)))
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

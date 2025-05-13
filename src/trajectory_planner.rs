//Generates trajectories for tests



const IMPL_TRAJS : [&str; 1] = ["line"];


//Selects a trajectory bsaed on string input from user
pub fn traj_gen(traj: &str) -> Option<Vec<(f32, f32, f32)>>{

    //Define the trajcetory - it will be of unknown size so have to store on the heap
    let mut trajectory: Vec<(f32, f32, f32)> = Vec::new();
    



    match traj.to_lowercase().as_str() {
        //Line trajectory
        "line" => {
            //Define all the starting points etc
            let line_x: f32 = 262.0;
            let line_z: f32 = 85.0;
            let start_y: f32 = 1650.0;
            let end_y: f32 = 2550.0;
            let start_pos : (f32, f32, f32) = (line_x, start_y, line_z);
            let end_pos: (f32, f32, f32) = (line_x, end_y, line_z);

            trajectory = vec![start_pos, end_pos];


        },


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

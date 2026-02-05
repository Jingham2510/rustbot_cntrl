/*
The experiment model, calculates/estimates the states of the experiment
It is mainly used to generate the joint positions required to control the velocity of the end-effector.
Used to control the XYZ speeds for an experiment with predetermined lateral speeds
 */
use anyhow::bail;
use crate::modelling::irb6400_model::IRB6400Model;
use std::sync::mpsc::{Sender, Receiver};
use std::time::{Duration, SystemTime};

//Agnostic model type so it could work with other robot DH models
pub struct ExpModel<T>{
    //The robot model
    rob_model : T,
    //Robot trajectory defined as xyz coordinates in reference to the robot base frame
    trajectory : Vec<(f32, f32, f32)>,
    des_lat_speed : f32
}

impl ExpModel<IRB6400Model> {

    pub fn create_exp_model(trajectory: Vec<(f32, f32, f32)>, des_lat_speed : f32) -> Result<ExpModel<IRB6400Model>, anyhow::Error>{

        //Verify that the trajectory is not empty
        if trajectory.len() < 2{
            bail!("Invalid trajectory!")
        }
        if des_lat_speed <= 0.0{
            bail!("Invalid lateral speed!")
        }

        //Create the experimential model
        Ok(ExpModel{
            rob_model : IRB6400Model::create_model(),
            //(XYZ) -- (mm/s)
            trajectory,
            //(mm/s)
            des_lat_speed
        })
    }

    //Updates the desired lateral speed of the end-effector - mm/s
    pub fn update_des_lat_speed(&mut self, new_lat : f32) -> Result<(), anyhow::Error>{
        if new_lat <= 0.0{
            bail!("Invalid lateral speed!")
        }
        self.des_lat_speed = new_lat;

        Ok(())

    }



    //Calculates the required xy speeds to achieve a desired trajectory
    //Return format (time of speed (s), (X speed (mm/s), Y speed (mm/s))
    pub fn calc_xy_timing(&mut self) -> Vec<(f32, (f32, f32))>{

        let mut timing_instructions: Vec<(f32, (f32, f32))> = vec!();


        let mut last_pnt = self.trajectory[0];

        //Iterate through the trajectory of the robot
         for (i, pnt) in self.trajectory.iter_mut().enumerate() {
             //Skip the first point
             if i == 0{
                 continue
             }

             let xy_distances : (f32, f32)= (pnt.0 - last_pnt.0, pnt.1 - last_pnt.1);

             //Calculate distance between points
             let lat_distance = (xy_distances.0.powi(2) + xy_distances.1.powi(2)).sqrt();

             let req_x_speed;
             let req_y_speed;

             //If either X or Y are not changing
             if xy_distances.0 == 0.0{
                req_x_speed = 0.0;
                req_y_speed = self.des_lat_speed
             }
             else if xy_distances.1 == 0.0{
                 req_x_speed = self.des_lat_speed;
                 req_y_speed = 0.0;
             } else{
                 //Determine the end-eff speed as a combination of XY based on the XY ratio of the distance
                 let xy_ratio = xy_distances.0/xy_distances.1;

                 req_y_speed = (self.des_lat_speed.powi(2)/(xy_ratio.powi(2) + 1.0)).sqrt();
                 req_x_speed = xy_ratio * req_y_speed;

             }



             //Create the timing instruction
             timing_instructions.push((lat_distance/self.des_lat_speed,(req_x_speed, req_y_speed)));

             last_pnt = *pnt;
         }


        timing_instructions
    }

    //Simulates the trajectory and joint angles required for it
    //Takes three messengers - one that signals for a joint angle, one publishes joint angles, one that recieves desired z speeds
    pub fn run_model_traj(&mut self, initial_joint_cfg : [f32;6],trig : Receiver<bool>, pub_j : Sender<[f32; 6]>, desired_z : Receiver<f32>){

        //Set the initial joint configuration
        self.rob_model.update_joints(initial_joint_cfg);

        //Calculate the timing instructions
        let timing_instructions = self.calc_xy_timing();

        //Get the length of the instruction list
        let no_of_instructions = timing_instructions.len();
        let mut index = 0;

        let mut joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        //Wait for the first trigger
        while trig.recv().unwrap(){}

        while index < no_of_instructions{

            let curr_instruction = timing_instructions[index];

            //Start timer
            let start_time = SystemTime::now();

            //Get current time limit
            let time_lim = Duration::from_secs_f32(curr_instruction.0);

            //Continue computing at the prescribed speed until time limit is reached
            loop {

                //Update joint angles in model based on time passed and currently modelled speed
                self.rob_model.move_joints(joint_speed, start_time.elapsed().unwrap());


                //Get the desired speed for the current instruction (and desired veritcal speed)
                let des_end_eff_speed = (curr_instruction.1.0,curr_instruction.1.1,desired_z.recv().unwrap());
                //Set the joint speed required to achieve this
                joint_speed = self.rob_model.get_joint_speed(des_end_eff_speed);

                //Send the joint angles out of the thread
                self.rob_model.get_joints();

                //Check if current time reached
                if start_time.elapsed().unwrap() > time_lim {
                    break
                }
            }
            //Update index
            index = index + 1;
        }
    }
}
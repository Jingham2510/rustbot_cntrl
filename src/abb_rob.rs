

use std::io::{stdin};
use crate::{tcp_sock, string_tools};
use crate::tcp_sock::create_sock;

pub struct AbbRob {
    socket : tcp_sock::TcpSock,
    pos : Option<(f32, f32, f32)>,
    ori : Option<(f32, f32, f32)>,
    jnt_angles : Option<(f32, f32, f32, f32, f32, f32)>,
    force : Option<(f32, f32, f32, f32, f32, f32)>,
    move_flag : bool,
    traj_done_flag : bool
}



pub const IMPL_COMMDS : [&str; 10] = ["info", "cmds", "disconnect", "set joints", "set orientation", "move tool", "set pos", "req xyz", "req force", "req ori"];


impl AbbRob {


    pub fn create_rob(ip: String, port: u32) -> Option<AbbRob> {

        //Create the robots socket
        let mut rob_sock = create_sock(ip, port);
        //Attempt to connect to the robot
        if rob_sock.connect() == false {
            //Failed to connect
            None
        }
        else{
            let new_rob = AbbRob{
                socket : rob_sock,
                pos : None,
                ori : None,
                jnt_angles : None,
                force : None,
                move_flag : false,
                traj_done_flag : false
            };

            Option::from(new_rob)
        }

    }

    //TODO - Implement robot reconnect function



    //Disconnect from the robot - don't change any robot info, chances are the robot is going out of scope after this
    pub fn disconnect_rob(&mut self){
        self.socket.disconnect();
    }


    pub fn rob_cmd_handler(&mut self){


        //Loop until command given
        loop {
            //Get user input
            let mut user_inp = String::new();
            stdin()
                .read_line(&mut user_inp)
                .expect("Failed to read line");

            //Check user inout
            match user_inp.to_lowercase().trim() {
                "info" => {
                    println!("Robot controller");
                }
                //Print out the commands in the valid commands list
                "cmds" => {
                    println!("Robot commands:");
                    for cmd in IMPL_COMMDS {
                        println!("\t {cmd}");
                    }
                }
                //Disconnect robot has to be defined for every robot type
                "disconnect" => {
                    self.disconnect_rob();
                    return;
                },

                "req xyz" => {
                  self.req_xyz();
                },

                "req ori" =>{
                    self.req_ori();
                }

                //Whatever function is being tested at the moment
                "test" => {

                },

                _ => println!("Unknown command - see CMDs for list of commands"),
            }
        }
    }


    pub fn ping(&mut self){
        let s = self.socket.req("ECHO:PING");

        println!("Ping recieved - {}", s.unwrap());

    }

    fn set_joints(&self){
        todo!()
    }

    fn set_ori(&self){
        todo!()
    }

    fn set_speed(&self){
        todo!()
    }

    fn move_tool(&self){
        todo!()
    }

    fn set_pos(&self){
        todo!()
    }

    fn traj_queue_add_trans(&self){
        todo!()
    }

    fn traj_queue_add_rot(&self){
        todo!()
    }

    fn traj_queue_go(&self){
        todo!()
    }

    fn traj_queue_stop(&self){
        todo!()
    }

    fn update_traj_done(&mut self) -> Option<bool>{
        
        //Safe socket read - incase the socket crashes
        match self.socket.req("TJDN:?"){
            Some(recv) => {
                if recv == "true" {
                    self.traj_done_flag = true;
                    Option::from(true)
                }else{
                    self.traj_done_flag = false;
                    Option::from(false)
                }
            },
            None => {
                println!("WARNING ROBOT DISCONNECTED");
                None
            }
        }
        
       

    }


    //Requests the xyz position of the TCP from the robot and stores it in the robot info
    fn req_xyz(&mut self){

        //Request the info
        if let Some(recv) = self.socket.req("GTPS:0"){

            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let xyz_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if xyz_vec.len() != 3{
                println!("XYZ pos read error!");
                return;
            }
            else{
                //Store the pos in the robot info
                self.pos = Option::from((xyz_vec[0], xyz_vec[1], xyz_vec[2]));
            }


        }else{
                //If the socket request returns nothing
                println!("WARNING ROBOT DISCONNECTED");
                return;
        }







    }

    fn req_ori(&mut self){
        //Request the info
        if let Some(recv) = self.socket.req("GTOR:0"){

            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let ori_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if ori_vec.len() != 3{
                println!("XYZ pos read error!");
                return;
            }
            else{
                //Store the pos in the robot info
                self.ori = Option::from((ori_vec[0], ori_vec[1], ori_vec[2]));
            }


        }else{
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            return;
        }
    }

    fn req_jnt_angs(&self){
        todo!()
    }

    fn req_force(&self){
        todo!()
    }

    fn req_rob_mov_state(&self){
        todo!()
    }

    fn req_model(&self){
        todo!()
    }

    fn update_rob_info(&self) {
        todo!()
    }



}
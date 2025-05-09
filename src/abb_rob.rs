

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



pub const IMPL_COMMDS : [&str; 9] = ["info", "cmds", "disconnect", "set joints", "set orientation", "move tool", "set pos", "req xyz", "req ori"];


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
        println!("Disconnected... Moving back to core command handler");
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
                    println!("Robot controller connected to - {}", self.req_model());
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
                    self.traj_queue_go();
                    self.traj_queue_stop();
                },

                _ => println!("Unknown command - see CMDs for list of commands"),
            }
        }
    }

    //Ping the robot to check the connection
    pub fn ping(&mut self){
        let s = self.socket.req("ECHO:PING");

        println!("Ping recieved - {}", s.unwrap());

    }

    //Request the robot move to specific joint angles
    fn set_joints(&mut self, angs : (f32, f32, f32, f32, f32, f32)){

        //Check to see if a response was returned
        if let Some(resp) = self.socket.req(&format!("STJT:[[{},{},{},{},{},{}], [9E9,9E9,9E9,9E9,9E9,9E9]]",angs.0, angs.1, angs.2, angs.3, angs.4, angs.5)){
            println!("{}", resp);
        }else{
            //Warn the user that the robot didn't respond
            println!("Warning no response! Robot may not have moved");
        }

    }

    //Set the orientation of the robots tcp
    fn set_ori(&self){
        todo!()
    }

    //Set the speed of the robot TCP
    fn set_speed(&mut self, speed: f32){
        if let Some(resp) = self.socket.req(&format!("STSP:{}", speed)){
            //Do nothing - no user notification required
            println!("{resp}");
        }else{
            //Warn the user the speed might not have changed
            println!("Danger! No response recieved - unknown robot speed");
        }
    }


    //Move the tool in its own local coordinate system
    fn move_tool(&self){
        todo!()
    }

    //Set the TCP point within the global coordinate system
    fn set_pos(&self){
        todo!()
    }

    //Add a translational movement to the robot movement queue
    fn traj_queue_add_trans(&self){
        todo!()
    }

    //Add a rotational movement to the robot movement queue
    fn traj_queue_add_rot(&self){
        todo!()
    }

    //Set the trajectory queue flag high - telling the robot to begin the trajectory queue
    fn traj_queue_go(&mut self){
        if let Some(resp) = self.socket.req("TJGO:0"){
            println!("{resp}");
        }else{
            println!("Warning - no response from robot trajectory may not begin!");
        }
    }

    //Set the trajectory queue flag low - telling the robot to stop the trajectory queue
    fn traj_queue_stop(&mut self){
        if let Some(resp) = self.socket.req("TJST:0"){
            println!("{resp}");
        }else{
            println!("Warning - no response from robot trajectory may not stop!");
        }
    }

    //Requests robot state of trajectory
    fn get_traj_done_flag(&mut self) -> Option<bool>{
        
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

    //Requests the orientation information
    fn req_ori(&mut self){
        //Request the info
        if let Some(recv) = self.socket.req("GTOR:0"){

            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let ori_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if ori_vec.len() != 3{
                println!("ORI pos read error!");
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

    //Requests joint angle information
    fn req_jnt_angs(&mut self){
        //Request the info
        if let Some(recv) = self.socket.req("GTJA:0"){

            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let jtang_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if jtang_vec.len() != 6{
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", jtang_vec.len());
                return;
            }
            else{
                //Store the pos in the robot info
                self.jnt_angles = Option::from((jtang_vec[0], jtang_vec[1], jtang_vec[2], jtang_vec[3], jtang_vec[4], jtang_vec[5]));
            }


        }else{
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            return;
        }
    }

    //Requests 6-axis force information
    fn req_force(&mut self){
        //Request the info
        if let Some(recv) = self.socket.req("GTFC:0"){

            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let fc_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if fc_vec.len() != 6{
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", fc_vec.len());
                return;
            }
            else{
                //Store the pos in the robot info
                self.force = Option::from((fc_vec[0], fc_vec[1], fc_vec[2], fc_vec[3], fc_vec[4], fc_vec[5]));
            }


        }else{
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            return;
        }
    }

    //Requests robot move state information flag
    fn req_rob_mov_state(&mut self){

        //Get the value of the move state flag - 1 indicating not moving
        if let Some(truth_val) = self.socket.req("MVST:0"){

            match truth_val.as_str() {
                "0" => {
                    self.move_flag = true;
                },
                "1" => {
                    self.move_flag = false
                }
                _ => {println!("Warning - invalid get move response! - got {}", truth_val.as_str())}
            }
        }else{
            println!("Warning - Robot possibly disconnected!");
        }



    }

    //Requests the model name of the robot
    fn req_model(&mut self) -> String{
        //Request the model name
        if let Some(model) = self.socket.req("RMDL:0"){
            model
        }else{
            String::from("[WRN - Unable to identify model]")
        }
    }

    //Helper function that requests all the update information from the robot
    fn update_rob_info(&mut self) {

        self.req_xyz();
        self.req_ori();
        self.req_force();
        self.req_rob_mov_state();
        self.get_traj_done_flag();

    }



}
use std::io;
use std::io::{Error, ErrorKind};
use crate::tcp_sock;
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


pub const IMPL_COMMDS: [&str; 5] = ["", "", "", "", ""];


impl AbbRob {   
    
    
    
    pub fn create_rob(ip: String, port: u32) -> Result<AbbRob, io::Error> {

        //Create the robots socket
        let mut rob_sock = create_sock(ip, port);
        //Attempt to connect to the robot
        if rob_sock.connect() == false {
            //Failed to connect
            Err(Error::from(ErrorKind::NotConnected))
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

            Ok(new_rob)
        }

    }

    //TODO - When communicating with robot - handle chance that robot has disconnected without crashing program


    //Disconnect from the robot - don't change any robot info, chances are the robot is going out of scope after this
    pub fn disconnect_rob(&mut self){
        self.socket.disconnect();
    }


    pub fn ping(&mut self){
        let s = self.socket.req(String::from("ECHO:PING"));

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
        match self.socket.req(String::from("TJDN:?")){
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



    fn req_xyz(&self){
        todo!()
    }

    fn req_ori(&self){
        todo!()
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
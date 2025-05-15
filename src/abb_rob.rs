use std::fs::OpenOptions;
use crate::tcp_sock::create_sock;
use crate::{angle_tools, string_tools, tcp_sock, trajectory_planner};
use std::io::{stdin, prelude::*};
use std::time::SystemTime;

pub struct AbbRob {
    socket: tcp_sock::TcpSock,
    pos: Option<(f32, f32, f32)>,
    ori: Option<(f32, f32, f32)>,
    jnt_angles: Option<(f32, f32, f32, f32, f32, f32)>,
    force: Option<(f32, f32, f32, f32, f32, f32)>,
    move_flag: bool,
    traj_done_flag: bool,
    disconnected : bool,
}

pub const IMPL_COMMDS: [&str; 9] = [
    "info",
    "cmds",
    "disconnect",
    "set joints",
    "set orientation",
    "move tool",
    "set pos",
    "req xyz",
    "req ori",
];

impl AbbRob {
    pub fn create_rob(ip: String, port: u32) -> Option<AbbRob> {
        //Create the robots socket
        let mut rob_sock = create_sock(ip, port);
        //Attempt to connect to the robot
        if rob_sock.connect() == false {
            //Failed to connect
            println!("Robot not connected");
            None
        } else {
            let new_rob = AbbRob {
                socket: rob_sock,
                pos: None,
                ori: None,
                jnt_angles: None,
                force: None,
                move_flag: false,
                traj_done_flag: false,
                disconnected : false
            };

            Option::from(new_rob)
        }
    }

    //TODO - Implement robot reconnect function

    //Disconnect from the robot - don't change any robot info, chances are the robot is going out of scope after this
    pub fn disconnect_rob(&mut self) {
        self.socket.disconnect();
        println!("Disconnected... Moving back to core command handler");
    }

    pub fn rob_cmd_handler(&mut self) {
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
                }

                "req xyz" => {
                    self.req_xyz();
                }

                "req ori" => {
                    self.req_ori();
                }

                "trajectory" =>{
                    self.run_test();
                }

                //Whatever function is being tested at the moment
                "test" => {
                    self.traj_queue_add_rot(angle_tools::Quartenion {
                        w: 0.07731,
                        x: -0.88577,
                        y: 0.45533,
                        z: -0.04604,
                    });
                    self.traj_queue_add_trans((790.0, 2300.0, 1400.0));

                    self.traj_queue_go();
                }


                _ => println!("Unknown command - see CMDs for list of commands"),
            }
        }
    }

    //Ping the robot to check the connection
    pub fn ping(&mut self) {
        let s = self.socket.req("ECHO:PING");

        println!("Ping recieved - {}", s.unwrap());
    }

    //Request the robot move to specific joint angles
    fn set_joints(&mut self, angs: (f32, f32, f32, f32, f32, f32)) {
        //Check to see if a response was returned
        if let Some(_resp) = self.socket.req(&format!(
            "STJT:[[{},{},{},{},{},{}], [9E9,9E9,9E9,9E9,9E9,9E9]]",
            angs.0, angs.1, angs.2, angs.3, angs.4, angs.5
        )) {
            //Update the robot info
            self.update_rob_info();
        } else {
            //Warn the user that the robot didn't respond
            println!("Warning no response! Robot may not have moved");
        }
    }

    //Set the orientation of the robots tcp
    //Currently assumes that the quartenion is valid
    //q - the desired orientation
    fn set_ori(&mut self, q: angle_tools::Quartenion) {
        if let Some(_resp) = self
            .socket
            .req(&format!("STOR:[{}, {}, {}, {}]", q.w, q.x, q.y, q.z))
        {
            println!("Response!");
            self.update_rob_info();
        } else {
            println!("Warning - no robot response! - Robot might not reorient!");
        }
    }

    //Set the speed of the robot TCP
    fn set_speed(&mut self, speed: f32) {
        if let Some(resp) = self.socket.req(&format!("STSP:{}", speed)) {
            //Do nothing - no user notification required
            println!("{resp}");
        } else {
            //Warn the user the speed might not have changed
            println!("Danger! No response recieved - unknown robot speed");
        }
    }

    //Move the tool relative to its own local coordinate system
    //xyz - how far the tcp will move in each caridnal direction
    fn move_tool(&mut self, xyz: (f32, f32, f32)) {
        if let Some(_resp) = self
            .socket
            .req(&format!("MVTL:[{},{},{}]", xyz.0, xyz.1, xyz.2))
        {
            self.update_rob_info();
        } else {
            println!("Warning - repsonse not recieved! - Robot may not move")
        }
    }

    //Set the TCP point within the global coordinate system
    //xyz - desired position in cartesian coordinates
    fn set_pos(&mut self, xyz: (f32, f32, f32)) {
        if let Some(_resp) = self
            .socket
            .req(&format!("MVTO:[{},{},{}]", xyz.0, xyz.1, xyz.2))
        {
            println!("Response!");
            self.update_rob_info();
        } else {
            println!("Warning - no response - robot may not move");
        }
    }

    //Add a translational movement to the robot movement queue
    fn traj_queue_add_trans(&mut self, xyz: (f32, f32, f32)) {
        if let Some(_resp) = self
            .socket
            .req(&format!("TQAD:[{},{},{}]", xyz.0, xyz.1, xyz.2))
        {
            println!("trans traj added");
        } else {
            println!("Warning - no response - trajectory may differ from expected!");
        }
    }

    //Add a rotational movement to the robot movement queue
    fn traj_queue_add_rot(&mut self, q: angle_tools::Quartenion) {
        if let Some(_resp) = self
            .socket
            .req(&format!("RQAD:[{}, {}, {}, {}]", q.w, q.x, q.y, q.z))
        {
            println!("rot traj added");
        } else {
            println!("Warning - no response - trajectory may differ from expected!");
        }
    }

    //Set the trajectory queue flag high - telling the robot to begin the trajectory queue
    fn traj_queue_go(&mut self) {
        if let Some(resp) = self.socket.req("TJGO:0") {
            println!("{resp}");
        } else {
            println!("Warning - no response from robot trajectory may not begin!");
        }
    }

    //Set the trajectory queue flag low - telling the robot to stop the trajectory queue
    fn traj_queue_stop(&mut self) {
        if let Some(resp) = self.socket.req("TJST:0") {
            println!("{resp}");
        } else {
            println!("Warning - no response from robot trajectory may not stop!");
        }
    }

    //Requests robot state of trajectory
    fn get_traj_done_flag(&mut self) -> Option<bool> {
        //Safe socket read - incase the socket crashes
        match self.socket.req("TJDN:?") {
            Some(recv) => {
                if recv == "TRUE" {
                    self.traj_done_flag = true;
                    Option::from(true)
                }else if recv == "FALSE"{
                    self.traj_done_flag = false;
                    Option::from(false)
                }
                else {
                    println!("Warning - Trajectory flag error! - Unknown state!");
                    println!("Got response - `{recv}`");
                    None
                }
            }
            None => {
                println!("WARNING ROBOT DISCONNECTED");
                self.disconnected = true;
                None
            }
        }
    }

    //Essentially another command line handler - just for running specific tests
    fn run_test(&mut self){


        println!("Specifiy the trajectory you wish to run");

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
                    println!("Specify the trajectory you wish to run");
                }
                //Print out the commands in the valid commands list

                "exit" => {
                    return
                }

                //Capture all other
                other => {
                    //If the trajectory is valid - i.e. it has been programmed
                    if let Some(traj) = trajectory_planner::traj_gen(other) {

                        //Create the filename
                        println!("Please provide a filename");

                        //Get user input
                        let mut user_inp = String::new();
                        stdin()
                            .read_line(&mut user_inp)
                            .expect("Failed to read line");

                        let filename = format!("C:/Users/User/Documents/Results/rustbot_dumps/{}", user_inp);

                        //Place all the trajectories in the queue
                        for pnt in traj{
                            self.traj_queue_add_trans(pnt);
                        }

                        //Start the trajectory
                        self.traj_queue_go();

                        //Read the values once
                        self.update_rob_info();

                        let mut cnt = 0;

                        //Read the values until the trajectory is reported as done
                        while !self.traj_done_flag{
                            self.update_rob_info();
                            self.store_state(&filename, cnt);

                            //Increase the count
                            cnt = cnt + 1;



                            if self.disconnected{
                                println!("Warning - disconnected during test");
                                return;
                            }

                        }

                        println!("Trajectory done!");

                        return


                    }
                }

            }


        }
    }


    //Requests the xyz position of the TCP from the robot and stores it in the robot info
    fn req_xyz(&mut self) {
        //Request the info
        if let Some(recv) = self.socket.req("GTPS:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let xyz_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if xyz_vec.len() != 3 {
                println!("XYZ pos read error!");
                return;
            } else {
                //Store the pos in the robot info
                self.pos = Option::from((xyz_vec[0], xyz_vec[1], xyz_vec[2]));
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
            return;
        }
    }

    //Requests the orientation information
    fn req_ori(&mut self) {
        //Request the info
        if let Some(recv) = self.socket.req("GTOR:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let ori_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if ori_vec.len() != 3 {
                println!("ORI pos read error!");
                return;
            } else {
                //Store the pos in the robot info
                self.ori = Option::from((ori_vec[0], ori_vec[1], ori_vec[2]));
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
            return;
        }
    }

    //Requests joint angle information
    fn req_jnt_angs(&mut self) {
        //Request the info
        if let Some(recv) = self.socket.req("GTJA:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let jtang_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if jtang_vec.len() != 6 {
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", jtang_vec.len());
                return;
            } else {
                //Store the pos in the robot info
                self.jnt_angles = Option::from((
                    jtang_vec[0],
                    jtang_vec[1],
                    jtang_vec[2],
                    jtang_vec[3],
                    jtang_vec[4],
                    jtang_vec[5],
                ));
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
            return;
        }
    }

    //Requests 6-axis force information
    fn req_force(&mut self) {
        //Request the info
        if let Some(recv) = self.socket.req("GTFC:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let fc_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if fc_vec.len() != 6 {
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", fc_vec.len());
                return;
            } else {
                //Store the pos in the robot info
                self.force = Option::from((
                    fc_vec[0], fc_vec[1], fc_vec[2], fc_vec[3], fc_vec[4], fc_vec[5],
                ));
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
            return;
        }
    }

    //Requests robot move state information flag
    fn req_rob_mov_state(&mut self) {
        //Get the value of the move state flag - 1 indicating not moving
        if let Some(truth_val) = self.socket.req("MVST:0") {
            match truth_val.as_str() {
                "0" => {
                    self.move_flag = true;
                }
                "1" => self.move_flag = false,
                _ => {
                    println!(
                        "Warning - invalid get move response! - got {}",
                        truth_val.as_str()
                    )
                }
            }
        } else {
            println!("Warning - Robot possibly disconnected!");
            self.disconnected = true;
        }
    }

    //Requests the model name of the robot
    fn req_model(&mut self) -> String {
        //Request the model name
        if let Some(model) = self.socket.req("RMDL:0") {
            model
        } else {
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


    //Appends relevant test information to the provided filename
    fn store_state(&mut self, filename : &String, i : i32){
        

        //Open the file (or create if it doesn't exist)
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(filename.trim())
            .unwrap();

        //Format the line to write
        let line = format!("{},{:?},[{},{},{}],[{},{},{}],[{},{},{},{},{},{}]",
                           i,
                            SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs_f64(),
                            //Make sure that you dont print a lack of information in the data
                            //TODO - Create formatting function - that can print blank spots
                           self.pos.expect("Err - reading pos - x").0,
                           self.pos.expect("Err - reading pos - y").1,
                           self.pos.expect("Err - reading pos - z").2,
                           self.ori.expect("Err - reading ori - x").0,
                           self.ori.expect("Err - reading ori - y").1,
                           self.ori.expect("Err - reading pos - z").2,
                           self.force.expect("Err - reading force - x").0,
                           self.force.expect("Err - reading force - y").1,
                           self.force.expect("Err - reading force - z").2,
                           self.force.expect("Err - reading moment - x").3,
                           self.force.expect("Err - reading moment - y").4,
                           self.force.expect("Err - reading moment - z").5,
        );

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line){
                eprint!("Couldn't write to file: {}", e);
            }
        }



    }


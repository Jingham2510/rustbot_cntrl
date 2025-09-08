use crate::control::misc_tools::{angle_tools, string_tools};
use crate::control::{tcp_sock, trajectory_planner};
use crate::mapping::terr_map_sense;
use crate::mapping::terr_map_tools::Heightmap;
use anyhow::bail;
use std::fs::OpenOptions;
use std::io::{prelude::*, stdin};
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::thread;
use std::time::SystemTime;


pub struct AbbRob {
    socket: tcp_sock::TcpSock,
    pos: (f32, f32, f32),
    ori: (f32, f32, f32),
    jnt_angles: (f32, f32, f32, f32, f32, f32),
    force: (f32, f32, f32, f32, f32, f32),
    move_flag: bool,
    traj_done_flag: bool,
    disconnected : bool,
}

pub const IMPL_COMMDS: [&str; 8] = [
    "info",
    "cmds",
    "disconnect",
    "trajectory",
    "test",
    "home",
    "req xyz",
    "req ori",
];

impl AbbRob {
    pub fn create_rob(ip: String, port: u32) -> Result<AbbRob, anyhow::Error> {
        //Create the robots socket
        let mut rob_sock = tcp_sock::create_sock(ip, port);
        //Attempt to connect to the robot
        if rob_sock.connect() == false {
            //Failed to connect
            bail!("Robot not connected")

        } else {
            let new_rob = AbbRob {
                socket: rob_sock,
                pos: (f32::NAN, f32::NAN, f32::NAN),
                ori: (f32::NAN, f32::NAN, f32::NAN),
                jnt_angles: (f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN),
                force: (f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN),
                move_flag: false,
                traj_done_flag: false,
                disconnected : false
            };

            Ok(new_rob)
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

                "home" => {
                    self.go_home_pos();
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
        if let Ok(_resp) = self.socket.req(&format!(
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

    fn go_home_pos(&mut self){
        //Define the home point
        let home_pos = (177.77, 1777.27, 350.0);
        
        //Define the home orientation
        let home_ori = angle_tools::Quartenion{
            w : 0.02607,
            x : -0.76666,
            y : 0.64128,
            z : 0.01799
        };


        self.set_speed(500.0);

        //Set the pos and ori
        self.set_pos(home_pos);
        
        self.set_ori(home_ori);

        self.set_speed(50.0);

        println!("Home!");

    }

    //Set the orientation of the robots tcp
    //Currently assumes that the quartenion is valid
    //q - the desired orientation
    fn set_ori(&mut self, q: angle_tools::Quartenion) {
        if let Ok(_resp) = self
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
        if let Ok(resp) = self.socket.req(&format!("STSP:{}", speed)) {
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
        if let Ok(_resp) = self
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
        if let Ok(_resp) = self
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
        if let Ok(_resp) = self
            .socket
            .req(&format!("TQAD:[{},{},{}]", xyz.0, xyz.1, xyz.2))
        {
            //println!("trans traj added");
        } else {
            println!("Warning - no response - trajectory may differ from expected!");
        }
    }

    //Add a rotational movement to the robot movement queue
    fn traj_queue_add_rot(&mut self, q: angle_tools::Quartenion) {
        if let Ok(_resp) = self
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
        if let Ok(resp) = self.socket.req("TJGO:0") {
            println!("{resp}");
        } else {
            println!("Warning - no response from robot trajectory may not begin!");
        }
    }

    //Set the trajectory queue flag low - telling the robot to stop the trajectory queue
    fn traj_queue_stop(&mut self) {
        if let Ok(resp) = self.socket.req("TJST:0") {
            println!("{resp}");
        } else {
            println!("Warning - no response from robot trajectory may not stop!");
        }
    }

    //Requests robot state of trajectory
    fn get_traj_done_flag(&mut self) -> Result<bool, anyhow::Error> {
        //Safe socket read - incase the socket crashes
        match self.socket.req("TJDN:?") {
            Ok(recv) => {
                if recv == "TRUE" {
                    self.traj_done_flag = true;
                    Ok(true)
                }else if recv == "FALSE"{
                    self.traj_done_flag = false;
                    Ok(false)
                }
                else {
                    println!("Warning - Trajectory flag error! - Unknown state!");
                    println!("Got response - `{recv}`");
                    bail!("Trajectory flag error");
                }
            }
            Err(e) => {
                println!("WARNING ROBOT DISCONNECTED - {e}");
                self.disconnected = true;
                bail!("ROBOT DISCONNECTED");
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

                        let filename = format!("dump/{}.txt", user_inp.trim());

                        //Move to a starting point - above the starting point
                        self.set_pos((traj[0].0, traj[0].1, traj[0].2 + 25.0));


                        //Place all the trajectories in the queue
                        for pnt in traj{
                            self.traj_queue_add_trans(pnt);
                        }
                        
                        //Create a threading channel
                        let (tx, rx) = mpsc::channel();

                        //Create the thread that handles the depth camera
                        let cam_thread = thread::spawn( move ||
                            Self::depth_sensing(rx, &*user_inp.trim(), true));

                        //Start the trajectory
                        self.traj_queue_go();

                        //Read the values once
                        self.update_rob_info();

                        let mut cnt = 0;

                        const DEPTH_FREQ : i32 = 500;

                        //Read the values until the trajectory is reported as done
                        while !self.traj_done_flag{
                            self.update_rob_info();
                            self.store_state(&filename, cnt);

                            //Increase the count
                            cnt = cnt + 1;

                            if cnt % DEPTH_FREQ == 0{
                                if let Ok(_) = tx.send(true){
                                    //Do nothing here - normal operation
                                    println!("trigger sent")
                                }else{
                                    println!("Warning - Cam thread dead!");
                                }
                            }




                            if self.disconnected{
                                println!("Warning - disconnected during test");
                                return;
                            }

                        }

                        println!("Trajectory done!");

                        return


                    }else{
                        return;
                    }
                }

            }


        }
    }

    //Function which repeatedly takes depth measurements on trigger from another thread
    fn depth_sensing(rx : Receiver<bool>, test_name : &str, hmap : bool){

        //Create a camera
        let mut cam = terr_map_sense::RealsenseCam::initialise().expect("Failed to create camera");

        let mut cnt = 0;

        //Loop forever - will be killed once the test ends automatically
        loop {

            //Block until the trigger is recieved
            rx.recv().expect("recieve thread error");

            println!("Taking depth measure");

            //Create a pointcloud
            let mut curr_pcl = cam.get_depth_pnts().expect("Failed to get get pointcloud");

            //For now - rotate and filter the cloud automatically - assume that we are working with the terrain box in the TRL
            curr_pcl.rotate(std::f32::consts::PI / 4.0, 0.0, 0.0);
            //Empirically calculated passband to isolate terrain bed
            curr_pcl.passband_filter(-1.0, 1.0, -3.8, -0.9, -0.0, 1.3);

            //Save the pointcloud
            let filename = format!("{test_name}_{cnt}");
            println!("{}", filename);

            curr_pcl.save_to_file(&*filename).unwrap();


            if hmap{
                //Create a heightmap from the pointcloud
                let mut curr_hmap = Heightmap::create_from_pcl(curr_pcl, 250, 250, false);


                //Save the heightmap
                let filename = format!("{test_name}_hmap_{cnt}");
                curr_hmap.save_to_file(&*filename).unwrap()

            }

            //Increase the loop count
            cnt = cnt + 1;


        }





    }


    //Requests the xyz position of the TCP from the robot and stores it in the robot info
    fn req_xyz(&mut self) {
        //Request the info
        if let Ok(recv) = self.socket.req("GTPS:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let xyz_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if xyz_vec.len() != 3 {
                println!("XYZ pos read error!");
                self.pos = (f32::NAN, f32::NAN, f32::NAN);
                return
            } else {
                //Store the pos in the robot info
                self.pos = (xyz_vec[0], xyz_vec[1], xyz_vec[2]);
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
            return
        }
    }

    //Requests the orientation information
    fn req_ori(&mut self) {
        //Request the info
        if let Ok(recv) = self.socket.req("GTOR:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let ori_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if ori_vec.len() != 3 {
                println!("ORI pos read error!");
                self.ori = (f32::NAN, f32::NAN, f32::NAN);
                return;
            } else {
                //Store the pos in the robot info
                self.ori = (ori_vec[0], ori_vec[1], ori_vec[2]);
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
        if let Ok(recv) = self.socket.req("GTJA:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let jtang_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if jtang_vec.len() != 6 {
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", jtang_vec.len());
                self.jnt_angles = (f32::NAN, f32::NAN,f32::NAN, f32::NAN,f32::NAN, f32::NAN);
                return;
            } else {
                //Store the pos in the robot info
                self.jnt_angles = (
                    jtang_vec[0],
                    jtang_vec[1],
                    jtang_vec[2],
                    jtang_vec[3],
                    jtang_vec[4],
                    jtang_vec[5],
                );
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
        if let Ok(recv) = self.socket.req("GTFC:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&*recv);
            let fc_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if fc_vec.len() != 6 {
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", fc_vec.len());
                self.force = (f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN);
                return;
            } else {
                //Store the pos in the robot info
                self.force = (
                    fc_vec[0], fc_vec[1], fc_vec[2], fc_vec[3], fc_vec[4], fc_vec[5],
                );
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
        if let Ok(truth_val) = self.socket.req("MVST:0") {
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
        if let Ok(model) = self.socket.req("RMDL:0") {
            model
        } else {
            String::from("[WRN - Unable to identify model]")
        }
    }

    //Helper function that requests all the update information from the robot
    //Returns early from the function if the robot has disconnected
    //TODO - Check to see if there is a cleaner way to check if disconnected?
    fn update_rob_info(&mut self) {
        self.req_xyz();        
        if self.disconnected{
            return;
        }          
        self.req_ori();

        if self.disconnected{
            return;
        }
        self.req_force();

        if self.disconnected{
            return;
        }
        self.req_rob_mov_state();

        if self.disconnected{
            return;
        }
        
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
                           self.pos.0,
                           self.pos.1,
                           self.pos.2,
                           self.ori.0,
                           self.ori.1,
                           self.ori.2,
                           self.force.0,
                           self.force.1,
                           self.force.2,
                           self.force.3,
                           self.force.4,
                           self.force.5,
        );

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line){
                eprint!("Couldn't write to file: {}", e);
            }
        }



    }


use crate::control::misc_tools::{angle_tools, string_tools};
use crate::control::{tcp_sock, trajectory_planner};
use crate::mapping::terr_map_sense;
use crate::mapping::terr_map_tools::Heightmap;
use anyhow::bail;
use std::fs::OpenOptions;
use std::io::{prelude::*, stdin};
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, SystemTime};
use std::{fs, thread};
use std::thread::sleep;
use crate::config::{Config};

pub struct AbbRob<'a> {
    socket: tcp_sock::TcpSock,
    pos: (f32, f32, f32),
    ori: (f32, f32, f32),
    jnt_angles: (f32, f32, f32, f32, f32, f32),
    force: (f32, f32, f32, f32, f32, f32),
    move_flag: bool,
    traj_done_flag: bool,
    disconnected: bool,
    //Programme setup config
    config : &'a Config
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

const TRANSFORM_TO_WORK_SPACE : bool = true;

impl AbbRob<'_> {
    pub fn create_rob(ip: String, port: u32, config : &Config) -> Result<AbbRob, anyhow::Error> {
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
                disconnected: false,
                config
            };

            Ok(new_rob)
        }
    }

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

                "trajectory" => {
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

    fn go_home_pos(&mut self) {
        //Define the home point
        const HOME_POS : (f32, f32, f32) = (220.0, 1256.0, 955.0);

        //Define the home orientation
        const HOME_ORI : angle_tools::Quartenion = angle_tools::Quartenion {
            w: 0.00203,
            x: -0.98623,
            y: -0.16536,
            z: 0.00062,
        };

        self.set_speed(150.0);

        //Set the pos and ori
        self.set_pos(HOME_POS);

        self.set_ori(HOME_ORI);

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
    //block - indicates whether the move should block other transmission
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
                } else if recv == "FALSE" {
                    self.traj_done_flag = false;
                    Ok(false)
                } else {
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
    fn run_test(&mut self) {
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
                "exit" => return,

                //Capture all other
                other => {
                    //If the trajectory is valid - i.e. it has been programmed
                    if let Some(traj) = trajectory_planner::traj_gen(other) {
                        //Create the filename
                        println!("Please provide a test name");

                        //Get user input
                        let mut user_inp = String::new();
                        stdin()
                            .read_line(&mut user_inp)
                            .expect("Failed to read line");

                        //Create a folder to hold the test data
                        let new_fp = format!("{}/{}", self.config.test_fp(), user_inp.trim());
                        fs::create_dir(&new_fp).expect("FAILED TO CREATE NEW DIRECTORY");

                        let data_filename = format!("{}/data_{}.txt", new_fp, user_inp.trim());

                        let config_filename = format!("{}/conf_{}.txt", new_fp, user_inp.trim());

                        //Log the camera config
                        self.log_config(config_filename);

                        //Create a threading channel to trigger the camera
                        let (tx, rx) = mpsc::channel();

                        //Clone the cam configs to avoid handing ownership to the thread
                        let rel_pos = self.config.cam_info.rel_pos().clone();
                        let rel_ori = self.config.cam_info.rel_ori().clone();
                        let scale = self.config.cam_info.x_scale();


                        //Create the thread that handles the depth camera
                        thread::spawn(move || Self::depth_sensing(rx, new_fp, &*user_inp.trim(), true, rel_pos, rel_ori, scale));

                        sleep(Duration::from_secs(2));

                        if let Ok(_) = tx.send(4){}
                        //Move to the home position - blocker
                        self.go_home_pos();

                        //Trigger before the test begins
                        if let Ok(_) = tx.send(2){}


                        let start_pos = (traj[0].0, traj[0].1, traj[0].2 + 25.0);
                        
                        //Move to a starting point - above the starting point
                        self.set_pos(start_pos);


                        //Place all the trajectories in the queue
                        for pnt in traj {
                            self.traj_queue_add_trans(pnt);
                        }


                        //Start the trajectory
                        self.traj_queue_go();

                        //Read the values once
                        self.update_rob_info();

                        let mut cnt = 0;
                        const DEPTH_FREQ: i32 = 250;

                        //Read the values until the trajectory is reported as done
                        while !self.traj_done_flag {
                            self.update_rob_info();
                            self.store_state(&data_filename, cnt, TRANSFORM_TO_WORK_SPACE);

                            //Trigger at the start - or at a specified interval
                            if cnt % DEPTH_FREQ == 0 || cnt == 0 {
                                if let Ok(_) = tx.send(1) {
                                    //Do nothing here - normal operation
                                } else {
                                    println!("Warning - Cam thread dead!");
                                }
                            }

                            //Increase the count
                            cnt = cnt + 1;

                            if self.disconnected {
                                println!("Warning - disconnected during test");
                                tx.send(0);
                                return;
                            }
                        }
                        

                        //Go back to home pos
                        self.go_home_pos();
                        //No point error handling - if this fails the test is done anyway
                        tx.send(3);
                        
                        println!("Trajectory done!");

                        tx.send(0);

                        return;
                    } else {
                        return;
                    }
                }
            }
        }
    }

    //Function which repeatedly takes depth measurements on trigger from another thread
    fn depth_sensing(rx: Receiver<u32>, filepath : String, test_name: &str, hmap: bool, rel_pos : [f32;3], rel_ori : [f32;3], scale : f32) {
        //Create a camera
        let mut cam = terr_map_sense::RealsenseCam::initialise().expect("Failed to create camera");

        let mut cnt = 0;

        //Loop forever - will be killed once the test ends automatically
        loop {

            let opt = rx.recv().expect("recieve thread error");

            //Block until the trigger is recieved
            if opt > 0 {
                println!("Taking depth measure");

                //Create a pointcloud
                let mut curr_pcl = cam.get_depth_pnts().expect("Failed to get get pointcloud");

                //filter the data - save raw (transformed to robot frame)
                curr_pcl.scale_even(scale);
                curr_pcl.rotate( rel_ori[0], rel_ori[1], rel_ori[2]);
                curr_pcl.translate(rel_pos[0], rel_pos[1], rel_pos[2]);

                curr_pcl.passband_filter(-10.0, 2000.0, -10.0, 2000.0, -150.0, 200.0);

                //Save the pointcloud
                let pcl_filepath : String;

                match opt{
                    1 => {
                        pcl_filepath = format!("{filepath}/pcl_{test_name}_{cnt}")
                    }
                    2=> {
                        pcl_filepath = format!("{filepath}/pcl_{test_name}_START")
                    }
                    3=> {
                        pcl_filepath = format!("{filepath}/pcl_{test_name}_END")
                    }

                    //Sacrificial scan (i.e. to get a crappy one out the way
                    4 =>{
                        println!("THROWAWAY SCAN");
                        //Sleep to try let the cam warm
                        sleep(Duration::from_secs(2));
                        continue

                    }

                    //If invalid number just take a count
                    _ =>{
                        pcl_filepath = format!("{filepath}/pcl_{test_name}_{cnt}")
                    }
                }
                

                curr_pcl.save_to_file(&*pcl_filepath).unwrap();

                if hmap {
                    //Create a heightmap from the pointcloud
                    let mut curr_hmap = Heightmap::create_from_pcl(curr_pcl, 200, 200);

                    //Save the heightmap
                    let hmap_filepath:String;

                    match opt{
                        1 => {
                            hmap_filepath = format!("{filepath}/hmap_{test_name}_{cnt}")
                        }
                        2=> {
                            hmap_filepath = format!("{filepath}/hmap_{test_name}_START")
                        }
                        3=> {
                            hmap_filepath = format!("{filepath}/hmap_{test_name}_END")
                        }

                        //If invalid number just take a count
                        _ =>{
                            hmap_filepath = format!("{filepath}/hmap_{test_name}_{cnt}")
                        }
                    }

                    println!("{hmap_filepath}");
                    curr_hmap.save_to_file(&*hmap_filepath).unwrap()
                }

                if opt == 1{
                    //Increase the loop count
                    cnt = cnt + 1;
                }

            } else {
                println!("Closing cam thread");
                return;
            }
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
                return;
            } else {
                //Store the pos in the robot info
                self.pos = (xyz_vec[0], xyz_vec[1], xyz_vec[2]);
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
                self.jnt_angles = (f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN);
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
    fn update_rob_info(&mut self) {
        self.req_xyz();
        if self.disconnected {
            return;
        }
        self.req_ori();

        if self.disconnected {
            return;
        }
        self.req_force();

        if self.disconnected {
            return;
        }
        self.req_rob_mov_state();

        if self.disconnected {
            return;
        }

        self.get_traj_done_flag();
    }

    //Appends relevant test information to the provided filename
    fn store_state(&mut self, filename: &String, i: i32, transform_to_work_space : bool) {
        //Open the file (or create if it doesn't exist)
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(filename.trim())
            .unwrap();

        let mut line = String::new();

        //See whether to transofmr the data by the
        if !transform_to_work_space {
            //Format the line to write
            line = format!(
                "{},{:?},[{},{},{}],[{},{},{}],[{},{},{},{},{},{}]",
                i,
                SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_secs_f64(),
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
        }else{
            line = format!(
                "{},{:?},[{},{},{}],[{},{},{}],[{},{},{},{},{},{}]",
                i,
                SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_secs_f64(),
                //Make sure that you dont print a lack of information in the data
                self.pos.0 - self.config.rob_info.pos_to_zero()[0],
                self.pos.1 - self.config.rob_info.pos_to_zero()[1],
                self.pos.2 - self.config.rob_info.pos_to_zero()[2],
                self.ori.0 - self.config.rob_info.ori_to_zero()[0],
                self.ori.1 - self.config.rob_info.ori_to_zero()[1],
                self.ori.2 - self.config.rob_info.ori_to_zero()[2],
                self.force.0,
                self.force.1,
                self.force.2,
                self.force.3,
                self.force.4,
                self.force.5,
            );

        }

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line) {
            eprint!("Couldn't write to file: {}", e);
        }
    }

    fn log_config(&mut self, filepath: String){

        println!("{filepath}");

        //Create the config file and save the config info
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(filepath.trim())
            .unwrap();

        //Save the cam info
        let line = format!("CAM: POS:[{},{},{}] ORI:[{},{},{}] X_SC:[{}] Y_SC:[{}]",
                           self.config.cam_info.rel_pos()[0],
                           self.config.cam_info.rel_pos()[1],
                           self.config.cam_info.rel_pos()[2],
                           self.config.cam_info.rel_ori()[0],
                           self.config.cam_info.rel_ori()[1],
                           self.config.cam_info.rel_ori()[2],
                           self.config.cam_info.x_scale(),
                           self.config.cam_info.y_scale()

        );

        writeln!(file, "{}", line).expect("FAILED TO WRITE CAM TO CONFIG - CLOSING");

        //Save another line with the robot pos/ori config data
        let line = format!("ROB: NAME: \"{}\" POS:[{},{},{}] ORI:[{},{},{}]",
                           self.config.rob_info.rob_name(),
                           self.config.rob_info.pos_to_zero()[0],
                           self.config.rob_info.pos_to_zero()[1],
                           self.config.rob_info.pos_to_zero()[2],
                           self.config.rob_info.ori_to_zero()[0],
                           self.config.rob_info.ori_to_zero()[1],
                           self.config.rob_info.ori_to_zero()[2],
        );

        writeln!(file, "{}", line).expect("FAILED TO WRITE ROB TO CONFIG - CLOSING");


        //Save a line indicating if the data is pre-transformed
        let line = format!("COORDS PRE-TRANSFORMED?:{}", TRANSFORM_TO_WORK_SPACE);

        writeln!(file, "{}", line).expect("FAILED TO WRITE TRANSFORM FLAG TO CONFIG - CLOSING");


    }


}

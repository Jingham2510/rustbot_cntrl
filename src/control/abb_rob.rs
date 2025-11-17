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
use crate::control::force_control;

pub struct AbbRob<'a> {
    socket: tcp_sock::TcpSock,
    pos: (f32, f32, f32),
    ori: (f32, f32, f32),
    jnt_angles: (f32, f32, f32, f32, f32, f32),
    force: (f32, f32, f32, f32, f32, f32),
    move_flag: bool,
    traj_done_flag: bool,
    disconnected: bool,
    force_mode_flag : bool,
    force_axis : String,
    force_target : f32,
    //Programme setup config
    config : &'a Config
}

pub const IMPL_COMMDS: [&str; 9] = [
    "info",
    "cmds",
    "disconnect",
    "trajectory",
    "force traj",
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
                force_mode_flag: false,
                force_axis : "Z".to_string(),
                force_target : 0.0,
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
                    println!("Robot controller connected to - {}", self.req_model().unwrap());
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
                    self.set_force_control_mode(false).expect("FAILED TO SET FORCE MODE");
                    self.run_test();
                }

                "force traj" =>{
                    self.set_force_control_mode(true).expect("FAILED TO SET FORCE MODE");


                    println!("Please type the target force");

                    let mut user_inp = String::new();
                    stdin()
                        .read_line(&mut user_inp)
                        .expect("Failed to read line");
                    
                    if let  Ok(targ) = user_inp.trim().parse::<f32>(){

                        self.set_force_config("Z", targ).expect("FAILED TO SET FORCE CONFIG");
                        self.run_test();

                    }else{
                        println!("Invalid target.... returning");
                        }



                }

                //Whatever function is being tested at the moment
                "test" => {
                    if let Ok(_) = self.rel_mv_queue_add((10.0, 11.0, 12.0)){

                        println!("Sent okay! - check values in RAPID");

                    }else{
                        println!("FAILED");

                    }


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
        const HOME_POS : (f32, f32, f32) = (220.0, 1355.0, 955.0);

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

    //Set the robot force mode
    fn set_force_control_mode(&mut self, force_control : bool) -> Result<(), anyhow::Error>{


        let fc_cntrl_string = format!("STFM:{}", force_control);

        if let Ok(resp) = self.socket.req(&*fc_cntrl_string) {

            let expected_resp = format!("FM:{}", force_control);

            if !resp.eq_ignore_ascii_case(&*expected_resp){
                bail!("Incorrect response, mode not changed!");
            }else{
                //Update internal flag
                self.force_mode_flag = force_control;
                Ok(())
            }
        } else {
            //This is a bail because it is safety critical that the mode is known
            bail!("Error - no response robot control mode may be incorrect!");

        }
    }

    //Set the force requirements for force control
    //Safety critical - always bail if state is possibly unknown!
    fn set_force_config(&mut self, ax : &str, target : f32) -> Result<(), anyhow::Error>{


        //Format the string request (SeT ForceConfig)
        let conf_str = format!("STFC:{}.{}", ax, target);
        
        //Send the request
        if let Ok(resp) = self.socket.req(&*conf_str){

            //Check that the robot has responded with the correct values (otherwise bail)
            let expected_resp = format!("FC:{}.{}", ax, target);


            if !resp.eq_ignore_ascii_case(&*expected_resp){
                bail!("Incorrect config! Force control will be incorrect")
            }else{
                self.force_axis = ax.to_string();
                self.force_target = target;
                Ok(())
            }
        }else{
            bail!("Error - no repsonse, cannot verify force config set!")
        }
    }


    /*
    Adds a relative move to the RAPID relative move queue (for force control)
     */
    fn rel_mv_queue_add(&mut self, rel_xyz : (f32, f32, f32)) -> Result<(), anyhow::Error>{

        //Format the string request
        let rel_mv_str = format!("RLAD:[{},{},{}]", rel_xyz.0, -rel_xyz.1, rel_xyz.2);

        //Check that the correct response is sent
        if let Ok(resp) = self.socket.req(&*rel_mv_str){

            let expected_resp = "OK";

            //println!("RECIEVED: {}", resp);
            //println!("EXPECTED: {}", expected_resp);

            if !resp.eq(expected_resp){
                bail!("Incorrect response - Relative movement will be incorrect")
            }else{
                Ok(())
            }

        }else{
            bail!("Error - no repsonse, cannot verify relative move added!")
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

                //Parse the trajectory requested
                other => {

                    let traj;

                    if (self.force_mode_flag){
                        traj = trajectory_planner::relative_traj_gen(other);

                    }else{
                        traj = trajectory_planner::traj_gen(other);
                    }
                    //If the trajectory is valid - i.e. it has been programmed
                    if traj.is_ok() {

                        let traj = traj.unwrap();

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


                        let start_pos = (traj[0].0, traj[0].1, traj[0].2);

                        
                        //Move to a starting point - above the starting point
                        self.set_pos(start_pos);

                        self.set_speed(10.0);

                        if (self.force_mode_flag){
                            for i in 1..traj.len(){
                                self.rel_mv_queue_add((traj[i].0, traj[i].1, traj[i].2)).expect("Failed to add relative move - BAILING");
                            }

                        }else{
                            //Place all the trajectories in the queue
                            for pnt in traj {
                                self.traj_queue_add_trans(pnt);
                            }

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

                            //Calculate the force error and correct
                            if self.force_mode_flag{
                                self.calc_force_err().unwrap();
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
    fn req_model(&mut self) -> Result<String,anyhow::Error> {
        //Request the model name
        if let Ok(model) = self.socket.req("RMDL:0") {
            Ok(model)
        } else {
            bail!("Unable to identify model]")
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


    //Calculate the error between the
    fn calc_force_err(&self) -> Result<f32, anyhow::Error>{

        //Check that force mode is enabled (otherwise theres no point in calcing the error
        if self.force_mode_flag{
            let force_val : f32;
            //Extract the correct axis information
            match self.force_axis.as_str(){

                //Cover both case values
                 "Z" | "z" =>{
                     force_val = self.force.2;
                }
                _ => {bail!("Not implemented for axis {} yet", self.force_axis)}
            }
            //Return the error (not absed because we want to know if we are over or under)
            Ok(self.force_target - force_val)
        }else{
            bail!("Not in force mode! Force error meaningless");
        }
    }




}

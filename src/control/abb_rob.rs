///A set of functions used to communicate and control the ABB6400 robot
use crate::config::Config;
use crate::control::egm_control::abb_egm::{EgmRobot, EgmSensor};
use crate::control::egm_control::egm_udp::EgmServer;
use crate::control::force_control::force_control::{PHPIDController, PIDController};
use crate::control::misc_tools::angle_tools::Quaternion;
use crate::control::misc_tools::misc::wait_for_enter;
use crate::control::misc_tools::string_tools;
use crate::control::trajectory_planner;
use crate::control::trajectory_planner::calc_xy_timing;
use crate::mapping::terr_map_sense::{self};
use crate::mapping::terr_map_tools::Heightmap;
use crate::modelling::experiment_model::ExpModel;
use crate::networking::tcp_sock;
use anyhow::bail;
use nalgebra::Matrix4;
use std::fs::OpenOptions;
use std::io::{prelude::*, stdin};
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::thread::sleep;
use std::time::{Duration, SystemTime};
use std::{fs, thread};

///The robot controller and state tracker
pub struct AbbRob<'a> {
    ///The TCP socket that the controller is connected to
    socket: tcp_sock::TcpSock,
    ///Indicates if the robot is simulated on the local device
    local: bool,
    ///The current xyz position of the TCP
    pos: (f64, f64, f64),
    ///The current wxyz quaternion orientation of the TCP
    ori: (f64, f64, f64, f64),
    ///The current joint angles
    jnt_angles: (f64, f64, f64, f64, f64, f64),
    ///The currently measured force information
    force: (f64, f64, f64, f64, f64, f64),
    ///State value that indicates if the robot has disconnected
    disconnected: bool,
    ///State value that indicates if the robot is being controlled via force
    force_mode_flag: bool,
    ///The target axis for the force control
    force_axis: String,
    ///The target force
    force_target: f64,
    ///The current force error (current force - target force)
    force_err: f64,
    ///Programme setup config
    config: &'a mut Config,
}

///Contains all the relevant test data for when a test starts
struct TestData {
    ///The trajectory of the test
    traj: Vec<(f64, f64, f64)>,
    ///The name of the test
    test_name: String,
    ///The filepath to store all test information
    filepath: String,
    ///The file containing the test measured data
    data_filename: String,
    ///The file contianing the test configuration information
    config_filename: String,
}
impl TestData {
    ///Create a test data structure
    fn create_test_data(config_fp: String, forcemode: bool) -> TestData {
        let traj = Self::pick_trajectory(forcemode).unwrap();

        let test_name = Self::get_test_name();

        let filepath = format!("{}/{}", config_fp, test_name.clone());
        let data_filename = format!("{}/data_{}.txt", filepath, test_name.clone());
        let config_filename = format!("{}/conf_{}.txt", filepath, test_name.clone());

        //Create the test data structure
        let t_data = TestData {
            traj,
            test_name: test_name.clone(),
            filepath,
            data_filename,
            config_filename,
        };

        //Create all the directories that the test needs
        fs::create_dir(t_data.filepath.clone()).expect("FAILED TO CREATE NEW DIRECTORY");

        t_data
    }

    ///Lets the user pick a desired trajectory from a set of predetermined trajectories (or an earlier custom made one)
    fn pick_trajectory(forcemode: bool) -> Result<Vec<(f64, f64, f64)>, anyhow::Error> {
        let mut traj;
        //Loop until command given
        loop {
            println!("Specify the trajectory you wish to run");

            //Get user input
            let mut user_inp = String::new();
            stdin()
                .read_line(&mut user_inp)
                .expect("Failed to read line");

            //Check user inout
            let user_inp = user_inp.to_lowercase();
            let user_inp = user_inp.trim();

            if forcemode {
                traj = trajectory_planner::relative_traj_gen(user_inp);
            } else {
                traj = trajectory_planner::traj_gen(user_inp);
            }

            if traj.is_ok() {
                return traj;
            } else {
                println!("Invalid trajectory!");
                continue;
            }
        }
    }

    ///Allows the user to determine the name of the test
    fn get_test_name() -> String {
        //Create the filename
        println!("Please provide a test name");

        //Get user input
        let mut user_inp = String::new();
        stdin()
            .read_line(&mut user_inp)
            .expect("Failed to read line");

        //Check if invalid filename characters in string
        let invalid_chars = ["!", "?", ".", " "];

        for char in invalid_chars {
            if user_inp.contains(char) {
                println!("illegal character in input");
                return String::from("default");
            }
        }

        user_inp.trim().to_string()
    }

    ///Create a text file that contains the desired trajectory for the test
    ///Useful for comparing with performed trajectories generated via speed control
    fn store_desired_trajectory(&mut self, forcemode: bool) {
        //Store the desired trajectory in the filepath
        let traj_fp = format!("{}/des_traj_{}.txt", self.filepath, self.test_name);
        //Store a copy of the desired trajectory
        //Create the config file and save the config info
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(traj_fp)
            .unwrap();

        //If the trajectory is a relative one
        if forcemode {
            let mut point: (f64, f64, f64) = (0.0, 0.0, 0.0);

            for (i, pnt) in self.traj.iter_mut().enumerate() {
                if i == 0 {
                    point = *pnt;
                } else {
                    point.0 += pnt.0;
                    point.1 += pnt.1;
                    point.2 += pnt.2;
                }

                let line = format!("{:?}", point);
                writeln!(file, "{}", line).expect("FAILED TO WRITE TRAJ - CLOSING");
            }
        } else {
            for pnt in self.traj.iter() {
                let line = format!("{:?}", pnt);
                writeln!(file, "{}", line).expect("FAILED TO WRITE TRAJ - CLOSING");
            }
        }
    }
}

///A list of implemented user commands
pub const IMPL_COMMDS: [&str; 10] = [
    "info",
    "cmds",
    "disconnect",
    "trajectory",
    "force traj",
    "geo test",
    "test",
    "home",
    "req xyz",
    "req ori",
];

///Determines whether to pretransform data before being saved
const TRANSFORM_TO_WORK_SPACE: bool = false;

impl AbbRob<'_> {
    ///Connect to the ABB robot controller
    pub fn create_rob(
        ip: String,
        port: u32,
        config: &'_ mut Config,
    ) -> Result<AbbRob<'_>, anyhow::Error> {
        let mut local = false;

        if ip == "127.0.0.1" {
            local = true;
        }

        //Create the robots socket
        let mut rob_sock = tcp_sock::create_sock(ip, port);
        //Attempt to connect to the robot
        if !rob_sock.connect() {
            //Failed to connect
            bail!("Robot not connected")
        } else {
            let new_rob = AbbRob {
                socket: rob_sock,
                local,
                pos: (f64::NAN, f64::NAN, f64::NAN),
                ori: (f64::NAN, f64::NAN, f64::NAN, f64::NAN),
                jnt_angles: (f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN),
                force: (f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN),
                disconnected: false,
                force_mode_flag: false,
                force_axis: "Z".to_string(),
                force_target: 0.0,
                force_err: 0.0,
                config,
            };

            Ok(new_rob)
        }
    }

    ///Disconnect from the robot controller
    pub fn disconnect_rob(&mut self) {
        self.socket.req("CLOS").expect("FAILED TO CLOSE SOCKET");
        self.socket.disconnect();
        println!("Disconnected... Moving back to core command handler");
    }

    ///A user command line to control the robot
    pub fn rob_cmd_handler(&mut self) {
        //Loop until valid command given
        loop {
            //Get user input
            let mut user_inp = String::new();
            stdin()
                .read_line(&mut user_inp)
                .expect("Failed to read line");

            //Check user inout
            match user_inp.to_lowercase().trim() {
                "info" => {
                    println!(
                        "Robot controller connected to - {}",
                        self.req_model().unwrap()
                    );
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

                //Update and print the robots TCP position
                "req xyz" => {
                    self.req_xyz();
                    println!("X:{}, Y:{}, Z:{}", self.pos.0, self.pos.1, self.pos.2);
                }

                //Update the robots orientatoin
                "req ori" => {
                    self.req_ori();
                }

                //Start a "geo test" - force controlled test
                "geo test" => {
                    self.force_mode_flag = true;

                    println!("Please type the target force");

                    let mut user_inp = String::new();
                    stdin()
                        .read_line(&mut user_inp)
                        .expect("Failed to read line");

                    if let Ok(targ) = user_inp.trim().parse::<f64>() {
                        self.force_err = targ;
                    } else {
                        println!("Invalid force target... returning to cmd line");
                        return;
                    }

                    self.geo_test_regime();
                }

                "dumbtraj" => {
                    self.dumb_trajectory();
                }

                //Placeholder for when testing new functions
                "test" => {
                    //CURRENTLY TESTING - feature size depth measurements
                    //Setup and connect EGM
                    let egm_client = self.connect_egm_pose().expect("Failed to connect to EGM");

                    if self.start_egm_stream_speed().is_err() {
                        println!("Failed to start the egm stream")
                    }

                    //Virtual recieves 518 bytes
                    //Remote recieves 469 bytes why less?

                    println!(
                        "{:?}",
                        egm_client
                            .recv_and_connect()
                            .expect("Failed to return connection")
                    );

                    //Set desired z-speed
                    let desired_speed = [0.0, 0.0, 1.0];

                    let mut seqno = 0;

                    //Move down until target z-force reached
                    for _ in 0..100 {
                        let recv_msg = egm_client.recv_egm().unwrap();
                        let time = recv_msg.get_time().unwrap();

                        //Log the robot information gathered by the EGM using
                        let _ = self.egm_update_state(recv_msg);
                        //println!("{:?}", self.pos);

                        if self.limit_check() {
                            println!("Out of bounds");
                            egm_client.egm_end();
                            self.go_home_pos();
                            return;
                        }

                        //Update the robot EGM requirements
                        egm_client
                            .send_egm(EgmSensor::set_pose_set_speed(
                                seqno,
                                time,
                                [0.0, 0.0, 0.0],
                                self.ori.into(),
                                desired_speed,
                            ))
                            .unwrap();

                        seqno += 1;
                    }

                    println!("EGM DONE?");
                }

                //Send the robot to the pre-defined home position above the sand bed
                "home" => {
                    self.go_home_pos();
                }

                "camscan" => {
                    let _ = self.feature_scan();
                }

                _ => println!("Unknown command - see CMDs for list of commands"),
            }
        }
    }

    ///Ping the robot to check that the connection is valid
    pub fn ping(&mut self) {
        let _resp = self.socket.req("ECHO:PING");

        //println!("Ping recieved - {}", s.unwrap());
    }

    ///Request the robot move to specific joint angles
    fn set_joints(&mut self, angs: (f64, f64, f64, f64, f64, f64)) {
        //Check to see if a response was returned
        if let Ok(_resp) = self.socket.req(&format!(
            "STJT:[{},{},{},{},{},{}]",
            angs.0, angs.1, angs.2, angs.3, angs.4, angs.5
        )) {
            //Update the robot info
            //self.update_rob_info();
        } else {
            //Warn the user that the robot didn't respond
            println!("Warning no response! Robot may not have moved");
        }
    }

    ///Move the robot (quickly) to the predefined home position above the sandbed
    fn go_home_pos(&mut self) {
        self.set_speed(150.0);

        self.socket
            .req("HOME:0")
            .expect("Failed to send robot home - panicking!");

        self.set_speed(50.0);

        println!("Home!");
    }

    ///Set the orientation of the robots tcp - error checking done by robot controller
    fn set_ori(&mut self, q: &Quaternion) {
        if let Ok(_resp) = self
            .socket
            .req(&format!("STOR:[{}, {}, {}, {}]", q.w, q.x, q.y, q.z))
        {
            println!("Response!");
            //self.update_rob_info();
        } else {
            println!("Warning - no robot response! - Robot might not reorient!");
        }
    }

    ///Set the desired speed of the robot TCP
    fn set_speed(&mut self, speed: f64) {
        if let Ok(resp) = self.socket.req(&format!("STSP:{}", speed)) {
            //Do nothing - no user notification required
            println!("{resp}");
        } else {
            //Warn the user the speed might not have changed
            println!("Danger! No response recieved - unknown robot speed");
        }
    }

    ///Move the tool relative to its own local coordinate system
    ///xyz - how far the tcp will move in each caridnal direction
    fn move_tool(&mut self, xyz: (f64, f64, f64)) {
        if let Ok(_resp) = self
            .socket
            .req(&format!("MVTL:[{},{},{}]", xyz.0, xyz.1, xyz.2))
        {
            self.update_rob_info();
        } else {
            println!("Warning - repsonse not recieved! - Robot may not move")
        }
    }

    ///Set the TCP point within the global coordinate system
    ///xyz - desired position in cartesian coordinates
    fn set_pos(&mut self, xyz: (f64, f64, f64)) {
        if let Ok(_resp) = self
            .socket
            .req(&format!("MVTO:[{},{},{}]", xyz.0, xyz.1, xyz.2))
        {
            println!("Response!");
            //self.update_rob_info();
        } else {
            println!("Warning - no response - robot may not move");
        }
    }

    ///Allow the user to swap the currently held tool by moving to a predefined position
    fn swap_tool(&mut self) -> Result<(), anyhow::Error> {
        //First move to home to ensure an unobstructed movement
        self.go_home_pos();

        //DO NOT CHANGE WITHOUT MANUAL CONFIRMATION
        const TOOL_CHANGE_POS: (f64, f64, f64) = (0.0, 0.0, 0.0);

        self.set_pos(TOOL_CHANGE_POS);

        //Wait for user confirmation that tool is swapped
        println!("Please swap tool and press enter");
        wait_for_enter();

        //Move the robot back to home
        self.go_home_pos();

        Ok(())
    }

    ///A trajectory run that stores no information other than the desired trajectory
    fn dumb_trajectory(&mut self) {
        //Create the test data and the filepaths
        let mut test_data = TestData::create_test_data(self.config.test_fp(), self.force_mode_flag);
        //Store the desired trajectory
        test_data.store_desired_trajectory(self.force_mode_flag);

        //Star tin the home position
        self.go_home_pos();

        //For each trajectory point
        for pnt in test_data.traj {
            //Tell the robot to move to the trajectory points
            self.set_pos(pnt);
        }

        //Go back home and announce completion
        self.go_home_pos();

        println!("Test complete");
    }

    ///A geo test that consists of three phases and aims to impart a desired force in the sand
    fn geo_test_regime(&mut self) {
        if !self.force_mode_flag {
            println!("Force mode not set! Returning!");
            return;
        }

        //Create the test data and the filepaths
        let mut test_data = TestData::create_test_data(self.config.test_fp(), self.force_mode_flag);

        //Create copys of the config for the threads
        let fp_copy = test_data.filepath.clone();
        let test_name_copy = test_data.test_name.clone();

        //Store the desired trajectory
        test_data.store_desired_trajectory(self.force_mode_flag);

        //Calcualte the speed intructions
        let desired_lat_speed = 0.1;
        let speed_instructions = calc_xy_timing(&mut test_data.traj, desired_lat_speed);

        //Create the threading channels to trigger the camera
        let (tx, rx) = mpsc::channel();

        //Clone the cam configs to avoid handing ownership to the thread
        let tmat_0 = self.config.cam_infor.tmat();
        let scale_0 = self.config.cam_infor.x_scale();

        //Create the thread that handles the depth camera

        thread::spawn(move || {
            Self::depth_sensing(rx, &*fp_copy, &*test_name_copy, true, tmat_0, scale_0, 0)
        });

        //Create the threading channels to trigger the camera
        let (tx1, rx1) = mpsc::channel();

        //Clone the cam configs to avoid handing ownership to the thread
        let tmat_1 = self.config.cam_infol.tmat();
        let scale_1 = self.config.cam_infol.x_scale();

        //Create the thread that handles the second depth camera
        thread::spawn(move || {
            Self::depth_sensing(
                rx1,
                &*test_data.filepath.clone(),
                &test_data.test_name.clone(),
                true,
                tmat_1,
                scale_1,
                1,
            )
        });

        //Setup the seperate PID controllers
        let mut phase2_cntrl =
            PHPIDController::create_PHPID(0.001, 0.000, 0.00002, 0.0, 0.001, 0.0001, 0.00001);
        let mut phase3_cntrl = PIDController::create_PID(0.00005, 0.000005, 0.000);

        //Setup the config information
        self.config.set_phase2_cntrl(phase2_cntrl.to_string());
        self.config.set_phase3_cntrl(phase3_cntrl.to_string());

        //Log the config
        self.log_config(&test_data.config_filename);

        //Give the camera turn on process time to warm up
        sleep(Duration::from_secs(2));
        if tx.send(4).is_err() || tx1.send(4).is_err() {
            println!("Failed dummy cam trigger");
        }

        //Move the robot out of the way of the camera
        self.go_home_pos();

        //Trigger before the test begins
        if tx.send(2).is_err() || tx1.send(2).is_err() {
            println!("Failed initial cam trigger");
        }

        let start_pos = (
            test_data.traj[0].0,
            test_data.traj[0].1,
            test_data.traj[0].2,
        );

        println!("START height: {}", test_data.traj[0].2);

        self.write_marker(&test_data.data_filename, "TEST STARTED");

        //Move to the starting point
        self.set_pos(start_pos);

        self.set_speed(2.0);

        //Read the values once
        self.update_rob_info();

        let mut cnt = 0;

        //SETUP COMPLETE-----------------------

        //Phase 1 - position control until target force reached

        self.write_marker(&test_data.data_filename, "PHASE 1 STARTED");

        //Find the vert force------------------------------

        //Setup and connect EGM
        let egm_client = self.connect_egm_pose().expect("Failed to connect to EGM");

        if self.start_egm_stream_speed().is_err() {
            println!("Failed to start the egm stream")
        } else {
            println!("EGM stream started");
        };
        egm_client
            .recv_and_connect()
            .expect("Failed to return connection");

        //Set desired z-speed
        let mut desired_speed = [0.0, 0.0, -5.0];

        let mut seqno = 0;

        //Move down until target z-force reached
        while self.force.2 > self.force_target {
            let recv_msg = egm_client.recv_egm().unwrap();
            let time = recv_msg.get_time().unwrap();

            //Log the robot information gathered by the EGM using
            let _ = self.egm_update_state(recv_msg);
            self.store_state(&test_data.data_filename, cnt);

            if self.limit_check() {
                println!("Out of bounds");
                egm_client.egm_end();
                self.go_home_pos();
                self.write_marker(&test_data.data_filename, "TEST OUT OF  SAFETY BOUNDS");
                return;
            }

            //Update the robot EGM requirements
            egm_client
                .send_egm(EgmSensor::set_pose_set_speed(
                    seqno,
                    time,
                    [0.0, 0.0, 0.0],
                    self.ori.into(),
                    desired_speed,
                ))
                .unwrap();

            seqno += 1;
            cnt += 1;
        }

        //Take snapshot
        let _ = tx.send(1);
        let _ = tx1.send(1);

        println!("GEOTECH- Phase 1 Complete!");
        self.write_marker(&test_data.data_filename, "PHASE 1 END");

        //Phase 2 - force control until target force is stabilised (PID 1)

        let phase2_force_scaler = 2.0;
        self.force_target /= phase2_force_scaler;
        let mut force_stable = false;
        let mut force_errs: Vec<f64> = vec![];
        //Minimum of 500 measurements taken - just to prove its stable
        const FORCE_ERR_ROLL_AVG: usize = 300;
        let force_avg_threshold: f64 = match self.force_target {
            -5.0 => 0.45,
            -10.0 => 0.30,
            -25.0 => 0.20,
            _ => 0.10,
        };

        //Actual values have to be within 30% of the desired force
        const FORCE_THRESH_CNT: usize = 100;
        //The force threshold is based on the inverse of the magnitude
        let force_threshold: f64 = match self.force_target {
            -5.0 => 1.0,
            -10.0 => 0.6,
            -25.0 => 0.5,
            -50.0 => 0.25,
            _ => 0.10,
        };

        self.write_marker(&test_data.data_filename, "PHASE 2 STARTED");
        while !force_stable {
            let recv_msg = egm_client.recv_egm().unwrap();
            let time = recv_msg.get_time().unwrap();

            //Log the robot information gathered by the EGM using
            let _ = self.egm_update_state(recv_msg);
            self.store_state(&test_data.data_filename, cnt);

            if self.limit_check() {
                println!("Out of bounds");
                egm_client.egm_end();
                self.go_home_pos();
                self.write_marker(&test_data.data_filename, "TEST OUT OF  SAFETY BOUNDS");
                return;
            }

            //Apply the controller
            let des_z_speed = phase2_cntrl
                .calc_op(self.force_err)
                .expect("Failed to calculate desired z speed");

            desired_speed = [0.0, 0.0, des_z_speed];

            //Update the robot EGM requirements
            egm_client
                .send_egm(EgmSensor::set_pose_set_speed(
                    seqno,
                    time,
                    [0.0, 0.0, 0.0],
                    self.ori.into(),
                    desired_speed,
                ))
                .unwrap();

            seqno += 1;
            cnt += 1;

            //Calc the force error and add to rolling average list
            if force_errs.len() < FORCE_ERR_ROLL_AVG {
                force_errs.push(self.force_err);
            } else {
                force_errs.remove(0);
                force_errs.push(self.force_err);
            }

            //Check if the rolling average is within an acceptable range
            //Check if every value in the rolling queue is within the required threshold
            if force_errs.len() == FORCE_ERR_ROLL_AVG {
                let mut force_sum = 0.0;
                let force_avg: f64;

                let mut all_within = true;
                //Check that the rolling average is within 10% (globally correct)
                for (i, force) in force_errs.iter().enumerate() {
                    force_sum += force;

                    //Check that the actual previous values are within 30% (locally correct)
                    //Higher threshold to account for noise
                    if i >= FORCE_THRESH_CNT {
                        let curr_val = force / self.force_target;
                        if curr_val.abs() > force_threshold {
                            //println!("Failed at: {curr_val}");
                            all_within = false;
                            break;
                        }
                    }
                }

                if all_within {
                    //Check that the global average is okay
                    force_avg = force_sum / FORCE_ERR_ROLL_AVG as f64;
                    if (force_avg / self.force_target).abs() < force_avg_threshold {
                        //Count the force as stable
                        force_stable = true;
                    } else {
                        println!("GLOBAL AVG INCORRECT: {force_avg}")
                    }
                }
            }
        }

        println!("GEOTECH - PHASE 2 COMPLETE!");
        self.write_marker(&test_data.data_filename, "PHASE 2 ENDED");

        //Reset the force target to the desired
        self.force_target *= phase2_force_scaler;

        //Take snapshot
        let _ = tx.send(1);
        let _ = tx1.send(1);
        //Phase 3 - Complete trajectory whilst (PID)

        //Set the speed of the robot
        self.set_speed(5.0);

        //Start the trajectory
        self.write_marker(&test_data.data_filename, "PHASE 3 STARTED");

        const DEPTH_FREQ: i32 = 250;

        for instruction in speed_instructions.iter() {
            //Get the time limit
            let time_lim = Duration::from_secs_f64(instruction.0);

            //Start the timer
            let start_time = SystemTime::now();

            //Send the speed instruction to the robot via EGM
            let mut desired_speed: [f64; 3];

            //While the timer is running
            while start_time.elapsed().unwrap() < time_lim {
                //Get the egm message
                let msg = egm_client.recv_egm().expect("Failed to get egm message");

                let time = msg.get_time().expect("Failed to get egm time");

                //Log the robot information gathered by the EGM using
                let _ = self.egm_update_state(msg);
                self.store_state(&test_data.data_filename, cnt);

                if self.limit_check() {
                    println!("Out of bounds");
                    egm_client.egm_end();
                    self.go_home_pos();
                    self.write_marker(&test_data.data_filename, "TEST OUT OF  SAFETY BOUNDS");
                    return;
                }

                //Trigger the camera
                if cnt % DEPTH_FREQ == 0 || cnt == 0 {
                    if tx.send(1).is_ok() || tx1.send(1).is_ok() {
                        //Do nothing here - normal operation
                    } else {
                        println!("Warning - Cam thread dead!");
                    }
                }

                //Apply the controller
                let des_z_speed = phase3_cntrl
                    .calc_op(self.force_err)
                    .expect("Failed to calculate desired z speed");

                //Send the EGM control
                desired_speed = [instruction.1.0, instruction.1.1, des_z_speed];
                let sensor: EgmSensor = EgmSensor::set_pose_set_speed(
                    seqno,
                    time,
                    [0.0, 0.0, 0.0],
                    self.ori.into(),
                    desired_speed,
                );
                egm_client
                    .send_egm(sensor)
                    .expect("Failed to send sensor info");
                seqno += 1;
                cnt += 1;
            }
        }

        //End the EGM client
        egm_client.egm_end();

        self.write_marker(&test_data.data_filename, "PHASE 3 ENDED");

        //Go back to home pos
        self.go_home_pos();
        //No point error handling - if this fails the test is done anyway
        let _ = tx.send(3);
        let _ = tx1.send(3);

        println!("Trajectory done!");

        let _ = tx.send(0);

        self.write_marker(&test_data.data_filename, "TEST END");
    }

    ///A test regime for whether we can control the end effector linear speeds without EGM
    fn end_eff_speed_set_test(&mut self) {
        //Set up the test data
        //Creates the test data and the filepaths
        let mut test_data = TestData::create_test_data(self.config.test_fp(), self.force_mode_flag);
        //Store the desired trajectory
        test_data.store_desired_trajectory(self.force_mode_flag);

        //Log the config
        self.log_config(&test_data.config_filename);

        //Go to the start position
        self.set_pos(test_data.traj[0]);

        //Create the experiment model
        let mut exp_model = ExpModel::create_exp_model(test_data.traj, 1.0).unwrap();

        let (trig_tx, trig_rx): (mpsc::Sender<bool>, Receiver<bool>) = mpsc::channel();
        let (jnt_send, jnt_recv): (mpsc::Sender<[f64; 6]>, Receiver<[f64; 6]>) = mpsc::channel();
        let (z_pub, z_recv): (mpsc::Sender<f64>, Receiver<f64>) = mpsc::channel();

        //Setup the experiment model test
        self.req_jnt_angs();

        let curr_jnt = self.jnt_angles;

        thread::spawn(move || {
            exp_model.run_model_traj(<[f64; 6]>::from(curr_jnt), trig_rx, jnt_send, z_recv)
        });

        let mut cnt = 0;

        //log the info
        self.update_rob_info();
        self.store_state(&test_data.data_filename.clone(), cnt);
        cnt += 1;

        //Trigger the experiment model to start
        trig_tx.send(true).unwrap();

        let _ = z_pub.send(0.0);

        while let Ok(jnt_angles) = jnt_recv.recv() {
            //Get the joint angles and send to robot
            self.set_joints(<(f64, f64, f64, f64, f64, f64)>::from(jnt_angles));

            //Get all info from robot
            self.update_rob_info();
            self.store_state(&test_data.data_filename.clone(), cnt);

            //Send the requested z heigt (0 for this)
            let _ = z_pub.send(-1.0);

            cnt += 1;
        }

        //Go to the home position
        self.go_home_pos();
    }

    ///Function which repeatedly takes depth measurements on trigger
    ///Designed to be used in a parallel thread
    fn depth_sensing(
        rx: Receiver<u32>,
        filepath: &str,
        test_name: &str,
        hmap: bool,
        tmat: Matrix4<f64>,
        scale: f64,
        cam_no: usize,
    ) {
        //Create a camera
        let mut cam =
            terr_map_sense::RealsenseCam::initialise_raw(cam_no).expect("Failed to create camera");

        let mut cnt = 0;

        //Loop forever - will be killed once the test ends automatically
        loop {
            let opt = rx.recv().expect("recieve thread error");

            //Block until the trigger is recieved
            if opt > 0 {
                println!("Taking depth measure");

                //Create a pointcloud
                let mut curr_pcl = cam.get_depth_pnts().expect("Failed to get get pointcloud");

                //Scale/rotate/transform the depth data so it is useable
                curr_pcl.scale_even(scale);
                curr_pcl.transform_with(&tmat);

                //Pass band filter the transformed data to keep interest in the box only.
                curr_pcl.passband_filter(-10.0, 2000.0, -10.0, 2000.0, -150.0, 200.0);

                //Save the pointcloud
                let pcl_filepath: String;

                match opt {
                    1 => pcl_filepath = format!("{filepath}/pcl_C{cam_no}_{test_name}_{cnt}"),
                    2 => pcl_filepath = format!("{filepath}/pcl_C{cam_no}_{test_name}_START"),
                    3 => pcl_filepath = format!("{filepath}/pcl_C{cam_no}_{test_name}_END"),

                    //Sacrificial scan (i.e. to get a crappy one out the way
                    4 => {
                        println!("THROWAWAY SCAN");
                        //Sleep to try let the cam warm
                        sleep(Duration::from_secs(2));
                        continue;
                    }

                    //If invalid number just take a count
                    _ => pcl_filepath = format!("{filepath}/pcl_C{cam_no}_{test_name}_{cnt}"),
                }

                curr_pcl.save_to_file(&pcl_filepath).unwrap();

                if hmap {
                    //Create a heightmap from the pointcloud
                    let mut curr_hmap = Heightmap::create_from_pcl(curr_pcl, 200, 200);

                    //Save the heightmap
                    let hmap_filepath: String = match opt {
                        1 => {
                            format!("{filepath}/hmap_C{cam_no}_{test_name}_{cnt}")
                        }
                        2 => {
                            format!("{filepath}/hmap_C{cam_no}_{test_name}_START")
                        }
                        3 => {
                            format!("{filepath}/hmap_C{cam_no}_{test_name}_END")
                        }

                        //If invalid number just take a count
                        _ => {
                            format!("{filepath}/hmap_C{cam_no}_{test_name}_{cnt}")
                        }
                    };

                    //println!("{hmap_filepath}");
                    curr_hmap.save_to_file(&hmap_filepath).unwrap()
                }

                if opt == 1 {
                    //Increase the loop count
                    cnt += 1;
                }
            } else {
                println!("Closing cam thread");
                return;
            }
        }
    }

    ///Requests the xyz position of the TCP from the robot and stores it in the robot info
    fn req_xyz(&mut self) {
        //Request the info
        if let Ok(recv) = self.socket.req("GTPS:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&recv);
            let xyz_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if xyz_vec.len() != 3 {
                println!("XYZ pos read error!");
                self.pos = (f64::NAN, f64::NAN, f64::NAN);
            } else {
                //Store the pos in the robot info
                self.pos = (xyz_vec[0], xyz_vec[1], xyz_vec[2]);

                //println!("CURR POS: {:?}", self.pos);
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
        }
    }

    ///Requests the orientation information
    fn req_ori(&mut self) {
        //Request the info
        if let Ok(recv) = self.socket.req("GTOR:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&recv);

            let ori_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if ori_vec.len() != 4 {
                println!("ORI pos read error!");
                self.ori = (f64::NAN, f64::NAN, f64::NAN, f64::NAN);
            } else {
                //Store the pos in the robot info
                self.ori = (ori_vec[0], ori_vec[1], ori_vec[2], ori_vec[3]);
            }
        } else {
            //If the socket request returns nothing
            println!("WARNING ROBOT DISCONNECTED");
            self.disconnected = true;
        }
    }

    ///Requests joint angle information
    fn req_jnt_angs(&mut self) {
        //Request the info
        if let Ok(recv) = self.socket.req("GTJA:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&recv);
            let jtang_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if jtang_vec.len() != 6 {
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", jtang_vec.len());
                self.jnt_angles = (f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN);
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
        }
    }

    ///Requests 6-axis force information
    fn req_force(&mut self) {
        //Request the info
        if let Ok(recv) = self.socket.req("GTFC:0") {
            //Format the string
            let recv = string_tools::rem_first_and_last(&recv);
            let fc_vec = string_tools::str_to_vector(recv);

            //Check that the vector is the right length
            if fc_vec.len() != 6 {
                println!("joint angle read error!");
                println!("Expected: 6. Actual: {}", fc_vec.len());
                self.force = (f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN);
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
        }
    }

    ///Requests the model name of the robot
    fn req_model(&mut self) -> Result<String, anyhow::Error> {
        //Request the model name
        if let Ok(model) = self.socket.req("RMDL:0") {
            Ok(model)
        } else {
            bail!("Unable to identify model]")
        }
    }

    ///Helper function that requests all the relevant information from the robot
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

        if self.disconnected {
            return;
        }
    }

    ///Saves the robot state (i.e. the test data) in a given file
    fn store_state(&mut self, filename: &str, i: i32) {
        //Open the file (or create if it doesn't exist)
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(filename.trim())
            .unwrap();

        //See whether to transofmr the data by the
        let line: String =
            //Format the line to write
            format!(
                "{},{:?},[{},{},{}],[{},{},{},{}],[{},{},{},{},{},{}],{}",
                i,
                SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_secs_f64(),
                //Make sure that you don't print a lack of information in the data
                self.pos.0,
                self.pos.1,
                self.pos.2,
                self.ori.0,
                self.ori.1,
                self.ori.2,
                self.ori.3,
                self.force.0,
                self.force.1,
                self.force.2,
                self.force.3,
                self.force.4,
                self.force.5,
                self.force_err
            );

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line) {
            eprint!("Couldn't write to file: {}", e);
        }
    }

    ///Write a marker to the given file with a timestamp (used for marking certain milestones in tests)
    fn write_marker(&mut self, filename: &str, comment: &str) {
        //Open the file (or create if it doesn't exist)
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(filename.trim())
            .unwrap();

        let line = format!(
            "!{:?}: {}",
            SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
            comment
        );

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line) {
            eprint!("Couldn't write to file: {}", e);
        }
    }

    ///Save the test configuration information
    fn log_config(&mut self, filepath: &str) {
        //println!("{filepath}");

        //Create the config file and save the config info
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(filepath.trim())
            .unwrap();

        //Save the cam info
        let line = format!(
            "CAMR: EXT_MAT:[{:?}] X_SC:[{}] Y_SC:[{}]",
            self.config.cam_infor.tmat(),
            self.config.cam_infor.x_scale(),
            self.config.cam_infor.y_scale()
        );

        writeln!(file, "{}", line).expect("FAILED TO WRITE CAM TO CONFIG - CLOSING");

        let line = format!(
            "CAML: EXT_MAT:[{:?}] X_SC:[{}] Y_SC:[{}]",
            self.config.cam_infol.tmat(),
            self.config.cam_infol.x_scale(),
            self.config.cam_infol.y_scale()
        );

        writeln!(file, "{}", line).expect("FAILED TO WRITE CAM TO CONFIG - CLOSING");

        //Save another line with the robot pos/ori config data
        let line = format!(
            "ROB: NAME: \"{}\" POS:[{},{},{}] ORI:[{},{},{}] EMB:[{}]",
            self.config.rob_info.rob_name(),
            self.config.rob_info.pos_to_zero()[0],
            self.config.rob_info.pos_to_zero()[1],
            self.config.rob_info.pos_to_zero()[2],
            self.config.rob_info.ori_to_zero()[0],
            self.config.rob_info.ori_to_zero()[1],
            self.config.rob_info.ori_to_zero()[2],
            self.config.rob_info.min_embed_height()
        );

        writeln!(file, "{}", line).expect("FAILED TO WRITE ROB TO CONFIG - CLOSING");

        //Save a line indicating if the data is pre-transformed
        let line = format!("COORDS PRE-TRANSFORMED?:{}", TRANSFORM_TO_WORK_SPACE);

        writeln!(file, "{}", line).expect("FAILED TO WRITE TRANSFORM FLAG TO CONFIG - CLOSING");

        let line = format!(
            "FC_MODE:{} FC_AXIS:{}, FC_TARGET:{}",
            self.force_mode_flag, self.force_axis, self.force_target
        );
        writeln!(file, "{}", line).expect("FAILED TO WRITE FORCE CONTROL CONFIG - CLOSING");

        //If the force mode is high - store the controller configs
        if self.force_mode_flag {
            let line = format!("PHASE2 CONTROLLER: {}", self.config.phase2_cntrl_settings);
            writeln!(file, "{}", line)
                .expect("FAILED TO WRITE PHASE 2 CONTROLLER CONFIG - CLOSING");

            let line = format!("PHASE3 CONTROLLER: {}", self.config.phase3_cntrl_settings);
            writeln!(file, "{}", line)
                .expect("FAILED TO WRITE PHASE 3 CONTROLLER CONFIG - CLOSING");
        }
    }

    ///Calculate the error between the measured force and the target force
    fn calc_force_err(&mut self) -> Result<f64, anyhow::Error> {
        //Check that force mode is enabled (otherwise there's no point in calcing the error
        if self.force_mode_flag {
            let force_val: f64;
            //Extract the correct axis information
            match self.force_axis.as_str() {
                //Cover both case values
                "Z" | "z" => {
                    force_val = self.force.2;
                }
                _ => {
                    bail!("Not implemented for axis {} yet", self.force_axis)
                }
            }
            //Return the error (not absed because we want to know if we are over or under)

            self.force_err = force_val - self.force_target;

            Ok(self.force_err)
        } else {
            bail!("Not in force mode! Force error meaningless");
        }
    }

    //EGM commands---------------------------------------------------------------------------------
    ///Create a UDP EGM socket and ask the robot to connect
    fn connect_egm_pose(&mut self) -> Result<EgmServer, anyhow::Error> {
        let serv = if self.local {
            EgmServer::local()
        } else {
            EgmServer::remote()
        };

        //Request the robot connect to the UDP socket
        self.socket.req("EGPS:0")?;

        Ok(serv)
    }

    ///Start the EGM stream in speed mode
    fn start_egm_stream_speed(&mut self) -> Result<(), anyhow::Error> {
        println!("Requesting starting EGM");

        self.socket.req("EGSS:0")?;

        println!("EGM speed stream started");

        Ok(())
    }

    ///Start the EGM in pose mode
    fn start_egm_stream_pose(&mut self) -> Result<(), anyhow::Error> {
        self.socket.req("EGST:0")?;

        Ok(())
    }

    ///Stop the EGM stream
    fn stop_egm_stream(&mut self) -> Result<(), anyhow::Error> {
        self.socket.req("EGSP:0")?;

        Ok(())
    }

    ///Update the state of the robot using the EGM response from the robot
    fn egm_update_state(&mut self, msg: EgmRobot) -> Result<(), anyhow::Error> {
        //Update position
        if let Some(pos) = msg.get_pos_xyz() {
            self.pos = pos.into();
        } else {
            bail!("Failed to update state - pos");
        };

        //Update orientation
        if let Some(ori) = msg.get_quart_ori() {
            self.ori = ori.into();
        } else {
            bail!("Failed to update state - ori");
        };

        //Update current measured force
        if let Some(force) = msg.get_measured_force() {
            self.force = force.into();
        } else {
            bail!("Failed to update state - pos");
        };

        //if force mode update force error
        if self.force_mode_flag {
            self.calc_force_err()?;
        }

        Ok(())
    }

    ///Checks whether the robot is within the specified allowed cartesian limits
    fn limit_check(&mut self) -> bool {
        let min_x = -425.0;
        let min_y = 1350.0;
        let min_z = 95.0;
        let max_x = 650.0;
        let max_y = 2650.0;
        let max_z = 2000.0;

        if self.pos.0 > max_x || self.pos.0 < min_x {
            return true;
        }
        if self.pos.1 > max_y || self.pos.1 < min_y {
            return true;
        }
        if self.pos.2 > max_z || self.pos.2 < min_z {
            return true;
        }

        false
    }

    ///The load cell verification script
    /// Robot moves to three seperate poses and the force is measured for 1000 ticks and stored
    fn verify_load_cell(&mut self) {
        //The two positions
        let rotate_joints = (81.0, 12.2, 34.0, 0.0, 54.80, -118.38);
        let tool_change_joints = (21.15, 42.0, 59.0, 0.0, -57.07, -114.51);

        //The four orientations
        let ori_zero = Quaternion::from([0.0, 1.0, 0.0, 0.0]);
        let ori_one = Quaternion::from([0.35218, -0.39616, -0.59986, -0.59933]);
        let ori_two = Quaternion::from([0.187, -0.15665, 0.69299, 0.67843]);
        let ori_three = Quaternion::from([0.16195, 0.43884, 0.83970, 0.27583]);

        let oris = [ori_zero, ori_one, ori_two, ori_three];

        //Create the test file and the config
        let test_data = TestData::create_test_data(self.config.test_fp(), self.force_mode_flag);
        self.log_config(&test_data.config_filename);

        //Robot move to a position where its easy to attach tools etc
        println!("Moving to attach tool");
        self.set_joints(tool_change_joints);

        //Pause for user input to say tool changed
        wait_for_enter();

        //Go home to ensure clearance for movement to the rotation spot
        self.go_home_pos();

        //Pause for user input to say tool changed
        wait_for_enter();

        //Robot move to a position where it is safe to perform the rotations
        println!("Moving to rotation spot");
        self.set_joints(rotate_joints);

        self.write_marker(&test_data.data_filename, "Test start");
        //For every rotation
        let mut cnt = 0;
        for (i, ori) in oris.iter().enumerate() {
            //Move the robot to the requested orientation
            self.set_ori(ori);

            //Set the marker in the test file
            self.write_marker(&test_data.data_filename, &format!("ORIENTATION {}", i));

            //Take 1000 force measurements
            for _ in 0..1000 {
                self.update_rob_info();

                //Store the measurement
                self.store_state(&test_data.data_filename, cnt);
                cnt += 1;
            }
        }

        //Go back to the original orientation
        self.set_ori(&Quaternion::from([0.0, 1.0, 0.0, 0.0]));

        //Return home
        self.go_home_pos();

        self.write_marker(&test_data.data_filename, "Test end");
    }

    ///An experiment that moves the robot vertically in a straight line and saves pointclouds at specific heights
    fn feature_scan(&mut self) -> Result<(), anyhow::Error> {
        //Predetermined XY position
        let xy_pos = [417.0, 2115.0];

        //Predetermined heights
        let heights = [250.0, 350.0, 450.0, 550.0, 650.0, 750.0, 850.0, 950.0];

        //Number of pointclouds to take
        let number_of_pcls = 25;

        //Create the test data and the filepaths
        let test_data = TestData::create_test_data(self.config.test_fp(), self.force_mode_flag);

        //Move to cable attach position
        self.set_pos((940.0, 2139.0, 275.0));
        self.set_ori(&Quaternion::from([0.0, 0.39616, 0.91817, -0.00312]));

        println!("Please attach cable!");
        wait_for_enter();
        //Turn on the camera
        let mut cam = terr_map_sense::RealsenseCam::initialise_raw(0)?;

        println!("Cam initialised.... Moving to feature scan position");

        //Move to the first position and rotate to correct orientation
        self.set_pos((xy_pos[0], xy_pos[1], heights[0]));
        self.set_ori(&Quaternion::from([0.0, 0.39616, 0.91817, -0.00312]));

        //For each height
        for height in heights.iter() {
            //Setup the current robot config
            let _pcl_cnt = 0;
            let fp = format!(
                "{}/pcl_{}_{}mm",
                test_data.filepath, test_data.test_name, height
            );

            //Move the robot to the desired height
            self.set_pos((xy_pos[0], xy_pos[1], *height));

            //Take the pointclouds
            for i in 0..number_of_pcls {
                //Capture the depth points
                let mut curr_pcl = cam.get_depth_pnts()?;

                curr_pcl.save_to_file(&format!("{}_{}", fp, i))?;

                //Ping to keep the connection alive
                self.ping();
            }

            println!("Height completed - {}", height);
        }

        //Move to cable detach position
        self.set_pos((940.0, 2139.0, 275.0));
        self.set_ori(&Quaternion::from([0.0, 0.39616, 0.91817, -0.00312]));

        println!("Please disconnect cable!");
        wait_for_enter();

        Ok(())
    }
}

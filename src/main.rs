#![allow(dead_code)]
///rustbot control!
///A rust and headerless version of the robot controller designed to run tests in the soilbed
///Author(s) - Joe Ingham
use std::collections::HashMap;
use std::io::stdin;
mod config;
mod control;
mod networking;
mod cam_sys_cntrl;
use crate::config::Config;
use crate::control::force_control::force_function_generator::ForceFunctionGenerator;
use cam_sys_cntrl::cam_sys_cntrl::CamSysCntrl;
use std::sync::mpsc;
use rustgeomapping::data_types::heightmap::Heightmap;
use tokio::sync::watch;
use std::thread;
use std::process::{Command, Stdio};



use control::abb_rob;

const VER_NUM: &str = "V0.9";
//Program title
const TITLE: &str = "Rustbot Control";

///Main command loop
fn main() -> Result<(), anyhow::Error> {
    println!("RUSTBOT_CNTRL STARTUP....");

    //Load the program config
    let mut config: Config;
    let conf = Config::setup_config();

    match conf {
        Ok(conf) => {
            config = conf;
            println!("Set config loaded");
        }

        Err(conf) => {
            println!("Error loading config - {}", conf);
            println!("Loading default!");
            config = Config::default();
        }
    }

    //Run the command handler
    core_cmd_handler(&mut config);

    println!("Shutting down");

    Ok(())
}

///Handles commands given by the user - robot not required!
fn core_cmd_handler(config: &mut Config) {
    //Array of implemented commands
    const VALID_CMDS: [&str; 7] = [
        "info - get title and version number",
        "quit - close the program",
        "cmds - list the currently implemented commands",
        "ping - TEST - connect to and ping the robot studio",
        "connect - connect to a robot on a given ip and port (if successful unlocks robot specific commands",
        "analyse - analyse a previous tests data",
        "snsdpth - Take N heightmap measurements",
    ];

    println!("{TITLE} - {VER_NUM}");

    //Loop until command given
    loop {
        //Get user input
        let mut user_inp = String::new();
        stdin()
            .read_line(&mut user_inp)
            .expect("Failed to read line");

        //Check uesr inout
        match user_inp.to_lowercase().trim() {
            "info" => {
                println!("{TITLE} - {VER_NUM}");
                println!(
                    "This program is a headerless robot control tool - intended to remotely control 6 axis robots for the TRL - connect to a robot to gain access to more commands"
                );
            }
            //Print out the commands in the valid commands list
            "cmds" => {
                for cmd in VALID_CMDS {
                    println!("{cmd}");
                }
            }
            "quit" => break,

            "connect" => rob_connect(config),

            "test" => {
                //Currently testing - subcam system control

                let rust_filepath = "/home/joe/Documents/Data/test_dumps/hmap_stream/hmap";
                let python_filepath = "Data/test_dumps/hmap_stream/hmap";

                //Create the thread piping system
                let(pos_tx, pos_rx)  = watch::channel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

                let (hmap_tx, hmap_rx) = mpsc::channel();

                let (cntrl_tx, cntrl_rx) = watch::channel(0);

                //Spawn the cam system thread
                let cam_sys_thread = thread::spawn(|| {
                    if let Ok(mut cam_sys) = CamSysCntrl::default_connect(pos_rx, hmap_tx, cntrl_rx){

                        cam_sys.start_system().unwrap();

                        println!("Thread closed...");

                }else{
                    println!("failed");
                }});

                //Start "moving" the camera
                for i in 0..100{
                    pos_tx.send_replace([i as f32 * 0.01, i as f32 * 0.01, 0.0, 1.0, 0.0, 0.0, 0.0]);

                    let hmap = hmap_rx.recv().expect("Failed to get heightmap");

                    hmap.save_to_file(&format!("{}_{}",rust_filepath, i)).expect("Failed to write");

                    //println!("{}",&format!("{}_{}.txt {}_{}", filepath, i, filepath, i));

                    
                }

                //Close the connection
                cntrl_tx.send_replace(1);
               
                
            }


            //Catch all else
            _ => println!("Unknown command - see CMDs for list of commands"),
        }
    }
}

///Command line for logging into and controlling a robot
fn rob_connect(config: &mut Config) {
    //Available profiles available for connecting to the robot
    let profiles = HashMap::from([
        ("local", ["127.0.0.1", "8888"]),
        ("remote", ["192.168.125.1", "8888"]),
    ]);

    //User selected profile
    let profile: [&str; 2];

    loop {
        println!("Please select a profile");

        println!("Profiles available: ");

        for key in profiles.keys() {
            println!("\t {key}");
        }

        let mut choice = String::new();

        stdin().read_line(&mut choice).expect("Failed to read line");

        //Check user inout
        match profiles.get(choice.to_lowercase().trim()) {
            Some(login) => {
                //Set the profile based on the users choice
                profile = *login;
                break;
            }
            None => {
                println!("Invalid profile")
            }
        }
    }

    //Attempt to log in to the robot with the given profile
    println!("Logging into robot on : {}:{}", profile[0], profile[1]);

    //If connected - create the robot and keep it in scope to keep the connection open
    if let Ok(mut curr_rob) = abb_rob::AbbRob::create_rob(
        profile[0].parse().unwrap(),
        profile[1].parse().unwrap(),
        config,
    ) {
        println!("Connected!");

        //Open the robot command handler - must be defined for robot!
        curr_rob.rob_cmd_handler();
    } else {
        //Robot failed to connect - go up back to core cmd handler
        println!("{TITLE} - {VER_NUM}");
    }
}

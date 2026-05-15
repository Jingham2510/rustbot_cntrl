#![allow(dead_code)]
use nalgebra::Matrix4;
///rustbot control!
///A rust and headerless version of the robot controller designed to run tests in the soilbed
///Author(s) - Joe Ingham
use std::collections::HashMap;
use std::fs;
use std::fs::File;
use std::io::{Write, stdin};
use std::thread;
use std::time::{Duration, SystemTime};
mod analysis;
mod config;
mod control;
mod mapping;
mod networking;

mod helper_funcs;
mod modelling;

use crate::analysis::analyser::Analyser;
use crate::config::Config;
use crate::mapping::terr_map_tools::{
    Heightmap, PointCloud, average_heightmaps, low_pass_heightmaps,
};

use control::abb_rob;

const VER_NUM: &str = "V0.8";
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

            "analyse" => {
                if let Err(e) = analyse(&config) {
                    println!("ANALYSE ERROR - {e}")
                }
            }

            "temp" => {
                //CURRENTLY - saving faro pointclouds as parametric heightmaps
                let tests_to_gen = vec![
                    "flat_terrain_scan_1",
                    "small_indent_scan_1",
                    "medium_indent_scan_1",
                    "big_indent_scan_1",
                ];

                let averages: [u32; 25] = core::array::from_fn(|i| (i + 1) as u32);
                let resolutions: [u32; 495] = core::array::from_fn(|i| (i + 5) as u32);

                let identifiers: [&str; 6] = ["450mm", "550mm", "650mm", "750mm", "850mm", "950mm"];

                let ground_truths: [&str; 4] = [
                    "/home/joe/Documents/Data/test_dumps/faro_flat/pcl_faro_flat_processed.txt",
                    "/home/joe/Documents/Data/test_dumps/faro_flat/pcl_faro_flat_processed.txt",
                    "/home/joe/Documents/Data/test_dumps/faro_flat/pcl_faro_flat_processed.txt",
                    "/home/joe/Documents/Data/test_dumps/faro_flat/pcl_faro_flat_processed.txt",
                ];

                let height_deltas: [[f64; 6]; 4] = [
                    [0.513, 0.411, 0.312, 0.207, 0.103, 0.0],
                    [0.513, 0.410, 0.307, 0.205, 0.10, 0.0],
                    [0.512, 0.408, 0.307, 0.203, 0.103, 0.0],
                    [0.508, 0.407, 0.301, 0.202, 0.103, 0.0],
                ];

                for (i, test) in tests_to_gen.iter().enumerate() {
                    let t = thread::spawn(move || {
                        println!("Thread {} running", test);

                        let mut analyser = Analyser::init(
                            String::from("/home/joe/Documents/Data/test_dumps"),
                            String::from(test),
                        )
                        .expect("Failed to find test");

                        analyser.save_parametric_hmap_stats(
                            resolutions.to_vec(),
                            averages.to_vec(),
                            identifiers.to_vec(),
                            &PointCloud::create_from_file(String::from(ground_truths[i])).unwrap(),
                            height_delta[i],
                        );

                        println!("Thread {} complete", test);
                    });
                }
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
        return;
    }
}

///Analyse a tests gathered terrain information (usually changes regularly!)
fn analyse(config: &Config) -> Result<(), anyhow::Error> {
    //Print and number the list of tests in the DEPTH_TESTS folder (ignoring _archive)
    let depth_test_fp: String = config.test_fp();

    let paths = fs::read_dir(&depth_test_fp)?;

    //Test enumeration holder
    let mut test_enum: Vec<(i32, String)> = vec![];

    let mut test_cnt = 0;

    for path in paths {
        //Convert the path to a string
        let folder_str = path?
            .file_name()
            .into_string()
            .expect("FAILED TO CONVERT PATH TO STRING");

        //Ignore all folders starting with '_'
        if folder_str.starts_with("_") {
            continue;
        }
        test_enum.push((test_cnt, folder_str));

        test_cnt = test_cnt + 1;
    }
    //Ask the user to pick  the numbered tests they want to view
    let mut user_sel: usize;

    loop {
        //Display all the test options
        for test in test_enum.iter() {
            println!("{} - {}", test.0, test.1);
        }

        println!("Please select a test number:");
        //Get user input
        let mut user_inp = String::new();
        stdin()
            .read_line(&mut user_inp)
            .expect("Failed to read line");

        //check if the user selection is valid
        if let Ok(sel) = user_inp.trim().parse() {
            user_sel = sel;
            //If valid break
            if user_sel < test_enum.len() {
                break;
            }
        } else {
            continue;
        }
    }

    //Create analysis tool from chosen test
    let mut analyser = Analyser::init(depth_test_fp, test_enum[user_sel].1.clone())?;

    println!("Analyser created");

    //CURRENTLY ---------------------- RGDB vs FARO Scanner test analyses

    //Trim all the pointclouds down

    let min_x = -0.03;
    let max_x = 0.05;
    let min_y = -0.044;
    let max_y = 0.04;
    let min_z = -999.0;
    let max_z = 999.0;

    let _ = analyser.apply_passband(min_x, max_x, min_y, max_y, min_z, max_z);

    //Turn the pointcloud set into heightmaps
    let _ = analyser.create_parametric_hmaps(
        vec![5, 10, 25, 50, 75, 100, 200, 250, 500],
        vec![1, 2, 5, 10, 15, 20, 25],
        vec!["450mm", "550mm", "650mm", "750mm", "850mm", "950mm"],
    );

    let _ = analyser.display_all();

    Ok(())
}

//rustbot control!
//A rust and headerless version of the robot controller designed to run tests in the soilbed
//Version 0.0.0
//Author - Joe Ingham

use std::collections::HashMap;
use std::fs;
use std::fs::File;
use std::io::{stdin, Write};
use std::thread::sleep;
use std::time::Duration;
use raylib::ffi::PI;

mod analysis;
mod control;
mod mapping;
use crate::analysis::analyser::Analyser;
use control::abb_rob;
use crate::mapping::terr_map_sense::RealsenseCam;
use crate::mapping::terr_map_tools::Heightmap;

const VER_NUM: &str = "V0.3";
//Program title
const TITLE: &str = "Rustbot Control";

fn main() {
    println!("RUSTBOT_CNTRL STARTUP....");



    //Run the command handler
    core_cmd_handler();

    println!("Shutting down");
}

//Handles commands given by the user - without a robot
fn core_cmd_handler() {
    //Array of implemented commands
    const VALID_CMDS: [&str; 7] = [
        "info - get title and version number",
        "quit - close the program",
        "cmds - list the currently implemented commands",
        "ping - TEST - connect to and ping the robot studio",
        "connect - connect to a robot on a given ip and port (if successful unlocks robot specific commands",
        "analyse - analyse a previous tests data",
        "snsdpth - Take N heightmap measurements"
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

            "connect" => rob_connect(),

            "analyse" => {
                if let Err(e) = analyse() {
                    println!("ANALYSE ERROR - {e}")
                }
            }

            "snsdpth" => {
                if let Err(e) = save_n_heightmaps() {
                    println!("MEASURE ERROR - {e}");
                }
            }


            //Catch all else
            _ => println!("Unknown command - see CMDs for list of commands"),
        }
    }
}

//Command line for logging into and controlling a robot
fn rob_connect() {
    //Not const because you cant make constant hashmaps

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
                //Dereference login selection
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
    if let Ok(mut curr_rob) =
        abb_rob::AbbRob::create_rob(profile[0].parse().unwrap(), profile[1].parse().unwrap())
    {
        println!("Connected!");

        //Open the robot command handler - must be defined for robot!
        curr_rob.rob_cmd_handler();
    } else {
        //Robot failed to connect - go up back to core cmd handler
        println!("{TITLE} - {VER_NUM}");
        return;
    }
}

//Iterates through a tests generated heightmaps and displays them one by one
fn analyse() -> Result<(), anyhow::Error> {
    //Print and number the list of tests in the DEPTH_TESTS folder (ignoring _archive)
    const DEPTH_TEST_FP: &str = "C:/Users/User/Documents/Results/DEPTH_TESTS";

    let paths = fs::read_dir(DEPTH_TEST_FP)?;

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
    let mut analyser = Analyser::init(test_enum[user_sel].1.clone())?;

    analyser.save_coverage()?;


    Ok(())
}


//For testing coverage and variance of a realsense depth cam
fn save_n_heightmaps() -> Result<(), anyhow::Error>{


    //Create the filename
    println!("How many snapshots?");

    //Get user input
    let mut user_inp = String::new();
    stdin()
        .read_line(&mut user_inp)
        .expect("Failed to read line");

    //TODO - Handle erroneous user input
    let n = user_inp.trim().parse::<i32>()?;

    const DEPTH_TEST_FP: &str = "C:/Users/User/Documents/Results/DEPTH_TESTS";

    //Ask the user for a dataset name
    //Create the filename
    println!("Please provide a test name");

    //Get user input
    let mut user_inp = String::new();
    stdin()
        .read_line(&mut user_inp)
        .expect("Failed to read line");

    //Create a folder to hold the test data
    let new_fp = format!("{}/{}", DEPTH_TEST_FP, user_inp.trim());
    fs::create_dir(&new_fp).expect("FAILED TO CREATE NEW DIRECTORY");

    //Create a depth camera handler
    let mut cam = RealsenseCam::initialise()?;

    //Sleep for 3 seconds to let the camera warm up
    sleep(Duration::from_secs(3));

    //Measure n pcls then convert to heightmaps and save
    for i in 0..n{

        let mut curr_pcl = cam.get_depth_pnts()?;

        //Rotate the PCL to orient it correctly
        curr_pcl.rotate(PI as f32/2.0 - 0.524 , 0.0, 0.0);
        //Empirically calculated passband to isolate terrain bed
        curr_pcl.passband_filter(-1.0, 1.0, -3.8, -0.9, 0.6, 1.3);

        let mut curr_heightmap = Heightmap::create_from_pcl(curr_pcl, 250, 250, false);

        let hmap_fp = format!("{}/hmap_{}_{}", new_fp, user_inp.trim(), i);

        curr_heightmap.save_to_file(&*hmap_fp)?;
    }


    //Create an empty data file so that the folder can be used with the analyser
    let data_fp = format!("{}/data_{}.txt", new_fp, user_inp.trim());
    let mut dat_file = File::create(data_fp)?;
    dat_file.write("NODATA - PURE DEPTH TEST".as_bytes())?;


    println!("Heightmaps generated");



    Ok(())



}

//rustbot control! 
//A rust and headerless version of the robot controller designed to run tests in the soilbed 
//Version 0.0.0 
//Author - Joe Ingham

use std::io::stdin;
use std::thread;

mod tcp_sock;
mod abb_rob;

const VER_NUM: &str = "V0.0.0";
//Program title
const TITLE : &str = "Rustbot Control";

fn main() {

    //Run the command handler
    core_cmd_handler();

    println!("Shutting down");
}

//Handles commands given by the user
fn core_cmd_handler(){

    //Array of implemented commands
    const VALID_CMDS: [&str; 5]= ["info - get title and version number", 
                                  "quit - close the program", 
                                  "cmds - list the currently implemented commands", 
                                  "ping - TEST - connect to and ping the robot studio",
                                   "connect - connect to a robot on a given ip and port (if successful unlocks robot specific commands"];


    println!("{TITLE} - {VER_NUM}");

    //Loop until command given
    loop{

        //Get user input
        let mut user_inp = String::new();
        stdin()
            .read_line(&mut user_inp)
            .expect("Failed to read line");

        //Check uesr inout
        match user_inp.to_lowercase().trim() {
            "info" => {
                println!("{TITLE} - {VER_NUM}");
                println!("This program is a headerless robot control tool - intended to remotely control 6 axis robots for the TRL - create a robot to gain access to more commands");
            },
            //Print out the commands in the valid commands list
            "cmds" => {
                for cmd in VALID_CMDS{
                    println!("{cmd}");
                }
            },
            "quit" => break,

            "ping" => spam_ping(),

            //Catch all else
            _ => println!("Unknown command - see CMDs for list of commands"),
        }

    }

}

//TODO - remove ping once ABB robot implemented
//Test function just to check the TCP socket works - currently used to spam pings with a thread
fn spam_ping(){
    //Create the socket
    let mut sock = tcp_sock::create_sock("127.0.0.1".parse().unwrap(), 8888);

    //Connect to the socket
    sock.connect();

    //Spawn a thread - will use this as the basis for the robot control
    thread::spawn(move || {
        loop {
            //Write the ping message to the socket
            let s = sock.req(String::from("ECHO:ping"));

            if let Some(recv) = s{
                println!("{}", recv);
            }
            else{
                println!("Connection lost!");
                return
            }


        }
    });





    //It auto closes because it leaves the scope - noone has ownership
}

//TODO - Robot spawner - checks if robot is connected to tcp before any instructions

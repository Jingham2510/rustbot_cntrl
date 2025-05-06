//rustbot control! 
//A rust and headerless version of the robot controller designed to run tests in the soilbed 
//Version 0.0.0 
//Author - Joe Ingham

use std::io::stdin;

fn main() {
    
    loop{
        let mut user_inp = String::new();

        stdin()
            .read_line(&mut user_inp)
            .expect("Failed to read line");
        
        match user_inp.trim(){
            "test" => println!("Awesome"),
            _ => println!("Unknown"),
        }

        println!("{}", user_inp);

    }
}

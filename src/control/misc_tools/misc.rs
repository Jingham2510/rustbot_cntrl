use std::io::stdin;

///A collection of miscellaneous tools used throughout the program

///Waits for a user input (i.e. an entry)
/// Doesn't care what the input is
pub fn wait_for_enter(){

    println!("Press enter to continue...");

    //Get user input
    let mut user_inp = String::new();
    stdin()
        .read_line(&mut user_inp)
        .expect("Failed to read line");

}
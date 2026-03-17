///A collection of miscellaneous tools used throughout the program
use std::io::stdin;

///Waits for a user input (i.e. an entry)
///Used to manually pause the program
pub fn wait_for_enter() {
    println!("Press enter to continue...");

    //Block until a user presses enter
    let mut user_inp = String::new();
    stdin()
        .read_line(&mut user_inp)
        .expect("Failed to read line");
}

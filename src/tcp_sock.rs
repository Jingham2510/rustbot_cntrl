use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpStream;
use core::time::Duration;

//TCP socket structure
pub struct TcpSock {
    ip : String,
    port : i32,
    //The TCP stream itself
    stream : Option<TcpStream>,
    //Details of last error that occurred - for debugging
    last_error : Option<String>
}


//Methods for a TCP socket
impl TcpSock{

    //Connect to a socket
    pub fn connect(&mut self){
        //Connect and return the TCP _stream
        if let Ok(stream) = TcpStream::connect(format!("{}:{}", self.ip, self.port)){
            //Set the read timeout to 2 seconds - should stop blocking
            stream.set_read_timeout(Some(Duration::from_secs(2))).expect("Incorrect read timeout value!");

            self.stream = Option::from(stream);
            println!("Connected to {0}:{1}", self.ip, self.port);
        }
        //If cant connect
        else{
            self.last_error = Option::from(String::from("Failed to connect"));
            println!("Failed to connect...");

        }
    }

    //Writes a message to the TCP connection buffer
    //Returns a boolean true if successful
    fn write(& mut self, msg : String) -> bool{
        //Check if the stream exists
        if let Some(mut writer) = self.stream.as_ref(){
            //Attempt to write the data
            if let Ok(_ok) = writer.write_all(msg.as_bytes()){
                println!("Data written");
                true
            }
            else{
                println!("Failed to write!");
                self.last_error = Option::from(String::from("Write failure"));
                false
            }
        }
        else{
            println!("No connection... Not writing...");
            false
        }
    }



    //Reads from the TCP input buffer - returns the info as a string
    fn read(&mut self) -> Option<String>{


        //Create buffer for the message
        let mut recv = vec![];

        //Create the listener as a buffer reader so we can read until a given character
        let mut reader = BufReader::new(self.stream.as_ref().unwrap());

        //Attempt to read from the TCP stream until the '!' character is reached
        if let Ok(_size) = reader.read_until(b'!', &mut recv){

            //println!("{:?}", String::from_utf8(Vec::from(recv)));

            Option::from(String::from_utf8(Vec::from(recv)).expect("FAILED TO CONVERT RECEIVE STRING"))
        }
        else{
            println!("Failed to read TCP stream");
            self.last_error = Option::from(String::from("TCP Read Error"));
            Option::None
        }

  
    }

    
    //Public interface for requesting for a TCP connection
    pub fn req(&mut self, msg : String) -> Option<String> {
        self.write(msg);
        self.read()
    }


}


//Create a new socket and attempt to connect to it
pub fn create_sock(ip: String, port: i32) -> TcpSock{

    //Create a socket
    let new_sock = TcpSock{
        ip,
        port,
        stream : None,
        last_error : Option::from(String::new())
    };

    new_sock
}


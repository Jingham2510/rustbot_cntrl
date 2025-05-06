use std::io::Write;
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
            stream.set_read_timeout(Option::Some(Duration::from_secs(2))).expect("Incorrect read timeout value!");

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
    pub fn write(& mut self, msg : String) -> bool{

        //Check if 
        if let Some(mut writer) = self.stream.as_ref(){
            if let Ok(size) = writer.write_all(msg.as_bytes()){
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
    pub fn read(&self) -> String{
        todo!()
  
    }

    
    //Public interface for requesting for a TCP connection
    pub fn req(&mut self, msg : String) -> String{
        
        self.write( msg);
        
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


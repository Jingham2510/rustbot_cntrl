use anyhow::bail;
use core::time::Duration;
use std::io::{BufRead, BufReader, Write};
use std::net::{Shutdown, TcpStream};

//TCP socket structure
pub struct TcpSock {
    ip: String,
    port: u32,
    //The TCP stream itself
    stream: Option<TcpStream>,
    //Details of last error that occurred - for debugging
    last_error: Option<String>,
    //Indicates whether the socket is connected
    connected: bool,
}

//Methods for a TCP socket
impl TcpSock {
    //Connect to a socket
    pub fn connect(&mut self) -> bool {
        //Connect and return the TCP _stream
        if let Ok(stream) = TcpStream::connect(format!("{}:{}", self.ip, self.port)) {
            //Set the read timeout to 2 seconds - should stop blocking
            stream
                .set_read_timeout(Some(Duration::from_secs(120)))
                .expect("Incorrect read timeout value!");

            self.stream = Option::from(stream);
            self.connected = true;
            println!("Connected to {0}:{1}", self.ip, self.port);
            true
        }
        //If cant connect
        else {
            self.last_error = Option::from(String::from("Failed to connect"));
            println!("Failed to connect...");
            self.connected = false;
            false
        }
    }

    //Writes a message to the TCP connection buffer
    //Returns a boolean true if successful
    fn write(&mut self, msg: &str) -> bool {
        //Check if the stream exists
        if let Some(mut writer) = self.stream.as_ref() {
            //Attempt to write the data
            if let Ok(_ok) = writer.write_all(msg.as_bytes()) {
                true
            } else {
                println!("Failed to write!");
                self.last_error = Option::from(String::from("Write failure"));
                self.connected = false;
                false
            }
        } else {
            println!("No connection... Not writing...");
            false
        }
    }

    //Reads from the TCP input buffer - returns the info as a string
    fn read(&mut self) -> Result<String, anyhow::Error> {
        //Create buffer for the message
        let mut recv = vec![];

        //Create the listener as a buffer reader so we can read until a given character
        let mut reader = BufReader::new(self.stream.as_ref().unwrap());

        //Attempt to read from the TCP stream until the '!' character is reached
        if let Ok(_size) = reader.read_until(b'!', &mut recv) {

            let recv_str = String::from_utf8(Vec::from(recv))?;

            //println!("{:?}", recv_str);

            Ok(recv_str)
        } else {
            println!("Failed to read TCP stream");
            self.last_error = Option::from(String::from("TCP Read Error"));
            self.connected = false;
            bail!("Failed to read TCP stream");
        }
    }

    //Public interface for sending a request to the robot 
    pub fn req(&mut self, msg: &str) -> Result<String, anyhow::Error> {
        self.write(msg);

        let s = self.read();

        match s {
            Ok(mut s) => {
                //println!("{s}");
                //Remove the ! character
                s.pop();
                Ok(s)
            }
            Err(e) => bail!("Failed to read from TCP stream - {e}"),
        }
    }

    //Close the stream by shutting it down
    pub fn disconnect(&mut self) {




        self.stream
            .as_ref()
            .unwrap()
            .shutdown(Shutdown::Both)
            .expect("Failed to shutdown! Panic!");
        self.stream = None;
        self.connected = false;
    }
}

//Create a new socket and attempt to connect to it
pub fn create_sock(ip: String, port: u32) -> TcpSock {
    //Create a socket
    let new_sock = TcpSock {
        ip,
        port,
        stream: None,
        last_error: Option::from(String::new()),
        connected: false,
    };

    new_sock
}

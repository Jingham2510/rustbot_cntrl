/*
Handles all UPD EGM processes
Based on abbegm-rs by robohouse
 */

use crate::control::egm_control::abb_egm::{EgmRobot, EgmSensor};
use anyhow::bail;
use prost::Message;
use std::net::UdpSocket;

//The UDP socket that sends/recieves egm protobuffer
pub struct EgmServer {
    socket : UdpSocket
}

//The default EgmServer is a local connection for robotstudio
impl Default for EgmServer {
    fn default() -> Self {
        Self::local()
    }
}

impl EgmServer {
    //Creates the EGM udp socket (a close is not required as it will be dropped when it stops being used)
    pub fn create_egm_socket(socket: UdpSocket) -> Self {

        println!("UDP socket bound to: {:?}", socket.local_addr());

        EgmServer {
            socket
        }
    }

    pub fn local() -> Self{
        Self::create_egm_socket(UdpSocket::bind("127.0.0.1:6510").unwrap())
    }

    pub fn remote() -> Self{
        Self::create_egm_socket(UdpSocket::bind("192.168.10.20:6510").unwrap())
    }

    //TODO: send & recv to a bound connection
    pub fn send_egm(&self, msg: EgmSensor) -> Result<(), anyhow::Error> {

        //Encode the message into a btye vector
        let encoded_msg = msg.encode_to_vec();

        //Send the bytes over the UDP socket
        let bytes_sent = self.socket.send(&encoded_msg);

        //Check to make sure that the entire message was sent
        if bytes_sent? != encoded_msg.len() {
            bail!("Failed to send all bytes")
        }
        Ok(())
    }

    pub fn recv_egm(&self) -> Result<EgmRobot, anyhow::Error>{

        //Allocate a MB for recieving the data
        let mut buffer = vec![0u8; 1024];
        //Recieve the data
        let (bytes_recieved) = self.socket.recv(&mut buffer)?;
        //Decode the bytes
        Ok(EgmRobot::decode(&buffer[..bytes_recieved])?)
    }


    //Recieves a message from any UDP and then attempts to connect to it for sending messages
    pub fn recv_and_connect(&self) -> Result<EgmRobot, anyhow::Error>{

        //Allocate a MB for recieving the data
        let mut buffer = vec![0u8; 1024];
        //Recieve the data
        let (bytes_recieved, addr) = self.socket.recv_from(&mut buffer)?;

        //Attempt to connect to the socket address
        if self.socket.connect(addr).is_ok(){
            println!("Bound to {:?}", addr);
        }else{
            bail!("Failed to connect to EGM client");
        }


        //Decode the bytes
        Ok(EgmRobot::decode(&buffer[..bytes_recieved])?)

    }
}
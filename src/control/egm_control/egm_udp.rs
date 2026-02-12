/*
Handles all UPD EGM processes
Based on abbegm-rs by robohouse
 */

use std::net::UdpSocket;


//The UDP socket that sends/recieves egm protobuffer
pub struct EgmServer {
    socket : UdpSocket
}

//The default EgmServer is a local connectoin for robotstudio
impl Default for EgmServer {
    fn default() -> Self {
        EgmServer {
            socket : UdpSocket::bind("127.0.0.1:1234").unwrap()
        }
    }
}

impl EgmServer {

    //Creates the EGM udp socket (a close is not required as it will be dropped when it stops being used)
    pub fn create_egm_socket(socket : UdpSocket) -> Self{
        EgmServer {
            socket
        }
    }

    //TODO: send & recv to a bound connection



}
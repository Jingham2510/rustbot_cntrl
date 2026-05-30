//! Used to remotely control the camera subsystem - i.e. extract heightmap information
use std::net::{TcpStream, UdpSocket};
use ssh2::Session;
use ssh2::DisconnectCode;
use anyhow::bail;
use std::io::{Read,Write};
use std::time::Duration;
use std::thread::sleep;
use std::sync::mpsc::{Receiver, Sender};
use rustgeomapping::data_types::heightmap::Heightmap;


///Subsystem control module using ssh and serial
pub struct CamSysCntrl{
    //Ip of subsystem device
    ip : String,
    //User name of subsystem device
    user : String,
    //SSH control session of subsystem
    ssh_sess : Session,
    //Robot position/orientation reciever
    pos_ori_rx : Receiver<[f32;7]>,
    //Heightmap transmitter
    heightmap_tx : Sender<Heightmap>,
    //Control reciever
    cntrl_rx : Receiver<u32>

}


impl CamSysCntrl{

    pub fn default_connect(pos_ori_rx : Receiver<[f32;7]>, heightmap_tx : Sender<Heightmap>, cntrl_rx : Receiver<u32>) -> Result<Self, anyhow::Error>{


        const DEFAULT_IP : &str = "192.168.55.1";
        const DEFAULT_CNTRL_PORT : &str = "22";
        const DEFAULT_USER : &str = "trl";
        const DEFAULT_PASS : &str = "trl";

        println!("Connecting to camera sub-system...");

        //Connect the tcp stream to the device
        if let Ok(tcp) = TcpStream::connect(&format!("{}:{}", DEFAULT_IP, DEFAULT_CNTRL_PORT)){

            //Create the ssh session
            if let Ok(mut ssh_sess) = Session::new(){
                //Set the tcp stream
                ssh_sess.set_tcp_stream(tcp);

                //Confirm the ssh connection
                ssh_sess.handshake()?;

                println!("Connected!");

                ssh_sess.userauth_password(DEFAULT_USER, DEFAULT_PASS)?;

                if ssh_sess.authenticated(){

                    println!("Authenticated...");

                    Ok(Self{
                        ip : String::from(DEFAULT_IP),
                        user: String::from(DEFAULT_USER),
                        ssh_sess,
                        pos_ori_rx,
                        heightmap_tx,
                        cntrl_rx
                    })
                }else{
                    bail!("Failed to authenticate ssh");
                }


            }else{
                bail!("Failed to create ssh session")
            }


        }else{
            bail!("Failed to connect TCP stream to device")
        }
    }


    ///Runs a manual command - mainly for testing
    pub fn cmd_and_resp(&self, cmd : &str) -> String{   
       
        let mut ch_sess = self.ssh_sess.channel_session().unwrap();

        ch_sess.exec(cmd);

        let mut s = String::new();

        ch_sess.read_to_string(&mut s).unwrap();

        ch_sess.wait_close();

        s
    }


    pub fn start_system(&self) -> Result<(), anyhow::Error>{

       
        //setup-------------
        
        //Create a channel session and turn on the shell
        let mut ch_sess = self.ssh_sess.channel_session().unwrap();
        ch_sess.shell()?;


        //Go to the folder
        ch_sess.write(b"cd ../../media/trl/main/Programming/trl_cam_subsystem\n")?;
        
   
        //Make sure the rust program is up to date
        ch_sess.write(b"cargo build --release\n")?;
        ch_sess.write(b"cd target/release\n")?;
        //start the rust program
        ch_sess.write(b"./trl_cam_subsystem -auto\n")?;
        ch_sess.send_eof();



        //Sleep for 5 seconds to allow subsystem warmup
        sleep(Duration::from_secs(5));

        //control loop-----------
       self.run_system_stream();        

        
     
            


        Ok(())

    }

    //Streams the data to and from the camera subsystem
    fn run_system_stream(&self) -> Result<(), anyhow::Error>{

        //UDP setup----------

        //Open the local socket
        let mut data_stream = UdpSocket::bind("0.0.0.0:8080")?;

        //Connect to the remote socket
        data_stream.connect("192.168.55.1:8080")?;

        //Check the connection------------
        data_stream.send(b"CONNECT?")?;

        let mut udp_buf : [u8; 1024] = [0;1024];

        let n = data_stream.recv(&mut udp_buf)?;

        //If connection valid - do main loop
        if str::from_utf8(&udp_buf[..n])? == "YES"{

            //Get the heightmap width and height

            //Create the empty heightmap object with the specified size

            
            //Main loop

            //Send the position/orientation

            //Decode heightmap cell data -> update heightmap cells


            //Check cntrl (i.e. close connection?)


        }


        Ok(())
    }

    //Closes the remote session and consumes self
    pub fn close_connection(&mut self) -> Result<(), ssh2::Error> {
        self.ssh_sess.disconnect(Option::from(DisconnectCode::AuthCancelledByUser), "Manually closed connection", None)
    }

}
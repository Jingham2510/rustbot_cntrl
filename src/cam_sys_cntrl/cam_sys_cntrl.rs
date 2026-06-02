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
use tokio::sync::watch;

///Subsystem control module using ssh and serial
pub struct CamSysCntrl{
    ///Ip of subsystem device
    ip : String,
    ///User name of subsystem device
    user : String,
    ///SSH control session of subsystem
    ssh_sess : Session,
    ///Robot position/orientation reciever
    pos_ori_rx : watch::Receiver<[f32;7]>,
    ///Heightmap transmitter
    heightmap_tx : Sender<Heightmap>,
    ///Control reciever
    cntrl_rx : watch::Receiver<u32>

}


impl CamSysCntrl{

    pub fn default_connect(pos_ori_rx : watch::Receiver<[f32;7]>, heightmap_tx : Sender<Heightmap>, cntrl_rx : watch::Receiver<u32>) -> Result<Self, anyhow::Error>{


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


    pub fn start_system(&mut self) -> Result<(), anyhow::Error>{

       
        //setup-------------
        
        //Create a channel session and turn on the shell
       
        let mut ch_sess = self.ssh_sess.channel_session().unwrap();
        ch_sess.shell()?;

        //Go to the folder
        ch_sess.write(b"cd ../../media/ssd/Programming/trl_cam_subsystem\n")?;
        
   
        //Make sure the rust program is up to date
        ch_sess.write(b"cargo build --release\n")?;
        ch_sess.write(b"cd target/release\n")?;
        //start the rust program
        ch_sess.write(b"./trl_cam_subsystem -auto\n")?;
        ch_sess.send_eof();



        //Sleep for 5 seconds to allow subsystem warmup
        sleep(Duration::from_secs(5));
        

        //control loop-----------
       println!("subsytem UDP: {:?}", self.run_system_stream());       



       //Make sure to close the port
       ch_sess.close();


       Ok(())        
     
        
    }

    //Streams the data to and from the camera subsystem
    fn run_system_stream(&mut self) -> Result<(), anyhow::Error>{

        //UDP setup----------
        //Open the local socket
        let mut data_stream = UdpSocket::bind("0.0.0.0:8080")?;

        //Connect to the remote socket
        data_stream.connect("192.168.55.1:8080")?;

        data_stream.set_read_timeout(Some(Duration::from_secs(10)))?;

        //Check the connection------------
        data_stream.send(b"CONNECT?")?;

        let mut udp_buf : [u8; 1024] = [0;1024];

        let n = data_stream.recv(&mut udp_buf)?;

        //If connection valid - do main loop
        if str::from_utf8(&udp_buf[..n])? == "YES"{

            println!("UDP connection confirmed...");

            //Get the heightmap width and height
            let hmap_size = Self::get_hmap_size(&data_stream)?;

            //Create the empty heightmap object with the specified size
            let mut global_hmap = Heightmap::new(hmap_size[0], hmap_size[1]);

            println!("Global heightmap created - size W:{}-H:{}", global_hmap.width(), global_hmap.height());

            let mut i : f32 = 0.0;

            //Main loop
            loop{
                
                //Send the position/orientation
                if self.pos_ori_rx.has_changed()?{
                    //let curr_pos_ori = *self.pos_ori_rx.borrow_and_update();

                    let curr_pos_ori = [i, i, 0.0, 1.0, 0.0, 0.0, 0.0];

                    data_stream.send(format!("{},{},{},{},{},{},{}", curr_pos_ori[0], curr_pos_ori[1], curr_pos_ori[2], curr_pos_ori[3], curr_pos_ori[4], curr_pos_ori[5], curr_pos_ori[6]).as_bytes())?;

                    //Set the heightmap
                    self.set_heightmap(&data_stream, &mut global_hmap)?;

                    //Clone the heightmap to the main thread
                    self.heightmap_tx.send(global_hmap.clone())?;

                    i += 0.01;

                }               
                
                //Check cntrl (i.e. close connection)
                if self.cntrl_rx.has_changed()? /*|| i > 1.0*/{
                    data_stream.send(b"CLOSE")?;
                    break;
                }          
            }

        }else{
            bail!("Failed to connect UDP stream");
        }




        Ok(())
    }


        
    

    //Get and decode the heightmap size
    fn get_hmap_size(data_stream : &UdpSocket) -> Result<[usize; 2], anyhow::Error>{


        let mut hmap_size_buf : [u8;9] = [0;9];

        data_stream.send(b"GLOBAL_SIZE?")?;

        let n_recv = data_stream.recv(&mut hmap_size_buf)?;

        let sz_tokens : Vec<&str> = str::from_utf8(&hmap_size_buf[..n_recv])?.split(",").collect();


        Ok([sz_tokens[0].parse()?, sz_tokens[1].parse()?])
    }


    fn set_heightmap(&mut self, data_stream : &UdpSocket, global_hmap : &mut Heightmap) -> Result<(), anyhow::Error>{


        let i_max = global_hmap.width();
        let j_max = global_hmap.height();

        let mut cells : Vec<f32>  = vec![];

        let mut hmap_buf : [u8; 512] = [0;512];
        
        data_stream.recv(&mut hmap_buf)?;

        //Guaranteed that the number string is small
        let no_str = str::from_utf8(&hmap_buf[..4])?;
        let no_of_packets = no_str.trim().parse::<u32>()?;

        //Get the number of packets to recieve
        for i in 0..no_of_packets{
            //Receive the packet
            data_stream.recv(&mut hmap_buf)?;
            //Convert the array of float bytes into floats
            cells.append(&mut hmap_buf.chunks(4).map(TryInto::try_into).map(Result::unwrap).map(f32::from_be_bytes).collect());
            
            if i != no_of_packets{
                data_stream.send(b"NEXT")?;
            }
        }        

        //Go through each cell and update
        let mut total_cells = 0;
        for i in 0..i_max{
            for j in 0..j_max{ 

            if total_cells == cells.len(){
                break;
            }
            global_hmap.set_cell_height_no_check(i, j, cells[total_cells])?;  
            total_cells += 1; 
            }             
            
        }

        println!("Heightmap recieved");
        Ok(())

    }




    //Closes the remote session and consumes self
    pub fn close_connection(&mut self) -> Result<(), ssh2::Error> {

        self.ssh_sess.disconnect(Option::from(DisconnectCode::AuthCancelledByUser), "Manually closed connection", None)
    }

}
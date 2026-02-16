/*
Constructor functions for the EGM structs used in communication with ABBs EGM
Based off - abbegm-rs developed by robohouse in delft
 */
use std::ops::Add;
use std::time::Duration;
use crate::control::egm_control::abb_egm::*;
use crate::control::egm_control::abb_egm::egm_header::MessageType;



impl EgmHeader {

    //This device will only create messages that are of the correction type (i.e. not a header the robot will generate)
    pub fn create_header(seqno:u32, timestamp:u32) ->Self{
        EgmHeader{
            seqno : Some(seqno),
            //Timestamp in milliseconds
            tm : Some(timestamp),
            mtype : Some(MessageType::MsgtypeCorrection.into())
        }
    }
}

impl EgmCartesian{

    //Create the EGM cartesian message from provided
    pub fn create_egm_cart(xyz : [f64;3]) -> Self{
        EgmCartesian{
            x : xyz[0],
            y : xyz[1],
            z : xyz[2]
        }
    }
    pub fn get_coords(&self) -> [f64;3]{
        [self.x, self.y, self.z]
    }
}

impl EgmQuaternion {
    //Create a quaternion the robot uses to set the TCP orientation
    pub fn create_egm_quart(wxyz : [f64;4]) -> Self{
        EgmQuaternion{
            u0 : wxyz[0],
            u1 : wxyz[1],
            u2 : wxyz[2],
            u3 : wxyz[3]
        }
    }

    pub fn get_quart(&self) -> [f64;4]{
        [self.u0, self.u1, self.u2, self.u3]
    }
}

impl EgmEuler {
    //Creates a set of euler angles the robot uses to determine TCP orientation
    pub fn create_egm_euler(xyz : [f64; 3]) -> Self{
        EgmEuler{
            x : xyz[0],
            y : xyz[1],
            z : xyz[2]
        }
    }

    pub fn get_euler(&self) -> [f64; 3]{
        [self.x ,self.y, self.z]
    }
}

//Speciifes the desired time of arrival for a given instruction
impl EgmClock {
    pub fn create_egm_clock(sec: u64, usec: u64) -> Self {
        EgmClock {
            sec,
            usec
        }
    }

    //Time functions taken from abbegm-rs
    pub fn elapsed_since_epoch(&self) -> Duration {
        let secs = self.sec + self.usec / 1_000_000;
        let nanos = (self.usec % 1_000_000) as u32 * 1_000;
        Duration::new(secs, nanos)
    }

    //Get the elapsed time as msecs since the epoch
    pub fn as_timestamp_ms(&self) -> u32 {
        self.sec.wrapping_mul(1_000).wrapping_add(self.usec / 1_000) as u32
    }


    pub fn as_tuple(&self) -> (u64, u64){
        (self.sec, self.usec)
    }
}

impl Add<Duration> for EgmClock {
    type Output = Self;

    fn add(self, right: Duration) -> Self::Output {
        let usec = self.usec + u64::from(right.subsec_micros());
        EgmClock {
            sec: self.sec + right.as_secs() + usec / 1_000_000,
            usec: usec % 1_000_000,
        }
    }
}
impl Add<EgmClock> for Duration {
    type Output = EgmClock;
    fn add(self, right: EgmClock) -> Self::Output {
        right + self
    }
}

impl From<(u64, u64)> for EgmClock{
    fn from(other : (u64, u64)) -> Self{
        EgmClock{
            sec : other.0,
            usec : other.1
        }
    }
}

impl EgmTimestamp{
    //Timestamp for logging and reporting
    pub fn create_egm_timestamp(sec : u64, nsec : u64) -> Self{
        EgmTimestamp{
            sec : Some(sec),
            nsec : Some(nsec)
        }
    }
}



impl EgmPose{
    //We only want to specify poses using quartenions (much better to use)
    pub fn create_egm_pose(xyz : [f64;3], wxyz : [f64; 4]) -> Self{
        EgmPose{
            pos : Some(EgmCartesian::create_egm_cart(xyz)),
            orient : Some(EgmQuaternion::create_egm_quart(wxyz)),
            euler : None
        }
    }
}

impl EgmCartesianSpeed {

    //Create a speed reference in mm/s (xyz only - not interested in rotation)
    pub fn create_egm_cart_speed(xyz : [f64;3]) -> Self{
        EgmCartesianSpeed{
            value : vec![xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0]
        }
    }

}

impl EgmJoints{
    //Creates an EGM joint spec - based in degrees
    pub fn create_egm_joints (joint_list : [f64;6]) -> Self{
        EgmJoints{
            joints : Vec::from(joint_list)
        }
    }
}


impl EgmPlanned{
    //Create a set of joint angles that the robot will move to
    pub fn create_egm_planned_joints(joint_list : [f64;6], time : (u64, u64)) -> Self{
        EgmPlanned{
            joints : Some(EgmJoints::create_egm_joints(joint_list)),
            time : Some(time.into()),
            cartesian :None,
            external_joints : None,
            time_stamp : None
        }
    }

    //Create an xyz position and wxyz quaternion that the robot will move to
    pub fn create_egm_planned_cartesian(xyz : [f64;3], wxyz : [f64; 4], time : (u64, u64)) -> Self{
        EgmPlanned{
            cartesian : Some(EgmPose::create_egm_pose(xyz, wxyz)),
            time : Some(time.into()),
            joints : None,
            external_joints : None,
            time_stamp : None
        }
    }
}

//Creates a speed reference either linear (xyz mm/s) or joint speed (deg/s)
impl EgmSpeedRef{

    //Create a desired joint speed (deg/s) for the robot to move at
    pub fn create_egm_speed_joints(joints : [f64;6]) -> Self{
        EgmSpeedRef{
            joints : Some(EgmJoints::create_egm_joints(joints)),
            cartesians : None,
            external_joints : None
        }
    }

    //Create a desired linear speed (mm/s) that the TCP will move at
    pub fn create_egm_speed_linear(xyz : [f64;3]) -> Self{
        EgmSpeedRef{
            cartesians : Some(EgmCartesianSpeed::create_egm_cart_speed(xyz)),
            joints : None,
            external_joints : None
        }
    }
}

//Note: No need to extract feedback vars as all are pub
//We extract all through the header anyways
impl EgmRobot{

    //Header info
    pub fn get_sqno(&self) -> Option<u32> {
        self.header?.seqno
    }
    pub fn get_timestamp(&self) -> Option<u32>{
        self.header?.tm
    }

    //Feedback info
    pub fn get_joint_pos(&self) -> Option<&Vec<f64>>{
        Some(&self.feed_back.as_ref()?.joints.as_ref()?.joints)
    }

    pub fn get_pos_xyz(&self) -> Option<[f64; 3]>{
        let xyz = &self.feed_back.as_ref()?.cartesian?.pos?;

        Some([xyz.x, xyz.y, xyz.z])
    }

    pub fn get_quart_ori(&self) -> Option<[f64;4]>{
        let wxyz = &self.feed_back.as_ref()?.cartesian?.orient?;
        Some([wxyz.u0, wxyz.u1, wxyz.u2, wxyz.u3])
    }

    pub fn get_time(&self) -> Option<(u64, u64)>{
        let time = &self.feed_back.as_ref()?.time?;

        Some((time.sec, time.usec))
    }

    //Force info
    pub fn get_measured_force(&self) -> Option<[f64; 6]>{
        let force = &self.measured_force.as_ref()?.force;

        if force.len() != 6{
            None
        }else {
            Some([force[0], force[1], force[2], force[3], force[4], force[5]])
        }
    }
}

impl EgmSensor {

    //Create a desired pose message that the robot should move to
    pub fn set_pose(seqno : u32, time : (u64, u64), xyz : [f64;3], wxyz : [f64;4]) -> Self{
        let clock = EgmClock::from(time);

        EgmSensor{
            header : Some(EgmHeader::create_header(seqno, clock.as_timestamp_ms())),
            planned : Some(EgmPlanned::create_egm_planned_cartesian(xyz, wxyz, time)),
            speed_ref : None,
            rapi_dto_robot : None
        }
    }

    //Createa desired pose message with a desired speed
    pub fn set_pose_set_speed(seqno : u32, time : (u64, u64), xyz : [f64;3], wxyz : [f64; 4], xyz_speed : [f64;3]) -> Self{

        let clock = EgmClock::from(time);

        EgmSensor{
            header : Some(EgmHeader::create_header(seqno, clock.as_timestamp_ms())),
            planned : Some(EgmPlanned::create_egm_planned_cartesian(xyz, wxyz, time)),
            speed_ref : Some(EgmSpeedRef::create_egm_speed_linear(xyz_speed)),
            rapi_dto_robot : None
        }
    }

    //Create a set of desired joint positions
    pub fn set_joints(seqno : u32, time : (u64, u64), joints : [f64; 6]) -> Self{
        let clock = EgmClock::from(time);

        EgmSensor{
            header : Some(EgmHeader::create_header(seqno, clock.as_timestamp_ms())),
            planned : Some(EgmPlanned::create_egm_planned_joints(joints, time)),
            speed_ref : None,
            rapi_dto_robot : None
        }
    }

    //Create a set of desired joint positions with a desired joint speed
    pub fn set_joints_set_speed(seqno : u32, time : (u64, u64), joints : [f64; 6], joints_speed : [f64; 6]) -> Self{
        let clock = EgmClock::from(time);

        EgmSensor{
            header : Some(EgmHeader::create_header(seqno, clock.as_timestamp_ms())),
            planned : Some(EgmPlanned::create_egm_planned_joints(joints, time)),
            speed_ref : Some(EgmSpeedRef::create_egm_speed_joints(joints_speed)),
            rapi_dto_robot : None
        }
    }


}


//Quartenion structure



//Implementations/Calculations taken from
//https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
pub struct Quartenion {
    pub w : f32,
    pub x : f32,
    pub y : f32,
    pub z : f32,
}

impl Quartenion{
    //Print out the quartenion in human-readable format
    pub fn print(&self){
        println!("X: {}, Y: {}, Z: {}, W: {}", self.x, self.y, self.z, self.w);
    }
}


struct Euler {
    //x
    roll : f32,
    //y
    pitch : f32,
    //z
    yaw : f32,
}

const PI : f32 = std::f32::consts::PI;



//Euler structure
// //x - roll, y - pitch, z - yaw,
pub fn quart_to_euler(q: Quartenion) -> Euler{

    let pitch = (2.0 * ((q.w * q.y) - (q.x * q.z))).asin();

    //Check for gimbal lock
    //Positive gimbal lock
    if pitch == PI/2.0 {
        let roll = 0.0;
        let yaw = -2.0 * q.x.atan2(q.w);

        Euler{yaw, roll, pitch}
    }
    //Negative gimbal lock
    else if pitch == -PI/2.0{
        let roll = 0.0;
        let yaw = 2.0 * q.x.atan2(q.w);

        Euler{yaw, roll, pitch}

    }
    //No gimbal lock
    else {
        let roll = (2.0 * ((q.w * q.x) + (q.y * q.z))).atan2((q.w.powi(2)) - (q.x.powi(2)) - (q.y.powi(2)) + (q.z.powi(2)));

        let yaw = (2.0 * ((q.w * q.z) + (q.x * q.y))).atan2((q.w.powi(2)) + (q.x.powi(2)) - (q.y.powi(2)) - (q.z.powi(2)));

        Euler { yaw, roll, pitch }
    }
}

//Converts a ZYX euler angle to a quartenion
pub fn euler_to_quart(e: Euler) -> Quartenion{

    let w = ((e.roll/2.0).cos() * (e.pitch/2.0).cos() * ((e.yaw/2.0).cos())) + ((e.roll/2.0).sin() * (e.pitch/2.0).sin() * (e.yaw/2.0).sin());

    let x = ((e.roll/2.0).sin() * (e.pitch/2.0).cos() * ((e.yaw/2.0).cos())) + ((e.roll/2.0).cos() * (e.pitch/2.0).sin() * (e.yaw/2.0).sin());

    let y= ((e.roll/2.0).cos() * (e.pitch/2.0).sin() * ((e.yaw/2.0).cos())) + ((e.roll/2.0).sin() * (e.pitch/2.0).cos() * (e.yaw/2.0).sin());

    let z = ((e.roll/2.0).cos() * (e.pitch/2.0).cos() * ((e.yaw/2.0).sin())) + ((e.roll/2.0).sin() * (e.pitch/2.0).sin() * (e.yaw/2.0).cos());


    Quartenion{w, x, y, z}
}



//Rotate q1 via  q2 another quartenion using hamiltonian multiplication
pub fn quart_rotate(q1: Quartenion, q2: Quartenion) -> Quartenion{
    let w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
    let x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y*q2.z) - (q1.z * q2.y);
    let y = (q1.w * q2.y) + (q1.y * q2.w) + (q1.z*q2.x) - (q1.x * q2.z);
    let z = (q1.w * q2.z) + (q1.z * q2.w) + (q1.x * q2.y) - (q1.y * q2.x);

    Quartenion{w, x, y, z}
}










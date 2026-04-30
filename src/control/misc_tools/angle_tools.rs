///A set of tools to convert from Quaternions to ZYX Euler angles
///Implementations/Calculations taken from
///https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

///The quaternion structure
#[derive(Clone, Copy)]
pub struct Quaternion {
    ///Real value
    pub w: f64,
    ///x
    pub x: f64,
    ///y
    pub y: f64,
    ///z
    pub z: f64,
}

impl Quaternion {
    ///Print out the quaternion in human-readable format
    pub fn print(&self) {
        println!("X: {}, Y: {}, Z: {}, W: {}", self.x, self.y, self.z, self.w);
    }
}

impl From<[f64; 4]> for Quaternion {
    ///Create a quaternion from an array
    fn from(value: [f64; 4]) -> Self {
        Quaternion {
            w: value[0],
            x: value[1],
            y: value[2],
            z: value[3],
        }
    }
}

///Euler angle
///x - roll, y - pitch, z - yaw,
pub struct Euler {
    ///x
    roll: f64,
    ///y
    pitch: f64,
    ///z
    yaw: f64,
}

///f64 Pi alias
const PI: f64 = std::f64::consts::PI;

///Convert a quaternion to a Euler angle
pub fn quart_to_euler(q: Quaternion) -> Euler {
    let pitch = (2.0 * ((q.w * q.y) - (q.x * q.z))).asin();

    //Check for gimbal lock
    //Positive gimbal lock
    if pitch == PI / 2.0 {
        let roll = 0.0;
        let yaw = -2.0 * q.x.atan2(q.w);

        Euler { yaw, roll, pitch }
    }
    //Negative gimbal lock
    else if pitch == -PI / 2.0 {
        let roll = 0.0;
        let yaw = 2.0 * q.x.atan2(q.w);

        Euler { yaw, roll, pitch }
    }
    //No gimbal lock
    else {
        let roll = (2.0 * ((q.w * q.x) + (q.y * q.z)))
            .atan2((q.w.powi(2)) - (q.x.powi(2)) - (q.y.powi(2)) + (q.z.powi(2)));

        let yaw = (2.0 * ((q.w * q.z) + (q.x * q.y)))
            .atan2((q.w.powi(2)) + (q.x.powi(2)) - (q.y.powi(2)) - (q.z.powi(2)));

        Euler { yaw, roll, pitch }
    }
}

///Converts a ZYX euler angle to a quaternion
pub fn euler_to_quart(e: Euler) -> Quaternion {
    let w = ((e.roll / 2.0).cos() * (e.pitch / 2.0).cos() * ((e.yaw / 2.0).cos()))
        + ((e.roll / 2.0).sin() * (e.pitch / 2.0).sin() * (e.yaw / 2.0).sin());

    let x = ((e.roll / 2.0).sin() * (e.pitch / 2.0).cos() * ((e.yaw / 2.0).cos()))
        + ((e.roll / 2.0).cos() * (e.pitch / 2.0).sin() * (e.yaw / 2.0).sin());

    let y = ((e.roll / 2.0).cos() * (e.pitch / 2.0).sin() * ((e.yaw / 2.0).cos()))
        + ((e.roll / 2.0).sin() * (e.pitch / 2.0).cos() * (e.yaw / 2.0).sin());

    let z = ((e.roll / 2.0).cos() * (e.pitch / 2.0).cos() * ((e.yaw / 2.0).sin()))
        + ((e.roll / 2.0).sin() * (e.pitch / 2.0).sin() * (e.yaw / 2.0).cos());

    Quaternion { w, x, y, z }
}

///Rotate q1 via  q2 another quaternion using hamiltonian multiplication
pub fn quart_rotate(q1: Quaternion, q2: Quaternion) -> Quaternion {
    let w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
    let x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y);
    let y = (q1.w * q2.y) + (q1.y * q2.w) + (q1.z * q2.x) - (q1.x * q2.z);
    let z = (q1.w * q2.z) + (q1.z * q2.w) + (q1.x * q2.y) - (q1.y * q2.x);

    Quaternion { w, x, y, z }
}

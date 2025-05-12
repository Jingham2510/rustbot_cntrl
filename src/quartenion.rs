
//Quartenion structure



//Implementations/Calculations taken from
//https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
struct Quartenion {
    x : f32,
    y : f32,
    z : f32,
    w : f32,
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
    else{
        
        let roll = (2.0 * ((q.w*q.x) + (q.y*q.z))).atan2((q.w.powi(2)) - (q.x.powi(2)) - (q.y.powi(2)) + (q.z.powi(2)));
        
        let yaw = (2.0* ((q.w * q.z) + (q.x * q.y))).atan2((q.w.powi(2)) + (q.x.powi(2)) - (q.y.powi(2)) - (q.z.powi(2)));

        Euler{yaw, roll, pitch}
    }


}












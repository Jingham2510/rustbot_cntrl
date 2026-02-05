//Models the IRB 6400 robot
extern crate nalgebra as na;

use na::Matrix4;
use nalgebra::{Matrix6, Matrix6xX};

const PI_2 : f32 = std::f32::consts::FRAC_PI_2;
const PI : f32 = std::f32::consts::PI;
//A(mm) - CURRENT TOOL SPHERE!
const LINK_LENGTHS : [f32;7] = [240.0, 1050.0, 225.0, 0.0, 0.0, 0.0, 0.0];
//Alpha (radians)
const LINK_TWISTS : [f32;7] = [-PI_2, 0.0, -PI_2, PI_2, -PI_2, 0.0, 0.0];
//D (mm)
//NOTE: RobotStudio does not take tool length into account? will need to test this downstairs
const LINK_OFFSETS : [f32;7] = [800.0, 0.0, 0.0, 1520.0, 0.0, 200.0, 000.0];

//7 LINKS - (6 joints = end effector)
const NUM_OF_LINKS: i32 = 7;


//Robot kinematic model
pub struct IRB6400Model {

    //Theta (radians)
    joint_twists : [f32;6],
    //Theta offsets (radians) - stored dynamically because 3rd joint changes
    joint_offsets : [f32;6],

    t_matrices : [Matrix4<f32>; 7],
    //The final transformation matrix
    dh_trans: Matrix4<f32>
}

impl IRB6400Model{

    pub fn create_model() -> IRB6400Model{



        //No joint twist yet so we can predefine this
        let joint_offsets = [0.0, -PI_2, 0.0, 0.0, 0.0, 0.0];

        //Home value
        let joint_twists: [f32;6] = [0.0, joint_offsets[1], 0.0, 0.0, 30.0_f32.to_radians(), 0.0];


        let t_matrices = calc_t_matrices(joint_twists);

        let dh_trans = t_matrices[6];


        IRB6400Model {
            joint_offsets,
            joint_twists,
            t_matrices,
            dh_trans
        }
    }


    //Calculate an updated transform
    fn calc_transform(&mut self) {

        self.t_matrices = calc_t_matrices(self.joint_twists);

        self.dh_trans = self.t_matrices[6];
    }


    //Takes new joint angles (in radians)
    pub fn update_joints(&mut self, new_angles : [f32;6]){

        //Update the joints and the joint offsets
        for (jnt, angle) in new_angles.iter().enumerate(){
            if jnt == 1 {
                self.joint_twists[jnt] = *angle + self.joint_offsets[1];
                self.joint_offsets[2] = -*angle;
            }else if jnt == 2 {
                self.joint_twists[jnt] = *angle + self.joint_offsets[2];
                println!("JOINT {}",  self.joint_twists[jnt]);
            }else{
                self.joint_twists[jnt] = *angle;
            }

        }
        self.calc_transform();
    }

    //Return the transform to the end effecto
    pub fn get_transform(&self) -> Matrix4<f32>{
        self.dh_trans
    }

    //Calculate the jacobian of the transformation (i.e. the matrix that relates joint speed to end-effector speed)
    //This simplified version assumes that the end effector is an extension of the end plate of the robot (i.e. jacobian is 6x6 as such is computationally easier to invert)
    //Also assumes that the end effector is completely axially symmetrical (a sphere)
    //I.e. This is the jacobian of the first 6 joints in the robot.
    pub fn calc_simple_jacobian(&self) -> Matrix6<f32>{


        //The jacobian is constructed from the joint axis of rotation and the translated origin for the joint
        let mut jacobian: Matrix6<f32> = Matrix6::zeros();

        let o_6 = na::Vector3::new(self.t_matrices[5].m14, self.t_matrices[5].m24, self.t_matrices[5].m34);

        //Exclude the final link
        for (index, mut col) in jacobian.column_iter_mut().enumerate(){
            println!("{}", self.t_matrices[index]);
            let z_i;
            let o_i;

            //First frame is the base frame (reference frame)
            if index == 0{
                z_i = na::Vector3::new(0.0, 0.0, 1.0);
                o_i = na::Vector3::new(0.0, 0.0, 0.0);
            }else {
                z_i = na::Vector3::new(self.t_matrices[index - 1].m13, self.t_matrices[index - 1].m23, self.t_matrices[index - 1].m33);


                o_i = na::Vector3::new(self.t_matrices[index - 1].m14, self.t_matrices[index - 1].m24, self.t_matrices[index - 1].m34);
            }


            let j_v = z_i.cross(&(o_6 - o_i));


            col.x = j_v.x;
            col.y = j_v.y;
            col.z = j_v.z;
            col.w = z_i.x;
            col.a = z_i.y;
            col.b = z_i.z;
        }



        eprint!("{}", jacobian);

        jacobian

    }

    //Attemps to inverts the simplified jacobian
    pub fn get_inv_simple_jacobian(&self, simple : bool) -> Option<Matrix6<f32>>{

        let jacobian = self.calc_simple_jacobian();

        jacobian.try_inverse()
    }


}




fn calc_t_matrices(joint_twists : [f32;6]) -> [Matrix4<f32>; 7]{

    let mut t_matrices : [Matrix4<f32>;7] = [Matrix4::zeros(), Matrix4::zeros(), Matrix4::zeros(),
                          Matrix4::zeros(), Matrix4::zeros(), Matrix4::zeros(), Matrix4::zeros()];


    //Iterate through the transformation
    for i in 0..NUM_OF_LINKS {

        let index = i as usize;

        let l_length = LINK_LENGTHS[index];
        let c_ltwist = LINK_TWISTS[index].cos();
        let s_ltwist = LINK_TWISTS[index].sin();

        //If the transform matrix is a joint
        if index < (NUM_OF_LINKS as usize - 1) {
            let c_jtwist = joint_twists[index].cos();
            let s_jtwist = joint_twists[index].sin();


            if i == 0 {
                //Calculate the base transformation matrix
                t_matrices[index] = Matrix4::new(c_jtwist, -s_jtwist * c_ltwist, s_jtwist * s_ltwist, l_length * c_jtwist,
                                          s_jtwist, c_jtwist * c_ltwist, -c_jtwist * s_ltwist, l_length * s_jtwist,
                                          0.0, s_ltwist, c_ltwist, LINK_OFFSETS[index],
                                          0.0, 0.0, 0.0, 1.0);
            } else {
                t_matrices[index] = t_matrices[index-1] * Matrix4::new(c_jtwist, -s_jtwist * c_ltwist, s_jtwist * s_ltwist, l_length * c_jtwist,
                                                       s_jtwist, c_jtwist * c_ltwist, -c_jtwist * s_ltwist, l_length * s_jtwist,
                                                       0.0, s_ltwist, c_ltwist, LINK_OFFSETS[index],
                                                       0.0, 0.0, 0.0, 1.0);
            }
        }else{
            //No joint angles - just link info
            t_matrices[index] = t_matrices[index-1] * Matrix4::new(1.0, 0.0, 0.0, l_length,
                                                   0.0, 1. * c_ltwist, 0.0, 0.0,
                                                   0.0, s_ltwist, c_ltwist, LINK_OFFSETS[index],
                                                   0.0, 0.0, 0.0, 1.0);
        }

    }

    t_matrices
}
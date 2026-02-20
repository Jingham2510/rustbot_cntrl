//Models the IRB 6400 robot
extern crate nalgebra as na;

use na::Matrix4;
use nalgebra::{Matrix6, Vector6};

const PI_2: f64 = std::f64::consts::FRAC_PI_2;
const PI: f64 = std::f64::consts::PI;
//A(mm) - CURRENT TOOL SPHERE!
const LINK_LENGTHS: [f64; 7] = [240.0, 1050.0, 225.0, 0.0, 0.0, 0.0, 0.0];
//Alpha (radians)
const LINK_TWISTS: [f64; 7] = [-PI_2, 0.0, -PI_2, PI_2, -PI_2, 0.0, 0.0];
//D (mm)
//NOTE: RobotStudio does not take tool length into account? will need to test this downstairs
const LINK_OFFSETS: [f64; 7] = [800.0, 0.0, 0.0, 1520.0, 0.0, 200.0, 000.0];

//7 LINKS - (6 joints = end effector)
const NUM_OF_LINKS: i32 = 7;

//Robot kinematic model
pub struct IRB6400Model {
    //Theta (radians)
    joint_twists: [f64; 6],
    //Theta offsets (radians) - stored dynamically because 3rd joint changes
    joint_offsets: [f64; 6],

    t_matrices: [Matrix4<f64>; 7],
    //The final transformation matrix
    dh_trans: Matrix4<f64>,
}

impl IRB6400Model {
    pub fn create_model() -> IRB6400Model {
        //No joint twist yet so we can predefine this
        let joint_offsets = [0.0, -PI_2, 0.0, 0.0, 0.0, 0.0];

        //Home value
        let joint_twists: [f64; 6] = [0.0, joint_offsets[1], 0.0, 0.0, 30.0_f64.to_radians(), 0.0];

        let t_matrices = calc_t_matrices(joint_twists);

        let dh_trans = t_matrices[6];

        IRB6400Model {
            joint_offsets,
            joint_twists,
            t_matrices,
            dh_trans,
        }
    }

    //Calculate an updated transform
    fn calc_transform(&mut self) {
        self.t_matrices = calc_t_matrices(self.joint_twists);

        self.dh_trans = self.t_matrices[6];
    }

    //Takes new joint angles (in radians)
    pub fn update_joints(&mut self, new_angles: [f64; 6]) {
        //Update the joints and the joint offsets
        for (jnt, angle) in new_angles.iter().enumerate() {
            if jnt == 1 {
                self.joint_twists[jnt] = *angle + self.joint_offsets[1];
                self.joint_offsets[2] = -*angle;
            } else if jnt == 2 {
                self.joint_twists[jnt] = *angle + self.joint_offsets[2];
            } else {
                self.joint_twists[jnt] = *angle;
            }
        }
        self.calc_transform();
    }

    //Return the transform to the end effecto
    pub fn get_transform(&self) -> Matrix4<f64> {
        self.dh_trans
    }

    //Calculate the jacobian of the transformation (i.e. the matrix that relates joint speed to end-effector speed)
    //This simplified version assumes that the end effector is an extension of the end plate of the robot (i.e. jacobian is 6x6 as such is computationally easier to invert)
    //Also assumes that the end effector is completely axially symmetrical (a sphere)
    //I.e. This is the jacobian of the first 6 joints in the robot.
    pub fn calc_simple_jacobian(&self) -> Matrix6<f64> {
        //The jacobian is constructed from the joint axis of rotation and the translated origin for the joint
        let mut jacobian: Matrix6<f64> = Matrix6::zeros();

        let o_6 = na::Vector3::new(
            self.t_matrices[5].m14,
            self.t_matrices[5].m24,
            self.t_matrices[5].m34,
        );

        //Exclude the final link
        for (index, mut col) in jacobian.column_iter_mut().enumerate() {
            let z_i;
            let o_i;

            //First frame is the base frame (reference frame)
            if index == 0 {
                z_i = na::Vector3::new(0.0, 0.0, 1.0);
                o_i = na::Vector3::new(0.0, 0.0, 0.0);
            } else {
                z_i = na::Vector3::new(
                    self.t_matrices[index - 1].m13,
                    self.t_matrices[index - 1].m23,
                    self.t_matrices[index - 1].m33,
                );

                o_i = na::Vector3::new(
                    self.t_matrices[index - 1].m14,
                    self.t_matrices[index - 1].m24,
                    self.t_matrices[index - 1].m34,
                );
            }

            let j_v = z_i.cross(&(o_6 - o_i));

            //BOOK WAY
            col.x = j_v.x;
            col.y = j_v.y;
            col.z = j_v.z;
            col.w = z_i.x;
            col.a = z_i.y;
            col.b = z_i.z;

            //MATLAB WAY - opposite to Spongs book

            /*
            col.w = j_v.x;
            col.a = j_v.y;
            col.b = j_v.z;
            col.x = z_i.x;
            col.y = z_i.y;
            col.z = z_i.z;
            */
        }

        jacobian
    }

    //Attemps to inverts the simplified jacobian
    pub fn get_inv_simple_jacobian(&self) -> Option<Matrix6<f64>> {
        let jacobian = self.calc_simple_jacobian();

        jacobian.try_inverse()
    }

    //Check if the robot is at a singularity (loss of motion possibilities)
    fn check_singularity(&self) -> bool {
        //Theory shows that if the determinant of jacobian is 0 - the robot is at a singularity
        let det_j = self.calc_simple_jacobian().determinant();

        det_j == 0.0
    }

    //Get the raw joint values (w/o offset rotation)
    pub fn get_raw_joints(&self) -> [f64; 6] {
        [
            self.joint_twists[0],
            self.joint_twists[1] - self.joint_offsets[1],
            self.joint_twists[2] - self.joint_offsets[2],
            self.joint_twists[3],
            self.joint_twists[4],
            self.joint_twists[5],
        ]
    }

    //Return the raw joints in degrees
    pub fn get_raw_joints_as_degs(&self) -> [f64; 6] {
        [
            self.joint_twists[0].to_degrees(),
            (self.joint_twists[1] - self.joint_offsets[1]).to_degrees(),
            (self.joint_twists[2] - self.joint_offsets[2]).to_degrees(),
            self.joint_twists[3].to_degrees(),
            self.joint_twists[4].to_degrees(),
            self.joint_twists[5].to_degrees(),
        ]
    }

    //Moves the current joints based on a joint speed and an amount of time passed (rad/s and seconds)
    pub fn move_joints(&mut self, joint_speed: [f64; 6], time_secs: f64) {
        //Get the new joint positions - from raw joints (no offsets)
        let new_joints = [
            self.joint_twists[0] + (joint_speed[0] * time_secs),
            (self.joint_twists[1] - self.joint_offsets[1]) + (joint_speed[1] * time_secs),
            (self.joint_twists[2] - self.joint_offsets[2]) + (joint_speed[2] * time_secs),
            self.joint_twists[3] + (joint_speed[3] * time_secs),
            self.joint_twists[4] + (joint_speed[4] * time_secs),
            self.joint_twists[5] + (joint_speed[5] * time_secs),
        ];

        self.update_joints(new_joints);
    }

    //Calculates and returns the required joint speed (rad/s) for a given desired end effector linear speed (mm/s)
    pub fn get_joint_speed(&self, des_end_eff_speed: (f64, f64, f64)) -> [f64; 6] {
        //Assume no rotational speed required
        //Multiply the inverse jacobian by the desried end_effector speed
        let j_speed_vec = self.get_inv_simple_jacobian().unwrap()
            * Vector6::new(
                des_end_eff_speed.0,
                des_end_eff_speed.1,
                des_end_eff_speed.2,
                0.0,
                0.0,
                0.0,
            );

        //These are the joint speeds required (transposed)
        [
            j_speed_vec.x,
            j_speed_vec.y,
            j_speed_vec.z,
            j_speed_vec.w,
            j_speed_vec.a,
            j_speed_vec.b,
        ]
    }
}

fn calc_t_matrices(joint_twists: [f64; 6]) -> [Matrix4<f64>; 7] {
    let mut t_matrices: [Matrix4<f64>; 7] = [
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
        Matrix4::zeros(),
    ];

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
                t_matrices[index] = Matrix4::new(
                    c_jtwist,
                    -s_jtwist * c_ltwist,
                    s_jtwist * s_ltwist,
                    l_length * c_jtwist,
                    s_jtwist,
                    c_jtwist * c_ltwist,
                    -c_jtwist * s_ltwist,
                    l_length * s_jtwist,
                    0.0,
                    s_ltwist,
                    c_ltwist,
                    LINK_OFFSETS[index],
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                );
            } else {
                t_matrices[index] = t_matrices[index - 1]
                    * Matrix4::new(
                        c_jtwist,
                        -s_jtwist * c_ltwist,
                        s_jtwist * s_ltwist,
                        l_length * c_jtwist,
                        s_jtwist,
                        c_jtwist * c_ltwist,
                        -c_jtwist * s_ltwist,
                        l_length * s_jtwist,
                        0.0,
                        s_ltwist,
                        c_ltwist,
                        LINK_OFFSETS[index],
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    );
            }
        } else {
            //No joint angles - just link info
            t_matrices[index] = t_matrices[index - 1]
                * Matrix4::new(
                    1.0,
                    0.0,
                    0.0,
                    l_length,
                    0.0,
                    1. * c_ltwist,
                    0.0,
                    0.0,
                    0.0,
                    s_ltwist,
                    c_ltwist,
                    LINK_OFFSETS[index],
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                );
        }
    }

    t_matrices
}

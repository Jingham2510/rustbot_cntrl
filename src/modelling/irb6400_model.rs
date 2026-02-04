//Models the IRB 6400 robot
extern crate nalgebra as na;

use na::Matrix4;

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

    //The final transformation matrix
    dh_trans: Matrix4<f32>,
}

impl IRB6400Model{

    pub fn create_model() -> IRB6400Model{



        //No joint twist yet so we can predefine this
        let joint_offsets = [0.0, -PI_2, 0.0, 0.0, 0.0, 0.0];

        //Home value
        let joint_twists: [f32;6] = [0.0, joint_offsets[1], 0.0, 0.0, 30.0_f32.to_radians(), 0.0];



        let dh_trans = calc_inital_transform(joint_twists);


        IRB6400Model {
            joint_offsets,
            joint_twists,
            dh_trans

        }
    }


    //Calculate an updated transform
    fn calc_transform(&mut self) {
        self.dh_trans = calc_inital_transform(self.joint_twists);
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

    //Return the transform
    pub fn get_transform(&self) -> Matrix4<f32>{
        self.dh_trans
    }


}


fn calc_inital_transform(joint_twists : [f32;6]) -> Matrix4<f32>{

    let mut curr_trans : Matrix4<f32> = Matrix4::zeros();


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
                curr_trans = Matrix4::new(c_jtwist, -s_jtwist * c_ltwist, s_jtwist * s_ltwist, l_length * c_jtwist,
                                          s_jtwist, c_jtwist * c_ltwist, -c_jtwist * s_ltwist, l_length * s_jtwist,
                                          0.0, s_ltwist, c_ltwist, LINK_OFFSETS[index],
                                          0.0, 0.0, 0.0, 1.0);
            } else {
                curr_trans = curr_trans * Matrix4::new(c_jtwist, -s_jtwist * c_ltwist, s_jtwist * s_ltwist, l_length * c_jtwist,
                                                       s_jtwist, c_jtwist * c_ltwist, -c_jtwist * s_ltwist, l_length * s_jtwist,
                                                       0.0, s_ltwist, c_ltwist, LINK_OFFSETS[index],
                                                       0.0, 0.0, 0.0, 1.0);
            }
        }else{
            //No joint angles - just link info
            curr_trans = curr_trans * Matrix4::new(1.0, 0.0, 0.0, l_length,
                                                   0.0, 1. * c_ltwist, 0.0, 0.0,
                                                   0.0, s_ltwist, c_ltwist, LINK_OFFSETS[index],
                                                   0.0, 0.0, 0.0, 1.0);
        }

        eprintln!("{}", curr_trans);


    }

    curr_trans





}
///A collection of controller implementations used for force control
///NOTE - all control functions must have a footprint of fn function_name(err: f32)->Result<f32, anyhow::Error>
///This is to comply with the callback function used in the main test procedure
use chrono;
use chrono::{DateTime, Local};
use std::fmt::Display;
use tch::{nn, nn::Module, nn::OptimizerConfig, Device, Tensor};


///Provides a constant step up/down based only on the polarity of the error
pub fn polarity_step_control(err: f32) -> Result<f32, anyhow::Error> {
    //How far the step should be
    const STEP_VALUE: f32 = 1.0;

    //Check the polarity of the error
    if err < 0.0 {
        Ok(STEP_VALUE)
    } else if err > 0.0 {
        Ok(-STEP_VALUE)
    } else {
        Ok(0.0)
    }
}

//a proportional gain controller based on error magnitude
pub fn prop_gain_control(err: f32) -> Result<f32, anyhow::Error> {
    //How far the step should be if the error is a value of 1N
    const BASE_STEP: f32 = 0.05;

    Ok(BASE_STEP * -err)
}

///A Proportional derivative controller
pub struct PDController {
    ///The error
    prev_err: f64,
    ///The time when the error was reported
    prev_time: DateTime<Local>,
    ///Proportional gain
    kp_gain: f64,
    ///Derivative gain
    kd_gain: f64,
}

impl PDController {
    #![allow(nonstandard_style)]
    ///Create a PD controller
    pub fn create_PD(KP_gain: f64, KD_gain: f64) -> PDController {
        PDController {
            prev_err: 0.0,
            prev_time: chrono::offset::Local::now(),
            kp_gain: KP_gain,
            kd_gain: KD_gain,
        }
    }

    ///Calculate the output of the controller
    pub fn calc_op(&mut self, err: f64) -> Result<f64, anyhow::Error> {
        //Get current time
        let now = chrono::offset::Local::now();

        //Calculate the derivative of the current slope
        let derr = (err - self.prev_err) / ((now - self.prev_time).as_seconds_f64());

        //Update the previous values
        self.prev_time = now;
        self.prev_err = err;

        Ok(self.kp_gain * -err + self.kd_gain * -derr)
    }
}

///A PID controller
pub struct PIDController {
    ///Error history
    errs: Vec<f64>,
    ///Timestamps of the errors
    timestamps: Vec<DateTime<Local>>,
    ///The current calculated integral error
    curr_integral: f64,
    ///Proportional gain
    kp_gain: f64,
    ///Integral gain
    ki_gain: f64,
    ///Derivative gain
    kd_gain: f64,
}

impl Display for PIDController {
    ///Display the controller setup
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let string = format!(
            "PID - P:{},I:{},D:{}",
            self.kp_gain, self.ki_gain, self.kd_gain
        );

        write!(f, "{}", string)
    }
}

impl PIDController {
    #![allow(nonstandard_style)]
    ///Create a PID controller
    pub fn create_PID(KP_gain: f64, KI_gain: f64, KD_gain: f64) -> PIDController {
        PIDController {
            errs: vec![0.0],
            timestamps: vec![Local::now()],
            curr_integral: 0.0,
            kp_gain: KP_gain,
            ki_gain: KI_gain,
            kd_gain: KD_gain,
        }
    }

    ///Calulcate the output of the controller
    pub fn calc_op(&mut self, err: f64) -> Result<f64, anyhow::Error> {
        self.timestamps.push(Local::now());
        self.errs.push(err);

        //Calculate the derivative in seconds
        let derr = (err - self.errs[self.errs.len() - 2])
            / ((self.timestamps[self.timestamps.len() - 1]
                - self.timestamps[self.timestamps.len() - 2])
                .as_seconds_f64());

        //Calculate the integral 
        let ierr = self.calc_integral_reimann_approx();

        //println!("KP - {}, KI - {}, KD - {}", err, ierr, derr);

        let move_dist = (self.kp_gain * err) + (self.ki_gain * ierr) + (self.kd_gain * derr);

        //println!("dist to move - {}", move_dist);

        Ok(move_dist)
    }

    ///Approximate the integral area using the trapezium approximation
    fn calc_integral_reimann_approx(&mut self) -> f64 {
        let curr_err = self.errs[self.errs.len() - 1];
        let prev_err = self.errs[self.errs.len() - 2];
        let curr_time = self.timestamps[self.timestamps.len() - 1].timestamp();
        let prev_time = self.timestamps[self.timestamps.len() - 2].timestamp();
        let time_delta = curr_time - prev_time;

        //Check the magnitudes of the errors 
        let new_area =
            if curr_err != -prev_err {
                (time_delta as f64) * ((prev_err + curr_err) / 2.0)
            } else {
                0.0
            };

        self.curr_integral += new_area;
        self.curr_integral
    }

    ///Update the current gain values of the controller
    pub fn update_gains(&mut self, prop_gain : f64, int_gain : f64, deri_gain : f64){

        self.kp_gain = prop_gain;
        self.ki_gain = int_gain;
        self.kd_gain = deri_gain;

    }

}



///EXPERIMENTAL - Self tuning PID controller
pub struct PIDWithNNTuner{
    controller : PIDController,
    net_tuner : nn::Sequential,
    opt : nn::Optimizer
}


impl PIDWithNNTuner{
    ///Create the pid controller and tuning network with initial values
    pub fn create(KP_gain : f64, KI_gain : f64, KD_gain : f64, no_of_hidden_layers : u32) -> Self{

        //check if cuda can be used
        let vs = nn::VarStore::new(Device::cuda_if_available());
        const IN : i64 = 4;
        const HIDDEN_NODES : i64 = 128;
        const OUT : i64 = 3;

        //Create the 
        let net_tuner: nn::Sequential = {
            //Create the network and add the input layer
            let net = nn::seq().add(nn::linear(&vs.root() / "layer1", 4, HIDDEN_NODES, Default::default())).add_fn(|xs| xs.relu());


            //Add the hidden layers
            for i in 0..no_of_hidden_layers{

                if i == no_of_hidden_layers - 1{
                     &net.add(nn::linear(&vs.root(), HIDDEN_NODES, OUT , Default::default())).add_fn(|xs| xs.relu());
                }else{
                     &net.add(nn::linear(&vs.root(), HIDDEN_NODES, HIDDEN_NODES , Default::default())).add_fn(|xs| xs.relu());
                }
            };          

            net 
            

        };        

        let opt = nn::Adam::default().build(&vs, 1e-3).unwrap();

        PIDWithNNTuner { controller: PIDController::create_PID(KP_gain, KI_gain, KD_gain) , net_tuner, opt}

    }

    ///Calculate the output from the PID and tune the parameters at the same time
    pub fn calc_op_and_tune(&mut self, err : f64, target : f64){
        //Update the neural net
        let net_out = self.net_tuner.forward(&Tensor::from_slice(&[self.controller.kp_gain, self.controller.ki_gain, self.controller.kd_gain, err]));      

        //Optimise the PID 
        self.opt.backward_step(&Tensor::from(err));      

        println!("Loss: {}", err);

        //Update the PID values
        self.controller.update_gains(<f64>::from(net_out.i(0)), net_out.i(1), net_out.i(2));

        //Calculate the output
        self.controller.calc_op(err);
    }
    ///Calculate the output without running a tuning step
    pub fn calc_op(&mut self, err : f64){
        self.controller.calc_op(err);
    }


}
///A collection of controller implementations used for force control
///NOTE - all control functions must have a footprint of fn function_name(err: f32)->Result<f32, anyhow::Error>
///This is to comply with the callback function used in the main test procedure
use chrono;
use chrono::{DateTime, Local};
use std::fmt::Display;
use tch::{Tensor, nn ,Device, nn::OptimizerConfig, nn::Module};


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








///EXPERIMENTAL - Self tuning PID controller using a neural network
pub struct PIDWithNNTuner{
    controller : PIDController,
    net_tuner : nn::Sequential,
    opt : nn::Optimizer,
    loss : Tensor,
    prev_err : f64,
    prev_u : f64,
}


impl PIDWithNNTuner{
    ///Create the pid controller and tuning network with initial values
    pub fn create(KP_gain : f64, KI_gain : f64, KD_gain : f64) -> Self{

        println!("Creating controller with NN tuner");

        //check if cuda can be used and create the variable storage
        let vs = nn::VarStore::new(Device::cuda_if_available());


        const IN : i64 = 4;
        const HIDDEN_NODES : i64 = 128;
        const OUT : i64 = 3;

        //Create the neural network 4 -> 128 -> 128 -> 3 (all relu)
        let net_tuner : nn::Sequential = nn::seq()
            .add(nn::linear(&vs.root() / "layer1", IN, HIDDEN_NODES, Default::default())).add_fn(|xs| xs.relu())
            .add(nn::linear(&vs.root(), HIDDEN_NODES, HIDDEN_NODES/2 , Default::default())).add_fn(|xs| xs.relu())
            .add(nn::linear(&vs.root(), HIDDEN_NODES/2, HIDDEN_NODES/4 , Default::default())).add_fn(|xs| xs.relu())
            .add(nn::linear(&vs.root(), HIDDEN_NODES/4, OUT , Default::default())).add_fn(|xs| xs.relu());
                

        //Create the optimiser and link it to the vs
        let opt = nn::Adam::default().build(&vs, 1e-3).unwrap();

        //Add the "error" to the graph and turn on the gradient tracking
        let mut loss = vs.root().add("err", Tensor::from_slice(&[9999.0]), false);
        let loss = loss.requires_grad_(true);
        

        println!("Controller created");

        PIDWithNNTuner { controller: PIDController::create_PID(KP_gain, KI_gain, KD_gain) , net_tuner, opt,  loss, prev_err : 0.0, prev_u : 0.0}

    }

    ///Calculate the output from the PID and tune the parameters at the same time
    pub fn calc_op_and_tune(&mut self, err : f64 , err_hist : &[f64]) -> Result<f64, anyhow::Error> {

        //Calculate the output
        let prev_u = self.prev_u;
        let u = self.controller.calc_op(err)?;

        //Update the neural net
        let net_out = self.net_tuner.forward(&Tensor::from_slice(&[err as f32, self.prev_err as f32, u as f32, prev_u as f32]));        

        //Backpropogate the network using the past 100 error measurements
        let new_loss = Tensor::zeros(1, (tch::Kind::Double, Device::cuda_if_available()));
        //Take the square of every error and sum them
        let mut err_hist_sum : f64 = err_hist.iter().fold(0.0, |acc, x| acc + (x.powi(2)));
        //err_hist_sum += self.controller.kp_gain.powi(2) + self.controller.ki_gain.powi(2) + self.controller.kd_gain.powi(2);
        self.loss.set_data(&new_loss.fill( err_hist_sum/err_hist.len() as f64));
        self.opt.backward_step(&self.loss);

        //Update the PID values
        self.controller.update_gains(net_out.f_double_value(&[0]).unwrap(), net_out.f_double_value(&[1]).unwrap(), net_out.f_double_value(&[2]).unwrap());

        println!("Controller: {} - Loss: {}", self.controller, err_hist_sum);

        self.prev_err = err;

        return Ok(u)


    }
    ///Calculate the output without running a tuning step
    pub fn calc_op(&mut self, err : f64) -> Result<f64, anyhow::Error> {

        let u = self.controller.calc_op(err)?;
        
        self.prev_u = u;

        return Ok(u)
    }

    ///Get the controller object
    pub fn controller(&mut self) -> &PIDController{
        &self.controller
    }

    ///Update the gains
    pub fn update_gains(&mut self, prop_gain : f64, int_gain : f64, deri_gain : f64){

        self.controller.update_gains(prop_gain, int_gain, deri_gain);
    }


}
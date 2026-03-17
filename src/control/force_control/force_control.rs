///A collection of controller implementations used for force control
///NOTE - all control functions must have a footprint of fn function_name(err: f32)->Result<f32, anyhow::Error>
///This is to comply with the callback function used in the main test procedure
use chrono;
use chrono::{DateTime, Local};
use std::fmt::Display;

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

        //Calculate the derivative - also converting s to ms
        let derr = (err - self.errs[self.errs.len() - 2])
            / ((self.timestamps[self.timestamps.len() - 1]
                - self.timestamps[self.timestamps.len() - 2])
                .as_seconds_f64());

        //Calculate the integral (iteratively)
        self.calc_integral_trap_approx();
        let ierr = self.curr_integral;

        //println!("KP - {}, KI - {}, KD - {}", err, ierr, derr);

        let move_dist = (self.kp_gain * err) + (self.ki_gain * ierr) + (self.kd_gain * derr);

        //println!("dist to move - {}", move_dist);

        Ok(move_dist)
    }

    ///Approximate the integral area using the trapezium approximation
    fn calc_integral_trap_approx(&mut self) -> f64 {
        let new_area: f64;

        let curr_err = self.errs[self.errs.len() - 1];
        let prev_err = self.errs[self.errs.len() - 2];
        let curr_time = self.timestamps[self.timestamps.len() - 1].timestamp();
        let prev_time = self.timestamps[self.timestamps.len() - 2].timestamp();
        let time_delta = curr_time - prev_time;

        //Check the sign of the error and the previous error
        //BOTH ERRS POS
        if (curr_err >= 0.0) & (prev_err >= 0.0) {
            new_area = (time_delta as f64) * ((prev_err + curr_err) / 2.0)
        }
        //BOTH ERRS NEGATIVE
        else if (curr_err <= 0.0) & (prev_err <= 0.0) {
            new_area = (time_delta as f64) * ((prev_err + curr_err) / 2.0)
        } else {
            //Calc the midpoint (where it crosses the polarity approx)

            //WILL APPROXIMATE TO 0 HERE - if we are calculating the midpoint it will always have equal area on both sides
            new_area = 0.0;
        }

        self.curr_integral += new_area;
        self.curr_integral
    }
}

///Proportional Heaviside PID Controller (basically two PID controllers)
pub struct PHPIDController {
    ///Error history
    errs: Vec<f64>,
    ///Timestamp of the errors
    timestamps: Vec<DateTime<Local>>,
    ///The current integral value
    curr_integral: f64,
    ///Positive error proportional gain
    hi_kp_gain: f64,
    ///Positive error integral gain
    hi_ki_gain: f64,
    ///Positive error derivative gain
    hi_kd_gain: f64,
    ///Value that determines error polarity
    heaviside_val: f64,
    ///Low error proportional gain
    lo_kp_gain: f64,
    ///Low error integral gain
    lo_ki_gain: f64,
    ///Low error derivative gain
    lo_kd_gain: f64,
}

impl Display for PHPIDController {
    ///Print the PHPID information
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let string = format!(
            "PHPID - HP:{},HI:{},HD:{},LP:{},LI:{},LD:{}",
            self.hi_kp_gain,
            self.hi_ki_gain,
            self.hi_kd_gain,
            self.lo_kp_gain,
            self.lo_ki_gain,
            self.lo_kd_gain
        );

        write!(f, "{}", string)
    }
}

impl PHPIDController {
    #![allow(nonstandard_style)]
    ///Create a PHPID controller
    pub fn create_PHPID(
        Hi_KP_gain: f64,
        Hi_KI_gain: f64,
        Hi_KD_gain: f64,
        heaviside_val: f64,
        Lo_KP_gain: f64,
        Lo_KI_gain: f64,
        Lo_KD_gain: f64,
    ) -> PHPIDController {
        PHPIDController {
            errs: vec![0.0],
            timestamps: vec![Local::now()],
            curr_integral: 0.0,
            hi_kp_gain: Hi_KP_gain,
            hi_ki_gain: Hi_KI_gain,
            hi_kd_gain: Hi_KD_gain,
            heaviside_val,
            lo_kp_gain: Lo_KP_gain,
            lo_ki_gain: Lo_KI_gain,
            lo_kd_gain: Lo_KD_gain,
        }
    }

    ///Calculate the output of the controller
    pub fn calc_op(&mut self, err: f64) -> Result<f64, anyhow::Error> {
        self.timestamps.push(Local::now());
        self.errs.push(err);

        //Calculate the derivative - also converting s to ms
        let derr = (err - self.errs[self.errs.len() - 2])
            / ((self.timestamps[self.timestamps.len() - 1]
                - self.timestamps[self.timestamps.len() - 2])
                .as_seconds_f64());

        //Calculate the integral (iteratively)
        self.calc_integral_trap_approx();
        let ierr = self.curr_integral;

        //println!("KP - {}, KI - {}, KD - {}", err, ierr, derr);

        let move_dist = if err > self.heaviside_val {
            (self.hi_kp_gain * err) + (self.hi_ki_gain * ierr) + (self.hi_kd_gain * derr)
        } else {
            (self.lo_kp_gain * err) + (self.lo_ki_gain * ierr) + (self.lo_kd_gain * derr)
        };

        Ok(move_dist)
    }

    ///Approximate the integral area using the trapezium approximation
    fn calc_integral_trap_approx(&mut self) -> f64 {
        let new_area: f64;

        let curr_err = self.errs[self.errs.len() - 1];
        let prev_err = self.errs[self.errs.len() - 2];
        let curr_time = self.timestamps[self.timestamps.len() - 1].timestamp();
        let prev_time = self.timestamps[self.timestamps.len() - 2].timestamp();
        let time_delta = curr_time - prev_time;

        //Check the sign of the error and the previous error
        //BOTH ERRS POS
        if (curr_err >= 0.0) & (prev_err >= 0.0) {
            new_area = (time_delta as f64) * ((prev_err + curr_err) / 2.0)
        }
        //BOTH ERRS NEGATIVE
        else if (curr_err <= 0.0) & (prev_err <= 0.0) {
            new_area = (time_delta as f64) * ((prev_err + curr_err) / 2.0)
        } else {
            //Calc the midpoint (where it crosses the polarity approx)

            //WILL APPROXIMATE TO 0 HERE - if we are calculating the midpoint it will always have equal area on both sides
            new_area = 0.0;
        }

        self.curr_integral += new_area;
        self.curr_integral
    }
}

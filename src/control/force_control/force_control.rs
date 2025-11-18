use anyhow::bail;

use chrono;
use chrono::{DateTime, Local};
//NOTE - all control functions must have a footprint of fn function_name(err: f32)->Result<f32, anyhow::Error>
//This is to comply with the callback function used in the main test procedure


//Provides a constant step up/down based only on the polarity of the error
pub fn polarity_step_control(err :f32) -> Result<f32, anyhow::Error>{

    //How far the step should be
    const STEP_VALUE : f32 = 1.0;

    //Check the polarity of the error
    if err < 0.0 {
        Ok(STEP_VALUE)
    } else if err > 0.0 {
        Ok(-STEP_VALUE)
    } else {
        Ok(0.0)
    }
}



//Impleemtns a proportional gain controller based on error magnitude
pub fn prop_gain_control(err:f32) -> Result<f32, anyhow::Error>{

    //How far the step should be if the error is a value of 1N
    const BASE_STEP :f32 = 0.1;

    Ok(BASE_STEP * err)

}


struct PdController {
    prev_err : f32,
    prev_time : DateTime<Local>
}

impl PdController {

    pub fn calc_op(&mut self, err:f32) ->Result<f32, anyhow::Error>{

        //gain
        const KD_GAIN : f32 = 1.0;

        //Get current time
        let now = chrono::offset::Local::now();

        //Calculate the derivative of the current slope
        let derr = (err - self.prev_err)/((now - self.prev_time).as_seconds_f32());

        //Update the previous values
        self.prev_time = now;
        self.prev_err = err;

        Ok(derr)
    }

}
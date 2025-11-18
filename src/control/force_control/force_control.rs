use anyhow::bail;

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
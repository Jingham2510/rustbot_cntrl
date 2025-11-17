
//Provides a step up/down based only on the polarity of the error
pub fn polarity_step_control(err :f32) -> f32{

    const STEP_VALUE : f32 = 0.25;

    //Check the polarity of the error
    if err < 0.0{
        STEP_VALUE
    }
    else if err > 0.0{
        -STEP_VALUE
    }
    else
    {
        0.0
    }
}
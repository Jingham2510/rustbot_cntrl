///This file contains functions required for creation of force function generators
use anyhow::bail;
use std::fs::OpenOptions;
use std::io::{Write, stdin};

///Types of signal that can be generated
#[derive(Debug)]
enum SignalType {
    ///Held at one value
    Constant,
    ///Change between two forces
    Step,
    ///Ramp between two values at a constant rate
    Ramp,
    ///Ramp up to a peak value then ramp back down
    Triangle,
    ///Step up to a value then step down
    StepUpDown,
    ///Custom function based on user input
    Custom,
}

///Function specification
pub struct ForceFunctionGenerator {
    ///The type of signal form
    sig_type: SignalType,
    ///The total number of changes
    force_changes: usize,
    ///The values desired in the signal (in chronological order)
    sig_vals: Vec<f64>,
    ///The percentage time that each force is desired for (in chronological order)
    sig_time: Vec<f64>,
}

impl ForceFunctionGenerator {
    pub fn user_interface() -> Result<Self, anyhow::Error> {
        loop {
            println!("Select a force type:");

            //Get user input
            let mut user_inp = String::new();
            stdin()
                .read_line(&mut user_inp)
                .expect("Failed to read line");

            match user_inp.to_lowercase().trim() {
                "ramp" | "step" | "triangle" | "stepupdown" => {
                    let min_force: f64;
                    let max_force: f64;

                    println!("Type the start value");

                    //Get user input
                    let mut val_sel = String::new();
                    stdin()
                        .read_line(&mut val_sel)
                        .expect("Failed to read line");

                    if let Ok(val) = val_sel.trim().parse::<f64>() {
                        min_force = val;
                    } else {
                        println!("Invalid value");
                        continue;
                    }

                    println!("Type the end value");

                    //Get user input
                    let mut val_sel = String::new();
                    stdin()
                        .read_line(&mut val_sel)
                        .expect("Failed to read line");

                    if let Ok(val) = val_sel.trim().parse::<f64>() {
                        max_force = val;
                    } else {
                        println!("Invalid value");
                        continue;
                    }

                    if user_inp.to_lowercase().trim() == "ramp" {
                        return ForceFunctionGenerator::ramp_force(min_force, max_force);
                    }else if user_inp.to_lowercase().trim() == "triangle"{
                        return ForceFunctionGenerator::triangle_force(min_force, max_force)
                    }                    
                     else {
                        println!("Type how many steps are required");
                        //Get user input
                        let mut val_sel = String::new();
                        stdin()
                            .read_line(&mut val_sel)
                            .expect("Failed to read line");

                        if let Ok(val) = val_sel.trim().parse::<usize>(){
                            if user_inp.to_lowercase().trim() == "step"{
                                return ForceFunctionGenerator::step_force(min_force, max_force, val);
                            }else{
                                return ForceFunctionGenerator::step_up_down_force(min_force, max_force, val);
                            }
                        } else {
                            println!("Invalid value");
                            continue;
                        }
                    }
                }

                "constant" => {
                    println!("Type the desired force");

                    //Get user input
                    let mut val_sel = String::new();
                    stdin()
                        .read_line(&mut val_sel)
                        .expect("Failed to read line");

                    if let Ok(val) = val_sel.trim().parse::<f64>() {
                        return ForceFunctionGenerator::constant_force(val);
                    } else {
                        println!("Invalid value");
                        continue;
                    }
                }

                _ => {
                    println!("Invalid choice")
                }
            }
        }
    }

    ///Create a force function with a constant value
    fn constant_force(desired_force: f64) -> Result<Self, anyhow::Error> {
        Ok(ForceFunctionGenerator {
            sig_type: SignalType::Constant,
            force_changes: 0,
            sig_vals: vec![desired_force],
            sig_time: vec![100.0],
        })
    }

    ///Create a force function with a stepped value between two values
    fn step_force(
        min_force: f64,
        max_force: f64,
        no_of_steps: usize,
    ) -> Result<Self, anyhow::Error> {
        //Ensure valid force profile
        if no_of_steps == 0 || min_force == max_force {
            println!("Too few steps! Generating constant force");
            return ForceFunctionGenerator::constant_force(min_force);
        }
        if no_of_steps >= 100000 {
            println!("Too many steps! Generating ramp");
            return ForceFunctionGenerator::ramp_force(min_force, max_force);
        }

        let mut sig_vals: Vec<f64> = vec![];
        let mut sig_time: Vec<f64> = vec![];

        let step_size = (max_force - min_force) / (no_of_steps as f64);

        for i in 0..=no_of_steps {
            let force_val = min_force + i as f64 * step_size;

            sig_vals.push(force_val);

            sig_time.push(100.0 / (1.0 + no_of_steps as f64));
        }

        verify_time_constraint(&sig_time)?;

        Ok(ForceFunctionGenerator {
            sig_type: SignalType::Step,
            force_changes: no_of_steps,
            sig_vals,
            sig_time,
        })
    }

     ///Create a force function with a stepped value between two values
    fn step_up_down_force(
        min_force: f64,
        max_force: f64,
        no_of_steps: usize,
    ) -> Result<Self, anyhow::Error> {
        //Ensure valid force profile
        if no_of_steps == 0 || min_force == max_force {
            println!("Too few steps! Generating constant force");
            return ForceFunctionGenerator::constant_force(min_force);
        }
        if no_of_steps >= 100000 {
            println!("Too many steps! Generating ramp");
            return ForceFunctionGenerator::ramp_force(min_force, max_force);
        }

        let mut sig_vals: Vec<f64> = vec![];
        let mut sig_time: Vec<f64> = vec![];

        let step_size = (max_force - min_force) / (2.0 * no_of_steps as f64);

        for i in 0..=no_of_steps {
            let force_val = min_force + i as f64 * step_size;

            sig_vals.push(force_val);

            sig_time.push(100.0 / (1.0 + no_of_steps as f64));
        }

        for i in 0..=no_of_steps {
            let force_val = max_force - i as f64 * step_size;

            sig_vals.push(force_val);

            sig_time.push(100.0 / (1.0 + no_of_steps as f64));
        }

        verify_time_constraint(&sig_time)?;

        Ok(ForceFunctionGenerator {
            sig_type: SignalType::StepUpDown,
            force_changes: no_of_steps,
            sig_vals,
            sig_time,
        })
    }


    ///Create a force function that ramps between two values
    ///Essentially a very fine step function
    fn ramp_force(min_force: f64, max_force: f64) -> Result<Self, anyhow::Error> {
        if min_force == max_force {
            println!("No change! Generating constant force");
            return ForceFunctionGenerator::constant_force(min_force);
        }

        const RAMP_VAR: usize = 100000;

        let mut sig_vals: Vec<f64> = vec![];
        let mut sig_time: Vec<f64> = vec![];

        let step_size = (max_force - min_force) / (RAMP_VAR as f64);

        for i in 0..RAMP_VAR {
            let force_val = min_force + (i as f64 * step_size);

            sig_vals.push(force_val);

            sig_time.push(100.0 / RAMP_VAR as f64);
        }

        verify_time_constraint(&sig_time)?;

        Ok(ForceFunctionGenerator {
            sig_type: SignalType::Ramp,
            force_changes: RAMP_VAR,
            sig_vals,
            sig_time,
        })
    }

    ///Create a force function that ramps up to a peak then ramps down
    fn triangle_force(min_force: f64, max_force : f64) -> Result<Self, anyhow::Error>{ 
        if min_force == max_force {
            println!("No change! Generating constant force");
            return ForceFunctionGenerator::constant_force(min_force);
        }

        const RAMP_VAR: usize = 100000;

        let mut sig_vals: Vec<f64> = vec![];
        let mut sig_time: Vec<f64> = vec![];

        let step_size = (max_force - min_force) / (RAMP_VAR as f64);

        for i in 0..RAMP_VAR/2 {
            let force_val = min_force + (i as f64 * step_size);

            sig_vals.push(force_val);

            sig_time.push(100.0 / RAMP_VAR as f64);
        }

        for i in 0..RAMP_VAR/2 {
            let force_val = max_force - (i as f64 * step_size);

            sig_vals.push(force_val);

            sig_time.push(100.0 / RAMP_VAR as f64);
        }

        verify_time_constraint(&sig_time)?;

        Ok(ForceFunctionGenerator {
            sig_type: SignalType::Triangle,
            force_changes: RAMP_VAR,
            sig_vals,
            sig_time,
        })


    }

    ///Create a force function that follows a custom pattern
    pub fn custom_force(sig_vals: Vec<f64>, sig_time: Vec<f64>) -> Result<Self, anyhow::Error> {
        verify_time_constraint(&sig_time)?;

        Ok(ForceFunctionGenerator {
            sig_type: SignalType::Custom,
            force_changes: sig_vals.len(),
            sig_vals,
            sig_time,
        })
    }

    ///Converts the relative percentage times to timestamps to compare against
    ///They are additive i.e. two one second blocks will display as [1.0, 2.0]
    pub fn as_time_f64(&self, total_time: f64) -> Vec<f64> {
        let mut timestamp_vec: Vec<f64> = vec![];

        let mut curr_time = 0.0;

        for perc_time in self.sig_time.iter() {
            //Update the current time
            curr_time += total_time * (perc_time / 100.0);

            timestamp_vec.push(curr_time);
        }

        timestamp_vec
    }

    pub fn save_to_file(&self, filepath: &str) -> Result<(), anyhow::Error> {
        //Open the file (or create if it doesn't exist)
        let mut file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(filepath.trim())
            .unwrap();

        let line: String = "Desired force profile".to_string();

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line) {
            bail!("Couldn't write to file: {}", e);
        }

        //See whether to transofmr the data by the
        let line: String =
            //Format the line to write
            format!(
                "TYPE:{:?}|VAL_CNT:{}|VALS:{:?}|PERC_TIMES:{:?}",
                self.sig_type,
                self.force_changes,
                self.sig_vals,
                self.sig_time
            );

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line) {
            bail!("Couldn't write to file: {}", e);
        }

        Ok(())
    }

    pub fn force_changes(&self) -> usize {
        self.force_changes
    }

    ///Get the signal values
    pub fn sig_vals(&self) -> Vec<f64> {
        self.sig_vals.clone()
    }

    ///Get the signal times
    pub fn sig_time(&self) -> Vec<f64> {
        self.sig_time.clone()
    }
}

///Verify that the signal time adds up to 100%
fn verify_time_constraint(sig_time: &[f64]) -> Result<(), anyhow::Error> {
    let mut total = 0.0;

    for i in sig_time.iter() {
        total += i;
    }

    if total.round() == 100.0 {
        Ok(())
    } else {
        bail!("Invalid time constraints")
    }
}

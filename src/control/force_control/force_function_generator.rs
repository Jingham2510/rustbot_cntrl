use anyhow::{Error, bail};
use std::fmt;
use std::fs::OpenOptions;
use std::io::Write;
///This file contains functions required for creation of force function generators

///Types of signal that can be generated

#[derive(Debug)]
enum SignalType {
    ///Held at one value
    CONSTANT,
    ///Change between two forces
    STEP,
    ///Ramp between two values at a constant rate
    RAMP,
    ///Custom function based on user input
    CUSTOM,
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
    ///Create a force function with a constant value
    pub fn constant_force(desired_force: f64) -> Result<Self, anyhow::Error> {
        Ok(ForceFunctionGenerator {
            sig_type: SignalType::CONSTANT,
            force_changes: 0,
            sig_vals: vec![desired_force],
            sig_time: vec![100.0],
        })
    }

    ///Create a force function with a stepped value between two values
    pub fn step_force(
        min_force: f64,
        max_force: f64,
        no_of_steps: usize,
    ) -> Result<Self, anyhow::Error> {
        //Ensure valid force profile
        if no_of_steps == 0 {
            return ForceFunctionGenerator::constant_force(min_force);
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
            sig_type: SignalType::STEP,
            force_changes: no_of_steps,
            sig_vals,
            sig_time,
        })
    }

    ///Create a force function that ramps between two values
    ///Essentially a very fine step function
    pub fn ramp_force(min_force: f64, max_force: f64) -> Result<Self, anyhow::Error> {
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
            sig_type: SignalType::RAMP,
            force_changes: RAMP_VAR,
            sig_vals,
            sig_time,
        })
    }

    ///Create a force function that follows a custom pattern
    pub fn custom_force(sig_vals: Vec<f64>, sig_time: Vec<f64>) -> Result<Self, anyhow::Error> {
        verify_time_constraint(&sig_time)?;

        Ok(ForceFunctionGenerator {
            sig_type: SignalType::CUSTOM,
            force_changes: sig_vals.len(),
            sig_vals,
            sig_time,
        })
    }

    pub fn save_to_file(&self, filepath: &str) -> Result<(), anyhow::Error> {
        //Open the file (or create if it doesn't exist)
        let mut file = OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(filepath.trim())
            .unwrap();

        let line: String = format!("Desired force profile");

        //Write to the file - indicating if writing failed (but don't worry about it!)
        if let Err(e) = writeln!(file, "{}", line) {
            bail!("Couldn't write to file: {}", e);
        }

        //See whether to transofmr the data by the
        let line: String =
            //Format the line to write
            format!(
                "TYPE:{:?},VAL_CNT:{},VALS:{:?},PERC_TIMES:{:?}",
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
}

///Verify that the signal time adds up to 100%
fn verify_time_constraint(sig_time: &Vec<f64>) -> Result<(), anyhow::Error> {
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

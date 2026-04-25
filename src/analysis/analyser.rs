///Analyses test results in the form of displaying heightmap results
use crate::analysis::data_handler::DataHandler;
use crate::config::{CamInfo, Config, RobInfo};
use crate::helper_funcs;
use crate::helper_funcs::helper_funcs::ColOpt;
use crate::helper_funcs::helper_funcs::{display_magnitude_map, trans_to_heightmap};
use crate::mapping::terr_map_tools;
use crate::mapping::terr_map_tools::{Heightmap, PointCloud};
use anyhow::bail;
use std::fs;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};

///The analyser structure which contains all the relevant test information
pub struct Analyser {
    ///Filepath location of the test data
    filepath: String,
    ///Name of the test
    test_name: String,
    ///Folder location holding the test data
    test_fp: String,
    ///The struct containing all the raw pos/force info etc
    data_handler: Option<DataHandler>,
    ///Number of pointclouds taken in this test
    no_of_pcl: i32,
    ///The camera configuration information for the given test
    cam_info: Option<Vec<CamInfo>>,
    ///The robot configuration information for the given test
    rob_info: Option<RobInfo>,
    ///Coordination pre-transform flag - True if trajectory already transformed
    pre_trans: bool,
}

impl Analyser {
    ///Create an analyser structure based on the provided file
    pub fn init(filepath: String, test_name: String) -> Result<Self, anyhow::Error> {
        //Check that the filepath exists
        if !fs::read_dir(&filepath)?
            .any(|e| e.unwrap().file_name().into_string().unwrap() == test_name)
        {
            bail!("Test {} does not exist!", test_name);
        }

        let test_fp = format!("{}/{}", filepath, test_name);

        //Count the number of pointclouds
        let mut no_of_pcl = 0;
        for path in fs::read_dir(&test_fp)? {
            if path?.file_name().to_str().unwrap().starts_with("pcl_") {
                no_of_pcl += 1;
            }
        }

        //Create the data handler
        let data_fp = format!("{}/data_{}.txt", test_fp, test_name);

        println!("Loading data - {data_fp}");
        let data_handler: Option<DataHandler>;
        if let Ok(data) = DataHandler::read_data_from_file(data_fp) {
            data_handler = Some(data);
        } else {
            println!("WARNING: No data file");
            data_handler = None;
        }

        //Get the config info
        println!("Loading config - {test_fp}");
        let rob_info: Option<RobInfo>;
        let mut pre_trans = false;
        let cam_info: Option<Vec<CamInfo>>;
        if let Ok(info) = get_config(&test_fp, &test_name) {
            rob_info = Some(info.0);
            cam_info = Some(info.1);
            pre_trans = info.2;
        } else {
            rob_info = None;
            cam_info = None;
        }

        Ok(Self {
            filepath,
            test_name: test_name.to_string(),
            test_fp,
            data_handler,
            no_of_pcl,
            rob_info,
            cam_info,
            pre_trans,
        })
    }

    ///Get the rectangular trajectory bound of the test
    pub fn get_traj_bounds(&self) -> Result<[f64; 4], anyhow::Error> {
        if self.data_handler.is_some() {
            self.data_handler
                .as_ref()
                .expect("Failed to unwrap existing data handler")
                .get_traj_rect_bnds()
        } else {
            bail!("No data file associated with test");
        }
    }

    ///Compare the first and last heightmap from the test (i.e. the overall change)
    pub fn disp_overall_change(&mut self) -> Result<(), anyhow::Error> {
        let hmap_cnt = self.get_hmap_cnt()?;

        let first_hmap: Heightmap;
        let last_hmap: Heightmap;

        //If the heightmaps haven't been generated - generate them now
        if self.no_of_pcl > hmap_cnt {
            let first_fp = format!("{}/pcl_{}_START.txt", self.test_fp, self.test_name);

            first_hmap = Heightmap::create_from_pcl_file(first_fp, 250, 250)?;

            let last_fp = format!("{}/pcl_{}_END.txt", self.test_fp, self.test_name);

            last_hmap = Heightmap::create_from_pcl_file(last_fp, 250, 250)?;
        }
        //If the hmaps already exist, just load them
        else {
            println!("loading from heightmap!");
            let first_fp = format!("{}/hmap_{}_START.txt", self.test_fp, self.test_name);
            first_hmap = Heightmap::create_from_file(first_fp)?;

            println!(
                "first hmap: Width:{}, height:{}",
                first_hmap.width(),
                first_hmap.height()
            );

            let end_fp = format!("{}/hmap_{}_END.txt", self.test_fp, self.test_name);
            last_hmap = Heightmap::create_from_file(end_fp)?;
        }

        //Create a new heightmap that compares the first and last
        let mut comp_hmap = terr_map_tools::comp_maps(&last_hmap, &first_hmap)?;

        comp_hmap.disp_map()?;

        Ok(())
    }

    ///Display every heightmap for the test
    pub fn display_all(&mut self) -> Result<(), anyhow::Error> {
        //Generate the heightmaps if required
        let heightmaps = if self.no_of_pcl > self.get_hmap_cnt()? {
            //If not generate them
            self.gen_all_hmaps(250, 250)?
        } else {
            self.load_all_genned_hmaps()?
        };

        //Go through every heightmap and display it
        for mut hmap in heightmaps {
            println!("{}", hmap.filename);
            hmap.disp_map()?;
        }

        Ok(())
    }

    ///Calculates the total coverage of a measured area (i.e. how much unmeasured space there is)
    /// Unmeasured/unknown space is represented as a black pixel/NaN value
    pub fn calc_coverage(&mut self) -> Result<Vec<f32>, anyhow::Error> {
        //Check if the hmaps have been generated
        let hmaps = if self.no_of_pcl > self.get_hmap_cnt()? {
            //If not generate them
            self.gen_all_hmaps(250, 250)?
        } else {
            self.load_all_genned_hmaps()?
        };

        let mut coverages: Vec<f32> = vec![];

        //Check each map
        for mut map in hmaps {
            //Calculate coverage as the percentage of filled cells
            let mut filled_cells = 0;
            for cell in map.get_flattened_cells()? {
                if !f64::is_nan(cell) {
                    filled_cells += 1;
                }
            }
            coverages.push(filled_cells as f32 / (map.no_of_cells as f32))
        }

        Ok(coverages)
    }

    ///Calculates and saves the coverage of each heightmap to a text file
    pub fn save_coverage(&mut self) -> Result<(), anyhow::Error> {
        //Get the coverages of the heightmaps
        let covs = self.calc_coverage()?;

        //Save them
        let covs_fp = format!("{}/cov_{}.txt", self.test_fp, self.test_name);

        let mut cov_file = File::create(covs_fp)?;

        for cov in covs {
            let curr_line = format!("{}\n", cov);

            cov_file.write_all(curr_line.as_bytes())?;
        }

        println!("Covereage saved");

        Ok(())
    }

    ///Counts the number of generated hmaps saved in the test dir
    fn get_hmap_cnt(&mut self) -> Result<i32, anyhow::Error> {
        //Count the number of heightmaps saved in the test
        let mut hmap_cnt = 0;

        for path in fs::read_dir(&self.test_fp)? {
            if path?.file_name().to_str().unwrap().starts_with("hmap_") {
                hmap_cnt += 1;
            }
        }

        Ok(hmap_cnt)
    }

    ///Generates all the heightmaps for each pcl in the test directory
    fn gen_all_hmaps(&mut self, width: u32, height: u32) -> Result<Vec<Heightmap>, anyhow::Error> {
        println!("GENERATING HEIGHTMAPS ---------");

        let mut heightmaps: Vec<Heightmap> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the pcl files
            if path_str.starts_with("pcl_") {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Create the heightmaps from the pcl
                heightmaps.push(Heightmap::create_from_pcl_file(fp, width, height)?)
            }
        }
        Ok(heightmaps)
    }

    ///Generate a heightmap from a specific point cloud
    fn gen_hmap_n(&mut self, width: u32, height: u32, n: i32) -> Result<Heightmap, anyhow::Error> {
        println!("GENERATING HEIGHTMAP {n} ---------");

        //Check that the n number is valid
        if n >= self.no_of_pcl {
            bail!("Invalid hmap number")
        }

        //Create the filepath
        let fp: String;
        if n == -1 {
            fp = format!("{}/pcl_{}_START.txt", self.test_fp, self.test_name);
        } else if n == -2 {
            fp = format!("{}/pcl_{}_END.txt", self.test_fp, self.test_name);
        } else {
            fp = format!("{}/pcl_{}_{}.txt", self.test_fp, self.test_name, n);
        }
        //Generate the heightmap from the pcl file
        let hmap = Heightmap::create_from_pcl_file(fp, width, height)?;

        Ok(hmap)
    }

    ///Load all the hmaps saved from a test directory to a vector
    fn load_all_genned_hmaps(&mut self) -> Result<Vec<Heightmap>, anyhow::Error> {
        println!("LOADING HEIGHTMAPS ---------");

        let mut heightmaps: Vec<Heightmap> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the hmap files
            if path_str.starts_with("hmap_") {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Load the heightmap file
                heightmaps.push(Heightmap::create_from_file(fp)?)
            }
        }
        Ok(heightmaps)
    }

    ///Load all the pointclouds saved from a test directory to a vector
    fn load_all_pcl(&mut self) -> Result<Vec<PointCloud>, anyhow::Error> {
        println!("LOADING POINTCLOUDS ----------");

        let mut pcls: Vec<PointCloud> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the hmap files
            if path_str.starts_with("pcl_") {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Load the heightmap file
                pcls.push(PointCloud::create_from_file(fp)?)
            }
        }
        Ok(pcls)
    }

    ///Get pcls that have the identifier in the filepath
    fn get_pcl_with_identifier(
        &mut self,
        identifier: &str,
    ) -> Result<Vec<PointCloud>, anyhow::Error> {
        println!("LOADING POINTCLOUDS----------");

        let mut pcls: Vec<PointCloud> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the hmap files
            if path_str.starts_with("pcl_") && path_str.contains(identifier) {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Load the heightmap file
                pcls.push(PointCloud::create_from_file(fp)?)
            }
        }
        Ok(pcls)
    }

    ///Generates a passband filter for test pointclouds based on the trajectory of the robot
    ///iso_(x/y)_radius determine how far the filter extends from the trajectory
    fn gen_iso_rect(
        &mut self,
        iso_x_radius: f64,
        iso_y_radius: f64,
    ) -> Result<[f64; 4], anyhow::Error> {
        //Get the bounds of the trajectory
        let mut traj_bounds = self.get_traj_bounds()?;

        //Check that the robot info exists
        let info = self
            .rob_info
            .as_ref()
            .expect("Failed to get robot info")
            .pos_to_zero();

        if !self.pre_trans {
            traj_bounds[0] = traj_bounds[0] + info[0];
            traj_bounds[1] = traj_bounds[1] + info[0];
            traj_bounds[2] = traj_bounds[2] + info[1];
            traj_bounds[3] = traj_bounds[3] + info[1];
        }

        println!("Trajectory bounds - {:?}", traj_bounds);

        //Increase the bounds by the iso radius
        traj_bounds[0] -= iso_x_radius;
        traj_bounds[1] += iso_x_radius;
        traj_bounds[2] -= iso_y_radius;
        traj_bounds[3] += iso_y_radius;

        //No need to transform as data is transformed to correct frame when saved

        //Return the new trajectory bounds
        Ok(traj_bounds)
    }

    ///Isolates the trajectory path in a pointcloud and turns it into a heightmap
    ///iso_(x/y)_radius determine how far the filter extends from the trajectory
    pub fn disp_iso_traj_path(
        &mut self,
        iso_x_radius: f64,
        iso_y_radius: f64,
    ) -> Result<(), anyhow::Error> {
        //Load every pointcloud taken during the test
        let pcls = self.load_all_pcl()?;

        //Calculate the isolation rectangle bounds
        let iso_bounds = self.gen_iso_rect(iso_x_radius, iso_y_radius)?;

        //Apply the iso-radius bounds as a pass-band filter to each pointcloud
        for mut pcl in pcls {
            if pcl.is_end() {
                //Z is extreme because we have no modification to the z data
                pcl.passband_filter(
                    iso_bounds[0],
                    iso_bounds[1],
                    iso_bounds[2],
                    iso_bounds[3],
                    -999.0,
                    999.0,
                );

                //Transform the pointcloud to a heightmap and display it
                let mut curr_hmap = Heightmap::create_from_pcl(pcl, 100, 100);
                curr_hmap.disp_map()?;
            }
        }

        Ok(())
    }

    ///Calculates the action map of a test
    /// The action map is a geometric representation of the trajectory
    /// Height of the trajectory is represented using the same colour scale as the terrain maps
    fn calc_action_map(
        &mut self,
        width: usize,
        height: usize,
    ) -> Result<Vec<Vec<f64>>, anyhow::Error> {
        //Get the base pointcloud - calculate the bounds
        let fp = format!("{}/pcl_{}_START.txt", self.test_fp, self.test_name);

        //Load the heightmap file
        let bounds = PointCloud::create_from_file(fp)?.get_bounds();
        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        //Get the trajectory
        let mut traj = self
            .data_handler
            .as_ref()
            .expect("Failed to unwrap data handler")
            .get_traj();

        //Check if the trajectory has been transformed
        if !self.pre_trans {
            traj = self.translate_traj(traj);
        }

        //Transform the trajectory to the heightmap space
        trans_to_heightmap(
            traj,
            width,
            height,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            helper_funcs::helper_funcs::MapGenOpt::Mean,
        )
    }

    ///Displays the action map of the test (i.e. graphically encoded trajectory) - height indicate by pixel intensity
    pub fn disp_action_map(&mut self, width: usize, height: usize) -> Result<(), anyhow::Error> {
        //Calculate the action map matrix
        if let Ok(ac_mat) = self.calc_action_map(width, height) {
            display_magnitude_map("Action map", ac_mat, width, height, ColOpt::Median)?;
        } else {
            bail!("Error when displaying action map");
        }

        Ok(())
    }

    ///Calculate the force map (force data transformed to the heightmap space)
    /// option determines the method for calculating the intensity of the pixels
    fn calc_force_map(
        &mut self,
        width: usize,
        height: usize,
        option: ForceSel,
    ) -> Result<Vec<Vec<f64>>, anyhow::Error> {
        //Get the base pointcloud - calculate the bounds
        let fp = format!("{}/pcl_{}_START.txt", self.test_fp, self.test_name);

        //Load the heightmap file
        let bounds = PointCloud::create_from_file(fp)?.get_bounds();
        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        //Get the trajectory data coupled with the force data
        let mut traj_force_dat = self
            .data_handler
            .as_mut()
            .expect("Failed to unwrap data handler")
            .get_traj_force_pairs();

        //If the trajectory hasn't already been trasnformed
        if !self.pre_trans {
            //Extract the trajectory
            let mut traj: Vec<[f64; 3]> = vec![];

            for pair in traj_force_dat.iter() {
                traj.push(pair.0);
            }

            //Transform the trajectory
            traj = self.translate_traj(traj);

            //Rezip back into the trajectory pairs
            for (index, pnt) in traj.iter().enumerate() {
                traj_force_dat[index].0 = *pnt;
            }
        }

        let mut pnts: Vec<[f64; 3]> = vec![];

        //Calculate the map with different intensity values based on
        match option {
            ForceSel::ForceAvg => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([
                        data_pnt.0[0],
                        data_pnt.0[1],
                        ((data_pnt.1[0] + data_pnt.1[1] + data_pnt.1[2]) / 3.0).abs(),
                    ]);
                }
            }

            ForceSel::MomAvg => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([
                        data_pnt.0[0],
                        data_pnt.0[1],
                        ((data_pnt.1[3] + data_pnt.1[4] + data_pnt.1[5]) / 3.0).abs(),
                    ]);
                }
            }

            ForceSel::X => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([data_pnt.0[0], data_pnt.0[1], data_pnt.1[0].abs()]);
                }
            }

            ForceSel::Y => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([data_pnt.0[0], data_pnt.0[1], data_pnt.1[1].abs()]);
                }
            }

            ForceSel::Z => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([data_pnt.0[0], data_pnt.0[1], data_pnt.1[2].abs()]);
                }
            }

            ForceSel::Xmom => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([data_pnt.0[0], data_pnt.0[1], data_pnt.1[3].abs()]);
                }
            }

            ForceSel::Ymom => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([data_pnt.0[0], data_pnt.0[1], data_pnt.1[4].abs()]);
                }
            }

            ForceSel::Zmom => {
                //Couple the average force with xy positions
                for data_pnt in traj_force_dat {
                    pnts.push([data_pnt.0[0], data_pnt.0[1], data_pnt.1[5].abs()]);
                }
            }
        }

        trans_to_heightmap(
            pnts,
            width,
            height,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            helper_funcs::helper_funcs::MapGenOpt::Mean,
        )
    }

    ///Displays the force map of the test (i.e. graphically encoded force experience) - force indicated by pixel colour
    pub fn disp_force_map(
        &mut self,
        width: usize,
        height: usize,
        option: ForceSel,
    ) -> Result<(), anyhow::Error> {
        //Calculate the force map matrix
        if let Ok(fc_mat) = self.calc_force_map(width, height, option) {
            display_magnitude_map("Force map", fc_mat, width, height, ColOpt::Intensity)?;
        } else {
            bail!("Error when displaying action map");
        }
        Ok(())
    }

    ///Rotate all PCLs associated with the test, and regenerate the heightmaps
    /// Rotations specified in radians
    pub fn rotate_and_regen(
        &mut self,
        yaw: f64,
        pitch: f64,
        roll: f64,
        width: u32,
        height: u32,
    ) -> Result<(), anyhow::Error> {
        let mut cnt = 0;

        //Iterate through every pcl file in the test
        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the pcl files
            if path_str.starts_with("pcl_") {
                println!("Rotating PCL: {}", cnt);

                let pcl_fp = format!("{}/{}", self.test_fp, path_str);

                let mut curr_pcl = PointCloud::create_from_file(pcl_fp.clone())?;

                //Rotate the PCL
                curr_pcl.rotate(yaw, pitch, roll);

                //Resave the PCL
                curr_pcl.save_to_file(pcl_fp.strip_suffix(".txt").unwrap())?;

                cnt += 1;
            }
        }

        //Regenerate the heightmaps
        self.regen_hmaps(width, height)?;

        Ok(())
    }

    ///Regenerate the heightmaps associated with a test
    /// width/height determine the number of pixels used to represent the pointcloud
    pub fn regen_hmaps(&mut self, width: u32, height: u32) -> Result<(), anyhow::Error> {
        let mut cnt = 0;

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the pcl files
            if path_str.starts_with("pcl_") {
                println!("Generating HMAP: {}", cnt);

                let pcl_fp = format!("{}/{}", self.test_fp, path_str);

                //create the newly sized hmap
                let mut curr_hmap = Heightmap::create_from_pcl_file(pcl_fp, width, height)?;

                //Get the suffix associated with the current pcl
                let path_split: Vec<&str> = path_str.split("_").collect::<Vec<&str>>();

                let hmap_filestring = format!(
                    "hmap_{}_{}",
                    self.test_name,
                    path_split.last().unwrap().strip_suffix(".txt").unwrap()
                );

                let hmap_fp = format!("{}/{}", self.test_fp, hmap_filestring);
                curr_hmap.save_to_file(&hmap_fp)?;

                cnt += 1;
            }
        }

        Ok(())
    }

    ///Translate the trajectory in xyz frame relative to the provided config information
    fn translate_traj(&mut self, traj: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
        let mut new_traj: Vec<[f64; 3]> = vec![];
        let transform = self
            .rob_info
            .as_ref()
            .expect("Failed to get robot info")
            .pos_to_zero();

        for pnt in traj.iter() {
            new_traj.push([
                pnt[0] + transform[0],
                pnt[1] + transform[1],
                pnt[2] + transform[2],
            ]);
        }
        new_traj
    }

    ///Load all PCLs and cut them to meet the specified passband
    ///WARNING: Will delete your data
    pub fn apply_passband(
        &mut self,
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
        min_z: f64,
        max_z: f64,
    ) -> Result<(), anyhow::Error> {
        let mut cnt = 0;

        //Iterate through every pcl file in the test
        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the pcl files
            if path_str.starts_with("pcl_") {
                println!("Trimming PCL: {}", cnt);

                let pcl_fp = format!("{}/{}", self.test_fp, path_str);

                let mut curr_pcl = PointCloud::create_from_file(pcl_fp.clone())?;

                //Rotate the PCL
                curr_pcl.passband_filter(min_x, max_x, min_y, max_y, min_z, max_z);

                //Resave the PCL
                curr_pcl.save_to_file(pcl_fp.strip_suffix(".txt").unwrap())?;

                cnt += 1;
            }
        }

        println!("Trimming complete");

        Ok(())
    }

    ///From a tests pointclouds - create a set of parametric heightmaps
    ///Define the resolutions, averages and identifiers
    pub fn create_parametric_hmaps(
        &mut self,
        resolutions: Vec<u32>,
        averages: Vec<u32>,
        identifiers: Vec<&str>,
    ) -> Result<(), anyhow::Error> {
        //Get the maximum average value
        let max_avg = *averages.iter().max().expect("Failed to get maximum") as i32;

        //For each identifier specified
        for identifier in identifiers.iter() {
            //Loda all the pointclouds with that identifier
            let mut curr_pcls: Vec<PointCloud> = self
                .get_pcl_with_identifier(*identifier)
                .expect("Failed to find identifier");

            //check that there are enough for the maxmimum average
            if curr_pcls.len() < max_avg as usize {
                bail!("Not enough pointclouds for the average required!")
            }

            for resolution in resolutions.iter() {
                let mut cnt: u32 = 0;

                //Turn all of the pointclouds into heightmaps
                let mut curr_hmaps: Vec<Heightmap> = vec![];

                for pcl in curr_pcls.iter() {
                    curr_hmaps.push(Heightmap::create_using_pcl_ref(
                        pcl,
                        *resolution,
                        *resolution,
                    ));

                    cnt += 1;

                    if averages.contains(&cnt) {
                        let filepath = format!(
                            "{}/hmap_{}_{}_res_{}_avg_{}",
                            self.test_fp, self.test_name, *identifier, *resolution, cnt
                        );

                        println!("{}", filepath);

                        terr_map_tools::average_heightmaps(&curr_hmaps).save_to_file(&filepath);

                        println!(
                            "Average created for {} number of pcls at resolution {} at identifier {}",
                            cnt, *resolution, *identifier
                        );
                    }
                }
                println!("Completed for resolution {}", *resolution);
            }

            //Empty the current list
            curr_pcls.clear();

            println!("Completed for identifier {}", *identifier);
        }

        Ok(())
    }
}

///Determines the method for force intensity calculation
pub enum ForceSel {
    ForceAvg,
    MomAvg,
    X,
    Y,
    Z,
    Xmom,
    Ymom,
    Zmom,
}

///Loads the stored test configuration
fn get_config(
    filepath: &String,
    test_name: &String,
) -> Result<(RobInfo, Vec<CamInfo>, bool), anyhow::Error> {
    //Get the first line of the cam config file
    let config_fp = format!("{}/conf_{}.txt", filepath, test_name);

    let conf_file = File::open(config_fp)?;
    let mut line_reader = BufReader::new(conf_file);

    //Open the file and read the first line
    let mut cam0_info_line = String::new();
    line_reader.read_line(&mut cam0_info_line)?;

    //Open the file and read the first line
    let mut cam1_info_line = String::new();
    line_reader.read_line(&mut cam1_info_line)?;

    //Read the second line to get the robot info
    let mut rob_info_line = String::new();
    line_reader.read_line(&mut rob_info_line)?;

    //Check if trajectory data has already been transformed (to the box frame)
    println!("Loading transform flag");
    let mut trans_flag_str = String::new();
    line_reader.read_line(&mut trans_flag_str)?;
    println!("{trans_flag_str}");
    let trans_flag = trans_flag_str.contains("true");

    //Attempt to create both the configs inplace
    Ok((
        RobInfo::create_rob_info_from_line(rob_info_line)?,
        vec![
            CamInfo::create_cam_info_from_line(cam0_info_line)?,
            CamInfo::create_cam_info_from_line(cam1_info_line)?,
        ],
        trans_flag,
    ))
}

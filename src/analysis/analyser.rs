use crate::analysis::data_handler::DataHandler;
use crate::config::{CamInfo, RobInfo};
use crate::mapping::terr_map_tools;
use crate::mapping::terr_map_tools::{Heightmap, PointCloud};
use anyhow::bail;
use std::fs;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};

pub struct Analyser {
    //Filepath location of the test data
    filepath: String,
    //Name of the test
    test_name: String,
    //Folder location holding the test data
    test_fp: String,
    //The struct containing all the raw pos/force info etc
    data_handler: DataHandler,
    //Number of pointlcouds taken in this test
    no_of_pcl: i32,
    //The camera information for the given test
    cam_info: CamInfo,
    //The robot information for the given test
    rob_info: RobInfo
}

impl Analyser {
    //Create a data analyser
    pub fn init(filepath: String, test_name: String) -> Result<Self, anyhow::Error> {
        let filepath = filepath;

        //Check that the test exists
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
                no_of_pcl = no_of_pcl + 1;
            }
        }

        //Create the data handler
        let data_fp = format!("{}/data_{}.txt", test_fp, test_name);

        println!("Loading data - {data_fp}");
        let data_handler = DataHandler::read_data_from_file(data_fp)?;

        //Get the config info
        let config_info = get_config(&test_fp, &test_name)?;

        println!("{:?}", config_info.0);



        Ok(Self {
            filepath,
            test_name: test_name.to_string(),
            test_fp,
            data_handler,
            no_of_pcl,
            cam_info : config_info.1,
            rob_info : config_info.0
        })
    }

    //Get the rectangular bounds of the test (i.e. where the end-effector interacted with the soil)
    pub fn get_traj_bounds(&mut self) -> Result<[f32; 4], anyhow::Error> {
        self.data_handler.get_traj_rect_bnds()
    }

    //Display the height change from the first to the last heightmap
    pub fn disp_overall_change(&mut self) -> Result<(), anyhow::Error> {


        let hmap_cnt = self.get_hmap_cnt()?;

        let mut first_hmap: Heightmap = Default::default();
        let mut last_hmap: Heightmap = Default::default();

        //If the heightmaps haven't been generated - generate them now
        if self.no_of_pcl > hmap_cnt {
            let first_fp = format!("{}/pcl_{}_0.txt", self.test_fp, self.test_name);

            first_hmap = Heightmap::create_from_pcl_file(first_fp, 250, 250, false)?;

            let last_fp = format!(
                "{}/pcl_{}_{}.txt",
                self.test_fp,
                self.test_name,
                self.no_of_pcl - 1
            );

            last_hmap = Heightmap::create_from_pcl_file(last_fp, 250, 250, false)?;
        }
        //If the hmaps already exist, just load them
        else {
            let first_fp = format!("{}/hmap_{}_0.txt", self.test_fp, self.test_name);

            first_hmap = Heightmap::create_from_file(first_fp)?;

            let last_fp = format!(
                "{}/hmap_{}_{}.txt",
                self.test_fp,
                self.test_name,
                hmap_cnt - 1
            );

            last_hmap = Heightmap::create_from_file(last_fp)?;
        }

        //Create a new heightmap that compares the first and last

        let mut comp_hmap = terr_map_tools::comp_maps(&last_hmap, &first_hmap)?;

        comp_hmap.disp_map();

        Ok(())
    }

    //Displays all of heightmaps associated with a test
    //TODO: Check if hmaps havent been genereated, then generate and display from pcl
    pub fn display(&mut self) -> Result<(), anyhow::Error> {
        let mut heightmaps: Vec<Heightmap> = vec![];

        //Check if the hmaps have been generated
        if self.no_of_pcl > self.get_hmap_cnt()? {
            //If not generate them
            heightmaps = self.gen_all_hmaps(250, 250)?;
        } else {
            heightmaps = self.load_all_genned_hmaps()?
        }

        //Go through every heightmap and display it
        for mut hmap in heightmaps {
            hmap.disp_map();
        }

        Ok(())
    }

    //Calculates the total coverage of a measured area (i.e. how much unknown space there is)
    pub fn calc_coverage(&mut self) -> Result<Vec<f32>, anyhow::Error> {
        let mut hmaps: Vec<Heightmap> = vec![];

        //Check if the hmaps have been generated
        if self.no_of_pcl > self.get_hmap_cnt()? {
            //If not generate them
            hmaps = self.gen_all_hmaps(250, 250)?;
        } else {
            hmaps = self.load_all_genned_hmaps()?
        }

        let mut coverages: Vec<f32> = vec![];

        //Check each map
        for mut map in hmaps {
            //Calculate coverage as the percentage of filled cells

            let mut filled_cells = 0;
            for cell in map.get_flattened_cells()? {
                if !f32::is_nan(cell) {
                    filled_cells = filled_cells + 1;
                }
            }
            coverages.push(filled_cells as f32 / (map.no_of_cells as f32))
        }

        Ok(coverages)
    }

    //Calculates and saves the coverage of a test to the test folder
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

    //Counts the number of generated hmaps saved in the test dir
    fn get_hmap_cnt(&mut self) -> Result<i32, anyhow::Error> {
        //Count the number of heightmaps saved in the test
        let mut hmap_cnt = 0;

        for path in fs::read_dir(&self.test_fp)? {
            if path?.file_name().to_str().unwrap().starts_with("hmap_") {
                hmap_cnt = hmap_cnt + 1;
            }
        }

        Ok(hmap_cnt)
    }

    //Generates all the heightmaps for each pcl in the test directory
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
                heightmaps.push(Heightmap::create_from_pcl_file(fp, width, height, false).unwrap())
            }
        }
        Ok(heightmaps)
    }

    fn gen_hmap_n(&mut self, width : u32, height : u32, n : i32) -> Result<Heightmap, anyhow::Error> {
        println!("GENERATING HEIGHTMAP {n} ---------");

        //Check that the n number is valid
        if n >= self.no_of_pcl{
            bail!("Invalid hmap number")
        }

        //Create the filepath
        let fp = format!("{}/pcl_{}_{}.txt", self.test_fp, self.test_name, n);

        //Generate the heightmap from the pcl file
        let hmap = Heightmap::create_from_pcl_file(fp, width,height, false)?;

        Ok(hmap)
    }

    //Load all the hmaps saved to a test directory
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

    //Generates the isolation rectangle to surround the trajectory
    fn gen_iso_rect(&mut self, iso_x_radius : f32, iso_y_radius : f32) -> Result<[f32; 4], anyhow::Error>{

        //Get the bounds of the trajectory
        let mut traj_bounds = self.get_traj_bounds()?;

        println!("Trajectory bounds - {:?}", traj_bounds);

        //Increase the bounds by the iso radius
        traj_bounds[0] = traj_bounds[0] - iso_x_radius;
        traj_bounds[1] = traj_bounds[1] + iso_x_radius;
        traj_bounds[2] = traj_bounds[2] - iso_y_radius;
        traj_bounds[3] = traj_bounds[3] + iso_y_radius;

        //No need to transform as data is transformed to correct frame when saved


        //Return the new trajectory bounds
        Ok(traj_bounds)
    }


    //Isolates the region in each pointcloud that is potenitally affected by the trajectory
    //Turns it into a heightmap and displays it
    pub fn disp_iso_traj_path(&mut self, iso_x_radius: f32, iso_y_radius: f32) -> Result<(), anyhow::Error> {
        //Load every pointcloud taken during the test
        let pcls = self.load_all_pcl()?;

        //Calculate the isolation rectangle bounds
        let iso_bounds = self.gen_iso_rect(iso_x_radius, iso_y_radius)?;

        //Apply the iso-radius bounds as a pass-band filter to each pointcloud
        for mut pcl in pcls {
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
            let mut curr_hmap = Heightmap::create_from_pcl(pcl, 250, 250, false);
            curr_hmap.disp_map();
        }

        Ok(())
    }





}


fn get_config(filepath:  &String, test_name: &String) -> Result<(RobInfo, CamInfo), anyhow::Error>{

    //Get the first line of the cam config file
    let config_fp = format!("{}/conf_{}.txt", filepath, test_name);

    let conf_file = File::open(config_fp)?;
    let mut line_reader = BufReader::new(conf_file);

    //Open the file and read the first line
    let mut cam_info_line = String::new();
    line_reader.read_line(&mut cam_info_line)?;

    

    //Read the second line to get the robot info
    let mut rob_info_line = String::new();
    line_reader.read_line(&mut rob_info_line)?;

    //Attempt to create both the configs inplace
    Ok((RobInfo::create_rob_info_from_line(rob_info_line)?, CamInfo::create_cam_info_from_line(cam_info_line)?))


}

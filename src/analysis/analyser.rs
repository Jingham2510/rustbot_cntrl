use crate::analysis::data_handler::DataHandler;
use crate::mapping::terr_map_tools;
use crate::mapping::terr_map_tools::{Heightmap, PointCloud};
use anyhow::bail;
use std::fs;
use std::fs::File;
use std::io::Write;
use crate::config::CamInfo;

pub struct Analyser {

    //Filepath location of the test data
    filepath : String,
    //Name of the test
    test_name: String,
    //Folder location holding the test data
    test_fp: String,
    //The struct containing all the raw pos/force info etc
    data_handler: DataHandler,
    //Number of pointlcouds taken in this test
    no_of_pcl: i32,
    //The camera information for the given test
    cam_info : CamInfo
}

impl Analyser {
    //Create a data analyser
    pub fn init(filepath : String, test_name: String) -> Result<Self, anyhow::Error> {

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

        println!("{data_fp}");

        let mut data_handler = DataHandler::read_data_from_file(data_fp)?;
        let cam_info = data_handler.get_cam_info()?;

        println!("{:?}", cam_info);

        Ok(Self {
            filepath,
            test_name: test_name.to_string(),
            test_fp,
            data_handler,
            no_of_pcl,
            cam_info
        })
    }

    //Get the rectangular bounds of the test (i.e. where the end-effector interacted with the soil)
    pub fn get_traj_bounds(&mut self) -> [f32; 4] {
        self.data_handler.get_traj_rect_bnds()
    }

    //Display the height change from the first to the last heightmap
    pub fn disp_overall_change(&mut self) -> Result<(), anyhow::Error> {
        if self.no_of_pcl < 2 {
            bail!("Not enough PCLS to perform analysis!")
        }

        let hmap_cnt = self.get_hmap_cnt();

        let mut first_hmap: Heightmap = Default::default();
        let mut last_hmap: Heightmap = Default::default();

        //If the heightmaps haven't been genereated - generate them now
        if self.no_of_pcl > hmap_cnt? {
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
                self.no_of_pcl - 1
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
    pub fn save_coverage(&mut self) -> Result<(), anyhow::Error>{

        //Get the coverages of the heightmaps
        let covs = self.calc_coverage()?;

        //Save them
        let covs_fp = format!("{}/cov_{}.txt", self.test_fp, self.test_name);

        let mut cov_file = File::create(covs_fp)?;

        for cov in covs{
            let curr_line = format!("{}\n", cov);

            cov_file.write_all(curr_line.as_bytes())?;

        }


        println!("Covereage saved");

        Ok(())




    }



    //Counts the numbre of generated hmaps saved in the test dir
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

    fn load_all_pcl(&mut self) -> Result<Vec<PointCloud>, anyhow::Error>{
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


    //Isolates the region in each pointcloud that is potenitally affected by the trajectory
    //Turns it into a heightmap and displays it
    pub fn disp_iso_traj_path(&mut self, iso_x_radius : f32, iso_y_radius : f32){

        //Load every pointcloud taken during the test
        let pcls = self.load_all_pcl().unwrap();

        //Get the bounds of the trajectory
        let mut traj_bounds = self.get_traj_bounds();

        //Stretch the bounds by the iso radius
        traj_bounds[0] = traj_bounds[0] - iso_x_radius;
        traj_bounds[1] = traj_bounds[1] + iso_x_radius;
        traj_bounds[2] = traj_bounds[2] - iso_y_radius;
        traj_bounds[3] = traj_bounds[3] + iso_y_radius;

        //Transform the iso-radius bounds into the camera frame

        todo!("Haven't got the transformation yet!");

        //Stretch the points to fit the camera frame
        //Need to think about this, do we want to stretch the points about the origin?
        //Or do we want to strech about the middle of the bounds?
        //There might not even be a scale? just change from mm to m
        let x_stretch_factor : f32;
        let y_stretch_factor : f32;

        //For now from the origin
        traj_bounds[0] = traj_bounds[0] * x_stretch_factor;
        traj_bounds[1] = traj_bounds[1] * x_stretch_factor;

        traj_bounds[2] = traj_bounds[2] * y_stretch_factor;
        traj_bounds[3] = traj_bounds[3] * y_stretch_factor;


        //Rotate the points
        let theta : f32;
        traj_bounds[0] = traj_bounds[0]*theta.cos() - traj_bounds[2]*theta.sin();
        traj_bounds[1] = traj_bounds[1]*theta.cos() - traj_bounds[3]*theta.sin();
        traj_bounds[2] = traj_bounds[0] * theta.sin() + traj_bounds[2] * theta.cos();
        traj_bounds[3] = traj_bounds[1] * theta.sin() + traj_bounds[3] * theta.cos();


        //Translate the points
        let x_trans_factor : f32;
        let y_trans_factor : f32;
        traj_bounds[0] = traj_bounds[0] + x_trans_factor;
        traj_bounds[1] = traj_bounds[1] + x_trans_factor;
        traj_bounds[2] = traj_bounds[2] + y_trans_factor;
        traj_bounds[3] = traj_bounds[3] + y_trans_factor;



        let iso_passband : [f32;4] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN];


        //Apply the iso-radius bounds as a pass-band filter to each pointcloud
        for mut pcl in pcls{
            pcl.passband_filter(iso_passband[0], iso_passband[1], iso_passband[2], iso_passband[3], -999.0, 999.0);

            //Transform the pointcloud to a heightmap and display it
            let mut curr_hmap = Heightmap::create_from_pcl(pcl, 250, 250, false);
            curr_hmap.disp_map();
       
        }



    }



}

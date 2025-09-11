use std::{default, fs};
use anyhow::bail;
use crate::analysis::data_handler::DataHandler;
use crate::mapping::terr_map_tools;
use crate::mapping::terr_map_tools::{Heightmap, PointCloud};

pub struct Analyser{
    //Name of the test
    test_name : String,
    //Folder location holding the test data
    test_fp : String,
    //The struct containing all the raw pos/force info etc
    data_handler : DataHandler,
    //Number of pointlcouds taken in this test
    no_of_pcl : i32
}

const DEFAULT_TEST_FP : &str = "C:/Users/User/Documents/Results/DEPTH_TESTS";

impl Analyser{

    //Create a data analyser
    pub fn init(test_name : String) -> Result<Self, anyhow::Error>{

        //Check that the test exists
        if !fs::read_dir(DEFAULT_TEST_FP)?.any( |e| e.unwrap().file_name().into_string().unwrap() == test_name){
            bail!("Test {} does not exist!", test_name);
        }

        let test_fp = format!("{}/{}", DEFAULT_TEST_FP, test_name);

        //Count the number of pointclouds
        let mut no_of_pcl = 0;
        for path in fs::read_dir(&test_fp)?{

            if path.unwrap().file_name().to_str().unwrap().starts_with("pcl_"){
                no_of_pcl = no_of_pcl + 1;
            }
        }

        //Create the data handler
        let data_fp = format!("{}/data_{}.txt", test_fp, test_name);


        let data_handler = DataHandler::read_data_from_file(data_fp)?;


        Ok(Self{
            test_name: test_name.to_string(),
            test_fp,
            data_handler,
            no_of_pcl,
        })


    }

    //Get the rectangular bounds of the test (i.e. where the end-effector interacted with the soil)
    pub fn get_traj_bounds(&mut self) -> [f32; 4]{

        self.data_handler.get_traj_rect_bnds()

    }


    //Display the height change from the first to the last heightmap
    pub fn disp_overall_change(&mut self) -> Result<(), anyhow::Error> {


        if self.no_of_pcl < 2{
            bail!("Not enough PCLS to perform analysis!")
        }


        //Count the number of heightmaps saved in the test
        let mut hmap_cnt = 0;

        for path in fs::read_dir(&self.test_fp)?{
            if path?.file_name().to_str().unwrap().starts_with("hmap_"){
                hmap_cnt = hmap_cnt + 1;
            }
        }

        let mut first_hmap : Heightmap = Default::default();
        let mut last_hmap : Heightmap = Default::default();

        //If the heightmaps haven't been genereated - generate them now
        if hmap_cnt < self.no_of_pcl{

            let first_fp = format!("{}/pcl_{}_0.txt", self.test_fp, self.test_name);

            let first_pcl = PointCloud::create_from_file(first_fp)?;

            first_hmap = Heightmap::create_from_pcl(first_pcl, 250, 250, false);

            let last_fp = format!("{}/pcl_{}_{}.txt", self.test_fp, self.test_name, self.no_of_pcl - 1);

            let last_pcl = PointCloud::create_from_file(last_fp)?;

            last_hmap = Heightmap::create_from_pcl(last_pcl, 250, 250, false);
        }//If the hmaps already exist, just load them
        else{

            let first_fp = format!("{}/hmap_{}_0.txt", self.test_fp, self.test_name);

            first_hmap = Heightmap::create_from_file(first_fp)?;

            let last_fp = format!("{}/hmap_{}_{}.txt", self.test_fp, self.test_name, self.no_of_pcl - 1);

            last_hmap = Heightmap::create_from_file(last_fp)?;
        }


        //Create a new heightmap that compares the first and last

        let mut comp_hmap = terr_map_tools::comp_maps(&last_hmap, &first_hmap)?;

        comp_hmap.disp_map();



        Ok(())


    }
    
    
    //Displays all of the heightmaps associated with a test
    pub fn display(&mut self) -> Result<(), anyhow::Error> {

        //Go through each file in the test directory
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name().into_string().expect("FAILED TO CONVERT PATH TO STRING");

            //Only read the hmap files
            if !path_str.starts_with("hmap") {
                continue;
            } else {
                //Go through each hmap
                println!("{:?}", path_str);

                let full_fp = format!("{}/{}", self.test_fp, path_str);
                
                println!("{full_fp}");

                let mut curr_hmap = Heightmap::create_from_file(full_fp)?;

                curr_hmap.disp_map();
            }
        }
        Ok(())
    }
    
    
  
    
}
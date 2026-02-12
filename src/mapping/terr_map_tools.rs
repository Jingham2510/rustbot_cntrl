use crate::helper_funcs;
use crate::helper_funcs::helper_funcs::ColOpt;
use anyhow::bail;
use chrono::{DateTime, Utc};
use rand::RngExt;
use realsense_sys::rs2_vertex;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};

//Pointcloud structure - contains purely point information
//Basically a fancy vector wrapper
pub struct PointCloud {
    //List of the points stored in xyz format
    points: Vec<[f32; 3]>,
    //The number of points present
    no_of_points: usize,

    //The timestamp of when the pointcloud was captured (relative to when the camera started)
    rel_timestamp: f64,

    global_timestamp: DateTime<Utc>,
}

impl PointCloud {
    //Create a pointcloud from a list of points
    pub fn create_from_list(pnts: Vec<[f32; 3]>, timestamp: f64) -> Self {
        //Calculate the number of points
        let no_of_points = pnts.len();

        Self {
            points: pnts,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Utc::now(),
        }
    }

    pub fn create_from_iter(rs2_vertex: &[rs2_vertex], timestamp: f64) -> Self {
        let mut points: Vec<[f32; 3]> = vec![];
        let mut no_of_points = 0;

        for vertex in rs2_vertex.iter() {
            let pnt = vertex.xyz;

            //check if the point is valid - if not ignore it
            if pnt == [0.0, 0.0, 0.0] {
                continue;
            }

            points.push(pnt);
            no_of_points = no_of_points + 1;
        }

        Self {
            points,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Utc::now(),
        }
    }

    //Load a pointcloud file and create
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Create the filepath
        let filepath = format!("{}", filepath.to_string());

        //Open the file and create a buffer to read the lines
        let file = File::open(filepath)?;
        let mut line_reader = BufReader::new(file);

        //Read the first line to get the date
        let timestamp_str: &mut String = &mut "".to_string();
        line_reader.read_line(timestamp_str)?;

        //Convert the string to the datetime f64
        let global_timestamp: DateTime<Utc> = timestamp_str.parse()?;

        //Setup the empty points
        let mut points = vec![];

        for line in line_reader.lines() {
            //Create empty point
            let mut pnt: [f32; 3] = [f32::NAN, f32::NAN, f32::NAN];

            let mut cnt = 0;

            //Delimit the line based on commas
            for token in line?.split(",") {
                pnt[cnt] = token.parse()?;

                cnt = cnt + 1;
            }

            points.push(pnt);
        }

        let no_of_points = points.len();

        Ok(Self {
            points,
            no_of_points,
            //Relative timestamp is -1.0 because there is no reference start time
            rel_timestamp: -1.0,
            global_timestamp,
        })
    }

    pub fn points(&self) -> Vec<[f32; 3]> {
        self.points.clone()
    }

    //Print all points
    pub fn print_points(&mut self) {
        for pnt in self.points.iter() {
            println!("{:?}", pnt);
        }
    }

    //Calculate the xy bounds of the pointcloud (rectangular)
    //Assumes z is the height
    pub fn get_bounds(&mut self) -> [f32; 4] {
        //Predefine the values we are interested in
        let mut x_min: f32 = 9999.0;
        let mut y_min: f32 = 9999.0;
        let mut x_max: f32 = -9999.0;
        let mut y_max: f32 = -9999.0;

        //Check each point to if it escapes the set bounds
        //If points are sorted beforehand, no need! but sorting might take a while - and how do you sort?
        for pnt in self.points.iter() {
            //Check x-bounds
            if pnt[0] < x_min {
                x_min = pnt[0];
            } else if pnt[0] > x_max {
                x_max = pnt[0];
            }

            //Check y-bounds
            if pnt[1] < y_min {
                y_min = pnt[1];
            } else if pnt[1] > y_max {
                y_max = pnt[1];
            }
        }

        //Return the bounding coordinates of the rectangle
        [x_min, x_max, y_min, y_max]
    }

    //Rotate a pointcloud - inplace
    //Can cheat with the mat multiplication here, we know the predefined sizes already
    //Yaw - X
    //Pitch - Y
    //Roll - Z
    pub fn rotate(&mut self, yaw: f32, pitch: f32, roll: f32) {
        //Create the transform matrix rows
        let x_rot = [
            roll.cos() * pitch.cos(),
            -((roll.sin() * yaw.cos()) + (roll.cos() * pitch.sin() * yaw.sin())),
            (roll.sin() * yaw.sin()) + (roll.cos() * pitch.sin() * yaw.cos()),
        ];
        let y_rot = [
            roll.sin() * pitch.cos(),
            (roll.cos() * yaw.cos()) + (roll.sin() * pitch.sin() * yaw.sin()),
            -(roll.cos() * yaw.sin()) + (roll.sin() * pitch.sin() * yaw.cos()),
        ];
        let z_rot = [
            -pitch.sin(),
            pitch.cos() * yaw.sin(),
            pitch.cos() * yaw.cos(),
        ];

        //Iterate through every point in the pointcloud
        for pnt in self.points.iter_mut() {
            //store the original points
            let og_x = pnt[0];
            let og_y = pnt[1];
            let og_z = pnt[2];

            //Rotate by multiplying the vector by the transform matrix
            pnt[0] = x_rot[0] * og_x + x_rot[1] * og_y + x_rot[2] * og_z;
            pnt[1] = y_rot[0] * og_x + y_rot[1] * og_y + y_rot[2] * og_z;
            pnt[2] = z_rot[0] * og_x + z_rot[1] * og_y + z_rot[2] * og_z;
        }
    }

    //Translate a pointcloud with xyz coords - inplace
    pub fn translate(&mut self, x: f32, y: f32, z: f32) {
        //Iterate through every point
        for pnt in self.points.iter_mut() {
            //Transform the points using addition
            pnt[0] = pnt[0] + x;
            pnt[1] = pnt[1] + y;
            pnt[2] = pnt[2] + z;
        }
    }

    //Scale every point by the same value
    pub fn scale_even(&mut self, scale_val : f32){

        //Iterate through every point
        for pnt in self.points.iter_mut() {
            //Transform the points using addition
            pnt[0] = pnt[0] * scale_val;
            pnt[1] = pnt[1] * scale_val;
            pnt[2] = pnt[2] * scale_val;
        }
        
    }


    //Filter a pointcloud by specifying the bounds (bounds inclusive of points on the bound)
    pub fn passband_filter(
        &mut self,
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_z: f32,
        max_z: f32,
    ) {
        let pnts = self.points();

        let mut new_pnts: Vec<[f32; 3]> = vec![];

        for pnt in pnts {
            //Drop all the points that sit outside the bounds
            if pnt[0] < min_x
                || pnt[0] > max_x
                || pnt[1] < min_y
                || pnt[1] > max_y
                || pnt[2] < min_z
                || pnt[2] > max_z
            {
                continue;
            } else {
                new_pnts.push(pnt);
            }
        }

        //Store the passbanded poimnts
        self.points = new_pnts;
        //Update the point count
        self.no_of_points = self.points.len();
    }

    //Save the pointcloud to an ASCII file

    pub fn save_to_file(&mut self, filepath: &str) -> Result<(), anyhow::Error> {
        //Create a file
        let mut file = File::create(filepath.to_owned() + ".txt")?;

        //Save the timestamp from the frame as the first line;
        let datetime_fmt = format!("{}\n", self.global_timestamp);

        file.write_all(datetime_fmt.as_bytes())?;

        //Save the cloud points on seperate lines
        for i in 0..self.no_of_points {
            let pnt = format!(
                "{:?},{:?},{:?}\n",
                self.points[i][0], self.points[i][1], self.points[i][2]
            );

            file.write_all(pnt.as_bytes())?;
        }

        //Return the all clear
        Ok(())
    }
}

//Map structure - contains the size and height information for each cell
pub struct Heightmap {
    //Height and width of the terrain map
    height: u32,
    width: u32,

    //Indicates whether the grid is square or not
    square: bool,
    pub no_of_cells: u32,

    //The min and max cell heights
    min: f32,
    max: f32,
    //keeps track of whether the min or max need to be updated again
    min_updated: bool,
    max_updated: bool,

    //Cell location contianing the min and max height cells
    min_pos: (u32, u32),
    max_pos: (u32, u32),

    //the 2d vector representing the cells
    cells: Vec<Vec<f32>>,

    //The pointcloud bounds it was constructed from (used to position the heightmap in the real world)
    lower_coord_bounds: [f32;2],
    upper_coord_bounds: [f32;2],


    //Store the filepath for usage in analysis
    pub filename : String
}


impl Default for Heightmap {
    //Default heightmap is a blank 250x250
    fn default() -> Heightmap {
        Heightmap::new(250, 250)
    }
}

//Map tools - including display and modification etc
impl Heightmap {
    pub fn new(width: u32, height: u32) -> Self {
        //Initialise new variables for the object
        let mut square_check = false;

        if height == width {
            square_check = true;
        }


        let filename = "No filepath".to_string();

        //Generate an empty cell bed of height*width size
        Self {
            height,
            width,
            square: square_check,
            no_of_cells: height * width,
            cells: vec![vec![0.0; height as usize]; width as usize],
            min: 999.0,
            max: -999.0,
            min_updated: false,
            max_updated: false,
            min_pos: (0, 0),
            max_pos: (0, 0),
            lower_coord_bounds : [0.0,0.0],
            upper_coord_bounds : [0.0,0.0],
            filename
        }
    }

    //Create a heightmap from a pointcloud (takes ownership of pcl object)
    pub fn create_from_pcl(mut pcl: PointCloud, width: u32, height: u32) -> Self {
        //Get the bounds
        let bounds = pcl.get_bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        let cells = helper_funcs::helper_funcs::trans_to_heightmap(pcl.points, width as usize, height as usize, total_width, total_height, bounds[0], bounds[2], helper_funcs::helper_funcs::MapGenOpt::Mean).expect("Failed to convert PCL to heightmap");

        let square;
        //check if the heightmap is square
        if height == width {
            square = true;
        } else {
            square = false;
        }

        //create the filename from the relative timestamp
        let filename = format!("created from pcl- {}", pcl.rel_timestamp);

        Self {
                height,
                width,
                square,
                no_of_cells: height * width,
                min: 999.0,
                max: -999.0,
                min_updated: false,
                max_updated: false,
                min_pos: (0, 0),
                max_pos: (0, 0),
                cells,
                lower_coord_bounds : [bounds[0], bounds[2]],
                upper_coord_bounds : [bounds[1], bounds[3]],
                filename

        }
    }

    //Creates a heightmap from a hmap file
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Create the filepath

        //Open the file and create a buffer to read the lines
        let file = File::open(filepath.clone())?;
        let mut line_reader = BufReader::new(file);

        //Read the first line to extract the bounds
        let mut first_line = String::new();
        line_reader.read_line(&mut first_line)?;

        let bounds = Self::extract_bounds(first_line);

        let mut bounds_res : [f32;4] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN];

        if bounds.is_ok(){
            bounds_res = bounds?
        }


        let mut height = 0;
        let mut width = 0;
        let mut width_set = false;

        let mut cells: Vec<Vec<f32>> = vec![];

        //Go through each cell and update the
        for line in line_reader.lines() {
            //Create the empty row
            let mut row: Vec<f32> = vec![];

            //Split via comma then iterate
            for token in line?.split(",") {
                if !width_set {
                    width = width + 1;
                }

                //Check the slot isn't empty
                if !token.is_empty() {
                    row.push(token.parse::<f32>()?);
                }
            }

            if !width_set {
                width_set = true;
            }

            //Store the row
            cells.push(row);
            height = height + 1;
        }

        //Check to see if the grid is square
        let mut square = false;
        if height == width {
            square = true;
        }

        Ok(Self {
            height,
            width,
            square,
            no_of_cells: height * width,
            min: 999.0,
            max: -999.0,
            min_updated: false,
            max_updated: false,
            min_pos: (0, 0),
            max_pos: (0, 0),
            cells,
            lower_coord_bounds : [bounds_res[0], bounds_res[2]],
            upper_coord_bounds : [bounds_res[1], bounds_res[3]],
            filename : filepath
        })
    }

    //Creates a heightmap from a given pcl file
    pub fn create_from_pcl_file(
        filepath: String,
        width: u32,
        height: u32,
    ) -> Result<Self, anyhow::Error> {
        //Load the pcl file
        let pcl = PointCloud::create_from_file(filepath)?;

        //Turn the pcl into a heightmap and return it
        Ok(Heightmap::create_from_pcl(pcl, width, height))
    }

    pub fn print_cells(&self) {
        for row in &self.cells {
            for cell in row {
                print!("{} ", cell);
            }
            print!("\n");
        }
    }

    //Get the height for a given cell
    pub fn get_cell_height(&self, x: u32, y: u32) -> Result<f32, anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to read from cell that doesnt exist!");
        }

        Ok(self.cells[x as usize][y as usize])
    }

    //Set the height of a given cell
    pub fn set_cell_height(&mut self, x: u32, y: u32, new_height: f32) {
        if x > self.width || y > self.height {
            println!("Warning - attempting to write to cell that doesnt exist!");
            return;
        }

        self.cells[x as usize][y as usize] = new_height;

        //Check that the cell doesnt store the min or the max
        if (x, y) == self.max_pos {
            self.get_max();
        } else if (x, y) == self.min_pos {
            self.get_min();
        } else {
            if new_height > self.max {
                self.max = new_height;
                self.max_pos = (x, y);
            } else if new_height < self.min {
                self.min = new_height;
                self.min_pos = (x, y);
            } else {
                println!("{0} is not larger than {1}!", new_height, self.max);
            }
        }
    }

    //Set the height of all cells
    pub fn set_map(&mut self, new_heights: Heightmap) {
        //Sweep through each cell and replace with the new map height
        for (x, row) in self.cells.iter_mut().enumerate() {
            for (y, col) in row.iter_mut().enumerate() {
                *col = new_heights.cells[x][y]
            }
        }

        self.max_updated = false;
        self.min_updated = false;

        self.get_max();
        self.get_min();
    }

    //Get the maximum cell height
    fn get_max(&mut self) -> f32 {
        //Check whether a new maximum is required
        if !self.max_updated {
            let mut new_max: f32 = -999.0;

            //Check every value to see if its the largest
            for (_x, row) in self.cells.iter_mut().enumerate() {
                for (_y, col) in row.iter_mut().enumerate() {
                    if col > &mut new_max {
                        new_max = *col;
                    }
                }
            }

            self.max = new_max;
            self.max_updated = true;
        }

        self.max
    }

    //Get the minimum cell height
    fn get_min(&mut self) -> f32 {
        //Check whether a new minimum calc is required
        if !self.min_updated {
            let mut new_min: f32 = 999.0;

            //Check every value to see if its the smallest
            for (_x, row) in self.cells.iter_mut().enumerate() {
                for (_y, col) in row.iter_mut().enumerate() {
                    if col < &mut new_min {
                        new_min = *col;
                    }
                }
            }
            self.min = new_min;
            self.min_updated = true;
        }

        self.min
    }

    //Generates a pre-defined pattern - for testing purposes
    pub fn gen_test_pattern(&mut self) {
        //Go through every cell and place a pre-defined value in
        for (x, row) in self.cells.iter_mut().enumerate() {
            for (y, col) in row.iter_mut().enumerate() {
                *col = (x * y) as f32;
            }
        }

        //We can cheat - we precalc the minx and maxes
        self.min = 0f32;
        self.min_pos = (0, 0);
        self.max = (self.width * self.height) as f32;
        self.max_pos = (self.width, self.height);
    }

    //Generates a random pattern - for testing purposes
    pub fn gen_random_pattern(&mut self) {
        //Go through every cell and give it a random value
        for (_x, row) in self.cells.iter_mut().enumerate() {
            for (_y, col) in row.iter_mut().enumerate() {
                *col = rand::rng().random_range(0..100) as f32;
            }
        }

        self.get_max();
        self.get_min();
    }

    //Display the map as a grid - colouring in cells based on the distance from the median
    pub fn disp_map(&mut self) -> Result<(), anyhow::Error>{
       //Display the heightmap
        helper_funcs::helper_funcs::display_magnitude_map("Heightmap",self.cells.clone(), self.width as usize, self.height as usize, ColOpt::Median)?;
        Ok(())
    }



    //Returns the flattened cells (i.e. every single cell)
    pub fn get_flattened_cells(&mut self) -> Result<Vec<f32>, anyhow::Error> {
        let mut cell_list: Vec<f32> = vec![];

        for x in 0..(self.width - 1) as usize {
            for y in 0..(self.height - 1) as usize {
                cell_list.push(self.cells[x][y]);
            }
        }

        Ok(cell_list)
    }

    //Saves the heightmap to a text file
    pub fn save_to_file(&mut self, filepath: &str) -> Result<(), anyhow::Error> {
        //Create a file
        let mut file = File::create(filepath.to_owned() + ".txt")?;

        //Format the first line to store the real bounds
        let first_line = format!("bnds:[{},{}][{},{}]\n", self.lower_coord_bounds[0],self.lower_coord_bounds[1], self.upper_coord_bounds[0],self.upper_coord_bounds[1]);
        file.write_all(first_line.as_bytes())?;


        //Iterate thorugh each row
        for row in self.cells.iter() {
            for cell in row {
                let cell_val = format!("{:?},", cell);
                file.write_all(cell_val.as_bytes())?
            }

            file.write("\n".as_ref())?;
        }

        Ok(())
    }


    //Width getter
    pub fn width(& self) -> u32{
        self.width
    }
    //Height getter
    pub fn height(&self) -> u32{
        self.height
    }

    //Extracts the bounds from a string
    fn extract_bounds(bnd_line : String) -> Result<[f32;4], anyhow::Error> {

        let mut bounds : [f32;4] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN];

        let mut bnd_cnt = 0;

        //Split the line into 3 sections
        let bnd_split = bnd_line.split("[");

        let mut ignore = true;

        //Second and third sections have the data we want
        for split in bnd_split{

            //Ignore the first split
            if ignore{
                ignore = false;
                continue
            }

            //Tokenize the second and third section (also remove the final parentheses)
            let mut str = split.trim().to_string();
            str.pop();


            for token in str.split(","){


                bounds[bnd_cnt] = token.parse()?;
                bnd_cnt = bnd_cnt + 1;
            }
        }

        Ok(bounds)
    }


    //Calculates the middle coordinates of a given cell (based on the upper/lower bounds)
    pub fn calc_cell_mid_pnt(&self, n : u32, m : u32) -> Result<[f32;2], anyhow::Error>{


        if n >= self.width{
            bail!("Error - out of width bounds!")
        }
        if m >= self.height{
            bail!("Error - out of height bounds!");
        }


        let mut mid_pnt : [f32;2] = [f32::NAN, f32::NAN];

        let cell_width = (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]).abs() / self.width as f32;
        let cell_height = (self.upper_coord_bounds[1] - self.lower_coord_bounds[1]).abs() / self.width as f32;

        //Calc the mid point with the offset
        mid_pnt[0] = (n as f32 * cell_width) + self.lower_coord_bounds[0];
        mid_pnt[1] = (m as f32 * cell_height) + self.lower_coord_bounds[1];


        Ok(mid_pnt)


    }




}

//Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(
    curr_map: &Heightmap,
    desired_map: &Heightmap,
) -> Result<Heightmap, anyhow::Error> {
    //Check the maps are the same size - if not exit
    if curr_map.height != desired_map.height || curr_map.width != curr_map.width {
        bail!("Warning - Maps are not the same size - cannot be compared");
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Heightmap = Heightmap::new(curr_map.width, curr_map.height);

    //Sweep through each cell and replace with the new map height
    for (x, row) in diff_map.cells.iter_mut().enumerate() {
        //Ignore the final enumerator (outside range of the map?)
        if x == diff_map.height as usize {
            continue;
        }

        for (y, col) in row.iter_mut().enumerate() {
            //Ignore the final enumerator (outside range of the map?)
            if y == diff_map.width as usize {
                continue;
            }

            //First index is the row number (i.e. the height)
            let diff = curr_map.cells[x][y] - desired_map.cells[x][y];

            //Not entirely sure why y and x are the opposite way rounds but hey ho
            *col = diff;
        }
    }

    Ok(diff_map)
}




///Tools used to visualise andanalyse the measured terrain
///Includes pointclouds and heightmaps
use crate::helper_funcs::helper_funcs;
use crate::helper_funcs::helper_funcs::{ColOpt, add_nan};
use anyhow::bail;
use chrono::{DateTime, Utc};
use rand::RngExt;
use realsense_sys::rs2_vertex;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};

///A point cloud
pub struct PointCloud {
    ///Vector of the points stored in xyz format
    points: Vec<[f64; 3]>,
    ///The number of points present
    no_of_points: usize,
    ///The timestamp of when the pointcloud was captured (relative to when the camera started)
    rel_timestamp: f64,
    ///The timestamp of when the pointcloud was captured (relative to the epoch)
    global_timestamp: DateTime<Utc>,
    ///The name of the file the pointcloud was loaded from (if loaded)
    filename: Option<String>,
}

impl PointCloud {
    ///Create a pointcloud from a list of points
    pub fn create_from_list(pnts: Vec<[f64; 3]>, timestamp: f64) -> Self {
        //Calculate the number of points
        let no_of_points = pnts.len();

        Self {
            points: pnts,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Utc::now(),
            filename: None,
        }
    }

    ///Create a pointcloud from a list of realsense vertices
    pub fn create_from_iter(rs2_vertex: &[rs2_vertex], timestamp: f64) -> Self {
        let mut points: Vec<[f64; 3]> = vec![];
        let mut no_of_points = 0;

        for vertex in rs2_vertex.iter() {
            let pnt = vertex.xyz;

            //check if the point is valid - if not ignore it
            if pnt == [0.0, 0.0, 0.0] || pnt[2] > 2.0 {
                continue;
            }

            points.push([pnt[0] as f64, pnt[1] as f64, pnt[2] as f64]);
            no_of_points += 1;
        }

        Self {
            points,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Utc::now(),
            filename: None,
        }
    }

    ///Create a pointcloud from a loaded file
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Create the filepath
        let filepath = filepath.to_string();

        //Open the file and create a buffer to read the lines
        let file = File::open(filepath.clone())?;
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
            let mut pnt: [f64; 3] = [f64::NAN, f64::NAN, f64::NAN];

            //Delimit the line based on commas
            for (cnt, token) in line?.split(",").enumerate() {
                pnt[cnt] = token.parse()?;
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
            filename: Some(filepath),
        })
    }

    ///Get the points in the point cloud
    pub fn points(&self) -> Vec<[f64; 3]> {
        self.points.clone()
    }

    ///Print all points in the point cloud
    pub fn print_points(&mut self) {
        for pnt in self.points.iter() {
            println!("{:?}", pnt);
        }
    }

    ///Calculate the xy bounds of the pointcloud (rectangular)
    ///Assumes z is the height
    pub fn get_bounds(&mut self) -> [f64; 4] {
        //Predefine the values we are interested in
        let mut x_min: f64 = 9999.0;
        let mut y_min: f64 = 9999.0;
        let mut x_max: f64 = -9999.0;
        let mut y_max: f64 = -9999.0;

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

    ///Rotate a pointcloud
    ///Can cheat with the mat multiplication here, we know the predefined sizes already
    ///Yaw - X
    ///Pitch - Y
    ///Roll - Z
    pub fn rotate(&mut self, yaw: f64, pitch: f64, roll: f64) {
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

    ///Translate a pointcloud with xyz coords
    pub fn translate(&mut self, x: f64, y: f64, z: f64) {
        //Iterate through every point
        for pnt in self.points.iter_mut() {
            //Transform the points using addition
            pnt[0] += x;
            pnt[1] += y;
            pnt[2] += z;
        }
    }

    ///Scale every point by the same value
    pub fn scale_even(&mut self, scale_val: f64) {
        //Iterate through every point
        for pnt in self.points.iter_mut() {
            //Transform the points using addition
            pnt[0] *= scale_val;
            pnt[1] *= scale_val;
            pnt[2] *= scale_val;
        }
    }

    pub fn trans_extr(&mut self) {}

    ///Filter a pointcloud by specifying the valid bounds
    ///Inclusive of points that lie on the boundary
    pub fn passband_filter(
        &mut self,
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
        min_z: f64,
        max_z: f64,
    ) {
        let pnts = self.points();

        let mut new_pnts: Vec<[f64; 3]> = vec![];

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

    ///Save the pointcloud to a text file
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

    ///Returns whether the pointcloud is registered as the last of a series
    ///Based on the filename
    pub fn is_end(&self) -> bool {
        //Check that the pointcloud has a filename
        if self.filename.is_none() {
            false
        } else {
            if self.filename.as_ref().unwrap().contains("_END") {
                true
            } else {
                false
            }
        }
    }

    ///Absorbs another pointcloud into this pointcloud
    ///Timestamp of previous pointcloud becomes timestamp of this one
    pub fn combine(&mut self, other: PointCloud) {
        for pnt in other.points {
            self.points.push(pnt);
            self.no_of_points += 1;
        }
    }

    ///Applies a decimation filter to the pointcloud, removing every nth point
    pub fn decimation_filter(&mut self, n: i32) {
        //Go through every point
        for i in 0..self.no_of_points {
            if (i as i32) % n == 0 {
                self.points.remove(i);
            }
        }
    }
}

///Heightmap structure - contains the size and height information for each cell
pub struct Heightmap {
    ///Number of pixel rows
    height: u32,
    ///Number of pixel columns
    width: u32,

    ///Indicates whether the grid is square or not
    square: bool,
    ///The number of cells present in the heightmap
    pub no_of_cells: u32,

    ///The point defined minimum cell height
    min: f64,
    ///The point defined maximum cell height
    max: f64,

    ///State that determines whether the minimum height needs to be checked again
    min_updated: bool,
    ///State that determines whether the maximum height needs to be checked again
    max_updated: bool,

    ///Cell location at minimum height
    min_pos: (u32, u32),
    ///Cell location at maximum height
    max_pos: (u32, u32),

    ///the 2d vector representing the cells
    cells: Vec<Vec<f64>>,

    ///The pointcloud lower bounds the hieghtmap is constructed from
    lower_coord_bounds: [f64; 2],
    ///The pointcloud upper bounds the hieghtmap is constructed from
    upper_coord_bounds: [f64; 2],

    ///Store the filepath for usage in analysis
    pub filename: String,
}

impl Default for Heightmap {
    ///Default heightmap is a blank 250x250
    fn default() -> Heightmap {
        Heightmap::new(250, 250)
    }
}

//Map tools - including display and modification etc
impl Heightmap {
    ///Create a new empty heightmap
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
            lower_coord_bounds: [0.0, 0.0],
            upper_coord_bounds: [0.0, 0.0],
            filename,
        }
    }

    ///Create a heightmap from a pointcloud (takes ownership of pcl object)
    pub fn create_from_pcl(mut pcl: PointCloud, width: u32, height: u32) -> Self {
        //Get the bounds
        let bounds = pcl.get_bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        let cells = helper_funcs::trans_to_heightmap(
            pcl.points,
            width as usize,
            height as usize,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            helper_funcs::MapGenOpt::Mean,
        )
        .expect("Failed to convert PCL to heightmap");

        //check if the heightmap is square
        let square = height == width;

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
            lower_coord_bounds: [bounds[0], bounds[2]],
            upper_coord_bounds: [bounds[1], bounds[3]],
            filename,
        }
    }

    ///Lodas a hmap file and creates a heightmap from it
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Create the filepath

        //Open the file and create a buffer to read the lines
        let file = File::open(filepath.clone())?;
        let mut line_reader = BufReader::new(file);

        //Read the first line to extract the bounds
        let mut first_line = String::new();
        line_reader.read_line(&mut first_line)?;

        let bounds = Self::extract_bounds(first_line);

        let mut bounds_res: [f64; 4] = [f64::NAN, f64::NAN, f64::NAN, f64::NAN];

        if bounds.is_ok() {
            bounds_res = bounds?
        }

        let mut height = 0;
        let mut width = 0;
        let mut width_set = false;

        let mut cells: Vec<Vec<f64>> = vec![];
        //Go through each cell and update the
        for line in line_reader.lines() {
            //Create the empty row
            let mut row: Vec<f64> = vec![];

            //Split via comma then iterate
            for token in line?.split(",") {
                if !token.is_empty() {
                    if !width_set {
                        width += 1;
                    }

                    row.push(token.parse::<f64>()?);
                }
            }

            if !width_set {
                width_set = true;
            }

            //Store the row
            cells.push(row);
            height += 1;
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
            lower_coord_bounds: [bounds_res[0], bounds_res[2]],
            upper_coord_bounds: [bounds_res[1], bounds_res[3]],
            filename: filepath,
        })
    }

    ///Creates a heightmap from a given pcl file
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

    ///Print the value contained in each cell
    pub fn print_cells(&self) {
        for row in &self.cells {
            for cell in row {
                print!("{} ", cell);
            }
            println!();
        }
    }

    ///Get the height for a given cell
    pub fn get_cell_height(&self, x: u32, y: u32) -> Result<f64, anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to read from cell that doesnt exist!");
        }

        Ok(self.cells[x as usize][y as usize])
    }

    ///Set the height of a given cell
    pub fn set_cell_height(&mut self, x: u32, y: u32, new_height: f64) {
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
        } else if new_height > self.max {
            self.max = new_height;
            self.max_pos = (x, y);
        } else if new_height < self.min {
            self.min = new_height;
            self.min_pos = (x, y);
        } else {
            println!("{0} is not larger than {1}!", new_height, self.max);
        }
    }

    ///Set the height of all cells
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

    ///Get the maximum cell height
    fn get_max(&mut self) -> f64 {
        //Check whether a new maximum is required
        if !self.max_updated {
            let mut new_max: f64 = -999.0;

            //Check every value to see if its the largest
            for row in self.cells.iter_mut() {
                for col in row.iter_mut() {
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

    ///Get the minimum cell height
    fn get_min(&mut self) -> f64 {
        //Check whether a new minimum calc is required
        if !self.min_updated {
            let mut new_min: f64 = 999.0;

            //Check every value to see if its the smallest
            for row in self.cells.iter_mut() {
                for col in row.iter_mut() {
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

    ///Generates a pre-defined pattern - for testing purposes
    pub fn gen_test_pattern(&mut self) {
        //Go through every cell and place a pre-defined value in
        for (x, row) in self.cells.iter_mut().enumerate() {
            for (y, col) in row.iter_mut().enumerate() {
                *col = (x * y) as f64;
            }
        }

        //We can cheat - we precalc the minx and maxes
        self.min = 0f64;
        self.min_pos = (0, 0);
        self.max = (self.width * self.height) as f64;
        self.max_pos = (self.width - 1, self.height - 1);
    }

    ///Generates a random pattern - for testing purposes
    pub fn gen_random_pattern(&mut self) {
        //Go through every cell and give it a random value
        for row in self.cells.iter_mut() {
            for col in row.iter_mut() {
                *col = rand::rng().random_range(0..100) as f64;
            }
        }

        self.get_max();
        self.get_min();
    }

    ///Display the map as a grid - colouring in cells based on the distance from the median
    pub fn disp_map(&mut self) -> Result<(), anyhow::Error> {
        //if true {
        //    self.interpolate_nan();
        //}

        //Display the heightmap
        helper_funcs::display_magnitude_map(
            "Heightmap",
            self.cells.clone(),
            self.width as usize,
            self.height as usize,
            ColOpt::Median,
        )?;
        Ok(())
    }

    ///Returns the flattened cells (i.e. every single cell in a single vector)
    pub fn get_flattened_cells(&mut self) -> Result<Vec<f64>, anyhow::Error> {
        let mut cell_list: Vec<f64> = vec![];

        for x in 0..(self.width - 1) as usize {
            for y in 0..(self.height - 1) as usize {
                cell_list.push(self.cells[x][y]);
            }
        }

        Ok(cell_list)
    }

    ///Saves the heightmap to a text file
    pub fn save_to_file(&mut self, filepath: &str) -> Result<(), anyhow::Error> {
        //Create a file
        let mut file = File::create(filepath.to_owned() + ".txt")?;

        //Format the first line to store the real bounds
        let first_line = format!(
            "bnds:[{},{}][{},{}]\n",
            self.lower_coord_bounds[0],
            self.lower_coord_bounds[1],
            self.upper_coord_bounds[0],
            self.upper_coord_bounds[1]
        );
        file.write_all(first_line.as_bytes())?;

        //Iterate thorugh each row
        for row in self.cells.iter() {
            for cell in row {
                let cell_val = format!("{:?},", cell);
                file.write_all(cell_val.as_bytes())?
            }

            file.write_all("\n".as_ref())?;
        }

        Ok(())
    }

    ///Width getter
    pub fn width(&self) -> u32 {
        self.width
    }
    ///Height getter
    pub fn height(&self) -> u32 {
        self.height
    }

    ///Extracts the bounds from a string
    fn extract_bounds(bnd_line: String) -> Result<[f64; 4], anyhow::Error> {
        let mut bounds: [f64; 4] = [f64::NAN, f64::NAN, f64::NAN, f64::NAN];

        let mut bnd_cnt = 0;

        //Split the line into 3 sections
        let bnd_split = bnd_line.split("[");

        let mut ignore = true;

        //Second and third sections have the data we want
        for split in bnd_split {
            //Ignore the first split
            if ignore {
                ignore = false;
                continue;
            }

            //Tokenize the second and third section (also remove the final parentheses)
            let mut str = split.trim().to_string();
            str.pop();

            for token in str.split(",") {
                bounds[bnd_cnt] = token.parse()?;
                bnd_cnt += 1;
            }
        }

        Ok(bounds)
    }

    ///Calculates the middle coordinates of a given cell (based on the upper/lower bounds)
    pub fn calc_cell_mid_pnt(&self, n: u32, m: u32) -> Result<[f64; 2], anyhow::Error> {
        if n >= self.width {
            bail!("Error - out of width bounds!")
        }
        if m >= self.height {
            bail!("Error - out of height bounds!");
        }

        let mut mid_pnt: [f64; 2] = [f64::NAN, f64::NAN];

        let cell_width =
            (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]).abs() / self.width as f64;
        let cell_height =
            (self.upper_coord_bounds[1] - self.lower_coord_bounds[1]).abs() / self.width as f64;

        //Calc the mid point with the offset
        mid_pnt[0] = (n as f64 * cell_width) + self.lower_coord_bounds[0];
        mid_pnt[1] = (m as f64 * cell_height) + self.lower_coord_bounds[1];

        Ok(mid_pnt)
    }

    ///Remove any NaN entries in the provided data matrix
    /// Achieved by interpolating the point as an average between every surrounding point
    /// Assumes a 3x3 kernel (does not account for equal or smaller data matrices)
    fn interpolate_nan(&mut self) {
        //Flags to indicate whether the data is at the edge of matrix
        let mut l_edge_flag = false;
        let mut r_edge_flag = false;
        let mut t_edge_flag = false;
        let mut b_edge_flag = false;

        //Go through every row
        for i in 0..self.height {
            if i == 0 {
                t_edge_flag = true;
            } else if i == self.height - 1 {
                b_edge_flag = true;
            } else {
                t_edge_flag = false;
                b_edge_flag = false;
            }

            //Go through every pixel in each row
            for j in 0..self.width {
                if !self.cells[i as usize][j as usize].is_nan() {
                    continue;
                }

                if j == 0 {
                    l_edge_flag = true;
                } else if j == self.width - 1 {
                    r_edge_flag = true;
                } else {
                    l_edge_flag = false;
                    r_edge_flag = false;
                }

                let mut total = 0.0;
                let mut cnt = 0;

                //Check the top row
                if !t_edge_flag {
                    total = add_nan(total, self.cells[(i - 1) as usize][j as usize]);
                    cnt += 1;

                    if !l_edge_flag {
                        total = add_nan(total, self.cells[(i - 1) as usize][(j - 1) as usize]);
                        cnt += 1;
                    }
                    if !r_edge_flag {
                        total = add_nan(total, self.cells[(i - 1) as usize][(j + 1) as usize]);
                        cnt += 1;
                    }
                }
                //Check the middle row
                if !l_edge_flag {
                    total = add_nan(total, self.cells[i as usize][(j - 1) as usize]);
                    cnt += 1;
                }
                if !r_edge_flag {
                    total = add_nan(total, self.cells[i as usize][(j + 1) as usize]);
                    cnt += 1;
                }

                //check the bottom row
                if !b_edge_flag {
                    total = add_nan(total, self.cells[(i + 1) as usize][j as usize]);
                    cnt += 1;
                    if !l_edge_flag {
                        total = add_nan(total, self.cells[(i + 1) as usize][(j - 1) as usize]);
                        cnt += 1;
                    }
                    if !r_edge_flag {
                        total = add_nan(total, self.cells[(i + 1) as usize][(j + 1) as usize]);
                        cnt += 1;
                    }
                }

                self.cells[i as usize][j as usize] = total / (cnt as f64);
            }
        }
    }
}

///Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(
    curr_map: &Heightmap,
    desired_map: &Heightmap,
) -> Result<Heightmap, anyhow::Error> {
    //Check the maps are the same size - if not exit
    if curr_map.height != desired_map.height || curr_map.width != desired_map.width {
        bail!("Warning - Maps are not the same size - cannot be compared");
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Heightmap = Heightmap::new(curr_map.width, curr_map.height);

    //Sweep through each cell and replace with the new map height
    for (m, row) in diff_map.cells.iter_mut().enumerate() {
        for (n, col) in row.iter_mut().enumerate() {
            //First index is the row number (i.e. the height)
            let diff = curr_map.cells[m][n] - desired_map.cells[m][n];

            //Not entirely sure why y and x are the opposite way rounds but hey ho
            *col = diff;
        }
    }

    Ok(diff_map)
}

///Takes a list of heightmaps that are the same size and averages them
pub fn average_heightmaps(hmap_list: &Vec<Heightmap>) -> Heightmap {
    //Get the number of heightmaps
    let no_of_heightmaps = hmap_list.len();

    //Create an empty heightmap the same size as the heightmaps in the list
    let mut avg_hmap = Heightmap::new(hmap_list[0].width(), hmap_list[0].height());

    //Go through each point in the empty heightmap and take the mean of the genned heightmaps
    for (y, row) in avg_hmap.cells.iter_mut().enumerate() {
        for (x, val) in row.iter_mut().enumerate() {
            let mut avg_val = 0.0;
            for i in 0..no_of_heightmaps {
                avg_val = add_nan(avg_val, hmap_list[i].cells[y][x]);
            }
            *val = avg_val / no_of_heightmaps as f64;
        }
    }

    avg_hmap
}

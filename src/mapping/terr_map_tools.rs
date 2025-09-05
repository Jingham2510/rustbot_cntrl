use std::fmt::Debug;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use anyhow::bail;
use chrono::{DateTime, Local};
use raylib::prelude::*;
use rand::Rng;
use realsense_sys::rs2_vertex;


//Pointcloud structure - contains purely point information
//Basically a fancy vector wrapper
pub struct PointCloud{
    //List of the points stored in xyz format
    points : Vec<[f32; 3]>,
    //The number of points present
    no_of_points: usize,

    //The timestamp of when the pointcloud was captured (relative to when the camera started)
    rel_timestamp: f64,

    global_timestamp: DateTime<Local>


}


impl PointCloud{

    //Create a pointcloud from a list of points
    pub fn create_from_list(pnts:Vec<[f32; 3]>, timestamp:f64) -> Self{
        //Calculate the number of points
        let no_of_points = pnts.len();

        Self{
            points: pnts,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Local::now()
        }
    }

    pub fn create_from_iter(rs2_vertex: &[rs2_vertex], timestamp:f64) -> Self{

        let mut points : Vec<[f32; 3]> = vec![];
        let mut no_of_points = 0;

        for vertex in rs2_vertex.iter(){

            let pnt = vertex.xyz;

            //check if the point is valid - if not ignore it
            if pnt == [0.0, 0.0, 0.0]{
                continue;
            }
            //Stopband filter (i.e. no point futher than 3 meters)
            if pnt[0] < -1.5 || pnt[0] > 1.5 || pnt[1] < -1.5 || pnt[1] > 1.5 || pnt[2] > 4.0{
                continue;
            }

            points.push(pnt);
            no_of_points = no_of_points + 1;
        }

        Self{
            points,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Local::now()
        }


    }

    pub fn points(&self) -> Vec<[f32; 3]>{
        self.points.clone()
    }

    //Print all points
    pub fn print_points(&mut self){
        for pnt in self.points.iter(){
            println!("{:?}", pnt);
        }
    }

    //Calculate the xy bounds of the pointcloud (rectangular)
    //Assumes z is the height
    pub fn get_bounds(&mut self) -> [f32; 4]{

        //Predefine the values we are interested in
        let mut x_min : f32 = 9999.0;
        let mut y_min : f32 = 9999.0;
        let mut x_max : f32 = -9999.0;
        let mut y_max : f32 = -9999.0;

        //Check each point to if it escapes the set bounds
        //If points are sorted beforehand, no need! but sorting might take a while - and how do you sort?
        for pnt in self.points.iter(){
            
            //Check x-bounds
            if pnt[0] < x_min{
                x_min = pnt[0];
            }else if pnt[0] > x_max{
                x_max = pnt[0];
            }
            
            //Check y-bounds
            if pnt[1] < y_min{
                y_min = pnt[1];
            }else if pnt[1] > y_max{
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
    pub fn rotate(&mut self, yaw :f32, pitch : f32, roll : f32){

        //Create the transform matrix rows
        let x_rot = [roll.cos() * pitch.cos(), -((roll.sin()*yaw.cos()) + (roll.cos()*pitch.sin()*yaw.sin())), (roll.sin()*yaw.sin()) + (roll.cos()*pitch.sin()*yaw.cos())];
        let y_rot = [roll.sin()*pitch.cos(), (roll.cos()*yaw.cos()) + (roll.sin()*pitch.sin()*yaw.sin()), -(roll.cos()*yaw.sin())+(roll.sin()*pitch.sin()*yaw.cos())];
        let z_rot = [-pitch.sin(), pitch.cos()*yaw.sin(), pitch.cos()*yaw.cos()];

        //Iterate through every point in the pointcloud
        for pnt in self.points.iter_mut(){

            //store the original points
            let og_x = pnt[0];
            let og_y = pnt[1];
            let og_z = pnt[2];

            //Rotate by multiplying the vector by the transform matrix
            pnt[0] = x_rot[0]*og_x + x_rot[1]*og_y + x_rot[2]*og_z;
            pnt[1] = y_rot[0]*og_x + y_rot[1]*og_y + y_rot[2]*og_z;
            pnt[2] = z_rot[0]*og_x + z_rot[1]*og_y + z_rot[2]*og_z;
        }



    }


    //Translate a pointcloud with xyz coords - inplace
    pub fn translate(&mut self, x: f32, y:f32, z : f32){

        //Iterate through every point
        for pnt in self.points.iter_mut(){

            //Transform the points using addition
            pnt[0] = pnt[0] + x;
            pnt[1] = pnt[1] + y;
            pnt[2] = pnt[2] + z;

        }


    }


    //Save the pointcloud to an ASCII file

    pub fn save_to_file(&mut self, filename: &str) -> Result<(), anyhow::Error>{

        //Create a file
        let mut file = File::create( "dump/".to_owned() + filename + ".txt")?;

        //Save the timestamp from the frame as the first line;
        let datetime_fmt = self.global_timestamp.format("%Y-%m-%d %H:%M:%S\n").to_string();

        file.write_all(datetime_fmt.as_bytes())?;

        //Save the cloud points on seperate lines
        for i in 0..self.no_of_points{
            
            let pnt = format!("{:?},{:?},{:?}\n", self.points[i][0], self.points[i][1], self.points[i][2]);
            
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
    no_of_cells: u32,

    //The min and max cell heights
    min: f32,
    max: f32,
    //keeps track of whether the min or max need to be updated again
    min_updated: bool,
    max_updated: bool,

    min_pos: (u32, u32),
    max_pos: (u32, u32),

    //the 2d vector representing the cells
    cells: Vec<Vec<f32>>,
}

//Map tools - including display and modification etc
impl Heightmap {
    pub fn new(width: u32, height: u32) -> Self {
        //Initialise new variables for the object
        let mut square_check = false;

        if height == width {
            square_check = true;
        }

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
        }
    }


    //Create a heightmap from a pointcloud (takes ownership of pcl object)
    pub fn create_from_pcl(mut pcl : PointCloud, width : u32, height: u32, density :bool) -> Self{
        //Get the bounds
        let bounds = pcl.get_bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        //Create a matrix which tracks the number of points in each cell
        let mut cell_pnt_cnt = vec![vec![0.0f32; height as usize]; width as usize];

        //Create the empty cell matrix
        //Defined as NaN so that empty spots can be identified
        let mut cells = vec![vec![f32::NAN; height as usize]; width as usize];

        //Check each points and direct it to a cell (updating the average height)
        for pnt in pcl.points{


            let mut n = 0;
            let mut m = 0;

            let mut n_fnd = false;
            let mut m_fnd = false;

            //Find the horizontal pos
            while !n_fnd{

                if pnt[0] < ((total_width / width as f32) * n as f32) + bounds[0] {
                    n_fnd = true;
                }else{
                    n = n + 1;
                }

                //Check if end pos
                if n == (width-1) as usize{
                    n_fnd = true;
                }
            }

            //Find the vertical pos
            while !m_fnd{

                if pnt[1] < ((total_height / height as f32) * m as f32) + bounds[2]{
                    m_fnd = true;
                }else{
                    m = m +1;
                }
                //Check if end pos
                if m == (height - 1) as usize{
                    m_fnd = true;
                }
            }

            //Set the pcl to 0.0 at the start to avoid mem errors
            if cell_pnt_cnt[n][m] == 0.0{
                cells[n][m] = 0.0;
            }

            //Calculate the updated cumulitve average
            cells[n][m] = (pnt[2] + cell_pnt_cnt[n][m] as f32 * cells[n][m])/(cell_pnt_cnt[n][m] + 1.0);
            //Increase the point count
            cell_pnt_cnt[n][m] = cell_pnt_cnt[n][m] + 1.0;

        }

        let square;
        //check if the heightmap is square
        if height == width{
             square = true;
        }else{
            square = false;
        }

        if !density {
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
                cells
            }
        }else{
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
                cells : cell_pnt_cnt
            }
        }
    }


    //Creates a heightmap from a hmap file
    //TODO: Handle file nout found error
    pub fn create_from_file(filename : &str) -> Result<Self, anyhow::Error>{

        //Create the filepath
        let filepath = format!("dump/{}.txt", filename.to_string());

        //Open the file and create a buffer to read the lines
        let file = File::open(filepath)?;
        let line_reader = BufReader::new(file);


        let mut height = 0;
        let mut width = 0;
        let mut width_set = false;


        let mut cells : Vec<Vec<f32>> = vec![];

        //Go through each cell and update the
        for line in line_reader.lines(){
           //Create the empty row
            let mut row : Vec<f32> = vec![];

            //Split via comma then iterate
            for token in line?.split(","){

                if !width_set{
                    width = width + 1;
                }

                //Check the slot isnt empty
                if !token.is_empty(){
                    row.push(token.parse::<f32>()?);
                }

            }

            if !width_set{
                width_set = true;
            }

            //Store the row
            cells.push(row);
            height = height + 1;
        }

        //TODO - Do some checks (i.e. every row is the same length)



        //Check to see if the grid is square
        let mut square= false;
        if height == width{
            square = true;
        }


        Ok(Self{
            height,
            width,
            square,
            no_of_cells: height*width,
            min: 999.0,
            max: -999.0,
            min_updated: false,
            max_updated: false,
            min_pos: (0, 0),
            max_pos: (0, 0),
            cells
        })
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
    pub fn gen_random_pattern(& mut self){
        //Go through every cell and give it a random value
        for (_x, row) in self.cells.iter_mut().enumerate() {
            for (_y, col) in row.iter_mut().enumerate() {
                *col = rand::rng().random_range(0..100) as f32;
            }
        }

        self.get_max();
        self.get_min();

    }

    //Display the map as a grid - colouring in cells based on the height
    //TODO : ADD NON-RELATIVE COLOURING (I.E. BASED ON ACTUAL HEIGHT)
    pub fn disp_map(&mut self) {



        //Calcualte grid sizes etc
        //Square grid could be awkward if rectangular map but unlikely event

        //GUI window size
        const WINDOW_WIDTH: f32 = 1024.0;
        const WINDOW_HEIGHT: f32 = 768.0;

        //Precalced to save time
        const WINDOW_WIDTH_START: f32 = WINDOW_WIDTH * 0.1;
        const WINDOW_WIDTH_END: f32 = WINDOW_WIDTH * 0.9;
        const WINDOW_HEIGHT_START: f32 = WINDOW_HEIGHT * 0.1;
        const WINDOW_HEIGHT_END: f32 = WINDOW_HEIGHT * 0.9;

        //GUI grid width/height
        let grid_disp_width: f32 = WINDOW_WIDTH * 0.8;
        let grid_disp_height: f32 = WINDOW_HEIGHT * 0.8;

        //Line thickness
        const LINE_THICKNESS: f32 = 3.0;

        //calculate the cell width
        let cell_width: f32 =
            (grid_disp_width - ((self.width as f32 + 2.0) * LINE_THICKNESS)) / (self.width as f32);

        let cell_height: f32 = (grid_disp_height - ((self.height as f32 + 2.0) * LINE_THICKNESS))
            / (self.height as f32);

        //Create the window
        let (mut rl, thread) = raylib::init()
            .size(WINDOW_WIDTH as i32, WINDOW_HEIGHT as i32)
            .title("Terrain map")
            //Set log report level
            .log_level(TraceLogLevel::LOG_WARNING)
            .build();

        while !rl.window_should_close() {
            //Create the drawing tool
            let mut d = rl.begin_drawing(&thread);

            //Set the background colour
            d.clear_background(Color::WHITE);

            //Draw the grid outline
            d.draw_rectangle_lines_ex(
                Rectangle::new(
                    WINDOW_WIDTH_START,
                    WINDOW_HEIGHT_START,
                    grid_disp_width,
                    grid_disp_height,
                ),
                LINE_THICKNESS,
                Color::BLACK,
            );

            //Draw the grid lines
            for i in 1..self.width {
                let curr_x = (WINDOW_WIDTH_START + LINE_THICKNESS)
                    + (i as f32 * (cell_width + LINE_THICKNESS));

                d.draw_line_ex(
                    Vector2::new(curr_x, WINDOW_HEIGHT_START),
                    Vector2::new(curr_x, WINDOW_HEIGHT_END),
                    LINE_THICKNESS,
                    Color::BLACK,
                );
            }
            for i in 1..self.height {
                let curr_y = (WINDOW_HEIGHT_START + LINE_THICKNESS)
                    + (i as f32 * (cell_height + LINE_THICKNESS));

                d.draw_line_ex(
                    Vector2::new(WINDOW_WIDTH_START, curr_y),
                    Vector2::new(WINDOW_WIDTH_END, curr_y),
                    LINE_THICKNESS,
                    Color::BLACK,
                );
            }

            //Fill in the cell colours based on height values -----
            //Base colour gradient range on largest and minimum value (with a median to act as a neutral value)
            if !self.min_updated{
                self.get_min();
            }
            if !self.max_updated{
                self.get_max();
            }

            let median = (self.max + self.min) / 2.0;

            //Go through every cell and draw a coloured rectangle to represent it
            for (x, row) in self.cells.iter_mut().enumerate() {
                //Calculate the start point of the rectangle
                let curr_x = (WINDOW_WIDTH_START + (LINE_THICKNESS))
                    + (x as f32 * (cell_width + LINE_THICKNESS));

                for (y, col) in row.iter_mut().enumerate() {
                    //Calc the starting height
                    let curr_y = (WINDOW_HEIGHT_START + (LINE_THICKNESS))
                        + (y as f32 * (cell_height + LINE_THICKNESS));

                    //Calculate the colour
                    let cell_col: Color;

                    //println!("{0}", self.max);

                    //If the value in the cell is unknown - paint it black
                    if col.is_nan() {
                        cell_col = Color::BLACK;
                    } else if *col <= median {
                        cell_col = Color::new(
                            (256.0 * (1.0 - ((*col - self.min) / (median - self.min)))) as u8,
                            (256.0 * ((*col - self.min) / (median - self.min))) as u8,
                            0,
                            255,
                        );
                    } else {
                        cell_col = Color::new(
                            0,
                            (256.0 * (1.0 - ((*col - median) / (self.max - median)))) as u8,
                            (256.0 * ((*col - median) / (self.max - median))) as u8,
                            255,
                        );
                    }

                    //Create the coloured rectangle in the grid
                    d.draw_rectangle(
                        curr_x as i32,
                        curr_y as i32,
                        (cell_width + 2.0 * LINE_THICKNESS) as i32,
                        (cell_height + 2.0 * LINE_THICKNESS) as i32,
                        cell_col,
                    );
                }
            }
        }
    }


    pub fn save_to_file(&mut self, filename : &str) -> Result<(), anyhow::Error>{


        //Create a file
        let mut file = File::create( "dump/".to_owned() + filename + ".txt")?;

        //Iterate thorugh each row
        for row in self.cells.iter(){
            for cell in row{
                let cell_val = format!("{:?},", cell);
                file.write_all(cell_val.as_bytes())?
            }

            file.write("\n".as_ref())?;
        }



        Ok(())

    }



}

//Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(curr_map: &Heightmap, desired_map: &Heightmap) -> Result<Heightmap, anyhow::Error> {
    //Check the maps are the same size - if not exit
    if curr_map.height != desired_map.height || curr_map.width != curr_map.width {
        bail!("Warning - Maps are not the same size - cannot be compared");
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Heightmap = Heightmap::new(curr_map.height, curr_map.width);

    //Sweep through each cell and replace with the new map height
    for (x, row) in diff_map.cells.iter_mut().enumerate() {
        for (y, col) in row.iter_mut().enumerate() {

            let diff = curr_map.cells[y][x] - desired_map.cells[y][x];


            //Not entirely sure why y and x are the opposite way rounds but hey ho
            *col = diff;
        }
    }

    Ok(diff_map)
}

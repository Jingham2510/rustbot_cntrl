use raylib::prelude::*;
use rand::Rng;
use realsense_sys::rs2_vertex;

//Pointcloud structure - contains purely point information
//Basically a fancy vector wrapper 
pub struct PointCloud{
    //List of the points stored in xyz format
    points : Vec<[f32; 3]>,
    //The number of points present 
    no_of_points: usize
}


impl PointCloud{

    //Create a pointcloud from a list of points
    pub fn create_from_list(pnts:Vec<[f32; 3]>) -> Self{                
        //Calculate the number of points
        let no_of_points = pnts.len();
        
        Self{
            points: pnts,
            no_of_points
        }        
    }
    
    pub fn create_from_iter(rs2_vertex: &[rs2_vertex]) -> Self{
        
        let mut points : Vec<[f32; 3]> = vec![];
        let mut no_of_points = 0;
        
        for vertex in rs2_vertex.iter(){
            
            let pnt = vertex.xyz;
            
            //check if the point exists
            if pnt == [0.0, 0.0, 0.0]{
                continue;
            }                  
            
            points.push(pnt);
            no_of_points = no_of_points + 1;
        }        
        
        Self{
            points,
            no_of_points
        }
        
        
    }
    
    //Print all points
    pub fn print_points(&mut self){
        for i in 0..self.no_of_points{
            println!("{:?}", self.points[i]);
        }
    }
    
    //Calculate the bounds of a pointcloud

    //Rotate a pointcloud

    //Translate a pointcloud

    

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
            println!("Map is square");
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

    pub fn print_cells(&self) {
        for row in &self.cells {
            for cell in row {
                print!("{} ", cell);
            }
            print!("\n");
        }
    }

    //Get the height for a given cell
    pub fn get_cell_height(&self, x: u32, y: u32) -> Option<f32> {
        if x > self.width || y > self.height {
            println!("Warning - attempting to read from cell that doesnt exist!");
            return None;
        }

        Option::from(self.cells[x as usize][y as usize])
    }

    //Set the height of a given cell
    pub fn set_cell_height(&mut self, x: u32, y: u32, new_height: f32) {
        if x > self.width || y > self.height {
            println!("Warning - attempting to write to cell that doesnt exist!");
            return;
        }

        self.cells[x as usize][y as usize] = new_height;

        //Check that the cell doesnt store the min or the max
        if ((x, y) == self.max_pos) {
            self.get_max();
        } else if (x, y) == self.min_pos {
            self.get_min();
        } else {
            if (new_height > self.max) {
                self.max = new_height;
                self.max_pos = (x, y);
            } else if (new_height < self.min) {
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
        if (!self.max_updated) {
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
            .build();

        while !rl.window_should_close() {
            //Create the drawing tool
            let mut d = rl.begin_drawing(&thread);

            //Set the background colour
            d.clear_background(Color::WHITE);

            //Draw the grid outline
            d.draw_rectangle_lines_ex(
                (Rectangle::new(
                    (WINDOW_WIDTH_START),
                    WINDOW_HEIGHT_START,
                    grid_disp_width,
                    grid_disp_height,
                )),
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
                    } else if (*col <= median) {
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
}

//Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(curr_map: &Heightmap, desired_map: &Heightmap) -> Option<Heightmap> {
    //Check the maps are the same size - if not exit
    if curr_map.height != desired_map.height || curr_map.width != curr_map.width {
        println!("Warning - Maps are not the same size - cannot be compared");
        return None;
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Heightmap = Heightmap::new(curr_map.height, curr_map.width);

    //Sweep through each cell and replace with the new map height
    for (x, row) in diff_map.cells.iter_mut().enumerate() {
        for (y, col) in row.iter_mut().enumerate() {
            //Not entirely sure why y and x are the opposite way rounds but hey ho
            *col = curr_map.cells[y][x] - desired_map.cells[y][x];
        }
    }

    Option::from(diff_map)
}

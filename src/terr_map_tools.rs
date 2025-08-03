

//Map structure - contains the size and height information for each cell
pub struct Map {

    //Height and width of the terrain map
    height: i32,
    width: i32,

    //Indicates whether the grid is square or not
    square: bool,

    //the 2d vector representing the cells
    cells: Vec<Vec<f32>>
}

//Map tools - including display and modification etc
impl Map {

    pub fn new( width: i32, height: i32) -> Self{

        //Initialise new variables for the object
        let mut square_check = false;


        if(height == width){
            square_check = true;
        }

        //Generate an empty cell bed of height*width size
        Self{height, width, square: square_check, cells: vec![vec![0.0; width as usize]; height as usize]}
    }

    pub fn print_cells(&self){
        for row in &self.cells{
            for cell in row{
                print!("{} ", cell);
            }
            print!("\n");
        }
    }

    //Get the height for a given cell
    pub fn get_cell_height(&self, x: usize, y: usize) -> f32{
        self.cells[x][y]
    }

    //Set the height of a given cell
    pub fn set_cell_height(&mut self, x : usize, y: usize, new_height: f32){
        self.cells[x][y] = new_height;
    }

    //Set the height of all cells (utilising the set cell height function)
    pub fn set_map(&mut self, new_heights: Map){
        //Sweep through each cell and replace with the new map height
        for x in 0..self.width{
            for y in 0..self.height{
                //Cast to usize here because can't access the vectors using i32
                self.cells[x as usize][y as usize] = new_heights.cells[x as usize][y as usize]
            }
        }

    }


}

//Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(curr_map: &Map, desired_map: &Map) -> Option<Map>{

    //Check the maps are the same size - if not exit
    if(curr_map.height != desired_map.height || curr_map.width != curr_map.height){
        println!("Warning - Maps are not the same size - cannot be compared");
        return None
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Map = Map::new(curr_map.height, curr_map.width);

    //TODO: CHECK THE COMP MAPS THING

    //Sweep through cells and get the differences
    //NOT ABSOLUTE - we want to know the over/under
    for x in 0..diff_map.width{
        for y in 0..diff_map.height{
            diff_map.cells[x as usize][y as usize] = curr_map.cells[x as usize][y as usize] - desired_map.cells[x as usize][y as usize];
        }
    }


    Option::from(diff_map)
    
    



}


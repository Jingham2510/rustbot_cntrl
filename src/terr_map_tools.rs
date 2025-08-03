

//Map structure - contains the size and height information for each cell
pub struct Map {

    //Height and width of the terrain map
    height: u32,
    width: u32,

    //Indicates whether the grid is square or not
    square: bool,
    
    no_of_cells: u32,

    //the 2d vector representing the cells
    cells: Vec<Vec<f32>>
}

//Map tools - including display and modification etc
impl Map {

    pub fn new( width: u32, height: u32) -> Self{

        //Initialise new variables for the object
        let mut square_check = false;


        if(height == width){
            square_check = true;
        }

        //Generate an empty cell bed of height*width size
        Self{height, width, square: square_check, no_of_cells: height*width ,cells: vec![vec![0.0; height as usize]; width as usize]}
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
    pub fn get_cell_height(&self, x: u32, y: u32) -> Option<f32>{

        if (x > self.width || y > self.height){
            println!("Warning - attempting to read from cell that doesnt exist!");
            return None
        }

        Option::from(self.cells[x as usize][y as usize])
    }

    //Set the height of a given cell
    pub fn set_cell_height(&mut self, x : u32, y: u32, new_height: f32){

        if (x > self.width || y > self.height){
            println!("Warning - attempting to write to cell that doesnt exist!");
            return
        }

        self.cells[x as usize][y as usize] = new_height;
    }

    //Set the height of all cells (utilising the set cell height function)
    pub fn set_map(&mut self, new_heights: Map){
        //Sweep through each cell and replace with the new map height
        for (x, row) in self.cells.iter_mut().enumerate(){
            for (y, col) in row.iter_mut().enumerate(){

                *col = new_heights.cells[x][y]
            }
        }

    }


}

//Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(curr_map: &Map, desired_map: &Map) -> Option<Map>{

    //Check the maps are the same size - if not exit
    if(curr_map.height != desired_map.height || curr_map.width != curr_map.width){
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

        }
    }

    //Sweep through each cell and replace with the new map height
    for (x, row) in diff_map.cells.iter_mut().enumerate(){
        for (y, col) in row.iter_mut().enumerate(){
            //Not entirely sure why y and x are the opposite way rounds but hey ho
            *col= curr_map.cells[y][x] - desired_map.cells[y][x];
        }
    }


    Option::from(diff_map)
    
    



}


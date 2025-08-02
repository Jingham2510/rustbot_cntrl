

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
        Self{height, width, square: square_check, cells: vec![vec![0.0; height as usize]; width as usize]}
    }

    pub fn print_cells(self){
        for row in self.cells{
            for cell in row{
                print!("{} ", cell);
            }
            print!("\n");
        }
    }

    //TODO: CHANGE CELL VALUE, CHANGE ALL CELL VALUES

    //Get the height for a given cell
    pub fn get_cell_height(self, x: usize, y: usize) -> f32{
        self.cells[x][y]
    }


}

//Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(curr_map: Map, desired_map: Map) -> Option<Map>{

    //Check the maps are the same size - if not exit
    if(curr_map.height != desired_map.height || curr_map.width != curr_map.height){
        println!("Warning - Maps are not the same size - cannot be compared");
        None
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Map = Map::new(curr_map.height, curr_map.width);

    //TODO: FINISH SOME MAP UTILS

    //Access cell info
    for row in diff_map{
        for cell in row{
            //get the difference
            cell = 1;            
        }       
        
    }

    Option::from(diff_map)
    
    



}


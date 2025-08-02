

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

    pub fn new(height: i32, width: i32) -> Self{

        //Initialise new variables for the object
        let mut square_check = false;


        if(height == width){
            square_check = true;
        }

        //Generate an empty cell bed of height*width size
        Self{height, width, square: square_check, cells: vec![vec![0.0; width as usize]; height as usize]}
    }

    pub fn print_cells(self){

        for row in self.cells{
            for cell in row{
                print!("{} ", cell);
            }
            print!("\n");
        }

    }


}


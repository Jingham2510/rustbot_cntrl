//A collection of helper functions based around calculating and displaying 2.5D maps


use raylib::callbacks::TraceLogLevel;
use raylib::color::Color;
use raylib::consts::MouseButton;
use raylib::drawing::RaylibDraw;
use raylib::math::{Rectangle, Vector2};
use crate::helper_funcs::helper_funcs;

//Transforms 3 point data into a 2.5d heightmap ( where the 3rd data point is the "height"/intensity)
pub fn trans_to_heightmap(data : Vec<[f32;3]>, width: usize, height : usize, total_width : f32, total_height : f32, min_x_bnd : f32, min_y_bnd : f32) -> Result<Vec<Vec<f32>>, anyhow::Error>{

    //Create a matrix which tracks the number of trajectory points in each cell
    let mut cell_pnt_cnt = vec![vec![0.0f32; height]; width];

    //Create the empty cell matrix
    //NaN spots are areas with 0 action
    let mut cells = vec![vec![f32::NAN; height]; width];

    //Check where the trajectory lies within the cell space - copied from heightmap generation
    //Check each points and direct it to a cell (updating the average height)
    for pnt in data{
        let mut n = 0;
        let mut m = 0;

        let mut n_fnd = false;
        let mut m_fnd = false;

        //Find the horizontal pos
        while !n_fnd {
            if pnt[0] < ((total_width / width as f32) * n as f32) + min_x_bnd {
                n_fnd = true;
            } else {
                n = n + 1;
            }

            //Check if end pos
            if n == (width - 1) {
                n_fnd = true;
            }
        }

        //Find the vertical pos
        while !m_fnd {
            if pnt[1] < ((total_height / height as f32) * m as f32) + min_y_bnd {
                m_fnd = true;
            } else {
                m = m + 1;
            }
            //Check if end pos
            if m == (height - 1) {
                m_fnd = true;
            }
        }

        //Set the pcl to 0.0 at the start to avoid mem errors
        if cell_pnt_cnt[n][m] == 0.0 {
            cells[n][m] = 0.0;
        }

        //Calculate the updated cumulitve average
        cells[n][m] =
            (pnt[2] + cell_pnt_cnt[n][m] * cells[n][m]) / (cell_pnt_cnt[n][m] + 1.0);
        //Increase the point count
        cell_pnt_cnt[n][m] = cell_pnt_cnt[n][m] + 1.0;
    }

    Ok(cells)

}


//Returns the median value of a matrix
//Designed to be used with the 2.5D heightmaps
pub fn get_min_med_max(data : &Vec<Vec<f32>>) -> (f32, f32, f32){

    let mut max : f32= -9999.0;
    let mut min : f32= 9999.0;

    //Get the max min values
    for val in data.iter().flatten(){

        if val > &max {
            max = *val;
        }else if val < &min {
            min = *val;
        }
    }

    //Get the mid point between the max/min
    (min, max - (max-min/2.0).abs(), max)
}


pub enum ColOpt{
    Median,
    Intensity,
    InvIntensity,
    Uniform
}

//Display a magnitude map (i.e. xy - posiitons, z - magnitude)
//Options for different colour schemes
pub fn display_magnitude_map(wind_title : &str, mut data:Vec<Vec<f32>>, width : usize, height : usize, col_opt : ColOpt) -> Result<(), anyhow::Error>{
    //Constants to determine generic window size
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
        (grid_disp_width - ((width as f32 + 2.0) * LINE_THICKNESS)) / (width as f32);

    let cell_height: f32 = (grid_disp_height - ((height as f32 + 2.0) * LINE_THICKNESS))
        / (height as f32);

    //Get key data points
    let (min, med_val, max) = get_min_med_max(&data);

    //Data to display
    let mut data_height = f32::NAN;

    //Create the window
    let (mut rl, thread) = raylib::init()
        .size(WINDOW_WIDTH as i32, WINDOW_HEIGHT as i32)
        .title(wind_title)
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
        for i in 1..width {
            let curr_x = (WINDOW_WIDTH_START + LINE_THICKNESS)
                + (i as f32 * (cell_width + LINE_THICKNESS));

            d.draw_line_ex(
                Vector2::new(curr_x, WINDOW_HEIGHT_START),
                Vector2::new(curr_x, WINDOW_HEIGHT_END),
                LINE_THICKNESS,
                Color::BLACK,
            );
        }
        for i in 1..height {
            let curr_y = (WINDOW_HEIGHT_START + LINE_THICKNESS)
                + (i as f32 * (cell_height + LINE_THICKNESS));

            d.draw_line_ex(
                Vector2::new(WINDOW_WIDTH_START, curr_y),
                Vector2::new(WINDOW_WIDTH_END, curr_y),
                LINE_THICKNESS,
                Color::BLACK,
            );
        }
        //Go through every cell and draw a coloured rectangle to represent it
        for (x, row) in data.iter_mut().enumerate() {
            //Calculate the start point of the rectangle
            let curr_x = (WINDOW_WIDTH_START + (LINE_THICKNESS))
                + (x as f32 * (cell_width + LINE_THICKNESS));

            for (y, val) in row.iter_mut().enumerate() {
                //Calc the starting height
                let curr_y = (WINDOW_HEIGHT_START + (LINE_THICKNESS))
                    + (y as f32 * (cell_height + LINE_THICKNESS));

                //Calculate the colour
                let cell_col: Color;


                //If the value in the cell is unknown - paint it black
                //If the value in the cell is unknown - paint it black
                if val.is_nan() {
                    cell_col = Color::BLACK;
                } else {
                    match col_opt {
                        ColOpt::Median => { cell_col = median_cell_col(*val, min, med_val, max) }
                        ColOpt::Intensity => { cell_col = intensity_cell_col(*val, min, max) }
                        ColOpt::InvIntensity => { cell_col = inv_intensity_cell_col(*val, min, max) }
                        ColOpt::Uniform => { cell_col = Color::new(255, 255, 255, 255) }
                    }
                }

                //Create the coloured rectangle in the grid
                d.draw_rectangle(
                    curr_x as i32,
                    curr_y as i32,
                    (cell_width + 2.0 * LINE_THICKNESS) as i32,
                    (cell_height + 2.0 * LINE_THICKNESS) as i32,
                    cell_col,
                );

                //Draw the text info
                let data_str = format!("Height: {}", data_height);
                d.draw_text(
                    &*data_str,
                    WINDOW_WIDTH_START as i32,
                    WINDOW_HEIGHT_END as i32 + 25,
                    42,
                    Color::BLACK,
                );
            }
        }

        //Check if the mouse is clicked
        if d.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT) {
            let m_pos = d.get_mouse_position();

            //Check if inside the bounds of the heightmap graphics
            if m_pos.x < WINDOW_WIDTH_START
                || m_pos.x > WINDOW_WIDTH_END
                || m_pos.y < WINDOW_HEIGHT_START
                || m_pos.y > WINDOW_HEIGHT_END
            {
                continue;
            }

            //Remove window placement offset, then take the percentage across the screen
            let perc_x =
                (m_pos.x - WINDOW_WIDTH_START) / (WINDOW_WIDTH_END - WINDOW_WIDTH_START);
            let perc_y =
                (m_pos.y - WINDOW_HEIGHT_START) / (WINDOW_HEIGHT_END - WINDOW_HEIGHT_START);

            //Calculate which cell the position corresponds to
            let x_cell = (perc_x * width as f32).floor() as usize;
            let y_cell = (perc_y * height as f32).floor() as usize;
            data_height = data[x_cell][y_cell];
        }
    }
    Ok(())
}



//Calculate the colour gradient based on +/- distance from median
fn median_cell_col(val : f32, min : f32, med_val : f32, max : f32) -> Color{

    let cell_col : Color;

    if val <= med_val {
        cell_col = Color::new(
            (255.0 * (1.0 - ((val - min) / (med_val - min)))) as u8,
            (255.0 * ((val - min) / (med_val- min))) as u8,
            0,
            255,
        );
    } else {
        cell_col = Color::new(
            0,
            (255.0 * (1.0 - ((val - med_val) / (max - med_val)))) as u8,
            (255.0 * ((val - med_val) / (max - med_val))) as u8,
            255,
        );
    }

    cell_col
}

//Calculate the colour as a percentage of the distance from max value
fn intensity_cell_col(val: f32, min : f32, max : f32) -> Color{

    Color::new(
        255.0 as u8,
        255.0 as u8,
        255.0 as u8,
        (255.0 * (val - min)/(max - min)) as u8
    )

}

//Calculate the color as a percentage of the distance from the min value
fn inv_intensity_cell_col(val: f32, min : f32, max : f32) -> Color{

    Color::new(
        255.0 as u8,
        255.0 as u8,
        255.0 as u8,
        (255.0 * (1.0 - (val - min)/(max - min))) as u8
    )

}

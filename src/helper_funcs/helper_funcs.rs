//A collection of helper functions based around calculating and displaying 2.5D maps

use anyhow::bail;
use raylib::callbacks::TraceLogLevel;
use raylib::color::Color;
use raylib::consts::MouseButton;
use raylib::drawing::RaylibDraw;
use raylib::math::{Rectangle, Vector2};

pub enum MapGenOpt {
    Mean,
    //Warning! Median will be a slow process - you have to sort the list of points in each section
    Median,
    Min,
    Max,
}

//Transforms 3 point data into a 2.5d heightmap ( where the 3rd data point is the "height"/intensity)
pub fn trans_to_heightmap(
    data: Vec<[f64; 3]>,
    width: usize,
    height: usize,
    total_width: f64,
    total_height: f64,
    min_x_bnd: f64,
    min_y_bnd: f64,
    opt: MapGenOpt,
) -> Result<Vec<Vec<f64>>, anyhow::Error> {
    //Create the empty cell matrix
    //NaN spots are areas with 0 action
    let mut cells_pnt_list = vec![vec![vec![]; height]; width];

    //Check where the trajectory lies within the cell space - copied from heightmap generation
    //Check each points and direct it to a cell (updating the average height)
    for pnt in data {
        let mut n = 0;
        let mut m = 0;

        let mut n_fnd = false;
        let mut m_fnd = false;


        //Find the horizontal pos
        while !n_fnd {
            if pnt[0] < (((total_width / width as f64) * n as f64) + min_x_bnd) {
                n_fnd = true;
            } else {
                n += 1;
            }

            //Check if end pos
            if n == (width - 1) {
                n_fnd = true;
            }
        }

        //Find the vertical pos
        while !m_fnd {
            if pnt[1] < (((total_height / height as f64) * m as f64) + min_y_bnd) {
                m_fnd = true;
            } else {
                m += 1;
            }
            //Check if end pos
            if m == (height - 1) {
                m_fnd = true;
            }
        }

        //add the point to the cell point list
        cells_pnt_list[n][m].push(pnt[2]);
    }

    //Calculate the height of each cell based on the chosen hmap option
    let mut cells: Vec<Vec<f64>> = vec![vec![]; width];

    //Iterate through each point list
    for (i, pnt_list) in cells_pnt_list.iter_mut().enumerate() {
        for pnts in pnt_list {
            //If the cell is NAN - keep it as a null cell
            if pnts.iter().any(|x| x.is_nan()) || pnts.is_empty() {
                cells[i].push(f64::NAN);
                continue;
            }

            let mut pnt_to_add: f64 = 0.0;

            //Determine the value of the cell bsaed on the provided heightmap options
            match opt {
                MapGenOpt::Mean => {
                    let mut cnt = 0;

                    //sum all the points in the list
                    for pnt in pnts {
                        pnt_to_add += *pnt;
                        cnt += 1;
                    }

                    //Divide by the length of the list
                    pnt_to_add /= cnt as f64;
                }

                MapGenOpt::Median => {
                    //Sort the list - unstable because we dont care about initial order of indices and also its faster
                    pnts.sort_unstable_by(f64::total_cmp);

                    if pnts.len() == 1 {
                        pnt_to_add = pnts[0]
                    } else {
                        //check if length is odd/even
                        let even = { pnts.len() % 2 == 0 };

                        //Identify the median value
                        if even {
                            pnt_to_add =
                                (pnts[pnts.len().div_ceil(2)] + pnts[(pnts.len() - 1) / 2]) / 2.0
                        } else {
                            pnt_to_add = pnts[(pnts.len()) / 2]
                        }
                    }
                }

                MapGenOpt::Max => {
                    //Set the max default
                    pnt_to_add = -9999.0;
                    //Iterate through every point and get the max
                    for pnt in pnts {
                        if *pnt > pnt_to_add {
                            pnt_to_add = *pnt;
                        }
                    }
                }

                MapGenOpt::Min => {
                    //Iterate through every point and get the min
                    //Set the min default
                    pnt_to_add = 9999.0;
                    //Iterate through every point and get the max
                    for pnt in pnts {
                        if *pnt < pnt_to_add {
                            pnt_to_add = *pnt;
                        }
                    }
                }

                _ => {
                    bail!("Invalid map gen option")
                }
            }

            cells[i].push(pnt_to_add);
        }
    }

    Ok(cells)
}

//Returns the median value of a matrix
//Designed to be used with the 2.5D heightmaps
pub fn get_min_med_max(data: &Vec<Vec<f64>>) -> (f64, f64, f64) {
    let mut max: f64 = -9999.0;
    let mut min: f64 = 9999.0;

    //Get the max min values
    for val in data.iter().flatten() {
        if val > &max {
            max = *val;
        } else if val < &min {
            min = *val;
        }
    }

    //Get the mid point between the max/min
    (min, max - (max - min / 2.0).abs(), max)
}

pub enum ColOpt {
    Median,
    Intensity,
    InvIntensity,
    Uniform,
}

//Display a magnitude map (i.e. xy - posiitons, z - magnitude)
//Options for different colour schemes
pub fn display_magnitude_map(
    wind_title: &str,
    mut data: Vec<Vec<f64>>,
    width: usize,
    height: usize,
    col_opt: ColOpt,
) -> Result<(), anyhow::Error> {
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

    let cell_height: f32 =
        (grid_disp_height - ((height as f32 + 2.0) * LINE_THICKNESS)) / (height as f32);

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
            let curr_x =
                (WINDOW_WIDTH_START + LINE_THICKNESS) + (i as f32 * (cell_width + LINE_THICKNESS));

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
                        ColOpt::Median => cell_col = median_cell_col(*val, min, med_val, max),
                        ColOpt::Intensity => cell_col = intensity_cell_col(*val, min, max),
                        ColOpt::InvIntensity => cell_col = inv_intensity_cell_col(*val, min, max),
                        ColOpt::Uniform => cell_col = Color::new(255, 255, 255, 255),
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
                    &data_str,
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
            let perc_x = (m_pos.x - WINDOW_WIDTH_START) / (WINDOW_WIDTH_END - WINDOW_WIDTH_START);
            let perc_y =
                (m_pos.y - WINDOW_HEIGHT_START) / (WINDOW_HEIGHT_END - WINDOW_HEIGHT_START);

            //Calculate which cell the position corresponds to
            let x_cell = (perc_x * width as f32).floor() as usize;
            let y_cell = (perc_y * height as f32).floor() as usize;
            data_height = data[x_cell][y_cell] as f32;
        }
    }

    Ok(())
}

//Calculate the colour gradient based on +/- distance from median
fn median_cell_col(val: f64, min: f64, med_val: f64, max: f64) -> Color {
    if val <= med_val {
        Color::new(
            (255.0 * (1.0 - ((val - min) / (med_val - min)))) as u8,
            (255.0 * ((val - min) / (med_val - min))) as u8,
            0,
            255,
        )
    } else {
        Color::new(
            0,
            (255.0 * (1.0 - ((val - med_val) / (max - med_val)))) as u8,
            (255.0 * ((val - med_val) / (max - med_val))) as u8,
            255,
        )
    }
}

//Calculate the colour as a percentage of the distance from max value
fn intensity_cell_col(val: f64, min: f64, max: f64) -> Color {
    Color::new(
        255.0 as u8,
        255.0 as u8,
        255.0 as u8,
        (255.0 * (val - min) / (max - min)) as u8,
    )
}

//Calculate the color as a percentage of the distance from the min value
fn inv_intensity_cell_col(val: f64, min: f64, max: f64) -> Color {
    Color::new(
        255.0 as u8,
        255.0 as u8,
        255.0 as u8,
        (255.0 * (1.0 - (val - min) / (max - min))) as u8,
    )
}

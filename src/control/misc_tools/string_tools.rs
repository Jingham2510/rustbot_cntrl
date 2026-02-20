//Removes the first and last character from a string
pub fn rem_first_and_last(value: &str) -> &str {
    //Turn the string to characters - then manipulate
    let mut chars = value.chars();
    chars.next();
    chars.next_back();
    chars.as_str()
}

pub fn str_to_vector(inp: &str) -> Vec<f64> {
    //Create the vector delimited by ,
    let sep: Vec<_> = inp.split(",").collect();

    //Remap all the values to become f32s
    sep.iter().flat_map(|x| x.parse()).collect()
}

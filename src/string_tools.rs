
pub fn rem_first_and_last(value: &str) -> &str {
    let mut chars = value.chars();
    chars.next();
    chars.next_back();
    chars.as_str()
}


pub fn str_to_vector(inp: &str) -> Vec<f32>{    
    //Create the vector delimited by ,
    let  sep: Vec<_>  = inp.split(",").collect();
    
    //Remap all the values to become f32s
    sep.iter().map(|x| x.parse()).flatten().collect()   
    
}
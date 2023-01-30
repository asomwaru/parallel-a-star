use std::{
    fs::{self, File},
    io::Write,
};

use graph::{Vertex, WeightedEdge};

mod Maze;
mod graph;
mod path_finder;

fn main() {
    let maze: Maze::Maze<Vertex<i32>, WeightedEdge<i32, Vertex<i32>>> = Maze::Maze::new(20);

    write_to_file("maze.txt".to_string(), &maze);

    // maze.get_maze().iter().for_each(|vector| {
    //     vector.iter().for_each(|val| {
    //         print!("{}", val);
    //     });

    //     println!();
    // });


    // write_grid_to_file("test.txt".to_string(), &maze.render_maze());
}

fn write_to_file<T: std::fmt::Display>(file_name: String, object: &T) {
    File::create(&file_name).unwrap();

    let mut fi = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(&file_name)
        .unwrap();

    write!(fi, "{}", object).unwrap();
}

fn write_grid_to_file(file_name: String, grid: &Vec<Vec<String>>) {
    File::create(&file_name).unwrap();

    let mut fi = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(&file_name)
        .unwrap();

    grid.iter().for_each(|vector| {
        vector.iter().for_each(|val| {
            fi.write_all(val.as_bytes()).unwrap();
        });

        fi.write_all("\n".as_bytes()).unwrap();
    });
}

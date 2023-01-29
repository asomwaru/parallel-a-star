use std::{
    fs::{self, File},
    io::Write,
    thread,
    time::Duration,
};

use graph::{Vertex, WeightedEdge};

mod graph;
mod maze;

fn main() {
    let maze: maze::Maze<Vertex<i32>, WeightedEdge<i32, Vertex<i32>>> = maze::Maze::new(15);
    write_grid_to_file("test.txt".to_string(), &maze.render_maze());
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

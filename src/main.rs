use std::{
    fmt::Display,
    fs::{self, File},
    io::Write,
    thread,
    time::Duration,
};

use graph::{Vertex, WeightedEdge};
use path_finder::PathFinder;
use Maze::Point2D;

mod Maze;
mod graph;
mod path_finder;

fn main() {
    let maze: Maze::Maze<Vertex<i32>, WeightedEdge<i32, Vertex<i32>>> = Maze::Maze::new(20);

    let path_finder = PathFinder::new(maze.get_maze());
    let maze_with_path = path_finder.show_path(path_finder.find_path(
        Point2D::new(1, 1),
        Point2D::new(37, 29),
        true,
        path_finder::SearchAlgorithms::BFS,
    ));

    write_to_file_grid("maze_with_path.txt".to_string(), maze_with_path);
}

fn write_to_file<T: Display>(file_name: String, object: &T) {
    File::create(&file_name).unwrap();

    let mut fi = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(&file_name)
        .unwrap();

    write!(fi, "{}", object).unwrap();
}

fn write_to_file_grid<T: Display>(file_name: String, grid: Vec<Vec<T>>) {
    File::create(&file_name).unwrap();

    let mut fi = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(&file_name)
        .unwrap();

    grid.iter().for_each(|vector| {
        vector.iter().for_each(|val| {
            fi.write_all(format!("{}", val).as_bytes()).unwrap();
        });

        fi.write_all("\n".as_bytes()).unwrap();
    });
}

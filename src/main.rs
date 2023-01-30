use std::{
    fs::{self, File},
    io::Write,
    thread,
    time::Duration,
};

use colored::Colorize;
use graph::{Vertex, WeightedEdge};
use path_finder::PathFinder;
use Maze::{Point2D, MazeCell};

mod Maze;
mod graph;
mod path_finder;

fn main() {
    loop {
        let maze: Maze::Maze<Vertex<i32>, WeightedEdge<i32, Vertex<i32>>> = Maze::Maze::new(100);

        let path_finder = PathFinder::new(maze.get_maze());
        let maze_with_path = path_finder.show_path(path_finder.find_path(
            Point2D::new(1, 1),
            Point2D::new(199, 199),
            true,
            path_finder::SearchAlgorithms::BFS,
        ));


        write!(std::io::stdout(), "{}", to_string(&maze_with_path, true)).unwrap();
        // write!(get_file("maze_with_path.txt".to_string()), "{}", to_string(&maze_with_path, false)).unwrap();

        thread::sleep(Duration::from_millis(20));
    }
}

fn get_file(file_name: String) -> File {
    File::create(&file_name).unwrap();

    let mut fi = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(&file_name)
        .unwrap();

    return fi;
}

fn to_string<T>(grid: &Vec<Vec<MazeCell<T>>>, colored: bool) -> String {
    let mut output = String::with_capacity(grid.len() * grid[0].len());

    grid.iter().for_each(|vector| {
        vector.iter().for_each(|val| {
            let mut str = format!("{}", val);
            
            if colored {
                str = match val.cell_type {
                    Maze::CellType::Vertex => str,
                    Maze::CellType::LeftRightBorder => str.red().to_string(),
                    Maze::CellType::UpDownBorder => str.red().to_string(),
                    Maze::CellType::Path => str.blue().to_string(),
                    Maze::CellType::Start => str.blue().to_string(),
                    Maze::CellType::End => str.blue().to_string(),
                    Maze::CellType::None => str,
                };
            }

            output.push_str(str.as_str());
        });

        output.push_str("\n");
    });

    output.push_str("\n");

    return output;
}

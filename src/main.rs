use std::{
    fmt::Display,
    fs::{self, File},
    io::Write,
    thread,
    time::{Duration, Instant},
};

use colored::Colorize;
use graph::{Vertex, WeightedEdge};
use path_finder::PathFinder;
use Maze::{MazeCell, Point2D};

mod Maze;
mod graph;
mod path_finder;

fn main() {
    let maze_size: (usize, usize) = (50, 50);

    let start_pos = Point2D::new(1, 1);
    let end_pos = Point2D::new(maze_size.0 * 2 - 1, maze_size.1 * 2 - 1);

    let maze: Maze::Maze<Vertex<i32>, WeightedEdge<i32, Vertex<i32>>> =
        Maze::Maze::new(maze_size.0, maze_size.1);
    let path_finder = PathFinder::new(maze.get_maze());

    let mut start = Instant::now();
    // let mut _maze_with_path = path_finder.show_path(path_finder.find_path(
    //     start_pos,
    //     end_pos,
    //     true,
    //     path_finder::SearchAlgorithms::AStarParallelNaive,
    // ));
    // println!("Parallel Naive A* Execution Time: {:?}", start.elapsed());

    start = Instant::now();
    let mut _maze_with_path = path_finder.show_path(path_finder.find_path(
        start_pos,
        end_pos,
        true,
        path_finder::SearchAlgorithms::AStar,
    ));

    println!("A* Execution Time: {:?}", start.elapsed());

    start = Instant::now();
    _maze_with_path = path_finder.show_path(path_finder.find_path(
        start_pos,
        end_pos,
        true,
        path_finder::SearchAlgorithms::BidirectionalBFS,
    ));
    println!("Bidirectional BFS Execution Time: {:?}", start.elapsed());

    start = Instant::now();
    _maze_with_path = path_finder.show_path(path_finder.find_path(
        start_pos,
        end_pos,
        true,
        path_finder::SearchAlgorithms::DFS,
    ));
    println!("DFS Execution Time: {:?}", start.elapsed());

    start = Instant::now();
    _maze_with_path = path_finder.show_path(path_finder.find_path(
        start_pos,
        end_pos,
        true,
        path_finder::SearchAlgorithms::BFS,
    ));
    println!("BFS Execution Time: {:?}", start.elapsed());


    write!(std::io::stdout(), "{}", to_string(&_maze_with_path, true)).unwrap();
    write!(
        get_file("maze_with_path.txt".to_string()),
        "{}",
        to_string(&_maze_with_path, false)
    )
    .unwrap();
}

/**
 * Creates a file with specified file_name for "appending" operation.
 *
 * Returns File pointer.
 */
fn get_file(file_name: String) -> File {
    File::create(&file_name).unwrap();

    let fi = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(&file_name)
        .unwrap();

    return fi;
}

/**
 * Converts Maze grid to the String.
 * Colored parameter is only applicable if string is going to be printed in console.
 *
 * Returns String.
 */
fn to_string<T: Display>(grid: &Vec<Vec<MazeCell<T>>>, colored: bool) -> String {
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

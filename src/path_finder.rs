use std::collections::{HashSet, VecDeque};

use crate::{
    Maze::{ MazeCell, Point2D},
};

struct PathFinder<T> {
    maze_grid: Vec<Vec<MazeCell<T>>>
}

impl<T> Default for PathFinder<T>{
    fn default() -> Self {
        Self { maze_grid: Default::default() }
    }
}

impl<T> PathFinder<T> {
    pub fn new(maze_grid: Vec<Vec<MazeCell<T>>>) -> PathFinder<T> {
        return PathFinder { maze_grid: maze_grid };
    }

    // pub fn find_path(&self, src: Point2D<T>, dest: Point2D<T>) -> Vec<Vec<MazeCell<T>>> {
    //     let queue: VecDeque<MazeCell<T>> = VecDeque::new();
    //     let maze_with_path: Vec<Vec<MazeCell<T>>> = self.to_owned().maze_grid;


        

    //     return maze_with_path;
    // }
}

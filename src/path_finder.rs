use std::collections::{HashSet, VecDeque};

use crate::Maze::{CellType, MazeCell, Point2D};

pub struct PathFinder<T: Clone> {
    maze_grid: Vec<Vec<MazeCell<T>>>,
}

impl<T> Default for PathFinder<T>
where
    T: Clone,
{
    fn default() -> Self {
        Self {
            maze_grid: Default::default(),
        }
    }
}

pub enum SearchAlgorithms {
    BFS,
    DFS,
}

impl<T> PathFinder<T>
where
    T: Clone,
{
    pub fn new(maze_grid: Vec<Vec<MazeCell<T>>>) -> PathFinder<T> {
        return PathFinder {
            maze_grid: maze_grid,
        };
    }

    pub fn show_path(&self, path: Vec<Point2D<usize>>) -> Vec<Vec<MazeCell<T>>> {
        let mut maze_with_path: Vec<Vec<MazeCell<T>>> = self.maze_grid.clone();

        let src = path.first().unwrap();
        let dest = path.last().unwrap();

        path.iter().for_each(|pos| {
            let x = pos.x;
            let y = pos.y;

            if pos == src {
                maze_with_path[src.x][src.y].cell_type = CellType::Start;
            } else if pos == dest {
                maze_with_path[dest.x][dest.y].cell_type = CellType::End;
            } else {
                maze_with_path[x][y].cell_type = CellType::Path;
            }
        });

        return maze_with_path;
    }

    /**
     * Finds path on the grid between src and dest using specified algorithm.
     * If src or dest doesn't point to the eligible CellType, returns empty path.
     *
     * Returns Vec<Point2D<usize>> containing Point2D obejects represented for each (x,y) pair on the graph between src and dest.
     */
    pub fn find_path(
        &self,
        src: Point2D<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
        algorithm: SearchAlgorithms,
    ) -> Vec<Point2D<usize>> {
        if self.maze_grid[src.x][src.y].cell_type != CellType::Vertex
            || self.maze_grid[dest.x][dest.y].cell_type != CellType::Vertex
        {
            return Vec::new();
        }

        return match algorithm {
            SearchAlgorithms::BFS => self.find_path_bfs(src, dest, consider_obstacles),
            SearchAlgorithms::DFS => self.find_path_dfs(src, dest, consider_obstacles),
        };
    }

    /**
     * Finds path on the grid between src and dest using DFS.
     *
     * Returns Vec<Point2D<usize>> containing Point2D obejects represented for each (x,y) pair on the graph between src and dest.
     */
    fn find_path_dfs(
        &self,
        src: Point2D<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
    ) -> Vec<Point2D<usize>> {
        let grid = self.maze_grid.clone();

        let width = grid[0].len();
        let height = grid.len();

        let mut paths: Vec<usize> = vec![0; width * height];
        paths.iter_mut().enumerate().for_each(|(i, val)| *val = i);

        let mut stack: VecDeque<usize> = VecDeque::new();
        let mut visited: HashSet<usize> = HashSet::new();

        stack.push_back(src.x * width + src.y);

        while !stack.is_empty() {
            let hashed_pos = stack.pop_back().unwrap();

            let x = hashed_pos / width;
            let y = hashed_pos % width;

            if visited.contains(&hashed_pos) {
                continue;
            }

            visited.insert(hashed_pos);

            let directions: [i32; 8] = [-1, 0, 1, 0, 0, -1, 0, 1];

            for i in (0..directions.len() - 1).step_by(2) {
                let next_x = (directions[i] + x as i32) as usize;
                let next_y = (directions[i + 1] + y as i32) as usize;

                let hashed_next_pos = next_x * width + next_y;

                if next_x >= height || next_y >= width || visited.contains(&hashed_next_pos) {
                    continue;
                }

                let next_cell_type = grid[next_x][next_y].cell_type.clone();

                let is_next_cell_border = next_cell_type == CellType::LeftRightBorder
                    || next_cell_type == CellType::UpDownBorder;

                if consider_obstacles && is_next_cell_border {
                    continue;
                }

                stack.push_back(hashed_next_pos);
                paths[hashed_next_pos] = hashed_pos;

                if next_x == dest.x && next_y == dest.y {
                    stack.clear();
                    break;
                }
            }
        }

        let reconstructed_path =
            Self::reconstruct_path(&paths, src.x * width + src.y, dest.x * width + dest.y);
        let mut path: Vec<Point2D<usize>> = Vec::new();

        reconstructed_path.iter().for_each(|pos| {
            let x = pos / width;
            let y = pos % width;

            path.push(Point2D::new(x, y));
        });

        return path;
    }

    /**
     * Finds path on the grid between src and dest using BFS.
     *
     * Returns Vec<Point2D<usize>> containing Point2D obejects represented for each (x,y) pair on the graph between src and dest.
     */
    fn find_path_bfs(
        &self,
        src: Point2D<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
    ) -> Vec<Point2D<usize>> {
        let grid = self.maze_grid.clone();

        let width = grid[0].len();
        let height = grid.len();

        let mut paths: Vec<usize> = vec![0; width * height];
        paths.iter_mut().enumerate().for_each(|(i, val)| *val = i);

        let mut queue: VecDeque<usize> = VecDeque::new();
        let mut visited: HashSet<usize> = HashSet::new();

        queue.push_back(src.x * width + src.y);

        while !queue.is_empty() {
            let hashed_pos = queue.pop_front().unwrap();

            let x = hashed_pos / width;
            let y = hashed_pos % width;

            if visited.contains(&hashed_pos) {
                continue;
            }

            visited.insert(hashed_pos);

            let directions: [i32; 8] = [-1, 0, 1, 0, 0, -1, 0, 1];

            for i in (0..directions.len() - 1).step_by(2) {
                let next_x = (directions[i] + x as i32) as usize;
                let next_y = (directions[i + 1] + y as i32) as usize;

                let hashed_next_pos = next_x * width + next_y;

                if next_x >= height || next_y >= width || visited.contains(&hashed_next_pos) {
                    continue;
                }

                let next_cell_type = grid[next_x][next_y].cell_type.clone();

                let is_next_cell_border = next_cell_type == CellType::LeftRightBorder
                    || next_cell_type == CellType::UpDownBorder;

                if consider_obstacles && is_next_cell_border {
                    continue;
                }

                queue.push_back(hashed_next_pos);
                paths[hashed_next_pos] = hashed_pos;

                if next_x == dest.x && next_y == dest.y {
                    queue.clear();
                    break;
                }
            }
        }

        let reconstructed_path =
            Self::reconstruct_path(&paths, src.x * width + src.y, dest.x * width + dest.y);
        let mut path: Vec<Point2D<usize>> = Vec::new();

        reconstructed_path.iter().for_each(|pos| {
            let x = pos / width;
            let y = pos % width;

            path.push(Point2D::new(x, y));
        });

        return path;
    }

    /**
     * Reconstructs path from src to dest using paths vector.
     *
     * Returns Vec<usize> containing cells between src and dest.
     */
    fn reconstruct_path(paths: &Vec<usize>, src: usize, dest: usize) -> Vec<usize> {
        let mut path: Vec<usize> = Vec::new();

        let mut p = dest;

        while p != src {
            path.push(p);
            p = paths[p];
        }

        path.push(p);
        path.reverse();

        return path;
    }
}

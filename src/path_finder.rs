use std::{
    cmp::{Ordering, Reverse},
    collections::{HashSet, VecDeque},
    f64::consts::SQRT_2,
    fmt::Display,
    sync::{Arc, Mutex},
    thread,
    vec,
};

use crate::{
    Maze::{CellType, MazeCell, Point2D},
};

use binary_heap_plus::{BinaryHeap, FnComparator};

use keyed_priority_queue::KeyedPriorityQueue;

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
    BidirectionalBFS,
    DFS,
    Dijkstra,
    AStar,
    AStarParallelNaive,
}

impl<T> PathFinder<T>
where
    T: Clone + Display + Send + 'static,
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
            SearchAlgorithms::BidirectionalBFS => {
                self.find_path_bidirectional_bfs(src, dest, consider_obstacles)
            }
            SearchAlgorithms::DFS => self.find_path_dfs(src, dest, consider_obstacles),
            SearchAlgorithms::Dijkstra => self.find_path_dijkstra(src, dest, consider_obstacles),
            SearchAlgorithms::AStar => self.find_path_a_star(src, dest, consider_obstacles),
            SearchAlgorithms::AStarParallelNaive => {
                self.find_path_a_star_parallel_naive(src, dest, consider_obstacles, 4)
            }
        };
    }

    fn manhattan_distance(src: Point2D<usize>, dest: Point2D<usize>) -> i32 {
        return i32::abs((src.x - dest.x) as i32) + i32::abs((src.y - dest.y) as i32);
    }

    fn diagonal_distance(src: Point2D<usize>, dest: Point2D<usize>) -> i32 {
        let dx = i32::abs((src.x - dest.x) as i32);
        let dy = i32::abs((src.y - dest.y) as i32);

        return (dx + dy) + f64::floor((SQRT_2 - 2.0) * dx.min(dy) as f64) as i32;
    }

    fn euclidean_distance(src: Point2D<usize>, dest: Point2D<usize>) -> i32 {
        let dx = i32::abs((src.x - dest.x) as i32);
        let dy = i32::abs((src.y - dest.y) as i32);

        return f64::sqrt((dx.pow(2) + dy.pow(2)) as f64) as i32;
    }

    fn a_star_parallel_thread_naive(
        maze_grid: Vec<Vec<MazeCell<T>>>,
        heap: Arc<
            Mutex<
                BinaryHeap<
                    (usize, i32),
                    FnComparator<fn(&(usize, i32), &(usize, i32)) -> Ordering>,
                >,
            >,
        >,
        src: Point2D<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
    ) -> Vec<usize> {
        let grid = maze_grid.clone();

        let width = grid[0].len();
        let height = grid.len();

        let mut paths: Vec<usize> = vec![0; width * height];
        paths.iter_mut().enumerate().for_each(|(i, val)| *val = i);

        // The sum of g-score and h-score
        let mut f_score = vec![i32::MAX; width * height];

        // Absolute distance from starting node to current node
        let mut g_score = vec![i32::MAX; width * height];

        f_score[src.x * width + src.y] =
            Self::manhattan_distance(Point2D::new(src.x, src.y), Point2D::new(dest.x, dest.y));

        g_score[src.x * width + src.y] = 0;

        {
            heap.lock().unwrap().push((src.x * width + src.y, 0));
        }

        while !heap.lock().unwrap().is_empty() {
            let hashed_tuple;

            {
                hashed_tuple = heap.lock().unwrap().pop().unwrap();
            }

            let hashed_pos = hashed_tuple.0;

            let x = hashed_pos / width;
            let y = hashed_pos % width;

            let directions: [i32; 8] = [-1, 0, 1, 0, 0, -1, 0, 1];

            for i in (0..directions.len() - 1).step_by(2) {
                let next_x = (directions[i] + x as i32) as usize;
                let next_y = (directions[i + 1] + y as i32) as usize;

                let hashed_next_pos = next_x * width + next_y;

                if next_x >= height || next_y >= width {
                    continue;
                }

                let next_cell_type = grid[next_x][next_y].cell_type.clone();

                let is_next_cell_border = next_cell_type == CellType::LeftRightBorder
                    || next_cell_type == CellType::UpDownBorder;

                if consider_obstacles && is_next_cell_border {
                    continue;
                }

                let tentative_g_score = g_score[hashed_pos] + 1;

                if tentative_g_score < g_score[hashed_next_pos] {
                    let h_score = Self::manhattan_distance(
                        Point2D::new(next_x, next_y),
                        Point2D::new(dest.x, dest.y),
                    );

                    g_score[hashed_next_pos] = tentative_g_score;

                    f_score[hashed_next_pos] = tentative_g_score + h_score;

                    paths[hashed_next_pos] = hashed_pos;

                    {
                        heap.lock()
                            .unwrap()
                            .push((hashed_next_pos, f_score[hashed_next_pos]));
                    }
                }

                if next_x == dest.x && next_y == dest.y {
                    heap.lock().unwrap().clear();
                    return paths;
                }
            }
        }
        return vec![];
    }

    /**
     * This A* variant is working fine as long as you have an expensive heuristic function... If your heuristic calculation requires a small amount of time,
     * there is a chance that you will have a high contention and, therefore, one thread might always stay idle.
     */
    fn find_path_a_star_parallel_naive(
        &self,
        src: Point2D<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
        max_threds: u32,
    ) -> Vec<Point2D<usize>> {
        let heap: Arc<
            Mutex<
                BinaryHeap<
                    (usize, i32),
                    FnComparator<fn(&(usize, i32), &(usize, i32)) -> Ordering>,
                >,
            >,
        > = Arc::new(Mutex::new(BinaryHeap::new_by(
            |a: &(usize, i32), b: &(usize, i32)| b.1.cmp(&a.1),
        )));

        let mut threads = vec![];

        for _ in 0..max_threds {
            let heap_clone = heap.clone();
            let maze_clone = self.maze_grid.clone();

            threads.push(thread::spawn(move || {
                Self::a_star_parallel_thread_naive(maze_clone, heap_clone, src, dest, consider_obstacles)
            }))
        }

        let mut path = vec![];

        for thread in threads {
            path = thread.join().unwrap();
            break;
        }

        let reconstructed_path = Self::reconstruct_path(
            &path,
            src.x * self.maze_grid.first().unwrap().len() + src.y,
            dest.x * self.maze_grid.first().unwrap().len() + dest.y,
        );

        return Self::convert_hashed_path(
            reconstructed_path,
            self.maze_grid.first().unwrap().len(),
        );
    }

    /**
     * Finds path on the grid between src and dest using  A*.
     *
     * Returns Vec<Point2D<usize>> containing Point2D obejects represented for each (x,y) pair on the graph between src and dest.
     */
    fn find_path_a_star(
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

        let mut visited: HashSet<usize> = HashSet::new();

        // The sum of g-score and h-score
        let mut f_score = vec![i32::MAX; width * height];

        // Absolute distance from starting node to current node
        let mut g_score = vec![i32::MAX; width * height];

        f_score[src.x * width + src.y] =
            Self::manhattan_distance(Point2D::new(src.x, src.y), Point2D::new(dest.x, dest.y));

        g_score[src.x * width + src.y] = 0;

        let mut heap = BinaryHeap::new_by(|a: &(usize, i32), b: &(usize, i32)| b.1.cmp(&a.1));

        heap.push((src.x * width + src.y, 0));

        while !heap.is_empty() {
            let hashed_tuple = heap.pop().unwrap();
            let hashed_pos = hashed_tuple.0;

            visited.insert(hashed_pos);

            let x = hashed_pos / width;
            let y = hashed_pos % width;

            let directions: [i32; 8] = [-1, 0, 1, 0, 0, -1, 0, 1];

            for i in (0..directions.len() - 1).step_by(2) {
                let next_x = (directions[i] + x as i32) as usize;
                let next_y = (directions[i + 1] + y as i32) as usize;

                let hashed_next_pos = next_x * width + next_y;

                if next_x >= height || next_y >= width {
                    continue;
                }

                let next_cell_type = grid[next_x][next_y].cell_type.clone();

                let is_next_cell_border = next_cell_type == CellType::LeftRightBorder
                    || next_cell_type == CellType::UpDownBorder;

                if consider_obstacles && is_next_cell_border {
                    continue;
                }

                let tentative_g_score = g_score[hashed_pos] + 1;

                if tentative_g_score < g_score[hashed_next_pos] {
                    let h_score = Self::manhattan_distance(
                        Point2D::new(next_x, next_y),
                        Point2D::new(dest.x, dest.y),
                    );

                    g_score[hashed_next_pos] = tentative_g_score;

                    f_score[hashed_next_pos] = tentative_g_score + h_score;

                    paths[hashed_next_pos] = hashed_pos;

                    heap.push((hashed_next_pos, f_score[hashed_next_pos]));
                }

                if next_x == dest.x && next_y == dest.y {
                    heap.clear();
                    break;
                }
            }
        }

        let reconstructed_path =
            Self::reconstruct_path(&paths, src.x * width + src.y, dest.x * width + dest.y);

        return Self::convert_hashed_path(reconstructed_path, width);
    }

    /**
     * Finds path on the grid between src and dest using  Bidirectional BFS.
     *
     * Returns Vec<Point2D<usize>> containing Point2D obejects represented for each (x,y) pair on the graph between src and dest.
     */
    fn find_path_bidirectional_bfs(
        &self,
        src: Point2D<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
    ) -> Vec<Point2D<usize>> {
        let grid = self.maze_grid.clone();

        let width = grid[0].len();
        let height = grid.len();

        let mut src_paths: Vec<usize> = vec![0; width * height];
        src_paths
            .iter_mut()
            .enumerate()
            .for_each(|(i, val)| *val = i);

        let mut dest_paths: Vec<usize> = src_paths.clone();

        let mut src_queue: VecDeque<usize> = VecDeque::new();
        let mut src_visited: HashSet<usize> = HashSet::new();

        let mut dest_queue: VecDeque<usize> = VecDeque::new();
        let mut dest_visited: HashSet<usize> = HashSet::new();

        src_queue.push_back(src.x * width + src.y);
        dest_queue.push_back(dest.x * width + dest.y);

        let mut intersection_pos_hashed: i32 = -1;

        while !src_queue.is_empty() && !dest_queue.is_empty() {
            let src_vector = Self::perform_bfs_queue_operation(
                &mut src_queue,
                &grid,
                &mut src_visited,
                &mut src_paths,
                dest,
                consider_obstacles,
            );

            Self::perform_bfs_queue_operation(
                &mut dest_queue,
                &grid,
                &mut dest_visited,
                &mut dest_paths,
                src,
                consider_obstacles,
            );

            if !src_queue.is_empty() && !dest_queue.is_empty() {
                for val in src_vector.iter() {
                    if dest_visited.contains(&val) {
                        intersection_pos_hashed = *val as i32;

                        src_queue.clear();
                        dest_queue.clear();
                        break;
                    }
                }
            }
        }

        let path_from_dest_to_intersection = Self::reconstruct_path(
            &dest_paths,
            dest.x * width + dest.y,
            intersection_pos_hashed as usize,
        );
        let path_from_src_to_intersection = Self::reconstruct_path(
            &src_paths,
            src.x * width + src.y,
            intersection_pos_hashed as usize,
        );

        let mut reconstructed_path = Vec::with_capacity(
            path_from_dest_to_intersection.len() + path_from_src_to_intersection.len(),
        );

        reconstructed_path.extend(path_from_src_to_intersection.iter());
        reconstructed_path.extend(path_from_dest_to_intersection.iter());

        return Self::convert_hashed_path(reconstructed_path, width);
    }

    /**
     * Performs a single BFS iteration on the given queue.
     *
     * Returns the Vec containing node's neighbours that were added to the queue.
     */
    fn perform_bfs_queue_operation(
        queue: &mut VecDeque<usize>,
        grid: &Vec<Vec<MazeCell<T>>>,
        visited: &mut HashSet<usize>,
        paths: &mut Vec<usize>,
        dest: Point2D<usize>,
        consider_obstacles: bool,
    ) -> Vec<usize> {
        let mut neighbours: Vec<usize> = Vec::new();

        let width = grid[0].len();
        let height = grid.len();

        let hashed_pos = queue.pop_front().unwrap();

        let x = hashed_pos / width;
        let y = hashed_pos % width;

        if visited.contains(&hashed_pos) {
            return neighbours;
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

            neighbours.push(hashed_next_pos);

            queue.push_back(hashed_next_pos);
            paths[hashed_next_pos] = hashed_pos;

            if next_x == dest.x && next_y == dest.y {
                queue.clear();
                break;
            }
        }

        return neighbours;
    }

    /**
     * Finds path on the grid between src and dest using Dijkstra.
     *
     * Returns Vec<Point2D<usize>> containing Point2D obejects represented for each (x,y) pair on the graph between src and dest.
     */
    fn find_path_dijkstra(
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

        let mut distances: Vec<i32> = vec![i32::MAX; width * height];
        distances[src.x * width + src.y] = 0;

        let mut priority_queue = KeyedPriorityQueue::<usize, Reverse<i32>>::new();

        distances.iter().enumerate().for_each(|(pos, distance)| {
            priority_queue.push(pos, Reverse(*distance));
        });

        while !priority_queue.is_empty() {
            let tuple = priority_queue.pop().unwrap();

            let hashed_pos = tuple.0;
            let distance = tuple.1 .0;

            let x = hashed_pos / width;
            let y = hashed_pos % width;

            let directions: [i32; 8] = [-1, 0, 1, 0, 0, -1, 0, 1];

            for i in (0..directions.len() - 1).step_by(2) {
                let next_x = (directions[i] + x as i32) as usize;
                let next_y = (directions[i + 1] + y as i32) as usize;

                let hashed_next_pos = next_x * width + next_y;

                if next_x >= height || next_y >= width {
                    continue;
                }

                let next_cell_type = grid[next_x][next_y].cell_type.clone();

                let is_next_cell_border = next_cell_type == CellType::LeftRightBorder
                    || next_cell_type == CellType::UpDownBorder;

                if consider_obstacles && is_next_cell_border {
                    continue;
                }

                let next_cell_distance = distance + 1;

                if next_cell_distance < distances[hashed_next_pos] {
                    distances[hashed_next_pos] = next_cell_distance;
                    paths[hashed_next_pos] = hashed_pos;
                    priority_queue
                        .set_priority(&hashed_next_pos, Reverse(distances[hashed_next_pos]))
                        .unwrap();
                }

                if x == dest.x && y == dest.y {
                    priority_queue.clear();
                    break;
                }
            }
        }

        let reconstructed_path =
            Self::reconstruct_path(&paths, src.x * width + src.y, dest.x * width + dest.y);

        return Self::convert_hashed_path(reconstructed_path, width);
    }

    fn convert_hashed_path(hashed_path: Vec<usize>, width: usize) -> Vec<Point2D<usize>> {
        let mut path = Vec::new();

        hashed_path.iter().for_each(|pos| {
            let x = pos / width;
            let y = pos % width;

            path.push(Point2D::new(x, y));
        });

        return path;
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

        return Self::convert_hashed_path(reconstructed_path, width);
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
            Self::perform_bfs_queue_operation(
                &mut queue,
                &grid,
                &mut visited,
                &mut paths,
                dest,
                consider_obstacles,
            );
        }

        let reconstructed_path =
            Self::reconstruct_path(&paths, src.x * width + src.y, dest.x * width + dest.y);

        return Self::convert_hashed_path(reconstructed_path, width);
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

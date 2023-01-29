use std::{
    collections::{HashMap, HashSet, VecDeque},
    hash::Hash,
};

use rand::Rng;

use crate::graph::{Graph, IVertex, IWeightedEdge};

pub struct Maze<V, E> {
    grid: Vec<Vec<MazeCell<i32>>>,
    grid_graph: Graph<V, E>,
    vertex_to_grid_map: HashMap<V, Point2D<usize>>,
    mst: HashSet<E>,
}

#[derive(Clone, Debug)]
pub struct MazeCell<T> {
    pub val: T,
    pub cell_type: CellType,
}

impl<T> MazeCell<T> {
    pub fn new(val: T, cell_type: CellType) -> Self {
        return MazeCell {
            val: val,
            cell_type: cell_type,
        };
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum CellType {
    Vertex,
    LeftRightBorder,
    UpDownBorder,
    None,
}

#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug)]
pub struct Point2D<T> {
    x: T,
    y: T,
}

impl<T> Point2D<T> {
    pub fn new(x: T, y: T) -> Self {
        return Point2D { x: x, y: y };
    }
}

impl<V, E> Maze<V, E>
where
    V: IVertex<i32>,
    E: IWeightedEdge<i32, V>,
{
    pub fn new(size: usize) -> Self {
        let grid = Self::init_grid(size);
        let mut grid_graph: Graph<V, E> = Self::init_graph(&grid, Point2D::new(0, 0));
        let mst = grid_graph.find_mst(crate::graph::MSTAlgorithms::Kruskal);

        let mut extended_grid = Self::extend_grid(&grid);

        let mut vertex_to_grid_map: HashMap<V, Point2D<usize>> = HashMap::new();

        extended_grid.iter().enumerate().for_each(|(i, vector)| {
            vector.iter().enumerate().for_each(|(j, val)| {
                if val.cell_type == CellType::Vertex {
                    vertex_to_grid_map.insert(V::new(val.val), Point2D::new(i, j));
                }
            });
        });

        mst.iter().for_each(|edge| {
            let src = edge.get_src();
            let dest = edge.get_dest();

            let grid_src = vertex_to_grid_map.get(&src).unwrap();
            let grid_dest = vertex_to_grid_map.get(&dest).unwrap();

            let path = Self::find_path_on_grid(&extended_grid, *grid_src, *grid_dest);

            path.iter().for_each(|grid_pos| {
                let cell_type = extended_grid[grid_pos.x][grid_pos.y].cell_type.clone();

                if cell_type == CellType::LeftRightBorder || cell_type == CellType::UpDownBorder {
                    extended_grid[grid_pos.x][grid_pos.y].cell_type = CellType::None;
                };
            });
        });

        return Maze {
            grid: extended_grid,
            grid_graph: grid_graph,
            vertex_to_grid_map: vertex_to_grid_map,
            mst: mst,
        };
    }

    fn find_path_on_grid(
        grid: &Vec<Vec<MazeCell<i32>>>,
        src: Point2D<usize>,
        dest: Point2D<usize>,
    ) -> Vec<Point2D<usize>> {
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
            let y = pos % height;

            path.push(Point2D::new(x, y));
        });

        return path;
    }

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

    fn extend_grid(grid: &Vec<Vec<i32>>) -> Vec<Vec<MazeCell<i32>>> {
        let size = grid.len();

        let mut extended_grid =
            vec![vec![MazeCell::new(-1, CellType::None); size * 2 + 1]; size * 2 + 1];

        extended_grid
            .iter_mut()
            .enumerate()
            .for_each(|(i, vector)| {
                vector.iter_mut().for_each(|cell| {
                    if i % 2 == 0 {
                        cell.cell_type = CellType::UpDownBorder;
                    } else {
                        cell.cell_type = CellType::LeftRightBorder;
                    }
                });
            });

        for i in (1..extended_grid.len()).step_by(2) {
            for j in (1..extended_grid[i].len()).step_by(2) {
                extended_grid[i][j] = MazeCell::new(grid[i / 2][j / 2], CellType::Vertex);
            }
        }

        return extended_grid;
    }

    pub fn render_maze(&self) -> Vec<Vec<String>> {
        let mut rendered_grid: Vec<Vec<String>> =
            vec![vec![String::new(); self.grid.len()]; self.grid.len()];

        for i in (0..rendered_grid.len()).step_by(1) {
            for j in (0..rendered_grid[i].len()).step_by(1) {
                let cell_type = self.grid[i][j].cell_type.clone();

                match cell_type {
                    CellType::UpDownBorder => rendered_grid[i][j] = format!("―+―"),
                    CellType::LeftRightBorder => {
                        if j == 0 {
                            rendered_grid[i][j] = format!(" |");
                        } else if j % 2 == 0 {
                            rendered_grid[i][j] = format!("|");
                        }
                    }
                    CellType::Vertex => rendered_grid[i][j] = format!("     "),
                    CellType::None => {
                        if i % 2 == 0 {
                            rendered_grid[i][j] = format!("   ");
                        } else {
                            rendered_grid[i][j] = format!(" ");
                        }
                    }
                }
            }
        }

        return rendered_grid;
    }

    fn init_graph(grid: &Vec<Vec<i32>>, start_pos: Point2D<usize>) -> Graph<V, E> {
        let height = grid.len();
        let width = grid[0].len();

        let mut adj_map: HashMap<V, HashSet<E>> = HashMap::new();
        let mut visited: HashSet<usize> = HashSet::new();
        let mut queue: VecDeque<usize> = VecDeque::new();

        let hashed_start_pos = start_pos.x * width + start_pos.y;

        queue.push_back(hashed_start_pos);

        while !queue.is_empty() {
            let hashed_pos = queue.pop_front().unwrap();

            if visited.contains(&hashed_pos) {
                continue;
            }

            visited.insert(hashed_pos);

            let x = hashed_pos / width;
            let y = hashed_pos % width;

            let grid_pos = grid[x][y];

            if !adj_map.contains_key(&V::new(grid_pos)) {
                adj_map.insert(V::new(grid_pos), HashSet::new());
            }

            let directions = [-1, 0, 1, 0, 0, -1, 0, 1];

            for i in (0..directions.len() - 1).step_by(2) {
                let next_x = (directions[i] + x as i32) as usize;
                let next_y = (directions[i + 1] + y as i32) as usize;

                let hashed_next_pos = next_x * width + next_y;

                if next_x >= height || next_y >= width || visited.contains(&hashed_next_pos) {
                    continue;
                }

                queue.push_back(hashed_next_pos);

                let grid_next_pos = grid[next_x][next_y];

                if !adj_map.contains_key(&V::new(grid_next_pos)) {
                    adj_map.insert(V::new(grid_next_pos), HashSet::new());
                }

                let weight = rand::thread_rng().gen_range(1..(width * height)) as i32;

                adj_map
                    .get_mut(&V::new(grid_next_pos))
                    .unwrap()
                    .insert(<E as IWeightedEdge<i32, V>>::new(
                        V::new(grid_next_pos),
                        V::new(grid_pos),
                        weight,
                    ));

                adj_map.get_mut(&V::new(grid_pos)).unwrap().insert(
                    <E as IWeightedEdge<i32, V>>::new(
                        V::new(grid_pos),
                        V::new(grid_next_pos),
                        weight,
                    ),
                );
            }
        }

        return Graph::new(adj_map);
    }

    fn init_grid(size: usize) -> Vec<Vec<i32>> {
        let mut grid = vec![vec![0; size]; size];

        let mut counter = 0;
        grid.iter_mut().for_each(|vector| {
            vector.iter_mut().for_each(|val| {
                *val = counter;
                counter += 1;
            });
        });

        return grid;
    }
}

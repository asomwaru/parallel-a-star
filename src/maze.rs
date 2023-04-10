use std::{
    collections::{HashMap, HashSet, VecDeque},
    fmt::{Debug, Display},
    hash::Hash
};

use rand::Rng;

use crate::path_finder::PathFinder;
use crate::{
    graph::{Graph, IVertex, IWeightedEdge},
    path_finder::SearchAlgorithms,
};

pub struct Maze<V, E> {
    grid: Vec<Vec<MazeCell>>,
    original_grid_graph: Graph<V, E>,
}

#[derive(Clone, Debug)]
pub struct MazeCell {
    pub val: usize,
    pub cell_type: CellType,
}

impl MazeCell {
    pub fn new(val: usize, cell_type: CellType) -> Self {
        return MazeCell {
            val: val,
            cell_type: cell_type,
        };
    }
}

impl Display for MazeCell {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let output = match self.cell_type {
            CellType::UpDownBorder => format!("{}", "+"),
            CellType::LeftRightBorder => format!("{}", "|"),
            CellType::Vertex => format!(" ",),
            CellType::None => format!(" "),
            CellType::Path => format!("{}", "*"),
            CellType::Start => format!("{}", "o"),
            CellType::End => format!("{}", "x"),
        };


        // CellType::UpDownBorder => format!("{}", "―+―"),
        // CellType::LeftRightBorder => format!(" {} ", "|"),
        // CellType::Vertex => format!("   ",),
        // CellType::None => format!("   "),
        // CellType::Path => format!(" {} ", "*"),
        // CellType::Start => format!(" {} ", "o"),
        // CellType::End => format!(" {} ", "x"),
        return write!(f, "{}", output);
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum CellType {
    Vertex,
    LeftRightBorder,
    UpDownBorder,
    Path,
    Start,
    End,
    None,
}

#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug)]
pub struct Point2D<T> {
    pub x: T,
    pub y: T,
}

impl<T> Point2D<T> {
    pub fn new(x: T, y: T) -> Self {
        return Point2D { x: x, y: y };
    }
}

impl<T> Display for Point2D<T>
where
    T: Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return write!(f, "({},{})", self.x, self.y);
    }
}

impl<V, E> Display for Maze<V, E>
where
    V: IVertex<usize>,
    E: IWeightedEdge<usize, V>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let rendered_maze = self.render_maze();

        let mut output = String::new();

        rendered_maze.iter().for_each(|vector| {
            vector.iter().for_each(|val| {
                output.push_str(val);
            });

            output.push_str("\n");
        });

        return write!(f, "{}", output);
    }
}

impl<V, E> Default for Maze<V, E>
where
    V: IVertex<usize>,
    E: IWeightedEdge<usize, V>,
{
    fn default() -> Self {
        Self {
            grid: Vec::new(),
            original_grid_graph: Graph::new(HashMap::new())
        }
    }
}

impl<V, E> Maze<V, E>
where
    V: IVertex<usize>,
    E: IWeightedEdge<usize, V>,
{
    /**
     * Generates maze from the given size.
     *
     * Returns pointer to Maze<V, E>.
     */
    pub fn new(height: usize, width: usize) -> Self {
        let grid = Self::init_grid(height, width);

        let mut original_grid_graph: Graph<V, E> = Self::init_graph(&grid, Point2D::new(0, 0));
        let mst = original_grid_graph.find_mst(crate::graph::MSTAlgorithms::Kruskal);

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

            let path = PathFinder::new(extended_grid.clone()).find_path(
                *grid_src,
                *grid_dest,
                false,
                SearchAlgorithms::AStar,
            );

            path.iter().for_each(|grid_pos| {
                let cell_type = extended_grid[grid_pos.x][grid_pos.y].cell_type.clone();

                if cell_type == CellType::LeftRightBorder || cell_type == CellType::UpDownBorder {
                    extended_grid[grid_pos.x][grid_pos.y].cell_type = CellType::None;
                };
            });
        });

        for i in 0..extended_grid.len() {
            for j in 0..extended_grid[i].len() {
                extended_grid[i][j].val = i * extended_grid[i].len() + j;
            }
        }

        return Maze {
            grid: extended_grid,
            original_grid_graph,
        };
    }

    pub fn get_maze(&self) -> Vec<Vec<MazeCell>> {
        return self.grid.clone();
    }

    /**
     * Extends grid to make each cell being wrapped around the "barrier" of cells with width == 1.
     * Each cell will have MazeCell object containing information about the cell type (e.g. Vertex, Border, None, etc.).
     *
     * Returns Vec<Vec<MazeCell<i32>>>.
     */
    fn extend_grid(grid: &Vec<Vec<usize>>) -> Vec<Vec<MazeCell>> {
        let height = grid.len();
        let width = grid[0].len();

        let mut extended_grid =
            vec![vec![MazeCell::new(0, CellType::None); width * 2 + 1]; height * 2 + 1];

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

    /**
     * Renders maze into the Vec<Vec<String>> format.
     * Rendered version of the maze could be used for clear representation of the maze in file/console.
     *
     * Returns Vec<Vec<String>>.
     */
    pub fn render_maze(&self) -> Vec<Vec<String>> {
        let mut rendered_grid: Vec<Vec<String>> =
            vec![vec![String::new(); self.grid.len()]; self.grid.len()];

        self.grid.iter().enumerate().for_each(|(i, vector)| {
            vector.iter().enumerate().for_each(|(j, cell)| {
                rendered_grid[i][j] = format!("{}", cell);
            });
        });

        return rendered_grid;
    }

    /**
     * Creates a graph from the grid beginning at start_pos.
     * Graph will have |V| = grid.len() * grid[0].len() vertices and all vertices will be connected by the Edge.
     * Each Edge will have a randomly assigned weight from 1 to grid.len() * grid[0].len().
     *
     * Returns Graph<V, E>.
     */
    fn init_graph(grid: &Vec<Vec<usize>>, start_pos: Point2D<usize>) -> Graph<V, E> {
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

                if next_x >= height || next_y >= width {
                    continue;
                }

                let hashed_next_pos = next_x * width + next_y;

                if visited.contains(&hashed_next_pos) {
                    continue;
                }

                queue.push_back(hashed_next_pos);

                let grid_next_pos = grid[next_x][next_y];

                if !adj_map.contains_key(&V::new(grid_next_pos)) {
                    adj_map.insert(V::new(grid_next_pos), HashSet::new());
                }

                let weight = rand::thread_rng().gen_range(1..(width * height));

                adj_map
                    .get_mut(&V::new(grid_next_pos))
                    .unwrap()
                    .insert(<E as IWeightedEdge<usize, V>>::new(
                        V::new(grid_next_pos),
                        V::new(grid_pos),
                        weight,
                    ));

                adj_map.get_mut(&V::new(grid_pos)).unwrap().insert(
                    <E as IWeightedEdge<usize, V>>::new(
                        V::new(grid_pos),
                        V::new(grid_next_pos),
                        weight,
                    ),
                );
            }
        }

        return Graph::new(adj_map);
    }

    /**
     * Creates a grid of specified size.
     * Each cell of the grid will be numbered in the increasing order from 0 to size - 1.
     *
     * Returns a Vec<Vec<i32>> representation of the grid.
     */
    fn init_grid(height: usize, width: usize) -> Vec<Vec<usize>> {
        let mut grid = vec![vec![0; width]; height];

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

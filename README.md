# Parallel A*Algorithm

In this project we are working on implementation of the Parallel A* algorithm. 

For testing purposes, we developed a unique maze generation algorithm that utilizes the idea of MST to build a, well, unique maze of any size. 

Then, we implemented several Best Path searching algorithms. 

Currently we have:
- A*
- BFS
- Bi-directional BFS
- DFS
- Dijkstra
- A* Parallel (Naive approach)

Our plan is to utilize those algorithm to show the comparison in the performance of path searching algorithms compared to real Parallel A* algorithm implementation.

Our first idea was to implement a simple, naive, parallel A* algorithm by having multiple threads sharing the same memory address to the heap data structure used as an "open set" in the A*. Quite expectedly, this solution didn't prove to be useful because there are several drawbacks:

1. Our heuristic function is very basic (we simply calculate the distance between two points), not computational heavy. As such, we don't spend much time when we "explore" potential neighbors of the current point.
2. Due to #1, we have a high contention for a lock assigned to heap... In other words, threads spend most of their time by waiting for a lock (which they will never get because heuristic calculation takes too little time), instead of actually doing parallel exploration.

There are several solution to that problem, and we are currently reading several papers that explain different approaches that could be done to solve it.

## Installation
Simply use `cargo run --release` command to run the code in the "release" mod. Release mod provides optimizations and therefore, code runs faster.

## Maze Example
This is an example of generated maze and a path for source to destination.
![](https://i.imgur.com/a46Z6VO.png)

## License

[MIT](https://choosealicense.com/licenses/mit/)
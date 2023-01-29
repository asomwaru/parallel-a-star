use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};

use binary_heap_plus::BinaryHeap;

use std::fmt::Debug;
use std::hash::Hash;

#[derive(Hash, Ord, PartialOrd, Eq, PartialEq, Copy, Clone, Debug)]
pub struct Vertex<T> {
    val: T,
}

#[derive(Hash, PartialOrd, Eq, PartialEq, Clone, Copy, Debug)]
pub struct WeightedEdge<T, V> {
    src: V,
    dest: V,
    weight: T,
}

impl<T, V> Ord for WeightedEdge<T, V>
where
    T: Hash + Ord + Copy,
    V: IVertex<i32>,
{
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if self.weight > other.weight {
            return Ordering::Greater;
        } else if self.weight < other.weight {
            return Ordering::Less;
        } else {
            return Ordering::Equal;
        }
    }
}

impl<T, V> IWeightedEdge<T, V> for WeightedEdge<T, V>
where
    T: Hash + Ord + Copy + Debug,
    V: IVertex<i32>,
{
    fn new(src: V, dest: V, weight: T) -> Self
    where
        Self: IWeightedEdge<T, V>,
    {
        return WeightedEdge {
            src: src,
            dest: dest,
            weight: weight,
        };
    }

    fn get_weight(&self) -> T {
        return self.weight;
    }

    fn get_weight_mut(&mut self) -> T {
        return self.weight;
    }

    fn get_dest(&self) -> V {
        return self.dest;
    }

    fn get_dest_mut(&mut self) -> V {
        return self.dest;
    }

    fn get_src(&self) -> V {
        return self.src;
    }

    fn get_src_mut(&mut self) -> V {
        return self.src;
    }
}

impl<T> IVertex<T> for Vertex<T>
where
    T: Ord + Hash + Copy + Debug,
{
    fn new(val: T) -> Self
    where
        Self: IVertex<T>,
    {
        return Vertex { val: val };
    }

    fn get_val(&self) -> T {
        return self.val;
    }

    fn get_val_mut(&mut self) -> T {
        return self.val;
    }
}

pub trait IVertex<T>: Ord + Hash + Copy + Debug {
    fn new(val: T) -> Self
    where
        Self: IVertex<T>;

    fn get_val(&self) -> T;
    fn get_val_mut(&mut self) -> T;
}

pub trait IWeightedEdge<T, V>: Ord + Hash + Copy + Debug {
    fn new(src: V, dest: V, weight: T) -> Self
    where
        Self: IWeightedEdge<T, V>;

    fn get_dest(&self) -> V;
    fn get_dest_mut(&mut self) -> V;

    fn get_src(&self) -> V;
    fn get_src_mut(&mut self) -> V;

    fn get_weight(&self) -> T;
    fn get_weight_mut(&mut self) -> T;
}

#[derive(Debug)]
pub struct Graph<V, E> {
    adj_map: HashMap<V, HashSet<E>>,
}

impl<V, E> Graph<V, E>
where
    V: IVertex<i32>,
    E: IWeightedEdge<i32, V>,
{
    /**
     * Initializes a Graph object.
     * 
     * Returns pointer to Graph<V, E>.
     */
    pub fn new(adj_map: HashMap<V, HashSet<E>>) -> Self {
        return Graph { adj_map: adj_map };
    }

    /**
     * Finds Minimmum Spanning Tree in the graph.
     * 
     * Returns HashSet<E> containing all E of the the MST.
     */
    pub fn find_mst(&mut self, algorithm: MSTAlgorithms) -> HashSet<E> {
        return match algorithm {
            MSTAlgorithms::Kruskal => Self::kruskal(self),
            MSTAlgorithms::Prim => Self::prim(),
        };
    }

    /**
     * Kruskal's algorithm for finding MST in the graph.
     * 
     * Returns HashSet<E> containing all E of the the MST.
     */
    fn kruskal(&mut self) -> HashSet<E> {
        let mut set: HashSet<E> = HashSet::new();

        let mut heap = BinaryHeap::new_by(|a: &E, b: &E| b.to_owned().cmp(a));

        let mut subset: HashMap<V, V> = HashMap::new();

        self.adj_map.iter().for_each(|(v0, set)| {
            subset.insert(*v0, *v0);

            set.iter().for_each(|e| {
                if *v0 != e.get_dest() {
                    heap.push(*e);
                }
            });
        });

        while !heap.is_empty() {
            let edge = heap.pop().unwrap();

            if Self::union(&mut subset, edge.get_src(), edge.get_dest()) {
                set.insert(edge);
            }
        }

        return set;
    }

    //#[TODO: 1]: Implement Prim's algorithm for MST. 
    /**
     * Prim's algorithm for finding MST in the graph.
     * 
     * Returns HashSet<E> containing all E of the the MST.
     */
    fn prim() -> HashSet<E> {
        return HashSet::new();
    }

    /**
     * Find operation of the Disjoint Set algorithm.
     * 
     * Returns parent V of the v.
     */
    fn find(subset: &mut HashMap<V, V>, v: V) -> V {
        if *subset.get(&v).unwrap() != v {
            let v = Self::find(subset, *subset.get(&v).unwrap());
            subset.insert(v, v);
            return *subset.get(&v).unwrap();
        }

        return v;
    }

    /**
     * Union operation of the Disjoint Set algorithm.
     * 
     * Returns true if union could be performed and false otherwise.
     */
    fn union(subset: &mut HashMap<V, V>, v0: V, v1: V) -> bool {
        let v0_root = Self::find(subset, v0);
        let v1_root = Self::find(subset, v1);

        if v0_root == v1_root {
            return false;
        }

        subset.insert(v0_root, v1_root);
        return true;
    }
}

pub enum MSTAlgorithms {
    Kruskal,
    Prim,
}

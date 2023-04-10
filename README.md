# Lecture 20: Bellman-Ford Algorithm for Shortest Paths

>Note: The most of the information of these lectures was extracted and adapted from Dr Bajuelos and from Weiss’s book, “Data Structures and Algorithm Analysis in Java". They are provided for COP3530 students only. Not to be published or publicly distributed without permission by the publisher. 

## Definition

The Bellman–Ford algorithm is an algorithm that computes shortest paths from a single source vertex to all of the other vertices in a weighted digraph.

It is slower than Dijkstra's algorithm for the same problem, but more versatile, as it is capable of handling graphs in which some of the edge weights are negative numbers.

### Single-Source Shortest-Path (SP) Problem.

* Given a weighted graph, G = (V,E), and a distinguished vertex, s, find the shortest weighted path from s to every other vertex in G.

The Problem (adaptation for the Dikstra’s Algorithm): 

* Given a digraph with non-negative edge weights, G=(V,E), and a distinguished source vertex s in V. 
* Determine the distance and a shortest path (SP) from the source vertex to every vertex in the digraph.

Bellman-Ford algorithm: more general than Dijkstra’s algorithm for shortest path problem: 
* Edge-weights can be negative
* Detects the existence of negative-weight cycle(s) reachable from s.

### Relaxation

Lemma 1: Let p<sub>1k</sub> = ‹v<sub>1</sub>, v<sub>2</sub>, …, v<sub>k</sub>› be a SP from v<sub>1</sub> to v<sub>k</sub>.  Then p<sub>ij</sub> = ‹v<sub>1i</sub>, v<sub>i+1</sub>, …, v<sub>j</sub>› is a SP from v<sub>i</sub> to v<sub>j</sub>, where 1 <= i <= j <= k. 

So, for shortest path problem we have the optimal-substructure property.


![Graph](images/BF1.png)


Let δ(u,v) = weight of SP from u to v.
* Corollary: Let p = SP from s to v. 
	* Then, δ(s,v) = δ(s,u)  + w(u,v)

Lemma 2: (Triangle inequality) Let s ∈ V. 
* For all edges (u,v) ∈ E, we have:
	δ(s,v) <= δ(s,u) + w(u,v)

Algorithms keep track of d[v], π[v].  
Initialized as follows (pseudocode):
```text
Initialize(G,s)

for each v ∈ V
{
	d[v] = ∞;
	π[v] := NIL;
}
d[s] = 0;

```
These values are changed when an edge (u,v) is relaxed:
```text
Relax(u,v,w)
if d[v] > d[u] + w(u,v) then
{
    d[v] = d[u] + w(u,v);
	π[v] = u;
}

```
### Properties of Relaxation


![Graph](images/BF2.png)


* d[v], if not ∞, is the length of some path from s to v.
* d[v] either stays the same or decreases with time.
* Therefore, if d[v] = δ(s, v) at any time, this holds thereafter.
* d[v] >= δ(s, v) always.


## BellMan-Ford Algorithm

The input graph can have negative-weight edges.  

```text
Initialize(G, s);
for i = 1 to |V| –1 
{
	for each (u,v) in E 
	{
		Relax(u,v,w)
    }
};

//Detects the existence of negative-weight cycle(s) reachable from s.

for each (u,v) in E 
{ 
	if d[v] > d[u] + w(u,v)
    {
        error("Graph contains cycles of negative length");
		return false
	}
}
return true
```

Complexity: - O(\|V\|\|E\|)


![Graph](images/BF3.png)

![Graph](images/BF4.png)

![Graph](images/BF5.png)

![Graph](images/BF6.png)

![Graph](images/BF7.png)

Example:
http://www.programming-algorithms.net/article/47389/Bellman-Ford-algorithm
Visualization:
     https://visualgo.net/en/sssp

Java Implementation:
http://algs4.cs.princeton.edu/44sp/BellmanFordSP.java.html

## Example:

https://www.techiedelight.com/single-source-shortest-paths-bellman-ford-algorithm/

### Implementation

```java
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
 
// A class to store a graph edge
class Edge
{
    int source, dest, weight;
 
    public Edge(int source, int dest, int weight)
    {
        this.source = source;
        this.dest = dest;
        this.weight = weight;
    }
}
 
class Main
{
    // Recursive function to print the path of a given vertex from source vertex
    static void getPath(int parent[], int vertex, List<Integer> path)
    {
        if (vertex < 0) {
            return;
        }
 
        getPath(parent, parent[vertex], path);
        path.add(vertex);
    }
 
    // Function to run the Bellman–Ford algorithm from a given source
    public static void bellmanFord(List<Edge> edges, int source, int n)
    {
        // distance[] and parent[] stores the shortest path
        // (least cost/path) information
        int distance[] = new int[n];
        int parent[] = new int[n];
 
        // initialize `distance[]` and `parent[]`. Initially, all vertices
        // except source vertex weight INFINITY and no parent
        Arrays.fill(distance, Integer.MAX_VALUE);
        distance[source] = 0;
 
        Arrays.fill(parent, -1);
 
        // relaxation step (run V-1 times)
        for (int i = 0; i < n - 1; i++)
        {
            for (Edge edge: edges)
            {
                // edge from `u` to `v` having weight `w`
                int u = edge.source;
                int v = edge.dest;
                int w = edge.weight;
 
                // if the distance to destination `v` can be
                // shortened by taking edge (u, v)
                if (distance[u] != Integer.MAX_VALUE && distance[u] + w < distance[v])
                {
                    // update distance to the new lower value
                    distance[v] = distance[u] + w;
 
                    // set v's parent as `u`
                    parent[v] = u;
                }
            }
        }
 
        // run relaxation step once more for n'th time to
        // check for negative-weight cycles
        for (Edge edge: edges)
        {
            // edge from `u` to `v` having weight `w`
            int u = edge.source;
            int v = edge.dest;
            int w = edge.weight;
 
            // if the distance to destination `u` can be
            // shortened by taking edge (u, v)
            if (distance[u] != Integer.MAX_VALUE && distance[u] + w < distance[v])
            {
                System.out.println("Negative-weight cycle is found!!");
                return;
            }
        }
 
        for (int i = 0; i < n; i++)
        {
            if (i != source && distance[i] < Integer.MAX_VALUE) {
                List<Integer> path = new ArrayList<>();
                getPath(parent, i, path);
                System.out.println("The distance of vertex " + i + " from vertex " +
                        source + " is " + distance[i] + ". Its path is " + path);
            }
        }
    }
 
    public static void main(String[] args)
    {
        // List of graph edges as per the above diagram
        List<Edge> edges = Arrays.asList(
                // (x, y, w) —> edge from `x` to `y` having weight `w`
                new Edge(0, 1, -1), new Edge(0, 2, 4), new Edge(1, 2, 3),
                new Edge(1, 3, 2), new Edge(1, 4, 2), new Edge(3, 2, 5),
                new Edge(3, 1, 1), new Edge(4, 3, -3 )
        );
 
        // set the maximum number of nodes in the graph
        int n = 5;
 
        // run the Bellman–Ford algorithm from every node
        for (int source = 0; source < n; source++) {
            bellmanFord(edges, source, n);
        }
    }
}
```

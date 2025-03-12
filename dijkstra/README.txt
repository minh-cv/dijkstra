How to build:
- Put this folder into <your-workspace>/src
- Open the terminal. At the root of your workspace, run this command:
	colcon build --packages-select dijkstra

How to run:
- Server: ros2 run dijkstra server
- Client: ros2 run dijkstra client <file-path>
    where <file-path> is the path to a text file containing your input

Input: the first line contains the source node.
Each line after that contains 3 arguments representing an edge in a directed graph
- The first argument (string) is a node
- The second argument (string) is a node that forms an edge with the first (the edge points from the first to the second)
- The third argument (uint32) is the distance between them
Ex: see test.txt. The graph is basically like this:
A → B: 7
A → C: 9
A → F: 14
B → C: 10
B → D: 15
C → D: 11
C → F: 2
D → E: 6
E → F: 9
And the source node is A.
Output: each node in the graph, along with the shortest distance from the source node to it & a path to it.
Ex: the output for the above example is:
 	Node name: E, Distance: 26, Path: A->C->D->E
	Node name: F, Distance: 11, Path: A->C->F
	Node name: B, Distance: 7, Path: A->B
	Node name: D, Distance: 20, Path: A->C->D
	Node name: C, Distance: 9, Path: A->C
	Node name: A, Distance: 0, Path: A


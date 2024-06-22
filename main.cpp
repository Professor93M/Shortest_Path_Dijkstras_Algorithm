#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <chrono>

using namespace std;
using namespace chrono;

// Structure to represent a node in the graph
struct Node {
    int vertex;
    int weight;

    Node(int v, int w) : vertex(v), weight(w) {}
};

// Comparison function for the priority queue
struct CompareNodes {
    bool operator()(const Node &n1, const Node &n2) {
        return n1.weight > n2.weight;
    }
};

class Graph {
private:
    int vertices;
    vector<vector<int>> adjacencyMatrix;

public:
    Graph(int V) : vertices(V), adjacencyMatrix(V, vector<int>(V, INT_MAX)) {}

    // Add an edge to the graph
    void addEdge(int src, int dest, int weight) {
        adjacencyMatrix[src][dest] = weight;
        adjacencyMatrix[dest][src] = weight;
    }

    // Dijkstra's algorithm
    void dijkstra(int source, int destination) {
        // Priority queue to store vertices with their distances
        priority_queue<Node, vector<Node>, CompareNodes> pq;

        // Vector to store distances from the source
        vector<int> distance(vertices, INT_MAX);

        // Set the distance of the source vertex to 0
        distance[source] = 0;

        // Enqueue the source vertex with its distance
        pq.push(Node(source, 0));

        while (!pq.empty()) {
            // Extract the vertex with the minimum distance
            int u = pq.top().vertex;
            pq.pop();

            // Stop if the destination is reached
            if (u == destination)
                break;

            // Iterate through all adjacent vertices of u
            for (int v = 0; v < vertices; ++v) {
                int weightUV = adjacencyMatrix[u][v];

                // Relaxation step
                if (weightUV != INT_MAX && distance[u] + weightUV < distance[v]) {
                    distance[v] = distance[u] + weightUV;
                    pq.push(Node(v, distance[v]));
                }
            }
        }

        // Print the shortest distance to the destination using Dijkstra
        cout << "Dijkstra's: Shortest distance from the source (" << source << ") to destination (" << destination << "): ";
        if (distance[destination] != INT_MAX) {
            cout << distance[destination] << "\n";
        } else {
            cout << "Not reachable\n";
            return;
        }

        // Print all paths from source to destination
        printAllPaths(source, destination, distance);
    }

    // Print all paths from source to destination
    void printAllPaths(int source, int destination, vector<int>& distance) {
        vector<int> path;
        vector<bool> visited(vertices, false);
        cout << "Path: ";
        printAllPathsUtil(source, destination, distance, visited, path);
    }

    void printAllPathsUtil(int u, int destination, vector<int>& distance, vector<bool>& visited, vector<int>& path) {
        visited[u] = true;
        path.push_back(u);

        if (u == destination) {
            for (int i = 0; i < path.size(); ++i) {
                cout << path[i];
                if (i != path.size() - 1) cout << " -> ";
            }
            cout << "\n";
        } else {
            for (int v = 0; v < vertices; ++v) {
                if (adjacencyMatrix[u][v] != INT_MAX && !visited[v] && distance[u] + adjacencyMatrix[u][v] == distance[v]) {
                    printAllPathsUtil(v, destination, distance, visited, path);
                }
            }
        }
        // Backtrack
        visited[u] = false;
        path.pop_back();
    }

    // Floyd-Warshall algorithm
    void floydWarshall(int source, int destination) {
        vector<vector<int>> distance = adjacencyMatrix;
        vector<vector<int>> next(vertices, vector<int>(vertices, -1));

        for (int k = 0; k < vertices; ++k) {
            for (int i = 0; i < vertices; ++i) {
                for (int j = 0; j < vertices; ++j) {
                    if (distance[i][k] != INT_MAX && distance[k][j] != INT_MAX &&
                        distance[i][k] + distance[k][j] < distance[i][j]) {
                        distance[i][j] = distance[i][k] + distance[k][j];
                        next[i][j] = k; // Store the intermediate vertex
                    }
                }
            }
        }

        // Print the shortest distance using Floyd-Warshall
        cout << "Floyd-Warshall: Shortest distance from the source (" << source << ") to destination (" << destination << "): ";
        if (distance[source][destination] != INT_MAX) {
            cout << distance[source][destination] << "\n";
        } else {
            cout << "Not reachable\n";
            return;
        }

        // Print the path from source to destination using Floyd-Warshall
        cout << "Path: ";
        int intermediate = destination;

        if (next[source][intermediate] == -1) {
            cout << "No path found.";
        } else {
            vector<int> path;
            while (intermediate != -1 && intermediate != source) {
                path.push_back(intermediate);
                intermediate = next[source][intermediate];
            }
            cout << source;
            for (int i = path.size() - 1; i >= 0; --i) {
                cout << " -> " << path[i];
            }
        }
        cout << "\n";
    }
};

int main() {
    Graph g(8);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 5);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 3, 4);
    g.addEdge(1, 4, 6);
    g.addEdge(2, 5, 8);
    g.addEdge(3, 6, 3);
    g.addEdge(4, 6, 7);
    g.addEdge(5, 7, 4);
    g.addEdge(6, 7, 1);

    // Take user input for source and destination
    int source, destination;
    cout << "Enter source vertex: ";
    cin >> source;
    cout << "Enter destination vertex: ";
    cin >> destination;

    // Measure execution time for Dijkstra's
    auto dijkstra_start = high_resolution_clock::now();
    g.dijkstra(source, destination);
    auto dijkstra_stop = high_resolution_clock::now();
    auto dijkstra_duration = duration_cast<microseconds>(dijkstra_stop - dijkstra_start);

    // Measure execution time for Floyd-Warshall
    auto floyd_start = high_resolution_clock::now();
    g.floydWarshall(source, destination);
    auto floyd_stop = high_resolution_clock::now();
    auto floyd_duration = duration_cast<microseconds>(floyd_stop - floyd_start);

    // Print execution times
    cout << "Dijkstra's execution time: " << dijkstra_duration.count() << " microseconds\n";
    cout << "Floyd-Warshall execution time: " << floyd_duration.count() << " microseconds\n";

    return 0;
}

#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
using namespace std;

struct Edge {
    int u, v, weight;
    Edge(int u_, int v_, int w_) : u(u_), v(v_), weight(w_) {}
};

class Graph {
private:
    int V;
    vector<vector<Edge>> adj;
    vector<bool> removed;
    
public:
    Graph(int vertices) : V(vertices) {
        adj.resize(V);
        removed.resize(V, false);
    }
    
    void addEdge(int u, int v, int w) {
        adj[u].push_back(Edge(u, v, w));
        adj[v].push_back(Edge(v, u, w));
    }
    
    vector<Edge> partitionEdges() {
        vector<Edge> L, R;
        bool addToL = true;
        
        while (true) {
            // Find a vertex with incident edges
            int start = -1; 
            for (int i = 0; i < V; i++) {
                if (!removed[i] && !adj[i].empty()) {
                    start = i;
                    break;
                }
            }
            if (start == -1) break;
            cout << start << endl;
            int v = start; 
                // Find heaviest edge incident to v
                Edge* heaviest = nullptr;
                int maxWeight = -1;
                int nextV = -1;
                
                for (const Edge& e : adj[v]) {
                    if (!removed[e.v] && e.weight > maxWeight) {
                        maxWeight = e.weight;
                        heaviest = const_cast<Edge*>(&e);
                        nextV = e.v;
                    }
                }
                if (heaviest == nullptr) break;
                
                // Add edge to appropriate set
                if (addToL) {
                    L.push_back(*heaviest);
                } else {
                    R.push_back(*heaviest);
                }
                addToL = !addToL;
                
                // Remove vertex v and its edges
                removed[v] = true;
                
                // Move to next vertex
                v = nextV; 
        }
        
        // Return set with greater total weight
        int sumL = 0, sumR = 0;
        for (const Edge& e : L) sumL += e.weight;
        for (const Edge& e : R) sumR += e.weight;
        
        return sumL >= sumR ? L : R;
    }
};

int main() {
    // Example usage
    Graph g(5);
    g.addEdge(0, 1, 1);
    g.addEdge(1, 2, 3);
    g.addEdge(2, 3, 5);
    g.addEdge(3, 4, 3);
    g.addEdge(4, 0, 1);
    
    vector<Edge> result = g.partitionEdges();
    
    cout << "Selected edges:\n";
    for (const Edge& e : result) {
        cout << e.u << " -- " << e.v << " (weight: " << e.weight << ")\n";
    }
    
    return 0;
}
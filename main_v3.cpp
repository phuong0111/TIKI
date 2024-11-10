#include <bits/stdc++.h>
typedef long int ll;

struct Edge {
    int u, v;
    ll weight;

    Edge(int u, int v, ll weight) : u(u), v(v), weight(weight) {}

    // Operator for sorting edges by weight
    bool operator<(const Edge &other) const {
        return weight > other.weight; // Use > for min-heap
    }
};

class MinWeightMatching {
  private:
    std::vector<std::vector<ll>> graph;
    int n; // number of nodes

  public:
    MinWeightMatching(const std::vector<std::vector<ll>> &adjacency_matrix)
        : graph(adjacency_matrix), n(adjacency_matrix.size()) {
        if (n % 2 != 0) {
            throw runtime_error("Graph must have even number of nodes for perfect matching");
        }
    }

    std::vector<std::pair<int, int>> findMinWeightMatching() {
        // Priority queue to store edges sorted by weight
        std::priority_queue<Edge> edges;

        // Add all edges to priority queue
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                edges.push(Edge(i, j, graph[i][j]));
            }
        }

        // Keep track of matched nodes
        std::set<int> matched;
        std::vector<std::pair<int, int>> matches;

        // Greedily match nodes using smallest available edges
        while (!edges.empty() && matched.size() < n) {
            Edge e = edges.top();
            edges.pop();

            // If both nodes are unmatched, match them
            if (matched.find(e.u) == matched.end() &&
                matched.find(e.v) == matched.end()) {
                matched.insert(e.u);
                matched.insert(e.v);
                matches.push_back({e.u, e.v});
            }
        }

        return matches;
    }

    void printMatchingStats(const std::vector<std::pair<int, int>> &matches) {
        ll total_weight = 0;
        ll max_weight = -1;

        std::cout << "Matches:\n";
        for (const auto &match : matches) {
            int u = match.first;
            int v = match.second;
            ll weight = graph[u][v];

            total_weight += weight;
            max_weight = max(max_weight, weight);

            std::cout << "(" << u << "," << v << ") weight: " << weight << "\n";
        }

        std::cout << "Total weight: " << total_weight << "\n";
        std::cout << "Maximum edge weight: " << max_weight << "\n";
    }
};

const int MAX_POINT = 1001;
const int MAX_VEHICLES = 501;
const int MAX_REQUESTS = 1001;

enum Action {
    PICKUP_CONTAINER,
    PICKUP_CONTAINER_TRAILER,
    DROP_CONTAINER,
    DROP_CONTAINER_TRAILER,
    PICKUP_TRAILER,
    DROP_TRAILER,
    STOP
};

const std::vector<std::string> actions = {
    "PICKUP_CONTAINER",
    "PICKUP_CONTAINER_TRAILER",
    "DROP_CONTAINER",
    "DROP_CONTAINER_TRAILER",
    "PICKUP_TRAILER",
    "DROP_TRAILER",
    "STOP"};

enum ContainerSize {
    NONE = 0,
    TWENTY_FT = 20,
    FORTY_FT = 40
};

struct Request {
    int id;
    ContainerSize size;
    int pickup_point;
    Action pickup_action;
    ll pickup_duration;
    int drop_point;
    Action drop_action;
    ll drop_duration;
};

struct Route {
    int depot;
    std::list<int> list_reqs;
    ll cost;

    // Default constructor
    Route() : depot(0), cost(0) {}

    // Constructor with depot
    Route(int d) : depot(d), cost(0) {}

    // Get size of the route
    size_t size() const {
        return list_reqs.size();
    }
};

struct RequestContext {
    int request_id;
    ll transitionCost;                 // Cost of transitions (prev->curr->next)
    ll selfCost;                       // Cost of request itself (pickup + drop duration)
    int routeIdx;                      // Which route this request is in
    std::list<int>::iterator position; // Position in route
};

std::array<std::array<ll, MAX_POINT>, MAX_POINT> distances;
std::array<int, MAX_VEHICLES> vehicleDepots;
std::array<Route, MAX_VEHICLES> currentSolution;
std::vector<Request> requestTwentyFt;
std::vector<Request> requestFortyFt;
std::array<RequestContext, MAX_REQUESTS> requestContexts;
std::array<bool, MAX_REQUESTS> isRequestRemoved;

struct IO {
    std::vector<int> requestIdx;
    int num_vehicles;
    int trailer_point;
    int trailer_pickup_time;
    int alpha;
    int max_iterations;
    bool verbose;

    IO(int alpha, int max_iterations, bool verbose = false)
        : alpha(alpha), max_iterations(max_iterations), verbose(verbose) {}

    IO() {}

    Action getAction(std::string action) {
        if (action == "PICKUP_CONTAINER")
            return PICKUP_CONTAINER;
        if (action == "DROP_CONTAINER")
            return DROP_CONTAINER;
        if (action == "PICKUP_CONTAINER_TRAILER")
            return PICKUP_CONTAINER_TRAILER;
        return DROP_CONTAINER_TRAILER;
    }

    ContainerSize getContaierSize(int size) {
        return size == 20 ? TWENTY_FT : FORTY_FT;
    }

    void input() {
        std::string dummy_str;
        int N, dummy_num;
        std::cin >> dummy_str >> N;
        std::cin >> dummy_str >> dummy_num;

        for (int i = 0; i < N * N; i++) {
            int src, dst;
            std::cin >> src >> dst >> distances[src][dst];
        }

        std::cin >> dummy_str >> trailer_point >> trailer_pickup_time;

        std::cin >> dummy_str >> num_vehicles;
        for (int i = 0; i < num_vehicles; i++) {
            int truck_id, truck_point;
            std::cin >> truck_id >> truck_point;
            vehicleDepots[truck_id - 1] = truck_point;
        }

        while (true) {
            std::cin >> dummy_str;
            if (dummy_str == "#")
                break;

            int id, size, pickup_point, drop_point, pickup_duration, drop_duration;
            std::string pickup_action, drop_action;
            std::cin >> id >> size >> pickup_point >> pickup_action >> pickup_duration >> drop_point >> drop_action >> drop_duration;

            Request request = {
                id,
                getContaierSize(size),
                pickup_point,
                getAction(pickup_action),
                pickup_duration,
                drop_point,
                getAction(drop_action),
                drop_duration};
            requestIdx.push_back(id);
            if (size == 20)
                requestTwentyFt.push_back(request);
            else
                requestFortyFt.push_back(request);
        }
    }

    ll getDistance(int from, int to) {
        return distances[from][to];
    }

    ll calculateRequestTransitionCost(const Request &curr_req, const Request &next_req) {
        if (curr_req.drop_action == DROP_CONTAINER && next_req.pickup_action == PICKUP_CONTAINER_TRAILER)
            return getDistance(curr_req.drop_point, trailer_point) + getDistance(trailer_point, next_req.pickup_point) + trailer_pickup_time;
        if (curr_req.drop_action == DROP_CONTAINER_TRAILER && next_req.pickup_action == PICKUP_CONTAINER)
            return getDistance(curr_req.drop_point, trailer_point) + getDistance(trailer_point, next_req.pickup_point) + trailer_pickup_time;
        return getDistance(curr_req.drop_point, next_req.pickup_point);
    }

    ll calculateTwentyFtCombinationCost(const Request &req1, const Request &req2){
        ll cost_1 = calculateRequestTransitionCost(req1, req2); // pickup 1 -> drop 1 -> pickup 2 -> drop 2
        ll cost_2 = calculateRequestTransitionCost(req2, req1); // pickup 2 -> drop 2 -> pickup 1 -> drop 1
        // pickup 1 -> pickup 2 -> drop 1 -> drop 2
        // pickup 1 -> pickup 2 -> drop 2 -> drop 1
        // pickup 2 -> pickup 1 -> drop 1 -> drop 2
        // pickup 2 -> pickup 1 -> drop 2 -> drop 1
    }

    void pairMatching() {
        int num_of_nodes = requestTwentyFt.size();
        std::vector<std::vector<ll>> graph(num_of_nodes, std::vector<ll>(num_of_nodes, 0));
        for (int i = 0; i < num_of_nodes; i++)
            for (int j = i + 1; j < num_of_nodes; j++) {

            }
    }
};

// Example usage
int main() {
    // Example graph with 4 nodes
    int num_of_nodes = requestTwentyFt.size();
    std::vector<std::vector<ll>> graph(num_of_nodes, std::vector<ll>(num_of_nodes, 0));

    for (int i = 0; i < num_of_nodes; i++)
        for (int j = i + 1; j < num_of_nodes; j++) {
        }

    try {
        MinWeightMatching matcher(graph);
        auto matches = matcher.findMinWeightMatching();
        matcher.printMatchingStats(matches);
    } catch (const exception &e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
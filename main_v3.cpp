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
    int n; // Original number of nodes

  public:
    MinWeightMatching(const std::vector<std::vector<ll>> &adjacency_matrix)
        : graph(adjacency_matrix), n(adjacency_matrix.size()) {}

    // Returns {matches, remaining_node_id}, where remaining_node_id is -1 if none
    std::pair<std::vector<std::pair<int, int>>, int> findMinWeightMatching() {
        std::vector<std::pair<int, int>> matches;

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
        bool is_odd = (n % 2 != 0);

        // Greedily match nodes using smallest available edges
        while (!edges.empty() && matched.size() < n - (is_odd ? 1 : 0)) {
            Edge e = edges.top();
            edges.pop();

            // If both nodes are unmatched, match them
            if (matched.find(e.u) == matched.end() &&
                matched.find(e.v) == matched.end()) {
                matches.push_back({e.u, e.v});
                matched.insert(e.u);
                matched.insert(e.v);
            }
        }

        // Find remaining node if odd number of nodes
        int remaining_node = -1;
        if (is_odd) {
            for (int i = 0; i < n; i++) {
                if (matched.find(i) == matched.end()) {
                    remaining_node = i;
                    break;
                }
            }
        }

        return {matches, remaining_node};
    }

    void printMatchingStats(const std::vector<std::pair<int, int>> &matches, int remaining_node = -1) {
        ll total_weight = 0;
        ll max_weight = -1;

        std::cout << "Matches:\n";
        for (const auto &match : matches) {
            int u = match.first;
            int v = match.second;
            ll weight = graph[u][v];

            total_weight += weight;
            max_weight = std::max(max_weight, weight);

            std::cout << "(" << u << "," << v << ") weight: " << weight << "\n";
        }

        if (remaining_node != -1) {
            std::cout << "Remaining unmatched node: " << remaining_node << "\n";
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
std::array<Request, MAX_REQUESTS> requests;
std::array<RequestContext, MAX_REQUESTS> requestContexts;
std::array<bool, MAX_REQUESTS> isRequestRemoved;
std::vector<std::vector<ll>> graphWeight;
std::vector<std::vector<int>> combinationType;

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

    ll calculateRequestContextCost(const Request &req) {
        return req.pickup_duration + req.drop_duration + getDistance(req.pickup_point, req.drop_point);
    }

    std::pair<ll, int> calculateTwentyFtCombinationCost(const Request &req1, const Request &req2) {
        // Pre-calculate common values to avoid duplication
        ll req1_self_cost = req1.pickup_duration + req1.drop_duration;
        ll req2_self_cost = req2.pickup_duration + req2.drop_duration;
        ll total_durations = req1_self_cost + req2_self_cost;

        // Pre-calculate common distances
        ll dist_p1_p2 = getDistance(req1.pickup_point, req2.pickup_point);
        ll dist_p1_d1 = getDistance(req1.pickup_point, req1.drop_point);
        ll dist_p2_p1 = getDistance(req2.pickup_point, req1.pickup_point);
        ll dist_p2_d2 = getDistance(req2.pickup_point, req2.drop_point);
        ll dist_d1_d2 = getDistance(req1.drop_point, req2.drop_point);
        ll dist_p2_d1 = getDistance(req2.pickup_point, req1.drop_point);
        ll dist_p1_d2 = getDistance(req1.pickup_point, req2.drop_point);
        ll dist_d2_d1 = getDistance(req2.drop_point, req1.drop_point);

        std::vector<std::pair<ll, int>> combinations;

        // Sequential combinations (always possible)
        // 1. p1->d1->p2->d2 (pattern: 0011)
        combinations.push_back({req1_self_cost + req2_self_cost +
                                    dist_p1_d1 +
                                    calculateRequestTransitionCost(req1, req2) +
                                    dist_p2_d2,
                                0b0011});

        // 2. p2->d2->p1->d1 (pattern: 1100)
        combinations.push_back({req1_self_cost + req2_self_cost +
                                    dist_p2_d2 +
                                    calculateRequestTransitionCost(req2, req1) +
                                    dist_p1_d1,
                                0b1100});

        // Nested combinations (only if conditions met)
        bool can_nest_1 = (req2.pickup_action == PICKUP_CONTAINER);
        bool can_nest_2 = (req1.pickup_action == PICKUP_CONTAINER);

        if (can_nest_1) {
            // 3. p1->p2->d1->d2 (pattern: 0101)
            if (req1.drop_action == DROP_CONTAINER) {
                combinations.push_back({total_durations +
                                            dist_p1_p2 +
                                            dist_p2_d1 +
                                            dist_d1_d2,
                                        0b0101});
            }
            // 4. p1->p2->d2->d1 (pattern: 0110)
            if (req2.drop_action == DROP_CONTAINER) {
                combinations.push_back({total_durations +
                                            dist_p1_p2 +
                                            dist_p2_d2 +
                                            dist_d2_d1,
                                        0b0110});
            }
        }

        if (can_nest_2) {
            // 5. p2->p1->d1->d2 (pattern: 1001)
            if (req1.drop_action == DROP_CONTAINER) {
                combinations.push_back({total_durations +
                                            dist_p2_p1 +
                                            dist_p1_d1 +
                                            dist_d1_d2,
                                        0b1001});
            }
            // 6. p2->p1->d2->d1 (pattern: 1010)
            if (req2.drop_action == DROP_CONTAINER) {
                combinations.push_back({total_durations +
                                            dist_p2_p1 +
                                            dist_p1_d2 +
                                            dist_d2_d1,
                                        0b1010});
            }
        }

        // Return the combination with maximum cost
        return *min_element(combinations.begin(), combinations.end());
    }

    std::pair<std::vector<std::pair<int, int>>, int> pairMatching() {
        int num_of_nodes = requestTwentyFt.size();
        graphWeight.clear();
        graphWeight = std::vector<std::vector<ll>>(num_of_nodes, std::vector<ll>(num_of_nodes, 0));
        combinationType.clear();
        combinationType = std::vector<std::vector<int>>(num_of_nodes, std::vector<int>(num_of_nodes, 0));

        // Build the graph
        for (int i = 0; i < num_of_nodes; i++) {
            for (int j = i + 1; j < num_of_nodes; j++) {
                auto cost_combination = calculateTwentyFtCombinationCost(requestTwentyFt[i], requestTwentyFt[j]);
                graphWeight[i][j] = cost_combination.first;
                graphWeight[j][i] = cost_combination.first;
                combinationType[i][j] = cost_combination.second;
                combinationType[j][i] = cost_combination.second;
            }
        }

        MinWeightMatching matcher(graphWeight);
        auto [matches, remaining_node] = matcher.findMinWeightMatching();
        matcher.printMatchingStats(matches, remaining_node);
        // // Handle the remaining node if needed
        // if (remaining_node != -1) {
        //     // Process requestTwentyFt[remaining_node] separately
        // }
        return {matches, remaining_node};
    }

    void combineRequest(std::pair<std::vector<std::pair<int, int>>, int> matched_pair) {
        int i = 0;
        Request req1, req2;
        int pickup_point, drop_point;
        Action pickup_action, drop_action;
        for (auto x : matched_pair.first) {
            req1 = requestTwentyFt[x.first];
            req2 = requestTwentyFt[x.second];
        }
    }
};

// Example usage
int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    freopen("tc/6/inp.txt", "r", stdin);

    IO io(100000, 1000000, 0);
    io.input();
    io.pairMatching();

    return 0;
}
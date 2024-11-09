#include <bits/stdc++.h>
using namespace std;

typedef long int ll;

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

std::array<std::array<ll, MAX_POINT>, MAX_POINT> distances;
std::array<int, MAX_VEHICLES> vehicleDepots;
std::array<Route, MAX_VEHICLES> currentSolution;
std::array<Request, MAX_REQUESTS> requests;

class PDPSolver {
  private:
    std::vector<int> requestIdx;
    int num_vehicles;
    std::random_device rd;
    std::mt19937 gen;
    int alpha;
    int trailer_point;
    int trailer_pickup_time;
    int max_iterations;
    int max_attempt = 20;
    bool verbose;

    const double temperature = 100.0;
    const double coolingRate = 0.9995;

  public:
    void addRequest(Route &route, const int request_id) {
        route.list_reqs.push_back(request_id);
        route.cost = calculateRouteCost(route);
    }

    void insertRequest(Route &route, const int request_id, int position) {
        auto it = route.list_reqs.begin();
        std::advance(it, position);
        route.list_reqs.insert(it, request_id);
        route.cost = calculateRouteCost(route);
    }

    void removeRequest(Route &route, int position) {
        auto it = route.list_reqs.begin();
        std::advance(it, position);
        route.list_reqs.erase(it);
        route.cost = calculateRouteCost(route);
    }

    void removeStopsByRequestId(Route &route, int request_id) {
        route.list_reqs.remove_if([request_id](const int requestId) {
            return request_id == requestId;
        });
        route.cost = calculateRouteCost(route);
    }

    ll getDistance(int from, int to) {
        return distances[from][to];
    }

    // Helper function to calculate route cost
    ll calculateRouteCost(const Route &route) {
        if (route.list_reqs.empty()) {
            return getDistance(route.depot, route.depot);
        }

        ll total_cost = 0;
        bool has_trailer = false;
        int prev_point = route.depot;

        for (const int request_id : route.list_reqs) {
            const Request &req = requests[request_id];

            // Handle trailer pickup if needed
            if (req.pickup_action == PICKUP_CONTAINER && !has_trailer) {
                total_cost += getDistance(prev_point, trailer_point) + trailer_pickup_time;
                prev_point = trailer_point;
                has_trailer = true;
            } else if (req.pickup_action == PICKUP_CONTAINER_TRAILER && has_trailer) {
                total_cost += getDistance(prev_point, trailer_point) + trailer_pickup_time;
                prev_point = trailer_point;
                has_trailer = false;
            }

            // Add distance to pickup point and pickup operation
            total_cost += getDistance(prev_point, req.pickup_point);
            total_cost += req.pickup_duration;
            prev_point = req.pickup_point;

            // Add distance to drop point and drop operation
            total_cost += getDistance(prev_point, req.drop_point);
            total_cost += req.drop_duration;
            prev_point = req.drop_point;

            // Update trailer status
            if (req.pickup_action == PICKUP_CONTAINER_TRAILER)
                has_trailer = true;
            if (req.drop_action == DROP_CONTAINER_TRAILER)
                has_trailer = false;
        }

        // Handle final trailer drop if needed
        if (has_trailer) {
            total_cost += getDistance(prev_point, trailer_point) + trailer_pickup_time;
            prev_point = trailer_point;
        }

        // Return to depot
        total_cost += getDistance(prev_point, route.depot);

        return total_cost;
    }

    void removeRandomRequests(std::vector<int> &requestsToRemove, int max_attempt) {
        std::uniform_int_distribution<> routeDist(0, num_vehicles - 1);
        int attempt = 0;

        while (attempt++ < max_attempt) {
            int routeIdx = routeDist(gen);
            Route &route = currentSolution[routeIdx];

            if (!route.list_reqs.empty()) {
                // Collect valid request IDs
                std::vector<int> validRequestIds;
                for (const int request_id : route.list_reqs) {
                    if (std::find(requestsToRemove.begin(), requestsToRemove.end(), request_id) == requestsToRemove.end()) {
                        // Add unique request IDs
                        if (std::find(validRequestIds.begin(), validRequestIds.end(), request_id) == validRequestIds.end()) {
                            validRequestIds.push_back(request_id);
                        }
                    }
                }

                if (!validRequestIds.empty()) {
                    std::uniform_int_distribution<> requestDist(0, validRequestIds.size() - 1);
                    int selectedRequestIdx = requestDist(gen);
                    int selectedRequestId = validRequestIds[selectedRequestIdx];

                    // Remove the selected request
                    removeStopsByRequestId(route, selectedRequestId);
                    requestsToRemove.push_back(selectedRequestId);
                }
            }
        }
    }

    void insertRequests(const std::vector<int> &requestIds) {
        // Create a copy for random shuffling
        std::vector<int> randomRequestIds = requestIds;
        std::shuffle(randomRequestIds.begin(), randomRequestIds.end(), gen);

        // Process each request in random order
        for (int req_id : randomRequestIds) {
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            Route bestRouteConfig;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route &route = currentSolution[routeIdx];
                size_t routeSize = route.size();

                // Try each possible position
                for (size_t pos = 0; pos <= routeSize; pos++) {
                    // Create test route
                    Route testRoute = route;
                    insertRequest(testRoute, req_id, pos);

                    // Check if this is the best position so far
                    if (testRoute.cost < bestCost) {
                        bestCost = testRoute.cost;
                        bestRoute = routeIdx;
                        bestRouteConfig = testRoute;
                    }
                }
            }

            if (bestRoute != -1) {
                currentSolution[bestRoute] = bestRouteConfig;
            }
        }
    }

    const Request &findRequestById(int request_id) const {
        return requests[request_id];
    }

    ll calculateF1() {
        ll maxCost = 0;
        for (const Route &route : currentSolution) {
            maxCost = std::max(maxCost, route.cost);
        }
        return maxCost;
    }

    ll calculateF2() {
        ll totalCost = 0;
        for (const Route &route : currentSolution) {
            totalCost += route.cost;
        }
        return totalCost;
    }

    ll calculateSolutionCost() {
        return alpha * calculateF1() + calculateF2();
    }

    PDPSolver(const std::vector<int> &requestIdx,
              int num_vehicles,
              int alpha,
              int trailer_point,
              int trailer_pickup_time,
              int max_iterations,
              bool verbose)
        : requestIdx(requestIdx),
          num_vehicles(num_vehicles),
          alpha(alpha),
          trailer_point(trailer_point),
          trailer_pickup_time(trailer_pickup_time),
          max_iterations(max_iterations),
          verbose(verbose),
          gen(rd()) {
        for (int i = 0; i < num_vehicles; i++) {
            currentSolution[i].depot = vehicleDepots[i];
        }
    }

    PDPSolver() {}

    void solve() {
        auto start_time = chrono::high_resolution_clock::now();

        insertRequests(requestIdx);

        double currentTemp = temperature;
        ll currentSolutionCost = calculateSolutionCost();
        auto bestSolution = currentSolution;
        ll bestSolutionCost = currentSolutionCost;
        ll bestTotalCost = calculateF2(); // Thêm biến để lưu tổng chi phí tốt nhất

        for (int iter = 0; iter < max_iterations && currentTemp > 1e-8; iter++) {
            auto current_time = chrono::high_resolution_clock::now();
            double elapsed_time = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count() / 1000.0;
            if (elapsed_time >= 26)
                break;
            int numToRemove = max(2, min(40, static_cast<int>(0.1 + (0.3 * (rand() % 100) / 100.0) * requestIdx.size())));

            vector<int> removedRequests;
            removeRandomRequests(removedRequests, numToRemove);
            insertRequests(removedRequests);

            ll newSolutionCost = calculateSolutionCost();
            ll newTotalCost = calculateF2(); // Tính tổng chi phí của solution mới

            if (newSolutionCost < bestSolutionCost) {
                bestSolution = currentSolution;
                bestSolutionCost = newSolutionCost;
                bestTotalCost = newTotalCost;
                currentSolutionCost = newSolutionCost;
            } else if (newSolutionCost == bestSolutionCost) {
                // Nếu có cùng chi phí max, so sánh tổng chi phí
                if (newTotalCost < bestTotalCost) {
                    bestSolution = currentSolution;
                    bestSolutionCost = newSolutionCost;
                    bestTotalCost = newTotalCost;
                    currentSolutionCost = newSolutionCost;
                }
            } else {
                if (currentTemp > 1e-8) {
                    double acceptanceProbability = exp((currentSolutionCost - newSolutionCost) / currentTemp);
                    uniform_real_distribution<> dist(0, 1);
                    if (dist(gen) < acceptanceProbability) {
                        currentSolutionCost = newSolutionCost;
                    } else {
                        currentSolution = bestSolution;
                    }
                }
            }

            // std::cout << "Iter: " << iter
            //           << " Cost: " << bestSolutionCost
            //           << " Total: " << bestTotalCost
            //           << " Time: " << fixed << setprecision(2) << elapsed_time << "s"
            //           << " Temp: " << currentTemp << endl;

            currentTemp *= coolingRate;
            if (elapsed_time >= 26)
                break;
        }

        currentSolution = bestSolution;
    }

    std::array<Route, MAX_VEHICLES> getSolution() {
        return currentSolution;
    }
};

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
            requests[id] = request;
            requestIdx.push_back(id);
        }
    }

    void output_route(const Route &route) {
        // Handle empty route
        if (route.list_reqs.empty()) {
            std::cout << route.depot << " STOP" << std::endl;
            std::cout << "#" << std::endl;
            return;
        }

        int prev_point = route.depot;
        for (auto it = route.list_reqs.begin(); it != route.list_reqs.end(); ++it) {
            const Request &req = requests[*it];

            // Check if need to handle trailer for pickup
            if (req.pickup_action == PICKUP_CONTAINER &&
                (it == route.list_reqs.begin() || requests[*std::prev(it)].drop_action == DROP_CONTAINER_TRAILER)) {
                std::cout << trailer_point << " " << actions[PICKUP_TRAILER] << std::endl;
                prev_point = trailer_point;
            }

            // Output pickup action
            std::cout << req.pickup_point << " " << actions[req.pickup_action] << " " << req.id << std::endl;
            prev_point = req.pickup_point;

            // Output drop action
            std::cout << req.drop_point << " " << actions[req.drop_action] << " " << req.id << std::endl;
            prev_point = req.drop_point;

            // Check if need to handle trailer for drop
            if ((req.drop_action == DROP_CONTAINER &&
                 (std::next(it) == route.list_reqs.end() || requests[*std::next(it)].pickup_action == PICKUP_CONTAINER_TRAILER))) {
                std::cout << trailer_point << " " << actions[DROP_TRAILER] << std::endl;
                prev_point = trailer_point;
            }
        }

        std::cout << route.depot << " STOP" << std::endl;
        std::cout << "#" << std::endl;
    }

    void output() {
        PDPSolver solver(requestIdx, num_vehicles, alpha, trailer_point,
                         trailer_pickup_time, max_iterations, verbose);

        solver.solve();
        // freopen("tc/2/out.txt", "w", stdout);

        std::array<Route, MAX_VEHICLES> solution = solver.getSolution();
        std::cout << "ROUTES " << num_vehicles << std::endl;
        for (size_t i = 0; i < num_vehicles; i++) {
            std::cout << "TRUCK " << i + 1 << std::endl;
            output_route(solution[i]);
        }
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    // freopen("tc/2/inp.txt", "r", stdin);

    IO io(100000, 100000, 0);
    io.input();
    io.output();
    
    return 0;
}
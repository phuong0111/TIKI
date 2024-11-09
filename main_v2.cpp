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
    ll getDistance(int from, int to) {
        return distances[from][to];
    }

    ll calculateRequestContextCost(const Request &req) {
        return req.pickup_duration + req.drop_duration + getDistance(req.pickup_point, req.drop_point);
    }

    ll calculateDepotToRequestCost(const int &depot, const Request &req) {
        if (req.pickup_action == PICKUP_CONTAINER)
            return getDistance(depot, trailer_point) + trailer_pickup_time + getDistance(trailer_point, req.pickup_point);
        return getDistance(depot, req.pickup_point); // PICKUP_CONTAINER_TRAILER
    }

    ll calculateRequestToDepotCost(const Request &req, const int &depot) {
        if (req.drop_action == DROP_CONTAINER)
            return getDistance(req.drop_point, trailer_point) + trailer_pickup_time + getDistance(trailer_point, depot);
        return getDistance(req.drop_point, depot); // DROP_CONTAINER_TRAILER
    }

    ll calculateRequestTransitionCost(const Request &curr_req, const Request &next_req) {
        if (curr_req.drop_action == DROP_CONTAINER && next_req.pickup_action == PICKUP_CONTAINER_TRAILER)
            return getDistance(curr_req.drop_point, trailer_point) + getDistance(trailer_point, next_req.pickup_point) + trailer_pickup_time;
        if (curr_req.drop_action == DROP_CONTAINER_TRAILER && next_req.pickup_action == PICKUP_CONTAINER)
            return getDistance(curr_req.drop_point, trailer_point) + getDistance(trailer_point, next_req.pickup_point) + trailer_pickup_time;
        return getDistance(curr_req.drop_point, next_req.pickup_point);
    }

    ll calculateInsertionCost(const Route &route, const int request_id, std::list<int>::iterator position) {
        const Request &req = requests[request_id];
        ll costDelta = 0;

        if (route.list_reqs.empty()) {
            // If route is empty, just calculate depot -> request -> depot
            return calculateDepotToRequestCost(route.depot, req) +
                   calculateRequestContextCost(req) +
                   calculateRequestToDepotCost(req, route.depot);
        }

        if (position == route.list_reqs.begin()) {
            // Inserting at start of route
            const Request &next_req = requests[*position];
            costDelta = calculateDepotToRequestCost(route.depot, req) +
                        calculateRequestContextCost(req) +
                        calculateRequestTransitionCost(req, next_req) -
                        calculateDepotToRequestCost(route.depot, next_req);
        } else if (position == route.list_reqs.end()) {
            // Inserting at end of route
            const Request &prev_req = requests[*std::prev(position)];
            costDelta = calculateRequestTransitionCost(prev_req, req) +
                        calculateRequestContextCost(req) +
                        calculateRequestToDepotCost(req, route.depot) -
                        calculateRequestToDepotCost(prev_req, route.depot);
        } else {
            // Inserting between two existing requests
            const Request &prev_req = requests[*std::prev(position)];
            const Request &next_req = requests[*position];
            costDelta = calculateRequestTransitionCost(prev_req, req) +
                        calculateRequestContextCost(req) +
                        calculateRequestTransitionCost(req, next_req) -
                        calculateRequestTransitionCost(prev_req, next_req);
        }

        return route.cost + costDelta;
    }

    void removeStopsByRequestId(Route &route, int request_id) {
        route.list_reqs.remove_if([request_id](const int requestId) {
            return request_id == requestId;
        });
        // route.cost = calculateRouteCost(route);
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
        // Create a vector of route indices sorted by cost (descending order)
        std::vector<std::pair<ll, int>> routeCosts;
        for (int i = 0; i < num_vehicles; i++) {
            if (!currentSolution[i].list_reqs.empty()) {
                routeCosts.push_back({currentSolution[i].cost, i});
            }
        }

        // Sort routes by cost in descending order
        std::sort(routeCosts.begin(), routeCosts.end(),
                  [](const auto &a, const auto &b) { return a.first > b.first; });

        if (routeCosts.empty())
            return;

        // Calculate selection probabilities for routes
        std::vector<double> probabilities(routeCosts.size());
        int k = std::min(static_cast<int>(routeCosts.size()), std::max(2, max_attempt / 4));

        // Higher probability for top k expensive routes
        for (size_t i = 0; i < routeCosts.size(); i++) {
            if (i < k) {
                probabilities[i] = 0.8 / k; // 80% chance to select from top k routes
            } else {
                probabilities[i] = 0.2 / (routeCosts.size() - k); // 20% chance for other routes
            }
        }

        int attempt = 0;
        std::set<int> modifiedRouteIdx;
        while (attempt < max_attempt) {
            // Select route based on probabilities
            std::discrete_distribution<> routeDist(probabilities.begin(), probabilities.end());
            int routeIdx = routeCosts[routeDist(gen)].second;
            Route &route = currentSolution[routeIdx];

            if (route.list_reqs.empty())
                continue;

            // Calculate contextual costs for requests in the selected route
            std::vector<std::pair<ll, int>> requestContextCosts;
            auto it = route.list_reqs.begin();

            while (it != route.list_reqs.end()) {
                int curr_req_id = *it;

                // Skip if already marked for removal
                if (std::find(requestsToRemove.begin(), requestsToRemove.end(),
                              curr_req_id) != requestsToRemove.end()) {
                    ++it;
                    continue;
                }

                const Request &curr_req = requests[curr_req_id];
                ll context_cost = 0;

                // Get previous request in route
                int prev_point = route.depot;
                bool has_trailer = false;
                if (it != route.list_reqs.begin()) {
                    auto prev_it = std::prev(it);
                    const Request &prev_req = requests[*prev_it];
                    prev_point = prev_req.drop_point;
                    has_trailer = (prev_req.drop_action != DROP_CONTAINER_TRAILER);
                }

                // Calculate transition cost from previous point/state
                if (curr_req.pickup_action == PICKUP_CONTAINER && !has_trailer) {
                    context_cost += getDistance(prev_point, trailer_point) +
                                    trailer_pickup_time +
                                    getDistance(trailer_point, curr_req.pickup_point);
                } else if (curr_req.pickup_action == PICKUP_CONTAINER_TRAILER && has_trailer) {
                    context_cost += getDistance(prev_point, trailer_point) +
                                    trailer_pickup_time +
                                    getDistance(trailer_point, curr_req.pickup_point);
                } else {
                    context_cost += getDistance(prev_point, curr_req.pickup_point);
                }

                // Get next request in route
                int next_point = route.depot;
                if (std::next(it) != route.list_reqs.end()) {
                    auto next_it = std::next(it);
                    const Request &next_req = requests[*next_it];
                    next_point = next_req.pickup_point;

                    bool needs_trailer_after = (next_req.pickup_action == PICKUP_CONTAINER_TRAILER);
                    bool has_trailer_after = (curr_req.drop_action != DROP_CONTAINER_TRAILER);

                    if (has_trailer_after && needs_trailer_after) {
                        context_cost += getDistance(curr_req.drop_point, trailer_point) +
                                        trailer_pickup_time +
                                        getDistance(trailer_point, next_point);
                    } else {
                        context_cost += getDistance(curr_req.drop_point, next_point);
                    }
                } else {
                    if (curr_req.drop_action != DROP_CONTAINER_TRAILER) {
                        context_cost += getDistance(curr_req.drop_point, trailer_point) +
                                        trailer_pickup_time +
                                        getDistance(trailer_point, route.depot);
                    } else {
                        context_cost += getDistance(curr_req.drop_point, route.depot);
                    }
                }

                requestContextCosts.push_back({context_cost, curr_req_id});
                ++it;
            }

            if (!requestContextCosts.empty()) {
                // Sort by context-aware costs
                std::sort(requestContextCosts.begin(), requestContextCosts.end(),
                          [](const auto &a, const auto &b) { return a.first > b.first; });

                // Select one of the top expensive requests (with some randomness)
                std::uniform_int_distribution<> topDist(0,
                                                        std::min(2, static_cast<int>(requestContextCosts.size()) - 1));
                int selectedIdx = topDist(gen);
                int selectedRequestId = requestContextCosts[selectedIdx].second;

                // Remove the selected request
                removeStopsByRequestId(route, selectedRequestId);
                modifiedRouteIdx.insert(routeIdx);
                requestsToRemove.push_back(selectedRequestId);
            }
            attempt++;
        }

        for (const int idx : modifiedRouteIdx)
            currentSolution[idx].cost = calculateRouteCost(currentSolution[idx]);
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
            std::list<int>::iterator bestPosition;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route &route = currentSolution[routeIdx];
                // Try each possible position
                auto it = route.list_reqs.begin();
                do {
                    ll newCost = calculateInsertionCost(route, req_id, it);

                    if (newCost < bestCost) {
                        bestCost = newCost;
                        bestRoute = routeIdx;
                        bestPosition = it;
                    }

                    if (it == route.list_reqs.end())
                        break;
                    ++it;
                } while (true);
            }

            if (bestRoute != -1) {
                Route &route = currentSolution[bestRoute];
                route.list_reqs.insert(bestPosition, req_id);
                route.cost = bestCost;
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
            if (elapsed_time >= 29.50)
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

            // std::cout << "Iter: " << iter + 1
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
        // freopen("tc/6/out.txt", "w", stdout);

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
    // freopen("tc/6/inp.txt", "r", stdin);

    IO io(100000, 100000, 0);
    io.input();
    io.output();

    return 0;
}
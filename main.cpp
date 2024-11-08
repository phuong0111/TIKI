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

struct StopNode {
    int request_id;
    ContainerSize size;
    int point;
    Action action;
    ll duration;

    StopNode(int req_id, ContainerSize sz, int pt, Action act, ll dur)
        : request_id(req_id), size(sz), point(pt), action(act), duration(dur) {}
};

struct Route {
    int depot;
    std::list<StopNode> stops;
    ll cost;

    Route() : depot(0), cost(0) {}
    Route(int d) : depot(d), cost(0) {}

    size_t size() const {
        return stops.size();
    }
};

std::array<std::array<ll, MAX_POINT>, MAX_POINT> distances;
std::array<int, MAX_VEHICLES> vehicleDepots;
std::array<Route, MAX_VEHICLES> currentSolution;
std::array<Request, MAX_REQUESTS> requests;

class PDPSolver {
  private:
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
    void addStop(Route &route, const StopNode &stop) {
        route.stops.push_back(stop);
    }

    void insertStop(Route &route, const StopNode &stop, int position) {
        auto it = route.stops.begin();
        std::advance(it, position);
        route.stops.insert(it, stop);
    }

    void removeStop(Route &route, int position) {
        auto it = route.stops.begin();
        std::advance(it, position);
        route.stops.erase(it);
    }

    void removeStopsByRequestId(Route &route, int requestId) {
        route.stops.remove_if([requestId](const StopNode &stop) {
            return stop.request_id == requestId;
        });
    }

    ll getDistance(int from, int to) {
        return distances[from][to];
    }

    bool isRouteValid(const Route &route) {
        bool has_trailer = false;
        int current_load = 0;
        std::vector<std::pair<int, ContainerSize>> current_container;

        for (const auto &curr : route.stops) {
            if (curr.action == PICKUP_TRAILER || curr.action == PICKUP_CONTAINER_TRAILER) {
                if (has_trailer)
                    return false;
                has_trailer = true;
            }

            if (curr.action == PICKUP_CONTAINER) {
                if (!has_trailer)
                    return false;
                if (current_load + static_cast<int>(curr.size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(curr.size);
                current_container.push_back({curr.request_id, curr.size});
            } else if (curr.action == PICKUP_CONTAINER_TRAILER) {
                if (current_load + static_cast<int>(curr.size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(curr.size);
                current_container.push_back({curr.request_id, curr.size});
            }

            if (curr.action == DROP_CONTAINER || curr.action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                auto it = std::find_if(current_container.begin(), current_container.end(),
                                       [&curr](const std::pair<int, ContainerSize> &element) {
                                           return element.first == curr.request_id;
                                       });
                if (it == current_container.end())
                    return false;
                current_load -= static_cast<int>(it->second);
                current_container.erase(it);
            }

            if (curr.action == DROP_TRAILER || curr.action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                if (!current_container.empty())
                    return false;
                has_trailer = false;
            }
        }

        return !has_trailer && current_container.empty() && current_load == 0;
    }

    ll calculateRouteCost(const Route &route, bool debug = false) {
        if (route.stops.empty())
            return getDistance(route.depot, route.depot);

        ll totalCost = getDistance(route.depot, route.stops.front().point);

        auto curr = route.stops.begin();
        auto next = std::next(curr);

        while (curr != route.stops.end()) {
            totalCost += curr->duration;
            if (next != route.stops.end()) {
                totalCost += getDistance(curr->point, next->point);
            }
            curr = next;
            if (next != route.stops.end()) {
                ++next;
            }
        }

        totalCost += getDistance(route.stops.back().point, route.depot);

        return totalCost;
    }

    void updateTrailerOperations(Route &route) {
        route.stops.remove_if([](const StopNode &stop) {
            return stop.request_id == -1;
        });

        if (route.stops.empty())
            return;

        bool current_has_trailer = false;
        Route newRoute(route.depot);

        if (route.stops.front().action == PICKUP_CONTAINER) {
            addStop(newRoute, StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time));
            current_has_trailer = true;
        }

        for (const auto &curr : route.stops) {
            if (!current_has_trailer && curr.action == PICKUP_CONTAINER) {
                addStop(newRoute, StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time));
                current_has_trailer = true;
            }

            addStop(newRoute, curr);

            if (curr.action == PICKUP_CONTAINER_TRAILER) {
                current_has_trailer = true;
            } else if (curr.action == DROP_CONTAINER_TRAILER) {
                current_has_trailer = false;
            }
        }

        if (current_has_trailer) {
            addStop(newRoute, StopNode(-1, NONE, trailer_point, DROP_TRAILER, trailer_pickup_time));
        }

        route = newRoute;
        route.cost = calculateRouteCost(newRoute);
    }

    void removeRandomRequests(std::vector<int> &requestsToRemove, int max_attempt) {
        std::uniform_int_distribution<> routeDist(0, num_vehicles - 1);
        int attempt = 0;

        while (attempt++ < max_attempt) {
            int routeIdx = routeDist(gen);
            Route &route = currentSolution[routeIdx];

            if (!route.stops.empty()) {
                std::vector<int> validRequestIds;
                for (const auto &stop : route.stops) {
                    if (stop.request_id != -1 &&
                        std::find(requestsToRemove.begin(), requestsToRemove.end(), stop.request_id) == requestsToRemove.end()) {
                        validRequestIds.push_back(stop.request_id);
                    }
                }

                if (!validRequestIds.empty()) {
                    std::uniform_int_distribution<> requestDist(0, validRequestIds.size() - 1);
                    int selectedRequestIdx = requestDist(gen);
                    int selectedRequestId = validRequestIds[selectedRequestIdx];

                    Route originalRoute = route;
                    removeStopsByRequestId(route, selectedRequestId);
                    updateTrailerOperations(route);

                    if (!isRouteValid(route)) {
                        route = originalRoute;
                        continue;
                    }

                    requestsToRemove.push_back(selectedRequestId);
                }
            }
        }
    }

    const Request &findRequestById(int request_id) const {
        auto it = std::find_if(requests.begin(), requests.end(),
                               [request_id](const Request &req) { return req.id == request_id; });
        if (it == requests.end()) {
            throw std::runtime_error("Request ID not found: " + std::to_string(request_id));
        }
        return *it;
    }

    void insertRequests(const std::vector<int> &requestIds) {
        std::vector<int> randomRequestIds = requestIds;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(randomRequestIds.begin(), randomRequestIds.end(), gen);

        for (int req_id : randomRequestIds) {
            const Request &req = findRequestById(req_id);
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            Route bestRouteConfig;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route &route = currentSolution[routeIdx];
                size_t routeSize = route.size();

                for (size_t pickup_pos = 0; pickup_pos <= routeSize; pickup_pos++) {
                    for (size_t delivery_pos = pickup_pos; delivery_pos <= routeSize; delivery_pos++) {
                        {
                            Route testRoute = route;
                            insertStop(testRoute,
                                       StopNode(req_id, req.size, req.pickup_point, req.pickup_action, req.pickup_duration),
                                       pickup_pos);
                            insertStop(testRoute,
                                       StopNode(req_id, req.size, req.drop_point, req.drop_action, req.drop_duration),
                                       delivery_pos + 1);

                            updateTrailerOperations(testRoute);

                            if (isRouteValid(testRoute)) {
                                ll newCost = calculateRouteCost(testRoute);
                                if (newCost < bestCost) {
                                    bestCost = newCost;
                                    bestRoute = routeIdx;
                                    bestRouteConfig = testRoute;
                                }
                            }
                        }

                        {
                            Route testRoute = route;

                            if (req.pickup_action == PICKUP_CONTAINER_TRAILER) {
                                insertStop(testRoute,
                                           StopNode(-1, NONE, trailer_point, DROP_TRAILER, trailer_pickup_time),
                                           pickup_pos);
                            } else {
                                insertStop(testRoute,
                                           StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time),
                                           pickup_pos);
                            }

                            insertStop(testRoute,
                                       StopNode(req_id, req.size, req.pickup_point, req.pickup_action, req.pickup_duration),
                                       pickup_pos + 1);
                            insertStop(testRoute,
                                       StopNode(req_id, req.size, req.drop_point, req.drop_action, req.drop_duration),
                                       delivery_pos + 2);

                            updateTrailerOperations(testRoute);

                            if (isRouteValid(testRoute)) {
                                ll newCost = calculateRouteCost(testRoute);
                                if (newCost < bestCost) {
                                    bestCost = newCost;
                                    bestRoute = routeIdx;
                                    bestRouteConfig = testRoute;
                                }
                            }
                        }
                    }
                }
            }

            if (bestRoute != -1) {
                currentSolution[bestRoute] = bestRouteConfig;
                currentSolution[bestRoute].cost = calculateRouteCost(currentSolution[bestRoute]);
            }
        }
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

    PDPSolver(int num_vehicles,
              int alpha,
              int trailer_point,
              int trailer_pickup_time,
              int max_iterations,
              bool verbose)
        : num_vehicles(num_vehicles),
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

    void solve() {
        auto start_time = chrono::high_resolution_clock::now();

        vector<int> unassignedRequests;
        for (const Request &req : requests) {
            unassignedRequests.push_back(req.id);
        }
        insertRequests(unassignedRequests);

        double currentTemp = temperature;
        ll currentSolutionCost = calculateSolutionCost();
        auto bestSolution = currentSolution;
        ll bestSolutionCost = currentSolutionCost;
        ll bestTotalCost = calculateF2();

        for (int iter = 0; iter < max_iterations && currentTemp > 1e-8; iter++) {
            auto current_time = chrono::high_resolution_clock::now();
            double elapsed_time = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count() / 1000.0;
            if (elapsed_time >= 26)
                break;
            int numToRemove = max(2, min(40, static_cast<int>(0.1 + (0.3 * (rand() % 100) / 100.0) * requests.size())));

            vector<int> removedRequests;
            removeRandomRequests(removedRequests, numToRemove);
            insertRequests(removedRequests);

            ll newSolutionCost = calculateSolutionCost();
            ll newTotalCost = calculateF2();

            if (newSolutionCost < bestSolutionCost) {
                bestSolution = currentSolution;
                bestSolutionCost = newSolutionCost;
                bestTotalCost = newTotalCost;
                currentSolutionCost = newSolutionCost;
            } else if (newSolutionCost == bestSolutionCost) {
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
            requests[id] = (request);
        }
    }

    void output_route(const Route &route) {
        for (const auto &curr : route.stops) {
            std::cout << curr.point << " " << actions[curr.action];
            if (curr.action == PICKUP_CONTAINER || curr.action == PICKUP_CONTAINER_TRAILER ||
                curr.action == DROP_CONTAINER || curr.action == DROP_CONTAINER_TRAILER) {
                std::cout << " " << curr.request_id;
            }
            std::cout << std::endl;
        }
        std::cout << route.depot << " STOP" << std::endl;
        std::cout << "#" << std::endl;
    }

    void output() {
        PDPSolver solver(num_vehicles, alpha, trailer_point,
                         trailer_pickup_time, max_iterations, verbose);

        solver.solve();

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

    IO io(100000, 1000000, 0);
    io.input();
    io.output();

    return 0;
}
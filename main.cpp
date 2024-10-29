// #include <algorithm>
// #include <iostream>
// #include <random>
// #include <string>
// #include <unordered_map>
// #include <vector>
#include <bits/stdc++.h>

typedef long int ll;

enum Action {
    PICKUP_CONTAINER,
    PICKUP_CONTAINER_TRAILER,
    DROP_CONTAINER,
    DROP_CONTAINER_TRAILER,
    PICKUP_TRAILER,
    DROP_TRAILER,
    STOP
};

const std::vector<std::string> actions = {"PICKUP_CONTAINER", "PICKUP_CONTAINER_TRAILER", "DROP_CONTAINER", "DROP_CONTAINER_TRAILER", "PICKUP_TRAILER", "DROP_TRAILER", "STOP"};

enum ContainerSize {
    NONE = 0,
    TWENTY_FT = 20,
    FORTY_FT = 40
};

struct Request {
    int id;
    ContainerSize size;
    int pickup_point; // Now 1-based
    Action pickup_action;
    ll pickup_duration;
    int drop_point; // Now 1-based
    Action drop_action;
    ll drop_duration;
};

struct Stop {
    int request_id; // -1 for trailer operations, otherwise 1-based
    ContainerSize size;
    int point; // Now 1-based
    Action action;
    ll duration;
};

struct Route {
    int depot; // Now 1-based
    std::vector<Stop> stops;
    ll cost;
};

int getTotalContainerSize(const std::vector<std::pair<int, ContainerSize>> &request_id_container) {
    int totalSize = 0;
    for (std::pair<int, ContainerSize> x : request_id_container)
        totalSize += x.second;
    return totalSize;
}

bool getValidTotalContainerSize(const std::vector<std::pair<int, ContainerSize>> &request_id_container) {
    return getTotalContainerSize(request_id_container) <= 40;
}

class PDPSolver {
  private:
    std::vector<Request> requests;
    std::vector<std::vector<ll>> distances;
    std::vector<int> vehicle_depots;
    int numVehicles;
    std::vector<Route> currentSolution;
    std::random_device rd;
    std::mt19937 gen;
    int alpha;
    int trailer_point; // Now 1-based
    int trailer_pickup_time;
    int max_iterations;
    bool verbose;
    const double temperature = 100.0;
    const double coolingRate = 0.9995;

    ll getDistance(int from, int to) {
        return distances[from][to];
    }

    bool isRouteValid(const Route &route) {
        bool has_trailer = false;
        int current_load = 0; // Track current load
        std::vector<std::pair<int, ContainerSize>> current_container;

        for (auto stop : route.stops) {
            // Trailer logic
            if (stop.action == PICKUP_TRAILER || stop.action == PICKUP_CONTAINER_TRAILER) {
                if (has_trailer)
                    return false;
                has_trailer = true;
            }

            // Container pickup logic
            if (stop.action == PICKUP_CONTAINER) {
                if (!has_trailer)
                    return false;
                // Check if adding this container would exceed capacity
                if (current_load + static_cast<int>(stop.size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(stop.size);
                current_container.push_back({stop.request_id, stop.size});
            } else if (stop.action == PICKUP_CONTAINER_TRAILER) {
                // Check if adding this container would exceed capacity
                if (current_load + static_cast<int>(stop.size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(stop.size);
                current_container.push_back({stop.request_id, stop.size});
            }

            // Container drop logic
            if (stop.action == DROP_CONTAINER || stop.action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                auto it = std::find_if(current_container.begin(), current_container.end(),
                                       [stop](const std::pair<int, ContainerSize> &element) {
                                           return element.first == stop.request_id;
                                       });
                if (it == current_container.end())
                    return false;
                current_load -= static_cast<int>(it->second); // Reduce current load
                current_container.erase(it);
            }

            // Trailer drop logic
            if (stop.action == DROP_TRAILER || stop.action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                if (current_container.size() > 0)
                    return false;
                has_trailer = false;
            }
        }

        // Final checks
        if (has_trailer)
            return false;
        if (!current_container.empty())
            return false;
        if (current_load != 0) // Ensure all containers have been dropped
            return false;

        return true;
    }

    ll calculateRouteCost(const Route &route) {
        if (route.stops.empty())
            return getDistance(route.depot, route.depot);

        ll totalCost = getDistance(route.depot, route.stops[0].point);
        totalCost += route.stops[0].duration;

        for (size_t i = 0; i < route.stops.size() - 1; i++) {
            totalCost += getDistance(route.stops[i].point, route.stops[i + 1].point);
            totalCost += route.stops[i + 1].duration;
        }

        totalCost += getDistance(route.stops.back().point, route.depot);

        return totalCost;
    }

    void updateTrailerOperations(Route &route) {
        route.stops.erase(
            std::remove_if(route.stops.begin(), route.stops.end(),
                           [](const Stop &stop) { return stop.request_id == -1; }),
            route.stops.end());

        if (route.stops.empty())
            return;

        bool current_has_trailer = false;
        std::vector<Stop> new_stops;

        Stop pickup_trailer = {
            -1,
            NONE,
            trailer_point,
            PICKUP_TRAILER,
            trailer_pickup_time};

        if (route.stops[0].action == PICKUP_CONTAINER) {
            new_stops.push_back(pickup_trailer);
            current_has_trailer = true;
        }

        for (const Stop &stop : route.stops) {
            if (!current_has_trailer && stop.action == PICKUP_CONTAINER) {
                new_stops.push_back(pickup_trailer);
                current_has_trailer = true;
            }

            new_stops.push_back(stop);

            if (stop.action == PICKUP_CONTAINER_TRAILER) {
                current_has_trailer = true;
            } else if (stop.action == DROP_CONTAINER_TRAILER) {
                current_has_trailer = false;
            }
        }

        if (current_has_trailer) {
            Stop drop_trailer = {
                -1,
                NONE,
                trailer_point,
                DROP_TRAILER,
                trailer_pickup_time};
            new_stops.push_back(drop_trailer);
        }

        route.stops = new_stops;
    }

    void removeRandomRequests(std::vector<int> &requestsToRemove, int count) {
        std::uniform_int_distribution<> routeDist(0, currentSolution.size() - 1);
        int max_attemp = 20;
        int attempt = 0;

        while (requestsToRemove.size() < count or attempt++ < max_attemp) {
            int routeIdx = routeDist(gen);
            Route &route = currentSolution[routeIdx];

            if (!route.stops.empty()) {
                std::vector<size_t> containerStopIndices;
                for (size_t i = 0; i < route.stops.size(); i++) {
                    if (route.stops[i].request_id != -1) {
                        containerStopIndices.push_back(i);
                    }
                }

                if (!containerStopIndices.empty()) {
                    std::uniform_int_distribution<> stopDist(0, containerStopIndices.size() - 1);
                    size_t selectedIdx = containerStopIndices[stopDist(gen)];
                    int req_id = route.stops[selectedIdx].request_id;

                    if (std::find(requestsToRemove.begin(), requestsToRemove.end(), req_id) == requestsToRemove.end()) {

                        // Store original route state
                        Route originalRoute = route;

                        // Try to remove the request
                        while (true) {
                            auto it = std::find_if(route.stops.begin(), route.stops.end(),
                                                   [req_id](const Stop &stop) {
                                                       return stop.request_id == req_id;
                                                   });
                            if (it != route.stops.end())
                                route.stops.erase(it);
                            else
                                break;
                        }

                        // Update trailer operations
                        updateTrailerOperations(route);

                        // Validate the modified route
                        if (!isRouteValid(route)) {
                            // If invalid, restore original route
                            route = originalRoute;
                            continue; // Try another random selection
                        }

                        // Valid removal - add to removal list
                        requestsToRemove.push_back(req_id);
                    }
                }
            }
        }
    }

    // Add a helper function to find request by ID
    const Request &findRequestById(int request_id) const {
        auto it = std::find_if(requests.begin(), requests.end(),
                               [request_id](const Request &req) { return req.id == request_id; });
        if (it == requests.end()) {
            throw std::runtime_error("Request ID not found: " + std::to_string(request_id));
        }
        return *it;
    }
    void insertRequests(const std::vector<int> &requestIds) {
        for (int req_id : requestIds) {
            const Request &req = findRequestById(req_id);
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            Route newBestRoute;

            Stop pickup = {
                req_id,
                req.size,
                req.pickup_point,
                req.pickup_action,
                req.pickup_duration};

            Stop delivery = {
                req_id,
                req.size,
                req.drop_point,
                req.drop_action,
                req.drop_duration};

            Stop drop_trailer = {
                req_id,
                NONE,
                trailer_point,
                DROP_TRAILER,
                trailer_pickup_time};

            Stop pickup_trailer = {
                req_id,
                NONE,
                trailer_point,
                PICKUP_TRAILER,
                trailer_pickup_time};

            for (size_t routeIdx = 0; routeIdx < currentSolution.size(); routeIdx++) {
                Route &route = currentSolution[routeIdx];

                for (size_t pickup_pos = 0; pickup_pos <= route.stops.size(); pickup_pos++) {
                    for (size_t delivery_pos = pickup_pos; delivery_pos <= route.stops.size(); delivery_pos++) {
                        route.stops.insert(route.stops.begin() + pickup_pos, pickup);
                        route.stops.insert(route.stops.begin() + delivery_pos + 1, delivery);

                        Route testRoute = route;
                        updateTrailerOperations(testRoute);

                        if (isRouteValid(testRoute)) {
                            ll newCost = calculateRouteCost(testRoute);
                            if (newCost < bestCost) {
                                bestCost = newCost;
                                bestRoute = routeIdx;
                                newBestRoute = testRoute;
                            }
                        }

                        route.stops.erase(route.stops.begin() + delivery_pos + 1);
                        route.stops.erase(route.stops.begin() + pickup_pos);

                        if (req.pickup_action == PICKUP_CONTAINER_TRAILER) {
                            route.stops.insert(route.stops.begin() + pickup_pos, drop_trailer);
                            route.stops.insert(route.stops.begin() + pickup_pos + 1, pickup);
                            route.stops.insert(route.stops.begin() + delivery_pos + 2, delivery);

                            testRoute = route;
                            updateTrailerOperations(testRoute);

                            if (isRouteValid(testRoute)) {
                                ll newCost = calculateRouteCost(testRoute);
                                if (newCost < bestCost) {
                                    bestCost = newCost;
                                    bestRoute = routeIdx;
                                    newBestRoute = testRoute;
                                }
                            }

                            route.stops.erase(route.stops.begin() + delivery_pos + 2);
                            route.stops.erase(route.stops.begin() + pickup_pos + 1);
                            route.stops.erase(route.stops.begin() + pickup_pos);
                        } else {
                            route.stops.insert(route.stops.begin() + pickup_pos, pickup_trailer);
                            route.stops.insert(route.stops.begin() + pickup_pos + 1, pickup);
                            route.stops.insert(route.stops.begin() + delivery_pos + 2, delivery);

                            testRoute = route;
                            updateTrailerOperations(testRoute);

                            if (isRouteValid(testRoute)) {
                                ll newCost = calculateRouteCost(testRoute);
                                if (newCost < bestCost) {
                                    bestCost = newCost;
                                    bestRoute = routeIdx;
                                    newBestRoute = testRoute;
                                }
                            }

                            route.stops.erase(route.stops.begin() + delivery_pos + 2);
                            route.stops.erase(route.stops.begin() + pickup_pos + 1);
                            route.stops.erase(route.stops.begin() + pickup_pos);
                        }
                    }
                }
            }

            if (bestRoute != -1) {
                currentSolution[bestRoute] = newBestRoute;
                updateTrailerOperations(currentSolution[bestRoute]);
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

  public:
    PDPSolver(const std::vector<Request> &requests,
              const std::vector<std::vector<ll>> &distances,
              const std::vector<int> &vehicle_depots,
              int alpha,
              int trailer_point,
              int trailer_pickup_time,
              int max_iterations,
              bool verbose)
        : requests(requests),
          distances(distances),
          vehicle_depots(vehicle_depots),
          numVehicles(vehicle_depots.size()),
          alpha(alpha),
          trailer_point(trailer_point),
          trailer_pickup_time(trailer_pickup_time),
          max_iterations(max_iterations),
          verbose(verbose),
          gen(rd()) {
        currentSolution.resize(numVehicles);
        for (int i = 0; i < numVehicles; i++) {
            currentSolution[i].depot = vehicle_depots[i];
        }
    }

    void solve() {
        std::vector<int> unassignedRequests;
        for (const Request &req : requests) {
            unassignedRequests.push_back(req.id); // Use the actual request ID
        }
        insertRequests(unassignedRequests);

        ll currentTemp = temperature;
        ll currentSolutionCost = calculateSolutionCost();
        auto bestSolution = currentSolution;
        ll bestSolutionCost = currentSolutionCost;

        for (int iter = 0; iter < max_iterations; iter++) {
            if (verbose)
                if (iter % 1 == 0) {
                    std::cout << "Iter: " << iter << " Cost: " << bestSolutionCost << std::endl;
                }
            int numToRemove = std::max(2, std::min(40,
                                                   static_cast<int>(0.1 + (0.3 * (std::rand() % 100) / 100.0) * requests.size())));

            std::vector<int> removedRequests;
            removeRandomRequests(removedRequests, numToRemove);
            insertRequests(removedRequests);
            ll newSolutionCost = calculateSolutionCost();

            if (newSolutionCost < bestSolutionCost) {
                bestSolution = currentSolution;
                bestSolutionCost = newSolutionCost;
                currentSolutionCost = newSolutionCost;
            } else {
                double acceptanceProbability = exp((currentSolutionCost - newSolutionCost) / currentTemp);
                std::uniform_real_distribution<> dist(0, 1);
                if (dist(gen) < acceptanceProbability) {
                    currentSolutionCost = newSolutionCost;
                } else {
                    currentSolution = bestSolution;
                }
            }

            currentTemp *= coolingRate;
        }

        currentSolution = bestSolution;
    }

    std::vector<Route> getSolution() {
        return currentSolution;
    }
};

struct IO {
    std::vector<Request> requests;
    std::vector<std::vector<ll>> distances;
    std::vector<int> vehicle_depots;
    int trailer_point;
    int trailer_pickup_time;
    int alpha;
    int max_iterations;
    bool verbose;

    IO(int alpha, int max_iterations, bool verbose = false) : alpha(alpha), max_iterations(max_iterations), verbose(verbose) {}
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
        if (size == 20)
            return TWENTY_FT;
        return FORTY_FT;
    }

    void input() {
        std::string dummy_str;
        int N, dummy_num;
        std::cin >> dummy_str >> N;
        std::cin >> dummy_str >> dummy_num;

        distances.resize(N + 1);
        for (int i = 0; i < distances.size(); i++)
            distances[i].resize(N + 1);
        for (int i = 0; i < N * N; i++) {
            int src, dst;
            std::cin >> src >> dst >> distances[src][dst]; // Convert to 0-based for array storage
        }

        std::cin >> dummy_str >> trailer_point >> trailer_pickup_time;
        // No need to adjust trailer_point anymore as we keep it 1-based

        int num_vehicles;
        std::cin >> dummy_str >> num_vehicles;
        vehicle_depots.resize(num_vehicles);
        for (int i = 0; i < num_vehicles; i++) {
            int truck_id, truck_point;
            std::cin >> truck_id >> truck_point;
            vehicle_depots[truck_id - 1] = truck_point; // Store depot as 1-based
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
            requests.push_back(request);
        }
    }

    void output_route(const Route &route) {
        for (const Stop &stop : route.stops) {
            std::cout << stop.point << " " << actions[stop.action];
            if (stop.action == PICKUP_CONTAINER || stop.action == PICKUP_CONTAINER_TRAILER ||
                stop.action == DROP_CONTAINER || stop.action == DROP_CONTAINER_TRAILER) {
                std::cout << " " << stop.request_id; // Convert to 0-based for array access
            }
            std::cout << std::endl;
        }
        std::cout << route.depot << " " << "STOP" << std::endl;
        std::cout << "#" << std::endl;
    }

    void output() {
        PDPSolver solver(requests, distances, vehicle_depots, alpha, trailer_point, trailer_pickup_time, max_iterations, verbose);
        solver.solve();
        std::vector<Route> solution = solver.getSolution();

        // freopen("tc/6/out.txt", "w", stdout);
        std::cout << "ROUTES " << solution.size() << std::endl;
        for (size_t i = 0; i < solution.size(); i++) {
            const Route &route = solution[i];
            std::cout << "TRUCK " << i + 1 << std::endl;
            output_route(route);
        }
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    // freopen("tc/6/inp.txt", "r", stdin);
    IO io(1000, 20, false);
    io.input();
    io.output();
    return 0;
}
#include <bits/stdc++.h>

typedef long int ll;

const int MAX_POINT = 1001;
const int MAX_VEHICLES = 501;

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
std::array<int, MAX_VEHICLES> vehicle_depots;
std::array<Route, MAX_VEHICLES> currentSolution;

class PDPSolver {
  private:
    std::vector<Request> requests;
    int num_vehicles;
    std::random_device rd;
    std::mt19937 gen;
    int alpha;
    int trailer_point;
    int trailer_pickup_time;
    int max_iterations;
    int max_attempt = 50;
    bool verbose;

    const double temperature = 100.0;
    const double coolingRate = 0.9995;

  public:
    void insertStop(Route &route, const StopNode &stop, int position) {
        if (position == 0) {
            // If inserting at start, handle depot to first stop distance
            if (!route.stops.empty()) {
                route.cost -= getDistance(route.depot, route.stops.front().point); // Remove old depot->first distance
                route.cost += getDistance(route.depot, stop.point);                // Add new depot->new first distance
                route.cost += getDistance(stop.point, route.stops.front().point);  // Add new first->second distance
            } else {
                // If this is the only stop, add depot->stop->depot distance
                route.cost = getDistance(route.depot, stop.point) +
                             getDistance(stop.point, route.depot);
            }
            route.cost += stop.duration; // Add stop duration
            route.stops.push_front(stop);
            return;
        }

        auto it = route.stops.begin();
        for (int i = 0; i < position - 1 && it != route.stops.end(); i++) {
            ++it;
        }

        if (it != route.stops.end()) {
            // Remove old distance between current and next
            auto next = std::next(it);
            if (next != route.stops.end()) {
                route.cost -= getDistance(it->point, next->point);
            } else {
                // If inserting at end, remove old last->depot distance
                route.cost -= getDistance(it->point, route.depot);
            }

            // Add new distances
            route.cost += getDistance(it->point, stop.point); // Previous->new
            if (next != route.stops.end()) {
                route.cost += getDistance(stop.point, next->point); // New->next
            } else {
                route.cost += getDistance(stop.point, route.depot); // New->depot
            }

            route.cost += stop.duration; // Add stop duration
            route.stops.insert(next, stop);
        }
    }

    void removeStop(Route &route, int position) {
        if (route.stops.empty()) {
            return;
        }

        if (position == 0) {
            // Update costs for removing first stop
            route.cost -= route.stops.front().duration; // Remove stop duration
            if (route.stops.size() > 1) {
                route.cost -= getDistance(route.depot, route.stops.front().point);                           // Remove depot->removed
                route.cost -= getDistance(route.stops.front().point, std::next(route.stops.begin())->point); // Remove removed->next
                route.cost += getDistance(route.depot, std::next(route.stops.begin())->point);               // Add depot->new first
            } else {
                // If this was the only stop
                route.cost -= getDistance(route.depot, route.stops.front().point);
                route.cost -= getDistance(route.stops.front().point, route.depot);
            }
            route.stops.pop_front();
            return;
        }

        auto it = route.stops.begin();
        for (int i = 0; i < position - 1 && it != route.stops.end(); i++) {
            ++it;
        }

        if (it != route.stops.end() && std::next(it) != route.stops.end()) {
            auto to_remove = std::next(it);
            // Update costs
            route.cost -= to_remove->duration; // Remove stop duration

            auto next = std::next(to_remove);
            if (next != route.stops.end()) {
                // If removing from middle
                route.cost -= getDistance(it->point, to_remove->point);   // Remove prev->removed
                route.cost -= getDistance(to_remove->point, next->point); // Remove removed->next
                route.cost += getDistance(it->point, next->point);        // Add prev->next
            } else {
                // If removing last stop
                route.cost -= getDistance(it->point, to_remove->point);   // Remove prev->removed
                route.cost -= getDistance(to_remove->point, route.depot); // Remove removed->depot
                route.cost += getDistance(it->point, route.depot);        // Add new last->depot
            }

            route.stops.erase(to_remove);
        }
    }

    void insertStopAfter(Route &route, const StopNode &stop, std::list<StopNode>::iterator after) {
        // Case 1: Insert at beginning (after is begin())
        if (after == route.stops.end()) {
            if (!route.stops.empty()) {
                // Update costs for inserting at start
                route.cost -= getDistance(route.depot, route.stops.front().point); // Remove old depot->first
                route.cost += getDistance(route.depot, stop.point);                // Add new depot->new
                route.cost += getDistance(stop.point, route.stops.front().point);  // Add new->old first
            } else {
                // Empty route, just add depot->stop->depot
                route.cost = getDistance(route.depot, stop.point) +
                             getDistance(stop.point, route.depot);
            }
            route.cost += stop.duration;
            route.stops.push_front(stop);
            return;
        }

        // Case 2: Insert after the given node
        auto next = std::next(after);
        // Remove old distance between current and next
        if (next != route.stops.end()) {
            // Inserting in middle
            route.cost -= getDistance(after->point, next->point);

            // Add new distances
            route.cost += getDistance(after->point, stop.point); // Previous->new
            route.cost += getDistance(stop.point, next->point);  // New->next
        } else {
            // Inserting at end
            route.cost -= getDistance(after->point, route.depot);

            // Add new distances
            route.cost += getDistance(after->point, stop.point); // Previous->new
            route.cost += getDistance(stop.point, route.depot);  // New->depot
        }

        route.cost += stop.duration;
        route.stops.insert(next, stop);
    }

    void removeStopAfter(Route &route, std::list<StopNode>::iterator after) {
        if (route.stops.empty())
            return;

        // If after is end(), remove first stop
        if (after == route.stops.end()) {
            // Update costs
            route.cost -= route.stops.front().duration; // Remove stop duration
            if (route.stops.size() > 1) {
                route.cost -= getDistance(route.depot, route.stops.front().point);                           // Remove depot->removed
                route.cost -= getDistance(route.stops.front().point, std::next(route.stops.begin())->point); // Remove removed->next
                route.cost += getDistance(route.depot, std::next(route.stops.begin())->point);               // Add depot->new first
            } else {
                // If this was the only stop
                route.cost -= getDistance(route.depot, route.stops.front().point);
                route.cost -= getDistance(route.stops.front().point, route.depot);
            }
            route.stops.pop_front();
            return;
        }

        // If trying to remove after the last node
        auto to_remove = std::next(after);
        if (to_remove == route.stops.end())
            return;

        // Update costs
        route.cost -= to_remove->duration; // Remove stop duration

        auto next = std::next(to_remove);
        if (next != route.stops.end()) {
            // Removing from middle
            route.cost -= getDistance(after->point, to_remove->point); // Remove prev->removed
            route.cost -= getDistance(to_remove->point, next->point);  // Remove removed->next
            route.cost += getDistance(after->point, next->point);      // Add prev->next
        } else {
            // Removing last stop
            route.cost -= getDistance(after->point, to_remove->point); // Remove prev->removed
            route.cost -= getDistance(to_remove->point, route.depot);  // Remove removed->depot
            route.cost += getDistance(after->point, route.depot);      // Add new last->depot
        }

        route.stops.erase(to_remove);
    }

    // Helper function to remove stops by request ID
    void removeStopsByRequestId(Route &route, int requestId) {
        // Handle removals from the start of the route
        while (!route.stops.empty() && route.stops.front().request_id == requestId) {
            // Update cost when removing first stop
            route.cost -= route.stops.front().duration; // Remove stop duration
            if (route.stops.size() > 1) {
                // If there's a next stop, update distances
                route.cost -= getDistance(route.depot, route.stops.front().point);                           // Remove depot->removed distance
                route.cost -= getDistance(route.stops.front().point, std::next(route.stops.begin())->point); // Remove removed->next distance
                route.cost += getDistance(route.depot, std::next(route.stops.begin())->point);               // Add depot->new first distance
            } else {
                // If this was the only stop
                route.cost -= getDistance(route.depot, route.stops.front().point); // Remove depot->removed distance
                route.cost -= getDistance(route.stops.front().point, route.depot); // Remove removed->depot distance
            }
            route.stops.pop_front();
        }

        // If all stops were removed
        if (route.stops.empty()) {
            route.cost = getDistance(route.depot, route.depot); // Set cost to empty route
            return;
        }

        // Handle removals from the middle/end of the route
        auto curr = route.stops.begin();
        while (curr != route.stops.end()) {
            auto next = std::next(curr);
            if (next != route.stops.end() && next->request_id == requestId) {
                // Update cost
                route.cost -= next->duration; // Remove stop duration

                auto next_next = std::next(next);
                if (next_next != route.stops.end()) {
                    // If removing from middle
                    route.cost -= getDistance(curr->point, next->point);      // Remove prev->removed distance
                    route.cost -= getDistance(next->point, next_next->point); // Remove removed->next distance
                    route.cost += getDistance(curr->point, next_next->point); // Add prev->next distance
                } else {
                    // If removing last stop
                    route.cost -= getDistance(curr->point, next->point); // Remove prev->removed distance
                    route.cost -= getDistance(next->point, route.depot); // Remove removed->depot distance
                    route.cost += getDistance(curr->point, route.depot); // Add new last->depot distance
                }

                route.stops.erase(next);
            } else {
                curr++;
            }
        }
    }

    ll getDistance(int from, int to) {
        return distances[from][to];
    }

    bool isRouteValid(const Route &route) {
        bool has_trailer = false;
        int current_load = 0;
        std::vector<std::pair<int, ContainerSize>> current_container;

        for (const auto &stop : route.stops) {
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
                if (current_load + static_cast<int>(stop.size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(stop.size);
                current_container.push_back({stop.request_id, stop.size});
            } else if (stop.action == PICKUP_CONTAINER_TRAILER) {
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
                                       [&stop](const std::pair<int, ContainerSize> &element) {
                                           return element.first == stop.request_id;
                                       });
                if (it == current_container.end())
                    return false;
                current_load -= static_cast<int>(it->second);
                current_container.erase(it);
            }

            // Trailer drop logic
            if (stop.action == DROP_TRAILER || stop.action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                if (!current_container.empty())
                    return false;
                has_trailer = false;
            }
        }

        return !has_trailer && current_container.empty() && current_load == 0;
    }

    ll calculateRouteCost(const Route &route) {
        if (route.stops.empty())
            return getDistance(route.depot, route.depot);

        ll totalCost = getDistance(route.depot, route.stops.front().point);
        totalCost += route.stops.front().duration;

        auto curr = route.stops.begin();
        auto next = std::next(curr);

        while (next != route.stops.end()) {
            totalCost += getDistance(curr->point, next->point);
            totalCost += next->duration;
            curr++;
            next++;
        }

        totalCost += getDistance(route.stops.back().point, route.depot);

        return totalCost;
    }

    void updateTrailerOperations(Route &route) {
        if (route.stops.empty()) {
            return;
        }

        bool current_has_trailer = false;
        bool need_trailer = false;

        // First pass: Mark for removal any unnecessary trailer operations
        for (auto curr = route.stops.begin(); curr != route.stops.end(); ++curr) {
            // Check next nodes to see if we need a trailer
            need_trailer = false;
            for (auto look_ahead = std::next(curr); look_ahead != route.stops.end(); ++look_ahead) {
                if (look_ahead->action == PICKUP_CONTAINER ||
                    look_ahead->action == PICKUP_CONTAINER_TRAILER ||
                    look_ahead->action == DROP_CONTAINER ||
                    look_ahead->action == DROP_CONTAINER_TRAILER) {
                    need_trailer = true;
                    break;
                }
            }

            // Update trailer state based on current action
            if (curr->action == PICKUP_TRAILER) {
                if (!need_trailer) {
                    // Mark for removal by setting request_id to -2
                    curr->request_id = -2;
                }
                current_has_trailer = true;
            } else if (curr->action == DROP_TRAILER) {
                if (need_trailer) {
                    // Mark for removal
                    curr->request_id = -2;
                }
                current_has_trailer = false;
            } else if (curr->action == PICKUP_CONTAINER_TRAILER ||
                       curr->action == DROP_CONTAINER_TRAILER) {
                current_has_trailer = (curr->action == PICKUP_CONTAINER_TRAILER);
            }
        }

        // Remove marked nodes
        auto curr = route.stops.begin();
        while (curr != route.stops.end()) {
            if (curr->request_id == -2) {
                curr = route.stops.erase(curr);
            } else {
                ++curr;
            }
        }

        // Second pass: Insert necessary trailer operations
        current_has_trailer = false;
        auto prev = route.stops.end();

        for (auto curr = route.stops.begin(); curr != route.stops.end(); ++curr) {
            if (!current_has_trailer &&
                (curr->action == PICKUP_CONTAINER ||
                 curr->action == DROP_CONTAINER)) {
                // Need trailer but don't have one
                insertStopAfter(route,
                                StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time),
                                prev);
                current_has_trailer = true;
            } else if (curr->action == PICKUP_CONTAINER_TRAILER ||
                       curr->action == PICKUP_TRAILER) {
                current_has_trailer = true;
            } else if (curr->action == DROP_CONTAINER_TRAILER ||
                       curr->action == DROP_TRAILER) {
                current_has_trailer = false;
            }

            prev = curr;
        }

        // Add final trailer drop if needed
        if (current_has_trailer) {
            // Look ahead to see if we really need to keep the trailer
            bool need_final_drop = true;
            bool found_trailer_op = false;

            for (const auto &stop : route.stops) {
                if (stop.action == DROP_TRAILER || stop.action == DROP_CONTAINER_TRAILER) {
                    found_trailer_op = true;
                }
                if (found_trailer_op &&
                    (stop.action == PICKUP_CONTAINER ||
                     stop.action == DROP_CONTAINER ||
                     stop.action == PICKUP_CONTAINER_TRAILER ||
                     stop.action == DROP_CONTAINER_TRAILER)) {
                    need_final_drop = false;
                    break;
                }
            }

            if (need_final_drop) {
                insertStopAfter(route,
                                StopNode(-1, NONE, trailer_point, DROP_TRAILER, trailer_pickup_time),
                                prev);
            }
        }
    }

    void removeRandomRequests(std::vector<int> &requestsToRemove, int max_attempt) {
        std::uniform_int_distribution<> routeDist(0, num_vehicles - 1);
        int attempt = 0;

        while (attempt++ < max_attempt) {
            int routeIdx = routeDist(gen);
            Route &route = currentSolution[routeIdx];

            if (!route.stops.empty()) {
                // Count valid stops
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

                    // Store original route
                    Route originalRoute = route;

                    // Remove the selected request
                    removeStopsByRequestId(route, selectedRequestId);

                    // Update trailer operations
                    updateTrailerOperations(route);

                    // Validate modified route
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
        // Create a copy to shuffle randomly
        std::vector<int> randomRequestIds = requestIds;
        std::shuffle(randomRequestIds.begin(), randomRequestIds.end(), gen);

        // Process each request in random order
        for (int req_id : randomRequestIds) {
            const Request &req = findRequestById(req_id);
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            Route bestRouteConfig;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route &route = currentSolution[routeIdx];
                size_t routeSize = route.size();

                auto pickup_it = route.stops.end();
                size_t pickup_pos = 0;
                do {
                    // Skip invalid container size combinations
                    if (pickup_it != route.stops.end() &&
                        (pickup_it->action == PICKUP_CONTAINER || pickup_it->action == PICKUP_CONTAINER_TRAILER) &&
                        (req.size == FORTY_FT || pickup_it->size == FORTY_FT)) {
                        (pickup_it != route.stops.end()) ? ++pickup_it : (pickup_it = route.stops.begin());
                        ++pickup_pos;
                        continue;
                    }

                    auto drop_it = pickup_it;
                    size_t delivery_pos = pickup_pos;
                    do {
                        
                        // Try regular container operations
                        {
                            Route testRoute = route;

                            // Insert pickup and delivery
                            insertStop(testRoute,
                                       StopNode(req_id, req.size, req.pickup_point, req.pickup_action, req.pickup_duration),
                                       pickup_pos);
                            insertStop(testRoute,
                                       StopNode(req_id, req.size, req.drop_point, req.drop_action, req.drop_duration),
                                       delivery_pos + 1);

                            updateTrailerOperations(testRoute);

                            if (isRouteValid(testRoute)) {
                                ll newCost = testRoute.cost;
                                if (newCost < bestCost) {
                                    bestCost = newCost;
                                    bestRoute = routeIdx;
                                    bestRouteConfig = testRoute;
                                }
                            }
                        }

                        (drop_it != route.stops.end()) ? ++drop_it : (drop_it = route.stops.begin());
                        ++delivery_pos;
                    } while (delivery_pos <= ((req.size == TWENTY_FT) ? routeSize : pickup_pos));

                    (pickup_it != route.stops.end()) ? ++pickup_it : (pickup_it = route.stops.begin());
                    ++pickup_pos;
                } while (pickup_pos <= routeSize);
            }

            if (bestRoute != -1) {
                currentSolution[bestRoute] = bestRouteConfig;
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

    PDPSolver(const std::vector<Request> &requests,
              int num_vehicles,
              int alpha,
              int trailer_point,
              int trailer_pickup_time,
              int max_iterations,
              bool verbose)
        : requests(requests),
          num_vehicles(num_vehicles),
          alpha(alpha),
          trailer_point(trailer_point),
          trailer_pickup_time(trailer_pickup_time),
          max_iterations(max_iterations),
          verbose(verbose),
          gen(rd()) {
        for (int i = 0; i < num_vehicles; i++) {
            currentSolution[i].depot = vehicle_depots[i];
        }
    }

    void solve() {
        // Add time tracking
        const auto start_time = std::chrono::steady_clock::now();
        const int TIME_LIMIT_SECONDS = 30 - 2;

        // Initialize unassigned requests
        std::vector<int> unassignedRequests;
        for (const Request &req : requests) {
            unassignedRequests.push_back(req.id);
        }
        // std::cout << "Initialize: ";
        insertRequests(unassignedRequests);

        ll currentTemp = temperature;
        ll currentSolutionCost = calculateSolutionCost();
        auto bestSolution = currentSolution;
        ll bestSolutionCost = currentSolutionCost;
        ll lastBestCost = bestSolutionCost;

        for (int iter = 0; iter < max_iterations; iter++) {
            // Check time limit
            const auto current_time = std::chrono::steady_clock::now();
            const auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (elapsed_time >= TIME_LIMIT_SECONDS) {
                if (verbose) {
                    std::cout << "Time limit reached after " << iter << " iterations" << std::endl;
                }
                break;
            }

            std::vector<int> removedRequests;
            removeRandomRequests(removedRequests, max_attempt);
            // std::cout << "Iter: " << iter + 1 << " ";
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

            lastBestCost = bestSolutionCost;
            currentTemp *= coolingRate;

            if (verbose) {
                std::cout << "Iter: " << iter + 1
                          << " Cost: " << bestSolutionCost << std::endl;
            }
        }

        currentSolution = bestSolution;
    }

    std::array<Route, MAX_VEHICLES> getSolution() {
        return currentSolution;
    }
};

struct IO {
    std::vector<Request> requests;
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
            vehicle_depots[truck_id - 1] = truck_point;
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
        for (const auto &stop : route.stops) {
            std::cout << stop.point << " " << actions[stop.action];
            if (stop.action == PICKUP_CONTAINER || stop.action == PICKUP_CONTAINER_TRAILER ||
                stop.action == DROP_CONTAINER || stop.action == DROP_CONTAINER_TRAILER) {
                std::cout << " " << stop.request_id;
            }
            std::cout << std::endl;
        }
        std::cout << route.depot << " STOP" << std::endl;
        std::cout << "#" << std::endl;
    }

    void output() {
        PDPSolver solver(requests, num_vehicles, alpha, trailer_point,
                         trailer_pickup_time, max_iterations, verbose);

        solver.solve();
        std::array<Route, MAX_VEHICLES> solution = solver.getSolution();

        freopen("tc/6/out.txt", "w", stdout);
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
    freopen("tc/6/inp.txt", "r", stdin);

    IO io(100000, 0, 1);
    io.input();
    io.output();

    return 0;
}
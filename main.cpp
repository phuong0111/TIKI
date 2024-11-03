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
    StopNode *next;

    // Constructor
    StopNode(int req_id, ContainerSize sz, int pt, Action act, ll dur)
        : request_id(req_id), size(sz), point(pt), action(act), duration(dur), next(nullptr) {}

    // Copy constructor
    StopNode(const StopNode &other)
        : request_id(other.request_id),
          size(other.size),
          point(other.point),
          action(other.action),
          duration(other.duration),
          next(nullptr) {}
};

struct Route {
    int depot;
    StopNode *stops;
    ll cost;

    // Default constructor
    Route() : depot(0), stops(nullptr), cost(0) {}

    // Constructor with depot
    Route(int d) : depot(d), stops(nullptr), cost(0) {}

    // Deep copy constructor
    Route(const Route &other) : depot(other.depot), cost(other.cost) {
        if (other.stops == nullptr) {
            stops = nullptr;
            return;
        }

        stops = new StopNode(*other.stops);
        StopNode *curr = stops;
        StopNode *otherCurr = other.stops->next;

        while (otherCurr != nullptr) {
            curr->next = new StopNode(*otherCurr);
            curr = curr->next;
            otherCurr = otherCurr->next;
        }
    }

    // Assignment operator
    Route &operator=(const Route &other) {
        if (this != &other) {
            // Delete existing nodes
            StopNode *current = stops;
            while (current != nullptr) {
                StopNode *next = current->next;
                delete current;
                current = next;
            }

            depot = other.depot;
            cost = other.cost;

            if (other.stops == nullptr) {
                stops = nullptr;
                return *this;
            }

            stops = new StopNode(*other.stops);
            StopNode *curr = stops;
            StopNode *otherCurr = other.stops->next;

            while (otherCurr != nullptr) {
                curr->next = new StopNode(*otherCurr);
                curr = curr->next;
                otherCurr = otherCurr->next;
            }
        }
        return *this;
    }

    // Destructor
    ~Route() {
        StopNode *current = stops;
        while (current != nullptr) {
            StopNode *next = current->next;
            delete current;
            current = next;
        }
    }

    // Get size of the route (number of stops)
    size_t size() const {
        size_t count = 0;
        StopNode *current = stops;
        while (current != nullptr) {
            count++;
            current = current->next;
        }
        return count;
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
    int max_attempt = 20;
    bool verbose;

    const double temperature = 100.0;
    const double coolingRate = 0.9995;

  public:
    // Helper function to insert a stop at specific position
    void insertStop(Route &route, const StopNode &stop, int position) {
        StopNode *newNode = new StopNode(stop);
        if (position == 0) {
            // If inserting at start, handle depot to first stop distance
            if (route.stops != nullptr) {
                route.cost -= getDistance(route.depot, route.stops->point); // Remove old depot->first distance
                route.cost += getDistance(route.depot, stop.point);         // Add new depot->new first distance
                route.cost += getDistance(stop.point, route.stops->point);  // Add new first->second distance
            } else {
                // If this is the only stop, add depot->stop->depot distance
                route.cost = getDistance(route.depot, stop.point) +
                             getDistance(stop.point, route.depot);
            }
            route.cost += stop.duration; // Add stop duration

            newNode->next = route.stops;
            route.stops = newNode;
            return;
        }

        StopNode *curr = route.stops;
        int currentPos = 0;
        while (curr != nullptr && currentPos < position - 1) {
            curr = curr->next;
            currentPos++;
        }

        if (curr != nullptr) {
            // Remove old distance between current and next
            if (curr->next != nullptr) {
                route.cost -= getDistance(curr->point, curr->next->point);
            } else {
                // If inserting at end, remove old last->depot distance
                route.cost -= getDistance(curr->point, route.depot);
            }

            // Add new distances
            route.cost += getDistance(curr->point, stop.point); // Previous->new
            if (curr->next != nullptr) {
                route.cost += getDistance(stop.point, curr->next->point); // New->next
            } else {
                route.cost += getDistance(stop.point, route.depot); // New->depot
            }

            route.cost += stop.duration; // Add stop duration

            newNode->next = curr->next;
            curr->next = newNode;
        }
    }

    // Helper function to insert stop method with pointer parameter
    void insertStopAfter(Route &route, const StopNode &stop, StopNode *after) {
        StopNode *newNode = new StopNode(stop);

        // Case 1: Insert at beginning (after is nullptr)
        if (after == nullptr) {
            if (route.stops != nullptr) {
                // Update costs for inserting at start
                route.cost -= getDistance(route.depot, route.stops->point); // Remove old depot->first
                route.cost += getDistance(route.depot, stop.point);         // Add new depot->new
                route.cost += getDistance(stop.point, route.stops->point);  // Add new->old first
            } else {
                // Empty route, just add depot->stop->depot
                route.cost = getDistance(route.depot, stop.point) +
                             getDistance(stop.point, route.depot);
            }
            route.cost += stop.duration;

            newNode->next = route.stops;
            route.stops = newNode;
            return;
        }

        // Case 2: Insert after the given node
        // Remove old distance between current and next
        if (after->next != nullptr) {
            // Inserting in middle
            route.cost -= getDistance(after->point, after->next->point);

            // Add new distances
            route.cost += getDistance(after->point, stop.point);       // Previous->new
            route.cost += getDistance(stop.point, after->next->point); // New->next
        } else {
            // Inserting at end
            route.cost -= getDistance(after->point, route.depot);

            // Add new distances
            route.cost += getDistance(after->point, stop.point); // Previous->new
            route.cost += getDistance(stop.point, route.depot);  // New->depot
        }

        route.cost += stop.duration;

        // Insert the node
        newNode->next = after->next;
        after->next = newNode;
    }

    void removeStopAfter(Route &route, StopNode *after) {
        if (route.stops == nullptr)
            return;

        // If after is nullptr, remove first stop
        if (after == nullptr) {
            StopNode *temp = route.stops;

            // Update costs
            route.cost -= temp->duration; // Remove stop duration
            if (temp->next != nullptr) {
                route.cost -= getDistance(route.depot, temp->point);       // Remove depot->removed
                route.cost -= getDistance(temp->point, temp->next->point); // Remove removed->next
                route.cost += getDistance(route.depot, temp->next->point); // Add depot->new first
            } else {
                // If this was the only stop
                route.cost -= getDistance(route.depot, temp->point);
                route.cost -= getDistance(temp->point, route.depot);
            }

            route.stops = route.stops->next;
            delete temp;
            return;
        }

        // If trying to remove after the last node
        if (after->next == nullptr)
            return;

        StopNode *temp = after->next;

        // Update costs
        route.cost -= temp->duration; // Remove stop duration

        if (temp->next != nullptr) {
            // Removing from middle
            route.cost -= getDistance(after->point, temp->point);       // Remove prev->removed
            route.cost -= getDistance(temp->point, temp->next->point);  // Remove removed->next
            route.cost += getDistance(after->point, temp->next->point); // Add prev->next
        } else {
            // Removing last stop
            route.cost -= getDistance(after->point, temp->point); // Remove prev->removed
            route.cost -= getDistance(temp->point, route.depot);  // Remove removed->depot
            route.cost += getDistance(after->point, route.depot); // Add new last->depot
        }

        after->next = temp->next;
        delete temp;
    }

    // Helper function to remove stops by request ID
    void removeStopsByRequestId(Route &route, int requestId) {
        // Handle removals from the start of the route
        while (route.stops != nullptr && route.stops->request_id == requestId) {
            StopNode *temp = route.stops;

            // Update cost when removing first stop
            route.cost -= temp->duration; // Remove stop duration
            if (temp->next != nullptr) {
                // If there's a next stop, update distances
                route.cost -= getDistance(route.depot, temp->point);       // Remove depot->removed distance
                route.cost -= getDistance(temp->point, temp->next->point); // Remove removed->next distance
                route.cost += getDistance(route.depot, temp->next->point); // Add depot->new first distance
            } else {
                // If this was the only stop
                route.cost -= getDistance(route.depot, temp->point); // Remove depot->removed distance
                route.cost -= getDistance(temp->point, route.depot); // Remove removed->depot distance
            }

            route.stops = route.stops->next;
            delete temp;
        }

        // If all stops were removed
        if (route.stops == nullptr) {
            route.cost = getDistance(route.depot, route.depot); // Set cost to empty route
            return;
        }

        // Handle removals from the middle/end of the route
        StopNode *curr = route.stops;
        while (curr->next != nullptr) {
            if (curr->next->request_id == requestId) {
                StopNode *temp = curr->next;

                // Update cost
                route.cost -= temp->duration; // Remove stop duration

                if (temp->next != nullptr) {
                    // If removing from middle
                    route.cost -= getDistance(curr->point, temp->point);       // Remove prev->removed distance
                    route.cost -= getDistance(temp->point, temp->next->point); // Remove removed->next distance
                    route.cost += getDistance(curr->point, temp->next->point); // Add prev->next distance
                } else {
                    // If removing last stop
                    route.cost -= getDistance(curr->point, temp->point); // Remove prev->removed distance
                    route.cost -= getDistance(temp->point, route.depot); // Remove removed->depot distance
                    route.cost += getDistance(curr->point, route.depot); // Add new last->depot distance
                }

                curr->next = curr->next->next;
                delete temp;
            } else {
                curr = curr->next;
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

        StopNode *curr = route.stops;
        while (curr != nullptr) {
            // Trailer logic
            if (curr->action == PICKUP_TRAILER || curr->action == PICKUP_CONTAINER_TRAILER) {
                if (has_trailer)
                    return false;
                has_trailer = true;
            }

            // Container pickup logic
            if (curr->action == PICKUP_CONTAINER) {
                if (!has_trailer)
                    return false;
                if (current_load + static_cast<int>(curr->size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(curr->size);
                current_container.push_back({curr->request_id, curr->size});
            } else if (curr->action == PICKUP_CONTAINER_TRAILER) {
                if (current_load + static_cast<int>(curr->size) > FORTY_FT)
                    return false;
                current_load += static_cast<int>(curr->size);
                current_container.push_back({curr->request_id, curr->size});
            }

            // Container drop logic
            if (curr->action == DROP_CONTAINER || curr->action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                auto it = std::find_if(current_container.begin(), current_container.end(),
                                       [curr](const std::pair<int, ContainerSize> &element) {
                                           return element.first == curr->request_id;
                                       });
                if (it == current_container.end())
                    return false;
                current_load -= static_cast<int>(it->second);
                current_container.erase(it);
            }

            // Trailer drop logic
            if (curr->action == DROP_TRAILER || curr->action == DROP_CONTAINER_TRAILER) {
                if (!has_trailer)
                    return false;
                if (!current_container.empty())
                    return false;
                has_trailer = false;
            }

            curr = curr->next;
        }

        return !has_trailer && current_container.empty() && current_load == 0;
    }

    ll calculateRouteCost(const Route &route, bool debug = false) {
        if (route.stops == nullptr)
            return getDistance(route.depot, route.depot);

        ll totalCost = getDistance(route.depot, route.stops->point);
        totalCost += route.stops->duration;

        StopNode *curr = route.stops;
        while (curr->next != nullptr) {
            totalCost += getDistance(curr->point, curr->next->point);
            totalCost += curr->next->duration;
            curr = curr->next;
        }

        totalCost += getDistance(curr->point, route.depot);

        return totalCost;
    }

    void updateTrailerOperations(Route &route) {
        if (route.stops == nullptr)
            return;

        bool current_has_trailer = false;
        StopNode *curr = route.stops;
        StopNode *prev = nullptr;

        // First pass: Check initial state and need for initial trailer pickup
        if (curr->action == PICKUP_CONTAINER) {
            insertStopAfter(route,
                            StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time),
                            nullptr); // nullptr indicates insert at start
            current_has_trailer = true;
        }

        // Main pass: Process all stops
        while (curr != nullptr) {
            if (curr->action == PICKUP_CONTAINER_TRAILER ||
                curr->action == PICKUP_TRAILER) {
                current_has_trailer = true;
            } else if (curr->action == DROP_CONTAINER_TRAILER ||
                       curr->action == DROP_TRAILER) {
                current_has_trailer = false;
            } else if (!current_has_trailer && curr->action == PICKUP_CONTAINER) {
                // Need trailer but don't have one
                insertStopAfter(route,
                                StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time),
                                prev);
                current_has_trailer = true;
            }

            prev = curr;
            curr = curr->next;
        }

        // Add final trailer drop if needed
        if (current_has_trailer) {
            insertStopAfter(route,
                            StopNode(-1, NONE, trailer_point, DROP_TRAILER, trailer_pickup_time),
                            prev);
        }
    }

    void removeRandomRequests(std::vector<int> &requestsToRemove, int max_attempt) {
        std::uniform_int_distribution<> routeDist(0, num_vehicles - 1);
        int attempt = 0;

        while (attempt++ < max_attempt) {
            int routeIdx = routeDist(gen);
            Route &route = currentSolution[routeIdx];

            if (route.stops != nullptr) {
                // Count valid stops
                std::vector<int> validRequestIds;
                StopNode *curr = route.stops;
                while (curr != nullptr) {
                    if (curr->request_id != -1 &&
                        std::find(requestsToRemove.begin(), requestsToRemove.end(), curr->request_id) == requestsToRemove.end()) {
                        validRequestIds.push_back(curr->request_id);
                    }
                    curr = curr->next;
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
        // Tạo một bản sao để có thể sắp xếp ngẫu nhiên
        std::vector<int> randomRequestIds = requestIds;

        // Tạo random generator
        std::random_device rd;
        std::mt19937 gen(rd());

        // Shuffle ngẫu nhiên danh sách request IDs
        // std::shuffle(randomRequestIds.begin(), randomRequestIds.end(), gen);

        // Sau đó xử lý từng request theo thứ tự ngẫu nhiên mới
        for (int req_id : randomRequestIds) {
            const Request &req = findRequestById(req_id);
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            Route bestRouteConfig;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route &route = currentSolution[routeIdx];
                size_t routeSize = route.size();

                StopNode *pickup_ptr = nullptr;
                for (size_t pickup_pos = 0; pickup_pos <= routeSize; pickup_pos++) {
                    // Added optimization: Skip invalid container size combinations
                    if (pickup_ptr != nullptr && (pickup_ptr->action == PICKUP_CONTAINER || pickup_ptr->action == PICKUP_CONTAINER_TRAILER) && (req.size == FORTY_FT || pickup_ptr->size == FORTY_FT)) {
                        pickup_ptr = (pickup_ptr == nullptr) ? route.stops : pickup_ptr->next;
                        continue;
                    }
                    StopNode *drop_ptr = pickup_ptr;
                    for (size_t delivery_pos = pickup_pos; delivery_pos <= ((req.size == TWENTY_FT) ? routeSize : pickup_pos); delivery_pos++) {
                        std::cout << delivery_pos << std::endl;
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

                        // Try container with trailer operations if applicable
                        {
                            Route testRoute = route;

                            // Add trailer operations
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
                                ll newCost = testRoute.cost;
                                if (newCost < bestCost) {
                                    bestCost = newCost;
                                    bestRoute = routeIdx;
                                    bestRouteConfig = testRoute;
                                }
                            }
                        }
                        drop_ptr = (drop_ptr == nullptr) ? route.stops : drop_ptr->next;
                    }
                    pickup_ptr = (pickup_ptr == nullptr) ? route.stops : pickup_ptr->next;
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
        StopNode *curr = route.stops;
        while (curr != nullptr) {
            std::cout << curr->point << " " << actions[curr->action];
            if (curr->action == PICKUP_CONTAINER || curr->action == PICKUP_CONTAINER_TRAILER ||
                curr->action == DROP_CONTAINER || curr->action == DROP_CONTAINER_TRAILER) {
                std::cout << " " << curr->request_id;
            }
            std::cout << std::endl;
            curr = curr->next;
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
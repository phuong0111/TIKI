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

struct Route1 {
    int depot;
    StopNode *stops;
    ll cost;

    // Default constructor
    Route1() : depot(0), stops(nullptr), cost(0) {}

    // Constructor with depot
    Route1(int d) : depot(d), stops(nullptr), cost(0) {}

    // Deep copy constructor
    Route1(const Route1 &other) : depot(other.depot), cost(other.cost) {
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
    Route1 &operator=(const Route1 &other) {
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
    ~Route1() {
        StopNode *current = stops;
        while (current != nullptr) {
            StopNode *next = current->next;
            delete current;
            current = next;
        }
    }

    // Get size of the Route1 (number of stops)
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

struct RequestContext {
    int request_id;
    ll transitionCost;                 // Cost of transitions (prev->curr->next)
    int routeIdx;                      // Which route this request is in
    std::list<int>::iterator position; // Position in route
};

std::array<std::array<ll, MAX_POINT>, MAX_POINT> distances;
std::array<int, MAX_VEHICLES> vehicleDepots;
std::array<int, MAX_REQUESTS> Matched, Real_idx;
std::array<Route1, MAX_VEHICLES> currentSolution1;

std::array<Route, MAX_VEHICLES> currentSolution;
std::array<Request, MAX_REQUESTS> requests, requests_20ft, requests_origin;
std::array<RequestContext, MAX_REQUESTS> requestContexts;
std::array<bool, MAX_REQUESTS> isRequestRemoved;
std::vector<std::vector<ll>> graphWeight;
std::vector<std::vector<int>> combinationType;
int idx = 0, n_new = 0, n_total = 0;

std::array<std::array<ll, MAX_REQUESTS>, MAX_REQUESTS> transitionCostRequestRequest;
std::array<std::array<ll, MAX_REQUESTS>, MAX_POINT> transitionCostDepotRequest;
std::array<std::array<ll, MAX_POINT>, MAX_REQUESTS> transitionCostRequestDepot;
std::array<ll, MAX_REQUESTS> requestContextCost;

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

    ll calculateRequestContextCost(const int &request_id) {
        if (requestContextCost[request_id] == 0) {
            const Request &req = requests[request_id];
            requestContextCost[request_id] = req.pickup_duration + req.drop_duration + getDistance(req.pickup_point, req.drop_point);
        }

        return requestContextCost[request_id];
    }

    ll calculateDepotToRequestCost(const int &depot, const int &req_id) {
        if (transitionCostDepotRequest[depot][req_id] == 0) {
            const Request &req = requests[req_id];
            if (req.pickup_action == PICKUP_CONTAINER)
                transitionCostDepotRequest[depot][req_id] = getDistance(depot, trailer_point) + trailer_pickup_time + getDistance(trailer_point, req.pickup_point);
            else
                transitionCostDepotRequest[depot][req_id] = getDistance(depot, req.pickup_point); // PICKUP_CONTAINER_TRAILER
        }

        return transitionCostDepotRequest[depot][req_id];
    }

    ll calculateRequestToDepotCost(const int &req_id, const int &depot) {
        if (transitionCostRequestDepot[req_id][depot] == 0) {
            const Request &req = requests[req_id];
            if (req.drop_action == DROP_CONTAINER)
                transitionCostRequestDepot[req_id][depot] = getDistance(req.drop_point, trailer_point) + trailer_pickup_time + getDistance(trailer_point, depot);
            else
                transitionCostRequestDepot[req_id][depot] = getDistance(req.drop_point, depot); // DROP_CONTAINER_TRAILER
        }

        return transitionCostRequestDepot[req_id][depot];
    }

    ll calculateRequestTransitionCost(const int &curr_req_id, const int &next_req_id) {
        if (transitionCostRequestRequest[curr_req_id][next_req_id] == 0) {
            const Request &curr_req = requests[curr_req_id];
            const Request &next_req = requests[next_req_id];
            if ((curr_req.drop_action == DROP_CONTAINER && next_req.pickup_action == PICKUP_CONTAINER_TRAILER) ||
                (curr_req.drop_action == DROP_CONTAINER_TRAILER && next_req.pickup_action == PICKUP_CONTAINER))
                transitionCostRequestRequest[curr_req_id][next_req_id] = getDistance(curr_req.drop_point, trailer_point) + getDistance(trailer_point, next_req.pickup_point) + trailer_pickup_time;
            else
                transitionCostRequestRequest[curr_req_id][next_req_id] = getDistance(curr_req.drop_point, next_req.pickup_point);
        }

        return transitionCostRequestRequest[curr_req_id][next_req_id];
    }

    ll calculateInsertionCost(const Route &route, const int request_id, std::list<int>::iterator position) {
        ll costDelta = 0;

        if (route.list_reqs.empty()) {
            // If route is empty, just calculate depot -> request -> depot
            return calculateDepotToRequestCost(route.depot, request_id) +
                   calculateRequestContextCost(request_id) +
                   calculateRequestToDepotCost(request_id, route.depot);
        }

        if (position == route.list_reqs.begin()) {
            // Inserting at start of route
            const int &next_req_id = *position;
            costDelta = calculateDepotToRequestCost(route.depot, request_id) +
                        calculateRequestContextCost(request_id) +
                        calculateRequestTransitionCost(request_id, next_req_id) -
                        calculateDepotToRequestCost(route.depot, next_req_id);
        } else if (position == route.list_reqs.end()) {
            // Inserting at end of route
            const int &prev_req_id = *std::prev(position);
            costDelta = calculateRequestTransitionCost(prev_req_id, request_id) +
                        calculateRequestContextCost(request_id) +
                        calculateRequestToDepotCost(request_id, route.depot) -
                        calculateRequestToDepotCost(prev_req_id, route.depot);
        } else {
            // Inserting between two existing requests
            const int &prev_req_id = *std::prev(position);
            const int &next_req_id = *position;
            costDelta = calculateRequestTransitionCost(prev_req_id, request_id) +
                        calculateRequestContextCost(request_id) +
                        calculateRequestTransitionCost(request_id, next_req_id) -
                        calculateRequestTransitionCost(prev_req_id, next_req_id);
        }

        return route.cost + costDelta;
    }

    ll calculateRemovalCost(Route &route, const int request_id) {
        auto it = std::find(route.list_reqs.begin(), route.list_reqs.end(), request_id);
        if (it == route.list_reqs.end())
            return route.cost; // Request not found

        ll costDelta = -calculateRequestContextCost(request_id); // Remove request's own cost

        if (route.list_reqs.size() == 1) {
            // If this is the only request
            costDelta -= calculateDepotToRequestCost(route.depot, request_id);
            costDelta -= calculateRequestToDepotCost(request_id, route.depot);
            return 0; // Route will be empty
        }

        if (it == route.list_reqs.begin()) {
            // If removing first request
            const int &next_req_id = *std::next(it);
            costDelta -= calculateDepotToRequestCost(route.depot, request_id);
            costDelta -= calculateRequestTransitionCost(request_id, next_req_id);
            costDelta += calculateDepotToRequestCost(route.depot, next_req_id);
        } else if (std::next(it) == route.list_reqs.end()) {
            // If removing last request
            const int &prev_req_id = *std::prev(it);
            costDelta -= calculateRequestTransitionCost(prev_req_id, request_id);
            costDelta -= calculateRequestToDepotCost(request_id, route.depot);
            costDelta += calculateRequestToDepotCost(prev_req_id, route.depot);
        } else {
            // If removing from middle
            const int &prev_req_id = *std::prev(it);
            const int &next_req_id = *std::next(it);
            costDelta -= calculateRequestTransitionCost(prev_req_id, request_id);
            costDelta -= calculateRequestTransitionCost(request_id, next_req_id);
            costDelta += calculateRequestTransitionCost(prev_req_id, next_req_id);
        }

        return route.cost + costDelta;
    }

    void removeStopsByRequestId(Route &route, int request_id) {
        // Calculate new cost before modifying the list
        ll newCost = calculateRemovalCost(route, request_id);

        // Remove the request
        route.list_reqs.remove(request_id);

        // Update route cost
        route.cost = newCost;
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

    // The updateRequestContext method becomes cleaner without trailer tracking
    void updateRequestContext(int req_id, Route &route, std::list<int>::iterator it) {
        RequestContext &context = requestContexts[req_id];
        context.request_id = req_id;
        context.routeIdx = &route - &currentSolution[0];
        context.position = it;

        // Calculate transition cost
        ll transitionCost = 0;

        if (it == route.list_reqs.begin()) {
            // First request
            transitionCost += calculateDepotToRequestCost(route.depot, req_id);
            if (std::next(it) != route.list_reqs.end()) {
                const int &next_req_id = *std::next(it);
                transitionCost += calculateRequestTransitionCost(req_id, next_req_id);
                transitionCost -= calculateDepotToRequestCost(route.depot, next_req_id);
            } else {
                transitionCost += calculateRequestToDepotCost(req_id, route.depot);
            }
        } else {
            const int &prev_req_id = *std::prev(it);

            if (std::next(it) == route.list_reqs.end()) {
                // Last request
                transitionCost += calculateRequestTransitionCost(prev_req_id, req_id);
                transitionCost += calculateRequestToDepotCost(req_id, route.depot);
                transitionCost -= calculateRequestToDepotCost(prev_req_id, route.depot);
            } else {
                // Middle request
                const int &next_req_id = *std::next(it);
                transitionCost += calculateRequestTransitionCost(prev_req_id, req_id);
                transitionCost += calculateRequestTransitionCost(req_id, next_req_id);
                transitionCost -= calculateRequestTransitionCost(prev_req_id, next_req_id);
            }
        }

        context.transitionCost = transitionCost;
    }

    void removeRandomRequests(std::vector<int> &requestsToRemove, int max_attempt) {
        // Create a vector of route indices sorted by cost (descending order)
        std::vector<std::pair<ll, int>> routeCosts;
        for (int i = 0; i < num_vehicles; i++) {
            if (!currentSolution[i].list_reqs.empty()) {
                routeCosts.push_back({currentSolution[i].cost, i});
            }
        }

        std::sort(routeCosts.begin(), routeCosts.end(),
                  [](const auto &a, const auto &b) { return a.first > b.first; });

        if (routeCosts.empty())
            return;

        // Calculate selection probabilities for routes
        std::vector<double> probabilities(routeCosts.size());
        int k = std::min(static_cast<int>(routeCosts.size()), std::max(2, max_attempt / 4));

        for (size_t i = 0; i < routeCosts.size(); i++) {
            probabilities[i] = (i < k) ? 0.8 / k : 0.2 / (routeCosts.size() - k);
        }

        int attempt = 0;
        std::set<int> routeIdxWithAllRequestContextUpdated;
        while (requestsToRemove.size() < max_attempt) {
            std::discrete_distribution<> routeDist(probabilities.begin(), probabilities.end());
            int routeIdx = routeCosts[routeDist(gen)].second;
            Route &route = currentSolution[routeIdx];

            if (route.list_reqs.empty())
                continue;

            // Find highest cost requests in this route
            std::vector<std::pair<ll, int>> routeRequestCosts;

            if (routeIdxWithAllRequestContextUpdated.find(routeIdx) == routeIdxWithAllRequestContextUpdated.end()) {
                routeIdxWithAllRequestContextUpdated.insert(routeIdx);
                for (auto it = route.list_reqs.begin(); it != route.list_reqs.end(); ++it) {
                    int req_id = *it;
                    if (!isRequestRemoved[req_id])
                        updateRequestContext(req_id, route, it);
                }
            }
            for (auto it = route.list_reqs.begin(); it != route.list_reqs.end(); ++it) {
                int req_id = *it;
                if (!isRequestRemoved[req_id])
                    routeRequestCosts.push_back({requestContexts[req_id].transitionCost, req_id});
            }

            if (!routeRequestCosts.empty()) {
                std::sort(routeRequestCosts.begin(), routeRequestCosts.end(),
                          [](const auto &a, const auto &b) { return a.first > b.first; });

                std::uniform_int_distribution<> topDist(0,
                                                        std::min(2, static_cast<int>(routeRequestCosts.size()) - 1));
                int selectedIdx = topDist(gen);
                int selectedRequestId = routeRequestCosts[selectedIdx].second;

                // Store prev and next request IDs before removal
                auto currentPos = requestContexts[selectedRequestId].position;
                auto prevPos = currentPos, nextPos = currentPos;
                int prevRequestId = -1, nextRequestId = -1;
                bool hasPrev = currentPos != route.list_reqs.begin();
                bool hasNext = std::next(currentPos) != route.list_reqs.end();

                if (hasPrev) {
                    prevPos = std::prev(currentPos);
                    prevRequestId = *prevPos;
                }
                if (hasNext) {
                    nextPos = std::next(currentPos);
                    nextRequestId = *nextPos;
                }

                // Remove request
                removeStopsByRequestId(route, selectedRequestId);
                requestsToRemove.push_back(selectedRequestId);
                isRequestRemoved[selectedRequestId] = true;

                // Update contexts of adjacent requests after removal
                if (hasPrev) {
                    updateRequestContext(prevRequestId, route, prevPos);
                }
                if (hasNext) {
                    updateRequestContext(nextRequestId, route, nextPos);
                }
            }
            attempt++;
        }
    }

    void insertRequests(const std::vector<int> &requestIds) {
        std::vector<int> randomRequestIds = requestIds;
        std::shuffle(randomRequestIds.begin(), randomRequestIds.end(), gen);

        for (int req_id : randomRequestIds) {
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            std::list<int>::iterator bestPosition;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route &route = currentSolution[routeIdx];
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
                bestPosition = route.list_reqs.insert(bestPosition, req_id);
                route.cost = bestCost;
                isRequestRemoved[req_id] = false;
                requestContexts[req_id].position = bestPosition;
                requestContexts[req_id].routeIdx = bestRoute;
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
        // Initialize removal status
        std::fill(isRequestRemoved.begin(), isRequestRemoved.end(), true);
        insertRequests(requestIdx);

        double currentTemp = temperature;
        ll currentSolutionCost = calculateSolutionCost();
        auto bestSolution = currentSolution;
        ll bestSolutionCost = currentSolutionCost;
        ll bestTotalCost = calculateF2(); // Thêm biến để lưu tổng chi phí tốt nhất

        for (int iter = 0; iter < max_iterations && currentTemp > 1e-8; iter++) {
            auto current_time = chrono::high_resolution_clock::now();
            double elapsed_time = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count() / 1000.0;
            if (elapsed_time >= 25)
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

            // if (iter % 100 == 0)
            //     std::cout << "Iter: " << iter + 1
            //               << " Cost: " << bestSolutionCost
            //               << " Total: " << bestTotalCost
            //               << " Time: " << fixed << setprecision(2) << elapsed_time << "s"
            //               << " Temp: " << currentTemp << endl;

            currentTemp *= coolingRate;
            if (elapsed_time >= 25)
                break;
        }

        currentSolution = bestSolution;
    }

    std::array<Route, MAX_VEHICLES> getSolution() {
        return currentSolution;
    }
};

class PDPSolver1 {
  private:
    std::array<Request, MAX_REQUESTS> requests;
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
    // Helper function to add a stop to the end of Route1
    void addStop(Route1 &route, const StopNode &stop) {
        StopNode *newNode = new StopNode(stop);
        if (route.stops == nullptr) {
            route.stops = newNode;
        } else {
            StopNode *curr = route.stops;
            while (curr->next != nullptr) {
                curr = curr->next;
            }
            curr->next = newNode;
        }
    }

    // Helper function to insert a stop at specific position
    void insertStop(Route1 &route, const StopNode &stop, int position) {
        StopNode *newNode = new StopNode(stop);
        if (position == 0) {
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
            newNode->next = curr->next;
            curr->next = newNode;
        }
    }

    // Helper function to remove a stop from specific position
    void removeStop(Route1 &route, int position) {
        if (route.stops == nullptr)
            return;

        if (position == 0) {
            StopNode *temp = route.stops;
            route.stops = route.stops->next;
            delete temp;
            return;
        }

        StopNode *curr = route.stops;
        int currentPos = 0;
        while (curr != nullptr && currentPos < position - 1) {
            curr = curr->next;
            currentPos++;
        }

        if (curr != nullptr && curr->next != nullptr) {
            StopNode *temp = curr->next;
            curr->next = curr->next->next;
            delete temp;
        }
    }

    // Helper function to remove stops by request ID
    void removeStopsByRequestId(Route1 &route, int requestId) {
        while (route.stops != nullptr && route.stops->request_id == requestId) {
            StopNode *temp = route.stops;
            route.stops = route.stops->next;
            delete temp;
        }

        if (route.stops == nullptr)
            return;

        StopNode *curr = route.stops;
        while (curr->next != nullptr) {
            if (curr->next->request_id == requestId) {
                StopNode *temp = curr->next;
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

    bool isRouteValid(const Route1 &route) {
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

    ll calculateRouteCost(const Route1 &route, bool debug = false) {
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

    void updateTrailerOperations(Route1 &route) {
        // Remove all existing trailer operations
        StopNode *curr = route.stops;
        StopNode *prev = nullptr;

        while (curr != nullptr) {
            if (curr->request_id == -1) {
                if (prev == nullptr) {
                    route.stops = curr->next;
                    delete curr;
                    curr = route.stops;
                } else {
                    prev->next = curr->next;
                    delete curr;
                    curr = prev->next;
                }
            } else {
                prev = curr;
                curr = curr->next;
            }
        }

        if (route.stops == nullptr)
            return;

        bool current_has_trailer = false;
        Route1 newRoute(route.depot);

        // Add trailer pickup if needed for first stop
        if (route.stops->action == PICKUP_CONTAINER) {
            addStop(newRoute, StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time));
            current_has_trailer = true;
        }

        // Process all stops
        curr = route.stops;
        while (curr != nullptr) {
            if (!current_has_trailer && curr->action == PICKUP_CONTAINER) {
                addStop(newRoute, StopNode(-1, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time));
                current_has_trailer = true;
            }

            addStop(newRoute, *curr);

            if (curr->action == PICKUP_CONTAINER_TRAILER) {
                current_has_trailer = true;
            } else if (curr->action == DROP_CONTAINER_TRAILER) {
                current_has_trailer = false;
            }

            curr = curr->next;
        }

        // Add final trailer drop if needed
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
            Route1 &route = currentSolution1[routeIdx];

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
                    Route1 originalRoute = route;

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
        std::shuffle(randomRequestIds.begin(), randomRequestIds.end(), gen);

        // Sau đó xử lý từng request theo thứ tự ngẫu nhiên mới
        for (int req_id : randomRequestIds) {
            const Request &req = findRequestById(req_id);
            ll bestCost = std::numeric_limits<ll>::max();
            int bestRoute = -1;
            Route1 bestRouteConfig;

            for (size_t routeIdx = 0; routeIdx < num_vehicles; routeIdx++) {
                Route1 &route = currentSolution1[routeIdx];
                size_t routeSize = route.size();

                for (size_t pickup_pos = 0; pickup_pos <= routeSize; pickup_pos++) {
                    for (size_t delivery_pos = pickup_pos; delivery_pos <= routeSize; delivery_pos++) {
                        // Try regular container operations
                        {
                            Route1 testRoute = route;

                            // Insert pickup and delivery
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

                        // Try container with trailer operations if applicable
                        {
                            Route1 testRoute = route;

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
                currentSolution1[bestRoute] = bestRouteConfig;
                currentSolution1[bestRoute].cost = calculateRouteCost(currentSolution1[bestRoute]);
            }
        }
    }

    ll calculateF1() {
        ll maxCost = 0;
        for (const Route1 &route : currentSolution1) {
            maxCost = std::max(maxCost, route.cost);
        }
        return maxCost;
    }

    ll calculateF2() {
        ll totalCost = 0;
        for (const Route1 &route : currentSolution1) {
            totalCost += route.cost;
        }
        return totalCost;
    }

    ll calculateSolutionCost() {
        return alpha * calculateF1() + calculateF2();
    }

    PDPSolver1(std::array<Request, MAX_REQUESTS> &requests,
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
            currentSolution1[i].depot = vehicleDepots[i];
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
        auto bestSolution = currentSolution1;
        ll bestSolutionCost = currentSolutionCost;
        ll bestTotalCost = calculateF2(); // Thêm biến để lưu tổng chi phí tốt nhất

        for (int iter = 0; iter < max_iterations && currentTemp > 1e-8; iter++) {
            auto current_time = chrono::high_resolution_clock::now();
            double elapsed_time = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count() / 1000.0;
            if (elapsed_time >= 29)
                break;
            int numToRemove = max(2, min(40, static_cast<int>(0.1 + (0.3 * (rand() % 100) / 100.0) * requests.size())));

            vector<int> removedRequests;
            removeRandomRequests(removedRequests, numToRemove);
            insertRequests(removedRequests);

            ll newSolutionCost = calculateSolutionCost();
            ll newTotalCost = calculateF2(); // Tính tổng chi phí của solution mới

            if (newSolutionCost < bestSolutionCost) {
                bestSolution = currentSolution1;
                bestSolutionCost = newSolutionCost;
                bestTotalCost = newTotalCost;
                currentSolutionCost = newSolutionCost;
            } else if (newSolutionCost == bestSolutionCost) {
                // Nếu có cùng chi phí max, so sánh tổng chi phí
                if (newTotalCost < bestTotalCost) {
                    bestSolution = currentSolution1;
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
                        currentSolution1 = bestSolution;
                    }
                }
            }

            // cerr << "Iter: " << iter
            //     << " Cost: " << bestSolutionCost
            //     << " Total: " << bestTotalCost
            //     << " Time: " << fixed << setprecision(2) << elapsed_time << "s"
            //     << " Temp: " << currentTemp << endl;

            currentTemp *= coolingRate;
            if (elapsed_time >= 29)
                break;
        }

        currentSolution1 = bestSolution;
    }

    std::array<Route1, MAX_VEHICLES> getSolution() {
        return currentSolution1;
    }
};
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
    int n;

    class BlossomNode {
      public:
        int value;
        int parent;
        int mate;
        bool mark;
        bool in_blossom;
        int blossom;

        // Add default constructor
        BlossomNode() : value(-1), parent(-1), mate(-1), mark(false),
                        in_blossom(false), blossom(-1) {}

        BlossomNode(int v) : value(v), parent(-1), mate(-1), mark(false),
                             in_blossom(false), blossom(-1) {}
    };

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &p) const {
            return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
        }
    };

    class WeightedGraph {
      private:
        std::unordered_map<int, std::vector<int>> graph;
        std::unordered_map<std::pair<int, int>, int, pair_hash> weights;
        std::unordered_set<int> vertices;

      public:
        void addEdge(int u, int v, int weight) {
            graph[u].push_back(v);
            graph[v].push_back(u);
            weights[{u, v}] = weight;
            weights[{v, u}] = weight;
            vertices.insert(u);
            vertices.insert(v);
        }

        std::vector<int> findAugmentingPath(int root,
                                            std::unordered_map<int, BlossomNode> &nodes) {
            std::queue<int> q;
            q.push(root);
            std::unordered_map<int, std::vector<int>> path;
            path[root] = std::vector<int>();

            while (!q.empty()) {
                int vertex = q.front();
                q.pop();

                for (int neighbor : graph[vertex]) {
                    if (path.find(neighbor) == path.end()) {
                        BlossomNode &current = nodes[neighbor];
                        if (current.mate == -1) {
                            std::vector<int> augPath = path[vertex];
                            augPath.push_back(vertex);
                            augPath.push_back(neighbor);
                            return augPath;
                        } else if (path.find(neighbor) == path.end()) {
                            path[neighbor] = path[vertex];
                            path[neighbor].push_back(vertex);
                            path[neighbor].push_back(neighbor);
                            q.push(current.mate);
                        }
                    }
                }
            }
            return std::vector<int>();
        }

        std::vector<int> findCycle(int start, int end,
                                   std::unordered_map<int, BlossomNode> &nodes) {
            std::unordered_map<int, std::vector<int>> visited;
            std::queue<int> q;
            visited[start] = {start};
            q.push(start);

            while (!q.empty()) {
                int vertex = q.front();
                q.pop();

                for (int neighbor : graph[vertex]) {
                    if (neighbor == end && visited[vertex].size() > 2) {
                        std::vector<int> cycle = visited[vertex];
                        cycle.push_back(end);
                        return cycle;
                    }
                    if (visited.find(neighbor) == visited.end()) {
                        visited[neighbor] = visited[vertex];
                        visited[neighbor].push_back(neighbor);
                        q.push(neighbor);
                    }
                }
            }
            return std::vector<int>();
        }

        std::unordered_set<int> contractBlossom(const std::vector<int> &cycle,
                                                int base, std::unordered_map<int, BlossomNode> &nodes) {
            std::unordered_set<int> blossom(cycle.begin(), cycle.end());
            for (int vertex : cycle) {
                nodes[vertex].in_blossom = true;
                nodes[vertex].blossom = base;
            }
            return blossom;
        }

        void expandBlossom(const std::unordered_set<int> &blossom,
                           std::unordered_map<int, int> &matching,
                           int base, std::unordered_map<int, BlossomNode> &nodes) {
            for (int v : blossom) {
                nodes[v].in_blossom = false;
                nodes[v].blossom = -1;
            }

            if (nodes[base].mate == -1)
                return;

            std::vector<int> blossomList(blossom.begin(), blossom.end());
            auto baseIt = std::find(blossomList.begin(), blossomList.end(), base);
            int baseIdx = std::distance(blossomList.begin(), baseIt);
            int length = blossomList.size();

            for (int i = 0; i < length; i++) {
                int v = blossomList[i];
                int nextV = blossomList[(i + 1) % length];

                if (i % 2 == 0) {
                    matching[v] = nextV;
                    matching[nextV] = v;
                    nodes[v].mate = nextV;
                    nodes[nextV].mate = v;
                }
            }
        }

        std::pair<std::unordered_map<int, int>, int> findMaximumWeightedMatching() {
            std::unordered_map<int, BlossomNode> nodes;
            for (int v : vertices) {
                nodes.emplace(v, BlossomNode(v));
            }

            std::unordered_map<int, int> matching;
            std::priority_queue<std::tuple<int, int, int>> edges;

            for (int u : vertices) {
                for (int v : graph[u]) {
                    if (u < v) {
                        edges.push({weights[{u, v}], u, v});
                    }
                }
            }

            while (!edges.empty()) {
                auto [weight, u, v] = edges.top();
                edges.pop();

                if (matching.find(u) == matching.end() &&
                    matching.find(v) == matching.end()) {
                    matching[u] = v;
                    matching[v] = u;
                    nodes[u].mate = v;
                    nodes[v].mate = u;
                }
            }

            while (true) {
                bool augmented = false;
                for (int v : vertices) {
                    if (matching.find(v) == matching.end()) {
                        std::vector<int> path = findAugmentingPath(v, nodes);
                        if (!path.empty()) {
                            for (size_t i = 0; i < path.size() - 1; i += 2) {
                                int u = path[i], w = path[i + 1];
                                matching[u] = w;
                                matching[w] = u;
                                nodes[u].mate = w;
                                nodes[w].mate = u;
                            }
                            augmented = true;
                            break;
                        }

                        for (int u : graph[v]) {
                            if (nodes[u].in_blossom)
                                continue;
                            std::vector<int> cycle = findCycle(v, u, nodes);
                            if (!cycle.empty()) {
                                int base = cycle[0];
                                contractBlossom(cycle, base, nodes);
                                break;
                            }
                        }
                    }
                }
                if (!augmented)
                    break;
            }

            int totalWeight = 0;
            for (const auto &[u, v] : matching) {
                if (u < v) {
                    totalWeight += weights[{u, v}];
                }
            }

            return {matching, totalWeight};
        }
    };

  public:
    MinWeightMatching(const std::vector<std::vector<ll>> &adjacency_matrix)
        : graph(adjacency_matrix), n(adjacency_matrix.size()) {}

    // // Returns {matches, remaining_node_id}, where remaining_node_id is -1 if none
    // std::pair<std::vector<std::pair<int, int>>, int> findMinWeightMatching() {
    //     std::vector<std::pair<int, int>> matches;
    //     // Priority queue to store edges sorted by weight
    //     std::priority_queue<Edge> edges;

    //     // Add all edges to priority queue
    //     for (int i = 0; i < n; i++) {
    //         for (int j = i + 1; j < n; j++) {
    //             edges.push(Edge(i, j, graph[i][j]));
    //         }
    //     }

    //     // Keep track of matched nodes
    //     std::set<int> matched;
    //     bool is_odd = (n % 2 != 0);

    //     // Greedily match nodes using smallest available edges
    //     while (!edges.empty() && matched.size() < n - (is_odd ? 1 : 0)) {
    //         Edge e = edges.top();
    //         edges.pop();

    //         // If both nodes are unmatched, match them
    //         if (matched.find(e.u) == matched.end() &&
    //             matched.find(e.v) == matched.end()) {
    //             matches.push_back({e.u, e.v});
    //             matched.insert(e.u);
    //             matched.insert(e.v);
    //         }
    //     }

    //     // Find remaining node if odd number of nodes
    //     int remaining_node = -1;
    //     if (is_odd) {
    //         for (int i = 0; i < n; i++) {
    //             if (matched.find(i) == matched.end()) {
    //                 remaining_node = i;
    //                 break;
    //             }
    //         }
    //     }

    //     return {matches, remaining_node};
    // }

    std::pair<std::vector<std::pair<int, int>>, int> findMinWeightMatching() {
        std::vector<std::pair<int, int>> matches;
        bool is_odd = (n % 2 != 0);
        ll max_weight = 0;

        // Find maximum weight
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                max_weight = std::max(max_weight, graph[i][j]);
            }
        }

        // Create transformed graph with inverted weights
        WeightedGraph transformed_graph;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                int new_weight = max_weight + 1 - graph[i][j];
                transformed_graph.addEdge(i, j, new_weight);
            }
        }

        if (is_odd) {
            for (int i = 0; i < n; i++) {
                transformed_graph.addEdge(i, n, 0);
            }
        }

        // Find maximum weight matching on transformed graph
        auto [max_matching, transformed_weight] = transformed_graph.findMaximumWeightedMatching();

        // Convert matching to vector of pairs and calculate original weight
        int remaining_node = -1;
        int original_weight = 0;
        for (const auto &[u, v] : max_matching) {
            if (u < v) {
                matches.push_back({u, v});
                original_weight += graph[u][v];
                if (v == n && is_odd)
                    remaining_node = v;
            }
        }

        return {matches, remaining_node};
    }

    ContainerSize getContainerSize(int size) {
        return size == 20 ? TWENTY_FT : FORTY_FT;
    }

    void printMatchingStats(const std::vector<std::pair<int, int>> &matches, int remaining_node = -1) {
        ll total_weight = 0;
        ll max_weight = -1;
        n_new = 0;

        for (const auto &match : matches) {
            int u = match.first;
            int v = match.second;
            // std::cout << u << " " << v << "\n";

            ll weight = graph[u][v];

            total_weight += weight;
            max_weight = std::max(max_weight, weight);

            int pickup_point, drop_point;
            Action pickup_action, drop_action;
            if (combinationType[u][v] == 0b0011 || combinationType[u][v] == 0b1100) {
                requests[++n_new] = requests_20ft[u];
                requests[n_new].id = n_new; // Thêm dòng này
                Real_idx[n_new] = requests_20ft[u].id;

                requests[++n_new] = requests_20ft[v];
                requests[n_new].id = n_new; // Thêm dòng này
                Real_idx[n_new] = requests_20ft[v].id;
                continue;
            }
            Matched[requests_20ft[u].id] = requests_20ft[v].id;
            Matched[requests_20ft[v].id] = requests_20ft[u].id;
            if (combinationType[u][v] == 0b0101) {
                pickup_point = requests_20ft[u].pickup_point;
                pickup_action = requests_20ft[u].pickup_action;
                drop_point = requests_20ft[v].drop_point;
                drop_action = requests_20ft[v].drop_action;
            } else if (combinationType[u][v] == 0b1010) {
                pickup_point = requests_20ft[v].pickup_point;
                pickup_action = requests_20ft[v].pickup_action;
                drop_point = requests_20ft[u].drop_point;
                drop_action = requests_20ft[u].drop_action;
            } else if (combinationType[u][v] == 0b0110) {
                pickup_point = requests_20ft[u].pickup_point;
                pickup_action = requests_20ft[u].pickup_action;
                drop_point = requests_20ft[u].drop_point;
                drop_action = requests_20ft[u].drop_action;
            } else {
                pickup_point = requests_20ft[v].pickup_point;
                pickup_action = requests_20ft[v].pickup_action;
                drop_point = requests_20ft[v].drop_point;
                drop_action = requests_20ft[v].drop_action;
            }
            Request request = {
                ++n_new,
                getContainerSize(40),
                pickup_point,
                pickup_action,
                graphWeight[u][v] - distances[pickup_point][drop_point],
                drop_point,
                drop_action,
                0};

            requests[n_new] = request;
            Real_idx[n_new] = requests_20ft[u].id;
        }
        if (remaining_node != -1) {
            requests[++n_new] = requests_20ft[remaining_node];
            requests[n_new].id = n_new;
            Real_idx[n_new] = requests_20ft[remaining_node].id;
        }
        for (auto t : requests_origin)
            if (t.size == 40) {
                requests[++n_new] = t;
                requests[n_new].id = n_new;
                Real_idx[n_new] = t.id;
            }
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

    ContainerSize getContainerSize(int size) {
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
            // cout << id << endl;
            Request request = {
                id,
                getContainerSize(size),
                pickup_point,
                getAction(pickup_action),
                pickup_duration,
                drop_point,
                getAction(drop_action),
                drop_duration};
            requests_origin[id] = request;
            if (size == 20)
                requests_20ft[idx++] = request;
            n_total++;
        }
    }
    void output_route1(const Route1 &route) {
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

            // Check if this is a matched 40ft container
            int matched_id = Matched[Real_idx[req.id]];
            if (req.size == FORTY_FT && matched_id != 0) {
                // Get the original 20ft requests
                Request &req1 = requests_origin[Real_idx[req.id]];
                Request &req2 = requests_origin[matched_id];

                // Handle trailer pickup if needed
                if (req.pickup_action == PICKUP_CONTAINER &&
                    (it == route.list_reqs.begin() || requests[*std::prev(it)].drop_action == DROP_CONTAINER_TRAILER)) {
                    std::cout << trailer_point << " " << actions[PICKUP_TRAILER] << std::endl;
                    prev_point = trailer_point;
                }

                // Find the combination pattern using two 20ft request IDs
                int idx1 = -1, idx2 = -1;
                for (int i = 0; i < idx; i++) {
                    if (requests_20ft[i].id == Real_idx[req.id])
                        idx1 = i;
                    if (requests_20ft[i].id == matched_id)
                        idx2 = i;
                }
                int pattern = combinationType[idx1][idx2];

                // Output based on pattern
                switch (pattern) {
                case 0b0101: // p1->p2->d1->d2
                    std::cout << req1.pickup_point << " " << actions[req1.pickup_action] << " " << req1.id << std::endl;
                    std::cout << req2.pickup_point << " " << actions[req2.pickup_action] << " " << req2.id << std::endl;
                    std::cout << req1.drop_point << " " << actions[req1.drop_action] << " " << req1.id << std::endl;
                    std::cout << req2.drop_point << " " << actions[req2.drop_action] << " " << req2.id << std::endl;
                    break;

                case 0b0110: // p1->p2->d2->d1
                    std::cout << req1.pickup_point << " " << actions[req1.pickup_action] << " " << req1.id << std::endl;
                    std::cout << req2.pickup_point << " " << actions[req2.pickup_action] << " " << req2.id << std::endl;
                    std::cout << req2.drop_point << " " << actions[req2.drop_action] << " " << req2.id << std::endl;
                    std::cout << req1.drop_point << " " << actions[req1.drop_action] << " " << req1.id << std::endl;
                    break;

                case 0b1001: // p2->p1->d1->d2
                    std::cout << req2.pickup_point << " " << actions[req2.pickup_action] << " " << req2.id << std::endl;
                    std::cout << req1.pickup_point << " " << actions[req1.pickup_action] << " " << req1.id << std::endl;
                    std::cout << req1.drop_point << " " << actions[req1.drop_action] << " " << req1.id << std::endl;
                    std::cout << req2.drop_point << " " << actions[req2.drop_action] << " " << req2.id << std::endl;
                    break;

                case 0b1010: // p2->p1->d2->d1
                    std::cout << req2.pickup_point << " " << actions[req2.pickup_action] << " " << req2.id << std::endl;
                    std::cout << req1.pickup_point << " " << actions[req1.pickup_action] << " " << req1.id << std::endl;
                    std::cout << req2.drop_point << " " << actions[req2.drop_action] << " " << req2.id << std::endl;
                    std::cout << req1.drop_point << " " << actions[req1.drop_action] << " " << req1.id << std::endl;
                    break;
                }

                // Handle trailer drop if needed
                if ((req.drop_action == DROP_CONTAINER &&
                     (std::next(it) == route.list_reqs.end() || requests[*std::next(it)].pickup_action == PICKUP_CONTAINER_TRAILER))) {
                    std::cout << trailer_point << " " << actions[DROP_TRAILER] << std::endl;
                    prev_point = trailer_point;
                }
            } else {
                // Handle regular non-matched containers as before
                if (req.pickup_action == PICKUP_CONTAINER &&
                    (it == route.list_reqs.begin() || requests[*std::prev(it)].drop_action == DROP_CONTAINER_TRAILER)) {
                    std::cout << trailer_point << " " << actions[PICKUP_TRAILER] << std::endl;
                    prev_point = trailer_point;
                }

                // Output pickup action
                std::cout << req.pickup_point << " " << actions[req.pickup_action] << " " << Real_idx[req.id] << std::endl;
                prev_point = req.pickup_point;

                // Output drop action
                std::cout << req.drop_point << " " << actions[req.drop_action] << " " << Real_idx[req.id] << std::endl;
                prev_point = req.drop_point;

                // Check if need to handle trailer for drop
                if ((req.drop_action == DROP_CONTAINER &&
                     (std::next(it) == route.list_reqs.end() || requests[*std::next(it)].pickup_action == PICKUP_CONTAINER_TRAILER))) {
                    std::cout << trailer_point << " " << actions[DROP_TRAILER] << std::endl;
                    prev_point = trailer_point;
                }
            }
        }

        std::cout << route.depot << " STOP" << std::endl;
        std::cout << "#" << std::endl;
    }

    void output() {
        PDPSolver solver(requestIdx, num_vehicles, alpha, trailer_point,
                         trailer_pickup_time, max_iterations, verbose);

        solver.solve();
        // freopen("tc/3/out.txt", "w", stdout);

        std::array<Route, MAX_VEHICLES> solution = solver.getSolution();
        std::cout << "ROUTES " << num_vehicles << std::endl;
        for (size_t i = 0; i < num_vehicles; i++) {
            std::cout << "TRUCK " << i + 1 << std::endl;
            output_route(solution[i]);
        }
    }

    void output1() {
        PDPSolver1 solver(requests_origin, num_vehicles, alpha, trailer_point,
                          trailer_pickup_time, max_iterations, verbose);

        solver.solve();
        // freopen("/media/nhdandz/Data/Hackathon/TIKI/tc/5/out.txt", "w", stdout);

        std::array<Route1, MAX_VEHICLES> solution = solver.getSolution();
        std::cout << "ROUTES " << num_vehicles << std::endl;
        for (size_t i = 0; i < num_vehicles; i++) {
            std::cout << "TRUCK " << i + 1 << std::endl;
            output_route1(solution[i]);
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
                                    dist_p2_d2,
                                0b0011});

        // 2. p2->d2->p1->d1 (pattern: 1100)
        combinations.push_back({req1_self_cost + req2_self_cost +
                                    dist_p2_d2 +
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

    void pairMatching() {
        int num_of_nodes = idx;
        int i = 0;
        for (auto t : requests_20ft) {
            i++;
            if (i == idx)
                break;
        }
        graphWeight.clear();
        graphWeight = std::vector<std::vector<ll>>(num_of_nodes, std::vector<ll>(num_of_nodes, 0));
        combinationType.clear();
        combinationType = std::vector<std::vector<int>>(num_of_nodes, std::vector<int>(num_of_nodes, 0));
        // Build the graph
        for (int i = 0; i < num_of_nodes; i++) {
            for (int j = i + 1; j < num_of_nodes; j++) {
                auto cost_combination = calculateTwentyFtCombinationCost(requests_20ft[i], requests_20ft[j]);
                graphWeight[i][j] = cost_combination.first;
                graphWeight[j][i] = cost_combination.first;
                combinationType[i][j] = cost_combination.second;
                combinationType[j][i] = cost_combination.second;
            }
        }
        MinWeightMatching matcher(graphWeight);
        auto [matches, remaining_node] = matcher.findMinWeightMatching();
        matcher.printMatchingStats(matches, remaining_node);
        for (int i = 1; i <= n_new; i++)
            requestIdx.push_back(requests[i].id);
        // Handle the remaining node if needed
        if (remaining_node != -1) {
            // Process requests_20ft[remaining_node] separately
        }
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    // freopen("tc/3/inp.txt", "r", stdin); 

    IO io(100000, 1000000, 0);
    io.input();
    if (n_total > 100) {
        io.pairMatching();
        io.output();
    } else
        io.output1();

    return 0;
}
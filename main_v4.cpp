#include <bits/stdc++.h>
using namespace std;

typedef long int ll;

const int MAX_POINT = 1001;
const int MAX_VEHICLES = 501;
const int MAX_REQUESTS = 1001;
std::chrono::high_resolution_clock::time_point start_time;

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
    ll selfCost = 0;                   // Cost of request itself (pickup + drop duration)
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

    ll calculateRemovalCost(Route &route, const int request_id) {
        auto it = std::find(route.list_reqs.begin(), route.list_reqs.end(), request_id);
        if (it == route.list_reqs.end())
            return route.cost; // Request not found

        const Request &req = requests[request_id];
        ll costDelta = -calculateRequestContextCost(req); // Remove request's own cost

        if (route.list_reqs.size() == 1) {
            // If this is the only request
            costDelta -= calculateDepotToRequestCost(route.depot, req);
            costDelta -= calculateRequestToDepotCost(req, route.depot);
            return 0; // Route will be empty
        }

        if (it == route.list_reqs.begin()) {
            // If removing first request
            const Request &next_req = requests[*std::next(it)];
            costDelta -= calculateDepotToRequestCost(route.depot, req);
            costDelta -= calculateRequestTransitionCost(req, next_req);
            costDelta += calculateDepotToRequestCost(route.depot, next_req);
        } else if (std::next(it) == route.list_reqs.end()) {
            // If removing last request
            const Request &prev_req = requests[*std::prev(it)];
            costDelta -= calculateRequestTransitionCost(prev_req, req);
            costDelta -= calculateRequestToDepotCost(req, route.depot);
            costDelta += calculateRequestToDepotCost(prev_req, route.depot);
        } else {
            // If removing from middle
            const Request &prev_req = requests[*std::prev(it)];
            const Request &next_req = requests[*std::next(it)];
            costDelta -= calculateRequestTransitionCost(prev_req, req);
            costDelta -= calculateRequestTransitionCost(req, next_req);
            costDelta += calculateRequestTransitionCost(prev_req, next_req);
        }

        return route.cost + costDelta;
    }

    ll calculateRemovalCost(Route &route, std::list<int>::iterator it) {
        if (it == route.list_reqs.end())
            return route.cost; // Request not found

        auto request_id = *it;
        const Request &req = requests[request_id];
        ll costDelta = -calculateRequestContextCost(req); // Remove request's own cost

        if (route.list_reqs.size() == 1) {
            // If this is the only request
            costDelta -= calculateDepotToRequestCost(route.depot, req);
            costDelta -= calculateRequestToDepotCost(req, route.depot);
            return 0; // Route will be empty
        }

        if (it == route.list_reqs.begin()) {
            // If removing first request
            const Request &next_req = requests[*std::next(it)];
            costDelta -= calculateDepotToRequestCost(route.depot, req);
            costDelta -= calculateRequestTransitionCost(req, next_req);
            costDelta += calculateDepotToRequestCost(route.depot, next_req);
        } else if (std::next(it) == route.list_reqs.end()) {
            // If removing last request
            const Request &prev_req = requests[*std::prev(it)];
            costDelta -= calculateRequestTransitionCost(prev_req, req);
            costDelta -= calculateRequestToDepotCost(req, route.depot);
            costDelta += calculateRequestToDepotCost(prev_req, route.depot);
        } else {
            // If removing from middle
            const Request &prev_req = requests[*std::prev(it)];
            const Request &next_req = requests[*std::next(it)];
            costDelta -= calculateRequestTransitionCost(prev_req, req);
            costDelta -= calculateRequestTransitionCost(req, next_req);
            costDelta += calculateRequestTransitionCost(prev_req, next_req);
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

    void removeStopsByRequestIt(Route &route, std::list<int>::iterator it) {
        // Calculate new cost before modifying the list
        ll newCost = calculateRemovalCost(route, it);

        // Remove the request
        route.list_reqs.erase(it);

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

        const Request &curr_req = requests[req_id];
        // context.selfCost = calculateRequestContextCost(curr_req);

        // Calculate transition cost
        ll transitionCost = 0;

        if (it == route.list_reqs.begin()) {
            // First request
            transitionCost += calculateDepotToRequestCost(route.depot, curr_req);
            if (std::next(it) != route.list_reqs.end()) {
                const Request &next_req = requests[*std::next(it)];
                transitionCost += calculateRequestTransitionCost(curr_req, next_req);
                transitionCost -= calculateDepotToRequestCost(route.depot, next_req);
            } else {
                transitionCost += calculateRequestToDepotCost(curr_req, route.depot);
            }
        } else {
            const Request &prev_req = requests[*std::prev(it)];

            if (std::next(it) == route.list_reqs.end()) {
                // Last request
                transitionCost += calculateRequestTransitionCost(prev_req, curr_req);
                transitionCost += calculateRequestToDepotCost(curr_req, route.depot);
                transitionCost -= calculateRequestToDepotCost(prev_req, route.depot);
            } else {
                // Middle request
                const Request &next_req = requests[*std::next(it)];
                transitionCost += calculateRequestTransitionCost(prev_req, curr_req);
                transitionCost += calculateRequestTransitionCost(curr_req, next_req);
                transitionCost -= calculateRequestTransitionCost(prev_req, next_req);
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
        while (requestsToRemove.size() < max_attempt) {
            std::discrete_distribution<> routeDist(probabilities.begin(), probabilities.end());
            int routeIdx = routeCosts[routeDist(gen)].second;
            Route &route = currentSolution[routeIdx];

            if (route.list_reqs.empty())
                continue;

            // Find highest cost requests in this route
            std::vector<std::pair<ll, int>> routeRequestCosts;
            for (auto it = route.list_reqs.begin(); it != route.list_reqs.end(); ++it) {
                int req_id = *it;
                if (!isRequestRemoved[req_id]) {
                    updateRequestContext(req_id, route, it);
                    routeRequestCosts.push_back({requestContexts[req_id].transitionCost, req_id});
                }
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

            // if (iter % 100 == 0)
            //     std::cout << "Iter: " << iter + 1
            //               << " Cost: " << bestSolutionCost
            //               << " Total: " << bestTotalCost
            //               << " Time: " << fixed << setprecision(2) << elapsed_time << "s"
            //               << " Temp: " << currentTemp << endl;

            currentTemp *= coolingRate;
            if (elapsed_time >= 29.50)
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
            if (elapsed_time >= 29.50)
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
            if (elapsed_time >= 29.50)
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

#define EVEN 2
#define ODD 1
#define UNLABELED 0

#define EPSILON 0.000001
#define INFINITO 1000000000.0
#define GREATER(A, B) ((A) - (B) > EPSILON)
#define LESS(A, B) ((B) - (A) > EPSILON)
#define EQUAL(A, B) (fabs((A) - (B)) < EPSILON)
#define GREATER_EQUAL(A, B) (GREATER((A), (B)) || EQUAL((A), (B)))
#define LESS_EQUAL(A, B) (LESS((A), (B)) || EQUAL((A), (B)))
#define MIN(A, B) (LESS((A), (B)) ? (A) : (B))
#define MAX(A, B) (LESS((A), (B)) ? (B) : (A))

class BinaryHeap {
  private:
    vector<double> key;
    vector<int> pos;
    vector<int> satellite;
    int size;

  public:
    BinaryHeap() : satellite(1), size(0) {}

    void Clear() {
        key.clear();
        pos.clear();
        satellite.clear();
        satellite.resize(1);
        size = 0;
    }

    void Insert(double k, int s) {
        if (s >= (int)pos.size()) {
            pos.resize(s + 1, -1);
            key.resize(s + 1);
            satellite.resize(s + 2);
        } else if (pos[s] != -1) {
            throw "Error: satellite already in heap";
        }

        int i;
        for (i = ++size; i / 2 > 0 && GREATER(key[satellite[i / 2]], k); i /= 2) {
            satellite[i] = satellite[i / 2];
            pos[satellite[i]] = i;
        }
        satellite[i] = s;
        pos[s] = i;
        key[s] = k;
    }

    int Size() { return size; }

    int DeleteMin() {
        if (size == 0)
            throw "Error: empty heap";

        int min = satellite[1];
        int slast = satellite[size--];

        int child;
        int i;
        for (i = 1, child = 2; child <= size; i = child, child *= 2) {
            if (child < size && GREATER(key[satellite[child]], key[satellite[child + 1]]))
                child++;

            if (GREATER(key[slast], key[satellite[child]])) {
                satellite[i] = satellite[child];
                pos[satellite[child]] = i;
            } else
                break;
        }
        satellite[i] = slast;
        pos[slast] = i;
        pos[min] = -1;
        return min;
    }

    void ChangeKey(double k, int s) {
        Remove(s);
        Insert(k, s);
    }

    void Remove(int s) {
        int i;
        for (i = pos[s]; i / 2 > 0; i /= 2) {
            satellite[i] = satellite[i / 2];
            pos[satellite[i]] = i;
        }
        satellite[1] = s;
        pos[s] = 1;
        DeleteMin();
    }
};

class Matching {
  private:
    const vector<vector<double>> &G;
    list<int> free;
    vector<int> outer;
    vector<list<int>> deep;
    vector<list<int>> shallow;
    vector<int> tip;
    vector<bool> active;
    vector<int> type;
    vector<int> forest;
    vector<int> root;
    vector<bool> blocked;
    vector<double> dual;
    vector<vector<double>> slack;
    vector<int> mate;
    int n;
    bool perfect;
    list<int> forestList;
    vector<int> visited;

    bool IsEdgeBlocked(int u, int v) {
        return GREATER(slack[u][v], 0);
    }

    bool IsAdjacent(int u, int v) {
        return !IsEdgeBlocked(u, v);
    }

    void DestroyBlossom(int t) {
        if ((t < n) || (blocked[t] && GREATER(dual[t], 0)))
            return;

        for (list<int>::iterator it = shallow[t].begin(); it != shallow[t].end(); it++) {
            int s = *it;
            outer[s] = s;
            for (list<int>::iterator jt = deep[s].begin(); jt != deep[s].end(); jt++)
                outer[*jt] = s;

            DestroyBlossom(s);
        }

        active[t] = false;
        blocked[t] = false;
        AddFreeBlossomIndex(t);
        mate[t] = -1;
    }

    void Expand(int u, bool expandBlocked = false) {
        int v = outer[mate[u]];

        double minSlack = INFINITO;
        int p = -1, q = -1;

        for (list<int>::iterator it = deep[u].begin(); it != deep[u].end(); it++) {
            int di = *it;
            for (list<int>::iterator jt = deep[v].begin(); jt != deep[v].end(); jt++) {
                int dj = *jt;
                if (IsAdjacent(di, dj) && LESS(slack[di][dj], minSlack)) {
                    minSlack = slack[di][dj];
                    p = di;
                    q = dj;
                }
            }
        }

        mate[u] = q;
        mate[v] = p;

        if (u < n || (blocked[u] && !expandBlocked))
            return;

        bool found = false;
        for (list<int>::iterator it = shallow[u].begin(); it != shallow[u].end() && !found;) {
            int si = *it;
            for (list<int>::iterator jt = deep[si].begin(); jt != deep[si].end() && !found; jt++) {
                if (*jt == p)
                    found = true;
            }
            it++;
            if (!found) {
                shallow[u].push_back(si);
                shallow[u].pop_front();
            }
        }

        list<int>::iterator it = shallow[u].begin();
        mate[*it] = mate[u];
        it++;

        while (it != shallow[u].end()) {
            list<int>::iterator itnext = it;
            itnext++;
            mate[*it] = *itnext;
            mate[*itnext] = *it;
            itnext++;
            it = itnext;
        }

        for (list<int>::iterator it = shallow[u].begin(); it != shallow[u].end(); it++) {
            int s = *it;
            outer[s] = s;
            for (list<int>::iterator jt = deep[s].begin(); jt != deep[s].end(); jt++)
                outer[*jt] = s;
        }
        active[u] = false;
        AddFreeBlossomIndex(u);

        for (list<int>::iterator it = shallow[u].begin(); it != shallow[u].end(); it++)
            Expand(*it, expandBlocked);
    }

    void Augment(int u, int v) {
        int p = outer[u];
        int q = outer[v];
        int outv = q;
        int fp = forest[p];
        mate[p] = q;
        mate[q] = p;
        Expand(p);
        Expand(q);

        while (fp != -1) {
            q = outer[forest[p]];
            p = outer[forest[q]];
            fp = forest[p];

            mate[p] = q;
            mate[q] = p;
            Expand(p);
            Expand(q);
        }

        p = outv;
        fp = forest[p];
        while (fp != -1) {
            q = outer[forest[p]];
            p = outer[forest[q]];
            fp = forest[p];

            mate[p] = q;
            mate[q] = p;
            Expand(p);
            Expand(q);
        }
    }

    int Blossom(int u, int v) {
        int t = GetFreeBlossomIndex();
        vector<bool> isInPath(2 * n, false);

        int u_ = u;
        while (u_ != -1) {
            isInPath[outer[u_]] = true;
            u_ = forest[outer[u_]];
        }

        int v_ = outer[v];
        while (!isInPath[v_])
            v_ = outer[forest[v_]];
        tip[t] = v_;

        list<int> circuit;
        u_ = outer[u];
        circuit.push_front(u_);
        while (u_ != tip[t]) {
            u_ = outer[forest[u_]];
            circuit.push_front(u_);
        }

        shallow[t].clear();
        deep[t].clear();
        for (list<int>::iterator it = circuit.begin(); it != circuit.end(); it++) {
            shallow[t].push_back(*it);
        }

        v_ = outer[v];
        while (v_ != tip[t]) {
            shallow[t].push_back(v_);
            v_ = outer[forest[v_]];
        }

        for (list<int>::iterator it = shallow[t].begin(); it != shallow[t].end(); it++) {
            u_ = *it;
            outer[u_] = t;
            for (list<int>::iterator jt = deep[u_].begin(); jt != deep[u_].end(); jt++) {
                deep[t].push_back(*jt);
                outer[*jt] = t;
            }
        }

        forest[t] = forest[tip[t]];
        type[t] = EVEN;
        root[t] = root[tip[t]];
        active[t] = true;
        outer[t] = t;
        mate[t] = mate[tip[t]];

        return t;
    }

    void Reset() {
        for (int i = 0; i < 2 * n; i++) {
            forest[i] = -1;
            root[i] = i;

            if (i >= n && active[i] && outer[i] == i)
                DestroyBlossom(i);
        }

        visited.assign(2 * n, 0);
        forestList.clear();
        for (int i = 0; i < n; i++) {
            if (mate[outer[i]] == -1) {
                type[outer[i]] = 2;
                if (!visited[outer[i]])
                    forestList.push_back(i);
                visited[outer[i]] = true;
            } else
                type[outer[i]] = 0;
        }
    }

    int GetFreeBlossomIndex() {
        int i = free.back();
        free.pop_back();
        return i;
    }

    void AddFreeBlossomIndex(int i) {
        free.push_back(i);
    }

    void ClearBlossomIndices() {
        free.clear();
        for (int i = n; i < 2 * n; i++)
            AddFreeBlossomIndex(i);
    }

    void Grow() {
        Reset();

        while (!forestList.empty()) {
            int w = outer[forestList.front()];
            forestList.pop_front();

            for (list<int>::iterator it = deep[w].begin(); it != deep[w].end(); it++) {
                int u = *it;

                for (int v = 0; v < n; v++) {
                    if (u == v)
                        continue;

                    if (IsEdgeBlocked(u, v))
                        continue;

                    if (type[outer[v]] == ODD)
                        continue;

                    if (type[outer[v]] != EVEN) {
                        int vm = mate[outer[v]];

                        forest[outer[v]] = u;
                        type[outer[v]] = ODD;
                        root[outer[v]] = root[outer[u]];
                        forest[outer[vm]] = v;
                        type[outer[vm]] = EVEN;
                        root[outer[vm]] = root[outer[u]];

                        if (!visited[outer[vm]]) {
                            forestList.push_back(vm);
                            visited[outer[vm]] = true;
                        }
                    } else if (root[outer[v]] != root[outer[u]]) {
                        Augment(u, v);
                        Reset();
                        goto next_iteration;
                    } else if (outer[u] != outer[v]) {
                        int b = Blossom(u, v);
                        forestList.push_front(b);
                        visited[b] = true;
                        goto next_iteration;
                    }
                }
            }
        next_iteration:;
        }

        perfect = true;
        for (int i = 0; i < n; i++)
            if (mate[outer[i]] == -1)
                perfect = false;
    }

    void Heuristic() {
        vector<int> degree(n, 0);
        BinaryHeap B;

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u != v && !IsEdgeBlocked(u, v)) {
                    degree[u]++;
                }
            }
        }

        for (int i = 0; i < n; i++)
            B.Insert(degree[i], i);

        while (B.Size() > 0) {
            int u = B.DeleteMin();
            if (mate[outer[u]] == -1) {
                int min = -1;
                for (int v = 0; v < n; v++) {
                    if (u == v)
                        continue;

                    if (IsEdgeBlocked(u, v) ||
                        (outer[u] == outer[v]) ||
                        (mate[outer[v]] != -1))
                        continue;

                    if (min == -1 || degree[v] < degree[min])
                        min = v;
                }
                if (min != -1) {
                    mate[outer[u]] = min;
                    mate[outer[min]] = u;
                }
            }
        }
    }

    void UpdateDualCosts() {
        double e1 = 0, e2 = 0, e3 = 0;
        bool inite1 = false, inite2 = false, inite3 = false;

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u == v)
                    continue;

                if ((type[outer[u]] == EVEN && type[outer[v]] == UNLABELED) ||
                    (type[outer[v]] == EVEN && type[outer[u]] == UNLABELED)) {
                    if (!inite1 || GREATER(e1, slack[u][v])) {
                        e1 = slack[u][v];
                        inite1 = true;
                    }
                } else if ((outer[u] != outer[v]) && type[outer[u]] == EVEN && type[outer[v]] == EVEN) {
                    if (!inite2 || GREATER(e2, slack[u][v])) {
                        e2 = slack[u][v];
                        inite2 = true;
                    }
                }
            }
        }

        for (int i = n; i < 2 * n; i++) {
            if (active[i] && i == outer[i] && type[outer[i]] == ODD &&
                (!inite3 || GREATER(e3, dual[i]))) {
                e3 = dual[i];
                inite3 = true;
            }
        }

        double e = 0;
        if (inite1)
            e = e1;
        else if (inite2)
            e = e2;
        else if (inite3)
            e = e3;

        if (GREATER(e, e2 / 2.0) && inite2)
            e = e2 / 2.0;
        if (GREATER(e, e3) && inite3)
            e = e3;

        for (int i = 0; i < 2 * n; i++) {
            if (i != outer[i])
                continue;

            if (active[i] && type[outer[i]] == EVEN)
                dual[i] += e;
            else if (active[i] && type[outer[i]] == ODD)
                dual[i] -= e;
        }

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u == v)
                    continue;

                if (outer[u] != outer[v]) {
                    if (type[outer[u]] == EVEN && type[outer[v]] == EVEN)
                        slack[u][v] -= 2.0 * e;
                    else if (type[outer[u]] == ODD && type[outer[v]] == ODD)
                        slack[u][v] += 2.0 * e;
                    else if ((type[outer[v]] == UNLABELED && type[outer[u]] == EVEN) ||
                             (type[outer[u]] == UNLABELED && type[outer[v]] == EVEN))
                        slack[u][v] -= e;
                    else if ((type[outer[v]] == UNLABELED && type[outer[u]] == ODD) ||
                             (type[outer[u]] == UNLABELED && type[outer[v]] == ODD))
                        slack[u][v] += e;
                }
            }
        }

        for (int i = n; i < 2 * n; i++) {
            if (GREATER(dual[i], 0)) {
                blocked[i] = true;
            } else if (active[i] && blocked[i]) {
                if (mate[i] == -1) {
                    DestroyBlossom(i);
                } else {
                    blocked[i] = false;
                    Expand(i);
                }
            }
        }
    }

    void PositiveCosts() {
        double minEdge = 0;
        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u == v)
                    continue;
                if (GREATER(minEdge - slack[u][v], 0))
                    minEdge = slack[u][v];
            }
        }

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u == v)
                    continue;
                slack[u][v] -= minEdge;
            }
        }
    }

    void Clear() {
        ClearBlossomIndices();

        for (int i = 0; i < 2 * n; i++) {
            outer[i] = i;
            deep[i].clear();
            if (i < n)
                deep[i].push_back(i);
            shallow[i].clear();
            if (i < n)
                active[i] = true;
            else
                active[i] = false;

            type[i] = 0;
            forest[i] = -1;
            root[i] = i;
            blocked[i] = false;
            dual[i] = 0;
            mate[i] = -1;
            tip[i] = i;
        }
        slack.assign(n, vector<double>(n, 0));
    }

    vector<pair<int, int>> RetrieveMatching() {
        vector<pair<int, int>> matching;

        for (int i = 0; i < 2 * n; i++)
            if (active[i] && mate[i] != -1 && outer[i] == i)
                Expand(i, true);

        for (int u = 0; u < n; u++) {
            for (int v = u + 1; v < n; v++) {
                if (mate[u] == v)
                    matching.push_back({u, v});
            }
        }
        return matching;
    }

  public:
    Matching(const vector<vector<double>> &graph) : G(graph),
                                                    outer(2 * graph.size()),
                                                    deep(2 * graph.size()),
                                                    shallow(2 * graph.size()),
                                                    tip(2 * graph.size()),
                                                    active(2 * graph.size()),
                                                    type(2 * graph.size()),
                                                    forest(2 * graph.size()),
                                                    root(2 * graph.size()),
                                                    blocked(2 * graph.size()),
                                                    dual(2 * graph.size()),
                                                    slack(graph.size(), vector<double>(graph.size())),
                                                    mate(2 * graph.size()),
                                                    n(graph.size()),
                                                    visited(2 * graph.size()) {
    }

    pair<vector<pair<int, int>>, double> SolveMinimumCostPerfectMatching() {
        SolveMaximumMatching();
        if (!perfect)
            throw "Error: The graph does not have a perfect matching";

        Clear();

        // Initialize slacks
        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u != v)
                    slack[u][v] = G[u][v];
            }
        }

        PositiveCosts();

        perfect = false;
        while (!perfect) {
            Heuristic();
            Grow();
            UpdateDualCosts();
            Reset();
        }

        vector<pair<int, int>> matching = RetrieveMatching();

        double obj = 0;
        for (const auto &edge : matching) {
            obj += G[edge.first][edge.second];
        }

        double dualObj = 0;
        for (int i = 0; i < 2 * n; i++) {
            if (i < n)
                dualObj += dual[i];
            else if (blocked[i])
                dualObj += dual[i];
        }

        return {matching, obj};
    }

    vector<pair<int, int>> SolveMaximumMatching() {
        Clear();
        Grow();
        return RetrieveMatching();
    }
};

class MinWeightMatching {
  private:
    std::vector<std::vector<ll>> graph;
    int n; // Original number of nodes

  public:
    MinWeightMatching(const std::vector<std::vector<ll>> &adjacency_matrix)
        : graph(adjacency_matrix), n(adjacency_matrix.size()) {}

    std::pair<std::vector<std::pair<int, int>>, int> findMinWeightMatching() {
        std::vector<std::pair<int, int>> matches;
        int remaining_node = -1;
        int is_odd = (n & 1);

        vector<vector<double>> g(n + is_odd, vector<double>(n + is_odd));
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                g[i][j] = g[j][i] = graph[i][j];
            }
        }

        if (is_odd) {
            for (int i = 0; i < n; i++) {
                g[i][n] = g[n][i] = 0;
            }
        }

        Matching M(g);
        auto [matching, cost] = M.SolveMinimumCostPerfectMatching();

        for (const auto &edge : matching) {
            auto u = edge.first, v = edge.second;
            if (u > v)
                swap(u, v);
            if (is_odd && v == n)
                remaining_node = u;
            else
                matches.push_back({u, v});
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
        // std::cout << "Matches:\n";
        for (const auto &match : matches) {
            int u = match.first;
            int v = match.second;
            ll weight = graph[u][v];

            total_weight += weight;
            max_weight = std::max(max_weight, weight);
            // cout << requests_20ft[u].id << " " << requests_20ft[v].id << endl;
            //  std::cout << "(" << u << "," << v << ") weight: " << weight << " " << requests_20ft[u].id << " " << requests_20ft[v].id << "\n";
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

            // std::cout << "Remaining unmatched node: " << remaining_node << "\n";
        }
        for (auto t : requests_origin)
            if (t.size == 40) {
                requests[++n_new] = t;
                requests[n_new].id = n_new;
                Real_idx[n_new] = t.id;
            }
        // for(int i = 1; i <= n_new; i++)
        //    cout << requests[i].id << " " << requests[i].size << " " << Real_idx[requests[i].id] << " " << Matched[Real_idx[requests[i].id]] << endl;
        // for (int i = 1; i <= n_new; i++)
        //     cout << i << " " << requests[i].id << " " << Real_idx[requests[i].id] << " " << Matched[Real_idx[requests[i].id]] << " " << requests[i].pickup_point << " " << actions[requests[i].pickup_action] << " " << requests[i].drop_point << " " << actions[requests[i].drop_action] << endl;
        // cout << endl;
        // std::cout << "Total weight: " << total_weight << "\n";
        // std::cout << "Maximum edge weight: " << max_weight << "\n";
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
        // freopen("tc/6/out.txt", "w", stdout);

        std::array<Route, MAX_VEHICLES> solution = solver.getSolution();
        std::cout << "ROUTES " << num_vehicles << std::endl;
        for (size_t i = 0; i < num_vehicles; i++) {
            std::cout << "TRUCK " << i + 1 << std::endl;
            output_route(solution[i]);
        }
        // for (size_t i = 0; i < num_vehicles; i++)
        // {
        //     std::cout << "TRUCK " << i + 1 << std::endl;
        //     output_route1(solution[i]);
        // }
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
        // for (size_t i = 0; i < num_vehicles; i++)
        // {
        //     std::cout << "TRUCK " << i + 1 << std::endl;
        //     output_route1(solution[i]);
        // }
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

    void pairMatching() {
        int num_of_nodes = idx;
        int i = 0;
        for (auto t : requests_20ft) {
            i++;
            // cout << t.id << " " << t.pickup_point << " " << t.drop_point << endl;
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
    start_time = chrono::high_resolution_clock::now();
    // freopen("tc/6/inp.txt", "r", stdin);

    IO io(100000, 100000, 0);
    io.input();
    if (n_total > 100) {
        io.pairMatching();
        io.output();
    } else
        io.output1();

    return 0;
}
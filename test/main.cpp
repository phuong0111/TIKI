#include <bits/stdc++.h>

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
    int max_attempt = 50;
    bool verbose;

    const double temperature = 100.0;
    const double coolingRate = 0.9995;

  public:
    void insertStopAfter(Route &route, const StopNode &stop, std::list<StopNode>::iterator &after) {
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
            after++;
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
        after++;
    }

    void removeStopAt(Route &route, std::list<StopNode>::iterator &at) {
        if (at == route.stops.end()) {
            return;
        }

        // Update cost: remove duration of the stop
        route.cost -= at->duration;

        // If it's the only stop
        if (route.stops.size() == 1) {
            route.cost -= getDistance(route.depot, at->point) + getDistance(at->point, route.depot);
            route.stops.erase(at);
            at = route.stops.end();
            return;
        }

        // If it's the first stop
        if (at == route.stops.begin()) {
            auto next = std::next(at);
            route.cost -= getDistance(route.depot, at->point) + getDistance(at->point, next->point);
            route.cost += getDistance(route.depot, next->point);
            route.stops.erase(at);
            at = route.stops.end();
            return;
        }

        // If it's the last stop
        if (std::next(at) == route.stops.end()) {
            auto prev = std::prev(at);
            route.cost -= getDistance(prev->point, at->point) + getDistance(at->point, route.depot);
            route.cost += getDistance(prev->point, route.depot);
            route.stops.erase(at);
            at--;
            return;
        }

        // If it's in the middle
        auto prev = std::prev(at);
        auto next = std::next(at);
        route.cost -= getDistance(prev->point, at->point) + getDistance(at->point, next->point);
        route.cost += getDistance(prev->point, next->point);
        route.stops.erase(at);
        at--;
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

    bool insertRequestToRoute(Route &route, const Request &req,
                              std::list<StopNode>::iterator pickup_it,
                              std::list<StopNode>::iterator drop_it) {
        // Create stops for pickup, drop and trailer operations
        StopNode pickup_stop = {req.id, req.size, req.pickup_point, req.pickup_action, req.pickup_duration};
        StopNode drop_stop = {req.id, req.size, req.drop_point, req.drop_action, req.drop_duration};
        StopNode pickup_trailer_stop = {req.id, NONE, trailer_point, PICKUP_TRAILER, trailer_pickup_time};
        StopNode drop_trailer_stop = {req.id, NONE, trailer_point, DROP_TRAILER, trailer_pickup_time};

        // Insert pickup and drop stops at specified positions
        auto curr_pickup_it = pickup_it;
        insertStopAfter(route, pickup_stop, curr_pickup_it);
        auto curr_drop_it = ((drop_it == route.stops.end()) ? route.stops.begin() : std::next(drop_it));
        insertStopAfter(route, drop_stop, curr_drop_it);

        // Implement trailer operation
        if (curr_pickup_it->action == PICKUP_CONTAINER) {
            // prev action: PICKUP_CONTAINER, DROP_CONTAINER, PICKUP_TRAILER, PICKUP_CONTAINER_TRAILER: OK
            // prev action: DROP_TRAILER +) this was final stop before new req added
        }

        return true;
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

    PDPSolver() {}

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
            requests[id] = request;
        }
    }

    void output_route(const Route &route) {
        for (const auto &stop : route.stops) {
            std::cout << stop.point << " " << actions[stop.action];
            if (stop.action == PICKUP_CONTAINER || stop.action == PICKUP_CONTAINER_TRAILER ||
                stop.action == DROP_CONTAINER || stop.action == DROP_CONTAINER_TRAILER) {
                std::cout << " " << stop.request_id << " " << stop.size;
            }
            std::cout << std::endl;
        }
        std::cout << route.depot << " STOP" << std::endl;
        std::cout << "#" << std::endl;
    }

    void check_route(const Route &route) {
        PDPSolver solver;
        std::cout << "Valid: " << solver.isRouteValid(route) << "\n";
        std::cout << "Cost update: " << (route.cost == solver.calculateRouteCost(route)) << "\n";
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    freopen("inp.txt", "r", stdin);

    IO io;
    io.input();

    Route route;
    PDPSolver solver;
    solver.insertRequestToRoute(route, requests[1], route.stops.end(), route.stops.end());
    solver.insertRequestToRoute(route, requests[2], std::next(std::next(route.stops.begin())), std::next(std::next(route.stops.begin())));
    io.output_route(route);
    io.check_route(route);

    return 0;
}
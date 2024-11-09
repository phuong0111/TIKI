#include <bits/stdc++.h>
using namespace std;

typedef long int ll;

class HungarianMatching {
  private:
    std::vector<std::vector<int>> cost_matrix;
    std::vector<int> lx, ly; // Labels for X and Y vertices
    std::vector<int> xy, yx; // Matching pairs (xy[v1] = v2, yx[v2] = v1)
    std::vector<bool> S, T;  // Sets for alternating tree
    std::vector<int> slack;  // Slack variables
    std::vector<int> slackx; // Vertices giving slack
    int n;                   // Matrix size
    const int INF = std::numeric_limits<int>::max();

    void init_labels() {
        lx.assign(n, 0);
        ly.assign(n, 0);
        for (int x = 0; x < n; x++) {
            for (int y = 0; y < n; y++) {
                lx[x] = std::max(lx[x], cost_matrix[x][y]);
            }
        }
    }

    void update_labels() {
        int delta = INF;
        for (int y = 0; y < n; y++) {
            if (!T[y])
                delta = std::min(delta, slack[y]);
        }

        for (int x = 0; x < n; x++) {
            if (S[x])
                lx[x] -= delta;
        }
        for (int y = 0; y < n; y++) {
            if (T[y])
                ly[y] += delta;
            else
                slack[y] -= delta;
        }
    }

    void add_to_tree(int x, int prevx) {
        S[x] = true;
        for (int y = 0; y < n; y++) {
            if (lx[x] + ly[y] - cost_matrix[x][y] < slack[y]) {
                slack[y] = lx[x] + ly[y] - cost_matrix[x][y];
                slackx[y] = x;
            }
        }
    }

    bool augment() {
        if (n == 0)
            return true;

        int x, y, root;
        std::vector<int> q(n), prev(n, -1);

        S.assign(n, false);
        T.assign(n, false);
        for (int x = 0; x < n; x++) {
            if (xy[x] == -1) {
                q[0] = x;
                root = x;
                prev[x] = -2;
                S[x] = true;
                break;
            }
        }

        slack.assign(n, INF);
        slackx.assign(n, 0);

        for (int y = 0; y < n; y++) {
            slack[y] = lx[root] + ly[y] - cost_matrix[root][y];
            slackx[y] = root;
        }

        int wr = 1;
        int rd = 0;

        while (true) {
            while (rd < wr) {
                x = q[rd++];
                for (y = 0; y < n; y++) {
                    if (cost_matrix[x][y] < lx[x] + ly[y])
                        continue;
                    if (!T[y]) {
                        if (yx[y] == -1)
                            break;
                        T[y] = true;
                        q[wr++] = yx[y];
                        add_to_tree(yx[y], x);
                    }
                }
                if (y < n)
                    break;
            }
            if (y < n)
                break;

            update_labels();

            wr = rd = 0;
            for (y = 0; y < n; y++) {
                if (!T[y] && slack[y] == 0) {
                    if (yx[y] == -1) {
                        x = slackx[y];
                        break;
                    }
                    T[y] = true;
                    if (!S[yx[y]]) {
                        q[wr++] = yx[y];
                        add_to_tree(yx[y], slackx[y]);
                    }
                }
            }
            if (y < n)
                break;
        }

        if (y < n) {
            int ty;
            while (x != -2) {
                ty = y;
                y = xy[x];
                yx[ty] = x;
                xy[x] = ty;
                x = prev[x];
            }
            return true;
        }
        return false;
    }

  public:
    HungarianMatching(const std::vector<std::vector<int>> &costs)
        : cost_matrix(costs), n(costs.size()) {
        lx.resize(n);
        ly.resize(n);
        xy.assign(n, -1);
        yx.assign(n, -1);
    }

    std::pair<std::vector<std::pair<int, int>>, int> solve() {
        init_labels();

        for (int i = 0; i < n; i++) {
            while (!augment()) {
                update_labels();
            }
        }

        // Collect results
        int total_weight = 0;
        std::vector<std::pair<int, int>> matching;
        for (int i = 0; i < n; i++) {
            if (xy[i] != -1) {
                matching.push_back({i, xy[i]});
                total_weight += cost_matrix[i][xy[i]];
            }
        }

        return {matching, total_weight};
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
std::vector<Request> requestTwenty;
std::vector<Request> requestForty;

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
                requestTwenty.push_back(request);
            else
                requestForty.push_back(request);
        }
    }

    void output() {
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    // freopen("tc/6/inp.txt", "r", stdin);

    IO io(100000, 1000000, 0);
    io.input();
    io.output();

    return 0;
}
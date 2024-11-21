#include <bits/stdc++.h>

typedef long int ll;

class MinWeightMatching {
  private:
    std::vector<std::vector<ll>> graph;
    int n;

  public:
    MinWeightMatching(const std::vector<std::vector<ll>> &adjacency_matrix)
        : graph(adjacency_matrix), n(adjacency_matrix.size()) {}

    std::pair<std::vector<std::pair<int, int>>, int> findMinWeightMatching() {
        // int extra_node = -1;
        // if (n & 1) {
        //     extra_node = n;
        //     n += 1;
        // }
        std::vector<std::vector<ll>> cost(n, std::vector<ll>(n));
        ll max_weight = 0;
        for (const auto &row : graph)
            max_weight = std::max(max_weight, *std::max_element(row.begin(), row.end()));
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                cost[i][j] = max_weight - graph[i][j];
                cost[j][i] = cost[i][j];
            }
            cost[i][i] = 0;
        }

        
    }
};

int main() {
    return 0;
}
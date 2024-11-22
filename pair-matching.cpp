#include <bits/stdc++.h>
using namespace std;

typedef long int ll;

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
        }
        else if (pos[s] != -1) {
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
        if (size == 0) throw "Error: empty heap";

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
            }
            else break;
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
    const vector<vector<double>>& G;
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
            }
            else
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
                    if (u == v) continue;

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
                    }
                    else if (root[outer[v]] != root[outer[u]]) {
                        Augment(u, v);
                        Reset();
                        goto next_iteration;
                    }
                    else if (outer[u] != outer[v]) {
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
                    if (u == v) continue;
                    
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
                if (u == v) continue;

                if ((type[outer[u]] == EVEN && type[outer[v]] == UNLABELED) ||
                    (type[outer[v]] == EVEN && type[outer[u]] == UNLABELED)) {
                    if (!inite1 || GREATER(e1, slack[u][v])) {
                        e1 = slack[u][v];
                        inite1 = true;
                    }
                }
                else if ((outer[u] != outer[v]) && type[outer[u]] == EVEN && type[outer[v]] == EVEN) {
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
                if (u == v) continue;
                
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
            }
            else if (active[i] && blocked[i]) {
                if (mate[i] == -1) {
                    DestroyBlossom(i);
                }
                else {
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
                if (u == v) continue;
                if (GREATER(minEdge - slack[u][v], 0))
                    minEdge = slack[u][v];
            }
        }

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (u == v) continue;
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
    Matching(const vector<vector<double>>& graph) : 
        G(graph),
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
                if (u != v) slack[u][v] = G[u][v];
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
        for (const auto& edge : matching) {
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

int main() {
    int n = 10; // number of vertices
    
    // Create complete weighted graph
    vector<vector<double>> graph(n, vector<double>(n));
    srand(1337);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double weight = 1 + rand() % 100;
            graph[i][j] = graph[j][i] = weight;
        }
    }

    // Solve minimum cost perfect matching
    Matching M(graph);
    auto [matching, cost] = M.SolveMinimumCostPerfectMatching();

    cout << "Optimal matching cost: " << cost << endl;
    cout << "Edges in the matching:" << endl;
    for (const auto& edge : matching) {
        cout << edge.first << " " << edge.second << endl;
    }
    return 0;
}
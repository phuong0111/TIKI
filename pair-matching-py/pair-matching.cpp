#include <bits/stdc++.h>

// Forward declaration
class Blossom;
using BlossomPtr = std::shared_ptr<Blossom>;
using Node = int;

// Define NoNode as constant -1
const Node NoNode = -1;

class Blossom {
  public:
    std::vector<Node> nodeChilds;
    std::vector<BlossomPtr> blossomChilds;
    std::vector<std::pair<Node, Node>> edges; // pairs of vertices representing edges
    std::vector<std::pair<Node, Node>> myBestEdges;

    // Get all leaf vertices in the blossom
    std::vector<Node> leaves() {
        std::vector<Node> leafVertices;
        std::stack<BlossomPtr> stack;

        // First add direct node children
        leafVertices.insert(leafVertices.end(), nodeChilds.begin(), nodeChilds.end());

        // Add all blossom children to stack
        for (const auto &blossom : blossomChilds) {
            stack.push(blossom);
        }

        // Process blossoms
        while (!stack.empty()) {
            auto current = stack.top();
            stack.pop();

            // Add direct nodes from this blossom
            leafVertices.insert(leafVertices.end(),
                                current->nodeChilds.begin(),
                                current->nodeChilds.end());

            // Add sub-blossoms to stack
            for (const auto &subBlossom : current->blossomChilds) {
                stack.push(subBlossom);
            }
        }

        return leafVertices;
    }
};

// A type that can be either a Node or a BlossomPtr
using NodeOrBlossom = std::variant<Node, BlossomPtr>;

class MaxWeightMatching {
  private:
    // Graph data
    std::vector<std::vector<int>> graph_matrix;
    std::vector<Node> gnodes;
    int num_nodes;
    bool maxcardinality;
    int maxweight;
    bool allinteger;

    // Core matching state
    std::map<Node, Node> mate;
    std::map<NodeOrBlossom, int> label; // Changed from Node to NodeOrBlossom
    std::map<NodeOrBlossom, std::pair<Node, Node>> labeledge;

    // Blossom management
    std::map<Node, NodeOrBlossom> inblossom; // Changed return type to NodeOrBlossom
    std::map<NodeOrBlossom, Node> blossombase;
    std::map<BlossomPtr, BlossomPtr> blossomparent;
    std::map<BlossomPtr, int> blossomdual;

    // Edge management
    std::map<NodeOrBlossom, std::pair<Node, Node>> bestedge;
    std::map<std::pair<Node, Node>, bool> allowedge;
    std::vector<Node> queue;

    // Dual variables
    std::map<NodeOrBlossom, int> dualvar;

    // Helper function to create NodeOrBlossom from Node
    NodeOrBlossom makeNodeOrBlossom(Node n) {
        return NodeOrBlossom(n);
    }

    // Helper function to create NodeOrBlossom from BlossomPtr
    NodeOrBlossom makeNodeOrBlossom(BlossomPtr b) {
        return NodeOrBlossom(b);
    }

  public:
    MaxWeightMatching(const std::vector<std::vector<int>> &matrix, bool max_cardinality = false)
        : graph_matrix(matrix), maxcardinality(max_cardinality) {

        num_nodes = matrix.size();
        if (num_nodes == 0)
            return;

        gnodes.resize(num_nodes);
        for (int i = 0; i < num_nodes; i++) {
            gnodes[i] = i;
        }

        maxweight = 0;
        allinteger = true;
        for (int i = 0; i < num_nodes; i++) {
            for (int j = i + 1; j < num_nodes; j++) {
                int wt = matrix[i][j];
                if (wt > maxweight) {
                    maxweight = wt;
                }
                allinteger = allinteger && (wt == static_cast<int>(wt));
            }
        }

        // Initialize mappings
        for (Node n : gnodes) {
            inblossom[n] = makeNodeOrBlossom(n);
            blossombase[makeNodeOrBlossom(n)] = n;
            dualvar[makeNodeOrBlossom(n)] = maxweight;
        }
    }

  private:
    // Helper method to get slack of an edge
    int slack(Node i, Node j) const {
        auto varI = dualvar.at(inblossom.at(i));
        auto varJ = dualvar.at(inblossom.at(j));
        return varI + varJ - 2 * graph_matrix[i][j];
    }

    void assignLabel(Node w, int t, Node v) {
        auto b = inblossom[w];
        assert(label.find(b) == label.end());
        label[b] = t;

        if (v != NoNode) {
            labeledge[b] = std::make_pair(v, w);
        } else {
            labeledge[b] = std::make_pair(NoNode, NoNode);
        }

        bestedge[b] = std::make_pair(NoNode, NoNode);

        if (t == 1) {
            // Check if b is a BlossomPtr
            if (auto *blossom = std::get_if<BlossomPtr>(&b)) {
                auto leaves = (*blossom)->leaves();
                queue.insert(queue.end(), leaves.begin(), leaves.end());
            } else {
                queue.push_back(std::get<Node>(b));
            }
        } else if (t == 2) {
            Node base = blossombase[b];
            assert(mate.find(base) != mate.end());
            assignLabel(mate[base], 1, base);
        }
    }

    Node scanBlossom(Node v, Node w) {
        std::vector<NodeOrBlossom> path;
        Node base = NoNode;

        while (v != NoNode) {
            auto b = inblossom[v];

            if (label[b] & 4) {
                base = blossombase[b];
                break;
            }

            assert(label[b] == 1);
            path.push_back(b);

            label[b] = 5;

            if (labeledge[b].first == NoNode) {
                // assert(blossombase[b] != mate.find(blossombase[b]) != mate.end());
                v = NoNode;
            } else {
                assert(labeledge[b].first == mate[blossombase[b]]);
                v = labeledge[b].first;

                b = inblossom[v];
                assert(label[b] == 2);
                v = labeledge[b].first;
            }

            if (w != NoNode) {
                std::swap(v, w);
            }
        }

        for (const auto &b : path) {
            label[b] = 1;
        }

        return base;
    }

    void addBlossom(Node base, Node v, Node w) {
        auto bb = inblossom[base];
        auto bv = inblossom[v];
        auto bw = inblossom[w];

        // Create new blossom
        auto b = std::make_shared<Blossom>();
        blossombase[makeNodeOrBlossom(b)] = base;
        blossomparent[b] = nullptr;

        // Set bb's parent to b only if it's a blossom
        if (auto *blossom = std::get_if<BlossomPtr>(&bb)) {
            blossomparent[*blossom] = b;
        }

        // Initialize b's children with proper path tracking
        std::vector<NodeOrBlossom> path;
        b->edges.push_back(std::make_pair(v, w));

        // Trace back from v to base, collecting path
        while (bv != bb) {
            if (auto *blossom = std::get_if<BlossomPtr>(&bv)) {
                blossomparent[*blossom] = b;
            }
            path.push_back(bv);
            b->edges.push_back(labeledge[bv]);
            v = labeledge[bv].first;
            bv = inblossom[v];
        }
        path.push_back(bb);

        // Reverse paths appropriately
        std::reverse(path.begin(), path.end());
        std::reverse(b->edges.begin(), b->edges.end());

        // Trace from w to base
        while (bw != bb) {
            if (auto *blossom = std::get_if<BlossomPtr>(&bw)) {
                blossomparent[*blossom] = b;
            }
            path.push_back(bw);
            b->edges.push_back(std::make_pair(labeledge[bw].second, labeledge[bw].first));
            w = labeledge[bw].first;
            bw = inblossom[w];
        }

        // Properly organize nodes and blossoms in the new blossom
        for (const auto &p : path) {
            if (auto *node = std::get_if<Node>(&p)) {
                b->nodeChilds.push_back(*node);
            } else if (auto *blossom = std::get_if<BlossomPtr>(&p)) {
                b->blossomChilds.push_back(*blossom);
            }
        }

        // Update blossom properties
        label[makeNodeOrBlossom(b)] = 1;
        labeledge[makeNodeOrBlossom(b)] = labeledge[bb];
        blossomdual[b] = 0;

        // Update all vertex mappings to point to new blossom
        for (Node v : b->leaves()) {
            inblossom[v] = makeNodeOrBlossom(b);
        }

        // Handle remaining edge updates as before
        // ... (rest of addBlossom implementation)
        // Handle best edges
        std::map<NodeOrBlossom, std::pair<Node, Node>> bestedgeto;

        for (const auto &bv : path) {
            std::vector<std::pair<Node, Node>> nblist;

            // Get neighbor list based on whether bv is a blossom or node
            if (auto *blossom = std::get_if<BlossomPtr>(&bv)) {
                if (!(*blossom)->myBestEdges.empty()) {
                    nblist = (*blossom)->myBestEdges;
                    (*blossom)->myBestEdges.clear();
                } else {
                    // Generate all possible edges from leaves
                    auto leaves = (*blossom)->leaves();
                    for (Node v : leaves) {
                        for (int w = 0; w < num_nodes; w++) {
                            if (v != w) {
                                nblist.push_back(std::make_pair(v, w));
                            }
                        }
                    }
                }
            } else {
                Node node = std::get<Node>(bv);
                for (int w = 0; w < num_nodes; w++) {
                    if (node != w) {
                        nblist.push_back(std::make_pair(node, w));
                    }
                }
            }

            // Check all edges in nblist
            for (const auto &k : nblist) {
                Node i = k.first, j = k.second;
                auto bj = inblossom[j];

                // j is in b?
                if (bj == makeNodeOrBlossom(b)) {
                    std::swap(i, j);
                    bj = inblossom[j];
                }

                if (bj != makeNodeOrBlossom(b) &&
                    label[bj] == 1 &&
                    (bestedgeto.find(bj) == bestedgeto.end() ||
                     slack(i, j) < slack(bestedgeto[bj].first, bestedgeto[bj].second))) {
                    bestedgeto[bj] = std::make_pair(i, j);
                }
            }
            bestedge[bv] = std::make_pair(NoNode, NoNode);
        }

        // Store best edges in blossom
        b->myBestEdges.clear();
        for (const auto &[_, edge] : bestedgeto) {
            b->myBestEdges.push_back(edge);
        }

        // Find the best edge for the blossom
        std::pair<Node, Node> mybestedge = std::make_pair(NoNode, NoNode);
        int mybestslack = std::numeric_limits<int>::max();

        for (const auto &k : b->myBestEdges) {
            int kslack = slack(k.first, k.second);
            if (mybestedge.first == NoNode || kslack < mybestslack) {
                mybestedge = k;
                mybestslack = kslack;
            }
        }
        bestedge[makeNodeOrBlossom(b)] = mybestedge;
    }

    void augmentMatching(Node v, Node w) {
        // For each s,j in ((v,w), (w,v)):
        for (auto [s, j] : std::vector<std::pair<Node, Node>>{{v, w}, {w, v}}) {
            while (true) {
                auto bs = inblossom[s];
                // assert(label[bs] == 1);
                // assert(
                //     labeledge[bs].first == NoNode && blossombase[bs] != mate.find(blossombase[bs]) != mate.end() ||
                //     labeledge[bs].first == mate[blossombase[bs]]);

                // If bs is a top-level blossom, stop
                if (auto *blossom = std::get_if<BlossomPtr>(&bs)) {
                    augmentBlossom(*blossom, s);
                }

                // Update mate[s]
                mate[s] = j;

                // Trace one step back
                if (labeledge[bs].first == NoNode) {
                    break;
                }

                Node t = labeledge[bs].first;
                auto bt = inblossom[t];
                assert(label[bt] == 2);

                // Trace one step back
                s = labeledge[bt].first;
                j = t;

                // Augment recursively
                if (auto *blossom = std::get_if<BlossomPtr>(&bt)) {
                    augmentBlossom(*blossom, j);
                }
                mate[j] = s;
            }
        }
    }

    void augmentBlossom(BlossomPtr b, Node v) {
        // Recursively augment this blossom from vertex v
        auto _recurse = [this](BlossomPtr b, Node v, auto &_self) -> void {
            // Bubble up through blossom tree from vertex v
            Node t = v;
            NodeOrBlossom parent = makeNodeOrBlossom(b);

            while (true) {
                // Find position of t in b's child list
                size_t pos = -1;
                bool found = false;
                for (size_t i = 0; i < b->nodeChilds.size(); i++) {
                    if (b->nodeChilds[i] == t) {
                        pos = i;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    for (size_t i = 0; i < b->blossomChilds.size(); i++) {
                        auto child = makeNodeOrBlossom(b->blossomChilds[i]);
                        if (child == inblossom[t]) {
                            pos = i + b->nodeChilds.size();
                            found = true;
                            break;
                        }
                    }
                }

                if (blossomparent[b] != nullptr && !found) {
                    t = NoNode;
                    break;
                }

                if (pos == -1)
                    break;

                if (pos & 1) {
                    // pos is odd; go forward and wrap
                    pos -= b->nodeChilds.size() + b->blossomChilds.size();
                    int jstep = 1;
                    int j = pos;

                    // Move along the blossom until we reach the base
                    while (j != 0) {
                        // Step to next sub-blossom and augment recursively
                        j += jstep;
                        Node w, x;
                        if (jstep == 1) {
                            w = b->edges[j].first;
                            x = b->edges[j].second;
                        } else {
                            x = b->edges[j - 1].first;
                            w = b->edges[j - 1].second;
                        }

                        if (j < b->nodeChilds.size()) {
                            // Node
                            _self(b, w, _self);
                        } else {
                            // Blossom
                            auto child = b->blossomChilds[j - b->nodeChilds.size()];
                            if (inblossom[w] == makeNodeOrBlossom(child)) {
                                _self(child, w, _self);
                            }
                        }

                        // Step to next sub-blossom and augment recursively
                        j += jstep;
                        if (j < b->nodeChilds.size()) {
                            // Node
                            _self(b, x, _self);
                        } else {
                            // Blossom
                            auto child = b->blossomChilds[j - b->nodeChilds.size()];
                            if (inblossom[x] == makeNodeOrBlossom(child)) {
                                _self(child, x, _self);
                            }
                        }

                        // Match the edge connecting the sub-blossoms
                        mate[w] = x;
                        mate[x] = w;
                    }
                } else {
                    // pos is even; go backward
                    int jstep = -1;
                    int j = pos;

                    // Move along the blossom until we reach the base
                    while (j != 0) {
                        // Step to next sub-blossom and augment recursively
                        j += jstep;
                        Node w, x;
                        if (jstep == 1) {
                            w = b->edges[j].first;
                            x = b->edges[j].second;
                        } else {
                            x = b->edges[j - 1].first;
                            w = b->edges[j - 1].second;
                        }

                        if (j < b->nodeChilds.size()) {
                            // Node
                            _self(b, w, _self);
                        } else {
                            // Blossom
                            auto child = b->blossomChilds[j - b->nodeChilds.size()];
                            if (inblossom[w] == makeNodeOrBlossom(child)) {
                                _self(child, w, _self);
                            }
                        }

                        // Step to next sub-blossom and augment recursively
                        j += jstep;
                        if (j < b->nodeChilds.size()) {
                            // Node
                            _self(b, x, _self);
                        } else {
                            // Blossom
                            auto child = b->blossomChilds[j - b->nodeChilds.size()];
                            if (inblossom[x] == makeNodeOrBlossom(child)) {
                                _self(child, x, _self);
                            }
                        }

                        // Match the edge connecting the sub-blossoms
                        mate[w] = x;
                        mate[x] = w;
                    }
                }

                // Move up to parent blossom if we're not at a top-level blossom
                if (blossomparent[b] == nullptr)
                    break;
                t = v;
                auto parent = blossomparent[b];
                if (parent == nullptr)
                    break;
                b = parent;
            }
        };

        _recurse(b, v, _recurse);
    }

    void expandBlossom(BlossomPtr b, bool endstage) {
        // Recursive helper function
        auto _recurse = [this](BlossomPtr b, bool endstage, auto &_self) -> void {
            // Convert nested sub-blossoms into top-level ones
            for (size_t i = 0; i < b->nodeChilds.size() + b->blossomChilds.size(); i++) {
                NodeOrBlossom s;
                if (i < b->nodeChilds.size()) {
                    s = makeNodeOrBlossom(b->nodeChilds[i]);
                } else {
                    s = makeNodeOrBlossom(b->blossomChilds[i - b->nodeChilds.size()]);
                }

                // Remove parent pointer for this sub-blossom
                if (auto *blossom = std::get_if<BlossomPtr>(&s)) {
                    blossomparent[*blossom] = nullptr;
                }

                if (endstage && std::holds_alternative<BlossomPtr>(s)) {
                    // Recursively expand this sub-blossom
                    auto sb = std::get<BlossomPtr>(s);
                    if (blossomdual[sb] == 0) {
                        _self(sb, endstage, _self);
                    } else {
                        // Still in optimization stage: don't expand recursive sub-blossoms
                        for (Node v : sb->leaves()) {
                            inblossom[v] = s;
                        }
                    }
                } else if (std::holds_alternative<Node>(s)) {
                    inblossom[std::get<Node>(s)] = s;
                }
            }

            // If we expand a sub-blossom during a stage, its vertices may be reachable:
            // add them to the queue if appropriate
            if (!endstage && label.find(makeNodeOrBlossom(b)) != label.end() && label[makeNodeOrBlossom(b)] == 2) {
                // B became an S-vertex/blossom; add it to queue
                auto entrychild = inblossom[labeledge[makeNodeOrBlossom(b)].second];

                // Find entrychild in b's children list
                size_t j = -1;
                for (size_t i = 0; i < b->nodeChilds.size(); i++) {
                    if (makeNodeOrBlossom(b->nodeChilds[i]) == entrychild) {
                        j = i;
                        break;
                    }
                }
                if (j == -1) {
                    for (size_t i = 0; i < b->blossomChilds.size(); i++) {
                        if (makeNodeOrBlossom(b->blossomChilds[i]) == entrychild) {
                            j = i + b->nodeChilds.size();
                            break;
                        }
                    }
                }

                // Start at the node/blossom through which the expanding blossom obtained its label
                int jstep;
                if (j & 1) {
                    // Start index is odd; go backward and wrap
                    j -= b->nodeChilds.size() + b->blossomChilds.size();
                    jstep = 1;
                } else {
                    // Start index is even; go forward
                    jstep = -1;
                }

                // Move along the blossom until we get to the base
                Node v, w;
                auto p = labeledge[makeNodeOrBlossom(b)];
                while (j != 0) {
                    // Get the next vertex to assign label to
                    if (jstep == 1) {
                        v = b->edges[j].first;
                        w = b->edges[j].second;
                    } else {
                        v = b->edges[j - 1].second;
                        w = b->edges[j - 1].first;
                    }

                    // Extend the S-path by assigning label T to w
                    label[w] = label[inblossom[w]] = NoNode;
                    label[v] = label[inblossom[v]] = NoNode;
                    assignLabel(w, 2, v);
                    allowedge[{v, w}] = allowedge[{w, v}] = true;
                    j += jstep;

                    // Step to the next S-vertex/blossom and assign label S
                    if (jstep == 1) {
                        v = b->edges[j].first;
                        w = b->edges[j].second;
                    } else {
                        v = b->edges[j - 1].second;
                        w = b->edges[j - 1].first;
                    }
                    allowedge[{v, w}] = allowedge[{w, v}] = true;
                    j += jstep;
                }

                // Assign label to base
                auto bw = (j < b->nodeChilds.size()) ? makeNodeOrBlossom(b->nodeChilds[j]) : makeNodeOrBlossom(b->blossomChilds[j - b->nodeChilds.size()]);
                label[w] = label[bw] = 2;
                labeledge[w] = labeledge[bw] = p;
                bestedge[bw] = std::make_pair(NoNode, NoNode);

                // Continue along the blossom until we get back to entrychild
                j += jstep;
                while (true) {
                    // Examine the next vertex in the blossom
                    auto bv = (j < b->nodeChilds.size()) ? makeNodeOrBlossom(b->nodeChilds[j]) : makeNodeOrBlossom(b->blossomChilds[j - b->nodeChilds.size()]);
                    if (bv == entrychild)
                        break;

                    if (label.find(bv) == label.end() || label[bv] == 1) {
                        j += jstep;
                        continue;
                    }

                    // Get vertices for this sub-blossom
                    std::vector<Node> vertices;
                    if (std::holds_alternative<BlossomPtr>(bv)) {
                        auto subblossom = std::get<BlossomPtr>(bv);
                        vertices = subblossom->leaves();
                    } else {
                        vertices = {std::get<Node>(bv)};
                    }

                    for (Node v : vertices) {
                        if (label.find(inblossom[v]) != label.end()) {
                            assert(label[inblossom[v]] == 2);
                            assert(inblossom[v] == bv);
                            label[v] = NoNode;
                            label[mate[blossombase[bv]]] = NoNode;
                            assignLabel(v, 2, labeledge[v].first);
                        }
                    }
                    j += jstep;
                }
            }

            // Clean up
            label.erase(makeNodeOrBlossom(b));
            labeledge.erase(makeNodeOrBlossom(b));
            bestedge.erase(makeNodeOrBlossom(b));
            blossomparent.erase(b);
            blossombase.erase(makeNodeOrBlossom(b));
            blossomdual.erase(b);
        };

        _recurse(b, endstage, _recurse);
    }

  public:
    std::pair<std::vector<std::pair<Node, Node>>, Node> maxWeightMatching() {
        // Main loop: continue until no further improvement is possible
        while (true) {
            // Clear all labels, edges and queues
            label.clear();
            labeledge.clear();
            bestedge.clear();
            // Clear mybestedges for all blossoms
            for (const auto &[blossom, _] : blossomdual) {
                blossom->myBestEdges.clear();
            }
            allowedge.clear();
            queue.clear();

            // Label single vertices with S and put them in queue
            for (Node v : gnodes) {
                if (mate.find(v) == mate.end() && label.find(inblossom[v]) == label.end()) {
                    assignLabel(v, 1, NoNode);
                }
            }

            // Loop until we succeed in augmenting the matching
            bool augmented = false;
            while (true) {
                // Keep processing until queue is empty
                while (!queue.empty() && !augmented) {
                    Node v = queue.back();
                    queue.pop_back();

                    assert(label[inblossom[v]] == 1);

                    // Look for neighbours w
                    for (Node w = 0; w < num_nodes; w++) {
                        if (v == w)
                            continue;

                        // w is a neighbour of v?
                        auto bv = inblossom[v];
                        auto bw = inblossom[w];
                        if (bv == bw)
                            continue;

                        // Check if (v,w) is an allowable edge
                        if (allowedge.find({v, w}) == allowedge.end()) {
                            int kslack = slack(v, w);
                            if (kslack <= 0) {
                                allowedge[{v, w}] = allowedge[{w, v}] = true;
                            }
                        }

                        if (allowedge.find({v, w}) != allowedge.end()) {
                            if (label.find(bw) == label.end()) {
                                // (b) w is not labeled; label it T and add to queue
                                assignLabel(w, 2, v);
                            } else if (label[bw] == 1) {
                                // (c) w is an S-vertex; look for an augmenting path
                                Node base = scanBlossom(v, w);
                                if (base != NoNode) {
                                    // Found a new blossom; add it to the blossom bookkeeping
                                    addBlossom(base, v, w);
                                } else {
                                    // Found an augmenting path; augment the matching and end this phase
                                    augmentMatching(v, w);
                                    augmented = true;
                                    break;
                                }
                            } else if (label[w] == NoNode) {
                                // w's blossom is labeled T; propagate label to w
                                assert(label[bw] == 2);
                                label[w] = 2;
                                labeledge[w] = std::make_pair(v, w);
                            }
                        } else {
                            // This is where we need to fix the logic
                            int kslack = slack(v, w);

                            // First case: bw is an S-vertex/blossom
                            if (label.find(bw) != label.end() && label[bw] == 1) {
                                if (bestedge.find(bv) == bestedge.end() || (bestedge[bv].first == NoNode && bestedge[bv].second == NoNode) ||
                                    kslack < slack(bestedge[bv].first, bestedge[bv].second)) {
                                    bestedge[bv] = std::make_pair(v, w);
                                }
                            }
                            // Second case: w is unlabeled
                            else if (label.find(w) == label.end() || label[w] == NoNode) {
                                if (bestedge.find(w) == bestedge.end() ||
                                    kslack < slack(bestedge[w].first, bestedge[w].second)) {
                                    bestedge[w] = std::make_pair(v, w);
                                }
                            }
                        }
                    }
                }

                if (augmented)
                    break;

                // There is no augmenting path under these constraints;
                // compute delta and reduce slack in optimization problem
                int deltatype = -1;
                int delta = 0;
                std::pair<Node, Node> deltaedge = {NoNode, NoNode};
                BlossomPtr deltablossom = nullptr;

                // Compute delta1: the minimum value of any vertex dual
                if (deltatype == -1) {
                    deltatype = 1;
                    delta = std::numeric_limits<int>::max();
                    for (const auto &[v, dual] : dualvar) {
                        if (std::holds_alternative<Node>(v)) {
                            delta = std::min(delta, dual);
                        }
                    }
                }

                // Compute delta2: the minimum slack on any edge between an S-vertex and an unlabeled vertex
                for (Node v : gnodes) {
                    if (label.find(inblossom[v]) == label.end() && bestedge.find(v) != bestedge.end()) {
                        int d = slack(bestedge[v].first, bestedge[v].second);
                        if (deltatype == -1 || d < delta) {
                            delta = d;
                            deltatype = 2;
                            deltaedge = bestedge[v];
                        }
                    }
                }

                // Compute delta3: half the minimum slack on any edge between a pair of S-blossoms
                for (const auto &[b, _] : blossomparent) {
                    if (blossomparent[b] == nullptr &&
                        label.find(makeNodeOrBlossom(b)) != label.end() &&
                        label[makeNodeOrBlossom(b)] == 1 &&
                        bestedge.find(makeNodeOrBlossom(b)) != bestedge.end()) {
                        int kslack = slack(bestedge[makeNodeOrBlossom(b)].first, bestedge[makeNodeOrBlossom(b)].second);
                        int d = allinteger ? (kslack / 2) : (kslack / 2.0);
                        if (deltatype == -1 || d < delta) {
                            delta = d;
                            deltatype = 3;
                            deltaedge = bestedge[makeNodeOrBlossom(b)];
                        }
                    }
                }

                // Compute delta4: minimum z variable of any T-blossom
                for (const auto &[b, d] : blossomdual) {
                    if (blossomparent[b] == nullptr &&
                        label.find(makeNodeOrBlossom(b)) != label.end() &&
                        label[makeNodeOrBlossom(b)] == 2) {
                        if (deltatype == -1 || d < delta) {
                            delta = d;
                            deltatype = 4;
                            deltablossom = b;
                        }
                    }
                }

                if (deltatype == -1) {
                    // No further improvement possible; optimum reached
                    break;
                }

                // Update dual variables
                for (Node v : gnodes) {
                    auto bv = inblossom[v];
                    if (label.find(bv) != label.end()) {
                        if (label[bv] == 1) {
                            dualvar[makeNodeOrBlossom(v)] -= delta;
                        } else if (label[bv] == 2) {
                            dualvar[makeNodeOrBlossom(v)] += delta;
                        }
                    }
                }

                for (const auto &[b, _] : blossomparent) {
                    if (blossomparent[b] == nullptr) {
                        if (label.find(makeNodeOrBlossom(b)) != label.end()) {
                            if (label[makeNodeOrBlossom(b)] == 1) {
                                blossomdual[b] += delta;
                            } else if (label[makeNodeOrBlossom(b)] == 2) {
                                blossomdual[b] -= delta;
                            }
                        }
                    }
                }

                // Take action at the point where minimum delta occurred
                if (deltatype == 1) {
                    break;
                } else if (deltatype == 2) {
                    // Add edge (v,w) to allowedge
                    allowedge[deltaedge] = allowedge[{deltaedge.second, deltaedge.first}] = true;
                    Node v = deltaedge.first;
                    queue.push_back(v);
                } else if (deltatype == 3) {
                    // Add edge (v,w) to allowedge
                    allowedge[deltaedge] = allowedge[{deltaedge.second, deltaedge.first}] = true;
                    Node v = deltaedge.first;
                    queue.push_back(v);
                } else if (deltatype == 4) {
                    expandBlossom(deltablossom, false);
                }
            }

            // Verify that the matching is symmetric
            for (const auto &[v, w] : mate) {
                assert(mate[w] == v);
            }

            if (!augmented) {
                break;
            }

            // Expand all S-blossoms with zero dual
            for (auto it = blossomdual.begin(); it != blossomdual.end();) {
                auto b = it->first;
                ++it; // Increment iterator before potential erasure

                if (blossomdual.find(b) == blossomdual.end())
                    continue;

                if (blossomparent[b] == nullptr &&
                    label.find(makeNodeOrBlossom(b)) != label.end() &&
                    label[makeNodeOrBlossom(b)] == 1 &&
                    blossomdual[b] == 0) {
                    expandBlossom(b, true);
                }
            }
        }

        return get_matching();
    }

    std::pair<std::vector<std::pair<Node, Node>>, Node> get_matching() {
        std::vector<std::pair<Node, Node>> matching;
        std::vector<int> mark(graph_matrix.size(), 0);
        Node remaining_node = -1;
        for (const auto &[v, w] : mate) {
            if (v < w) {
                matching.emplace_back(v, w);
                mark[v] = mark[w] = 1;
            }
        }
        for (const auto &node : mark) {
            if (node == 0) {
                remaining_node = node;
                break;
            }
        }

        return {matching, remaining_node};
    }
};

int main() {
    auto num_of_nodes = 4;
    std::vector<std::vector<int>> matrix(num_of_nodes, std::vector<int>(num_of_nodes, 0));
    freopen("inp.txt", "r", stdin);
    for (int i = 0; i < num_of_nodes; i++) {
        for (int j = 0; j < num_of_nodes; j++) {
            std::cin >> matrix[i][j];
        }
    }
    MaxWeightMatching match(matrix);
    auto [matching, remaining_node] = match.maxWeightMatching();
    for (auto const &[u, v] : matching) {
        std::cout << u << " " << v << "\n";
    }
    return 1;
}
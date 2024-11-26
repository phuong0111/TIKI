#include <bits/stdc++.h>

typedef long long ll;

// Forward declarations
class Blossom;
using BlossomPtr = std::shared_ptr<Blossom>;
using Node = int;
using Weight = ll;
using Edge = std::pair<Node, Node>;

// Custom hasher for Edge
struct EdgeHash {
    std::size_t operator()(const Edge &edge) const {
        return std::hash<Node>()(edge.first) ^ (std::hash<Node>()(edge.second) << 1);
    }
};

// Result structure for matching
struct MatchingResult {
    std::vector<Edge> matches;
    std::optional<Node> remaining_node;
};

// Utility class for blossom representation
class Blossom {
  public:
    std::vector<std::variant<Node, BlossomPtr>> childs;
    std::vector<Edge> edges;
    std::optional<std::vector<Edge>> mybestedges;

    // Get all leaf vertices in the blossom
    std::vector<Node> leaves() const {
        std::vector<Node> leaf_vertices;
        std::vector<std::variant<Node, BlossomPtr>> stack = childs;

        while (!stack.empty()) {
            auto current = stack.back();
            stack.pop_back();

            if (auto *blossom_ptr = std::get_if<BlossomPtr>(&current)) {
                const auto &child_blossom = *blossom_ptr;
                stack.insert(stack.end(),
                             child_blossom->childs.begin(),
                             child_blossom->childs.end());
            } else if (auto *node_ptr = std::get_if<Node>(&current)) {
                leaf_vertices.push_back(*node_ptr);
            }
        }
        return leaf_vertices;
    }
};

class GraphMatching {
  private:
    // Graph representation
    std::vector<std::vector<Weight>> graph;
    Weight max_weight = 0;
    size_t num_nodes;
    bool max_cardinality = true;

    // Matching state
    std::unordered_map<Node, Node> mate;
    std::unordered_map<std::variant<Node, BlossomPtr>, int> label;
    std::unordered_map<std::variant<Node, BlossomPtr>, std::optional<Edge>> labeledge;
    std::unordered_map<Node, std::variant<Node, BlossomPtr>> inblossom;
    std::unordered_map<std::variant<Node, BlossomPtr>,
                       std::optional<std::variant<Node, BlossomPtr>>>
        blossomparent;
    std::unordered_map<std::variant<Node, BlossomPtr>, Node> blossombase;
    std::unordered_map<std::variant<Node, BlossomPtr>,
                       std::optional<Edge>>
        bestedge;
    std::unordered_map<std::variant<Node, BlossomPtr>, Weight> dualvar;
    std::unordered_map<BlossomPtr, Weight> blossomdual;
    std::unordered_map<Edge, bool, EdgeHash> allowedge;
    std::vector<Node> queue;

    // Utility methods
    Weight slack(Node v, Node w) const {
        // Return 2 * slack of edge (v,w)
        return dualvar.at(v) + dualvar.at(w) - 2 * graph[v][w];
    }

    void assign_label(Node w, int t, std::optional<Node> v) {
        auto b = inblossom.at(w);
        assert(!label.count(w) && !label.count(b));

        label[w] = label[b] = t;
        if (v) {
            labeledge[w] = labeledge[b] = Edge(*v, w);
        } else {
            labeledge[w] = labeledge[b] = std::nullopt;
        }

        bestedge[w] = bestedge[b] = std::nullopt;

        if (t == 1) {
            // b is a top-level S-vertex/blossom; enqueue its sub-vertices
            if (auto *blossom_ptr = std::get_if<BlossomPtr>(&b)) {
                auto leaves = (*blossom_ptr)->leaves();
                queue.insert(queue.end(), leaves.begin(), leaves.end());
            } else {
                queue.push_back(std::get<Node>(b));
            }
        } else if (t == 2) {
            // b is a top-level T-vertex/blossom; assign label S to its mate
            // (if b is a non-trivial blossom, its base is the only vertex with a mate)
            auto base = blossombase.at(b);
            assert(mate.count(base));
            assign_label(mate[base], 1, base);
        }
    }

    std::optional<Node> scan_blossom(Node v, Node w) {
        // Find a new blossom or augmenting path starting at vertices v and w.
        // Returns the base vertex of the new blossom or nullopt if an augmenting path was found.

        std::vector<std::variant<Node, BlossomPtr>> path;
        std::optional<Node> base;

        // Helper lambda to mark all vertices in path as T-vertices
        auto label_path = [this](const std::vector<std::variant<Node, BlossomPtr>> &path) {
            for (const auto &b : path) {
                label[b] = 1;
            }
        };

        // Loop variables
        std::optional<Node> current_v = v;
        std::optional<Node> current_w = w;

        // Keep going until we find a common ancestor or augmenting path
        while (current_v.has_value()) {
            auto bv = inblossom.at(current_v.value());
            if (label[bv] & 4) { // Vertex is already marked - found ancestor
                base = blossombase.at(bv);
                break;
            }

            assert(label[bv] == 1); // Must be an S-vertex
            path.push_back(bv);     // Add to path
            label[bv] = 5;          // Mark as scanned

            // Check if we've hit an unmatched vertex - augmenting path found
            if (!labeledge[bv].has_value()) {
                assert(!mate.count(blossombase.at(bv)));
                // Reset v to terminate the loop but leave w to explore the other side
                current_v = std::nullopt;
            } else {
                // Get the next vertex to explore
                auto edge = labeledge[bv].value();
                assert(mate[blossombase.at(bv)] == edge.first);

                // Move to the matched vertex
                current_v = edge.first;
                auto b = inblossom.at(current_v.value());
                assert(label[b] == 2); // Must be a T-vertex

                // And then to its neighbor
                auto next_edge = labeledge[b].value();
                current_v = next_edge.first;
            }

            // Swap v and w so we explore both paths alternately
            if (current_w.has_value()) {
                std::swap(current_v, current_w);
            }
        }

        // Reset labels back to S-vertices
        label_path(path);
        return base;
    }

    void add_blossom(Node base, Node v, Node w) {
        // Create a new blossom with the given base vertex, edge (v,w), and path
        auto bb = inblossom[base];
        auto bv = inblossom[v];
        auto bw = inblossom[w];

        // Create the new blossom
        auto b = std::make_shared<Blossom>();
        blossombase[b] = base;
        blossomparent[b] = std::nullopt;

        // Make list of blossom's sub-vertices and edges, and their connections
        std::vector<std::variant<Node, BlossomPtr>> path;
        std::vector<Edge> edges;

        // Trace back from v to base
        auto current = v;
        while (inblossom[current] != bb) {
            // Add bv to the new blossom
            path.push_back(bv);
            edges.push_back(labeledge[bv].value());
            assert(label[bv] == 2 ||
                   (label[bv] == 1 &&
                    labeledge[bv].value().first == mate[blossombase[bv]]));

            // Trace one step back
            current = labeledge[bv].value().first;
            bv = inblossom[current];
        }

        // Add base to the new blossom
        path.push_back(bb);

        // Trace forward from w to base
        current = w;
        while (inblossom[current] != bb) {
            // Add bw to the new blossom
            path.push_back(bw);
            edges.push_back(Edge(labeledge[bw].value().second,
                                 labeledge[bw].value().first));
            assert(label[bw] == 2 ||
                   (label[bw] == 1 &&
                    labeledge[bw].value().first == mate[blossombase[bw]]));

            // Trace one step forward
            current = labeledge[bw].value().first;
            bw = inblossom[current];
        }

        // Reverse lists to put them in the correct order
        std::reverse(path.begin(), path.end());
        std::reverse(edges.begin(), edges.end());

        // Set blossom's children and edges
        b->childs = std::move(path);
        b->edges = std::move(edges);

        // Set vertices' new blossom parent
        for (const auto &child : b->childs) {
            if (auto *blossom_ptr = std::get_if<BlossomPtr>(&child)) {
                blossomparent[*blossom_ptr] = b;
            }
        }

        // Make sure the blossom's base is the first vertex in its child list
        assert(std::holds_alternative<Node>(b->childs[0]) &&
               std::get<Node>(b->childs[0]) == base);

        // Compute blossom's label:
        label[b] = 1; // S-vertex
        // Add the new blossom to the label dicts
        labeledge[b] = labeledge[bb];
        // Set the new blossom's dual variable to zero
        blossomdual[b] = 0;

        // Relabel vertices and find the best edge for the new blossom
        std::unordered_map<std::variant<Node, BlossomPtr>, Edge> best_edge_to;

        for (const auto &child : b->childs) {
            // Update inblossom for all vertices in the blossom
            if (auto *blossom_ptr = std::get_if<BlossomPtr>(&child)) {
                for (Node v : (*blossom_ptr)->leaves()) {
                    inblossom[v] = b;
                }
                // Compute the best edge for the sub-blossom
                if ((*blossom_ptr)->mybestedges.has_value()) {
                    for (const auto &edge : (*blossom_ptr)->mybestedges.value()) {
                        update_best_edge(edge, *blossom_ptr, best_edge_to);
                    }
                    (*blossom_ptr)->mybestedges = std::nullopt; // Clear cached best edges
                } else {
                    // Compute best edges for all vertices in the sub-blossom
                    for (Node v : (*blossom_ptr)->leaves()) {
                        for (Node u = 0; u < num_nodes; ++u) {
                            if (v != u) {
                                update_best_edge(Edge(v, u), *blossom_ptr, best_edge_to);
                            }
                        }
                    }
                }
                // Clear blossom's best edge cache
                bestedge[*blossom_ptr] = std::nullopt;
            } else {
                Node v = std::get<Node>(child);
                inblossom[v] = b;
                // Compute best edges for the vertex
                for (Node u = 0; u < num_nodes; ++u) {
                    if (v != u) {
                        update_best_edge(Edge(v, u), v, best_edge_to);
                    }
                }
                bestedge[v] = std::nullopt;
            }
        }

        // Save the best edges for the blossom
        b->mybestedges = std::vector<Edge>();
        for (const auto &[_, edge] : best_edge_to) {
            b->mybestedges.value().push_back(edge);
        }

        // Select the edge with minimum slack as the blossom's best edge
        std::optional<Edge> best_edge;
        Weight best_slack = std::numeric_limits<Weight>::infinity();

        for (const auto &edge : b->mybestedges.value()) {
            Weight current_slack = slack(edge.first, edge.second);
            if (!best_edge || current_slack < best_slack) {
                best_edge = edge;
                best_slack = current_slack;
            }
        }

        bestedge[b] = best_edge;
    }

    void update_best_edge(Edge edge, const std::variant<Node, BlossomPtr> &bv,
                          std::unordered_map<std::variant<Node, BlossomPtr>, Edge> &best_edge_to) {
        Node i = edge.first;
        Node j = edge.second;

        if (inblossom[j] != inblossom[i]) {
            auto bj = inblossom[j];
            if (label.count(bj) && label[bj] == 1) {
                // j's blossom is also S-vertex; save edge with minimum slack
                if (!best_edge_to.count(bj) ||
                    slack(i, j) < slack(best_edge_to[bj].first, best_edge_to[bj].second)) {
                    best_edge_to[bj] = edge;
                }
            }
        }
    }

    void augment_blossom(const BlossomPtr &b, Node v) {
        std::function<void(const BlossomPtr &, Node)> _recurse;
        _recurse = [&](const BlossomPtr &b, Node v) {
            Node t = v;
            auto &childs = b->childs;
            auto &edges = b->edges;

            // Find position of v in blossom's child list
            size_t i = 0;
            for (; i < childs.size(); ++i) {
                if (std::holds_alternative<Node>(childs[i])) {
                    if (std::get<Node>(childs[i]) == v)
                        break;
                } else {
                    auto bv = std::get<BlossomPtr>(childs[i]);
                    auto leaves = bv->leaves();
                    if (std::find(leaves.begin(), leaves.end(), v) != leaves.end())
                        break;
                }
            }
            assert(i < childs.size());

            // Determine direction and step size
            size_t j = i;
            int jstep;
            if (i & 1) {
                j -= childs.size();
                jstep = 1;
            } else {
                jstep = -1;
            }

            // Match alternating edges along the blossom
            while (j != 0) {
                j += jstep;
                Node w = jstep == 1 ? edges[j].first : edges[j - 1].second;
                Node x = jstep == 1 ? edges[j].second : edges[j - 1].first;

                if (std::holds_alternative<BlossomPtr>(childs[j])) {
                    _recurse(std::get<BlossomPtr>(childs[j]), w);
                }

                j += jstep;
                if (std::holds_alternative<BlossomPtr>(childs[j])) {
                    _recurse(std::get<BlossomPtr>(childs[j]), x);
                }

                // Match the edge between sub-blossoms
                mate[w] = x;
                mate[x] = w;
            }

            // Rotate blossom if necessary
            if (i > 0) {
                std::rotate(childs.begin(), childs.begin() + i, childs.end());
                std::rotate(edges.begin(), edges.begin() + i, edges.end());
                blossombase[b] = v;
            }
        };

        _recurse(b, v);
    }

    void augment_matching(Node v, Node w) {
        for (auto [s, j] : std::vector<std::pair<Node, Node>>{{v, w}, {w, v}}) {
            while (true) {
                auto bs = inblossom[s];
                assert(label[bs] == 1);
                assert(!labeledge[bs].has_value() ||
                       labeledge[bs].value().first == mate[blossombase[bs]]);

                if (std::holds_alternative<BlossomPtr>(bs)) {
                    augment_blossom(std::get<BlossomPtr>(bs), s);
                }

                // Match the vertices
                mate[s] = j;

                if (!labeledge[bs].has_value()) {
                    break;
                }

                Node t = labeledge[bs].value().first;
                auto bt = inblossom[t];
                assert(label[bt] == 2);
                auto edge = labeledge[bt].value();
                s = edge.first;
                j = t;

                if (std::holds_alternative<BlossomPtr>(bt)) {
                    augment_blossom(std::get<BlossomPtr>(bt), j);
                }
                
                mate[j] = s; // Add this line to ensure symmetric matching
            }
        }
    }

    void expand_blossom(const BlossomPtr &b, bool endstage) {
        // Convert sub-blossoms into top-level blossoms.

        // Recursive lambda to handle nested expansion
        std::function<void(const BlossomPtr &, bool)> expand_recursive;
        expand_recursive = [&](const BlossomPtr &b, bool endstage) {
            // Convert sub-blossoms into top-level blossoms
            for (auto &s : b->childs) {
                // Remove the parent pointer for each sub-vertex/blossom
                blossomparent[s] = std::nullopt;

                if (auto *sub_blossom_ptr = std::get_if<BlossomPtr>(&s)) {
                    if (endstage && blossomdual[*sub_blossom_ptr] == 0) {
                        // Recursively expand this sub-blossom
                        expand_recursive(*sub_blossom_ptr, endstage);
                    } else {
                        // Just assign vertices to the top-level blossom
                        for (Node v : (*sub_blossom_ptr)->leaves()) {
                            inblossom[v] = *sub_blossom_ptr;
                        }
                    }
                } else {
                    // Simple vertex - just assign it to itself
                    Node v = std::get<Node>(s);
                    inblossom[v] = v;
                }
            }

            // If we're expanding a T-blossom during a stage (not endstage),
            // we need to process additional label information
            if (!endstage && label.count(b) && label[b] == 2) {
                // Find the S-vertex through which the expanding blossom obtained its label
                Node entrychild = std::visit([&](const auto &val) -> Node {
                    if constexpr (std::is_same_v<std::decay_t<decltype(val)>, Node>) {
                        return val;
                    } else {
                        return blossombase[val];
                    }
                },
                                             inblossom[labeledge[b].value().second]);

                // Find its position in the blossom's child list
                auto it = std::find_if(b->childs.begin(), b->childs.end(),
                                       [&](const auto &child) {
                                           return std::visit([&](const auto &val) -> bool {
                                               if constexpr (std::is_same_v<std::decay_t<decltype(val)>, Node>) {
                                                   return val == entrychild;
                                               } else {
                                                   return blossombase[val] == entrychild;
                                               }
                                           },
                                                             child);
                                       });
                assert(it != b->childs.end());
                size_t j = std::distance(b->childs.begin(), it);

                // Determine traversal direction
                int jstep;
                if (j & 1) {
                    j -= b->childs.size();
                    jstep = 1;
                } else {
                    jstep = -1;
                }

                // Walk around the blossom maintaining labels
                Node v = labeledge[b].value().first;
                Node w = labeledge[b].value().second;

                while (j != 0) {
                    // Traverse forwards or backwards through the blossom structure
                    if (jstep == 1) {
                        Node p = b->edges[j].first;
                        Node q = b->edges[j].second;

                        // Label the T-vertex
                        label.erase(w);
                        label.erase(q);
                        assign_label(w, 2, v);

                        // Allow the connecting edge
                        allowedge[{p, q}] = allowedge[{q, p}] = true;
                        j += jstep;

                        // Update v and w for next iteration
                        v = b->edges[j].first;
                        w = b->edges[j].second;
                    } else {
                        Node q = b->edges[j - 1].first;
                        Node p = b->edges[j - 1].second;

                        // Label the T-vertex
                        label.erase(w);
                        label.erase(q);
                        assign_label(w, 2, v);

                        // Allow the connecting edge
                        allowedge[{p, q}] = allowedge[{q, p}] = true;
                        j += jstep;

                        // Update v and w for next iteration
                        w = b->edges[j - 1].first;
                        v = b->edges[j - 1].second;
                    }
                    allowedge[{v, w}] = allowedge[{w, v}] = true;
                    j += jstep;
                }

                // Relabel the base T-vertex
                auto bw = b->childs[j];
                label[w] = label[bw] = 2;
                labeledge[w] = labeledge[bw] = Edge(v, w);
                bestedge[bw] = std::nullopt;

                // Continue consistently labeling the rest of the blossom
                j += jstep;
                while (std::visit([&](const auto &val) -> bool {
                    if constexpr (std::is_same_v<std::decay_t<decltype(val)>, Node>) {
                        return val != entrychild;
                    } else {
                        return blossombase[val] != entrychild;
                    }
                },
                                  b->childs[j])) {
                    auto bv = b->childs[j];
                    if (label.count(bv) && label[bv] == 1) {
                        j += jstep;
                        continue;
                    }

                    // Find the vertex to relabel
                    Node v;
                    if (auto *sub_blossom_ptr = std::get_if<BlossomPtr>(&bv)) {
                        v = [&]() -> Node {
                            for (Node vertex : (*sub_blossom_ptr)->leaves()) {
                                if (label.count(vertex))
                                    return vertex;
                            }
                            assert(false);
                            return -1;
                        }();
                    } else {
                        v = std::get<Node>(bv);
                    }

                    // Verify it's a T-vertex and relabel it
                    if (label.count(v)) {
                        assert(label[v] == 2);
                        assert(inblossom[v] == bv);
                        label.erase(v);
                        label.erase(mate[blossombase[bv]]);
                        assign_label(v, 2, labeledge[v].value().first);
                    }
                    j += jstep;
                }
            }

            // Clean up the expanding blossom
            label.erase(b);
            labeledge.erase(b);
            bestedge.erase(b);
            blossomparent.erase(b);
            blossombase.erase(b);
            blossomdual.erase(b);
        };

        // Start the recursive expansion
        expand_recursive(b, endstage);
    }

  public:
    GraphMatching(const std::vector<std::vector<Weight>> &graph_matrix)
        : graph(graph_matrix),
          num_nodes(graph_matrix.size()) {
    }

    // Get the final matching result
    MatchingResult get_matching() const {
        MatchingResult result;
        std::vector<Node> nodes;
        for (Node i = 0; i < num_nodes; i++) {
            nodes.push_back(i);
        }

        // Need to check both k->v and v->k in mate map
        for (Node k = 0; k < num_nodes; k++) {
            auto it = mate.find(k);
            if (it != mate.end()) {
                Node v = it->second;
                if (k < v && nodes[k] != -1 && nodes[v] != -1) {
                    result.matches.emplace_back(k, v);
                    nodes[k] = nodes[v] = -1;
                }
            }
        }

        // Find remaining unmatched node
        auto it = std::find_if(nodes.begin(), nodes.end(),
                               [](Node n) { return n != -1; });
        if (it != nodes.end()) {
            result.remaining_node = *it;
        }

        return result;
    }

    MatchingResult max_weight_matching() {
        for (const auto &adj : graph)
            for (const auto &wei : adj)
                max_weight = std::max(max_weight, wei);

        // Initialize inblossom map
        for (Node i = 0; i < num_nodes; ++i) {
            inblossom[i] = i;
            blossombase[i] = i;
            bestedge[i] = std::nullopt;
            dualvar[i] = max_weight;
        }

        // Main loop: continue until no further improvement is possible
        while (true) {
            // Reset all labels and status
            label.clear();
            labeledge.clear();
            bestedge.clear();
            for (const auto &[b, _] : blossomdual) {
                b->mybestedges = std::nullopt;
            }
            allowedge.clear();
            queue.clear();

            // Label single vertices with S
            for (Node v = 0; v < num_nodes; ++v) {
                if (!mate.count(v) && !label.count(inblossom[v])) {
                    assign_label(v, 1, std::nullopt);
                }
            }

            // Loop until we succeed in augmenting the matching
            bool augmented = false;
            while (true) {
                // Keep searching for a path while we have vertices in queue
                while (!queue.empty() && !augmented) {
                    Node v = queue.back();
                    queue.pop_back();
                    assert(label[inblossom[v]] == 1);

                    // Look at neighbors of v
                    for (Node w = 0; w < num_nodes; ++w) {
                        if (v == w)
                            continue;

                        auto bv = inblossom[v];
                        auto bw = inblossom[w];
                        if (bv == bw)
                            continue; // Skip vertices in same blossom

                        if (!allowedge.count({v, w})) {
                            // Compute slack
                            Weight kslack = slack(v, w);
                            if (kslack <= 0) {
                                // Edge is allowable - label endpoint
                                allowedge[{v, w}] = allowedge[{w, v}] = true;
                            }
                        }

                        if (allowedge.count({v, w})) {
                            if (!label.count(bw)) {
                                // (b) is unexposed; label with T and add to queue
                                assign_label(w, 2, v);
                            } else if (label[bw] == 1) {
                                // (c) is an S-vertex - handle blossom
                                auto base = scan_blossom(v, w);
                                if (base) {
                                    add_blossom(*base, v, w);
                                } else {
                                    // Found an augmenting path
                                    augment_matching(v, w);
                                    augmented = true;
                                    break;
                                }
                            } else if (!label.count(w)) {
                                assert(label[bw] == 2);
                                label[w] = 2;
                                labeledge[w] = Edge(v, w);
                            }
                        } else if (label[bw] == 1) {
                            // Keep track of the least-slack edge to the blossom
                            if (!bestedge.count(bv) || !bestedge[bv].has_value() ||
                                (bestedge[bv].has_value() && slack(v, w) < slack(bestedge[bv].value().first, bestedge[bv].value().second))) {
                                bestedge[bv] = Edge(v, w);
                            }
                        } else if (!label.count(w)) {
                            if (!bestedge.count(w) || !bestedge[w].has_value() ||
                                (bestedge[w].has_value() && slack(v, w) < slack(bestedge[w].value().first, bestedge[w].value().second))) {
                                bestedge[w] = Edge(v, w);
                            }
                        }
                    }
                }

                if (augmented) {
                    break;
                }

                // No augmenting path found - compute delta for dual variables
                int deltatype = -1;
                Weight delta;
                std::optional<Edge> deltaedge;
                std::optional<BlossomPtr> deltablossom;

                // Check if we can improve dual variables
                for (Node v = 0; v < num_nodes; ++v) {
                    if (!label.count(inblossom[v]) && bestedge.count(v)) {
                        Weight d = slack(bestedge[v].value().first, bestedge[v].value().second);
                        if (deltatype == -1 || d < delta) {
                            delta = d;
                            deltatype = 2;
                            deltaedge = bestedge[v];
                        }
                    }
                }

                for (const auto &[b, _] : blossomparent) {
                    if (!blossomparent[b].has_value() && label.count(b) && label[b] == 1 && bestedge.count(b)) {
                        Weight kslack = slack(bestedge[b].value().first, bestedge[b].value().second);
                        Weight d = kslack / 2;
                        if (deltatype == -1 || d < delta) {
                            delta = d;
                            deltatype = 3;
                            deltaedge = bestedge[b];
                        }
                    }
                }

                for (const auto &[b, dual] : blossomdual) {
                    if (!blossomparent[b].has_value() && label.count(b) && label[b] == 2 &&
                        (deltatype == -1 || dual < delta)) {
                        delta = dual;
                        deltatype = 4;
                        deltablossom = b;
                    }
                }

                if (deltatype == -1) {
                    deltatype = 1;
                    delta = std::max(Weight(0), std::min_element(dualvar.begin(), dualvar.end(), [](const auto &a, const auto &b) { return a.second < b.second; })->second);
                }

                // Update dual variables
                // std::cout << "deltatype: " << deltatype << ", delta: " << delta << "\n";
                for (Node v = 0; v < num_nodes; ++v) {
                    if (label.count(inblossom[v])) {
                        if (label[inblossom[v]] == 1) {
                            dualvar[v] -= delta;
                        } else if (label[inblossom[v]] == 2) {
                            dualvar[v] += delta;
                        }
                    }
                }

                for (const auto &[b, dual] : blossomdual) {
                    if (!blossomparent[b].has_value()) {
                        if (label.count(b)) {
                            if (label[b] == 1) {
                                blossomdual[b] += delta;
                            } else if (label[b] == 2) {
                                blossomdual[b] -= delta;
                            }
                        }
                    }
                }

                // Take action at the point where minimum delta occurred
                if (deltatype == 1) {
                    break; // No further improvement possible
                } else if (deltatype == 2) {
                    allowedge[deltaedge.value()] = allowedge[{deltaedge.value().second, deltaedge.value().first}] = true;
                    Node v = deltaedge.value().first;
                    queue.push_back(v);
                } else if (deltatype == 3) {
                    allowedge[deltaedge.value()] = allowedge[{deltaedge.value().second, deltaedge.value().first}] = true;
                    Node v = deltaedge.value().first;
                    queue.push_back(v);
                } else if (deltatype == 4) {
                    expand_blossom(deltablossom.value(), false);
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
            for (const auto &[b, dual] : blossomdual) {
                if (!blossomparent[b].has_value() && label.count(b) &&
                    label[b] == 1 && dual == 0) {
                    expand_blossom(b, true);
                }
            }
        }

        // Return the final matching
        return get_matching();
    }

    MatchingResult min_weight_matching() {
        // Get number of nodes from matrix dimensions
        size_t num_nodes = graph.size();
        if (num_nodes == 0) {
            return MatchingResult{}; // Return empty result for empty graph
        }

        for (size_t i = 0; i < num_nodes; ++i) {
            for (size_t j = 0; j < num_nodes; ++j) {
                if (i != j && graph[i][j] != 0) {
                    graph[i][j] = (max_weight + 1) - graph[i][j];
                }
            }
        }

        return max_weight_matching();
    }
};

// Utility function to create a test graph
std::vector<std::vector<Weight>> create_test_graph(size_t num_nodes) {
    std::vector<std::vector<Weight>> matrix(num_nodes,
                                            std::vector<Weight>(num_nodes, 0));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1.0, 50.0);

    for (size_t i = 0; i < num_nodes; ++i) {
        for (size_t j = i + 1; j < num_nodes; ++j) {
            Weight weight = dis(gen);
            matrix[i][j] = weight;
            matrix[j][i] = weight; // Make it symmetric
        }
    }

    for (size_t i = 0; i < num_nodes; ++i) {
        for (size_t j = 0; j < num_nodes; ++j) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << "\n";
    }

    return matrix;
}

int main() {
    auto matrix = create_test_graph(6);
    // freopen("inp.txt", "r", stdin);
    // for (int i = 0; i < 6; i++)
    //     for (int j = 0; j < 6; j++)
    //         std::cin >> matrix[i][j];
    auto match = GraphMatching(matrix);
    auto [matches, remaining_node] = match.max_weight_matching();
    for (const auto &[u, v] : matches)
        std::cout << u << " " << v << "\n";
    return 0;
}
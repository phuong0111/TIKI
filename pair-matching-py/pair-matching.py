def get_matching(mate: dict[int, int], gnodes: list[int]):
    matches = []
    remaining_node = -1
    nodes = gnodes.copy()
    for k, v in mate.items():
        if k < v:
            matches.append((k, v))
            nodes[k] = nodes[v] = -1
    for node in nodes:
        if node != -1:
            remaining_node = node
    return matches, remaining_node
    

def max_weight_matching(graph_matrix, maxcardinality=True):
    """Compute a maximum-weighted matching using a 2D adjacency matrix.
    
    Parameters:
    -----------
    graph_matrix : List[List[int]]
        2D adjacency matrix where graph_matrix[i][j] represents the weight of edge (i,j)
    maxcardinality : bool, optional (default=False)
        If True, compute maximum-cardinality matching with maximum weight
        
    Returns:
    --------
    set : A maximal matching represented as a set of edge tuples
    """
    class NoNode:
        """Dummy value which is different from any node."""

    class Blossom:
        """Representation of a non-trivial blossom or sub-blossom."""
        __slots__ = ["childs", "edges", "mybestedges"]
        
        def leaves(self):
            """Return all leaves (vertices) in the blossom"""
            leaf_vertices = []
            stack = [*self.childs]
            while stack:
                t = stack.pop()
                if isinstance(t, Blossom):
                    stack.extend(t.childs)
                else:
                    leaf_vertices.append(t)
            return leaf_vertices
    
    # Get number of nodes from matrix dimensions
    num_nodes = len(graph_matrix)
    if num_nodes == 0:
        return set()
    
    # Create list of nodes
    gnodes = list(range(num_nodes))
    
    # Find maximum edge weight
    maxweight = 0
    allinteger = True
    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            wt = graph_matrix[i][j]
            if wt > maxweight:
                maxweight = wt
            allinteger = allinteger and isinstance(wt, (int))
    
    # Initialize matching state
    mate = {}
    label = {}
    labeledge = {}
    inblossom = dict(zip(gnodes, gnodes))
    blossomparent = dict(zip(gnodes, [None] * len(gnodes)))
    blossombase = dict(zip(gnodes, gnodes))
    bestedge = {}
    dualvar = dict(zip(gnodes, [maxweight] * len(gnodes)))
    blossomdual = {}
    allowedge = {}
    queue = []

    def slack(v, w):
        """Return 2 * slack of edge (v,w)"""
        return dualvar[v] + dualvar[w] - 2 * graph_matrix[v][w]

    def assignLabel(w, t, v):
        """Assign label t to the top-level blossom containing vertex w"""
        b = inblossom[w]
        assert label.get(w) is None and label.get(b) is None
        label[w] = label[b] = t
        if v is not None:
            labeledge[w] = labeledge[b] = (v, w)
        else:
            labeledge[w] = labeledge[b] = None
        bestedge[w] = bestedge[b] = None
        if t == 1:
            if isinstance(b, Blossom):
                queue.extend(b.leaves())
            else:
                queue.append(b)
        elif t == 2:
            base = blossombase[b]
            assignLabel(mate[base], 1, base)

    def scanBlossom(v, w):
        """Find a new blossom or augmenting path"""
        path = []
        base = NoNode
        while v is not NoNode:
            b = inblossom[v]
            if label[b] & 4:
                base = blossombase[b]
                break
            assert label[b] == 1
            path.append(b)
            label[b] = 5
            if labeledge[b] is None:
                assert blossombase[b] not in mate
                v = NoNode
            else:
                assert labeledge[b][0] == mate[blossombase[b]]
                v = labeledge[b][0]
                b = inblossom[v]
                assert label[b] == 2
                v = labeledge[b][0]
            if w is not NoNode:
                v, w = w, v
        for b in path:
            label[b] = 1
        return base

    def addBlossom(base, v, w):
        """Construct a new blossom"""
        bb = inblossom[base]
        bv = inblossom[v]
        bw = inblossom[w]
        b = Blossom()
        blossombase[b] = base
        blossomparent[b] = None
        blossomparent[bb] = b
        b.childs = path = []
        b.edges = edgs = [(v, w)]
        while bv != bb:
            blossomparent[bv] = b
            path.append(bv)
            edgs.append(labeledge[bv])
            assert label[bv] == 2 or (
                label[bv] == 1 and labeledge[bv][0] == mate[blossombase[bv]]
            )
            v = labeledge[bv][0]
            bv = inblossom[v]
        path.append(bb)
        path.reverse()
        edgs.reverse()
        while bw != bb:
            blossomparent[bw] = b
            path.append(bw)
            edgs.append((labeledge[bw][1], labeledge[bw][0]))
            assert label[bw] == 2 or (
                label[bw] == 1 and labeledge[bw][0] == mate[blossombase[bw]]
            )
            w = labeledge[bw][0]
            bw = inblossom[w]
        assert label[bb] == 1
        label[b] = 1
        labeledge[b] = labeledge[bb]
        blossomdual[b] = 0
        for v in b.leaves():
            if label[inblossom[v]] == 2:
                queue.append(v)
            inblossom[v] = b
        bestedgeto = {}
        for bv in path:
            if isinstance(bv, Blossom):
                if bv.mybestedges is not None:
                    nblist = bv.mybestedges
                    bv.mybestedges = None
                else:
                    nblist = [
                        (v, w) 
                        for v in bv.leaves() 
                        for w in range(num_nodes) 
                        if v != w
                    ]
            else:
                nblist = [(bv, w) for w in range(num_nodes) if bv != w]
            for k in nblist:
                (i, j) = k
                if inblossom[j] == b:
                    i, j = j, i
                bj = inblossom[j]
                if (
                    bj != b
                    and label.get(bj) == 1
                    and ((bj not in bestedgeto) or slack(i, j) < slack(*bestedgeto[bj]))
                ):
                    bestedgeto[bj] = k
            bestedge[bv] = None
        b.mybestedges = list(bestedgeto.values())
        mybestedge = None
        bestedge[b] = None
        for k in b.mybestedges:
            kslack = slack(*k)
            if mybestedge is None or kslack < mybestslack:
                mybestedge = k
                mybestslack = kslack
        bestedge[b] = mybestedge

    def augmentMatching(v, w):
        """Swap matched/unmatched edges over alternating path"""
        for s, j in ((v, w), (w, v)):
            while True:
                bs = inblossom[s]
                assert label[bs] == 1
                assert (labeledge[bs] is None and blossombase[bs] not in mate) or (
                    labeledge[bs][0] == mate[blossombase[bs]]
                )
                if isinstance(bs, Blossom):
                    augmentBlossom(bs, s)
                mate[s] = j
                if labeledge[bs] is None:
                    break
                t = labeledge[bs][0]
                bt = inblossom[t]
                assert label[bt] == 2
                s, j = labeledge[bt]
                assert blossombase[bt] == t
                if isinstance(bt, Blossom):
                    augmentBlossom(bt, j)
                mate[j] = s

    def augmentBlossom(b, v):
        """Swap matched/unmatched edges over alternating path in blossom b between vertex v and base vertex"""
        def _recurse(b, v):
            # Bubble up through blossom tree from vertex v
            t = v
            while blossomparent[t] != b:
                t = blossomparent[t]
                
            # Recursively deal with first sub-blossom
            if isinstance(t, Blossom):
                _recurse(t, v)
                
            # Decide direction around the blossom
            i = j = b.childs.index(t)
            if i & 1:
                # Start index is odd; go forward and wrap
                j -= len(b.childs)
                jstep = 1
            else:
                # Start index is even; go backward
                jstep = -1
                
            # Move along the blossom until reaching base
            while j != 0:
                # Handle next sub-blossom
                j += jstep
                t = b.childs[j]
                if jstep == 1:
                    w, x = b.edges[j]
                else:
                    x, w = b.edges[j - 1]
                if isinstance(t, Blossom):
                    _recurse(t, w)
                    
                # Handle the sub-blossom after that
                j += jstep
                t = b.childs[j]
                if isinstance(t, Blossom):
                    _recurse(t, x)
                    
                # Match the edge between sub-blossoms
                mate[w] = x
                mate[x] = w
                
            # Rotate list of sub-blossoms to put new base at front
            b.childs = b.childs[i:] + b.childs[:i]
            b.edges = b.edges[i:] + b.edges[:i]
            blossombase[b] = blossombase[b.childs[0]]
            assert blossombase[b] == v
            
        _recurse(b, v)

    def expandBlossom(b, endstage):
        """Expand the given top-level blossom"""
        def _recurse(b, endstage):
            # Convert sub-blossoms into top-level blossoms
            for s in b.childs:
                blossomparent[s] = None
                if isinstance(s, Blossom):
                    if endstage and blossomdual[s] == 0:
                        # Recursively expand this sub-blossom
                        _recurse(s, endstage)
                    else:
                        for v in s.leaves():
                            inblossom[v] = s
                else:
                    inblossom[s] = s
                    
            # Handle relabeling of T-blossoms during a stage
            if (not endstage) and label.get(b) == 2:
                # Find sub-blossom through which expanding blossom got its label
                entrychild = inblossom[labeledge[b][1]]
                j = b.childs.index(entrychild)
                if j & 1:
                    j -= len(b.childs)
                    jstep = 1
                else:
                    jstep = -1
                    
                # Move along blossom until reaching base
                v, w = labeledge[b]
                while j != 0:
                    if jstep == 1:
                        p, q = b.edges[j]
                    else:
                        q, p = b.edges[j - 1]
                        
                    label[w] = None
                    label[q] = None
                    assignLabel(w, 2, v)
                    allowedge[(p, q)] = allowedge[(q, p)] = True
                    j += jstep
                    
                    if jstep == 1:
                        v, w = b.edges[j]
                    else:
                        w, v = b.edges[j - 1]
                    allowedge[(v, w)] = allowedge[(w, v)] = True
                    j += jstep
                    
                # Relabel base T-sub-blossom
                bw = b.childs[j]
                label[w] = label[bw] = 2
                labeledge[w] = labeledge[bw] = (v, w)
                bestedge[bw] = None
                
                # Continue until getting back to entrychild
                j += jstep
                while b.childs[j] != entrychild:
                    bv = b.childs[j]
                    if label.get(bv) == 1:
                        j += jstep
                        continue
                        
                    if isinstance(bv, Blossom):
                        for v in bv.leaves():
                            if label.get(v):
                                break
                    else:
                        v = bv
                        
                    if label.get(v):
                        assert label[v] == 2
                        assert inblossom[v] == bv
                        label[v] = None
                        label[mate[blossombase[bv]]] = None
                        assignLabel(v, 2, labeledge[v][0])
                    j += jstep
                    
            # Clean up blossom
            label.pop(b, None)
            labeledge.pop(b, None) 
            bestedge.pop(b, None)
            del blossomparent[b]
            del blossombase[b]
            del blossomdual[b]
            
        _recurse(b, endstage)

    # Main loop: continue until no further improvement is possible.
    while True:
        label.clear()
        labeledge.clear()
        bestedge.clear()
        for b in blossomdual:
            b.mybestedges = None
        allowedge.clear()
        queue[:] = []
        
        for v in gnodes:
            if (v not in mate) and label.get(inblossom[v]) is None:
                assignLabel(v, 1, None)
        
        augmented = 0
        while True:
            while queue and not augmented:
                v = queue.pop()
                assert label[inblossom[v]] == 1
                
                for w in range(num_nodes):
                    if v == w:
                        continue
                    bv = inblossom[v]
                    bw = inblossom[w]
                    if bv == bw:
                        continue
                    if (v, w) not in allowedge:
                        kslack = slack(v, w)
                        if kslack <= 0:
                            allowedge[(v, w)] = allowedge[(w, v)] = True
                    if (v, w) in allowedge:
                        if label.get(bw) is None:
                            assignLabel(w, 2, v)
                        elif label.get(bw) == 1:
                            base = scanBlossom(v, w)
                            if base is not NoNode:
                                addBlossom(base, v, w)
                            else:
                                augmentMatching(v, w)
                                augmented = 1
                                break
                        elif label.get(w) is None:
                            assert label[bw] == 2
                            label[w] = 2
                            labeledge[w] = (v, w)
                    elif label.get(bw) == 1:
                        print(v)
                        print(bestedge)
                        if bestedge.get(bv) is None or kslack < slack(*bestedge[bv]):
                            bestedge[bv] = (v, w)
                    elif label.get(w) is None:
                        if bestedge.get(w) is None or kslack < slack(*bestedge[w]):
                            bestedge[w] = (v, w)

            if augmented:
                break

            # There is no augmenting path under these constraints;
            # compute delta and reduce slack in the optimization problem.
            deltatype = -1
            delta = deltaedge = deltablossom = None

            for v in gnodes:
                if label.get(inblossom[v]) is None and bestedge.get(v) is not None:
                    d = slack(*bestedge[v])
                    if deltatype == -1 or d < delta:
                        delta = d
                        deltatype = 2
                        deltaedge = bestedge[v]

            for b in blossomparent:
                if (
                    blossomparent[b] is None
                    and label.get(b) == 1
                    and bestedge.get(b) is not None
                ):
                    kslack = slack(*bestedge[b])
                    if allinteger:
                        assert (kslack % 2) == 0
                        d = kslack // 2
                    else:
                        d = kslack / 2.0
                    if deltatype == -1 or d < delta:
                        delta = d
                        deltatype = 3
                        deltaedge = bestedge[b]

            for b in blossomdual:
                if (
                    blossomparent[b] is None
                    and label.get(b) == 2
                    and (deltatype == -1 or blossomdual[b] < delta)
                ):
                    delta = blossomdual[b]
                    deltatype = 4
                    deltablossom = b

            if deltatype == -1:
                deltatype = 1
                delta = max(0, min(dualvar.values()))

            # Update dual variables according to delta
            for v in gnodes:
                if label.get(inblossom[v]) == 1:
                    dualvar[v] -= delta
                elif label.get(inblossom[v]) == 2:
                    dualvar[v] += delta
            for b in blossomdual:
                if blossomparent[b] is None:
                    if label.get(b) == 1:
                        blossomdual[b] += delta
                    elif label.get(b) == 2:
                        blossomdual[b] -= delta

            # Take action at the point where minimum delta occurred
            if deltatype == 1:
                break
            elif deltatype == 2:
                (v, w) = deltaedge
                assert label[inblossom[v]] == 1
                allowedge[(v, w)] = allowedge[(w, v)] = True
                queue.append(v)
            elif deltatype == 3:
                (v, w) = deltaedge
                allowedge[(v, w)] = allowedge[(w, v)] = True
                assert label[inblossom[v]] == 1
                queue.append(v)
            elif deltatype == 4:
                expandBlossom(deltablossom, False)

        # Verify matching is symmetric
        for v in mate:
            assert mate[mate[v]] == v

        if not augmented:
            break

        # Expand all S-blossoms with zero dual
        for b in list(blossomdual.keys()):
            if b not in blossomdual:
                continue
            if blossomparent[b] is None and label.get(b) == 1 and blossomdual[b] == 0:
                expandBlossom(b, True)

    return get_matching(mate=mate, gnodes=gnodes)

def min_weight_matching(graph_matrix):
    """Compute a minimum-weight maximal matching using a 2D adjacency matrix.
    
    This function converts the minimum weight matching problem to a maximum weight
    matching problem by subtracting all edge weights from (max_weight + 1).
    Then it uses max_weight_matching() to find the optimal solution.
    
    Parameters:
    -----------
    graph_matrix : List[List[float]]
        2D adjacency matrix where graph_matrix[i][j] represents the weight of edge (i,j).
        Matrix must be symmetric (undirected graph).
        
    Returns:
    --------
    dict : A minimum weight matching represented as a dictionary mapping vertices 
           to their matched partners
    """
    num_nodes = len(graph_matrix)
    if num_nodes == 0:
        return {}
        
    # Find maximum weight in the graph
    max_weight = 0
    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            if graph_matrix[i][j] > max_weight:
                max_weight = graph_matrix[i][j]
    
    # Create inverted weight matrix where new_weight = (max_weight + 1) - old_weight
    inv_matrix = [[0] * num_nodes for _ in range(num_nodes)]
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j and graph_matrix[i][j] != 0:
                inv_matrix[i][j] = (max_weight + 1) - graph_matrix[i][j]
    
    # Find maximum weight matching on inverted graph
    return max_weight_matching(inv_matrix)

# Example usage
def create_test_graph(num_nodes):
    """Create a test weighted adjacency matrix"""
    import random
    matrix = [[0] * num_nodes for _ in range(num_nodes)]
    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            weight = random.randint(1, 50)
            matrix[i][j] = weight
            matrix[j][i] = weight  # Make it symmetric
    return matrix

# Test the implementation
if __name__ == "__main__":
    # Create a test graph
    # num_nodes = 100
    # test_matrix = create_test_graph(num_nodes=num_nodes)
    test_matrix = []
    import sys
    with open("inp.txt") as f:
        lines = f.readlines()
        for line in lines:
            _ = line.strip()
            test_matrix.append([int(x) for x in line.split()])
    
    # Find minimum weight matching
    matching, remaining_node = max_weight_matching(test_matrix)
    
    # Calculate and print total weight
    total_weight = sum(test_matrix[v][w] for v, w in matching)
    print("\nMinimum Weight Matching:", matching)
    print("Remaining node (odd case): ", remaining_node)
    print("Total Weight:", total_weight)
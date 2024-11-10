from typing import List, Dict, Set, Tuple
import heapq

def min_weight_perfect_matching(graph: List[List[float]]) -> List[Tuple[int, int]]:
    """
    Find minimum weight perfect matching in a complete undirected weighted graph.
    Uses a greedy approach that minimizes the maximum weight of any matched edge.
    
    Args:
        graph: n x n adjacency matrix where graph[i][j] is the weight between nodes i and j
        
    Returns:
        List of (node1, node2) pairs representing the matching
    """
    n = len(graph)
    if n % 2 != 0:
        raise ValueError("Graph must have even number of nodes for perfect matching")
    
    # Create list of all edges with weights
    edges = []
    for i in range(n):
        for j in range(i + 1, n):
            edges.append((graph[i][j], i, j))
    
    # Sort edges by weight
    edges.sort()
    
    # Keep track of matched nodes
    matched: Set[int] = set()
    matches: List[Tuple[int, int]] = []
    
    # Greedily match nodes using smallest available edges
    for weight, u, v in edges:
        if u not in matched and v not in matched:
            matched.add(u)
            matched.add(v)
            matches.append((u, v))
            
            # If we've matched all nodes, we're done
            if len(matched) == n:
                break
                
    return matches

def print_matching_stats(graph: List[List[float]], matches: List[Tuple[int, int]]) -> None:
    """Print statistics about the matching"""
    total_weight = 0
    max_weight = float('-inf')
    
    for u, v in matches:
        weight = graph[u][v]
        total_weight += weight
        max_weight = max(max_weight, weight)
        
    print(f"Total weight of matching: {total_weight}")
    print(f"Maximum edge weight in matching: {max_weight}")
    print("Matches:", matches)

# Example usage
if __name__ == "__main__":
    # Example graph with 4 nodes
    example_graph = [
        [0, 2, 4, 1],
        [2, 0, 3, 5],
        [4, 3, 0, 2],
        [1, 5, 2, 0]
    ]
    
    matches = min_weight_perfect_matching(example_graph)
    print_matching_stats(example_graph, matches)
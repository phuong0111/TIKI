import sys

# input
with open("tc/2/inp.txt") as f:
    sys.stdin = f
    # request = {id: (size, time)}
    requests: dict[int, tuple[int, int]] = {}
    # trucks = {id: depot}
    trucks: dict[int, int] = {}
    # num of point
    num_of_points: int
    # distances
    distances: list[list[int]]
    # trailer depot
    trailer_depot: int
    # trailer load up time
    trailer_load_up_time: int

    num_of_points = int(input().split()[-1]) 
    input()
    distances = [[0 for _ in range(num_of_points + 1)] for _ in range(num_of_points + 1)]
    for _ in range(num_of_points * num_of_points):
        i, j, d = map(int, input().split())
        distances[i][j] = d
        
    trailer_depot, trailer_load_up_time = map(int, input().split()[1:])
    num_of_routes = int(input().split()[-1])
    for _ in range(num_of_routes):
        i, j = map(int, input().split())
        trucks[i] = j
    while True:
        line = input()
        if line == '#':
            break
        _ = line.split()
        id, size, pickup_duration, drop_duration = int(_[1]), int(_[2]), int(_[5]), int(_[8])
        requests[id] = (size, pickup_duration, drop_duration)
    
PICKUP_CONTAINER = "PICKUP_CONTAINER"
PICKUP_CONTAINER_TRAILER = "PICKUP_CONTAINER_TRAILER"
DROP_CONTAINER = "DROP_CONTAINER"
DROP_CONTAINER_TRAILER = "DROP_CONTAINER_TRAILER"
PICKUP_TRAILER = "PICKUP_TRAILER"
DROP_TRAILER = "DROP_TRAILER"
STOP = "STOP"
    
alpha = 100000
def checkValidRouteAndCalculateScore(truck_id: int, routes: list[str]):
    has_trailer = False
    current_container = []
    route_score = 0
    prev_point = trucks[truck_id]
    for stop in routes:
        stop = stop.split()
        point, action, request_id = int(stop[0]), stop[1], None
        if len(stop) == 3:
            request_id = int(stop[2])
        
        if action in [PICKUP_CONTAINER_TRAILER, PICKUP_TRAILER]:
            if has_trailer:
                return False, f"Truck {truck_id} at point {point} already has already trailer, can not do action {action}", 0
            has_trailer = True
        if action in [PICKUP_CONTAINER, PICKUP_CONTAINER_TRAILER]:
            total = sum([_[1] for _ in current_container])
            current_container.append((request_id, requests[request_id][0]))
            if total + requests[request_id][0] > 40:
                return False, f"Truck {truck_id} at point {point} already has load {total}, can not do action {action} with demand {requests[request_id][0]}", 0
        if action in [DROP_CONTAINER, DROP_CONTAINER_TRAILER]:
            if any([_[0] == request_id for _ in current_container]) == False:
                return False, f"Truck {truck_id} at point {point} has no container for request {request_id}, can not do action {action}", 0
            current_container.remove((request_id, requests[request_id][0]))
        if action in [DROP_TRAILER, DROP_CONTAINER_TRAILER]:
            if has_trailer == False:
                return False, f"Truck {truck_id} at point {point} has no trailer, can not do action {action}", 0
            has_trailer = False
        if action == STOP:
            if has_trailer:
                return False, f"Truck {truck_id} at point {point} has trailer, can not do action {action}", 0
            if len(current_container) > 0:
                return False, f"Truck {truck_id} at point {point} has container, can not do action {action}", 0
        route_score += distances[prev_point][point] 
        prev_point = point
        if request_id:
            route_score += requests[request_id][1] if "pickup" in action.lower() else requests[request_id][2]
    return True, "Correct answer", route_score
    
# output
with open("tc/2/out.txt") as f:
    sys.stdin = f
    _ = int(input().split()[-1]) 
    assert _ == num_of_routes
    scores = []
    for _ in range(num_of_routes):
        truck_id = int(input().split()[-1]) 
        lines = []
        while True:
            line = input()
            if line == '#':
                break
            lines.append(line)
        check, message, score = checkValidRouteAndCalculateScore(truck_id=truck_id, routes=lines)
        scores.append(score)
        if check == False:
            print(message)
            break
            
    if check:
        print(f"{sum(scores) = }")
        print(f"{max(scores) = }")
        print(f"Score: {int(1e9) - (sum(scores) + alpha * max(scores))}")
        
        
        
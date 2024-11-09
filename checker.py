import sys

# input
with open("tc/6/inp.txt") as f:
    sys.stdin = f
    # request = {id: (size, pickup_duration, drop_duration)}
    requests: dict[int, tuple[int, int, int]] = {}
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
    travel_time = 0  # F2 component - only travel time
    completion_time = 0  # Track total time including service times
    prev_point = trucks[truck_id]
    
    for stop in routes:
        stop = stop.split()
        point, action, request_id = int(stop[0]), stop[1], None
        if len(stop) == 3:
            request_id = int(stop[2])
        
        # Validate trailer and container operations
        if action in [PICKUP_CONTAINER_TRAILER, PICKUP_TRAILER]:
            if has_trailer:
                return False, f"Truck {truck_id} at point {point} already has already trailer, can not do action {action}", 0, 0
            has_trailer = True
            
        if action in [PICKUP_CONTAINER, PICKUP_CONTAINER_TRAILER]:
            total = sum([_[1] for _ in current_container])
            current_container.append((request_id, requests[request_id][0]))
            if total + requests[request_id][0] > 40:
                return False, f"Truck {truck_id} at point {point} already has load {total}, can not do action {action} with demand {requests[request_id][0]}", 0, 0
                
        if action in [DROP_CONTAINER, DROP_CONTAINER_TRAILER]:
            if not any([_[0] == request_id for _ in current_container]):
                return False, f"Truck {truck_id} at point {point} has no container for request {request_id}, can not do action {action}", 0, 0
            current_container.remove((request_id, requests[request_id][0]))
            
        if action in [DROP_TRAILER, DROP_CONTAINER_TRAILER]:
            if not has_trailer:
                return False, f"Truck {truck_id} at point {point} has no trailer, can not do action {action}", 0, 0
            has_trailer = False
            
        if action == STOP:
            if has_trailer:
                return False, f"Truck {truck_id} at point {point} has trailer, can not do action {action}", 0, 0
            if len(current_container) > 0:
                return False, f"Truck {truck_id} at point {point} has container, can not do action {action}", 0, 0

        # Update times
        completion_time += distances[prev_point][point]  # Total time includes travel
        
        # Add service times to completion time only (not to travel time)
        if action in [PICKUP_TRAILER, DROP_TRAILER]:
            service_time = trailer_load_up_time
        elif action == STOP:
            service_time = 0
        else:
            service_time = requests[request_id][1] if action in [PICKUP_CONTAINER, PICKUP_CONTAINER_TRAILER] else requests[request_id][2]
        completion_time += service_time
            
        prev_point = point
        
    return True, "Correct answer", completion_time, travel_time

# output
with open("tc/6/out.txt") as f:
    sys.stdin = f
    _ = int(input().split()[-1]) 
    assert _ == num_of_routes
    completion_times = []  # For F1
    
    check = True
    message = ""
    for _ in range(num_of_routes):
        truck_id = int(input().split()[-1]) 
        lines = []
        while True:
            line = input()
            if line == '#':
                break
            lines.append(line)
            
        valid, msg, completion_time, travel_time = checkValidRouteAndCalculateScore(truck_id=truck_id, routes=lines)
        if not valid:
            check = False
            message = msg
            break
            
        completion_times.append(completion_time)
        
    if check:
        F1 = max(completion_times)  # Maximum completion time
        F2 = sum(completion_times)      # Total travel time
        F = alpha * F1 + F2         # Combined objective
        
        print(f"F1 (Max completion time) = {F1}")
        print(f"F2 (Total travel time) = {F2}")
        print(f"F = α*F1 + F2 = {alpha}*{F1} + {F2} = {F}")
        print(f"Score = {int(1e9) - F}")
    else:
        print(message)
        
# Iter: 0 Cost: 27100942 Total: 942
import os, sys, random

PATH = "./tc/7/inp.txt"

os.makedirs(os.path.dirname(PATH), exist_ok=True)

PICKUP_CONTAINER = "PICKUP_CONTAINER"
PICKUP_CONTAINER_TRAILER = "PICKUP_CONTAINER_TRAILER"
DROP_CONTAINER = "DROP_CONTAINER"
DROP_CONTAINER_TRAILER = "DROP_CONTAINER_TRAILER"
PICKUP_TRAILER = "PICKUP_TRAILER"
DROP_TRAILER = "DROP_TRAILER"
STOP = "STOP"

distance_range = (10, 100)
operation_range = (1, 10)

def get_rand(range: tuple[int, int]) -> int:
    return random.randint(range[0], range[1])

def gen(num_point: int, num_vehicles: int, num_requests: int):
    print(f"Points {num_point}")
    print(f"DISTANCES {num_point * num_point}")
    distances = [[0 for j in range(0, num_point + 1)] for i in range(0, num_point + 1)]
    
    for i in range(1, num_point + 1):
        for j in range(1, num_point + 1):
            distances[i][j] = get_rand(distance_range)
    
    print(f"TRAILER {get_rand((1, num_point))} {get_rand(operation_range)}")
    print(f"TRUCK {num_vehicles}")
    for _ in range(num_vehicles):
        print(f"{_ + 1} {get_rand((1, num_point))}")
    
    for _ in range(num_requests):
        print(f"REQ {_ + 1} {random.choice([20, 40])} {get_rand((1, num_point))} {random.choice([PICKUP_CONTAINER, PICKUP_CONTAINER_TRAILER])} {get_rand(operation_range)} {get_rand((1, num_point))} {random.choice([DROP_CONTAINER, DROP_CONTAINER_TRAILER])} {get_rand(operation_range)}")
    print("#")
    pass

with open(PATH, "w") as fw:
    sys.stdout = fw
    gen(200, 40, 400)
    
    
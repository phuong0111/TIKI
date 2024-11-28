from enum import Enum
import random
from typing import List, Dict, Set, Optional
from dataclasses import dataclass
import math
from collections import defaultdict
import sys
import time

start_time = time.time()

# Constants
MAX_POINTS = 1001
MAX_VEHICLES = 501
MAX_REQUESTS = 1001


class Action(Enum):
    PICKUP_CONTAINER = 0
    PICKUP_CONTAINER_TRAILER = 1
    DROP_CONTAINER = 2
    DROP_CONTAINER_TRAILER = 3
    PICKUP_TRAILER = 4
    DROP_TRAILER = 5
    STOP = 6


actions = [
    "PICKUP_CONTAINER",
    "PICKUP_CONTAINER_TRAILER",
    "DROP_CONTAINER",
    "DROP_CONTAINER_TRAILER",
    "PICKUP_TRAILER",
    "DROP_TRAILER",
    "STOP",
]


class ContainerSize(Enum):
    NONE = 0
    TWENTY_FT = 20
    FORTY_FT = 40


@dataclass
class Request:
    id: int
    size: ContainerSize
    pickup_point: int
    pickup_action: Action
    pickup_duration: int
    drop_point: int
    drop_action: Action
    drop_duration: int


class Route:
    def __init__(self, depot=0):
        self.depot = depot
        self.list_reqs = []
        self.cost = 0

    def size(self):
        return len(self.list_reqs)


@dataclass
class RequestContext:
    request_id: int
    transition_cost: int = 0
    route_idx: int = 0
    position: int = 0


# Global variables
distances = [[0] * MAX_POINTS for _ in range(MAX_POINTS)]
vehicle_depots = [0] * MAX_VEHICLES
current_solution = [Route() for _ in range(MAX_VEHICLES)]
requests = {}  # Dictionary instead of array for more Pythonic approach
request_contexts = {}
is_request_removed = [False] * MAX_REQUESTS
transition_cost_request_request = [[0] * MAX_REQUESTS for _ in range(MAX_REQUESTS)]
transition_cost_depot_request = [[0] * MAX_REQUESTS for _ in range(MAX_POINTS)]
transition_cost_request_depot = [[0] * MAX_POINTS for _ in range(MAX_REQUESTS)]
request_context_cost = [0] * MAX_REQUESTS


class PDPSolver:
    def __init__(
        self,
        request_idx,
        num_vehicles,
        alpha,
        trailer_point,
        trailer_pickup_time,
        max_iterations,
        verbose=False,
    ):
        self.request_idx = request_idx
        self.num_vehicles = num_vehicles
        self.alpha = alpha
        self.trailer_point = trailer_point
        self.trailer_pickup_time = trailer_pickup_time
        self.max_iterations = max_iterations
        self.max_attempt = 20
        self.verbose = verbose
        self.temperature = 100.0
        self.cooling_rate = 0.9995

        # Initialize routes
        for i in range(num_vehicles):
            current_solution[i].depot = vehicle_depots[i]

    def get_distance(self, from_point, to_point):
        return distances[from_point][to_point]

    def calculate_request_context_cost(self, request_id):
        req = requests[request_id]
        return (
            req.pickup_duration
            + req.drop_duration
            + self.get_distance(req.pickup_point, req.drop_point)
        )

    def calculate_depot_to_request_cost(self, depot, req_id):
        req = requests[req_id]
        if req.pickup_action == Action.PICKUP_CONTAINER:
            transition_cost = (
                self.get_distance(depot, self.trailer_point)
                + self.trailer_pickup_time
                + self.get_distance(self.trailer_point, req.pickup_point)
            )
        else:
            transition_cost = self.get_distance(depot, req.pickup_point)
        return transition_cost

    def calculate_request_to_depot_cost(self, req_id, depot):
        req = requests[req_id]
        if req.drop_action == Action.DROP_CONTAINER:
            transition_cost = (
                self.get_distance(req.drop_point, self.trailer_point)
                + self.trailer_pickup_time
                + self.get_distance(self.trailer_point, depot)
            )
        else:
            transition_cost = self.get_distance(req.drop_point, depot)
        return transition_cost

    def calculate_request_transition_cost(self, curr_req_id, next_req_id):
        curr_req = requests[curr_req_id]
        next_req = requests[next_req_id]
        if (
            curr_req.drop_action == Action.DROP_CONTAINER
            and next_req.pickup_action == Action.PICKUP_CONTAINER_TRAILER
        ) or (
            curr_req.drop_action == Action.DROP_CONTAINER_TRAILER
            and next_req.pickup_action == Action.PICKUP_CONTAINER
        ):
            transition_cost = (
                self.get_distance(curr_req.drop_point, self.trailer_point)
                + self.get_distance(self.trailer_point, next_req.pickup_point)
                + self.trailer_pickup_time
            )
        else:
            transition_cost = self.get_distance(
                curr_req.drop_point, next_req.pickup_point
            )
        return transition_cost

    def calculate_insertion_cost(self, route, request_id, position):
        if not route.list_reqs:
            return (
                self.calculate_depot_to_request_cost(route.depot, request_id)
                + self.calculate_request_context_cost(request_id)
                + self.calculate_request_to_depot_cost(request_id, route.depot)
            )

        if position == 0:
            next_req_id = route.list_reqs[position]
            return route.cost + (
                self.calculate_depot_to_request_cost(route.depot, request_id)
                + self.calculate_request_context_cost(request_id)
                + self.calculate_request_transition_cost(request_id, next_req_id)
                - self.calculate_depot_to_request_cost(route.depot, next_req_id)
            )
        elif position == len(route.list_reqs):
            prev_req_id = route.list_reqs[position - 1]
            return route.cost + (
                self.calculate_request_transition_cost(prev_req_id, request_id)
                + self.calculate_request_context_cost(request_id)
                + self.calculate_request_to_depot_cost(request_id, route.depot)
                - self.calculate_request_to_depot_cost(prev_req_id, route.depot)
            )
        else:
            prev_req_id = route.list_reqs[position - 1]
            next_req_id = route.list_reqs[position]
            return route.cost + (
                self.calculate_request_transition_cost(prev_req_id, request_id)
                + self.calculate_request_context_cost(request_id)
                + self.calculate_request_transition_cost(request_id, next_req_id)
                - self.calculate_request_transition_cost(prev_req_id, next_req_id)
            )

    def calculate_removal_cost(self, route, request_id):
        try:
            position = route.list_reqs.index(request_id)
        except ValueError:
            return route.cost

        cost_delta = -self.calculate_request_context_cost(request_id)

        if len(route.list_reqs) == 1:
            cost_delta -= self.calculate_depot_to_request_cost(
                route.depot, request_id
            ) + self.calculate_request_to_depot_cost(request_id, route.depot)
            return 0

        if position == 0:
            next_req_id = route.list_reqs[1]
            cost_delta -= self.calculate_depot_to_request_cost(
                route.depot, request_id
            ) + self.calculate_request_transition_cost(request_id, next_req_id)
            cost_delta += self.calculate_depot_to_request_cost(route.depot, next_req_id)
        elif position == len(route.list_reqs) - 1:
            prev_req_id = route.list_reqs[position - 1]
            cost_delta -= self.calculate_request_transition_cost(
                prev_req_id, request_id
            ) + self.calculate_request_to_depot_cost(request_id, route.depot)
            cost_delta += self.calculate_request_to_depot_cost(prev_req_id, route.depot)
        else:
            prev_req_id = route.list_reqs[position - 1]
            next_req_id = route.list_reqs[position + 1]
            cost_delta -= self.calculate_request_transition_cost(
                prev_req_id, request_id
            ) + self.calculate_request_transition_cost(request_id, next_req_id)
            cost_delta += self.calculate_request_transition_cost(
                prev_req_id, next_req_id
            )

        return route.cost + cost_delta

    def remove_stops_by_request_id(self, route, request_id):
        new_cost = self.calculate_removal_cost(route, request_id)
        route.list_reqs.remove(request_id)
        route.cost = new_cost

    def update_request_context(self, req_id, route, position):
        context = request_contexts.get(req_id, RequestContext(req_id))
        context.route_idx = current_solution.index(route)
        context.position = position

        transition_cost = 0
        if position == 0:
            transition_cost += self.calculate_depot_to_request_cost(route.depot, req_id)
            if len(route.list_reqs) > position + 1:
                next_req_id = route.list_reqs[position + 1]
                transition_cost += self.calculate_request_transition_cost(
                    req_id, next_req_id
                )
                transition_cost -= self.calculate_depot_to_request_cost(
                    route.depot, next_req_id
                )
            else:
                transition_cost += self.calculate_request_to_depot_cost(
                    req_id, route.depot
                )
        else:
            prev_req_id = route.list_reqs[position - 1]
            if position == len(route.list_reqs) - 1:
                transition_cost += self.calculate_request_transition_cost(
                    prev_req_id, req_id
                )
                transition_cost += self.calculate_request_to_depot_cost(
                    req_id, route.depot
                )
                transition_cost -= self.calculate_request_to_depot_cost(
                    prev_req_id, route.depot
                )
            else:
                next_req_id = route.list_reqs[position + 1]
                transition_cost += self.calculate_request_transition_cost(
                    prev_req_id, req_id
                )
                transition_cost += self.calculate_request_transition_cost(
                    req_id, next_req_id
                )
                transition_cost -= self.calculate_request_transition_cost(
                    prev_req_id, next_req_id
                )

        context.transition_cost = transition_cost
        request_contexts[req_id] = context

    def remove_random_requests(self, requests_to_remove, max_attempt):
        route_costs = [
            (current_solution[i].cost, i)
            for i in range(self.num_vehicles)
            if current_solution[i].list_reqs
        ]
        route_costs.sort(reverse=True)

        if not route_costs:
            return

        k = min(len(route_costs), max(2, max_attempt // 4))
        probabilities = [
            (0.8 / k if i < k else 0.2 / (len(route_costs) - k))
            for i in range(len(route_costs))
        ]

        attempt = 0
        while len(requests_to_remove) < max_attempt:
            route_idx = random.choices(range(len(route_costs)), probabilities)[0]
            route_idx = route_costs[route_idx][1]
            route = current_solution[route_idx]

            if not route.list_reqs:
                continue

            route_request_costs = []
            for pos, req_id in enumerate(route.list_reqs):
                if not is_request_removed[req_id]:
                    self.update_request_context(req_id, route, pos)
                    context = request_contexts[req_id]
                    cost = context.transition_cost
                    route_request_costs.append((cost, req_id))

            if route_request_costs:
                route_request_costs.sort(reverse=True)
                selected_idx = random.randint(0, min(2, len(route_request_costs) - 1))
                selected_request_id = route_request_costs[selected_idx][1]

                current_pos = request_contexts[selected_request_id].position
                has_prev = current_pos > 0
                has_next = current_pos < len(route.list_reqs) - 1

                if has_prev:
                    prev_request_id = route.list_reqs[current_pos - 1]

                if has_next:
                    next_request_id = route.list_reqs[current_pos + 1]

                self.remove_stops_by_request_id(route, selected_request_id)
                requests_to_remove.append(selected_request_id)
                is_request_removed[selected_request_id] = True

                if has_prev:
                    self.update_request_context(prev_request_id, route, current_pos - 1)
                if has_next:
                    self.update_request_context(next_request_id, route, current_pos)

            attempt += 1

    def insert_requests(self, request_ids):
        random_request_ids = request_ids.copy()
        random.shuffle(random_request_ids)

        for req_id in random_request_ids:
            best_cost = float("inf")
            best_route = -1
            best_position = -1

            for route_idx in range(1):
                route = current_solution[route_idx]
                for pos in range(len(route.list_reqs) + 1):
                    new_cost = self.calculate_insertion_cost(route, req_id, pos)
                    if new_cost < best_cost:
                        best_cost = new_cost
                        best_route = route_idx
                        best_position = pos

            if best_route != -1:
                route = current_solution[best_route]
                route.list_reqs.insert(best_position, req_id)
                route.cost = best_cost
                is_request_removed[req_id] = False
                self.update_request_context(req_id, route, best_position)

    def calculate_f1(self):
        return max(route.cost for route in current_solution)

    def calculate_f2(self):
        return sum(route.cost for route in current_solution)

    def calculate_solution_cost(self):
        return self.alpha * self.calculate_f1() + self.calculate_f2()

    def solve(self):

        # Initialize removal status
        for i in range(MAX_REQUESTS):
            is_request_removed[i] = True

        self.insert_requests(self.request_idx)

        current_temp = self.temperature
        current_solution_cost = self.calculate_solution_cost()
        best_solution = [Route(r.depot) for r in current_solution]
        for i, route in enumerate(current_solution):
            best_solution[i].list_reqs = route.list_reqs.copy()
            best_solution[i].cost = route.cost

        best_solution_cost = current_solution_cost
        best_total_cost = self.calculate_f2()

        for iter in range(self.max_iterations):
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 29.50:
                break

            num_to_remove = max(
                2, min(40, int(0.1 + (0.3 * random.random()) * len(self.request_idx)))
            )
            removed_requests = []
            self.remove_random_requests(removed_requests, num_to_remove)
            self.insert_requests(removed_requests)

            new_solution_cost = self.calculate_solution_cost()
            new_total_cost = self.calculate_f2()

            if new_solution_cost < best_solution_cost:
                for i, route in enumerate(current_solution):
                    best_solution[i].list_reqs = route.list_reqs.copy()
                    best_solution[i].cost = route.cost
                best_solution_cost = new_solution_cost
                best_total_cost = new_total_cost
                current_solution_cost = new_solution_cost
            elif new_solution_cost == best_solution_cost:
                if new_total_cost < best_total_cost:
                    for i, route in enumerate(current_solution):
                        best_solution[i].list_reqs = route.list_reqs.copy()
                        best_solution[i].cost = route.cost
                    best_solution_cost = new_solution_cost
                    best_total_cost = new_total_cost
                    current_solution_cost = new_solution_cost
            else:
                if current_temp > 1e-8:
                    acceptance_probability = math.exp(
                        (current_solution_cost - new_solution_cost) / current_temp
                    )
                    if random.random() < acceptance_probability:
                        current_solution_cost = new_solution_cost
                    else:
                        for i, route in enumerate(best_solution):
                            current_solution[i].list_reqs = route.list_reqs.copy()
                            current_solution[i].cost = route.cost

            # if iter % 100 == 0:
            #     print(f"{iter = } {current_solution_cost = }")

            current_temp *= self.cooling_rate
            if elapsed_time >= 29.50:
                break

        # Restore best solution
        for i, route in enumerate(best_solution):
            current_solution[i].list_reqs = route.list_reqs.copy()
            current_solution[i].cost = route.cost

    def get_solution(self):
        return current_solution


class IO:
    def __init__(self, alpha=100000, max_iterations=1000000, verbose=False):
        self.request_idx = []
        self.num_vehicles = 0
        self.trailer_point = 0
        self.trailer_pickup_time = 0
        self.alpha = alpha
        self.max_iterations = max_iterations
        self.verbose = verbose

    def get_action(self, action_str):
        action_map = {
            "PICKUP_CONTAINER": Action.PICKUP_CONTAINER,
            "DROP_CONTAINER": Action.DROP_CONTAINER,
            "PICKUP_CONTAINER_TRAILER": Action.PICKUP_CONTAINER_TRAILER,
            "DROP_CONTAINER_TRAILER": Action.DROP_CONTAINER_TRAILER,
        }
        return action_map[action_str]

    def get_container_size(self, size):
        return ContainerSize.TWENTY_FT if size == 20 else ContainerSize.FORTY_FT

    def input(self):
        # Read distance matrix
        N = int(input().split()[1])
        input()

        for _ in range(N * N):
            src, dst, dist = map(int, input().split())
            distances[src][dst] = dist

        # Read trailer info
        self.trailer_point, self.trailer_pickup_time = map(int, input().split()[1:])

        # Read vehicles
        self.num_vehicles = int(input().split()[1])
        for _ in range(self.num_vehicles):
            truck_id, truck_point = map(int, input().split())
            vehicle_depots[truck_id - 1] = truck_point

        # Read requests
        while True:
            line = input()
            if line == "#":
                break

            (
                _,
                id,
                size,
                pickup_point,
                pickup_action,
                pickup_duration,
                drop_point,
                drop_action,
                drop_duration,
            ) = line.split()
            request = Request(
                id=int(id),
                size=self.get_container_size(int(size)),
                pickup_point=int(pickup_point),
                pickup_action=self.get_action(pickup_action),
                pickup_duration=int(pickup_duration),
                drop_point=int(drop_point),
                drop_action=self.get_action(drop_action),
                drop_duration=int(drop_duration),
            )
            requests[int(id)] = request
            self.request_idx.append(int(id))

    def output_route(self, route):
        if not route.list_reqs:
            print(f"{route.depot} STOP")
            print("#")
            return

        prev_point = route.depot
        for i, req_id in enumerate(route.list_reqs):
            req = requests[req_id]

            # Handle trailer for pickup
            if req.pickup_action == Action.PICKUP_CONTAINER and (
                i == 0
                or requests[route.list_reqs[i - 1]].drop_action
                == Action.DROP_CONTAINER_TRAILER
            ):
                print(f"{self.trailer_point} {actions[Action.PICKUP_TRAILER.value]}")
                prev_point = self.trailer_point

            # Output pickup action
            print(f"{req.pickup_point} {actions[req.pickup_action.value]} {req.id}")
            prev_point = req.pickup_point

            # Output drop action
            print(f"{req.drop_point} {actions[req.drop_action.value]} {req.id}")
            prev_point = req.drop_point

            # Handle trailer for drop
            if req.drop_action == Action.DROP_CONTAINER and (
                i == len(route.list_reqs) - 1
                or requests[route.list_reqs[i + 1]].pickup_action
                == Action.PICKUP_CONTAINER_TRAILER
            ):
                print(f"{self.trailer_point} {actions[Action.DROP_TRAILER.value]}")
                prev_point = self.trailer_point

        print(f"{route.depot} STOP")
        print("#")

    def output(self):
        solver = PDPSolver(
            self.request_idx,
            self.num_vehicles,
            self.alpha,
            self.trailer_point,
            self.trailer_pickup_time,
            self.max_iterations,
            self.verbose,
        )

        solver.solve()
        solution = solver.get_solution()

        print(f"ROUTES {self.num_vehicles}")
        for i in range(self.num_vehicles):
            print(f"TRUCK {i + 1}")
            self.output_route(solution[i])


def main():
    io = IO(alpha=100000, max_iterations=10000000, verbose=0)
    import sys

    start_time = time.time()
    # with open("tc/6/inp.txt", "r") as f:
    #     sys.stdin = f
    #     io.input()
    io.input()
    # with open("tc/1/out.txt", "w") as f:
    #     sys.stdout = f
    #     io.output()
    io.output()


if __name__ == "__main__":
    main()

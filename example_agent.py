from typing import override

from aegis import (
    CONNECT_OK,
    END_TURN,
    SEND_MESSAGE_RESULT,
    MOVE,
    MOVE_RESULT,
    OBSERVE_RESULT,
    PREDICT_RESULT,
    SAVE_SURV,
    SAVE_SURV_RESULT,
    SEND_MESSAGE,
    SLEEP_RESULT,
    TEAM_DIG,
    TEAM_DIG_RESULT,
    AgentCommand,
    AgentIDList,
    Direction,
    Rubble,
    SurroundInfo,
    Survivor,
)
from agent import BaseAgent, Brain, LogLevels
from aegis.common.world.cell import Cell
from aegis.common import Location
from aegis.common import AgentID
import heapq


class ExampleAgent(Brain):
    def __init__(self) -> None:
        super().__init__()
        self._agent: BaseAgent = BaseAgent.get_base_agent()
        self.sent_initial_message = False  # Track if "initial" message has been sent


    @override
    def handle_connect_ok(self, connect_ok: CONNECT_OK) -> None:
        BaseAgent.log(LogLevels.Always, "CONNECT_OK")

    @override
    def handle_disconnect(self) -> None:
        BaseAgent.log(LogLevels.Always, "DISCONNECT")

    @override
    def handle_dead(self) -> None:
        BaseAgent.log(LogLevels.Always, "DEAD")

    @override
    def handle_send_message_result(self, smr: SEND_MESSAGE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"Received message: {smr.message}")

        if smr.message == "helpDig":
            # Respond to helpDig with helpDidComing
            self._agent.send(
                SEND_MESSAGE(
                    AgentIDList(), "helpDidComing"
                )
            )

    @override
    def handle_move_result(self, mr: MOVE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"MOVE_RESULT: {mr}")
        BaseAgent.log(LogLevels.Test, f"{mr}")
        print("#--- You need to implement handle_move_result function! ---#")

    @override
    def handle_observe_result(self, ovr: OBSERVE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"OBSERVER_RESULT: {ovr}")
        BaseAgent.log(LogLevels.Test, f"{ovr}")
        print("#--- You need to implement handle_observe_result function! ---#")

    @override
    def handle_save_surv_result(self, ssr: SAVE_SURV_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"SAVE_SURV_RESULT: {ssr}")
        BaseAgent.log(LogLevels.Test, f"{ssr}")
        print("#--- You need to implement handle_save_surv_result function! ---#")

    @override
    def handle_predict_result(self, prd: PREDICT_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"PREDICT_RESULT: {prd}")
        BaseAgent.log(LogLevels.Test, f"{prd}")

    @override
    def handle_sleep_result(self, sr: SLEEP_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"SLEEP_RESULT: {sr}")
        BaseAgent.log(LogLevels.Test, f"{sr}")
        print("#--- You need to implement handle_sleep_result function! ---#")

    @override
    def handle_team_dig_result(self, tdr: TEAM_DIG_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"TEAM_DIG_RSULT: {tdr}")
        BaseAgent.log(LogLevels.Test, f"{tdr}")
        print("#--- You need to implement handle_team_dig_result function! ---#")

    @override
    def handle_send_message_result(self, smr: SEND_MESSAGE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"Message received: {smr.msg}")

        if smr.msg.startswith("LOC_UPDATE:"):
            # Parse the location update message
            try:
                _, content = smr.msg.split(":")
                sender_id, location = content.split("@")
                BaseAgent.log(LogLevels.Always, f"Agent {sender_id} is at {location}")
                
                # Process the location (e.g., assign tasks, update internal state)
                self.assign_task_to_agent(sender_id, location)
            except ValueError:
                BaseAgent.log(LogLevels.Error, "Malformed LOC_UPDATE message received!")

    #This heuristic method below is from under the "Heuristic search" section:
    #https://www.redblobgames.com/pathfinding/a-star/introduction.html
    @classmethod
    def heuristic(cls, a:Location, b:Location):
        # Manhattan distance
        result =  abs(a.x - b.x) + abs (a.y - b.y)
        return result

    @override
    def think(self) -> None:
        BaseAgent.log(LogLevels.Always, "Thinking")

        # Retrieve the current state of the world
        world = self.get_world()
        if world is None:
            self.send_and_end_turn(MOVE(Direction.CENTER))
            return

        # Step 1: Send location update to the team leader (Agent ID 1)
        team_leader_id = 1  # Assuming Agent ID 1 is the leader
        current_location = self._agent.get_location()
        location_update_message = f"LOC_UPDATE:{self._agent.get_agent_id().id}@{current_location}"

        # Send the location update message
        self._agent.send(
            SEND_MESSAGE(
                AgentIDList([AgentID(team_leader_id, self._agent.get_agent_id().gid)]),
                location_update_message
            )
        )
        BaseAgent.log(LogLevels.Always, f"Sent location update to Agent {team_leader_id}: {location_update_message}")

        # Step 2: Fetch the current cell
        cell = world.get_cell_at(current_location)
        if cell is None:
            self.send_and_end_turn(MOVE(Direction.CENTER))
            return

        # Step 3: Check for survivors or rubble and take action
        top_layer = cell.get_top_layer()
        if top_layer:
            if isinstance(top_layer, Survivor):
                self.send_and_end_turn(SAVE_SURV())
                return
            elif isinstance(top_layer, Rubble):
                self.send_and_end_turn(TEAM_DIG())
                return

        # Step 4: Pathfinding logic to locate the goal
        goal = None
        for h in range(world.height):
            for w in range(world.width):
                locationS = Location(w, h)
                survCell = world.get_cell_at(locationS)
                if survCell and survCell.survivor_chance != 0:
                    goal = survCell
                    break
            if goal:
                break

        if not goal:
            # No goal found; take a default action
            self.send_and_end_turn(MOVE(Direction.CENTER))
            return

        # Step 5: A* Pathfinding to the goal
        frontier = []
        start = current_location
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal.location:
                # Reconstruct the path to the goal
                path = []
                while current != start:
                    path.append(current)
                    current = came_from.get(current)
                    if current is None:
                        BaseAgent.log(LogLevels.Error, "Path reconstruction failed. No valid path to goal.")
                        self.send_and_end_turn(MOVE(Direction.CENTER))
                        return
                path.reverse()

                # Move along the path
                if len(path) > 1:
                    dx = path[1].x - path[0].x
                    dy = path[1].y - path[0].y
                    self.send_and_end_turn(MOVE(Direction(dx, dy)))
                else:
                    self.send_and_end_turn(MOVE(Direction.CENTER))
                return

            # Explore neighbors
            for direct in Direction:
                neighbor = Location(current.x + direct.dx, current.y + direct.dy)
                if not world.on_map(neighbor):
                    continue

                next_cell = world.get_cell_at(neighbor)
                if not next_cell or not next_cell.is_normal_cell():
                    continue

                new_cost = cost_so_far[current] + next_cell.move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + ExampleAgent.heuristic(goal.location, neighbor)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        # Step 6: Default fallback action
        self.send_and_end_turn(MOVE(Direction.CENTER))

    def send_and_end_turn(self, command: AgentCommand):
        """Send a command and end your turn."""
        BaseAgent.log(LogLevels.Always, f"SENDING {command}")
        self._agent.send(command)
        self._agent.send(END_TURN())

    def update_surround(self, surround_info: SurroundInfo):
        """
        Updates the current and surrounding cells of the agent.

        You can check assignment 1 to figure out how to use this function.
        """
        world = self.get_world()
        if world is None:
            return

        for dir in Direction:
            cell_info = surround_info.get_surround_info(dir)
            if cell_info is None:
                continue

            cell = world.get_cell_at(cell_info.location)
            if cell is None:
                continue

            cell.move_cost = cell_info.move_cost
            cell.set_top_layer(cell_info.top_layer)
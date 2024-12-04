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
import json

class ExampleAgent(Brain):
    def __init__(self) -> None:
        super().__init__()
        self._agent: BaseAgent = BaseAgent.get_base_agent()
        self.sent_initial_message = False  # Track if "initial" message has been sent
        self.leader = False
        self.leader_processing_locations = False
        self.should_send_location = True
        self.agent_locations = []
        self.goal_location = None
        self.get_help_digging = False
    
    # compute the distance between agent and all surviors
    def find_survivors(self):
        world = self.get_world()
        survivors= []
        for h in range(world.height):
            for w in range(world.width):
                locationS = Location(w, h)
                survCell = world.get_cell_at(locationS)
                if survCell and survCell.survivor_chance != 0:
                    survivors.append(locationS)
        print(f"SURVIVORS: {survivors}")
        return survivors

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
    def handle_move_result(self, mr: MOVE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"MOVE_RESULT: {mr}")
        BaseAgent.log(LogLevels.Test, f"{mr}")
        self.update_surround(mr.surround_info)

    @override
    def handle_observe_result(self, ovr: OBSERVE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"OBSERVER_RESULT: {ovr}")
        BaseAgent.log(LogLevels.Test, f"{ovr}")
        self.update_surround(ovr.surround_info)

    @override
    def handle_save_surv_result(self, ssr: SAVE_SURV_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"SAVE_SURV_RESULT: {ssr}")
        BaseAgent.log(LogLevels.Test, f"{ssr}")
        self.update_surround(ssr.surround_info)

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
        self.get_help_digging = True
     

        self.update_surround(tdr.surround_info)

    @override
    def handle_send_message_result(self, smr: SEND_MESSAGE_RESULT) -> None:
        BaseAgent.log(LogLevels.Always, f"Message received - FROM: {smr.from_agent_id} MSG: {smr.msg}")

        if smr.msg.startswith("ASSIGN:"):
            content = smr.msg.split(":")
            agent_id = int(content[1])
            agent_gid = int(content[2])
            x = int(content[3])
            y = int(content[4])
            survivor_location = Location(x,y)

            msg = f"MOVE:{x}:{y}"
            self._agent.send(
                SEND_MESSAGE(
                    AgentIDList([AgentID(agent_id, agent_gid)]),
                    msg
                )
            )
            self.agent_locations.append((smr.from_agent_id, Location(x,y)))

        if smr.msg.startswith("MOVE:"):
            content = smr.msg.split(":")
            x = int(content[1])
            y = int(content[2])

            self.goal_location = Location(x,y)
            self.leader_processing_locations = False

        if smr.msg.startswith("LOC_UPDATE:"):
            # Parse the location update message
            content = smr.msg.split(":")
            x = int(content[2])
            y = int(content[4])
            self.agent_locations.append((smr.from_agent_id, Location(x,y)))


        if smr.msg.startswith("HELP_DIG"):
            BaseAgent.log(LogLevels.Always, f"Message received - HELP DIG")
            # Respond to helpDig with helpDidComing
            content = smr.msg.split(":")
            x = int(content[1])
            y = int(content[2])
            self.goal_location = Location(x,y)
                
    #This heuristic method below is from under the "Heuristic search" section: #https://www.redblobgames.com/pathfinding/a-star/introduction.html
    @classmethod
    def heuristic(cls, a:Location, b:Location):
        # Manhattan distance
        result =  abs(a.x - b.x) + abs (a.y - b.y)
        return result

    @override
    def think(self) -> None:
        BaseAgent.log(LogLevels.Always, "Thinking")
        team_leader_id = 1  # Assuming Agent ID 1 is the leader
        current_location = self._agent.get_location()
          
        # Retrieve the current state of the world
        world = self.get_world()

        # Step 1: Send location update to the team leader (Agent ID 1)
        if self.should_send_location:
            location_update_message = f"LOC_UPDATE:x:{current_location.x}:y:{current_location.y}"

            # Send the location update message
            self._agent.send(
                SEND_MESSAGE(
                    AgentIDList([AgentID(team_leader_id, 1)]),
                    location_update_message
                )
            )

            BaseAgent.log(LogLevels.Always, f"Sent location update to Agent {team_leader_id}: {location_update_message}")
            self.should_send_location = False
            self.leader_processing_locations = True
            self.send_and_end_turn(MOVE(Direction.CENTER))

            # Mark agent as the leader
            self.leader = True if self._agent.get_agent_id().id == 1 and self._agent.get_agent_id().gid == 1 else False 
            return

        # Assign each agent a survivor
        if self.leader and self.leader_processing_locations:
            survivor_locations = self.find_survivors()
            assignments = self.assign_agents_to_survivors(self.agent_locations, survivor_locations)
            print(f"ASSIGNMENTS: {assignments}")

            # Send the location update message
            for agent in assignments:
                msg = f"ASSIGN:{agent.id}:{agent.gid}:{assignments[agent]}"
                self._agent.send(
                    SEND_MESSAGE(
                        AgentIDList([AgentID(team_leader_id, 1)]),
                        msg
                    )
                )
            self.send_and_end_turn(MOVE(Direction.CENTER))  
            return


        if self.leader_processing_locations:
            self.send_and_end_turn(MOVE(Direction.CENTER))
            return


        # Step 2: Fetch the current cell
        cell = world.get_cell_at(current_location)


        # Step 3: Check for survivors or rubble and take action
        if self.get_help_digging:
            msg = f"HELP_DIG:{current_location.x}:{current_location.y}"
            self._agent.send(
                    SEND_MESSAGE(
                        AgentIDList(),
                        msg
                    )
                )
            self.get_help_digging = False

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
        survCell = world.get_cell_at(self.goal_location)
        if survCell and survCell.survivor_chance != 0:
            # print(world.get_cell_at(locationS))
            goal = survCell
        if goal:
            print(f"goal at location ({self.goal_location.x}, {self.goal_location.y}) with chance: {goal.survivor_chance}")

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
                if len(path) > 0:
                    dx = path[0].x - current_location.x
                    dy = path[0].y - current_location.y
                    self.send_and_end_turn(MOVE(Direction(dx, dy)))
                    return
                else:
                    self.send_and_end_turn(MOVE(Direction.CENTER))

                if len(path) < 2 :
                    BaseAgent.log(LogLevels.Warning, "Path too short. Agent likely at the goal.")
                    self.send_and_end_turn(SAVE_SURV())  # Try saving if at the goal
                    goal.survivor_chance = 0
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

    def assign_agents_to_survivors(self, agents, survivors):
        assignments = {}  # Initialize assignments
        assigned_survivors = set()

        for agent in agents:
            min_distance = float('inf')
            assigned_survivor_idx = None
            assigned_survivor_location = None

            for survivor_idx, survivor in enumerate(survivors):
                if survivor_idx in assigned_survivors:
                    continue  # Skip already assigned survivors

                # Calculate Manhattan distance
                distance = ExampleAgent.heuristic(agent[1], survivor)
                # Assign the closest unassigned survivor
                if distance < min_distance:
                    min_distance = distance
                    assigned_survivor_idx = survivor_idx
                    assigned_survivor_location = survivor

            # Assign the survivor to the agent if found
            if assigned_survivor_idx is not None:
                print(f"ASSIGNING SURVIVOR: {assigned_survivor_idx}")
                assignments[agent[0]] = f"{assigned_survivor_location.x}:{assigned_survivor_location.y}"
                assigned_survivors.add(assigned_survivor_idx)

        return assignments
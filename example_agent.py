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
import heapq


class ExampleAgent(Brain):
    def __init__(self) -> None:
        super().__init__()
        self._agent: BaseAgent = BaseAgent.get_base_agent()

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
        BaseAgent.log(LogLevels.Always, f"SEND_MESSAGE_RESULT: {smr}")
        BaseAgent.log(LogLevels.Test, f"{smr}")
        print("#--- You need to implement handle_send_message_result function! ---#")

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

        # Send a message to other agents in my group.
        # Empty AgentIDList will send to group members.
        self._agent.send(
            SEND_MESSAGE(
                AgentIDList(), f"Hello from agent {self._agent.get_agent_id().id}"
            )
        )

        # Retrieve the current state of the world.
        world = self.get_world()
        if world is None:
            self.send_and_end_turn(MOVE(Direction.CENTER))
            return

        # Fetch the cell at the agent’s current location. If the location is outside the world’s bounds,
        # return a default move action and end the turn.
        cell = world.get_cell_at(self._agent.get_location())
        if cell is None:
            self.send_and_end_turn(MOVE(Direction.CENTER))
            return
    

        # Get the top layer at the agent’s current location.
        # If a survivor is present, save it and end the turn.
        top_layer = cell.get_top_layer()
        if top_layer:
            self.send_and_end_turn(SAVE_SURV())
            return

        # Initialize the goal cell to be used later
        goal = Cell ()
        for h in range(world.height):   # We iterate through two loops through the world size
            for w in range(world.width):
                locationS = Location (w, h) 
                survCell = world.get_cell_at(locationS) # Initialize a location
                if survCell.survivor_chance != 0:    # If this is the survivor cell (percent_chance is not 0,
                    goal = survCell                 # then make the goal equal to this cell)


        # The A* algorithm below is based from the pseudocode under "The A* algorithm" section: 
        # https://www.redblobgames.com/pathfinding/a-star/introduction.html
        frontier = []
        start = self._agent.get_location()      # Initialize the start location
        heapq.heappush(frontier, (0, start))    # Heap push in the frontier list and (priority, location)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None                 # initial values into came_from and cost_so_far 
        cost_so_far[start] = 0

        # While the frontier has something in it
        while frontier:
            prior, current = heapq.heappop(frontier)  # Put the higher priority in our frontier to current

            if current == goal.location:    # If current is at the goal location, we can break out of the while loop
                # This part below is from the reconstruct paths part in the "Breadth First Search" part:
                # https://www.redblobgames.com/pathfinding/a-star/introduction.html
                current = goal.location
                path = []
                while current != start:     # Creating the path list needed for the agent to move
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                first_location = path[0]    # Initialize the first and second locations with the 0th and 1st path   
                second_location = path[1]
                dx = second_location.x - first_location.x   
                dy = second_location.y - first_location.y
                mov_dir = Direction(dx, dy)  # Get the x and y direction by subtracting the second to the first location, then give this value to movDir

                self.send_and_end_turn(MOVE(mov_dir))    # Send the agent to the resulting movDir (Direction)
                break
            
            # Loop through all 9 directions around the current cell
            for direct in Direction:
                direct_location = Location(0, 0)             # Initialize the neighbor location 
                direct_location.x = current.x + direct.dx    # Then find the coordinates for the selected neighbour
                direct_location.y = current.y + direct.dy

                # If the direction location is not outside of bounds, then proceed
                if ((world.width-1 >=direct_location.x >= 0) and (world.height-1 >=direct_location.y >= 0)):
                    next_cost = world.get_cell_at(direct_location).move_cost # get the move cost at the neighbouring location
                    new_cost = cost_so_far[current] + next_cost      # Add the value in cost_so_far with the value of the neighbour location

                    # Handles if it is out of energy, we continue to the next iteration of the loop
                    if world.get_cell_at(direct_location).move_cost >= self._agent.get_energy_level():
                        continue
                    # Initialize the cell at the neighbour location
                    next_cell = world.get_cell_at(direct_location)      
                    # If the neighbour's location has not been accounted for, or the new_cost is less than the cost so far of neighbor
                    # And the neighboring cell is stable, we can proceed
                    if (direct_location not in cost_so_far or new_cost < cost_so_far[direct_location]) and next_cell.is_normal_cell():             
                        cost_so_far[direct_location] = new_cost         # Cost so far in the neighboring location is the new_cost
                        priority = new_cost + ExampleAgent.heuristic(goal.location, direct_location)    #Calculate priority by adding the new cost with the heuristic
                        heapq.heappush(frontier, (priority, direct_location))   # Push into the frontier the (priority, neighbour location)
                        came_from[direct_location] = current        # Came from allows us to know the path of where our agent should end

        # If a survivor is present, save it and end the turn.
        if isinstance(top_layer, Survivor):
            self.send_and_end_turn(SAVE_SURV())
            return

        # If rubble is present, clear it and end the turn.
        if isinstance(top_layer, Rubble):
            self.send_and_end_turn(TEAM_DIG())
            return

        # Default action: Move the agent north if no other specific conditions are met.
        self.send_and_end_turn(MOVE(Direction.NORTH))

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

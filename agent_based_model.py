"""
Assignment 4: Agent-Based Model for Structural Tessellation Generation

Author: Boas Olesen

Description:
This script implements an agent-based model using Object-Oriented Programming (OOP) principles.
It simulates the behavior of shoppers (agents) to generate flow patterns, exploring how changes in
rules and parameters affect the resulting flow in a pedestrian patterns.

Note: This script is being developed within Grasshopper's Python scripting component, 
meaning this is the main component, which other helper components should go into.
"""

# Import necessary libraries
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import random
import ghpythonlib.treehelpers

"""
### Inputs
Agent: Agent function
srf: Boundary surface
start_position: Entry point for the agent
steps: Number of steps in the simulation
max_speed: Maximum Speed for the agent
slow_radius: Proximity for the agent to the target to slow down
"""

# Agent import
class Agent:
    def __init__(self, srf, start_position):
        self.position = rg.Point3d(*start_position) # to ensure correct datatype
        self.velocity = rg.Vector3d(0, 0, 0) # to ensure correct datatype
        self.srf = srf
        self.avoidance_direction = None  # Keeps track of the current direction to go around obstacle

    def dist(self, target_point):
        """
        Calculate distance between between agent & target
        """
        target = rg.Point3d(*target_point) # to ensure correct datatype
        return self.position.DistanceTo(target)

    def move(self):
        "Move & update the agent's position"
        # Update position
        self.position += self.velocity

    def seek_target(self, current_target, max_speed, slow_radius):
        """
        Adjust velocity to seek target
        """
        target_point = rg.Point3d(*current_target)
        target_vector = target_point - self.position  # Vector pointing to the target
        distance = target_vector.Length  # Distance to the target

        if distance > 0:
            target_vector.Unitize()  # Normalize
            desired_velocity = target_vector * max_speed
            if distance < slow_radius:
                # Slow down as it approaches the target
                desired_velocity *= distance / slow_radius
            self.velocity = desired_velocity
    
    def avoid_obstacles(self, obstacles, avoidance_range, avoidance_factor, boost_factor, side_push_factor, min_speed=0.1):
        """
        Avoid obstacles by repulsion and following obstacles edge
        """
        # Initialize cumulative repulsion and tangential force vectors
        repulsion = rg.Vector3d(0, 0, 0)
        tangential_force = rg.Vector3d(0, 0, 0)

        # Project the Agent onto XY plane
        agent_position_2d = rg.Point3d(self.position.X, self.position.Y, 0)

        # Extended range for direction reset
        extended_range = avoidance_range * 1.2  # To ensure the agent follows the edge of the obstacle

        # Ensure obstacle is an iterable list
        if not isinstance(obstacles, list):
            obstacles =[obstacles]

        # Track wether the agent is near an obstacle
        within_range = False

        for obstacle in obstacles:
            # Determine closest point on the obstacle to the agent's position
            closest_point = None

            try:
                if isinstance(obstacle, rg.Rectangle3d):
                    # Convert rectangle to a curve and find the closest point
                    curve = obstacle.ToNurbsCurve()
                    success, param = curve.ClosestPoint(agent_position_2d)
                    if success:
                        closest_point = curve.PointAt(param)  # Retrieve Point3d from parameter
                            
                elif isinstance(obstacle, rg.Curve):
                    # For general curves
                    success, param = obstacle.ClosestPoint(agent_position_2d)
                    if success:
                        closest_point = obstacle.PointAt(param)  # Use 'param' to get the closest point

                # Project the closest point onto the XY plane
                if closest_point:
                    closest_point_2d = rg.Point3d(closest_point.X, closest_point.Y, 0)

                # Calculate distance in the XY plane
                distance = agent_position_2d.DistanceTo(closest_point_2d)

                # if within the avoidance_range             
                if distance < avoidance_range:
                    # The agent is within the avoidance range of an obstacle
                    within_range = True

                    # Calculate repulsion vector
                    repulsion_vector = rg.Vector3d(agent_position_2d - closest_point_2d)
                    repulsion_vector.Unitize()

                    # Apply non-linear scaling for stronger push closer to obstacle
                    repulsion += repulsion_vector * (avoidance_factor / (distance + 0.01) + boost_factor)

                    # Determine the tangential force directio, the direction to go around obstacles and keeps it while within range of the obstacle
                    if self.avoidance_direction is None: # checks if a direction has been chosen to go around the obstacle
                        self.avoidance_direction = random.choice([1, -1])   # Choses randomly between -1 & 1 (left or right)

                    # Apply tangential force in the chosen direction
                    tangent_weight = max(0.5, (avoidance_range - distance) / avoidance_range)  # Stronger closer to obstacle
                    tangential_force += self.avoidance_direction * rg.Vector3d(repulsion_vector.Y, -repulsion_vector.X, 0) * side_push_factor * tangent_weight

            except Exception as e: # Debugging
                print(f"Error processing obstacle {obstacle}: {e}")
        
        # Reset Direction when not in obstacle avoidance range
        if not within_range:
            distances_to_obstacles = [
                agent_position_2d.DistanceTo(rg.Point3d(obstacle.GetBoundingBox(True).ClosestPoint(agent_position_2d)))
                for obstacle in obstacles
            ]
            if all(distance > extended_range for distance in distances_to_obstacles): # Checks if the agent still is within the extended avoidance range of the obstacle 
                self.avoidance_direction = None # resets the left/right direction of the agent
            
        # Combine repulsion and tangential forces
        total_avoidance = repulsion + tangential_force

        # Add forward momentum to encourage progression
        forward_boost = rg.Vector3d(self.velocity)
        forward_boost.Unitize()
        forward_boost *= 0.1  # Adjust forward boost strength as needed

        # add the 2D avoidance force and forward boost to the current velocity
        self.velocity += total_avoidance + forward_boost

        # Ensure minimum speed to prevent agents getting stuck
        if self.velocity.Length < min_speed:
            self.velocity.Unitize()
            self.velocity *= min_speed

        # Ensure maximum speed
        if self.velocity.Length > max_speed:
            self.velocity.Unitize()
            self.velocity *= max_speed

"""
Simulate agent seeking the tartets
### simulation Inputs
srf: Boundary surface
start_position: Entry point for the agent
targets: the targets which the agent shall go to
steps: Number of steps in the simulation
max_speed: Maximum travel distance for the agent per step
slow_radius: Proximity for the agent to the target to slow down
obstacles: geometries to be avoided/walked around
avoidance_range: distance from obstacles to avoid
avoidance_factor: Repulsion force factor
boost_factor: A factor to help the agent from stalling
side_push_factor: a factor to help push the agent around obstacles
"""
# Flatten obstacles from input
obstacles_flatten = []
if isinstance(obstacles, list):  # Ensures it's a list of obstacles
    for obs in obstacles:
        if isinstance(obs, rg.Rectangle3d):
            obstacles_flatten.append(obs.ToNurbsCurve())  # Convert to a Curve
        else:
            obstacles_flatten.append(obs)  # Add directly if already a Curve
else:
    # If it's a single obstacle, convert it into a list
    if isinstance(obstacles, rg.Rectangle3d):
        obstacles_flatten.append(obstacles.ToNurbsCurve())  # Convert to Curve
    else:
        obstacles_flatten.append(obstacles)  # Add directly if already a Curve

# Debug output to confirm
# print(f"Flattened obstacles: {obstacles_flatten}")

# Determine how many agents to assign per start position
agents_per_start = [agents_number // len(start_positions)] * len(start_positions)

# Distribute any remaining agents randomly among start positions
remaining_agents = agents_number % len(start_positions)
for _ in range(remaining_agents):
    random_index = random.randint(0, len(start_positions) - 1)
    agents_per_start[random_index] += 1

# Initialize agents
agents = []  # List to store all agents
agent_id = 1  # To assign unique IDs for debugging
for start_pos, num_agents in zip(start_positions, agents_per_start):
    for _ in range(num_agents):
        # Randomly assign targets to each agent
        agent_targets = random.sample(targets, num_targets_per_agent)
        agent_targets.append(start_pos)  # Add the starting position as the final target (to loop back)

        # Create an agent instance
        agent = {
            "id": agent_id,  # Unique identifier for debugging
            "instance": Agent(srf, start_pos),
            "targets": agent_targets,
            "current_target_index": 0,  # Start with the first target
            "lines": [],  # For path visualization
            "points": [],  # For point visualization
            "prev_position": start_pos, # For path tracing
            "start_position": start_pos, # store the start position
            "finished": False, # Flag to indicate wether the agent has returned to start position
        }
        agents.append(agent)
        agent_id += 1

# Debug: Output assigned targets for each agent
for agent in agents:
    print(f"Agent {agent['id']} targets: {agent['targets']}")

# A flat list of all lines in order
lines = []
nested_lines = [[] for _ in range(steps)]  # Initialize a nested list for all steps

# Process each step in the simulation
for step in range(steps):
    step_lines = [] # Lines for this step
    for agent_data in agents[:]:  # Iterate over a copy of the agents list

        # If agent is finished, skip it
        if agent_data["finished"]:
            continue

        agent = agent_data["instance"]
        targets = agent_data["targets"]
        current_target_index = agent_data["current_target_index"]
        start_position = agent_data["start_position"]

        # Ensure agent starts moving if at the start position
        if current_target_index == 0 and agent.dist(start_position) < 1:
            if len(targets) > 0 and agent.dist(targets[0]) > 1:
                agent_data["current_target_index"] = 0
            else:
                agent_data["current_target_index"] = 1

        # Handle movement toward targets
        if current_target_index < len(targets):
            current_target = targets[current_target_index]
            agent.seek_target(current_target, max_speed, slow_radius)
            agent.avoid_obstacles(
                obstacles_flatten, avoidance_range, avoidance_factor, boost_factor, side_push_factor
            )
            agent.move()

            # Create a line only if moving
            if agent.dist(agent_data["prev_position"]) > 0:
                movement_line = rg.Line(agent_data["prev_position"], agent.position)
                lines.append(movement_line)
                agent_data["lines"].append(movement_line)
                step_lines.append(rg.LineCurve(movement_line))  # Add for this step
                agent_data["prev_position"] = agent.position

            # Stop if agent reaches the target
            if agent.dist(current_target) < 1:
                agent_data["current_target_index"] += 1

        else:
            # Move back to start position
            if agent.dist(start_position) > 1:
                agent.seek_target(start_position, max_speed, slow_radius)
                agent.move()

                # Create a line for movement back to start
                if agent.dist(agent_data["prev_position"]) > 0:
                    movement_line = rg.Line(agent_data["prev_position"], agent.position)
                    lines.append(movement_line)
                    agent_data["lines"].append(movement_line)
                    step_lines.append(rg.LineCurve(movement_line))
                    agent_data["prev_position"] = agent.position

            # Mark agent as finished if it reaches the start
            if agent.dist(start_position) < 1:
                agent_data["finished"] = True
                print(f"Agent {agent_data['id']} has finished its task.")
    
    # Store this step's lines in nested structure
    nested_lines[step] = step_lines

    # Debugging step lines
    print(f"Step {step + 1}: {len(step_lines)} lines generated.")
        
    # Check if all agents are done
    all_agents_done = all(agent_data["finished"] for agent_data in agents)
    if all_agents_done:
        print("All agents have completed their tasks. Ending simulation.")
        break

# Visualization for each step
steps_visualization = ghpythonlib.treehelpers.list_to_tree(nested_lines)

# Debug final nested_lines
for step, step_lines in enumerate(nested_lines):
    print(f"Step {step + 1}: {len(step_lines)} lines")
    for i, line in enumerate(step_lines):
        print(f"  Line {i + 1}: {line}")

# Output to Grasshopper
nested_lines = steps_visualization
print("Simulation complete.")
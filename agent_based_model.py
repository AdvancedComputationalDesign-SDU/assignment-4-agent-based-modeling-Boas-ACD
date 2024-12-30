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
import Rhino
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg

# If using Rhino/Grasshopper for visualization
# import Rhino
# import Rhino.Geometry as rg

# Empty Lists
Lines = []
Points = []

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

    def seek_target(self, target, max_speed, slow_radius):
        """
        Adjust velocity to seek target
        """
        target_point = rg.Point3d(*target)
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
        Avoid obstacles
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
        print(f"Step {step}: Repulsion: {repulsion}, Velocity: {self.velocity}")

        # Ensure minimum speed to prevent agents getting stuck
        if self.velocity.Length < min_speed:
            self.velocity.Unitize()
            self.velocity *= min_speed

        # Ensure maximum speed
        if self.velocity.Length > max_speed:
            self.velocity.Unitize()
            self.velocity *= max_speed

         # Optional: Print debug to ensure repulsion is accumulating from all obstacles
        print(f"Repulsion: {repulsion}, Tangential Force: {tangential_force}, Velocity: {self.velocity}")

# Simulate agent seeking the goal
prev_position = agent.position  # Store previous position for path tracing
for step in range(steps):  # steps in the simulation
    agent.seek_target(target, max_speed, slow_radius)
    agent.move()
    # print(f"Step {step}: Agent position: {agent.position}, Distance to goal: {agent.dist(target)}")

    # Visualize the agent's movement
    Lines.append(rg.Line(prev_position, agent.position))
    Points.append(agent.position)
    prev_position = agent.position

    # Stop if the agent is close enough to the goal
    if agent.dist(target) < 0.1:
        print("target reached!")
        break

# Empty Lists
Lines = []
Points = []

print(f"Obstacles input: {obstacles}")

"""
### simulation Inputs
srf: Boundary surface
start_position: Entry point for the agent
target: the target which the agent shall reach
steps: Number of steps in the simulation
max_speed: Maximum Speed for the agent
slow_radius: Proximity for the agent to the target to slow down
obstacles: geometries to be avoided
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
print(f"Flattened obstacles: {obstacles_flatten}")
# Agent import
agent = Agent(srf, start_position)

# Simulate agent seeking the goal
prev_position = agent.position  # Store previous position for path tracing
for step in range(steps):  # steps in the simulation 
    # Seek Target
    agent.seek_target(target, max_speed, slow_radius)

    # Avoid Obstacles
    agent.avoid_obstacles(obstacles_flatten, avoidance_range, avoidance_factor, boost_factor, side_push_factor)

    # Move Agent
    agent.move()
    

    # Visualize the agent's movement
    Lines.append(rg.Line(prev_position, agent.position))
    Points.append(agent.position)
    prev_position = agent.position

    # Stop if the agent is close enough to the target
    if agent.dist(target) < 1:
        print("target reached!")
        break

# Debug print statements
print("Simulation complete.")
print(f"Final Agent Position: {agent.position}")

# Define additional classes if needed (e.g., Environment, Obstacle)

# # Simulation parameters
# num_agents = 100  # Number of agents
# num_steps = 100   # Number of simulation steps
# agents = []       # List to hold agent instances

# # Initialize agents
# for i in range(num_agents):
#     # Initialize agents with random positions and velocities
#     position = (random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 10))
#     velocity = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
#     agent = Agent(position, velocity)
#     agents.append(agent)

# # Simulation loop
# for step in range(num_steps):
#     # Update each agent
#     for agent in agents:
#         agent.interact(agents)
#         agent.move()
#         agent.update()

#     # TODO: Collect data or update visualization
#     # For example, append agent positions to a list for plotting

# # After simulation, process results
# # TODO: Generate geometry or visualization based on agent data

# # Visualization code (if using Rhino/Grasshopper)
# # For example, create points or lines based on agent positions

# # Output variables (connect to Grasshopper outputs if applicable)
# # agent_positions = [agent.position for agent in agents]

# # If running as a standalone script, include visualization using matplotlib or other libraries
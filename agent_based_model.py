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
agent = Agent(srf, start_position)

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


# Define additional classes if needed (e.g., Environment, Obstacle)

# Simulation parameters
num_agents = 100  # Number of agents
num_steps = 100   # Number of simulation steps
agents = []       # List to hold agent instances

# Initialize agents
for i in range(num_agents):
    # Initialize agents with random positions and velocities
    position = (random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 10))
    velocity = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    agent = Agent(position, velocity)
    agents.append(agent)

# Simulation loop
for step in range(num_steps):
    # Update each agent
    for agent in agents:
        agent.interact(agents)
        agent.move()
        agent.update()

    # TODO: Collect data or update visualization
    # For example, append agent positions to a list for plotting

# After simulation, process results
# TODO: Generate geometry or visualization based on agent data

# Visualization code (if using Rhino/Grasshopper)
# For example, create points or lines based on agent positions

# Output variables (connect to Grasshopper outputs if applicable)
# agent_positions = [agent.position for agent in agents]

# If running as a standalone script, include visualization using matplotlib or other libraries
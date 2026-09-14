# SmartPath Autonomous Delivery Robot

SmartPath is a simulated autonomous delivery robot developed using Python and PyBullet. The project demonstrates how a mobile robot can navigate through an indoor environment while avoiding obstacles and traveling from a starting position to a destination.

## Features

- Simulated Husky mobile robot
- Indoor environment with walls and obstacles
- A* path-planning algorithm
- Automatic movement between waypoints
- Simulated distance sensors using PyBullet ray casting
- Green, yellow, and red sensor indicators based on obstacle distance
- Visible A* navigation path
- Automatic stopping at the goal

## AI Technique

The main AI technique used in this project is the A* search algorithm. A* calculates an efficient path from the robot's starting position to its destination while avoiding areas that contain obstacles.

The environment is represented as a grid. The algorithm checks neighboring locations and uses a heuristic to estimate the remaining distance to the goal. After a path is found, the robot follows the calculated waypoints until it reaches the destination.

## Sensors

The robot uses three simulated distance sensors pointed forward, front-left, and front-right. PyBullet ray casting is used to detect nearby objects.

The sensor beams change color based on distance:

- Green = clear
- Yellow = obstacle nearby
- Red = obstacle close

## Technologies Used

- Python
- PyBullet
- A* Search Algorithm
- VS Code

## Project Purpose

This prototype represents an autonomous delivery robot that could be used in places such as hospitals, warehouses, offices, or schools to transport small items.

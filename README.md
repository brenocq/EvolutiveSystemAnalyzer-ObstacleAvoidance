# Evolutive System Analyzer (ObstacleAvoidance)
---
### Introduction

This project aims to create an interface to monitor the evolution of robots within a simulated environment, being possible to choose different evolutionary techniques and different evolutionary pressures and evaluate the result with graphs.

This project was made using ROS, Gazebo and Matlab (Robotics System Toolbox and App Designer).

### Evolutionary System
The evolutionary system developed was based on the doctoral thesis of [Professor Eduardo Simões](https://github.com/simoesusp/).

the goal of the evolutionary system is to evolve the simulated robots to walk as fast as possible while they deviate from the obstacles.
Each robot has 3 distance sensor that are set as active when the distance threshold is reached. One sensor is fixed at the front and the other two (left and right) have different inclination angles (the data from a Lidar sensor was used to simulate the sensors in different position).

Each robot has 5 genes:
- Sensor Activation: Minimum distance to activate the sensor and start the rotation movement.
- Linear Velocity: robot velocity when moving.
- Rotation Time: Time the robot stays spinning
- Angular Velocity: Angular speed that the robot stays spinning.
- Sensor Angle: Angle between right/left sensor and front sensor.

When one sensor is activated the robot rotate to the opposite side. Otherwise, the robot go forward.

Nowadays it is possible to simulate crossing, mutation, neutralization, predation and back mutation prevention.

### Simulated Environment
 By the time the robots and the map used were acquired by the package turtlebot3 for gazebo.

 <p align="center">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/Turtlebot3.png" height="300">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/InitialSpawnSimulation.png" height="300>
</p>

### Matlab App
The Matlab App communicates with the *robot_obst_avoid* ROS node to set the robots proprieties and control the simulation environment.

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/MatlabApp1.png" height="800>
</p>

#### Graphs
The **Current Robot Fitness** graph shows the fitness of each robot per generation.

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/CurrentRobotFitnessGraph.png">
</p>

The **Average Robot Fitness** graph shows the average of the last 5 fitness of each robot per generation (data used to decide which robots will cross).
<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/AverageRobotFitnessGraph.png">
</p>

The other 6 graphs shows the current value of each gene on each robot per generation. (There are two graphs for the sensor angle gene).
<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/SensorActivationGraph.png">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/LinearVelocityGraph.png">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/RotationTimeGraph.png">
 </p>
 <p align="center">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/AngularVelocityGraph.png">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/CreateReadMe/Images/SensorAngleGraphs.png">
 </p>

#### Setting Fixed Genes

It is possible to set a fixed value to some genes. When this occur, this gene in the chromosome of each robot changes to the desired value. This makes possible the evolution of the genes of each robot in different time.

#### Selection of Types of Pressure

**Back mutation prevention:** After one gene suffer mutation, it can mutate again only if all other genes also suffered mutation).

**Predation:** Every X generation, the robot with the least average fitness is killed by the predator (all the genes are set as random).

#### Selection of Types of Diversity

**Mutation:**
**Neutralization:**
**Crossing:**

#### Simulation Connection

#### Save Images and PDFs

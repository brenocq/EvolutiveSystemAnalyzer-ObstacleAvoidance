# Evolutive System Analyzer (Obstacle Avoidance)

## Introduction

This project aims to create an interface to monitor the evolution of robots within a simulated environment, being possible to choose different evolutionary techniques and different evolutionary pressures and evaluate the result with graphs.

This project was made using ROS, Gazebo and Matlab (Robotics System Toolbox and App Designer).

The evolutionary system developed was based on the PhD work of [Professor Eduardo Simões](https://github.com/simoesusp/).

## Evolutionary System
The goal of the evolutionary system is to evolve the simulated robots to walk as fast as possible while they deviate from the obstacles.
Each robot has 3 distance sensor that are set as active when the distance threshold is reached. One sensor is fixed at the front and the other two (left and right) have different inclination angles (the data from a Lidar sensor was used to simulate the sensors in different position).

Each robot has 5 genes:
- Sensor Activation: Minimum distance to activate the sensor and start the rotation movement.
- Linear Velocity: robot velocity when moving.
- Rotation Time: Time the robot stays spinning
- Angular Velocity: Angular speed that the robot stays spinning.
- Sensor Angle: Angle between right/left sensor and front sensor.

When one sensor is activated the robot rotate to the opposite side. Otherwise, the robot go forward.

Nowadays it is possible to simulate crossing, mutation, neutralization, predation and back mutation prevention.

## Simulated Environment
 By the time the the map used was acquired by the package turtlebot3 for gazebo.

 <p align="center">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/Robot.png" height="300">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/World.png" height="300">
</p>


## Matlab App
The Matlab App communicates with the *robot_obst_avoid* ROS node to set the robots proprieties and control the simulation environment.

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/MatlabApp2.png" height="800">
</p>

### Graphs
The **Current Robot Fitness** graph shows the fitness of each robot per generation.

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/CurrentRobotFitnessGraph.png">
</p>

The **Average Robot Fitness** graph shows the average of the last N fitness values of each robot per generation (data used to decide which robots will cross).
<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/AverageRobotFitnessGraph.png">
</p>

The other 5 graphs shows the current value of each gene on each robot per generation. (There are two graphs for the sensor angle gene).
<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/SensorActivationGraph.png" height="150">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/LinearVelocityGraph.png" height="150">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/RotationTimeGraph.png" height="150">
 </p>

 <p align="center">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/AngularVelocityGraph.png" height="150">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/SensorAngleGraph.png"  height="150">
 </p>

### Analyze Evolution

#### Graph visualization

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/AnalyzeEvolution.png" height="150">
</p>

It is possible to show/hide each line in the graphs by clicking on its label. All the graphs will be updated with this new configuration.

#### Manage Generation

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/ManageGeneration.png" height="200">
</p>

It is possible to show information about only one section of the populations. You can change the first slider to set the minimum limit and change the second slider to set the maximum limit for all graphics.

In addition, it is possible to return to a previous generation (note: all data after this generation will be lost). To return to a previous generation, you need to change the `Current Generation`. After choosing which generation you want to return, click on the button `Yes`;

### Simulation Parameters

#### Setting Fixed Genes

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/FixedGenes.png" height="200">
</p>

It is possible to set a fixed value to some genes. When this occur, these gene in the chromosome of each robot are changed to the desired value. This feature makes possible the evolution of the genes of each robot in different time.

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/FixedGraphs.png" height="350">
</p>

#### Selection of Types of Pressure

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/Pressure.png" height="200">
</p>

**Back mutation prevention:** After one gene suffer mutation, it can mutate again only if all other genes in the chromosome also suffered mutation.

**Predation:** Every N generations, the robot with the least average fitness is killed by the predator (all the genes in the chromosome are set as random).

#### Selection of Evolutionary Parameters

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/Evolution.png" height="200">
</p>

**Mutation:** You can set the chance of each gene mutating.

**Neutralization:** You can set how strong the mutation will be.
 - 0%: Completely random number
 - 100%: No mutation

**Crossing:** The robot with the best average fitness will cross with the other robots that obtained the current fitness lower than the one of the best robot.

**Qty. generations in mean:** You can set the number of generations that will be considered when calculating the mean fitness.

**Qty. best robots:** The best robot can be choose randomly between the X robots with best fitness.

### Control Simulation

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/ControlSimulation1.png" height="200">
  <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/ControlSimulation2.png" height="200">
</p>

It is possible to set the ipv4 of the ROS master before and while the Matlab App is running. Also, the simulation can be controlled with the stop/start switch and pause/run switch.

### Data management

#### Manage Data

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/ManageData.png" height="200">
</p>

You can also import all the data generated to a txt file or export all data stored in a txt file to the graphs and simulation. With this feature you can continue to evolve the system and analyze that populations later. (The export and import process occurs in the same directory as the `EvolutiveSysAnalyze.mlapp` file is stored).

#### Save Graphs

<p align="center">
 <img src="https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance/blob/master/Images/SaveGraphs.png" height="200">
</p>


There are two ways to save the data. The first one is to select one of the graphs and generate a _png_ image of it. The second one is to save all the data as a PDF file. It is possible to choose the file name and where to save it.

## How to Install

First, make sure that you have the Matlab Robotics System Toolbox correctly setted to use ROS and Matlab.

After this, go to your workspace and clone the ROS package folder in this repository using the lines above on terminal:
```
$ cd ~/catkin_ws/src
$ svn export
https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance.git/trunk/obstacle_avoidance_simulation
```
After this, build again your ROS workspace:
```
$ cd ~/catkin_ws
$ catkin_make
```

If everything is fine until here, go to the folder where you store your matlab files and use:
```
$ svn export https://github.com/Brenocq/EvolutiveSystemAnalyzer-ObstacleAvoidance.git/trunk/MatlabApp
```
All the temp.txt and other data files must be stored in this folder.

Now you have the ROS package and Matlab App installed. It is still necessary to create the new types of messages for ROS on Matlab. To do this, first it is good to delete a folder inside the `obstacle_avoidance_simulation` ROS package.
```
$ rm -r ~/catkin_ws/src/obstacle_avoidance_simulation/matlabMsg/matlab_gen
```
After this, go to Matlab, write the line above and follow the instructions that will appear:
```
> rosgenmsg('~/catkin_ws/src/obstacle_avoidance_simulation/matlabMsg/')
```

## How to Run

First you need the ROS master to be running. Run on your teminal:
```
$ roscore
```
If you got an error while running `roscore`, go open you _.bashrc_ file and add the line below. Change `XX.XX.XX.XXX` to your ipv4.
``` txt
export ROS_MASTER_URI=http://XX.XX.XX.XXX:11311
export ROS_HOSTNAME=XX.XX.XX.XXX
```

Now, if your `roscore` command executed correctly, open another terminal and use roslaunch to initialize the ROS world:
```
$ roslaunch obstacle_avoidance_simulation main.launch
```
After this, go to another terminal and use rosrun to create the ros node:
```
$ rosrun obstacle_avoidance_simulation robot_obstacle_avoidance_node

```

Now everything on the simulation is working. Everything is stopped because them are waiting commands from the Matlab App.
On matlab, open `EvolutiveSysAnalyze.mlapp` and execute the App (make sure your ipv4 is defined correctly before run the App).

Right, now just set the switch stop/start as start and the plots will be updated at the end of each population.

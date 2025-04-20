# Maze-Solver-Robot

## Overview
The **Maze-Solver-Robot** is a project that demonstrates the use of robotics and algorithms to solve mazes. It integrates a robot with a navigation system designed to efficiently find the optimal path through a maze. The project utilizes hardware such as motors and sensors, combined with software for real-time decision-making and movement.

The robot employs a left-first maze-solving algorithm for maze solving. It features a custom-built line-sensing array consisting of five pairs of photodiodes and IR LEDs, enabling precise detection of path boundaries. For line following, it utilizes a PID control system, with the proportional (Kp), integral (Ki), and derivative (Kd) gains adjustable in real-time through a WebSocket-based web interface.

## Features
- **Maze Solving Algorithm**: Uses the left-first algorithm to find the shortest path through a maze.
- **Robot Navigation**: Integrates motor control for real-time movement within the maze.
- **Sensor Integration**: Incorporates photodiode and IR LED sensor pairs to detect boundaries for accurate navigation.

## Future Work

- Implement advanced maze-solving algorithms for improved efficiency.
- Integrate machine learning for dynamic maze-solving.
- Integrating front-facing IR sensors for real-time obstacle detection and enhancing the robot's ability to navigate dynamic environments.

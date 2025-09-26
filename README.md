# Micromouse V2 - Autonomous Maze-Solving Robot
## Project Overview

This repository contains the complete C/C++ firmware for Micromouse V2, an autonomous robot designed to explore and solve a 16x16 maze. The entire system is built on the ESP32 platform, leveraging the FreeRTOS real-time operating system to manage complex, concurrent tasks such as sensor reading, motion control, and navigation algorithms.

This project, developed 1.5 years ago, represents a significant milestone in my journey as an embedded systems engineer, demonstrating a strong foundation in robotics, control systems, and real-time software architecture.

---

## Core Technical Features

*   **Precise Motion Control:** Implemented a robust PID (Proportional-Integral-Derivative) controller for both linear and angular velocity, enabling precise, repeatable movements.
*   **Autonomous Navigation:** Utilizes the Flood Fill algorithm to efficiently map the maze during an exploration run and calculate the optimal path for a subsequent high-speed run.
*   **Real-Time Architecture:** Built on FreeRTOS with a multi-threaded design that separates sensor data acquisition from the main control and navigation logic, ensuring a non-blocking and responsive system.
*   **High-Resolution Sensing:** Integrates an array of 5 VL53L0X Time-of-Flight (ToF) sensors for accurate, high-speed wall detection.
*   **Low-Level Hardware Interfacing:** Direct control over motor PWM via the ESP-IDF's LEDC peripheral and high-frequency encoder readings using GPIO interrupts.

---

## Technical Deep Dive

This project was a deep dive into the core challenges of autonomous robotics. Here are some of the key technical solutions implemented:

### 1. Software Architecture

The system's backbone is FreeRTOS. The main application is divided into two primary tasks:

*   `ToFRead_Task`: A high-priority, non-blocking task dedicated to continuously polling the 5 ToF sensors. This ensures that the robot's perception of the world is always up-to-date.
*   `explore_maze_Task`: A lower-priority task that contains the main state machine. It consumes the latest sensor data, runs the Flood Fill algorithm, makes decisions, and issues high-level commands to the motion system (e.g., `forward()`, `turnLeft()`).

This separation ensures that complex calculations in the navigation logic never interfere with the critical timing of sensor readings.

### 2. Motion Control (PID & Encoders)

The core of the robot's physical precision lies in its closed-loop control system.

*   **Encoder Interrupts:** Quadrature encoder signals from both motors are handled by high-priority GPIO interrupts (`IRAM_ATTR`), ensuring that not a single "tick" of wheel rotation is missed, even while the main processor is busy.
*   **PID Controller:** A custom PID controller was implemented from scratch to translate a target encoder value (e.g., "move forward 820 ticks") into precise PWM signals for the motor drivers. The controller features anti-windup logic for the integral term and was tuned to provide fast, stable, and critically damped responses, minimizing overshoot and oscillation.

### 3. Navigation & Maze-Solving (Flood Fill)

The robot's intelligence is driven by the Flood Fill algorithm.
*   **Logic:** During the exploration phase, the robot uses the Flood Fill values to always move towards the cell with the lowest "distance" to the target. After detecting walls with its sensors, it updates its internal map of the maze and **re-runs the Flood Fill algorithm** from the target to propagate the new wall information across the entire map. This allows the robot to dynamically adapt its path as it discovers new parts of the maze.

---

## Hardware Overview

*   **MCU:** ESP32-WROOM-32
*   **Motor Driver:** 2x DRV8833
*   **Sensors:** 5x VL53L0X Time-of-Flight Distance Sensors
*   **Motors:** DC Micromotors with Quadrature Encoders

# Congestion-Aware Distributed MPC for Multi-AMR Control

This repository contains research and engineering code for congestion-aware distributed MPC (CA-DMPC) in multi-AMR simulation studies. The implementation is packaged as a ROS 2 Python project and is designed to run on top of an open-source simulation stack (Gazebo + TurtleBot3), focusing on local control, inter-robot trajectory sharing, congestion modeling, and experiment logging.

## Research Context / Problem

In multi-AMR navigation, robots that independently follow paths can create congestion, delays, and deadlocks in shared spaces. This project investigates distributed MPC with congestion awareness, where each robot plans locally while incorporating predicted neighbor motion and occupancy-based congestion cost.

## Key Components

- **CA-DMPC controller** (`ca_dmpc_experiments/ca_dmpc_controller.py`): local MPC with occupancy-based congestion penalty.
- **Baseline controllers**:
  - `ca_dmpc_experiments/standard_mpc_controller.py` (standard MPC baseline)
  - `ca_dmpc_experiments/rule_based_controller.py` and `path_follower.py` (reference baselines)
- **Distributed state/trajectory handling**:
  - `neighbor_trajectory_buffer.py` (communication-radius filtering)
  - `trajectory_broadcaster.py` (predicted trajectory publishing)
  - `occupancy_grid_builder.py` (time-indexed occupancy map construction)
  - `robot_state_collector.py` (fleet-state aggregation)
- **Evaluation/logging** (`metric_logger.py`): fleet state logs and deadlock interval detection.
- **Experiment orchestration** (`launch/*.launch.py`): multi-robot launch pipelines for standard MPC and CA-DMPC modes.

## Tech Stack

- ROS 2 (`rclpy`, `launch`, `launch_ros`, `ament_python`)
- Python
- SciPy optimization (`scipy.optimize`)
- ROS message interfaces (`geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`)
- Open-source simulation environment integration: Gazebo + TurtleBot3 (external)

## Repository Structure

- `ca_dmpc_experiments/`: core ROS 2 nodes for control, coordination, and logging
- `launch/`: experiment launch configurations
- `agv_grid/`: additional grid-based AMR simulation utilities

## Note on Open-Source Dependencies

Gazebo and TurtleBot3 are open-source simulation components used as the environment layer. This repository does **not** re-implement Gazebo or TurtleBot3; it contains the congestion-aware distributed control and experiment infrastructure built on top of that ecosystem.

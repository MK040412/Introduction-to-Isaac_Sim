# Welcome to the Intoduction-to-Isaac_Sim

This documentation focuses on controlling the **Franka-Emika Panda** robotic arm in the **Omniverse Isaac Sim / Isaac Lab** environment.  
You will learn how to load the robot, configure its assets, and implement various control strategies—starting with a classic **Proportional-Derivative (PD) joint-space controller**.

## What You’ll Find Here

| Section | Description |
|---------|-------------|
| **Franka Arm** | An end-to-end guide covering the asset structure, simulation setup, PD control implementation, and ROS 2 integration for the Panda arm. |
| **Asset API Reference** | Auto-generated (TypeDoc) API documentation for the core Isaac Lab asset classes—`AssetBase`, `Articulation`, `RigidObject`, etc. |

### Franka Arm Sub-Chapters

1. **Overview** – Asset layout (URDF, USD, configuration classes) and architecture of the Franka arm within Isaac Lab.  
2. **Simulation Setup** – Step-by-step instructions to spawn the robot in a stage, initialize PhysX views, and enable debug visualization.  
3. **PD Control** – Mathematical formulation and practical code examples for joint-space PD control, including gain tuning tips.  
4. **ROS 2 Integration** – How to bridge joint states and commands between Isaac Lab and external ROS 2 nodes.

## Prerequisites

* **NVIDIA Omniverse Isaac Sim ≥ 2023.1**  
* **Python ≥ 3.9** with `isaaclab`, `torch`, and optional `rclpy` for ROS 2.  
* Basic familiarity with robotics concepts (kinematics, dynamics, control).

## Quick Start

```python
from isaaclab.assets.articulation import Articulation
from franka_asset_cfg import FrankaArmCfg

cfg = FrankaArmCfg(prim_path="/World/Franka")
franka = Articulation(cfg)
franka.initialize(scene)
```

Continue to the next chapter to dive into the Franka asset structure and begin your simulation journey!

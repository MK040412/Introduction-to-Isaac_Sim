<html><head></head><body><h1>PD Control for Franka Robot</h1>
<p>This guide explains how to implement and tune Proportional-Derivative (PD) control for the Franka Emika Panda robot in NVIDIA Isaac Sim. PD control is a fundamental approach to robot joint control that provides stable and precise motion control.</p>
<h2>Overview of PD Control</h2>
<p>PD control is a feedback control mechanism that calculates control effort based on two terms:</p>
<ol>
<li><strong>Proportional (P)</strong> term: Provides a control signal proportional to the error between the current state and the desired state</li>
<li><strong>Derivative (D)</strong> term: Provides a control signal proportional to the rate of change of the error</li>
</ol>
<p>The PD controller calculates a control signal (torque/force) as:</p>
<pre><code>u = Kp * (x_desired - x_current) + Kd * (v_desired - v_current)
</code></pre>
<p>Where:</p>
<ul>
<li><code>u</code> is the control signal (joint torque)</li>
<li><code>Kp</code> is the proportional gain</li>
<li><code>Kd</code> is the derivative gain</li>
<li><code>x_desired</code> and <code>x_current</code> are the desired and current joint positions</li>
<li><code>v_desired</code> and <code>v_current</code> are the desired and current joint velocities</li>
</ul>
<h2>Implementation in Isaac Sim</h2>
<p>Isaac Sim provides several ways to implement PD control for the Franka robot. The most common approaches are:</p>
<ol>
<li>Using the built-in articulation controller</li>
<li>Implementing a custom PD controller with direct torque control</li>
</ol>
<h3>Using Built-in Articulation Controller</h3>
<p>The Isaac Sim articulation controller already implements PD control internally. Here's how to use it:</p>
<pre><code class="language-python">import omni.isaac.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Initialize simulation
simulation_context = SimulationContext(physics_dt=0.01, rendering_dt=0.01)
simulation_context.start_simulation()

# Load Franka robot
franka_robot = Robot(
    prim_path="/World/Franka",
    name="franka",
    usd_path="/Isaac/Robots/Franka/franka.usd"
)

# Get the articulation controller
controller = franka_robot.get_articulation_controller()

# Define target joint positions for the 7-DOF arm
target_positions = [0.0, -0.4, 0.0, -2.0, 0.0, 2.0, 0.75]
target_velocities = [0.0] * 7  # Zero desired velocity

# Set PD gains
# Note: These values need tuning for your specific application
kp = [500.0] * 7  # Proportional gains
kd = [50.0] * 7   # Derivative gains

# Apply PD control with specified gains
controller.set_gains(kp, kd)

# Apply the joint targets
controller.apply_action(
    ArticulationAction(
        joint_positions=target_positions,
        joint_velocities=target_velocities,
    )
)

# Run simulation for some steps
for _ in range(1000):
    simulation_context.step()
    # You can monitor the joint positions to see if they converge to the target
    current_positions = franka_robot.get_joint_positions()
    print(f"Current positions: {current_positions}")
</code></pre>
<h3>Implementing Custom PD Controller</h3>
<p>For more control over the PD control implementation, you can create a custom controller:</p>
<pre><code class="language-python">import numpy as np
import omni.isaac.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

class FrankaPDController:
    def __init__(self, robot, kp, kd):
        self.robot = robot
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.controller = robot.get_articulation_controller()
        
    def compute_control(self, target_positions, target_velocities=None):
        if target_velocities is None:
            target_velocities = np.zeros_like(target_positions)
            
        # Get current joint states
        current_positions = self.robot.get_joint_positions()
        current_velocities = self.robot.get_joint_velocities()
        
        # Compute position and velocity errors
        position_error = np.array(target_positions) - np.array(current_positions)
        velocity_error = np.array(target_velocities) - np.array(current_velocities)
        
        # Compute PD control torques
        torques = self.kp * position_error + self.kd * velocity_error
        
        return torques
    
    def apply_control(self, target_positions, target_velocities=None):
        torques = self.compute_control(target_positions, target_velocities)
        self.controller.apply_effort(torques)

# Initialize simulation
simulation_context = SimulationContext(physics_dt=0.01, rendering_dt=0.01)
simulation_context.start_simulation()

# Load Franka robot
franka_robot = Robot(
    prim_path="/World/Franka",
    name="franka",
    usd_path="/Isaac/Robots/Franka/franka.usd"
)

# Define PD gains (these need tuning)
kp = [500.0, 500.0, 500.0, 500.0, 100.0, 100.0, 100.0]
kd = [50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0]

# Create PD controller
pd_controller = FrankaPDController(franka_robot, kp, kd)

# Define target joint positions
target_positions = [0.0, -0.4, 0.0, -2.0, 0.0, 2.0, 0.75]

# Control loop
for i in range(1000):
    # Apply control
    pd_controller.apply_control(target_positions)
    
    # Step simulation
    simulation_context.step()
    
    # Optional: Monitor convergence
    if i % 100 == 0:
        current_positions = franka_robot.get_joint_positions()
        error = np.linalg.norm(np.array(target_positions) - np.array(current_positions))
        print(f"Step {i}, Position error: {error}")
</code></pre>
<h2>PD Controller in NVIDIA Isaac Gym/Lab</h2>
<p>For Isaac Lab (formerly Isaac Gym), the implementation is slightly different as it uses the <code>gymapi</code> API. Here's an example:</p>
<pre><code class="language-python">import numpy as np
from isaacgym import gymapi
from isaaclab.tasks.base.rl_task import RLTask

class FrankaPDController:
    def __init__(self, env, franka_handle, num_joints, kp, kd):
        self.env = env
        self.franka_handle = franka_handle
        self.num_joints = num_joints
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        
    def compute_control(self, target_positions, target_velocities=None):
        if target_velocities is None:
            target_velocities = np.zeros(self.num_joints)
            
        # Get current joint states
        current_positions, current_velocities = self.env.get_joint_states(self.franka_handle)
        
        # Compute position and velocity errors
        position_error = np.array(target_positions) - np.array(current_positions)
        velocity_error = np.array(target_velocities) - np.array(current_velocities)
        
        # Compute PD control torques
        torques = self.kp * position_error + self.kd * velocity_error
        
        return torques
    
    def apply_control(self, target_positions, target_velocities=None):
        torques = self.compute_control(target_positions, target_velocities)
        self.env.apply_joint_efforts(self.franka_handle, torques)

# Example usage in an Isaac Lab task
class FrankaPDTask(RLTask):
    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless):
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless)
        
        # Create Franka robots
        self.num_joints = 7
        self._franka_handles = []
        
        # Set up environment
        self._create_envs(self.num_envs, self.cfg["env"]["envSpacing"], int(np.sqrt(self.num_envs)))
        
        # Set up PD controllers for each robot
        self.pd_controllers = []
        kp = [500.0, 500.0, 500.0, 500.0, 100.0, 100.0, 100.0]
        kd = [50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0]
        
        for i in range(self.num_envs):
            controller = FrankaPDController(self.envs[i], self._franka_handles[i], self.num_joints, kp, kd)
            self.pd_controllers.append(controller)
        
    def _create_envs(self, num_envs, spacing, num_per_row):
        # Create environments with Franka robots
        # Implementation details would depend on your specific setup
        pass
        
    def pre_physics_step(self, actions):
        # Convert RL actions to joint targets
        target_positions = self._compute_target_positions(actions)
        
        # Apply PD control for each robot
        for i in range(self.num_envs):
            self.pd_controllers[i].apply_control(target_positions[i])
            
    def _compute_target_positions(self, actions):
        # Convert actions to target positions
        # Implementation depends on your action space
        pass
</code></pre>
<h3>Finding PD Controller Code in Isaac Sim/Lab Repository</h3>
<p>The PD controller implementation in Isaac Sim can be found in the following locations:</p>
<ol>
<li>
<p><strong>Isaac Sim Core</strong>:</p>
<ul>
<li>The articulation controller (which includes PD control) is defined in <code>omni/isaac/core/articulations/articulation.py</code></li>
<li>PD control parameters are handled in <code>omni/isaac/core/utils/types.py</code> in the <code>ArticulationAction</code> class</li>
</ul>
</li>
<li>
<p><strong>Isaac Lab (formerly Isaac Gym)</strong>:</p>
<ul>
<li>Basic PD control is implemented in <code>isaacgym/tasks/base/vec_task.py</code></li>
<li>Task-specific PD control can be found in <code>isaacgym/tasks/franka.py</code> (for Franka-specific implementations)</li>
</ul>
</li>
<li>
<p><strong>Franka-specific controllers</strong>:</p>
<ul>
<li>Look in <code>isaaclab/tasks/manipulation</code> directory for Franka manipulation tasks</li>
<li>PD control parameters are often defined in configuration files under <code>isaaclab/cfg/task</code></li>
</ul>
</li>
</ol>
<h2>Tuning PD Control Parameters</h2>
<p>Proper tuning of PD gains is crucial for optimal robot control. Here are some guidelines:</p>
<h3>General Tuning Process</h3>
<ol>
<li><strong>Start with low gains</strong>: Begin with low values for Kp and Kd to avoid instability</li>
<li><strong>Increase Kp gradually</strong>: Slowly increase the proportional gain until you get reasonable tracking</li>
<li><strong>Add Kd to dampen oscillations</strong>: Add derivative gain to reduce overshooting and oscillations</li>
<li><strong>Fine-tune</strong>: Adjust both gains to achieve the desired performance</li>
</ol>
<h3>Typical Gain Values for Franka Robot</h3>

Joint | Kp Range | Kd Range | Notes
-- | -- | -- | --
1 | 300-600 | 30-60 | Shoulder joint, can use higher gains
2 | 300-600 | 30-60 | Shoulder joint, can use higher gains
3 | 300-600 | 30-60 | Arm joint
4 | 300-600 | 30-60 | Elbow joint
5 | 100-300 | 10-30 | Wrist joint, needs lower gains
6 | 100-300 | 10-30 | Wrist joint, needs lower gains
7 | 100-300 | 10-30 | Wrist joint, needs lower gains


<blockquote>
<p><strong>Note</strong>: These values are starting points. The optimal values depend on your specific task and desired performance.</p>
</blockquote>
<h3>Common Tuning Issues</h3>
<ol>
<li><strong>Oscillations</strong>: If the robot oscillates around the target position, reduce Kp or increase Kd</li>
<li><strong>Slow response</strong>: If the robot moves too slowly to the target, increase Kp</li>
<li><strong>Overshoot</strong>: If the robot overshoots the target position, increase Kd</li>
<li><strong>Steady-state error</strong>: If the robot doesn't reach the exact target position, increase Kp</li>
</ol>
<h2>Advanced PD Control Variants</h2>
<p>Beyond basic PD control, several variants can improve performance:</p>
<h3>PD Control with Gravity Compensation</h3>
<pre><code class="language-python">def compute_control_with_gravity_comp(self, target_positions, target_velocities=None):
    # Compute basic PD control
    torques = self.compute_control(target_positions, target_velocities)
    
    # Add gravity compensation
    gravity_torques = self.robot.compute_gravity_compensation()
    
    # Combine torques
    total_torques = torques + gravity_torques
    
    return total_torques
</code></pre>
<h3>PD Control with Feedforward Dynamics</h3>
<pre><code class="language-python">def compute_control_with_feedforward(self, target_positions, target_velocities, target_accelerations):
    # Compute basic PD control
    torques = self.compute_control(target_positions, target_velocities)
    
    # Add feedforward term using inverse dynamics
    M = self.robot.get_mass_matrix()
    feedforward_torques = np.dot(M, target_accelerations)
    
    # Combine torques
    total_torques = torques + feedforward_torques
    
    return total_torques
</code></pre>
<h2>Implementing Task-Specific PD Control</h2>
<p>For specific tasks like pick-and-place, you may need to define trajectory-following PD control:</p>
<pre><code class="language-python">import numpy as np
from scipy.interpolate import CubicSpline
import time

class TrajectoryPDController:
    def __init__(self, robot, kp, kd):
        self.robot = robot
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.controller = robot.get_articulation_controller()
        self.trajectory = None
        self.start_time = None
        
    def set_trajectory(self, waypoints, durations):
        """
        Set a trajectory through waypoints with specified durations.
        
        Args:
            waypoints: List of joint position arrays
            durations: List of times to reach each waypoint
        """
        # Create cubic spline for each joint
        cum_times = np.cumsum(durations)
        self.trajectory = []
        
        for joint_idx in range(len(waypoints[0])):
            joint_waypoints = [wp[joint_idx] for wp in waypoints]
            spline = CubicSpline([0] + list(cum_times), [joint_waypoints[0]] + joint_waypoints)
            self.trajectory.append(spline)
            
        self.duration = cum_times[-1]
        self.start_time = None
        
    def start(self):
        """Start following the trajectory."""
        self.start_time = time.time()
        
    def update(self):
        """Update the controller based on the current time."""
        if self.start_time is None or self.trajectory is None:
            return
            
        # Calculate time since trajectory start
        current_time = time.time() - self.start_time
        
        if current_time &gt; self.duration:
            # Trajectory complete
            return False
            
        # Compute target positions and velocities from splines
        target_positions = []
        target_velocities = []
        
        for spline in self.trajectory:
            target_positions.append(spline(current_time))
            target_velocities.append(spline(current_time, 1))  # First derivative
            
        # Apply PD control
        torques = self.compute_control(target_positions, target_velocities)
        self.controller.apply_effort(torques)
        
        return True
        
    def compute_control(self, target_positions, target_velocities):
        # Get current joint states
        current_positions = self.robot.get_joint_positions()
        current_velocities = self.robot.get_joint_velocities()
        
        # Compute position and velocity errors
        position_error = np.array(target_positions) - np.array(current_positions)
        velocity_error = np.array(target_velocities) - np.array(current_velocities)
        
        # Compute PD control torques
        torques = self.kp * position_error + self.kd * velocity_error
        
        return torques
</code></pre>
<h2>Conclusion</h2>
<p>PD control is a fundamental approach to controlling the Franka robot in Isaac Sim and Isaac Lab. By understanding the underlying principles and implementation details, you can create stable and precise control strategies for various robotic tasks.</p>
<p>The code snippets provided in this guide serve as a starting point for implementing PD control in your own applications. Remember to tune the PD gains carefully for your specific robot and task requirements.</p>
<h2>Next Steps</h2>
<ul>
<li>Explore <a>Impedance Control</a> for compliant interaction with the environment</li>
<li>Learn about <a>Inverse Kinematics</a> for end-effector control</li>
<li>Try implementing PD control in the <a>Basic Pick-and-Place Practice</a> task</li>
</ul>
<h2>References</h2>
<ul>
<li>NVIDIA Isaac Sim Documentation: <a href="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_api.html">Articulation Controller API</a></li>
<li>Franka Emika Control Parameters: <a href="https://frankaemika.github.io/docs/control_parameters.html">Franka Control Interface</a></li>
<li><a href="https://studywolf.wordpress.com/2013/09/02/robot-control-pdpid-control/">Control of Robot Manipulators</a> by Travis DeWolf</li>
</ul></body></html># PD Control for Franka Robot

This guide explains how to implement and tune Proportional-Derivative (PD) control for the Franka Emika Panda robot in NVIDIA Isaac Sim. PD control is a fundamental approach to robot joint control that provides stable and precise motion control.

## Overview of PD Control

PD control is a feedback control mechanism that calculates control effort based on two terms:
1. **Proportional (P)** term: Provides a control signal proportional to the error between the current state and the desired state
2. **Derivative (D)** term: Provides a control signal proportional to the rate of change of the error

The PD controller calculates a control signal (torque/force) as:

```
u = Kp * (x_desired - x_current) + Kd * (v_desired - v_current)
```

Where:
- `u` is the control signal (joint torque)
- `Kp` is the proportional gain
- `Kd` is the derivative gain
- `x_desired` and `x_current` are the desired and current joint positions
- `v_desired` and `v_current` are the desired and current joint velocities

## Implementation in Isaac Sim

Isaac Sim provides several ways to implement PD control for the Franka robot. The most common approaches are:

1. Using the built-in articulation controller
2. Implementing a custom PD controller with direct torque control

### Using Built-in Articulation Controller

The Isaac Sim articulation controller already implements PD control internally. Here's how to use it:

```python
import omni.isaac.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Initialize simulation
simulation_context = SimulationContext(physics_dt=0.01, rendering_dt=0.01)
simulation_context.start_simulation()

# Load Franka robot
franka_robot = Robot(
    prim_path="/World/Franka",
    name="franka",
    usd_path="/Isaac/Robots/Franka/franka.usd"
)

# Get the articulation controller
controller = franka_robot.get_articulation_controller()

# Define target joint positions for the 7-DOF arm
target_positions = [0.0, -0.4, 0.0, -2.0, 0.0, 2.0, 0.75]
target_velocities = [0.0] * 7  # Zero desired velocity

# Set PD gains
# Note: These values need tuning for your specific application
kp = [500.0] * 7  # Proportional gains
kd = [50.0] * 7   # Derivative gains

# Apply PD control with specified gains
controller.set_gains(kp, kd)

# Apply the joint targets
controller.apply_action(
    ArticulationAction(
        joint_positions=target_positions,
        joint_velocities=target_velocities,
    )
)

# Run simulation for some steps
for _ in range(1000):
    simulation_context.step()
    # You can monitor the joint positions to see if they converge to the target
    current_positions = franka_robot.get_joint_positions()
    print(f"Current positions: {current_positions}")
```

### Implementing Custom PD Controller

For more control over the PD control implementation, you can create a custom controller:

```python
import numpy as np
import omni.isaac.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

class FrankaPDController:
    def __init__(self, robot, kp, kd):
        self.robot = robot
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.controller = robot.get_articulation_controller()
        
    def compute_control(self, target_positions, target_velocities=None):
        if target_velocities is None:
            target_velocities = np.zeros_like(target_positions)
            
        # Get current joint states
        current_positions = self.robot.get_joint_positions()
        current_velocities = self.robot.get_joint_velocities()
        
        # Compute position and velocity errors
        position_error = np.array(target_positions) - np.array(current_positions)
        velocity_error = np.array(target_velocities) - np.array(current_velocities)
        
        # Compute PD control torques
        torques = self.kp * position_error + self.kd * velocity_error
        
        return torques
    
    def apply_control(self, target_positions, target_velocities=None):
        torques = self.compute_control(target_positions, target_velocities)
        self.controller.apply_effort(torques)

# Initialize simulation
simulation_context = SimulationContext(physics_dt=0.01, rendering_dt=0.01)
simulation_context.start_simulation()

# Load Franka robot
franka_robot = Robot(
    prim_path="/World/Franka",
    name="franka",
    usd_path="/Isaac/Robots/Franka/franka.usd"
)

# Define PD gains (these need tuning)
kp = [500.0, 500.0, 500.0, 500.0, 100.0, 100.0, 100.0]
kd = [50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0]

# Create PD controller
pd_controller = FrankaPDController(franka_robot, kp, kd)

# Define target joint positions
target_positions = [0.0, -0.4, 0.0, -2.0, 0.0, 2.0, 0.75]

# Control loop
for i in range(1000):
    # Apply control
    pd_controller.apply_control(target_positions)
    
    # Step simulation
    simulation_context.step()
    
    # Optional: Monitor convergence
    if i % 100 == 0:
        current_positions = franka_robot.get_joint_positions()
        error = np.linalg.norm(np.array(target_positions) - np.array(current_positions))
        print(f"Step {i}, Position error: {error}")
```

## PD Controller in NVIDIA Isaac Gym/Lab

For Isaac Lab (formerly Isaac Gym), the implementation is slightly different as it uses the `gymapi` API. Here's an example:

```python
import numpy as np
from isaacgym import gymapi
from isaaclab.tasks.base.rl_task import RLTask

class FrankaPDController:
    def __init__(self, env, franka_handle, num_joints, kp, kd):
        self.env = env
        self.franka_handle = franka_handle
        self.num_joints = num_joints
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        
    def compute_control(self, target_positions, target_velocities=None):
        if target_velocities is None:
            target_velocities = np.zeros(self.num_joints)
            
        # Get current joint states
        current_positions, current_velocities = self.env.get_joint_states(self.franka_handle)
        
        # Compute position and velocity errors
        position_error = np.array(target_positions) - np.array(current_positions)
        velocity_error = np.array(target_velocities) - np.array(current_velocities)
        
        # Compute PD control torques
        torques = self.kp * position_error + self.kd * velocity_error
        
        return torques
    
    def apply_control(self, target_positions, target_velocities=None):
        torques = self.compute_control(target_positions, target_velocities)
        self.env.apply_joint_efforts(self.franka_handle, torques)

# Example usage in an Isaac Lab task
class FrankaPDTask(RLTask):
    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless):
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless)
        
        # Create Franka robots
        self.num_joints = 7
        self._franka_handles = []
        
        # Set up environment
        self._create_envs(self.num_envs, self.cfg["env"]["envSpacing"], int(np.sqrt(self.num_envs)))
        
        # Set up PD controllers for each robot
        self.pd_controllers = []
        kp = [500.0, 500.0, 500.0, 500.0, 100.0, 100.0, 100.0]
        kd = [50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0]
        
        for i in range(self.num_envs):
            controller = FrankaPDController(self.envs[i], self._franka_handles[i], self.num_joints, kp, kd)
            self.pd_controllers.append(controller)
        
    def _create_envs(self, num_envs, spacing, num_per_row):
        # Create environments with Franka robots
        # Implementation details would depend on your specific setup
        pass
        
    def pre_physics_step(self, actions):
        # Convert RL actions to joint targets
        target_positions = self._compute_target_positions(actions)
        
        # Apply PD control for each robot
        for i in range(self.num_envs):
            self.pd_controllers[i].apply_control(target_positions[i])
            
    def _compute_target_positions(self, actions):
        # Convert actions to target positions
        # Implementation depends on your action space
        pass
```

### Finding PD Controller Code in Isaac Sim/Lab Repository

The PD controller implementation in Isaac Sim can be found in the following locations:

1. **Isaac Sim Core**: 
   - The articulation controller (which includes PD control) is defined in `omni/isaac/core/articulations/articulation.py`
   - PD control parameters are handled in `omni/isaac/core/utils/types.py` in the `ArticulationAction` class

2. **Isaac Lab (formerly Isaac Gym)**:
   - Basic PD control is implemented in `isaacgym/tasks/base/vec_task.py`
   - Task-specific PD control can be found in `isaacgym/tasks/franka.py` (for Franka-specific implementations)

3. **Franka-specific controllers**:
   - Look in `isaaclab/tasks/manipulation` directory for Franka manipulation tasks
   - PD control parameters are often defined in configuration files under `isaaclab/cfg/task`

## Tuning PD Control Parameters

Proper tuning of PD gains is crucial for optimal robot control. Here are some guidelines:

### General Tuning Process

1. **Start with low gains**: Begin with low values for Kp and Kd to avoid instability
2. **Increase Kp gradually**: Slowly increase the proportional gain until you get reasonable tracking
3. **Add Kd to dampen oscillations**: Add derivative gain to reduce overshooting and oscillations
4. **Fine-tune**: Adjust both gains to achieve the desired performance

### Typical Gain Values for Franka Robot

| Joint | Kp Range | Kd Range | Notes |
|-------|----------|----------|-------|
| 1     | 300-600  | 30-60    | Shoulder joint, can use higher gains |
| 2     | 300-600  | 30-60    | Shoulder joint, can use higher gains |
| 3     | 300-600  | 30-60    | Arm joint |
| 4     | 300-600  | 30-60    | Elbow joint |
| 5     | 100-300  | 10-30    | Wrist joint, needs lower gains |
| 6     | 100-300  | 10-30    | Wrist joint, needs lower gains |
| 7     | 100-300  | 10-30    | Wrist joint, needs lower gains |

> **Note**: These values are starting points. The optimal values depend on your specific task and desired performance.

### Common Tuning Issues

1. **Oscillations**: If the robot oscillates around the target position, reduce Kp or increase Kd
2. **Slow response**: If the robot moves too slowly to the target, increase Kp
3. **Overshoot**: If the robot overshoots the target position, increase Kd
4. **Steady-state error**: If the robot doesn't reach the exact target position, increase Kp

## Advanced PD Control Variants

Beyond basic PD control, several variants can improve performance:

### PD Control with Gravity Compensation

```python
def compute_control_with_gravity_comp(self, target_positions, target_velocities=None):
    # Compute basic PD control
    torques = self.compute_control(target_positions, target_velocities)
    
    # Add gravity compensation
    gravity_torques = self.robot.compute_gravity_compensation()
    
    # Combine torques
    total_torques = torques + gravity_torques
    
    return total_torques
```

### PD Control with Feedforward Dynamics

```python
def compute_control_with_feedforward(self, target_positions, target_velocities, target_accelerations):
    # Compute basic PD control
    torques = self.compute_control(target_positions, target_velocities)
    
    # Add feedforward term using inverse dynamics
    M = self.robot.get_mass_matrix()
    feedforward_torques = np.dot(M, target_accelerations)
    
    # Combine torques
    total_torques = torques + feedforward_torques
    
    return total_torques
```

## Implementing Task-Specific PD Control

For specific tasks like pick-and-place, you may need to define trajectory-following PD control:

```python
import numpy as np
from scipy.interpolate import CubicSpline
import time

class TrajectoryPDController:
    def __init__(self, robot, kp, kd):
        self.robot = robot
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.controller = robot.get_articulation_controller()
        self.trajectory = None
        self.start_time = None
        
    def set_trajectory(self, waypoints, durations):
        """
        Set a trajectory through waypoints with specified durations.
        
        Args:
            waypoints: List of joint position arrays
            durations: List of times to reach each waypoint
        """
        # Create cubic spline for each joint
        cum_times = np.cumsum(durations)
        self.trajectory = []
        
        for joint_idx in range(len(waypoints[0])):
            joint_waypoints = [wp[joint_idx] for wp in waypoints]
            spline = CubicSpline([0] + list(cum_times), [joint_waypoints[0]] + joint_waypoints)
            self.trajectory.append(spline)
            
        self.duration = cum_times[-1]
        self.start_time = None
        
    def start(self):
        """Start following the trajectory."""
        self.start_time = time.time()
        
    def update(self):
        """Update the controller based on the current time."""
        if self.start_time is None or self.trajectory is None:
            return
            
        # Calculate time since trajectory start
        current_time = time.time() - self.start_time
        
        if current_time > self.duration:
            # Trajectory complete
            return False
            
        # Compute target positions and velocities from splines
        target_positions = []
        target_velocities = []
        
        for spline in self.trajectory:
            target_positions.append(spline(current_time))
            target_velocities.append(spline(current_time, 1))  # First derivative
            
        # Apply PD control
        torques = self.compute_control(target_positions, target_velocities)
        self.controller.apply_effort(torques)
        
        return True
        
    def compute_control(self, target_positions, target_velocities):
        # Get current joint states
        current_positions = self.robot.get_joint_positions()
        current_velocities = self.robot.get_joint_velocities()
        
        # Compute position and velocity errors
        position_error = np.array(target_positions) - np.array(current_positions)
        velocity_error = np.array(target_velocities) - np.array(current_velocities)
        
        # Compute PD control torques
        torques = self.kp * position_error + self.kd * velocity_error
        
        return torques
```

## Conclusion

PD control is a fundamental approach to controlling the Franka robot in Isaac Sim and Isaac Lab. By understanding the underlying principles and implementation details, you can create stable and precise control strategies for various robotic tasks.

The code snippets provided in this guide serve as a starting point for implementing PD control in your own applications. Remember to tune the PD gains carefully for your specific robot and task requirements.

## Next Steps

- Explore [Impedance Control](Impedance-Control) for compliant interaction with the environment
- Learn about [Inverse Kinematics](Inverse-Kinematics) for end-effector control
- Try implementing PD control in the [Basic Pick-and-Place Practice](Basic-Pick-and-Place) task

## References

- NVIDIA Isaac Sim Documentation: [[Articulation Controller API](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_api.html)](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_api.html)
- Franka Emika Control Parameters: [[Franka Control Interface](https://frankaemika.github.io/docs/control_parameters.html)](https://frankaemika.github.io/docs/control_parameters.html)
- [[Control of Robot Manipulators](https://studywolf.wordpress.com/2013/09/02/robot-control-pdpid-control/)](https://studywolf.wordpress.com/2013/09/02/robot-control-pdpid-control/) by Travis DeWolf
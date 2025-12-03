#!/usr/bin/env python3
"""Quick test script to verify the simulation runs without errors for a few frames."""

import sys
import pybullet as p
import pybullet_data
import time
import math
import os

# Add project dir to path
sys.path.insert(0, os.path.dirname(__file__))

from pid import PIDController
from robot_arm import RobotArm
from planner import avoid_point_obstacles
from logger_node_red import LoggerNodeRed
from perturbation import PerturbationTester

def test_basic():
    """Test basic functionality without GUI for faster feedback."""
    print("Testing basic setup...")

    # Setup logger
    logger = LoggerNodeRed(mqtt_broker=None, mqtt_topic=None, http_endpoint=None)

    # PyBullet init (no GUI for speed)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    plane = p.loadURDF("plane.urdf")
    print(f"✓ Plane loaded: {plane}")

    # Create URDF
    urdf_file = "planar_arm.urdf"
    if not os.path.exists(urdf_file):
        print(f"Creating {urdf_file}...")
        urdf_content = """
<?xml version="1.0"?>
<robot name="planar_arm">
  <material name="blue"><color rgba="0 0 0.8 1"/></material>
  <material name="red"><color rgba="0.8 0 0 1"/></material>

  <link name="base_link">
    <visual><geometry><cylinder length="0.1" radius="0.2"/></geometry><material name="blue"/></visual>
    <inertial><mass value="0"/></inertial>
  </link>

  <link name="link1">
    <visual>
      <geometry><box size="1.0 0.1 0.1"/></geometry>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
    <inertial><mass value="1"/></inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="5"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry><box size="1.0 0.1 0.1"/></geometry>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <inertial><mass value="1"/></inertial>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="5"/>
    <origin xyz="1.0 0 0"/>
  </joint>
</robot>
        """
        with open(urdf_file, "w") as f:
            f.write(urdf_content)

    # Load robot
    arm = RobotArm(urdf_file, base_position=(0, 0, 0), L1=1.0, L2=1.0)
    print(f"✓ Robot loaded: {arm.body}")

    # Test FK/IK
    q1_ref, q2_ref = arm.inverse_kinematics(1.0, 0.5)
    print(f"✓ IK computed: q1={q1_ref:.3f}, q2={q2_ref:.3f}")

    ee_x, ee_y = arm.forward_kinematics(q1_ref, q2_ref)
    print(f"✓ FK computed: x={ee_x:.3f}, y={ee_y:.3f}")

    # Test PID
    pid1 = PIDController(kp=60.0, ki=1.0, kd=5.0, output_limits=(-120, 120), windup_limit=5.0)
    u1, err1 = pid1.compute(q1_ref, 0.0, 1.0/240.0, angular=True)
    print(f"✓ PID computed: u1={u1:.3f}, err1={err1:.4f}")

    # Test perturbation
    perturb = PerturbationTester()
    payload_id = perturb.attach_payload_to_ee(arm, mass=0.3, radius=0.04)
    print(f"✓ Payload attached: {payload_id}")

    # Test collision checking
    obstacles = [{'pos': (0.8, 0.5), 'radius': 0.15}]
    is_free = arm.is_path_collision_free([q1_ref, q2_ref], obstacles, num_samples=5, safety_margin=0.08)
    print(f"✓ Collision check: path_free={is_free}")

    # Simulate 5 steps
    print("\nRunning 5 simulation steps...")
    for i in range(5):
        p.stepSimulation()
        jstates = arm.get_joint_states()
        print(f"  Step {i+1}: q1={jstates[0][0]:.4f}, q2={jstates[1][0]:.4f}")

    print("\n✓ All tests passed!")
    p.disconnect()

if __name__ == '__main__':
    try:
        test_basic()
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

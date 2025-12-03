import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import json
import os

from pid import PIDController
from robot_arm import RobotArm
from planner import avoid_point_obstacles
from logger_node_red import LoggerNodeRed
from perturbation import PerturbationTester

# Configuration
USE_MQTT = True
MQTT_BROKER = "test.mosquitto.org"
MQTT_TOPIC = "robotica/q1/logs"
HTTP_NODE_RED = None  # Example: "http://localhost:1880/endpoint"
ENABLE_PERTURBATION_TEST = False  # Set to True to add payload and test response


def create_robot_urdf():
  # keep existing URDF file if already present (user has planar_arm.urdf)
  if os.path.exists("planar_arm.urdf"):
    return
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
  with open("planar_arm.urdf", "w") as f:
    f.write(urdf_content)


def main():
  # Setup logger (MQTT + optional HTTP to Node-RED)
  logger = LoggerNodeRed(mqtt_broker=(MQTT_BROKER if USE_MQTT else None), mqtt_topic=MQTT_TOPIC, http_endpoint=HTTP_NODE_RED)

  # PyBullet init
  create_robot_urdf()
  p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.8)
  p.loadURDF("plane.urdf")

  # Create environment obstacles (point obstacles for simplicity)
  obstacles = [
    {'pos': (0.8, 0.5), 'radius': 0.15},
    {'pos': (1.2, 0.0), 'radius': 0.12}
  ]
  # Visualize obstacles
  obs_visuals = []
  for obs in obstacles:
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=obs['radius'], rgbaColor=[0.6, 0.2, 0.2, 0.6])
    body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[obs['pos'][0], obs['pos'][1], 0.1])
    obs_visuals.append(body)

  # Robot
  arm = RobotArm("planar_arm.urdf", base_position=(0, 0, 0), L1=1.0, L2=1.0)

  # Perturbation tester (optional payload)
  perturb = PerturbationTester()
  if ENABLE_PERTURBATION_TEST:
    perturb.attach_payload_to_ee(arm, mass=0.3, radius=0.04)
    print("Payload attached: 0.3 kg")

  # PID per joint
  pid1 = PIDController(kp=450.0, ki=5.0, kd=3.0, output_limits=(-200, 200), windup_limit=10.0)
  pid2 = PIDController(kp=450.0, ki=5.0, kd=3.0, output_limits=(-200, 200), windup_limit=10.0)

  # --- Real-time PID sliders (PyBullet user debug) ---
  kp1_slider = p.addUserDebugParameter('kp1', 0.0, 600.0, pid1.kp)
  ki1_slider = p.addUserDebugParameter('ki1', 0.0, 50.0, pid1.ki)
  kd1_slider = p.addUserDebugParameter('kd1', 0.0, 50.0, pid1.kd)

  kp2_slider = p.addUserDebugParameter('kp2', 0.0, 600.0, pid2.kp)
  ki2_slider = p.addUserDebugParameter('ki2', 0.0, 50.0, pid2.ki)
  kd2_slider = p.addUserDebugParameter('kd2', 0.0, 50.0, pid2.kd)

  # Buttons / toggles
  grasp_button = p.addUserDebugParameter('grasp_toggle', 0, 1, 0)
  release_button = p.addUserDebugParameter('release', 0, 1, 0)
  autotune_button = p.addUserDebugParameter('autotune', 0, 1, 0)

  # Target visual (sphere)
  target_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.06, rgbaColor=[0, 1, 0, 1])
  target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_vis, basePosition=[1.0, 0.5, 0.1])

  t = 0.0
  dt = 1.0 / 240.0

  # Metrics
  cumulative_error = 0.0
  steps = 0
  overshoot_max = 0.0
  energy_total = 0.0
  settling_time = None
  settle_tol = 0.03  # rad
  settle_required = 0.6  # seconds within tol to be considered settled
  within_since = None

  # Grasp / autotune state
  holding = False
  grasp_cid = None
  last_grasp_button_val = 0
  last_release_button_val = 0
  last_autotune_button_val = 0

  autotune_enabled = False
  tune_interval = 2.0
  last_tune_time = 0.0
  params = [pid1.kp, pid2.kp]
  dp = [max(1.0, 0.1 * pval) for pval in params]
  best_error = float('inf')
  tune_stage = 0
  param_index = 0
  error_window = []

  print("Iniciando simulação melhorada...")
  print(f"Target trajetória: circulo em x=1.0, y=0.5 com raio=0.4")
  print(f"Ganhos PID iniciais: kp1={pid1.kp}, kd1={pid1.kd}")
  print("")

  debug_step = 0
  while True:
    # moving target path (circle)
    raw_tx = 1.0 + 0.4 * math.cos(t * 0.6)
    raw_ty = 0.5 + 0.4 * math.sin(t * 0.6)

    # obstacle-aware target adjustment
    tx, ty = avoid_point_obstacles((raw_tx, raw_ty), obstacles, min_clearance=0.14)

    # update target visual
    p.resetBasePositionAndOrientation(target_body, [tx, ty, 0.1], [0, 0, 0, 1])

    # Read sliders and update PID gains in real time
    try:
      pid1.kp = p.readUserDebugParameter(kp1_slider)
      pid1.ki = p.readUserDebugParameter(ki1_slider)
      pid1.kd = p.readUserDebugParameter(kd1_slider)
      pid2.kp = p.readUserDebugParameter(kp2_slider)
      pid2.ki = p.readUserDebugParameter(ki2_slider)
      pid2.kd = p.readUserDebugParameter(kd2_slider)
    except Exception:
      pass

    # compute desired joint angles
    q1_ref, q2_ref = arm.inverse_kinematics(tx, ty)

    # Check collision-free path before executing
    collision_free = arm.is_path_collision_free([q1_ref, q2_ref], obstacles, num_samples=8, safety_margin=0.08)
    if not collision_free:
      # If collision detected, use last safe angles or stay put
      if debug_step % 240 == 0:
        print(f"[t={t:.1f}s] Collision detected! Path not safe")
      jstates = arm.get_joint_states()
      q1_ref, q2_ref = jstates[0][0], jstates[1][0]  # Keep current angles

    # read current joint states
    jstates = arm.get_joint_states()
    q1, dq1, tau1_applied = jstates[0]
    q2, dq2, tau2_applied = jstates[1]

    # PID compute
    u1, err1 = pid1.compute(q1_ref, q1, dt, angular=True)
    u2, err2 = pid2.compute(q2_ref, q2, dt, angular=True)

    # track metrics: overshoot is max absolute error from reference
    cumulative_error += abs(err1) + abs(err2)
    steps += 1
    debug_step += 1
    overshoot_max = max(overshoot_max, abs(err1), abs(err2))

    # apply torques
    arm.apply_torques([u1, u2])

    # Debug: print first few steps
    if debug_step <= 5:
      print(f"Step {debug_step}: target=({tx:.2f}, {ty:.2f}) | q_ref=({q1_ref:.3f}, {q2_ref:.3f}) | q_actual=({q1:.3f}, {q2:.3f}) | u=({u1:.2f}, {u2:.2f})")

    # Print progress every 240 steps (~2 seconds at 4x slower)
    if debug_step % 240 == 0:
      ee_x, ee_y = arm.end_effector_position()
      print(f"[t={t:.1f}s] EE=({ee_x:.3f}, {ee_y:.3f}) | Target=({tx:.3f}, {ty:.3f}) | Errors=({err1:.4f}, {err2:.4f})")

    # estimate instantaneous energy (torque * velocity)
    energy_total += (abs(u1 * dq1) + abs(u2 * dq2)) * dt

    # settling detection
    if abs(err1) < settle_tol and abs(err2) < settle_tol:
      if within_since is None:
        within_since = t
      elif settling_time is None and (t - within_since) >= settle_required:
        settling_time = within_since
    else:
      within_since = None

    # logging periodically
    if steps % 12 == 0:  # ~ every 0.05s
      payload = {
        'tempo': round(t, 3),
        'junta1': {'ref': round(q1_ref, 3), 'real': round(q1, 3), 'erro': round(err1, 4), 'torque': round(u1, 3)},
        'junta2': {'ref': round(q2_ref, 3), 'real': round(q2, 3), 'erro': round(err2, 4), 'torque': round(u2, 3)},
        'metrics': {
          'erro_medio': round(cumulative_error / max(1, steps), 4),
          'energia_total': round(energy_total, 4),
          'overshoot_max': round(overshoot_max, 4),
          'settling_time': (round(settling_time, 3) if settling_time is not None else None)
        },
        'target': {'x': round(tx, 3), 'y': round(ty, 3)}
      }
      logger.send(payload)

    # --- Grasping and Autotune handling ---
    ee_x, ee_y = arm.end_effector_position()
    dist_to_target = math.hypot(ee_x - tx, ee_y - ty)
    grasp_thresh = 0.06

    # read buttons (0/1)
    try:
      gv = int(p.readUserDebugParameter(grasp_button))
      rv = int(p.readUserDebugParameter(release_button))
      av = int(p.readUserDebugParameter(autotune_button))
    except Exception:
      gv = rv = av = 0

    # grasp button rising edge
    if gv == 1 and last_grasp_button_val == 0:
      if not holding and dist_to_target < 0.12:
        try:
          grasp_cid = arm.create_fixed_grasp(target_body, parent_link_index=1)
          holding = True
        except Exception:
          grasp_cid = None
    last_grasp_button_val = gv

    # release button rising edge
    if rv == 1 and last_release_button_val == 0:
      if holding and grasp_cid is not None:
        try:
          p.removeConstraint(grasp_cid)
        except Exception:
          pass
        holding = False
        grasp_cid = None
    last_release_button_val = rv

    # automatic grasp when close
    if (not holding) and (dist_to_target < grasp_thresh):
      try:
        grasp_cid = arm.create_fixed_grasp(target_body, parent_link_index=1)
        holding = True
      except Exception:
        grasp_cid = None

    # automatic release when reaching drop zone
    if holding and ee_x < 0.4:
      if grasp_cid is not None:
        try:
          p.removeConstraint(grasp_cid)
        except Exception:
          pass
      holding = False
      grasp_cid = None

    # autotune toggle
    if av == 1 and last_autotune_button_val == 0:
      autotune_enabled = not autotune_enabled
      print(f"Autotune enabled = {autotune_enabled}")
      last_tune_time = t
      error_window = []
      best_error = float('inf')
      params = [pid1.kp, pid2.kp]
      dp = [max(1.0, 0.1 * pval) for pval in params]
      param_index = 0
      tune_stage = 0
    last_autotune_button_val = av

    if autotune_enabled:
      error_window.append(abs(err1) + abs(err2))
      if (t - last_tune_time) >= tune_interval:
        avg_err = sum(error_window) / max(1, len(error_window))
        print(f"Tune eval time={t:.2f}, avg_err={avg_err:.4f}, params={params}, dp={dp}")
        if best_error == float('inf'):
          best_error = avg_err
        i = param_index
        if tune_stage == 0:
          params[i] += dp[i]
          tune_stage = 1
        elif tune_stage == 1:
          if avg_err < best_error:
            best_error = avg_err
            dp[i] *= 1.1
            param_index = (param_index + 1) % len(params)
            tune_stage = 0
          else:
            params[i] -= 2 * dp[i]
            tune_stage = 2
        elif tune_stage == 2:
          if avg_err < best_error:
            best_error = avg_err
            dp[i] *= 1.1
          else:
            params[i] += dp[i]
            dp[i] *= 0.9
          param_index = (param_index + 1) % len(params)
          tune_stage = 0

        pid1.kp = max(0.0, params[0])
        pid2.kp = max(0.0, params[1])
        error_window = []
        last_tune_time = t

    p.stepSimulation()
    time.sleep(dt * 1.5)  # Reduced slowdown (1.5x) for faster visible movement
    t += dt


if __name__ == '__main__':
  main()
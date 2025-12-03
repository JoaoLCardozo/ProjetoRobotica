import pybullet as p
import math
import numpy as np


class RobotArm:
    def __init__(self, urdf_path="planar_arm.urdf", base_position=(0, 0, 0), use_fixed_base=True, L1=1.0, L2=1.0):
        self.urdf_path = urdf_path
        self.base_position = base_position
        self.body = p.loadURDF(self.urdf_path, basePosition=list(self.base_position), useFixedBase=use_fixed_base)
        # assume two revolute joints in indices 0 and 1 for this simple URDF
        self.joint_indices = [0, 1]
        self.L1 = L1
        self.L2 = L2

    def get_joint_states(self):
        states = [p.getJointState(self.body, j) for j in self.joint_indices]
        # Each state: (position, velocity, reactionForces, appliedJointMotorTorque)
        return [(s[0], s[1], s[3]) for s in states]

    def apply_torques(self, torques):
        for j, tau in zip(self.joint_indices, torques):
            p.setJointMotorControl2(self.body, j, controlMode=p.TORQUE_CONTROL, force=float(tau))

    def forward_kinematics(self, q1, q2):
        # Compute end-effector (x, y) in planar coordinates
        x1 = self.L1 * math.cos(q1)
        y1 = self.L1 * math.sin(q1)
        x2 = x1 + self.L2 * math.cos(q1 + q2)
        y2 = y1 + self.L2 * math.sin(q1 + q2)
        return x2, y2

    def inverse_kinematics(self, x, y, elbow_up=True):
        """2-link IK with clamping to reachable workspace."""
        dist_sq = x * x + y * y
        max_reach = (self.L1 + self.L2)
        min_reach = abs(self.L1 - self.L2)

        r = math.sqrt(dist_sq)

        # Clamp to workspace
        if r > max_reach:
            scale = max_reach / r if r > 1e-6 else 1.0
            x = x * scale
            y = y * scale
            r = max_reach
        elif r < min_reach:
            scale = min_reach / r if r > 1e-6 else 1.0
            x = x * scale
            y = y * scale
            r = min_reach

        dist_sq = x * x + y * y

        # Law of cosines for q2
        cos_q2 = (dist_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_q2 = max(min(cos_q2, 1.0), -1.0)
        q2_raw = math.acos(cos_q2)
        q2 = q2_raw if elbow_up else -q2_raw

        # q1 from atan2
        k1 = self.L1 + self.L2 * math.cos(q2)
        k2 = self.L2 * math.sin(q2)
        q1 = math.atan2(y, x) - math.atan2(k2, k1)

        # Validate result
        if math.isnan(q1) or math.isnan(q2):
            return 0.0, 0.0

        return q1, q2

    def end_effector_position(self):
        jstates = self.get_joint_states()
        return self.forward_kinematics(jstates[0][0], jstates[1][0])

    def check_collision_with_point_obstacles(self, obstacles, safety_margin=0.05):
        """Check if end-effector collides with obstacles."""
        ee = self.end_effector_position()
        collisions = []
        for obs in obstacles:
            dx = ee[0] - obs['pos'][0]
            dy = ee[1] - obs['pos'][1]
            d = math.hypot(dx, dy)
            if d < obs['radius'] + safety_margin:
                collisions.append(obs)
        return collisions

    def is_path_collision_free(self, target_angles, obstacles, num_samples=10, safety_margin=0.05):
        """Check if linear path from current to target angles is collision-free."""
        current_states = self.get_joint_states()
        q_current = [current_states[0][0], current_states[1][0]]

        for i in range(num_samples + 1):
            alpha = i / max(1, num_samples)
            q_test = [q_current[j] + alpha * (target_angles[j] - q_current[j]) for j in range(2)]
            ee = self.forward_kinematics(q_test[0], q_test[1])

            for obs in obstacles:
                dx = ee[0] - obs['pos'][0]
                dy = ee[1] - obs['pos'][1]
                d = math.hypot(dx, dy)
                if d < obs['radius'] + safety_margin:
                    return False
        return True

    def create_fixed_grasp(self, object_id, parent_link_index=1):
        """Attach object rigidly to end-effector link."""
        try:
            cid = p.createConstraint(
                self.body, parent_link_index,
                object_id, -1,
                p.JOINT_FIXED,
                [0, 0, 0], [0, 0, 0], [0, 0, 0]
            )
            return cid
        except Exception as e:
            print(f"Grasp failed: {e}")
            return None

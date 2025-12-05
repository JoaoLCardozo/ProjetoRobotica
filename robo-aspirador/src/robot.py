"""Robô Aspirador com visual moderno."""

import pybullet as p
import numpy as np
import math


class VacuumRobot:
    """Robô aspirador diferencial."""

    def __init__(self, physics_client, start_pos=[0, 0], start_angle=0):
        self.client = physics_client
        self.start_pos = start_pos
        self.start_angle = start_angle

        # Parâmetros do robô
        self.radius = 0.15
        self.height = 0.06
        self.axle_length = 0.22

        # Sensores
        self.sensor_range = 1.0
        self.sensor_angles = [-90, -45, 0, 45, 90]

        # Velocidades
        self.max_velocity = 1.2
        self.max_angular_velocity = 3.5

        # Criar robô
        self.robot_id = self._create_robot()

        # Métricas
        self.energy_consumed = 0.0
        self.distance_traveled = 0.0
        self.last_position = np.array(start_pos)

    def _create_robot(self):
        """Cria o robô com visual moderno azul e branco."""
        # Corpo principal - azul escuro
        body_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.radius, height=self.height)
        body_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=self.radius, length=self.height,
                                          rgbaColor=[0.15, 0.35, 0.6, 1])

        robot_id = p.createMultiBody(
            baseMass=2.0,
            baseCollisionShapeIndex=body_collision,
            baseVisualShapeIndex=body_visual,
            basePosition=[self.start_pos[0], self.start_pos[1], self.height/2 + 0.005],
            baseOrientation=p.getQuaternionFromEuler([0, 0, self.start_angle])
        )

        # Física
        p.changeDynamics(robot_id, -1, lateralFriction=0.4, linearDamping=0.05, angularDamping=0.05)

        print(f"[ROBÔ] Criado em ({self.start_pos[0]:.1f}, {self.start_pos[1]:.1f})")
        return robot_id

    def get_sensor_readings(self):
        """Lê os sensores. Retorna distâncias normalizadas [0-1]."""
        readings = []
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        robot_angle = p.getEulerFromQuaternion(orn)[2]

        for angle_deg in self.sensor_angles:
            angle = robot_angle + math.radians(angle_deg)

            ray_start = [pos[0] + self.radius * math.cos(angle),
                        pos[1] + self.radius * math.sin(angle), pos[2] + 0.02]
            ray_end = [pos[0] + (self.radius + self.sensor_range) * math.cos(angle),
                      pos[1] + (self.radius + self.sensor_range) * math.sin(angle), pos[2] + 0.02]

            result = p.rayTest(ray_start, ray_end)
            if result[0][0] != -1 and result[0][0] != self.robot_id:
                distance = result[0][2] * self.sensor_range
            else:
                distance = self.sensor_range

            readings.append(distance / self.sensor_range)

        return np.array(readings)

    def get_pose(self):
        """Retorna posição (x, y) e orientação (theta)."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        return np.array([pos[0], pos[1]]), p.getEulerFromQuaternion(orn)[2]

    def set_wheel_velocities(self, left_vel, right_vel):
        """Define velocidades das rodas (-1 a 1)."""
        left_vel = np.clip(left_vel, -1, 1) * self.max_velocity
        right_vel = np.clip(right_vel, -1, 1) * self.max_velocity

        linear_vel = (left_vel + right_vel) / 2
        angular_vel = np.clip((right_vel - left_vel) / self.axle_length,
                              -self.max_angular_velocity, self.max_angular_velocity)

        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        theta = p.getEulerFromQuaternion(orn)[2]

        p.resetBaseVelocity(self.robot_id,
                           linearVelocity=[linear_vel * math.cos(theta), linear_vel * math.sin(theta), 0],
                           angularVelocity=[0, 0, angular_vel])

        self.energy_consumed += (abs(left_vel) + abs(right_vel)) * (1/240)

    def stop(self):
        """Para o robô."""
        self.set_wheel_velocities(0, 0)

    def update_metrics(self):
        """Atualiza distância percorrida."""
        pos, _ = self.get_pose()
        self.distance_traveled += np.linalg.norm(pos - self.last_position)
        self.last_position = pos.copy()

    def get_metrics(self):
        """Retorna métricas."""
        return {'energy_consumed': self.energy_consumed, 'distance_traveled': self.distance_traveled}

    def reset(self):
        """Reseta para posição inicial."""
        p.resetBasePositionAndOrientation(self.robot_id, [self.start_pos[0], self.start_pos[1], 0.0],
                                          p.getQuaternionFromEuler([0, 0, self.start_angle]))
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        self.energy_consumed = 0.0
        self.distance_traveled = 0.0
        self.last_position = np.array(self.start_pos)

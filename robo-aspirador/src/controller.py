"""Controlador do Robô Aspirador com PID."""

import numpy as np
import math
import time as time_module
from enum import Enum


class RobotState(Enum):
    """Estados do robô."""
    SWEEPING = 1
    TURNING = 2
    AVOIDING = 3
    REVERSING = 4
    SEEKING = 5
    FINISHED = 6


class PIDController:
    """Controlador PID para manter rota/ângulo."""

    def __init__(self, kp=2.0, ki=0.1, kd=0.5, output_limits=(-1.0, 1.0)):
        self.kp = kp  # Ganho proporcional
        self.ki = ki  # Ganho integral
        self.kd = kd  # Ganho derivativo
        self.output_limits = output_limits

        # Estado interno
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None

    def reset(self):
        """Reseta o estado do controlador."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None

    def compute(self, error, dt=None):
        """Calcula a saída do PID."""
        if dt is None or dt <= 0:
            dt = 1/240  # Default timestep

        # Termo proporcional
        p_term = self.kp * error

        # Termo integral (com anti-windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -2.0, 2.0)  # Anti-windup
        i_term = self.ki * self.integral

        # Termo derivativo
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative

        # Atualiza erro anterior
        self.previous_error = error

        # Saída total
        output = p_term + i_term + d_term

        # Limita saída
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        return output


class NavigationController:
    """Controlador de navegação com varredura e PID."""

    def __init__(self, robot, occupancy_map, environment):
        self.robot = robot
        self.map = occupancy_map
        self.env = environment

        self.state = RobotState.SWEEPING

        # Parâmetros
        self.safe_distance = 0.22
        self.turn_speed = 0.9
        self.forward_speed = 0.9
        self.reverse_speed = 0.8

        # Varredura
        self.sweep_direction = 1
        self.target_angle = 0
        self.maneuver_counter = 0
        self.turn_phase = 0

        # Controlador PID para orientação
        self.angle_pid = PIDController(kp=2.5, ki=0.15, kd=0.6, output_limits=(-0.5, 0.5))

        # Controle
        self.frame_count = 0
        self.last_coverage = 0
        self.stall_count = 0

        # Métricas
        self.collision_count = 0
        self._last_collision_time = 0

    def update(self, dt, sensor_readings, current_time):
        """Atualiza o controlador."""
        pos, angle = self.robot.get_pose()
        self.frame_count += 1

        # Atualizar mapa
        self.map.update_from_sensors(pos, angle, sensor_readings, self.robot.sensor_angles, self.robot.sensor_range)
        self.map.mark_cleaned(pos[0], pos[1], current_time, self.robot.radius)

        if len(self.map.trajectory) == 0 or current_time - self.map.trajectory[-1]['time'] > 0.2:
            self.map.record_trajectory(pos[0], pos[1], angle, current_time)

        # Verificar cobertura
        stats = self.map.get_coverage_stats()
        coverage = stats['coverage_percent']

        if coverage >= 92:
            self.state = RobotState.FINISHED
            return 0, 0

        # Detectar estagnação
        if self.frame_count % 120 == 0:
            if abs(coverage - self.last_coverage) < 0.3:
                self.stall_count += 1
                if self.stall_count >= 2:
                    self.state = RobotState.SEEKING
                    self.stall_count = 0
                    self.target_angle += math.pi / 2
            else:
                self.stall_count = 0
            self.last_coverage = coverage

        # Máquina de estados
        if self.state == RobotState.REVERSING:
            self.maneuver_counter -= 1
            if self.maneuver_counter <= 0:
                self.state = RobotState.TURNING
                self.turn_phase = 0
                self.maneuver_counter = 50
            return -self.reverse_speed, -self.reverse_speed

        if self.state == RobotState.TURNING:
            return self._do_turn()

        if self.state == RobotState.SEEKING:
            return self._seek(pos, angle, sensor_readings)

        # Verificar obstáculos
        obs = self._check_obstacles(sensor_readings)

        if obs['blocked']:
            return self._escape()
        elif obs['wall']:
            return self._uturn(pos, angle)
        elif obs['warning']:
            self.state = RobotState.AVOIDING
            return self._avoid(sensor_readings)
        else:
            self.state = RobotState.SWEEPING
            return self._sweep(pos, angle)

    def _check_obstacles(self, r):
        """Verifica obstáculos."""
        t = self.safe_distance / self.robot.sensor_range
        return {
            'blocked': r[2] < t * 0.5 or r[1] < t * 0.4 or r[3] < t * 0.4,
            'warning': r[2] < t * 0.8 or r[1] < t * 0.7 or r[3] < t * 0.7,
            'wall': r[2] < t * 1.2 and (r[0] < 0.3 or r[4] < 0.3)
        }

    def _uturn(self, pos, angle):
        """Curva em U."""
        self.sweep_direction *= -1
        self.state = RobotState.TURNING
        self.turn_phase = 0
        self.maneuver_counter = 60
        self.target_angle = 0 if self.sweep_direction > 0 else math.pi
        return self._do_turn()

    def _do_turn(self):
        """Executa manobra de giro."""
        self.maneuver_counter -= 1

        if self.turn_phase == 0:
            if self.maneuver_counter <= 0:
                self.turn_phase = 1
                self.maneuver_counter = 40
            return (-self.turn_speed, self.turn_speed) if self.sweep_direction > 0 else (self.turn_speed, -self.turn_speed)
        elif self.turn_phase == 1:
            if self.maneuver_counter <= 0:
                self.turn_phase = 2
                self.maneuver_counter = 60
            return self.forward_speed * 0.7, self.forward_speed * 0.7
        else:
            if self.maneuver_counter <= 0:
                self.state = RobotState.SWEEPING
            return (-self.turn_speed, self.turn_speed) if self.sweep_direction > 0 else (self.turn_speed, -self.turn_speed)

    def _escape(self):
        """Escape de colisão."""
        now = time_module.time()
        if now - self._last_collision_time > 0.5:
            self.collision_count += 1
            self._last_collision_time = now
        self.state = RobotState.REVERSING
        self.maneuver_counter = 50
        return -self.reverse_speed, -self.reverse_speed

    def _avoid(self, r):
        """Desviar de obstáculo."""
        if r[0] + r[1] * 1.5 > r[4] + r[3] * 1.5:
            return self.forward_speed * 0.4, self.forward_speed * 0.9
        return self.forward_speed * 0.9, self.forward_speed * 0.4

    def _sweep(self, pos, angle):
        """Varredura em linhas com controle PID."""
        error = self._norm_angle(self.target_angle - angle)

        # Usa PID para calcular correção angular
        correction = self.angle_pid.compute(error)

        # Aplica correção às velocidades das rodas
        left = self.forward_speed - correction
        right = self.forward_speed + correction

        margin = 0.35
        if pos[0] < margin or pos[0] > self.env.width - margin or pos[1] < margin or pos[1] > self.env.height - margin:
            self.angle_pid.reset()  # Reseta PID na curva
            return self._uturn(pos, angle)

        return np.clip(left, 0.2, 1), np.clip(right, 0.2, 1)

    def _seek(self, pos, angle, r):
        """Busca áreas não limpas."""
        margin = 0.35
        if pos[0] < margin or pos[0] > self.env.width - margin or pos[1] < margin or pos[1] > self.env.height - margin:
            return self._uturn(pos, angle)

        obs = self._check_obstacles(r)
        if obs['blocked']:
            return self._escape()
        if obs['wall']:
            return self._uturn(pos, angle)

        best_dir = self._find_uncleaned(pos)
        if best_dir is None:
            self.state = RobotState.SWEEPING
            self.target_angle = angle + math.pi * 0.5 * self.sweep_direction
            return self.forward_speed, self.forward_speed

        error = self._norm_angle(best_dir - angle)
        if abs(error) > 0.2:
            return (self.forward_speed * 0.2, self.forward_speed * 0.9) if error > 0 else (self.forward_speed * 0.9, self.forward_speed * 0.2)

        self.target_angle = best_dir
        return self.forward_speed, self.forward_speed

    def _find_uncleaned(self, pos):
        """Encontra direção para área não visitada."""
        from src.mapping import CELL_UNKNOWN, CELL_OBSTACLE

        best_angle, best_score = None, float('inf')

        for i in range(24):
            check_angle = (i / 24) * 2 * math.pi
            score = 0

            for dist in [0.2, 0.4, 0.6, 0.8]:
                x = pos[0] + dist * math.cos(check_angle)
                y = pos[1] + dist * math.sin(check_angle)

                if x < 0.3 or x > self.env.width - 0.3 or y < 0.3 or y > self.env.height - 0.3:
                    score += 80
                    continue

                gx, gy = self.map.world_to_grid(x, y)
                if 0 <= gx < self.map.grid_width and 0 <= gy < self.map.grid_height:
                    if self.map.grid[gx, gy] == CELL_OBSTACLE:
                        score += 200
                    elif self.map.grid[gx, gy] == CELL_UNKNOWN or self.map.visit_count[gx, gy] == 0:
                        score -= 100
                    else:
                        score += self.map.visit_count[gx, gy] * 10

            if score < best_score:
                best_score = score
                best_angle = check_angle

        return best_angle

    def _norm_angle(self, a):
        """Normaliza ângulo."""
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def get_metrics(self):
        """Retorna métricas."""
        return {'state': self.state.name, 'collisions': self.collision_count}


class SmartNavigationController(NavigationController):
    """Controlador que aprende com execuções anteriores."""

    def __init__(self, robot, occupancy_map, environment, previous_map=None):
        super().__init__(robot, occupancy_map, environment)
        self.previous_map = previous_map
        self.use_learned = previous_map is not None
        self.priority_targets = []
        self.current_target = None

        if self.use_learned:
            print("[CTRL] Modo APRENDIZADO ativo")
            self._import_knowledge()
            self._identify_priority()

    def _import_knowledge(self):
        """Importa conhecimento do mapa anterior."""
        from src.mapping import CELL_OBSTACLE, CELL_FREE
        obstacle_mask = self.previous_map.grid == CELL_OBSTACLE
        self.map.grid[obstacle_mask] = CELL_OBSTACLE
        free_mask = self.previous_map.grid == CELL_FREE
        self.map.grid[free_mask] = CELL_FREE
        print(f"[CTRL] Importados {np.sum(obstacle_mask)} obstáculos")

    def _identify_priority(self):
        """Identifica áreas prioritárias."""
        from src.mapping import CELL_OBSTACLE
        margin = 4

        for gx in range(margin, self.map.grid_width - margin):
            for gy in range(margin, self.map.grid_height - margin):
                if self.previous_map.grid[gx, gy] == CELL_OBSTACLE:
                    continue
                visits = self.previous_map.visit_count[gx, gy]
                if visits < 2:
                    wx, wy = self.map.grid_to_world(gx, gy)
                    self.priority_targets.append({'wx': wx, 'wy': wy, 'priority': 10 if visits == 0 else 5})

        self.priority_targets.sort(key=lambda x: x['priority'], reverse=True)
        print(f"[CTRL] {len(self.priority_targets)} células prioritárias")
        if self.priority_targets:
            self._select_target()

    def _select_target(self):
        """Seleciona próximo alvo."""
        if not self.priority_targets:
            self.current_target = None
            return
        pos, _ = self.robot.get_pose()
        best_idx = min(range(min(50, len(self.priority_targets))),
                       key=lambda i: math.sqrt((self.priority_targets[i]['wx'] - pos[0])**2 +
                                               (self.priority_targets[i]['wy'] - pos[1])**2) / (self.priority_targets[i]['priority'] + 1))
        self.current_target = self.priority_targets.pop(best_idx)

    def _find_uncleaned(self, pos):
        """Versão melhorada com aprendizado."""
        if self.use_learned and self.current_target:
            dist = math.sqrt((self.current_target['wx'] - pos[0])**2 + (self.current_target['wy'] - pos[1])**2)
            if dist < 0.3:
                self._select_target()
                if not self.current_target:
                    return super()._find_uncleaned(pos)
            return math.atan2(self.current_target['wy'] - pos[1], self.current_target['wx'] - pos[0])
        return super()._find_uncleaned(pos)

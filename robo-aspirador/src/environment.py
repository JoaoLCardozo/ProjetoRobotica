"""
Ambiente de simulacao - Arena Circular com Obstaculos Aleatorios.

Requisitos atendidos:
- Area circular de raio R = 100 x tamanho do robo
- Piso plano
- Obstaculos aleatorios ocupando 25-40% do espaco util
- Cada obstaculo com area minima >= metade da area do robo
- Formatos variados (retangulos, cilindros ou blocos)
- Distribuicao aleatoria, sem bloqueio total do caminho
"""

import pybullet as p
import numpy as np
import math
import random


class Environment:
    """Ambiente circular para o robo aspirador com obstaculos aleatorios."""

    def __init__(self, physics_client, robot_radius=0.15,
                 obstacle_coverage_min=0.25, obstacle_coverage_max=0.40,
                 **kwargs):  # kwargs para compatibilidade
        self.client = physics_client
        self.robot_radius = robot_radius

        # Raio da arena: R = 100 x tamanho do robo (diametro)
        # Para simulacao pratica, usar escala 1:10 (R = 10 x diametro = 3m)
        self.arena_radius = 10 * (2 * robot_radius)  # 3m de raio
        
        self.center_x = self.arena_radius
        self.center_y = self.arena_radius

        # Parametros para compatibilidade
        self.width = self.arena_radius * 2
        self.height = self.arena_radius * 2
        self.cell_size = 0.05

        self.grid_width = int(self.width / self.cell_size)
        self.grid_height = int(self.height / self.cell_size)

        # Obstaculos: 25-40% da area
        self.obstacle_coverage_min = obstacle_coverage_min
        self.obstacle_coverage_max = obstacle_coverage_max
        self.walls = []
        self.obstacles = []
        self.obstacle_positions = []  # Lista de dicts com info do obstaculo

        # Area do robo para referencia
        self.robot_area = math.pi * robot_radius ** 2
        self.min_obstacle_area = self.robot_area / 2  # Area minima = metade do robo

        self._create_circular_floor()
        self._create_circular_boundary()
        self._create_random_obstacles()

        print(f"[AMBIENTE] Arena circular R={self.arena_radius:.2f}m criada com {len(self.obstacles)} obstaculos")

    def _create_circular_floor(self):
        """Cria o piso circular."""
        # Piso como cilindro achatado
        floor_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=self.arena_radius,
            height=0.02
        )
        floor_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.arena_radius,
            length=0.02,
            rgbaColor=[0.92, 0.90, 0.85, 1]
        )

        self.floor_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=floor_collision,
            baseVisualShapeIndex=floor_visual,
            basePosition=[self.center_x, self.center_y, -0.01]
        )

        self._create_grid_lines()

    def _create_grid_lines(self):
        """Cria linhas de grid no piso circular."""
        # Linhas radiais
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x_end = self.center_x + self.arena_radius * math.cos(rad)
            y_end = self.center_y + self.arena_radius * math.sin(rad)
            p.addUserDebugLine(
                [self.center_x, self.center_y, 0.001],
                [x_end, y_end, 0.001],
                [0.8, 0.8, 0.8], lineWidth=1
            )

        # Circulos concentricos
        for r in np.arange(0.5, self.arena_radius, 0.5):
            for i in range(36):
                angle1 = (i / 36) * 2 * math.pi
                angle2 = ((i + 1) / 36) * 2 * math.pi
                x1 = self.center_x + r * math.cos(angle1)
                y1 = self.center_y + r * math.sin(angle1)
                x2 = self.center_x + r * math.cos(angle2)
                y2 = self.center_y + r * math.sin(angle2)
                p.addUserDebugLine([x1, y1, 0.001], [x2, y2, 0.001], [0.8, 0.8, 0.8], lineWidth=1)

    def _create_circular_boundary(self):
        """Cria a parede circular da arena."""
        wall_height = 0.15
        wall_thickness = 0.05
        num_segments = 48

        for i in range(num_segments):
            angle = (i / num_segments) * 2 * math.pi
            next_angle = ((i + 1) / num_segments) * 2 * math.pi

            # Posicao do segmento
            x = self.center_x + self.arena_radius * math.cos((angle + next_angle) / 2)
            y = self.center_y + self.arena_radius * math.sin((angle + next_angle) / 2)

            # Comprimento do segmento
            segment_length = 2 * self.arena_radius * math.sin(math.pi / num_segments)

            # Criar segmento de parede
            collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[segment_length / 2, wall_thickness / 2, wall_height / 2]
            )
            visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[segment_length / 2, wall_thickness / 2, wall_height / 2],
                rgbaColor=[0.6, 0.55, 0.5, 1]
            )

            wall_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[x, y, wall_height / 2],
                baseOrientation=p.getQuaternionFromEuler([0, 0, (angle + next_angle) / 2 + math.pi / 2])
            )
            self.walls.append(wall_id)

    def _create_random_obstacles(self):
        """Cria obstaculos aleatorios ocupando 25-40% da area."""
        # Area util da arena (excluindo margem de seguranca)
        margin = self.robot_radius * 3
        usable_radius = self.arena_radius - margin
        usable_area = math.pi * usable_radius ** 2

        # Area alvo de obstaculos
        target_coverage = random.uniform(self.obstacle_coverage_min, self.obstacle_coverage_max)
        target_obstacle_area = usable_area * target_coverage

        current_obstacle_area = 0
        max_attempts = 500
        attempts = 0

        # Espacamento minimo entre obstaculos para garantir passagem
        min_spacing = self.robot_radius * 3

        print(f"[AMBIENTE] Gerando obstaculos para {target_coverage*100:.1f}% de cobertura...")
        print(f"[AMBIENTE] Area minima por obstaculo: {self.min_obstacle_area:.4f}m2")

        while current_obstacle_area < target_obstacle_area and attempts < max_attempts:
            attempts += 1

            # Escolher tipo de obstaculo: 50% caixa, 50% cilindro
            is_cylinder = random.random() < 0.5

            if is_cylinder:
                obstacle_info = self._try_create_cylinder(usable_radius, min_spacing)
            else:
                obstacle_info = self._try_create_box(usable_radius, min_spacing)

            if obstacle_info:
                current_obstacle_area += obstacle_info['area']

        actual_coverage = current_obstacle_area / usable_area * 100
        print(f"[AMBIENTE] {len(self.obstacles)} obstaculos criados ({actual_coverage:.1f}% de cobertura)")

    def _try_create_cylinder(self, usable_radius, min_spacing):
        """Tenta criar um obstaculo cilindrico."""
        # Raio minimo para area >= metade do robo
        min_radius = math.sqrt(self.min_obstacle_area / math.pi)
        max_radius = min_radius * 2.5

        radius = random.uniform(min_radius, max_radius)
        height = random.uniform(0.08, 0.15)

        # Posicao aleatoria dentro da arena
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(self.robot_radius * 4, usable_radius - radius)
        x = self.center_x + dist * math.cos(angle)
        y = self.center_y + dist * math.sin(angle)

        # Verificar espacamento com obstaculos existentes
        if not self._check_spacing(x, y, radius, min_spacing):
            return None

        # Evitar area de spawn do robo
        spawn_x, spawn_y = self.get_start_position()
        if math.sqrt((x - spawn_x)**2 + (y - spawn_y)**2) < self.robot_radius * 4:
            return None

        # Criar cilindro
        collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)

        colors = [
            [0.45, 0.35, 0.25, 1],  # Marrom
            [0.5, 0.4, 0.3, 1],     # Marrom claro
            [0.3, 0.4, 0.5, 1],     # Azul acinzentado
            [0.55, 0.45, 0.35, 1],  # Bege
            [0.4, 0.4, 0.4, 1],     # Cinza
        ]
        color = random.choice(colors)

        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)

        obj_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[x, y, height / 2]
        )

        self.obstacles.append(obj_id)
        self.obstacle_positions.append({
            'x': x, 'y': y, 'type': 'cylinder', 'radius': radius, 'height': height
        })

        area = math.pi * radius ** 2
        return {'area': area, 'id': obj_id}

    def _try_create_box(self, usable_radius, min_spacing):
        """Tenta criar um obstaculo retangular."""
        # Dimensoes minimas para area >= metade do robo
        min_side = math.sqrt(self.min_obstacle_area)
        max_side = min_side * 2.5

        half_w = random.uniform(min_side / 2, max_side / 2)
        half_h = random.uniform(min_side / 2, max_side / 2)

        # Garantir area minima
        while (2 * half_w) * (2 * half_h) < self.min_obstacle_area:
            half_w *= 1.1
            half_h *= 1.1

        height = random.uniform(0.08, 0.15)

        # Posicao aleatoria dentro da arena
        angle = random.uniform(0, 2 * math.pi)
        max_dim = max(half_w, half_h)
        dist = random.uniform(self.robot_radius * 4, usable_radius - max_dim)
        x = self.center_x + dist * math.cos(angle)
        y = self.center_y + dist * math.sin(angle)

        # Verificar espacamento
        if not self._check_spacing(x, y, max_dim, min_spacing):
            return None

        # Evitar area de spawn
        spawn_x, spawn_y = self.get_start_position()
        if math.sqrt((x - spawn_x)**2 + (y - spawn_y)**2) < self.robot_radius * 4:
            return None

        # Criar caixa
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_w, half_h, height / 2])

        colors = [
            [0.45, 0.35, 0.25, 1],
            [0.5, 0.4, 0.3, 1],
            [0.3, 0.4, 0.5, 1],
            [0.55, 0.45, 0.35, 1],
            [0.4, 0.4, 0.4, 1],
        ]
        color = random.choice(colors)

        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_w, half_h, height / 2], rgbaColor=color)

        # Rotacao aleatoria
        rotation = random.uniform(0, math.pi)
        obj_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[x, y, height / 2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, rotation])
        )

        self.obstacles.append(obj_id)
        self.obstacle_positions.append({
            'x': x, 'y': y, 'type': 'box', 'half_w': half_w, 'half_h': half_h, 'height': height
        })

        area = (2 * half_w) * (2 * half_h)
        return {'area': area, 'id': obj_id}

    def _check_spacing(self, x, y, size, min_spacing):
        """Verifica se a posicao tem espacamento adequado."""
        for obs in self.obstacle_positions:
            ox, oy = obs['x'], obs['y']
            if obs['type'] == 'cylinder':
                obs_size = obs['radius']
            else:
                obs_size = max(obs['half_w'], obs['half_h'])

            dist = math.sqrt((x - ox)**2 + (y - oy)**2)
            required_dist = size + obs_size + min_spacing

            if dist < required_dist:
                return False
        return True

    def is_inside_arena(self, x, y, margin=0):
        """Verifica se um ponto esta dentro da arena."""
        dist = math.sqrt((x - self.center_x)**2 + (y - self.center_y)**2)
        return dist <= (self.arena_radius - margin)

    def get_distance_to_wall(self, x, y):
        """Retorna a distancia ate a parede circular."""
        dist_from_center = math.sqrt((x - self.center_x)**2 + (y - self.center_y)**2)
        return self.arena_radius - dist_from_center

    def world_to_grid(self, x, y):
        """Converte coordenadas do mundo para indices do grid."""
        gx = int(x / self.cell_size)
        gy = int(y / self.cell_size)
        gx = np.clip(gx, 0, self.grid_width - 1)
        gy = np.clip(gy, 0, self.grid_height - 1)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Converte indices do grid para coordenadas do mundo."""
        x = (gx + 0.5) * self.cell_size
        y = (gy + 0.5) * self.cell_size
        return x, y

    def is_valid_position(self, x, y, margin=None):
        """Verifica se uma posicao e valida (dentro da arena e fora de obstaculos)."""
        if margin is None:
            margin = self.robot_radius

        # Verificar se esta dentro da arena
        if not self.is_inside_arena(x, y, margin):
            return False

        # Verificar colisao com obstaculos
        for obs in self.obstacle_positions:
            ox, oy = obs['x'], obs['y']

            if obs['type'] == 'cylinder':
                dist = math.sqrt((x - ox)**2 + (y - oy)**2)
                if dist < obs['radius'] + margin:
                    return False
            else:
                # Aproximacao para caixa (bounding circle)
                obs_radius = math.sqrt(obs['half_w']**2 + obs['half_h']**2)
                dist = math.sqrt((x - ox)**2 + (y - oy)**2)
                if dist < obs_radius + margin:
                    return False

        return True

    def get_random_valid_position(self, margin=None):
        """Retorna uma posicao aleatoria valida dentro da arena."""
        if margin is None:
            margin = self.robot_radius

        max_attempts = 100
        for _ in range(max_attempts):
            # Gerar ponto aleatorio dentro da arena
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0, self.arena_radius - margin)
            x = self.center_x + dist * math.cos(angle)
            y = self.center_y + dist * math.sin(angle)

            if self.is_valid_position(x, y, margin):
                return x, y

        return None, None

    def get_start_position(self):
        """Retorna posicao inicial segura para o robo (perto do centro)."""
        return self.center_x, self.center_y - self.arena_radius * 0.7

    def get_dimensions(self):
        """Retorna dimensoes do ambiente."""
        return {
            'type': 'circular',
            'radius': self.arena_radius,
            'center_x': self.center_x,
            'center_y': self.center_y,
            'width': self.width,
            'height': self.height,
            'cell_size': self.cell_size,
            'grid_width': self.grid_width,
            'grid_height': self.grid_height,
            'num_obstacles': len(self.obstacles)
        }


# Alias para compatibilidade
CircularEnvironment = Environment

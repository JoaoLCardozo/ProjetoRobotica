"""Ambiente de simulação com paredes e obstáculos."""

import pybullet as p
import numpy as np


class Environment:
    """Ambiente para o robô aspirador - sala retangular simples."""

    def __init__(self, physics_client, width=2.5, height=2.5, cell_size=0.05):
        self.client = physics_client
        self.width = width
        self.height = height
        self.cell_size = cell_size

        self.grid_width = int(width / cell_size)
        self.grid_height = int(height / cell_size)

        self.walls = []
        self.obstacles = []
        self.obstacle_positions = []

        self._create_floor()
        self._create_walls()
        self._create_furniture()

        print(f"[AMBIENTE] {width}x{height}m criado")

    def _create_floor(self):
        """Cria o piso."""
        floor_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.width/2, self.height/2, 0.01])
        floor_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.width/2, self.height/2, 0.01],
                                           rgbaColor=[0.92, 0.90, 0.85, 1])

        self.floor_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision,
                                          baseVisualShapeIndex=floor_visual,
                                          basePosition=[self.width/2, self.height/2, -0.01])
        self._create_grid_lines()

    def _create_grid_lines(self):
        """Cria linhas de grid no piso."""
        for i in np.arange(0, self.width + 0.1, 0.5):
            p.addUserDebugLine([i, 0, 0.001], [i, self.height, 0.001], [0.8, 0.8, 0.8], lineWidth=1)
        for i in np.arange(0, self.height + 0.1, 0.5):
            p.addUserDebugLine([0, i, 0.001], [self.width, i, 0.001], [0.8, 0.8, 0.8], lineWidth=1)

    def _create_walls(self):
        """Cria as 4 paredes formando um quadrado fechado."""
        h = 0.15  # altura das paredes
        t = 0.025  # espessura das paredes
        cor = [0.6, 0.55, 0.5, 1]  # cor bege

        # Parede Sul (y = 0)
        self._add_wall([self.width/2, t/2, h/2], [self.width/2, t/2, h/2], cor)
        # Parede Norte (y = height)
        self._add_wall([self.width/2, self.height - t/2, h/2], [self.width/2, t/2, h/2], cor)
        # Parede Oeste (x = 0)
        self._add_wall([t/2, self.height/2, h/2], [t/2, self.height/2, h/2], cor)
        # Parede Leste (x = width)
        self._add_wall([self.width - t/2, self.height/2, h/2], [t/2, self.height/2, h/2], cor)

    def _add_wall(self, pos, size, color):
        """Adiciona uma parede."""
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        wall_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                                    baseVisualShapeIndex=visual, basePosition=pos)
        self.walls.append(wall_id)

    def _create_furniture(self):
        """Cria móveis simulando sofá, mesa de centro, poltrona e estante, para parecer mais com uma casa."""
        moveis = [
            # Sofá (retangular, canto superior)
            {'pos': [0.4, 2.0], 'size': [0.25, 0.12, 0.08], 'color': [0.45, 0.35, 0.25, 1]},
            # Mesa de centro (centro da sala)
            {'pos': [1.25, 1.25], 'size': [0.18, 0.12, 0.04], 'color': [0.5, 0.4, 0.3, 1]},
            # Poltrona (canto inferior direito)
            {'pos': [2.0, 0.5], 'size': [0.12, 0.12, 0.07], 'color': [0.3, 0.4, 0.5, 1]},
            # Estante/Rack (lado direito)
            {'pos': [2.1, 1.6], 'size': [0.08, 0.22, 0.1], 'color': [0.55, 0.45, 0.35, 1]},
        ]

        for m in moveis:
            pos, size, color = m['pos'], m['size'], m['color']
            collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
            obj_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                                       baseVisualShapeIndex=visual, basePosition=[pos[0], pos[1], size[2]])
            self.obstacles.append(obj_id)
            self.obstacle_positions.append((pos[0], pos[1], size[0], size[1]))

    def world_to_grid(self, x, y):
        """Converte coordenadas do mundo para índices do grid."""
        gx = int(x / self.cell_size)
        gy = int(y / self.cell_size)
        gx = np.clip(gx, 0, self.grid_width - 1)
        gy = np.clip(gy, 0, self.grid_height - 1)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Converte índices do grid para coordenadas do mundo."""
        x = (gx + 0.5) * self.cell_size
        y = (gy + 0.5) * self.cell_size
        return x, y

    def is_valid_position(self, x, y, margin=0.2):
        """Verifica se uma posição é válida."""
        if x < margin or x > self.width - margin:
            return False
        if y < margin or y > self.height - margin:
            return False

        for obs_x, obs_y, half_w, half_h in self.obstacle_positions:
            if (abs(x - obs_x) < half_w + margin and
                abs(y - obs_y) < half_h + margin):
                return False

        return True

    def get_dimensions(self):
        """Retorna dimensões do ambiente."""
        return {
            'width': self.width,
            'height': self.height,
            'cell_size': self.cell_size,
            'grid_width': self.grid_width,
            'grid_height': self.grid_height
        }

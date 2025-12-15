"""
Gerenciador de Sujeira Dinâmica.
Spawna pontos de sujeira ao longo do tempo e rastreia coleta.
"""

import pybullet as p
import numpy as np
import math
import random


class DirtPoint:
    """Representa um ponto de sujeira no ambiente."""

    def __init__(self, x, y, spawn_time, dirt_id, visual_id):
        self.x = x
        self.y = y
        self.spawn_time = spawn_time
        self.dirt_id = dirt_id  # ID do corpo no PyBullet
        self.visual_id = visual_id
        self.collected = False
        self.collection_time = None


class DirtManager:
    """Gerencia spawn e coleta de sujeira dinâmica."""

    def __init__(self, physics_client, environment, robot_radius=0.15):
        self.client = physics_client
        self.env = environment
        self.robot_radius = robot_radius

        # Parâmetros de spawn
        self.spawn_interval = 2.0  # segundos entre spawns
        self.spawn_duration = 120.0  # duração total do spawn (segundos)
        self.max_dirt_points = 60  # máximo de pontos

        # Distância de coleta: 1.2 × raio do robô
        self.collection_distance = 1.2 * robot_radius

        # Estado
        self.dirt_points = []
        self.last_spawn_time = 0.0
        self.total_spawned = 0
        self.total_collected = 0

        # Visual
        self.dirt_size = 0.03  # raio visual da sujeira
        self.dirt_color = [0.4, 0.25, 0.1, 1]  # marrom (sujeira)

        print(f"[SUJEIRA] Intervalo: {self.spawn_interval}s | Duração: {self.spawn_duration}s | Máx: {self.max_dirt_points}")

    def update(self, current_time, robot_pos):
        """Atualiza spawn e verifica coleta."""
        # Spawn de nova sujeira
        if current_time <= self.spawn_duration:
            if current_time - self.last_spawn_time >= self.spawn_interval:
                if self.total_spawned < self.max_dirt_points:
                    self._spawn_dirt(current_time)
                    self.last_spawn_time = current_time

        # Verificar coleta
        collected_now = self._check_collection(robot_pos, current_time)

        return collected_now

    def _spawn_dirt(self, current_time):
        """Spawna um novo ponto de sujeira em posição válida."""
        max_attempts = 50

        for _ in range(max_attempts):
            # Gerar posição aleatória dentro da arena
            x, y = self.env.get_random_valid_position(margin=self.robot_radius * 2)

            if x is None:
                continue

            # Verificar se não está muito perto de sujeira existente
            too_close = False
            for dirt in self.dirt_points:
                if not dirt.collected:
                    dist = math.sqrt((x - dirt.x)**2 + (y - dirt.y)**2)
                    if dist < self.robot_radius:
                        too_close = True
                        break

            if too_close:
                continue

            # Criar visual da sujeira
            visual_id = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=self.dirt_size,
                length=0.005,
                rgbaColor=self.dirt_color
            )

            dirt_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_id,
                basePosition=[x, y, 0.003]
            )

            dirt_point = DirtPoint(x, y, current_time, dirt_id, visual_id)
            self.dirt_points.append(dirt_point)
            self.total_spawned += 1

            print(f"[SUJEIRA] Spawn #{self.total_spawned} em ({x:.2f}, {y:.2f})")
            return True

        print("[SUJEIRA] Falha ao encontrar posição válida")
        return False

    def _check_collection(self, robot_pos, current_time):
        """Verifica se o robô coletou alguma sujeira."""
        collected = []

        for dirt in self.dirt_points:
            if dirt.collected:
                continue

            dist = math.sqrt((robot_pos[0] - dirt.x)**2 + (robot_pos[1] - dirt.y)**2)

            if dist <= self.collection_distance:
                dirt.collected = True
                dirt.collection_time = current_time
                self.total_collected += 1
                collected.append(dirt)

                # Mudar cor para verde (coletado)
                p.changeVisualShape(dirt.dirt_id, -1, rgbaColor=[0.2, 0.8, 0.2, 0.5])

                print(f"[SUJEIRA] Coletada! ({self.total_collected}/{self.total_spawned})")

        return collected

    def get_nearest_dirt(self, robot_pos):
        """Retorna a sujeira não coletada mais próxima."""
        nearest = None
        min_dist = float('inf')

        for dirt in self.dirt_points:
            if dirt.collected:
                continue

            dist = math.sqrt((robot_pos[0] - dirt.x)**2 + (robot_pos[1] - dirt.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = dirt

        return nearest, min_dist

    def get_oldest_uncollected(self):
        """Retorna a sujeira não coletada mais antiga (FIFO)."""
        for dirt in self.dirt_points:
            if not dirt.collected:
                return dirt
        return None

    def get_uncollected_count(self):
        """Retorna quantidade de sujeira não coletada."""
        return sum(1 for d in self.dirt_points if not d.collected)

    def get_stats(self):
        """Retorna estatísticas de sujeira."""
        uncollected = self.get_uncollected_count()

        # Tempo médio de coleta
        collection_times = [
            d.collection_time - d.spawn_time
            for d in self.dirt_points
            if d.collected
        ]
        avg_collection_time = np.mean(collection_times) if collection_times else 0

        return {
            'total_spawned': self.total_spawned,
            'total_collected': self.total_collected,
            'uncollected': uncollected,
            'collection_rate': self.total_collected / self.total_spawned if self.total_spawned > 0 else 0,
            'avg_collection_time': avg_collection_time
        }

    def get_all_uncollected_positions(self):
        """Retorna lista de posições de sujeira não coletada."""
        return [(d.x, d.y) for d in self.dirt_points if not d.collected]

    def cleanup(self):
        """Remove todos os objetos visuais."""
        for dirt in self.dirt_points:
            try:
                p.removeBody(dirt.dirt_id)
            except:
                pass
        self.dirt_points = []

"""
Simulação do Robô Aspirador - Sala Retangular com Sujeira Dinâmica.

Requisitos implementados:
- Sala retangular 2.5x2.5m (layout original)
- Obstáculos aleatórios pequenos (15-25% da área)
- Sujeira dinâmica: 1 ponto a cada 2s por 120s (máx 60 pontos)
- Tempo total de operação: 180s
- Velocidade máxima: Vmax = 0.5 × tamanho do robô
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import os
import sys
import json

sys.path.insert(0, os.path.dirname(__file__))

from src.robot import VacuumRobot
from src.environment import Environment
from src.mapping import OccupancyMap
from src.controller import NavigationController, SmartNavigationController, RobotState
from src.dirt_manager import DirtManager
from src import node_red_client

MAPS_DIR = os.path.join(os.path.dirname(__file__), 'saved_maps')
os.makedirs(MAPS_DIR, exist_ok=True)


def setup_pybullet(gui=True):
    """Configura o PyBullet."""
    client = p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)

    # Câmera centralizada na arena circular (centro em 3.0, 3.0)
    p.resetDebugVisualizerCamera(cameraDistance=7.0, cameraYaw=0, cameraPitch=-70,
                                  cameraTargetPosition=[3.0, 3.0, 0])
    return client


def run_execution(execution_num, total_executions, previous_map=None,
                  max_time=180, gui=True, send_to_node_red=True):
    """Executa uma sessão de limpeza com sujeira dinâmica."""
    print(f"\n{'='*50}")
    print(f"EXECUÇÃO {execution_num}/{total_executions}")
    print(f"{'='*50}")

    client = setup_pybullet(gui)

    # Criar ambiente retangular com obstáculos pequenos
    env = Environment(
        client,
        width=2.5,
        height=2.5,
        cell_size=0.05,
        robot_radius=0.15,
        obstacle_coverage_min=0.15,
        obstacle_coverage_max=0.25
    )

    # Mapa de ocupação
    occupancy_map = OccupancyMap(width=env.width, height=env.height, cell_size=env.cell_size)

    # Robô na posição inicial
    start_x, start_y = env.get_start_position()
    robot = VacuumRobot(client, start_pos=[start_x, start_y], start_angle=0.785)

    # Gerenciador de sujeira dinâmica
    dirt_manager = DirtManager(client, env, robot_radius=robot.radius)

    # Controlador com suporte a sujeira
    if previous_map is not None:
        controller = SmartNavigationController(robot, occupancy_map, env, previous_map, dirt_manager)
        print("[SIM] Usando controlador INTELIGENTE")
    else:
        controller = NavigationController(robot, occupancy_map, env, dirt_manager)
        print("[SIM] Usando controlador EXPLORATÓRIO")

    # Node-RED
    if send_to_node_red:
        nr_client = node_red_client.get_client()
        nr_client.send_start({
            'execution': execution_num,
            'total_executions': total_executions,
            'width': env.width,
            'height': env.height,
            'num_obstacles': len(env.obstacles)
        })

    sim_time = 0
    dt = 1/240
    last_log = 0
    start_real = time.time()
    simulation_interrupted = False

    print(f"[SIM] Iniciando... (máx {max_time}s)")
    print(f"[SIM] Sala: {env.width}x{env.height}m | Obstáculos: {len(env.obstacles)}")

    try:
        while sim_time < max_time:
            # Verificar se a conexão com PyBullet ainda está ativa
            if not p.isConnected(client):
                print("\n[SIM] Conexão com PyBullet perdida (janela fechada)")
                simulation_interrupted = True
                break

            # Atualizar sujeira dinâmica
            pos, _ = robot.get_pose()
            dirt_manager.update(sim_time, pos)

            # Controle do robô
            sensor_readings = robot.get_sensor_readings()
            left_vel, right_vel = controller.update(dt, sensor_readings, sim_time)
            robot.set_wheel_velocities(left_vel, right_vel)
            robot.update_metrics()
            p.stepSimulation()
            sim_time += dt

            # Log a cada 2s
            if sim_time - last_log >= 2.0:
                last_log = sim_time
                stats = occupancy_map.get_coverage_stats()
                metrics = robot.get_metrics()
                dirt_stats = dirt_manager.get_stats()

                print(f"[{sim_time:5.1f}s] Cobert: {stats['coverage_percent']:5.1f}% | "
                      f"Energia: {metrics['energy_consumed']:6.1f}J | "
                      f"Sujeira: {dirt_stats['total_collected']}/{dirt_stats['total_spawned']}")

                if send_to_node_red:
                    nr_client.send_periodic_update({
                        'execution': execution_num,
                        'coverage_percent': stats['coverage_percent'],
                        'time_elapsed': sim_time,
                        'energy': metrics['energy_consumed'],
                        'distance': metrics['distance_traveled'],
                        'state': controller.state.name,
                        'x': float(pos[0]),
                        'y': float(pos[1]),
                        'dirt_spawned': dirt_stats['total_spawned'],
                        'dirt_collected': dirt_stats['total_collected'],
                        'dirt_uncollected': dirt_stats['uncollected'],
                        'power_proxy': metrics.get('power_proxy', 0),
                        'collisions': metrics.get('collisions', 0)
                    })

            if gui:
                time.sleep(dt * 0.3)

    except p.error as e:
        print(f"\n[SIM] Erro PyBullet: {e}")
        simulation_interrupted = True

    # Métricas finais
    stats = occupancy_map.get_coverage_stats()
    metrics = robot.get_metrics()
    ctrl_metrics = controller.get_metrics()
    dirt_stats = dirt_manager.get_stats()

    final = {
        'execution': execution_num,
        'total_time': round(sim_time, 2),
        'coverage_percent': stats['coverage_percent'],
        'energy': round(metrics['energy_consumed'], 2),
        'distance': round(metrics['distance_traveled'], 2),
        'collisions': ctrl_metrics['collisions'],
        'dirt_spawned': dirt_stats['total_spawned'],
        'dirt_collected': dirt_stats['total_collected'],
        'dirt_collection_rate': round(dirt_stats['collection_rate'] * 100, 1),
        'avg_collection_time': round(dirt_stats['avg_collection_time'], 2)
    }

    if send_to_node_red:
        nr_client.send_end(final)

    occupancy_map.total_time = sim_time
    occupancy_map.collisions = ctrl_metrics['collisions']
    occupancy_map.save(os.path.join(MAPS_DIR, f'map_execution_{execution_num}.json'))

    print(f"\n{'─'*50}")
    print(f"RESUMO EXECUÇÃO {execution_num}:")
    print(f"  Cobertura: {stats['coverage_percent']:.1f}%")
    print(f"  Tempo: {sim_time:.1f}s")
    print(f"  Energia: {metrics['energy_consumed']:.1f}J")
    print(f"  Sujeira: {dirt_stats['total_collected']}/{dirt_stats['total_spawned']} ({dirt_stats['collection_rate']*100:.1f}%)")
    print(f"  Colisões: {ctrl_metrics['collisions']}")
    if simulation_interrupted:
        print(f"  [!] Simulação interrompida pelo usuário")
    print(f"{'─'*50}")

    # Limpar sujeira visual (somente se ainda conectado)
    if p.isConnected(client):
        dirt_manager.cleanup()
        p.disconnect()

    return occupancy_map, final


def run_multiple_executions(num_executions=3, max_time=180, gui=True, send_to_node_red=True):
    """Executa múltiplas sessões."""
    all_metrics = []
    previous_map = None

    for i in range(1, num_executions + 1):
        if i > 1:
            prev_path = os.path.join(MAPS_DIR, f'map_execution_{i-1}.json')
            if os.path.exists(prev_path):
                previous_map = OccupancyMap(2.5, 2.5, 0.05)
                previous_map.load(prev_path)

        final_map, metrics = run_execution(i, num_executions, previous_map, max_time, gui, send_to_node_red)
        all_metrics.append(metrics)
        previous_map = final_map

        if i < num_executions:
            print("\n⏳ Próxima execução em 2s...")
            time.sleep(2)

    # Comparativo
    print(f"\n{'='*60}")
    print("COMPARATIVO FINAL")
    print(f"{'='*60}")
    print(f"{'Exec':<6}{'Cobert%':<10}{'Tempo':<10}{'Energia':<10}{'Sujeira':<15}{'Colisões':<10}")
    print(f"{'-'*60}")
    for m in all_metrics:
        sujeira = f"{m['dirt_collected']}/{m['dirt_spawned']}"
        print(f"{m['execution']:<6}{m['coverage_percent']:<10.1f}{m['total_time']:<10.1f}{m['energy']:<10.1f}{sujeira:<15}{m['collisions']:<10}")

    if send_to_node_red:
        node_red_client.get_client().send_comparison(all_metrics)

    return all_metrics


def main():
    """Função principal."""
    parser = argparse.ArgumentParser(description='Robô Aspirador - Sujeira Dinâmica')
    parser.add_argument('--executions', '-e', type=int, default=1, help='Número de execuções')
    parser.add_argument('--time', '-t', type=int, default=180, help='Tempo máximo por execução (s)')
    parser.add_argument('--no-gui', action='store_true', help='Executar sem interface gráfica')
    parser.add_argument('--no-nodred', action='store_true', help='Não enviar para Node-RED')
    parser.add_argument('--clean', action='store_true', help='Limpar mapas salvos')
    args = parser.parse_args()

    if args.clean:
        import shutil
        if os.path.exists(MAPS_DIR):
            shutil.rmtree(MAPS_DIR)
            os.makedirs(MAPS_DIR)
            print("[SETUP] Mapas limpos")

    print("╔═══════════════════════════════════════════════════════╗")
    print("║    ROBÔ ASPIRADOR - SALA + SUJEIRA DINÂMICA           ║")
    print("╠═══════════════════════════════════════════════════════╣")
    print(f"║  Tempo: {args.time}s | Execuções: {args.executions}                       ║")
    print("╚═══════════════════════════════════════════════════════╝")

    all_metrics = run_multiple_executions(
        args.executions,
        args.time,
        not args.no_gui,
        not args.no_nodred
    )

    with open(os.path.join(MAPS_DIR, 'all_metrics.json'), 'w') as f:
        json.dump(all_metrics, f, indent=2)

    print("\n✅ Simulação completa!")
    return all_metrics


if __name__ == '__main__':
    main()

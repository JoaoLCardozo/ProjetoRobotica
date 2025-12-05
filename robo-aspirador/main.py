"""Simulação do Robô Aspirador."""

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

    p.resetDebugVisualizerCamera(cameraDistance=3.5, cameraYaw=0, cameraPitch=-70,
                                  cameraTargetPosition=[1.25, 1.25, 0])
    return client


def run_execution(execution_num, total_executions, previous_map=None,
                  max_time=120, gui=True, send_to_node_red=True):
    """Executa uma sessão de limpeza."""
    print(f"\n{'='*50}")
    print(f"EXECUÇÃO {execution_num}/{total_executions}")
    print(f"{'='*50}")

    client = setup_pybullet(gui)
    env = Environment(client, width=2.5, height=2.5, cell_size=0.05)
    occupancy_map = OccupancyMap(width=env.width, height=env.height, cell_size=env.cell_size)
    robot = VacuumRobot(client, start_pos=[0.4, 0.4], start_angle=0.785)

    # Controlador
    if previous_map is not None:
        controller = SmartNavigationController(robot, occupancy_map, env, previous_map)
        print("[SIM] Usando controlador INTELIGENTE")
    else:
        controller = NavigationController(robot, occupancy_map, env)
        print("[SIM] Usando controlador EXPLORATÓRIO")

    # Node-RED
    if send_to_node_red:
        nr_client = node_red_client.get_client()
        nr_client.send_start({'execution': execution_num, 'total_executions': total_executions,
                              'width': env.width, 'height': env.height})

    sim_time = 0
    dt = 1/240
    last_log = 0
    start_real = time.time()

    print(f"[SIM] Iniciando... (máx {max_time}s)")

    while sim_time < max_time:
        sensor_readings = robot.get_sensor_readings()
        left_vel, right_vel = controller.update(dt, sensor_readings, sim_time)
        robot.set_wheel_velocities(left_vel, right_vel)
        robot.update_metrics()
        p.stepSimulation()
        sim_time += dt

        if controller.state == RobotState.FINISHED:
            print(f"[SIM] ✓ Completo em {sim_time:.1f}s!")
            break

        # Log a cada 2s
        if sim_time - last_log >= 2.0:
            last_log = sim_time
            stats = occupancy_map.get_coverage_stats()
            metrics = robot.get_metrics()
            pos, _ = robot.get_pose()
            print(f"[{sim_time:5.1f}s] Cobert: {stats['coverage_percent']:5.1f}% | "
                  f"Energia: {metrics['energy_consumed']:6.1f}J")

            if send_to_node_red:
                nr_client.send_periodic_update({
                    'execution': execution_num, 'coverage_percent': stats['coverage_percent'],
                    'time_elapsed': sim_time, 'energy': metrics['energy_consumed'],
                    'distance': metrics['distance_traveled'], 'state': controller.state.name,
                    'x': float(pos[0]), 'y': float(pos[1])
                })

        if gui:
            time.sleep(dt * 0.3)

    # Métricas finais
    stats = occupancy_map.get_coverage_stats()
    metrics = robot.get_metrics()
    ctrl_metrics = controller.get_metrics()

    final = {
        'execution': execution_num,
        'total_time': round(sim_time, 2),
        'coverage_percent': stats['coverage_percent'],
        'energy': round(metrics['energy_consumed'], 2),
        'distance': round(metrics['distance_traveled'], 2),
        'collisions': ctrl_metrics['collisions']
    }

    if send_to_node_red:
        nr_client.send_end(final)

    occupancy_map.total_time = sim_time
    occupancy_map.collisions = ctrl_metrics['collisions']
    occupancy_map.save(os.path.join(MAPS_DIR, f'map_execution_{execution_num}.json'))

    print(f"\n{'─'*40}")
    print(f"RESUMO: Cobertura {stats['coverage_percent']:.1f}% | Tempo {sim_time:.1f}s | Energia {metrics['energy_consumed']:.1f}J")
    print(f"{'─'*40}")

    p.disconnect()
    return occupancy_map, final


def run_multiple_executions(num_executions=3, max_time=120, gui=True, send_to_node_red=True):
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
    print(f"\n{'='*50}")
    print("COMPARATIVO")
    print(f"{'='*50}")
    for m in all_metrics:
        print(f"Exec {m['execution']}: {m['coverage_percent']:.1f}% | {m['total_time']:.1f}s | {m['energy']:.1f}J")

    if send_to_node_red:
        node_red_client.get_client().send_comparison(all_metrics)

    return all_metrics


def main():
    """Função principal."""
    parser = argparse.ArgumentParser(description='Robô Aspirador')
    parser.add_argument('--executions', '-e', type=int, default=3)
    parser.add_argument('--time', '-t', type=int, default=90)
    parser.add_argument('--no-gui', action='store_true')
    parser.add_argument('--no-nodred', action='store_true')
    parser.add_argument('--clean', action='store_true')
    args = parser.parse_args()

    if args.clean:
        import shutil
        if os.path.exists(MAPS_DIR):
            shutil.rmtree(MAPS_DIR)
            os.makedirs(MAPS_DIR)
            print("[SETUP] Mapas limpos")

    print("╔═══════════════════════════════════════╗")
    print("║      ROBÔ ASPIRADOR INTELIGENTE       ║")
    print("╚═══════════════════════════════════════╝")

    all_metrics = run_multiple_executions(args.executions, args.time, not args.no_gui, not args.no_nodred)

    with open(os.path.join(MAPS_DIR, 'all_metrics.json'), 'w') as f:
        json.dump(all_metrics, f, indent=2)

    print("\n✅ Simulação completa!")
    return all_metrics


if __name__ == '__main__':
    main()

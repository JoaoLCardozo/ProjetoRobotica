import time
import math
import random
import sys
import requests
from collections import deque

import pybullet as p
import pybullet_data

GUI = True
SIM_DURATION_SEC = 180.0
DT = 1.0 / 240.0
LOG_HZ = 20
NODE_RED_URL = "https://team-igor-gusmao-igorgusmaogs2003-8ce0-37f8e15e.flowfuse.cloud/robotlog"

SIM_SPEEDUP = 50.0

TARGET_LIFETIME_SEC = 0.9
NEXT_TARGET_DELAY_SEC = 0.1

SENSOR_RADIUS = 1.5

EE_TOL = 0.15
SETTLE_CYCLES = 2

KP = 1800.0
KI = 0.0
KD = 180.0
TORQUE_LIMIT = 3200.0

APPROACH_SPEED = 30.0
JOINT_VEL_LIMIT = 18.0

IK_MAX_ITERS = 200
IK_RESIDUAL = 1e-4

R_WORKSPACE = 0.85

def limitar(x, lo, hi):
    return max(lo, min(hi, x))

def norma_vetor(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def enviar_seguro(url, payload, timeout=0.2):
    try:
        response = requests.post(url, json=payload, timeout=timeout)
        if response.status_code == 200:
            print(f"[NODE-RED] OK - Status: {response.status_code}")
        else:
            print(f"[NODE-RED] AVISO - Status: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"[NODE-RED] ERRO ao enviar: {e}")
    except Exception as e:
        print(f"[NODE-RED] ERRO inesperado: {e}")

def amostrar_ponto_na_esfera(raio_min, raio_max):
    while True:
        r = random.uniform(raio_min, raio_max)
        theta = random.uniform(0, 2 * math.pi)
        phi = random.uniform(0, math.pi)
        x = r * math.sin(phi) * math.cos(theta)
        y = r * math.sin(phi) * math.sin(theta)
        z = r * math.cos(phi)
        if 0.20 <= z <= 0.65:
            return (x, y, z)

print("Inicializando PyBullet...")
import warnings
warnings.filterwarnings("ignore")

if GUI:
    try:
        p.connect(p.GUI)
        print("[OK] Modo GUI ativado")
    except Exception as e:
        print(f"[AVISO] Não foi possível abrir GUI, usando modo DIRETO: {e}")
        p.connect(p.DIRECT)
        GUI = False
else:
    p.connect(p.DIRECT)
    print("[OK] Modo DIRETO ativado")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)
p.loadURDF("plane.urdf", useFixedBase=True)

robot_path = "kuka_iiwa/model.urdf"
try:
    robot_id = p.loadURDF(robot_path, basePosition=[0, 0, 0], useFixedBase=True)
except Exception as e:
    print("[ERRO] Não consegui carregar KUKA iiwa:", e)
    p.disconnect()
    sys.exit(1)

print("[OK] Robo carregado:", robot_path)

num_joints = p.getNumJoints(robot_id)
all_joints = [0, 1, 2, 3, 4, 5, 6]
report_joints = [0, 1, 2, 3, 4, 5]
num_dof_internal = 7
num_dof_report = 6
ee_link_index = 6

print("EE link index:", ee_link_index)
print("Juntas internas (7):", all_joints)
print("Juntas reportadas (6):", report_joints)

lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []
for j in all_joints:
    info = p.getJointInfo(robot_id, j)
    ll = info[8]
    ul = info[9]
    if ll > ul:
        ll, ul = ul, ll
    if ll < -10 or ul > 10:
        ll, ul = -2 * math.pi, 2 * math.pi
    lower_limits.append(ll)
    upper_limits.append(ul)
    joint_ranges.append(max(1e-3, ul - ll))
    rest_poses.append(0.0)

for j in all_joints:
    p.resetJointState(robot_id, j, 0.0, 0.0)
    p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)
    p.changeDynamics(robot_id, j, linearDamping=0.07, angularDamping=0.07)

LOCK_J7_POS = 0.0

integral = [0.0] * num_dof_report
prev_error = [0.0] * num_dof_report

def torque_pid(q, qd, q_ref, qd_ref):
    global integral, prev_error
    torques = []
    
    for i in range(num_dof_report):
        e_pos = q_ref[i] - q[i]
        e_vel = qd_ref[i] - qd[i]
        
        integral[i] += e_pos * DT
        integral[i] = limitar(integral[i], -0.10, 0.10)
        
        de_dt = (e_pos - prev_error[i]) / DT
        prev_error[i] = e_pos
        
        e_vel_filtrado = e_vel * 0.85
        derivativo = 0.6 * e_vel_filtrado + 0.4 * de_dt
        u = KP * e_pos + KI * integral[i] + KD * derivativo
        torques.append(limitar(u, -TORQUE_LIMIT, TORQUE_LIMIT))
    
    return torques

def ee_pos():
    return p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)[4]

def joint_states(indices):
    q, qd, tau = [], [], []
    for j in indices:
        s = p.getJointState(robot_id, j)
        q.append(s[0])
        qd.append(s[1])
        tau.append(s[3])
    return q, qd, tau

def ik_melhorado(pos):
    dist_origem = norma_vetor(pos)
    if dist_origem > R_WORKSPACE:
        fator = R_WORKSPACE / dist_origem
        pos = (pos[0] * fator, pos[1] * fator, pos[2] * fator)
    
    q_now, _, _ = joint_states(all_joints)
    
    try:
        q_sol = p.calculateInverseKinematics(
            robot_id,
            ee_link_index,
            pos,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=q_now,
            maxNumIterations=IK_MAX_ITERS,
            residualThreshold=IK_RESIDUAL
        )
        return [q_sol[j] for j in all_joints]
    except:
        return None

sensor_id = None
def criar_sensor():
    global sensor_id
    if sensor_id is not None:
        return
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=SENSOR_RADIUS, rgbaColor=[1, 1, 0, 0.25])
    sensor_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[0, 0, 0])

def atualizar_sensor(pos):
    if sensor_id is None:
        return
    p.resetBasePositionAndOrientation(sensor_id, [pos[0], pos[1], pos[2]], [0, 0, 0, 1])

target_id = None
texto_alvo_id = None
target_pos = None
target_spawn_t = None
target_deadline_t = None
next_spawn_t = 0.0

points_detected = 0
points_reached = 0
targets_generated = 0
MAX_TARGETS = 5
target_id_atual = None

dist_anterior_log = None
settle_counter = 0
move_start_t = None
time_to_settle = None
trajectory = deque(maxlen=3000)

def gerar_alvo(t_sim):
    global target_id, target_pos, target_spawn_t, target_deadline_t
    global settle_counter, move_start_t, time_to_settle
    global integral, prev_error
    global targets_generated, target_id_atual

    settle_counter = 0
    time_to_settle = None
    for i in range(num_dof_report):
        integral[i] = 0.0
        prev_error[i] = 0.0

    ee_atual = ee_pos()
    DIST_MINIMA_EE = 0.35
    DIST_MINIMA_BASE = 0.40

    max_tentativas = 300
    base_pos = [0, 0, 0]
    for _ in range(max_tentativas):
        x, y, z = amostrar_ponto_na_esfera(0.4 * R_WORKSPACE, 0.75 * R_WORKSPACE)
        dist_origem = math.sqrt(x*x + y*y + z*z)
        if dist_origem <= R_WORKSPACE:
            dist_ee = math.sqrt((x - ee_atual[0])**2 + (y - ee_atual[1])**2 + (z - ee_atual[2])**2)
            dist_base = math.sqrt((x - base_pos[0])**2 + (y - base_pos[1])**2 + (z - base_pos[2])**2)
            if dist_ee >= DIST_MINIMA_EE and dist_base >= DIST_MINIMA_BASE:
                target_pos = (x, y, z)
                break
    else:
        r_safe = 0.65 * R_WORKSPACE
        base_pos = [0, 0, 0]
        for tentativa in range(100):
            theta = random.uniform(0, 2 * math.pi)
            phi = random.uniform(0, math.pi)
            x = r_safe * math.sin(phi) * math.cos(theta)
            y = r_safe * math.sin(phi) * math.sin(theta)
            z = r_safe * math.cos(phi)
            if 0.25 <= z <= 0.60:
                dist_ee = math.sqrt((x - ee_atual[0])**2 + (y - ee_atual[1])**2 + (z - ee_atual[2])**2)
                dist_base = math.sqrt((x - base_pos[0])**2 + (y - base_pos[1])**2 + (z - base_pos[2])**2)
                if dist_ee >= DIST_MINIMA_EE and dist_base >= DIST_MINIMA_BASE:
                    target_pos = (x, y, z)
                    break
        else:
            dist_ee_norm = norma_vetor(ee_atual)
            if dist_ee_norm > 0.15:
                dir_oposta = (-ee_atual[0]/dist_ee_norm, -ee_atual[1]/dist_ee_norm, -ee_atual[2]/dist_ee_norm)
                target_pos = (dir_oposta[0] * r_safe, dir_oposta[1] * r_safe, dir_oposta[2] * r_safe)
            else:
                theta = random.uniform(0, 2 * math.pi)
                phi = random.uniform(math.pi/6, math.pi/2)
                target_pos = (r_safe * math.sin(phi) * math.cos(theta),
                             r_safe * math.sin(phi) * math.sin(theta),
                             r_safe * math.cos(phi))

    target_spawn_t = t_sim
    target_deadline_t = t_sim + TARGET_LIFETIME_SEC
    targets_generated += 1
    target_id_atual = f"A{targets_generated}"
    print(f"[DEBUG] target_spawn_t={target_spawn_t:.2f}, target_deadline_t={target_deadline_t:.2f}, diferença={target_deadline_t - target_spawn_t:.2f}s")
    print(f"[INFO] Alvo {targets_generated}/{MAX_TARGETS} gerado")
    
    success_rate = (points_reached/points_detected*100) if points_detected > 0 else 0.0
    time_left = TARGET_LIFETIME_SEC
    ee_atual = ee_pos()
    ee_err = norma_vetor((ee_atual[0]-target_pos[0], ee_atual[1]-target_pos[1], ee_atual[2]-target_pos[2]))
    
    payload_spawn = {
        "type": "target_spawn",
        "t": float(t_sim),
        "state": EST_GERAR,
        "target": {
            "id": target_id_atual,
            "pos": [float(target_pos[0]), float(target_pos[1]), float(target_pos[2])],
            "lifetime_sec": float(TARGET_LIFETIME_SEC)
        },
        "counts": {
            "detected": int(points_detected),
            "reached": int(points_reached),
            "success_rate": float(success_rate)
        },
        "metrics": {
            "time_left_sec": float(time_left),
            "ee_error_m": float(ee_err)
        }
    }
    enviar_seguro(NODE_RED_URL, payload_spawn)
    print(f"[NODE-RED] Enviado target_spawn para {target_id_atual}")

    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.06, rgbaColor=[1, 0, 0, 1])
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.06)
    target_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=list(target_pos),
    )
    move_start_t = t_sim

def remover_alvo():
    global target_id, texto_alvo_id, target_pos, target_spawn_t, target_deadline_t
    if target_id is not None:
        p.removeBody(target_id)
    if texto_alvo_id is not None and GUI:
        p.removeUserDebugItem(texto_alvo_id)

    target_id = None
    texto_alvo_id = None
    target_pos = None
    target_spawn_t = None
    target_deadline_t = None

def atualizar_texto_alvo(t_sim):
    global texto_alvo_id
    if not GUI or target_pos is None or target_deadline_t is None:
        return
    
    if not p.isConnected():
        return

    texto_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
    if texto_alvo_id is not None:
        try:
            p.removeUserDebugItem(texto_alvo_id)
        except:
            pass

    tempo_restante = target_deadline_t - t_sim
    if tempo_restante > 0:
        try:
            texto_alvo_id = p.addUserDebugText(
                f"ALVO ({tempo_restante:.1f}s)",
                texto_pos, textColorRGB=[1, 0, 0], textSize=2.0
            )
        except:
            pass

EST_GERAR = "GERAR"
EST_ESPERAR_DETECCAO = "ESPERAR_DETECCAO"
EST_MOVER = "MOVER"
EST_SEG = "SEGURAR"
EST_PARAR = "PARAR"

estado = EST_PARAR
t_ent = 0.0

def entrar(novo, t_sim):
    global estado, t_ent
    estado = novo
    t_ent = t_sim

def t_estado(t_sim):
    return t_sim - t_ent

print("\n=== INICIANDO SIMULAÇÃO ===")
print("- Controle melhorado com IK direto")
print("- Tolerâncias ajustadas para melhor performance")
print("=" * 60)

criar_sensor()

t_sim = 0.0
ultimo_log = 0.0
ultimo_dbg = 0.0
sim_start_real = time.time()

q_ref_atual = [0.0] * num_dof_report
movimento_ativo = False

try:
    while True:
        t_sim += DT
        if t_sim >= SIM_DURATION_SEC or (targets_generated >= MAX_TARGETS and target_pos is None):
            break

        ee = ee_pos()
        q6, qd6, tau6 = joint_states(report_joints)

        atualizar_sensor(ee)
        atualizar_texto_alvo(t_sim)

        if target_deadline_t is not None and t_sim >= target_deadline_t:
            tempo_vida = t_sim - target_spawn_t if target_spawn_t is not None else 0.0
            print(f"[{t_sim:.2f}s] [EXPIRADO] Alvo expirou após {tempo_vida:.2f}s (esperado {TARGET_LIFETIME_SEC}s) -> PARANDO")
            
            success_rate = (points_reached/points_detected*100) if points_detected > 0 else 0.0
            ee_err_final = norma_vetor((ee[0]-target_pos[0], ee[1]-target_pos[1], ee[2]-target_pos[2])) if target_pos is not None else None
            
            payload_expired = {
                "type": "target_expired",
                "t": float(t_sim),
                "state": EST_PARAR,
                "target": {
                    "id": target_id_atual if target_id_atual else "A0",
                    "pos": [float(target_pos[0]), float(target_pos[1]), float(target_pos[2])] if target_pos else None
                },
                "counts": {
                    "detected": int(points_detected),
                    "reached": int(points_reached),
                    "success_rate": float(success_rate)
                },
                "metrics": {
                    "ee_error_m": float(ee_err_final) if ee_err_final is not None else None
                }
            }
            enviar_seguro(NODE_RED_URL, payload_expired)
            print(f"[NODE-RED] Enviado target_expired para {target_id_atual if target_id_atual else 'A0'}")
            
            for j in all_joints:
                p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, 
                                      targetVelocity=0, force=200)

            remover_alvo()
            for i in range(num_dof_report):
                integral[i] = 0.0
                prev_error[i] = 0.0
            entrar(EST_PARAR, t_sim)
            next_spawn_t = t_sim + NEXT_TARGET_DELAY_SEC
            movimento_ativo = False
            print(f"[{t_sim:.2f}s] [EXPIRADO] Próximo alvo será gerado em t_sim={next_spawn_t:.2f}s")
            p.stepSimulation()
            if GUI:
                time.sleep(DT / max(1.0, SIM_SPEEDUP))
            continue

        if target_pos is None and t_sim >= next_spawn_t and targets_generated < MAX_TARGETS:
            gerar_alvo(t_sim)
            dist0 = norma_vetor((ee[0]-target_pos[0], ee[1]-target_pos[1], ee[2]-target_pos[2]))
            tempo_vida_esperado = target_deadline_t - target_spawn_t if target_spawn_t is not None else TARGET_LIFETIME_SEC
            print(f"\n[{t_sim:.2f}s] [ALVO] *** NOVO ALVO GERADO! *** {target_pos} | dist inicial={dist0*1000:.1f}mm | expira em t_sim={target_deadline_t:.2f}s (vida={tempo_vida_esperado:.2f}s) | estado={estado}")
            if dist0 <= SENSOR_RADIUS:
                points_detected += 1
                print(f"[{t_sim:.2f}s] [SENSOR] DETECTOU alvo! (dist={dist0*1000:.1f}mm <= {SENSOR_RADIUS*1000:.0f}mm) -> INDO pro alvo")
                entrar(EST_MOVER, t_sim)
                movimento_ativo = True
            else:
                entrar(EST_ESPERAR_DETECCAO, t_sim)
        
        elif estado == EST_ESPERAR_DETECCAO:
            if target_pos is None:
                entrar(EST_PARAR, t_sim)
            else:
                dist_sensor = norma_vetor((ee[0]-target_pos[0], ee[1]-target_pos[1], ee[2]-target_pos[2]))
                if dist_sensor <= SENSOR_RADIUS:
                    points_detected += 1
                    print(f"[{t_sim:.2f}s] [SENSOR] DETECTOU alvo! (dist={dist_sensor*1000:.1f}mm) -> INDO pro alvo")
                    entrar(EST_MOVER, t_sim)
                    movimento_ativo = True

        elif estado == EST_MOVER:
            if target_pos is None:
                entrar(EST_PARAR, t_sim)
                movimento_ativo = False
            else:
                dx = target_pos[0] - ee[0]
                dy = target_pos[1] - ee[1]
                dz = target_pos[2] - ee[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if dist < 0.18:
                    goal = target_pos
                elif dist < 0.35:
                    step = min(dist * 0.4, APPROACH_SPEED * DT * 1.2)
                    if dist > 1e-6:
                        ux, uy, uz = dx/dist, dy/dist, dz/dist
                        goal = (ee[0] + ux*step, ee[1] + uy*step, ee[2] + uz*step)
                    else:
                        goal = target_pos
                else:
                    fator_step = 1.8 if dist > 0.70 else 1.6 if dist > 0.50 else 1.4 if dist > 0.35 else 1.2
                    step = min(dist * 0.3, APPROACH_SPEED * DT * fator_step)
                    if dist > 1e-6:
                        ux, uy, uz = dx/dist, dy/dist, dz/dist
                        goal = (ee[0] + ux*step, ee[1] + uy*step, ee[2] + uz*step)
                        dist_goal = norma_vetor(goal)
                        if dist_goal > R_WORKSPACE:
                            fator = R_WORKSPACE / dist_goal
                            goal = (goal[0] * fator, goal[1] * fator, goal[2] * fator)
                    else:
                        goal = target_pos
                
                q7_ref = ik_melhorado(goal)
                
                if q7_ref is None:
                    q7_ref = ik_melhorado(target_pos)
                    if q7_ref is None:
                        goal_intermediario = ((ee[0] + target_pos[0])/2, 
                                            (ee[1] + target_pos[1])/2, 
                                            (ee[2] + target_pos[2])/2)
                        q7_ref = ik_melhorado(goal_intermediario)
                        if q7_ref is None and (t_sim - ultimo_dbg) >= 0.5:
                            print(f"[{t_sim:.2f}s] [ERRO] IK falhou para goal, target_pos E intermediário! dist={dist*1000:.1f}mm")
                            ultimo_dbg = t_sim
                
                if q7_ref is not None:
                    q_ref_atual = [q7_ref[j] for j in report_joints]
                    
                    p.setJointMotorControl2(robot_id, 6, p.POSITION_CONTROL, 
                                          targetPosition=LOCK_J7_POS, force=100, maxVelocity=2.0)
                    
                    qd_ref = []
                    max_erro_junta = 0.0
                    for i in range(num_dof_report):
                        e = q_ref_atual[i] - q6[i]
                        max_erro_junta = max(max_erro_junta, abs(e))
                        ganho_vel = 85.0 if abs(e) > 0.5 else 70.0 if abs(e) > 0.3 else 60.0 if abs(e) > 0.15 else 50.0
                        v = limitar(ganho_vel * e, -JOINT_VEL_LIMIT, JOINT_VEL_LIMIT)
                        qd_ref.append(v)
                    
                    torques = torque_pid(q6, qd6, q_ref_atual, qd_ref)
                    max_torque = max([abs(t) for t in torques])
                    for i, j in enumerate(report_joints):
                        p.setJointMotorControl2(robot_id, j, p.TORQUE_CONTROL, force=torques[i])
                else:
                    print(f"[{t_sim:.2f}s] [ERRO CRITICO] IK falhou completamente, parando movimento")
                    for j in report_joints:
                        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, targetVelocity=0, force=200)
                
                dist_final = norma_vetor((ee[0]-target_pos[0], ee[1]-target_pos[1], ee[2]-target_pos[2]))
                
                if (t_sim - ultimo_dbg) >= 0.3:
                    dentro_tol = "SIM" if dist_final <= EE_TOL else "NAO"
                    if dist_anterior_log is None:
                        vel_aprox_real = 0.0
                        dist_anterior_log = dist_final
                    else:
                        vel_aprox_real = abs(dist_anterior_log - dist_final) / DT if DT > 0 else 0
                        dist_anterior_log = dist_final
                    if q7_ref is not None and 'q_ref_atual' in locals():
                        max_erro_junta = max([abs(q_ref_atual[i] - q6[i]) for i in range(num_dof_report)])
                        max_torque_val = max([abs(t) for t in torques]) if 'torques' in locals() else 0
                        print(f"[{t_sim:.2f}s] [MOV] erro={dist_final*1000:.0f}mm | dentro_tol={dentro_tol} | settle={settle_counter}/{SETTLE_CYCLES} | vel_aprox={vel_aprox_real*1000:.0f}mm/s | max_erro_junta={max_erro_junta:.3f}rad | max_torque={max_torque_val:.0f}N")
                    else:
                        print(f"[{t_sim:.2f}s] [MOV] erro={dist_final*1000:.0f}mm | IK=FALHOU | dist={dist*1000:.0f}mm")
                    ultimo_dbg = t_sim

                dentro_tolerancia = dist_final <= EE_TOL
                
                if dentro_tolerancia:
                    settle_counter += 1
                    if settle_counter >= SETTLE_CYCLES:
                        points_reached += 1
                        time_to_settle = t_sim - move_start_t if move_start_t else None
                        time_to_reach = t_sim - target_spawn_t if target_spawn_t is not None else None
                        print(f"[{t_sim:.2f}s] [SUCESSO] ALCANÇOU! erro={dist_final*1000:.1f}mm | tempo={time_to_settle:.2f}s | dentro da tolerância de {EE_TOL*1000:.0f}mm")
                        
                        success_rate = (points_reached/points_detected*100) if points_detected > 0 else 0.0
                        
                        payload_reached = {
                            "type": "target_reached",
                            "t": float(t_sim),
                            "state": EST_SEG,
                            "target": {
                                "id": target_id_atual if target_id_atual else "A0",
                                "pos": [float(target_pos[0]), float(target_pos[1]), float(target_pos[2])]
                            },
                            "counts": {
                                "detected": int(points_detected),
                                "reached": int(points_reached),
                                "success_rate": float(success_rate)
                            },
                            "metrics": {
                                "time_to_reach_sec": float(time_to_reach) if time_to_reach is not None else None,
                                "ee_error_m": float(dist_final)
                            }
                        }
                        enviar_seguro(NODE_RED_URL, payload_reached)
                        print(f"[NODE-RED] Enviado target_reached para {target_id_atual if target_id_atual else 'A0'}")
                        
                        for i in range(num_dof_report):
                            integral[i] = 0.0
                            prev_error[i] = 0.0
                        entrar(EST_SEG, t_sim)
                        movimento_ativo = False
                        settle_counter = 0
                else:
                    if settle_counter > 0:
                        settle_counter = 0

        elif estado == EST_SEG:
            for i, j in enumerate(report_joints):
                p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, 
                                      targetPosition=q6[i], force=300, maxVelocity=1.0)
            p.setJointMotorControl2(robot_id, 6, p.POSITION_CONTROL, 
                                  targetPosition=LOCK_J7_POS, force=100, maxVelocity=1.0)

            if t_estado(t_sim) >= NEXT_TARGET_DELAY_SEC:
                print(f"[{t_sim:.2f}s] [SEG] Removendo alvo após {NEXT_TARGET_DELAY_SEC}s de segurar")
                remover_alvo()
                next_spawn_t = t_sim + NEXT_TARGET_DELAY_SEC
                entrar(EST_PARAR, t_sim)
                print(f"[{t_sim:.2f}s] [SEG] Próximo alvo será gerado em t_sim={next_spawn_t:.2f}s")

        elif estado == EST_PARAR:
            for j in report_joints:
                p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, 
                                      targetVelocity=0, force=150)
            p.setJointMotorControl2(robot_id, 6, p.POSITION_CONTROL, 
                                  targetPosition=LOCK_J7_POS, force=100, maxVelocity=1.0)

        trajectory.append((float(ee[0]), float(ee[1]), float(ee[2])))

        if (t_sim - ultimo_log) >= (1.0 / LOG_HZ):
            ultimo_log = t_sim
            ee_err = None
            if target_pos is not None:
                ee_err = norma_vetor((ee[0]-target_pos[0], ee[1]-target_pos[1], ee[2]-target_pos[2]))

            if target_pos is not None:
                success_rate = (points_reached/points_detected*100) if points_detected > 0 else 0.0
                
                time_left = None
                if target_deadline_t is not None:
                    time_left = max(0.0, target_deadline_t - t_sim)
                
                payload_status = {
                    "type": "status",
                    "t": float(t_sim),
                    "state": estado,
                    "target": {
                        "id": target_id_atual if target_id_atual else "A0",
                        "pos": [float(target_pos[0]), float(target_pos[1]), float(target_pos[2])]
                    },
                    "counts": {
                        "detected": int(points_detected),
                        "reached": int(points_reached),
                        "success_rate": float(success_rate)
                    },
                    "metrics": {
                        "time_left_sec": float(time_left) if time_left is not None else None,
                        "ee_error_m": float(ee_err) if ee_err is not None else None
                    }
                }
                enviar_seguro(NODE_RED_URL, payload_status)

        if p.isConnected():
            p.stepSimulation()
            if GUI:
                time.sleep(DT / max(1.0, SIM_SPEEDUP))
        else:
            print("[AVISO] Servidor PyBullet desconectado, encerrando simulação")
            break

except KeyboardInterrupt:
    print("\n[AVISO] Simulação interrompida (Ctrl+C)")

finally:
    print("\n" + "=" * 60)
    print("[RESULTADO] FINAL")
    print(f"- Alvos gerados: {targets_generated}/{MAX_TARGETS}")
    print(f"- Detectados: {points_detected}")
    print(f"- Alcançados: {points_reached}")
    if points_detected > 0:
        taxa_sucesso = (points_reached/points_detected)*100
        print(f"- Taxa sucesso: {taxa_sucesso:.1f}%")
    else:
        taxa_sucesso = 0.0
    print("=" * 60)
    
    payload_final = {
        "t": float(t_sim),
        "state": "FINALIZADO",
        "targets_generated": int(targets_generated),
        "max_targets": int(MAX_TARGETS),
        "points_detected": int(points_detected),
        "points_reached": int(points_reached),
        "success_rate": float(taxa_sucesso),
        "simulation_time": float(t_sim),
        "final": True
    }
    enviar_seguro(NODE_RED_URL, payload_final, timeout=1.0)
    print("[INFO] Métricas finais enviadas para Node-RED")

    try:
        p.disconnect()
    except:
        pass
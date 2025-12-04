"""
=============================================================================
BRAÇO ROBÓTICO PLANAR 2D COM CONTROLE PID E PINÇA FUNCIONAL
=============================================================================

Projeto de Robótica - Manipulador com Controle em Malha Fechada

REQUISITOS ATENDIDOS:
1. ✅ Braço articulado com 2 juntas rotacionais no plano XY
2. ✅ Controle PID por torque (malha fechada)
3. ✅ Pinça funcional com dedos móveis
4. ✅ Pick-and-place automático
5. ✅ Resposta a perturbações
6. ✅ Log de desempenho
7. ✅ Detecção de obstáculo e desvio automático de trajetória

ARQUITETURA:
- Sensores: Encoder virtual (ângulo), Sensor de torque
- Atuadores: Motores nas juntas (controle de torque)
- Controle: PID reativo (erro → torque)

Autor: Projeto de Robótica
Data: Dezembro 2025
=============================================================================
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import tempfile
import csv
import random
import json
import threading
from datetime import datetime

# HTTP para integração com Node-RED (sem necessidade de MQTT/Mosquitto)
try:
    import requests
    HTTP_DISPONIVEL = True
except ImportError:
    HTTP_DISPONIVEL = False
    print("⚠️  Biblioteca requests não instalada. Execute: pip install requests")


# =============================================================================
# PARÂMETROS DO ROBÔ
# =============================================================================
L1 = 0.40  # Comprimento do elo 1 (m)
L2 = 0.35  # Comprimento do elo 2 (m)


# =============================================================================
# 1. GERAÇÃO DO URDF - ROBÔ COM PINÇA FUNCIONAL
# =============================================================================
def create_robot_urdf(filename: str):
    """
    Cria URDF do braço robótico planar com pinça de dedos móveis.

    Estrutura:
    - base_link: Base fixa
    - link1: Primeiro elo (rotação em Z)
    - link2: Segundo elo (rotação em Z)
    - gripper_base: Base da pinça
    - finger_left/right: Dedos móveis (prismatic joints)
    """
    urdf_content = f"""<?xml version="1.0"?>
<robot name="braco_planar_pinca">

  <!-- ==================== BASE FIXA ==================== -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.10" length="0.05"/></geometry>
      <material name="cinza_escuro"><color rgba="0.2 0.2 0.2 1"/></material>
    </visual>
    <collision><geometry><cylinder radius="0.10" length="0.05"/></geometry></collision>
    <inertial>
      <mass value="0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>

  <!-- ==================== ELO 1 ({L1}m) ==================== -->
  <link name="link1">
    <visual>
      <origin xyz="{L1/2} 0 0"/>
      <geometry><box size="{L1} 0.06 0.04"/></geometry>
      <material name="vermelho"><color rgba="0.85 0.25 0.25 1"/></material>
    </visual>
    <collision>
      <origin xyz="{L1/2} 0 0"/>
      <geometry><box size="{L1} 0.06 0.04"/></geometry>
    </collision>
    <inertial>
      <origin xyz="{L1/2} 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="2.0"/>
    <dynamics damping="0.7" friction="0.1"/>
  </joint>

  <!-- ==================== ELO 2 ({L2}m) ==================== -->
  <link name="link2">
    <visual>
      <origin xyz="{L2/2} 0 0"/>
      <geometry><box size="{L2} 0.05 0.035"/></geometry>
      <material name="azul"><color rgba="0.25 0.45 0.85 1"/></material>
    </visual>
    <collision>
      <origin xyz="{L2/2} 0 0"/>
      <geometry><box size="{L2} 0.05 0.035"/></geometry>
    </collision>
    <inertial>
      <origin xyz="{L2/2} 0 0"/>
      <mass value="0.7"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="{L1} 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="80" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ==================== BASE DA PINÇA ==================== -->
  <link name="gripper_base">
    <visual>
      <origin xyz="0.03 0 0"/>
      <geometry><box size="0.06 0.08 0.03"/></geometry>
      <material name="cinza"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin xyz="0.03 0 0"/>
      <geometry><box size="0.06 0.08 0.03"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0.03 0 0"/>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="joint_gripper_base" type="fixed">
    <parent link="link2"/>
    <child link="gripper_base"/>
    <origin xyz="{L2} 0 0"/>
  </joint>

  <!-- ==================== DEDO ESQUERDO (MÓVEL) ==================== -->
  <link name="finger_left">
    <visual>
      <origin xyz="0.04 0 0"/>
      <geometry><box size="0.08 0.015 0.025"/></geometry>
      <material name="preto"><color rgba="0.15 0.15 0.15 1"/></material>
    </visual>
    <collision>
      <origin xyz="0.04 0 0"/>
      <geometry><box size="0.08 0.015 0.025"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0.04 0 0"/>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="joint_finger_left" type="prismatic">
    <parent link="gripper_base"/>
    <child link="finger_left"/>
    <origin xyz="0.03 0.025 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.01" upper="0.03" effort="50" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

  <!-- ==================== DEDO DIREITO (MÓVEL) ==================== -->
  <link name="finger_right">
    <visual>
      <origin xyz="0.04 0 0"/>
      <geometry><box size="0.08 0.015 0.025"/></geometry>
      <material name="preto"><color rgba="0.15 0.15 0.15 1"/></material>
    </visual>
    <collision>
      <origin xyz="0.04 0 0"/>
      <geometry><box size="0.08 0.015 0.025"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0.04 0 0"/>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="joint_finger_right" type="prismatic">
    <parent link="gripper_base"/>
    <child link="finger_right"/>
    <origin xyz="0.03 -0.025 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.03" upper="0.01" effort="50" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

</robot>
"""
    with open(filename, 'w') as f:
        f.write(urdf_content)


# =============================================================================
# 2. CINEMÁTICA DIRETA E INVERSA
# =============================================================================
def cinematica_direta(theta1, theta2):
    """
    Calcula a posição (x, y) do end-effector.

    Args:
        theta1: Ângulo da junta 1 (rad)
        theta2: Ângulo da junta 2 (rad)

    Returns:
        (x, y): Posição do end-effector
    """
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y


def cinematica_inversa(x, y):
    """
    Calcula os ângulos das juntas para alcançar (x, y).

    Args:
        x, y: Posição desejada do end-effector

    Returns:
        (theta1, theta2) ou None se fora do alcance
    """
    d = np.sqrt(x**2 + y**2)

    # Verifica alcance
    if d > (L1 + L2) - 0.01 or d < abs(L1 - L2) + 0.01:
        return None

    # Cinemática inversa (solução elbow-up)
    cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    beta = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    theta1 = np.arctan2(y, x) - beta

    return theta1, theta2


# =============================================================================
# 3. CONTROLADOR PID
# =============================================================================
class ControladorPID:
    """
    Controlador PID para controle de junta robótica.

    Equação: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt

    Onde:
    - e(t) = erro = referência - medido
    - u(t) = torque de saída
    """

    def __init__(self, kp, ki, kd, limite_integral=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limite_integral = limite_integral
        self.integral = 0.0
        self.erro_anterior = 0.0

    def reset(self):
        """Reseta o estado interno do controlador."""
        self.integral = 0.0
        self.erro_anterior = 0.0

    def calcular(self, referencia, medido, dt):
        """
        Calcula a saída do controlador PID.

        Args:
            referencia: Ângulo desejado (rad)
            medido: Ângulo medido pelo encoder (rad)
            dt: Intervalo de tempo (s)

        Returns:
            (torque, erro): Torque a aplicar e erro atual
        """
        # Calcula erro
        erro = referencia - medido

        # Normaliza para [-π, π]
        while erro > np.pi:
            erro -= 2 * np.pi
        while erro < -np.pi:
            erro += 2 * np.pi

        # Termo Proporcional
        P = self.kp * erro

        # Termo Integral (com anti-windup)
        self.integral += erro * dt
        self.integral = np.clip(self.integral, -self.limite_integral, self.limite_integral)
        I = self.ki * self.integral

        # Termo Derivativo
        if dt > 0:
            derivada = (erro - self.erro_anterior) / dt
        else:
            derivada = 0
        D = self.kd * derivada

        self.erro_anterior = erro

        # Saída PID
        torque = P + I + D
        return torque, erro


# =============================================================================
# 4. CLIENTE HTTP PARA NODE-RED
# =============================================================================
class NodeRedClient:
    """
    Cliente HTTP para enviar métricas ao Node-RED via POST.

    Configuração Node-RED:
    1. Instale o node 'node-red-dashboard' no Node-RED
    2. Importe o flow 'nodered_flow.json'
    3. Acesse o dashboard em http://localhost:1880/ui

    Endpoint: POST http://localhost:1880/planar

    Tipos de mensagem:
    - cycle_metrics: Métricas ao final de cada ciclo
    - event: Eventos de status (início, fim, etc)
    """

    def __init__(self, url="http://localhost:1880/planar"):
        self.url = url
        self.conectado = False
        self.trajectory = []  # Armazena trajetória do ciclo

        if HTTP_DISPONIVEL:
            self._testar_conexao()

    def _testar_conexao(self):
        """Testa conexão com Node-RED."""
        try:
            # Envia um evento de teste
            response = requests.post(
                self.url,
                json={"type": "event", "event": "Conectando..."},
                timeout=2
            )
            if response.status_code == 200:
                self.conectado = True
                print(f"✅ Conectado ao Node-RED ({self.url})")
                print("   📡 Enviando métricas via HTTP...")
            else:
                print(f"⚠️  Node-RED respondeu com status {response.status_code}")
        except requests.exceptions.ConnectionError:
            print(f"⚠️  Não foi possível conectar ao Node-RED ({self.url})")
            print("   Certifique-se que o Node-RED está rodando: node-red")
            self.conectado = False
        except Exception as e:
            print(f"⚠️  Erro ao conectar: {e}")
            self.conectado = False

    def _enviar_post(self, dados):
        """Envia dados via HTTP POST."""
        if not HTTP_DISPONIVEL:
            return

        try:
            requests.post(self.url, json=dados, timeout=1)
        except:
            pass  # Ignora erros silenciosamente

    def registrar_posicao(self, x, y):
        """Registra posição na trajetória do ciclo."""
        self.trajectory.append({"x": round(x, 4), "y": round(y, 4)})

    def limpar_trajetoria(self):
        """Limpa trajetória para novo ciclo."""
        self.trajectory = []

    def enviar_evento(self, evento):
        """Envia evento de status."""
        self._enviar_post({
            "type": "event",
            "event": evento
        })

    def enviar_metricas_ciclo(self, ciclo, erro_medio, energia, overshoot_max,
                               settling_time1=None, settling_time2=None):
        """
        Envia métricas completas ao final de um ciclo.

        Args:
            ciclo: Número do ciclo
            erro_medio: Erro médio em graus
            energia: Energia total em Joules
            overshoot_max: Overshoot máximo em graus
            settling_time1: Tempo de acomodação junta 1
            settling_time2: Tempo de acomodação junta 2
        """
        dados = {
            "type": "cycle_metrics",
            "cycle": ciclo,
            "data": {
                "mean_error": round(erro_medio, 4),
                "energy": round(energia, 2),
                "max_overshoot": round(overshoot_max, 4),
                "settling_time1": round(settling_time1, 3) if settling_time1 else None,
                "settling_time2": round(settling_time2, 3) if settling_time2 else None,
                "trajectory": self.trajectory[-100:]  # Últimos 100 pontos
            }
        }
        self._enviar_post(dados)

    def enviar_estado(self, estado, ciclo=0):
        """Envia estado atual do robô."""
        self.enviar_evento(f"Ciclo {ciclo} - {estado}")

    def desconectar(self):
        """Finaliza conexão (envia evento de desconexão)."""
        self.enviar_evento("Simulação finalizada")


# =============================================================================
# 5. DATA LOGGER COM MÉTRICAS AVANÇADAS
# =============================================================================
class DataLogger:
    """Registra dados de desempenho para análise, incluindo overshoot e settling time."""

    def __init__(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"log_braco_{timestamp}.csv"
        self.dados = []

        # Métricas avançadas por movimento
        self.reset_metricas_movimento()

    def reset_metricas_movimento(self):
        """Reseta métricas para um novo movimento."""
        # Para cálculo de overshoot
        self.theta1_alvo = None
        self.theta2_alvo = None
        self.theta1_max_desvio = 0.0  # Máximo desvio além do alvo
        self.theta2_max_desvio = 0.0

        # Para cálculo de settling time
        self.tempo_inicio_movimento = None
        self.tempo_acomodacao_1 = None
        self.tempo_acomodacao_2 = None
        self.threshold_acomodacao = np.radians(0.5)  # 0.5 graus
        self.acomodado_1 = False
        self.acomodado_2 = False
        self.amostras_estavel_1 = 0
        self.amostras_estavel_2 = 0
        self.amostras_para_estabilidade = 10  # Precisa estar estável por 10 amostras

    def definir_alvo(self, theta1_alvo, theta2_alvo):
        """Define o alvo para cálculo de overshoot."""
        self.theta1_alvo = theta1_alvo
        self.theta2_alvo = theta2_alvo
        self.tempo_inicio_movimento = time.time()
        self.acomodado_1 = False
        self.acomodado_2 = False
        self.amostras_estavel_1 = 0
        self.amostras_estavel_2 = 0

    def calcular_overshoot(self, theta_med, theta_alvo, theta_inicial):
        """
        Calcula overshoot (ultrapassagem do alvo).
        Overshoot = ((valor_max - valor_final) / (valor_final - valor_inicial)) * 100%
        """
        if theta_alvo is None or theta_inicial is None:
            return 0.0

        # Direção do movimento
        delta_alvo = theta_alvo - theta_inicial
        if abs(delta_alvo) < 0.01:
            return 0.0

        # Quanto passou do alvo na direção do movimento
        delta_atual = theta_med - theta_alvo

        # Se passou na mesma direção do movimento, é overshoot
        if delta_alvo > 0 and delta_atual > 0:
            return (delta_atual / abs(delta_alvo)) * 100
        elif delta_alvo < 0 and delta_atual < 0:
            return (abs(delta_atual) / abs(delta_alvo)) * 100

        return 0.0

    def registrar(self, tempo, t1_ref, t1_med, t2_ref, t2_med, erro1, erro2,
                  torque1, torque2, estado):
        """Registra uma amostra de dados com métricas avançadas."""

        # Calcula overshoot se tiver alvo definido
        overshoot1 = 0.0
        overshoot2 = 0.0

        if self.theta1_alvo is not None:
            # Verifica se ultrapassou o alvo
            erro_alvo1 = t1_med - self.theta1_alvo
            erro_alvo2 = t2_med - self.theta2_alvo

            # Atualiza máximo desvio (para overshoot)
            if abs(erro_alvo1) > abs(self.theta1_max_desvio):
                self.theta1_max_desvio = erro_alvo1
            if abs(erro_alvo2) > abs(self.theta2_max_desvio):
                self.theta2_max_desvio = erro_alvo2

            overshoot1 = abs(np.degrees(self.theta1_max_desvio))
            overshoot2 = abs(np.degrees(self.theta2_max_desvio))

            # Calcula settling time (tempo de acomodação)
            tempo_atual = time.time()

            # Junta 1
            if abs(erro_alvo1) < self.threshold_acomodacao:
                self.amostras_estavel_1 += 1
                if self.amostras_estavel_1 >= self.amostras_para_estabilidade and not self.acomodado_1:
                    self.tempo_acomodacao_1 = tempo_atual - self.tempo_inicio_movimento
                    self.acomodado_1 = True
            else:
                self.amostras_estavel_1 = 0
                self.acomodado_1 = False
                self.tempo_acomodacao_1 = None

            # Junta 2
            if abs(erro_alvo2) < self.threshold_acomodacao:
                self.amostras_estavel_2 += 1
                if self.amostras_estavel_2 >= self.amostras_para_estabilidade and not self.acomodado_2:
                    self.tempo_acomodacao_2 = tempo_atual - self.tempo_inicio_movimento
                    self.acomodado_2 = True
            else:
                self.amostras_estavel_2 = 0
                self.acomodado_2 = False
                self.tempo_acomodacao_2 = None

        self.dados.append({
            'tempo_s': round(tempo, 4),
            'theta1_ref_deg': round(np.degrees(t1_ref), 2),
            'theta1_med_deg': round(np.degrees(t1_med), 2),
            'theta2_ref_deg': round(np.degrees(t2_ref), 2),
            'theta2_med_deg': round(np.degrees(t2_med), 2),
            'erro1_deg': round(np.degrees(erro1), 3),
            'erro2_deg': round(np.degrees(erro2), 3),
            'torque1_Nm': round(torque1, 3),
            'torque2_Nm': round(torque2, 3),
            'overshoot1_deg': round(overshoot1, 3),
            'overshoot2_deg': round(overshoot2, 3),
            'settling_time1_s': round(self.tempo_acomodacao_1, 3) if self.tempo_acomodacao_1 else None,
            'settling_time2_s': round(self.tempo_acomodacao_2, 3) if self.tempo_acomodacao_2 else None,
            'estado': estado
        })

    def obter_metricas_movimento(self):
        """Retorna métricas do movimento atual."""
        return {
            'overshoot1_deg': round(abs(np.degrees(self.theta1_max_desvio)), 3),
            'overshoot2_deg': round(abs(np.degrees(self.theta2_max_desvio)), 3),
            'settling_time1_s': round(self.tempo_acomodacao_1, 3) if self.tempo_acomodacao_1 else None,
            'settling_time2_s': round(self.tempo_acomodacao_2, 3) if self.tempo_acomodacao_2 else None
        }

    def salvar(self):
        """Salva os dados em arquivo CSV."""
        if not self.dados:
            return

        with open(self.filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.dados[0].keys())
            writer.writeheader()
            writer.writerows(self.dados)

        print(f"\n📊 Log salvo: {self.filename}")
        print(f"   Total de amostras: {len(self.dados)}")


# =============================================================================
# 6. SIMULADOR DO BRAÇO ROBÓTICO
# =============================================================================
class BracoRobotico:
    """
    Simulador do braço robótico planar com pinça funcional.
    """

    def __init__(self):
        self.robot_id = None
        self.objeto_id = None

        # Índices das juntas
        self.JOINT_1 = 0          # Junta do elo 1
        self.JOINT_2 = 1          # Junta do elo 2
        self.JOINT_FINGER_L = 3   # Dedo esquerdo
        self.JOINT_FINGER_R = 4   # Dedo direito

        # Controladores PID (ganhos para controle de torque)
        self.pid1 = ControladorPID(kp=80.0, ki=10.0, kd=15.0)
        self.pid2 = ControladorPID(kp=60.0, ki=8.0, kd=12.0)

        # Logger
        self.logger = DataLogger()

        # Cliente HTTP para Node-RED
        self.nodered_client = None
        self.ciclo_atual = 0

        # Posições do cenário
        self.pos_repouso = [0.30, 0.0]      # Posição de repouso
        self.pos_spawn = [0.50, 0.30]       # Onde o objeto aparece
        self.pos_destino = [0.50, -0.30]    # Onde entregar

        # Obstáculo (None = sem obstáculo, ou [x, y, raio])
        self.obstaculo = None
        self.obstaculo_id = None

        # Métricas
        self.metricas = {
            "energia_total": 0.0,
            "erro_acumulado": 0.0,
            "amostras": 0
        }

        # Estado da pinça
        self.pinca_fechada = False

    def inicializar(self):
        """Inicializa a simulação PyBullet."""
        # Conecta ao PyBullet com GUI
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        # Configura câmera (vista de cima)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=0,
            cameraPitch=-89.9,
            cameraTargetPosition=[0.35, 0.0, 0]
        )
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        # Carrega o chão
        p.loadURDF("plane.urdf")

        # Cria e carrega o robô
        urdf_path = os.path.join(tempfile.gettempdir(), "braco_planar_pinca.urdf")
        create_robot_urdf(urdf_path)
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

        # Posição inicial (repouso)
        angulos = cinematica_inversa(self.pos_repouso[0], self.pos_repouso[1])
        if angulos:
            p.resetJointState(self.robot_id, self.JOINT_1, angulos[0])
            p.resetJointState(self.robot_id, self.JOINT_2, angulos[1])

        # Abre a pinça
        self.abrir_pinca()

        # Marcadores visuais
        self._criar_marcador(self.pos_spawn, [1, 0.8, 0, 0.5], "OBJETO")
        self._criar_marcador(self.pos_destino, [0, 0.8, 0.2, 0.5], "DESTINO")

        # Linha do eixo X
        p.addUserDebugLine([0, 0, 0.01], [0.9, 0, 0.01], [0.5, 0.5, 0.5], lineWidth=1)

        # Sliders para ajuste de PID
        self.slider_kp = p.addUserDebugParameter("Kp", 0, 150, 80.0)
        self.slider_ki = p.addUserDebugParameter("Ki", 0, 30, 10.0)
        self.slider_kd = p.addUserDebugParameter("Kd", 0, 30, 15.0)
        self.slider_perturb = p.addUserDebugParameter("Perturbação", 0, 10, 0)
        self.slider_obstaculo = p.addUserDebugParameter("Obstáculo ON/OFF", 0, 1, 1)  # JÁ COMEÇA ATIVO

        # Inicializa cliente HTTP para Node-RED
        self.nodered_client = NodeRedClient(url="http://localhost:1880/planar")

        # Cria obstáculo no meio da trajetória (entre objeto e destino)
        # Posicionado para bloquear o caminho direto
        self.criar_obstaculo([0.60, 0.0], raio=0.05)

    def _criar_marcador(self, pos, cor, texto):
        """Cria um marcador visual no cenário."""
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.04, length=0.01, rgbaColor=cor)
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                          basePosition=[pos[0], pos[1], 0.005])
        p.addUserDebugText(texto, [pos[0], pos[1], 0.08], [0, 0, 0], textSize=1.0)

    def criar_obstaculo(self, pos, raio=0.06):
        """Cria um obstáculo cilíndrico no cenário."""
        if self.obstaculo_id is not None:
            p.removeBody(self.obstaculo_id)

        self.obstaculo = [pos[0], pos[1], raio]

        # Cria cilindro vermelho
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=raio, length=0.20,
                                   rgbaColor=[1, 0.2, 0.2, 0.9])
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=raio, height=0.20)

        self.obstaculo_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[pos[0], pos[1], 0.10]
        )

        p.addUserDebugText("OBSTÁCULO", [pos[0], pos[1], 0.25], [1, 0, 0], textSize=1.0)
        print(f"⛔ Obstáculo criado em: ({pos[0]:.2f}, {pos[1]:.2f}) raio={raio}")

    def remover_obstaculo(self):
        """Remove o obstáculo do cenário."""
        if self.obstaculo_id is not None:
            p.removeBody(self.obstaculo_id)
            self.obstaculo_id = None
            self.obstaculo = None

    def verificar_colisao_caminho(self, pos_inicio, pos_fim):
        """
        Verifica se o caminho entre duas posições passa perto do obstáculo.
        Usa distância ponto-segmento para detectar colisão.

        Retorna:
            True se há colisão, False se caminho livre
        """
        if self.obstaculo is None:
            return False

        ox, oy, raio = self.obstaculo

        # Vetor do segmento
        dx = pos_fim[0] - pos_inicio[0]
        dy = pos_fim[1] - pos_inicio[1]
        comprimento_sq = dx**2 + dy**2

        if comprimento_sq < 0.001:
            return False

        # Projeta o obstáculo no segmento
        t = max(0, min(1, ((ox - pos_inicio[0]) * dx + (oy - pos_inicio[1]) * dy) / comprimento_sq))

        # Ponto mais próximo no segmento
        px = pos_inicio[0] + t * dx
        py = pos_inicio[1] + t * dy

        # Distância do obstáculo ao segmento
        dist = np.sqrt((px - ox)**2 + (py - oy)**2)

        # Margem de segurança (raio + tamanho do braço)
        margem = raio + 0.15

        return dist < margem

    def calcular_waypoints_desvio(self, pos_inicio, pos_fim):
        """
        Calcula pontos intermediários para desviar do obstáculo.
        Estratégia: SEMPRE passa por um ponto seguro (retraído) quando há obstáculo.

        Retorna:
            Lista de waypoints [pos1, pos2, ...]
        """
        if self.obstaculo is None:
            return [pos_fim]

        if not self.verificar_colisao_caminho(pos_inicio, pos_fim):
            return [pos_fim]

        ox, oy, raio = self.obstaculo

        # Ponto seguro: bem retraído em X (perto da base)
        x_seguro = 0.25

        waypoints = []

        # 1. Vai para posição segura mantendo Y atual
        ponto_retracao = [x_seguro, pos_inicio[1]]
        if cinematica_inversa(ponto_retracao[0], ponto_retracao[1]):
            waypoints.append(ponto_retracao)

        # 2. Move em Y para o lado do destino (ainda retraído)
        ponto_lateral = [x_seguro, pos_fim[1]]
        if cinematica_inversa(ponto_lateral[0], ponto_lateral[1]):
            waypoints.append(ponto_lateral)

        # 3. Avança para o destino final
        waypoints.append(pos_fim)

        return waypoints if waypoints else [pos_fim]

    def criar_objeto(self, pos=None):
        """Cria o objeto a ser pego."""
        if self.objeto_id is not None:
            return

        if pos is None:
            # Adiciona variação aleatória
            jitter_x = random.uniform(-0.05, 0.05)
            jitter_y = random.uniform(-0.05, 0.05)
            pos = [self.pos_spawn[0] + jitter_x, self.pos_spawn[1] + jitter_y]

        self.pos_objeto_atual = pos

        # Cria um cubo pequeno
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025],
                                   rgbaColor=[1, 0.6, 0, 1])

        self.objeto_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[pos[0], pos[1], 0.075]
        )

        # Configura física do objeto
        p.changeDynamics(self.objeto_id, -1,
                         lateralFriction=1.0,
                         spinningFriction=0.1,
                         rollingFriction=0.1)

        return pos

    def remover_objeto(self):
        """Remove o objeto da cena."""
        if self.objeto_id is not None:
            p.removeBody(self.objeto_id)
            self.objeto_id = None

    def abrir_pinca(self):
        """Abre os dedos da pinça."""
        # Move dedos para fora
        p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_L,
                                 p.POSITION_CONTROL, targetPosition=0.025, force=10)
        p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_R,
                                 p.POSITION_CONTROL, targetPosition=-0.025, force=10)
        self.pinca_fechada = False

        # Simula alguns passos para abrir
        for _ in range(50):
            p.stepSimulation()
            time.sleep(1/500)

    def fechar_pinca(self):
        """Fecha os dedos da pinça para agarrar."""
        # Move dedos para dentro
        p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_L,
                                 p.POSITION_CONTROL, targetPosition=-0.005, force=20)
        p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_R,
                                 p.POSITION_CONTROL, targetPosition=0.005, force=20)
        self.pinca_fechada = True

        # Simula alguns passos para fechar
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1/500)

    def mover_para_seguro(self, pos_alvo, duracao=2.0, estado="MOVENDO"):
        """
        Move o braço até a posição alvo, desviando de obstáculos se necessário.

        Esta função verifica se há obstáculos no caminho e calcula
        uma rota alternativa automaticamente.
        """
        # Obtém posição atual do end-effector
        try:
            theta1 = p.getJointState(self.robot_id, self.JOINT_1)[0]
            theta2 = p.getJointState(self.robot_id, self.JOINT_2)[0]
            pos_atual = cinematica_direta(theta1, theta2)
        except:
            pos_atual = self.pos_repouso

        # Verifica obstáculo e calcula waypoints
        waypoints = self.calcular_waypoints_desvio(pos_atual, pos_alvo)

        # Se há desvio, informa
        if len(waypoints) > 1:
            print(f"   🛡️  Obstáculo detectado! Desviando por {len(waypoints)} pontos...")

        # Executa movimento por cada waypoint
        duracao_por_wp = duracao / len(waypoints)
        for i, wp in enumerate(waypoints):
            estado_wp = f"{estado}_WP{i+1}" if len(waypoints) > 1 else estado
            if not self.mover_para(wp, duracao=duracao_por_wp, estado=estado_wp):
                return False

        return True

    def mover_para(self, pos_alvo, duracao=2.0, estado="MOVENDO"):
        """
        Move o braço até a posição alvo usando controle PID com torque.
        (Movimento direto, sem verificar obstáculos)

        Args:
            pos_alvo: [x, y] posição desejada
            duracao: Tempo do movimento (s)
            estado: Nome do estado para log

        Returns:
            True se sucesso, False se falha
        """
        # Calcula ângulos alvo
        angulos_alvo = cinematica_inversa(pos_alvo[0], pos_alvo[1])
        if angulos_alvo is None:
            print(f"⚠️  Posição ({pos_alvo[0]:.2f}, {pos_alvo[1]:.2f}) fora do alcance!")
            return False

        theta1_alvo, theta2_alvo = angulos_alvo

        # Configura logger para calcular overshoot/settling time
        self.logger.reset_metricas_movimento()
        self.logger.definir_alvo(theta1_alvo, theta2_alvo)

        # Parâmetros de simulação
        dt = 1.0 / 240.0
        passos = int(duracao * 240)
        tempo_total = 0.0

        # Lê posição atual
        try:
            theta1_atual = p.getJointState(self.robot_id, self.JOINT_1)[0]
            theta2_atual = p.getJointState(self.robot_id, self.JOINT_2)[0]
        except:
            return False

        # Atualiza ganhos PID dos sliders
        try:
            kp = p.readUserDebugParameter(self.slider_kp)
            ki = p.readUserDebugParameter(self.slider_ki)
            kd = p.readUserDebugParameter(self.slider_kd)
            perturb = p.readUserDebugParameter(self.slider_perturb)

            self.pid1.kp, self.pid1.ki, self.pid1.kd = kp, ki, kd
            self.pid2.kp, self.pid2.ki, self.pid2.kd = kp * 0.75, ki * 0.8, kd * 0.8
        except:
            perturb = 0

        # Desabilita controle de velocidade padrão
        p.setJointMotorControl2(self.robot_id, self.JOINT_1, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(self.robot_id, self.JOINT_2, p.VELOCITY_CONTROL, force=0)

        for passo in range(passos):
            try:
                # Fator de interpolação (0 a 1)
                t = passo / passos

                # Referência interpolada (trajetória suave)
                theta1_ref = theta1_atual + (theta1_alvo - theta1_atual) * t
                theta2_ref = theta2_atual + (theta2_alvo - theta2_atual) * t

                # === LEITURA DOS SENSORES (Encoder Virtual) ===
                estado_j1 = p.getJointState(self.robot_id, self.JOINT_1)
                estado_j2 = p.getJointState(self.robot_id, self.JOINT_2)

                theta1_med = estado_j1[0]  # Ângulo medido
                theta2_med = estado_j2[0]

                # === CONTROLE PID ===
                torque1, erro1 = self.pid1.calcular(theta1_ref, theta1_med, dt)
                torque2, erro2 = self.pid2.calcular(theta2_ref, theta2_med, dt)

                # === PERTURBAÇÃO ===
                if perturb > 0.1:
                    perturbacao = perturb * np.sin(passo * 0.03) * 3.0
                    torque2 += perturbacao

                # Limita torque (saturação)
                torque_max = 80.0
                torque1 = np.clip(torque1, -torque_max, torque_max)
                torque2 = np.clip(torque2, -torque_max, torque_max)

                # === APLICA TORQUE ===
                p.setJointMotorControl2(self.robot_id, self.JOINT_1,
                                         p.TORQUE_CONTROL, force=torque1)
                p.setJointMotorControl2(self.robot_id, self.JOINT_2,
                                         p.TORQUE_CONTROL, force=torque2)

                # === MANTÉM DEDOS NA POSIÇÃO (sempre, durante todo movimento) ===
                if self.pinca_fechada:
                    # Pinça fechada - dedos para dentro com força alta
                    p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_L,
                                             p.POSITION_CONTROL, targetPosition=-0.005, force=50)
                    p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_R,
                                             p.POSITION_CONTROL, targetPosition=0.005, force=50)
                else:
                    # Pinça aberta - dedos para fora com força para manter posição
                    p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_L,
                                             p.POSITION_CONTROL, targetPosition=0.02, force=30)
                    p.setJointMotorControl2(self.robot_id, self.JOINT_FINGER_R,
                                             p.POSITION_CONTROL, targetPosition=-0.02, force=30)

                # Simula
                p.stepSimulation()

                # === MÉTRICAS ===
                self.metricas["energia_total"] += (abs(torque1) + abs(torque2)) * dt
                self.metricas["erro_acumulado"] += abs(erro1) + abs(erro2)
                self.metricas["amostras"] += 1

                # === LOG ===
                if passo % 24 == 0:  # 10 Hz
                    self.logger.registrar(
                        tempo_total, theta1_ref, theta1_med, theta2_ref, theta2_med,
                        erro1, erro2, torque1, torque2, estado
                    )

                    # === REGISTRA PARA NODE-RED ===
                    if self.nodered_client:
                        x, y = cinematica_direta(theta1_med, theta2_med)
                        # Registra ponto da trajetória para envio ao final do ciclo
                        self.nodered_client.registrar_posicao(x, y)

                tempo_total += dt
                time.sleep(dt)

            except Exception as e:
                print(f"Erro: {e}")
                return False

        return True

    def executar_ciclo(self, numero_ciclo):
        """Executa um ciclo completo de pick-and-place."""
        self.ciclo_atual = numero_ciclo

        # Envia início do ciclo para Node-RED
        if self.nodered_client:
            self.nodered_client.limpar_trajetoria()
            self.nodered_client.enviar_estado("INICIANDO_CICLO", numero_ciclo)

        print(f"\n{'='*55}")
        print(f"   🔄 CICLO {numero_ciclo}")
        print(f"{'='*55}")

        tempo_inicio = time.time()
        energia_inicio = self.metricas["energia_total"]
        erro_inicio = self.metricas["erro_acumulado"]
        amostras_inicio = self.metricas["amostras"]

        # 1. Criar objeto
        pos_obj = self.criar_objeto()
        print(f"📦 Objeto criado em: ({pos_obj[0]:.2f}, {pos_obj[1]:.2f})")
        time.sleep(0.3)

        # Verifica se deve criar obstáculo
        try:
            obst_on = p.readUserDebugParameter(self.slider_obstaculo)
            if obst_on > 0.5 and self.obstaculo is None:
                # Cria obstáculo no meio do caminho
                obst_pos = [0.45, 0.0]  # Entre objeto e destino
                self.criar_obstaculo(obst_pos, raio=0.06)
            elif obst_on < 0.5 and self.obstaculo is not None:
                self.remover_obstaculo()
                print("⛔ Obstáculo removido")
        except:
            pass

        # 2. Mover até o objeto (com desvio se necessário)
        print("🎯 Indo até o objeto...")
        if not self.mover_para_seguro(pos_obj, duracao=1.8, estado="APROXIMAR"):
            return False

        # 3. Fechar pinça
        print("✊ Fechando pinça...")
        self.fechar_pinca()

        # 4. Levantar um pouco (para não arrastar)
        pos_elevada = [pos_obj[0] - 0.05, pos_obj[1]]
        if not self.mover_para(pos_elevada, duracao=0.8, estado="LEVANTAR"):
            return False

        # 5. Mover para destino (com desvio se necessário)
        print("🚚 Levando ao destino...")
        if not self.mover_para_seguro(self.pos_destino, duracao=2.0, estado="TRANSPORTAR"):
            return False

        # 6. Abrir pinça
        print("✋ Soltando objeto...")
        self.abrir_pinca()

        # 7. Aguardar objeto cair
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1/240)

        # 8. Remover objeto e voltar
        self.remover_objeto()

        print("🏠 Retornando à posição de repouso...")
        if not self.mover_para(self.pos_repouso, duracao=1.5, estado="RETORNO"):
            return False

        # === RELATÓRIO DO CICLO ===
        tempo_ciclo = time.time() - tempo_inicio
        amostras_ciclo = self.metricas["amostras"] - amostras_inicio
        erro_medio = (self.metricas["erro_acumulado"] - erro_inicio) / max(1, amostras_ciclo)
        energia_ciclo = self.metricas["energia_total"] - energia_inicio

        # Obtém métricas de movimento
        metricas_mov = self.logger.obter_metricas_movimento()

        print(f"\n{'─'*45}")
        print(f"📊 RELATÓRIO DO CICLO {numero_ciclo}")
        print(f"{'─'*45}")
        print(f"   ⏱️  Tempo total:      {tempo_ciclo:.2f} s")
        print(f"   📐 Erro médio:       {np.degrees(erro_medio):.2f}°")
        print(f"   ⚡ Energia gasta:    {energia_ciclo:.1f} J")
        print(f"   📈 Amostras:         {amostras_ciclo}")
        print(f"   📉 Overshoot J1:     {metricas_mov['overshoot1_deg']:.2f}°")
        print(f"   📉 Overshoot J2:     {metricas_mov['overshoot2_deg']:.2f}°")
        if metricas_mov['settling_time1_s']:
            print(f"   ⏳ Settling Time J1: {metricas_mov['settling_time1_s']:.3f} s")
        if metricas_mov['settling_time2_s']:
            print(f"   ⏳ Settling Time J2: {metricas_mov['settling_time2_s']:.3f} s")
        print(f"{'─'*45}")
        print(f"✅ Ciclo {numero_ciclo} concluído com sucesso!")

        # Envia relatório para Node-RED via HTTP
        if self.nodered_client:
            self.nodered_client.enviar_metricas_ciclo(
                ciclo=numero_ciclo,
                erro_medio=round(np.degrees(erro_medio), 2),
                overshoot_max=max(metricas_mov['overshoot1_deg'], metricas_mov['overshoot2_deg']),
                energia=round(energia_ciclo, 1),
                settling_time1=metricas_mov['settling_time1_s'],
                settling_time2=metricas_mov['settling_time2_s']
            )
            self.nodered_client.enviar_estado("CICLO_CONCLUIDO", numero_ciclo)

        # Reset PIDs
        self.pid1.reset()
        self.pid2.reset()

        return True

    def executar(self, num_ciclos=3):
        """Executa a simulação completa."""

        self.inicializar()

        print("\n" + "="*60)
        print("   🤖 BRAÇO ROBÓTICO PLANAR - CONTROLE PID COM PINÇA")
        print("="*60)
        print(f"   📍 Posição repouso:  ({self.pos_repouso[0]:.2f}, {self.pos_repouso[1]:.2f})")
        print(f"   📦 Posição objeto:   ({self.pos_spawn[0]:.2f}, {self.pos_spawn[1]:.2f}) ± 0.05")
        print(f"   🎯 Posição destino:  ({self.pos_destino[0]:.2f}, {self.pos_destino[1]:.2f})")
        print(f"   🔄 Total de ciclos:  {num_ciclos}")
        print("="*60)
        print("\n   📌 SLIDERS DISPONÍVEIS:")
        print("   • Kp, Ki, Kd - Ajuste dos ganhos PID")
        print("   • Perturbação - Simula peso extra no efetuador")
        print("   • Obstáculo ON/OFF - Ativa obstáculo no caminho\n")

        time.sleep(2)

        try:
            for i in range(1, num_ciclos + 1):
                if not self.executar_ciclo(i):
                    print("\n⚠️  Simulação interrompida!")
                    break
                time.sleep(1.0)

            print("\n" + "="*60)
            print(f"   🏁 SIMULAÇÃO COMPLETA! ({num_ciclos} ciclos)")
            print("="*60)

            self.logger.salvar()

            # Mantém janela aberta
            print("\n   [Feche a janela do PyBullet para sair]")
            while True:
                try:
                    p.stepSimulation()
                    time.sleep(0.1)
                except:
                    break

        except KeyboardInterrupt:
            print("\n⚠️  Interrompido pelo usuário")
            self.logger.salvar()

        finally:
            # Desconecta Node-RED
            if self.nodered_client:
                self.nodered_client.desconectar()

            try:
                p.disconnect()
            except:
                pass


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    robo = BracoRobotico()
    robo.executar(num_ciclos=3)

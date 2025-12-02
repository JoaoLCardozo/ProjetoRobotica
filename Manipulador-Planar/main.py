import pybullet as p
import pybullet_data
import numpy as np
import time

# Conecta ao PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

# Carrega o plano de fundo
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

# Define as dimensões do cubo
cubo_size = 0.1
cubo_pos = [0.5, 0, cubo_size/2]
cuboId = p.loadURDF("cube.urdf", cubo_pos, globalScaling=cubo_size)

# Define as dimensões do braço
base_pos = [0, 0, 0]
link1_len = 0.3
link2_len = 0.3
link3_len = 0.1

# Cria o braço articulado
baseId = p.loadURDF("base.urdf", base_pos)
joint1Id = p.loadURDF("joint.urdf", [0, 0, link1_len/2], p.getQuaternionFromEuler([0, 0, 0]))
joint2Id = p.loadURDF("joint.urdf", [0, 0, link1_len + link2_len/2], p.getQuaternionFromEuler([0, 0, 0]))
joint3Id = p.loadURDF("joint.urdf", [0, 0, link1_len + link2_len + link3_len/2], p.getQuaternionFromEuler([0, 0, 0]))

# Define as juntas
joint1 = p.createJoint(baseId, -1, joint1Id, -1, p.JOINT_REVOLUTE, [0, 0, 1], [0, 0, link1_len/2], [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]), p.getQuaternionFromEuler([0, 0, 0]))
joint2 = p.createJoint(joint1Id, -1, joint2Id, -1, p.JOINT_REVOLUTE, [0, 0, 1], [0, 0, link2_len/2], [0, 0, -link1_len/2], p.getQuaternionFromEuler([0, 0, 0]), p.getQuaternionFromEuler([0, 0, 0]))
joint3 = p.createJoint(joint2Id, -1, joint3Id, -1, p.JOINT_REVOLUTE, [0, 0, 1], [0, 0, link3_len/2], [0, 0, -link2_len/2], p.getQuaternionFromEuler([0, 0, 0]), p.getQuaternionFromEuler([0, 0, 0]))

# Define os limites das juntas
p.setJointLimits(joint1, -np.pi/2, np.pi/2)
p.setJointLimits(joint2, -np.pi/2, np.pi/2)
p.setJointLimits(joint3, -np.pi/2, np.pi/2)

# Função para mover o braço
def move_braco(joint1_angle, joint2_angle, joint3_angle):
    p.setJointMotorControl2(joint1, p.POSITION_CONTROL, targetPosition=joint1_angle)
    p.setJointMotorControl2(joint2, p.POSITION_CONTROL, targetPosition=joint2_angle)
    p.setJointMotorControl2(joint3, p.POSITION_CONTROL, targetPosition=joint3_angle)

# Loop de simulação
while True:
    # Move o braço para a posição inicial
    move_braco(0, 0, 0)
    p.stepSimulation()
    time.sleep(1/240)

    # Move o braço para a posição do cubo
    move_braco(np.pi/4, np.pi/4, np.pi/4)
    p.stepSimulation()
    time.sleep(1/240)

    # Fecha o braço
    move_braco(np.pi/4, np.pi/4, 0)
    p.stepSimulation()
    time.sleep(1/240)

    # Levanta o cubo
    move_braco(0, np.pi/4, 0)
    p.stepSimulation()
    time.sleep(1/240)

    # Desliga o PyBullet
    p.disconnect()
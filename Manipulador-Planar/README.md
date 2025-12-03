# Manipulador Planar 2-DOF com Controle PID

Simulação de um braço robótico planar com 2 juntas rotacionais, implementado em PyBullet com controle PID em malha fechada, detecção de colisão e testes de perturbação.

## Características Implementadas

### 1. Cinemática (FK/IK)
- **Forward Kinematics**: Cálculo da posição do efetuador a partir dos ângulos das juntas
- **Inverse Kinematics**: Solução robusta de 2 links com clamping para o espaço de trabalho
- Validação contra alcance mínimo e máximo

### 2. Controle PID
- Controlador PID individual para cada junta (q1, q2)
- Erro angular normalizado (diferença angular mínima)
- Anti-windup para evitar saturação do integral
- **Interface em tempo real**: Sliders no PyBullet para ajustar Kp, Ki, Kd em execução

### 3. Detecção de Colisão e Evasão
- Verificação de caminho sem colisão antes de executar movimento
- Obstáculos esféricos com margem de segurança ajustável
- Replanejamento automático: se colisão detectada, braço mantém ângulos seguros
- Função `avoid_point_obstacles()` que desloca alvos para fora de obstáculos

### 4. Pick & Drop
- Grasp automático quando efetuador próximo ao alvo (< 0.06 m)
- Constraint físico rígido (JOINT_FIXED) para transporte
- Drop automático ao atingir zona de deposito (x < 0.4)
- Botões de controle manual (`grasp_toggle`, `release`)

### 5. Teste de Perturbação
- Módulo `perturbation.py` para adicionar payload ao efetuador
- Simula diferentes massas no efetuador
- Testa resposta do controle PID a mudanças de carga
- Habilitar com `ENABLE_PERTURBATION_TEST = True`

### 6. Métricas de Desempenho
- **Erro médio de posição**: Média dos erros angulares absolutosdo simulação
- **Energia total**: Integral de |torque × velocidade| ao longo do tempo
- **Overshoot máximo**: Maior desvio do ângulo de referência
- **Tempo de estabilização**: Tempo até permanecer dentro de 0.03 rad por 0.6s
- Log enviado via MQTT/HTTP a cada ~0.05s

### 7. Autotuning
- Botão `autotune` para ativar rotina de sintonização automática
- Algoritmo tipo Twiddle: ajusta Kp de cada junta iterativamente
- Avalia desempenho em janelas de 2 segundos
- Imprime logs de progresso no console

## Arquitetura

```
projeto_robotica_q1.py      - Script principal (loop de simulação)
pid.py                      - Classe PIDController com anti-windup
robot_arm.py                - Classe RobotArm (FK/IK, colisão, grasp)
planner.py                  - Funções de planejamento (avoid_point_obstacles)
perturbation.py             - Classe PerturbationTester (payload)
logger_node_red.py          - LoggerNodeRed (MQTT + HTTP)
planar_arm.urdf             - Descrição URDF do braço (2 links, 2 juntas)
requirements.txt            - Dependências Python
```

## Instalação e Uso

### 1. Instalar dependências
```bash
cd "c:\Users\joluc\OneDrive\Documents\GitHub\ProjetoRobotica\Manipulador-Planar"
python -m pip install -r requirements.txt
```

### 2. Executar simulação básica
```bash
python projeto_robotica_q1.py
```

### 3. Testar com perturbação (payload)
Edite `projeto_robotica_q1.py` e altere:
```python
ENABLE_PERTURBATION_TEST = True
```

Depois execute:
```bash
python projeto_robotica_q1.py
```

O robô será simulado com um payload de 0.3 kg no efetuador.

### 4. Usar com Node-RED (MQTT/HTTP)
Altere a configuração no arquivo:
```python
USE_MQTT = True
MQTT_BROKER = "seu.broker.aqui"
MQTT_TOPIC = "robotica/q1/logs"
HTTP_NODE_RED = "http://localhost:1880/seu_endpoint"
```

Os dados serão enviados em formato JSON:
```json
{
  "tempo": 5.123,
  "junta1": {"ref": 0.5, "real": 0.48, "erro": 0.02, "torque": 50.2},
  "junta2": {"ref": 1.2, "real": 1.19, "erro": 0.01, "torque": 45.1},
  "metrics": {
    "erro_medio": 0.015,
    "energia_total": 234.5,
    "overshoot_max": 0.08,
    "settling_time": 2.5
  },
  "target": {"x": 1.2, "y": 0.6}
}
```

## Controles na GUI PyBullet

### Sliders
- `kp1`, `ki1`, `kd1`: Ganhos PID da junta 1
- `kp2`, `ki2`, `kd2`: Ganhos PID da junta 2

### Botões (0/1)
- `grasp_toggle`: Aciona grasp manual
- `release`: Libera objeto
- `autotune`: Ativa/desativa autotuning

## Parâmetros Ajustáveis

No código `projeto_robotica_q1.py`:

```python
# Obstáculos
obstacles = [
  {'pos': (0.8, 0.5), 'radius': 0.15},
  {'pos': (1.2, 0.0), 'radius': 0.12}
]

# Ganhos PID iniciais
pid1 = PIDController(kp=60.0, ki=1.0, kd=5.0, ...)
pid2 = PIDController(kp=60.0, ki=1.0, kd=5.0, ...)

# Tolerâncias
settle_tol = 0.03  # rad
settle_required = 0.6  # segundos

# Comprimentos dos links
L1 = 1.0
L2 = 1.0
```

## Limitações e Próximas Melhorias

1. **Cinemática inversa única**: Atualmente elege apenas um dos 2 possíveis ângulos (cotovelo para cima)
2. **Planejamento simplificado**: A evasão de obstáculos é reativa; não há planejamento global (RRT, etc.)
3. **Autotuning básico**: Ajusta apenas Kp; versão futura poderia incluir Ki, Kd e usar otimização mais robusta
4. **Física simplificada**: Sem atrito, sem limite de velocidade efetivo, sem deformações elásticas

## Exemplo de Execução

```
Iniciando simulação melhorada...
Payload attached: 0.3 kg
Autotune enabled = True
Tune eval time=2.04, avg_err=0.1245, params=[60.0, 60.0], dp=[6.0, 6.0]
Collision detected! Path not safe at t=15.32
Tune eval time=4.08, avg_err=0.0954, params=[66.0, 60.0], dp=[6.6, 6.0]
...
```

## Referências

- **PyBullet Documentation**: https://pybullet.org/
- **PID Control**: https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
- **Robotica**: Inverse kinematics for planar 2-link arm

# 🤖 Projeto de Robótica - Manipulador Planar 2-DOF

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![PyBullet](https://img.shields.io/badge/PyBullet-3.2+-green.svg)
![Node-RED](https://img.shields.io/badge/Node--RED-Dashboard-red.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

**Simulação de um braço robótico planar com controle PID, detecção de obstáculos e integração Node-RED**

[Funcionalidades](#-funcionalidades) •
[Instalação](#-instalação) •
[Execução](#-execução) •
[Arquitetura](#-arquitetura) •
[Node-RED](#-integração-node-red)

</div>

---

## 📋 Sobre o Projeto

Este projeto implementa um **manipulador robótico planar de 2 graus de liberdade (2-DOF)** com:
- Controle PID em malha fechada por torque
- Pinça funcional com dedos móveis (prismatic joints)
- Detecção automática de obstáculos e desvio de trajetória
- Operações de pick-and-place automatizadas
- Métricas de desempenho (overshoot, settling time, energia)
- Integração com Node-RED para visualização em tempo real

---

## 🛠️ Stack Tecnológica

| Tecnologia | Versão | Descrição |
|------------|--------|-----------|
| **Python** | 3.8+ | Linguagem principal |
| **PyBullet** | 3.2+ | Simulador de física e visualização 3D |
| **NumPy** | 1.21+ | Cálculos matemáticos e matrizes |
| **paho-mqtt** | 1.6+ | Cliente MQTT para Node-RED |
| **Node-RED** | 3.0+ | Dashboard de visualização (opcional) |
| **Mosquitto** | 2.0+ | Broker MQTT (opcional) |

---

## ✨ Funcionalidades

### Robô e Controle
- ✅ **Manipulador 2-DOF** - Braço articulado com 2 juntas rotacionais no plano XY
- ✅ **Controle PID** - Malha fechada com controle de torque
- ✅ **Cinemática** - Direta e Inversa implementadas
- ✅ **Pinça Funcional** - Dedos móveis com prismatic joints
- ✅ **Ajuste em Tempo Real** - Sliders para Kp, Ki, Kd

### Navegação
- ✅ **Detecção de Obstáculos** - Algoritmo de distância ponto-segmento
- ✅ **Desvio Automático** - Recálculo de trajetória com waypoints
- ✅ **Margem de Segurança** - Considera tamanho do braço

### Operações
- ✅ **Pick-and-Place** - Ciclos automatizados de pegar e soltar
- ✅ **Perturbação** - Simulação de peso extra no efetuador
- ✅ **Múltiplos Ciclos** - Configurável (padrão: 6 ciclos)

### Métricas e Logging
- ✅ **Overshoot** - Ultrapassagem máxima do alvo
- ✅ **Settling Time** - Tempo de acomodação (erro < 0.5°)
- ✅ **Energia Total** - Integral do torque aplicado
- ✅ **Erro Médio** - Erro angular médio por ciclo
- ✅ **Log CSV** - Exportação de dados com timestamp
- ✅ **MQTT** - Envio em tempo real para Node-RED

---

## 📁 Estrutura do Projeto

```
ProjetoRobotica/
├── README.md                    # Este arquivo
├── Manipulador-Planar/
│   ├── braco_robotico.py       # 🎯 Script principal da simulação
│   ├── projeto_robotica_q1.py  # Script alternativo (versão básica)
│   ├── planar_arm.urdf         # Descrição URDF do robô
│   ├── requirements.txt        # Dependências Python
│   ├── nodered_flow.json       # Flow para importar no Node-RED
│   ├── nodered_setup.md        # Instruções de configuração Node-RED
│   └── log_braco_*.csv         # Logs gerados pela simulação
```

---

## 🚀 Instalação

### Pré-requisitos
- Python 3.8 ou superior
- pip (gerenciador de pacotes Python)

### 1. Clonar o Repositório
```bash
git clone https://github.com/JoaoLCardozo/ProjetoRobotica.git
cd ProjetoRobotica/Manipulador-Planar
```

### 2. Criar Ambiente Virtual (Recomendado)
```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate
```

### 3. Instalar Dependências
```bash
pip install -r requirements.txt
```

Ou instale manualmente:
```bash
pip install pybullet numpy paho-mqtt
```

---

## ▶️ Execução

### Simulação Básica
```bash
cd Manipulador-Planar
python braco_robotico.py
```

### O que acontece:
1. 🖥️ Abre janela do PyBullet com visualização 3D
2. 🤖 Robô inicia na posição de repouso
3. ⛔ Obstáculo vermelho aparece no cenário
4. 📦 Objeto é criado na zona de spawn
5. 🔄 Robô executa 6 ciclos de pick-and-place
6. 📊 Log é salvo em arquivo CSV

### Controles na Interface

| Slider | Descrição | Valor Padrão |
|--------|-----------|--------------|
| `Kp` | Ganho Proporcional | 80.0 |
| `Ki` | Ganho Integral | 10.0 |
| `Kd` | Ganho Derivativo | 15.0 |
| `Perturbação` | Simula peso extra | 0 |
| `Obstáculo ON/OFF` | Ativa/desativa obstáculo | ON |

---

## 🏗️ Arquitetura

### Diagrama de Classes

```
┌─────────────────────────────────────────────────────────────┐
│                      BracoRobotico                          │
├─────────────────────────────────────────────────────────────┤
│ - robot_id: int                                             │
│ - pid1, pid2: ControladorPID                                │
│ - logger: DataLogger                                        │
│ - mqtt_client: NodeRedClient                                │
├─────────────────────────────────────────────────────────────┤
│ + inicializar()                                             │
│ + mover_para_seguro(pos_alvo, duracao)                      │
│ + verificar_colisao_caminho(pos_inicio, pos_fim)            │
│ + calcular_waypoints_desvio(pos_inicio, pos_fim)            │
│ + executar_ciclo(numero_ciclo)                              │
│ + abrir_pinca() / fechar_pinca()                            │
└─────────────────────────────────────────────────────────────┘
                              │
           ┌──────────────────┼──────────────────┐
           ▼                  ▼                  ▼
┌──────────────────┐  ┌───────────────┐  ┌───────────────────┐
│  ControladorPID  │  │   DataLogger  │  │   NodeRedClient   │
├──────────────────┤  ├───────────────┤  ├───────────────────┤
│ - kp, ki, kd     │  │ - dados[]     │  │ - client: mqtt    │
│ - integral       │  │ - overshoot   │  │ - broker: str     │
│ - erro_anterior  │  │ - settling    │  │ - topico_base     │
├──────────────────┤  ├───────────────┤  ├───────────────────┤
│ + calcular()     │  │ + registrar() │  │ + publicar()      │
│ + reset()        │  │ + salvar()    │  │ + enviar_*()      │
└──────────────────┘  └───────────────┘  └───────────────────┘
```

### Fluxo de Controle

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Referência │───▶│  Erro (e)   │───▶│     PID     │
│   (θ_ref)   │    │ θ_ref - θ   │    │ P + I + D   │
└─────────────┘    └─────────────┘    └──────┬──────┘
                                             │
                                             ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Encoder   │◀───│    Robô     │◀───│   Torque    │
│  (θ_medido) │    │  (PyBullet) │    │   (τ)       │
└─────────────┘    └─────────────┘    └─────────────┘
```

### Algoritmo de Desvio de Obstáculos

```
1. Verificar se caminho direto tem colisão
   └─▶ Calcula distância ponto-segmento ao obstáculo
   └─▶ Se distância < (raio + margem): COLISÃO

2. Se colisão detectada, calcular waypoints:
   └─▶ Waypoint 1: Retrair (X = 0.25, Y = atual)
   └─▶ Waypoint 2: Mover lateral (X = 0.25, Y = destino)
   └─▶ Waypoint 3: Avançar ao destino final

3. Executar movimento por cada waypoint
```

---

## 📊 Integração Node-RED

O projeto suporta visualização em tempo real via MQTT.

### Configuração Rápida

1. **Instalar Mosquitto** (broker MQTT):
   - Download: https://mosquitto.org/download/

2. **Iniciar Node-RED**:
   ```bash
   node-red
   ```

3. **Importar Flow**:
   - Acesse http://localhost:1880
   - Menu → Import → `nodered_flow.json`
   - Deploy

4. **Acessar Dashboard**:
   - http://localhost:1880/ui

### Tópicos MQTT Publicados

| Tópico | Dados | Frequência |
|--------|-------|------------|
| `robo/posicao` | `{x, y}` | 10 Hz |
| `robo/erro` | `{erro1_deg, erro2_deg}` | 10 Hz |
| `robo/pid` | `{torque1, torque2}` | 10 Hz |
| `robo/estado` | `{estado, ciclo}` | Por estado |
| `robo/metricas` | JSON completo | 10 Hz |
| `robo/relatorio_ciclo` | Resumo do ciclo | Por ciclo |

### Exemplo de Payload

```json
{
  "erro1_deg": 0.125,
  "erro2_deg": 0.087,
  "energia": 45.23,
  "posicao_x": 0.4521,
  "posicao_y": 0.2834,
  "ciclo": 2,
  "estado": "TRANSPORTAR",
  "overshoot1_deg": 1.234,
  "overshoot2_deg": 0.876,
  "settling_time1_s": 0.342,
  "settling_time2_s": 0.298
}
```

---

## 📈 Métricas de Desempenho

### Definições

| Métrica | Fórmula | Descrição |
|---------|---------|-----------|
| **Erro Médio** | $\bar{e} = \frac{1}{N}\sum_{i=1}^{N}\|e_i\|$ | Média dos erros absolutos |
| **Energia** | $E = \int_0^T \|\tau\| \, dt$ | Integral do torque aplicado |
| **Overshoot** | $OS = \frac{\theta_{max} - \theta_{alvo}}{\theta_{alvo} - \theta_0} \times 100\%$ | Ultrapassagem percentual |
| **Settling Time** | $t_s : \|e(t)\| < 0.5°, \forall t > t_s$ | Tempo até estabilização |

### Exemplo de Relatório

```
─────────────────────────────────────────────
📊 RELATÓRIO DO CICLO 3
─────────────────────────────────────────────
   ⏱️  Tempo total:      8.54 s
   📐 Erro médio:       0.23°
   ⚡ Energia gasta:    156.7 J
   📈 Amostras:         2050
   📉 Overshoot J1:     1.45°
   📉 Overshoot J2:     0.98°
   ⏳ Settling Time J1: 0.412 s
   ⏳ Settling Time J2: 0.356 s
─────────────────────────────────────────────
```

---

## ⚙️ Parâmetros Configuráveis

### No código `braco_robotico.py`:

```python
# Comprimento dos elos (metros)
L1 = 0.40  # Elo 1
L2 = 0.35  # Elo 2

# Ganhos PID
Kp = 80.0   # Proporcional
Ki = 10.0   # Integral
Kd = 15.0   # Derivativo

# Posições do cenário
pos_repouso = [0.30, 0.0]    # Posição inicial
pos_spawn = [0.50, 0.30]     # Onde objeto aparece
pos_destino = [0.50, -0.30]  # Onde entregar

# Obstáculo
obstaculo_pos = [0.60, 0.0]  # Posição X, Y
obstaculo_raio = 0.05        # Raio em metros

# Número de ciclos
num_ciclos = 6
```

---

## 🔧 Troubleshooting

### Erro: `ModuleNotFoundError: No module named 'pybullet'`
```bash
pip install pybullet
```

### Erro: `MQTT não conecta`
- Verifique se Mosquitto está rodando na porta 1883
- Teste: `netstat -an | findstr 1883`

### Braço não alcança posição
- Verifique se a posição está dentro do alcance:
  - Mínimo: |L1 - L2| = 0.05m
  - Máximo: L1 + L2 = 0.75m

### Simulação muito lenta
- Reduza a frequência de log (altere `passo % 24` para valor maior)
- Desative MQTT se não estiver usando

---

## 👥 Autores

- **João Lucas Cardozo** - [GitHub](https://github.com/JoaoLCardozo)

---

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para mais detalhes.

---

## 📚 Referências

- [PyBullet Documentation](https://pybullet.org/)
- [PID Control - Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Inverse Kinematics for Planar 2-Link Arm](https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/)
- [Node-RED Dashboard](https://flows.nodered.org/node/node-red-dashboard)
- [MQTT Protocol](https://mqtt.org/)

---

<div align="center">

**⭐ Se este projeto foi útil, considere dar uma estrela!**

</div>

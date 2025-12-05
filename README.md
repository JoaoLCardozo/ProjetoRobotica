# 🤖 Simulação de Robôs com PyBullet e Node-RED

Trabalho acadêmico de Robótica contendo dois projetos de simulação: um **manipulador planar** e um **robô aspirador autônomo**. Ambos utilizam PyBullet para física e Node-RED para supervisão.

---

## 📋 Índice

- [Visão Geral](#visão-geral)
- [Projeto 1: Manipulador Planar](#-manipulador-planar-2-dof)
- [Projeto 2: Robô Aspirador](#-robô-aspirador-autônomo)
- [Instalação](#instalação)
- [Node-RED](#configuração-node-red)

---

## Visão Geral

| Projeto | Tipo | Controle | Sensores |
|---------|------|----------|----------|
| **Manipulador Planar** | Braço articulado 2-DOF | PID por junta | Encoder, torque |
| **Robô Aspirador** | Móvel diferencial | Navegação reativa | 5x ultrassônico |

---

## 🦾 Manipulador Planar 2-DOF

<p align="center">
  <img src="images/manipulador-planar.png" alt="Manipulador Planar" width="450"/>
</p>

### Funcionalidades

| Funcionalidade | Descrição |
|----------------|-----------|
| Cinemática Inversa | Calcula ângulos para posição XY desejada |
| Controle PID | Malha fechada por torque em cada junta |
| Detecção de Obstáculos | Desvio automático de trajetória |
| Pick-and-Place | Pegar e soltar objetos automaticamente |
| Pinça Funcional | Dedos móveis com feedback de força |

### Métricas

| Métrica | Descrição |
|---------|-----------|
| Erro médio | Diferença entre ângulo alvo e atual |
| Overshoot | Ultrapassagem máxima |
| Energia | Trabalho dos motores (J) |
| Settling time | Tempo de estabilização |

### Execução

```bash
cd Manipulador-Planar
pip install -r requirements.txt
python braco_robotico.py
```

### Estrutura

```
Manipulador-Planar/
├── braco_robotico.py      # Script principal
├── planar_arm.urdf        # Modelo do robô
├── nodered_flow.json      # Flow Node-RED
└── log_braco_*.csv        # Logs gerados
```

---

## 🧹 Robô Aspirador Autônomo

<p align="center">
  <img src="images/robo-aspirador.png" alt="Robô Aspirador" width="450"/>
</p>

### Funcionalidades

| Funcionalidade | Descrição |
|----------------|-----------|
| Navegação Autônoma | Varredura sistemática (boustrophedon) |
| Evasão de Obstáculos | 5 sensores ultrassônicos |
| Mapeamento 2D | Grid de ocupação construído em tempo real |
| Aprendizado | Usa mapa anterior para otimizar rotas |

### Comportamento de Aprendizado

| Execução | Comportamento |
|----------|---------------|
| 1ª | Exploração completa, cria mapa |
| 2ª | Usa mapa salvo, evita áreas já limpas |
| 3ª+ | Otimização refinada |

### Métricas

| Métrica | Objetivo |
|---------|----------|
| Cobertura (%) | Maximizar |
| Tempo | Minimizar |
| Energia (J) | Minimizar |
| Colisões | Minimizar |

### Execução

```bash
cd Robo-Aspirador
python main.py
```

| Argumento | Descrição | Padrão |
|-----------|-----------|--------|
| `--executions` | Número de execuções | 3 |
| `--time` | Tempo máximo (s) | 90 |
| `--no-gui` | Sem interface | False |

### Estrutura

```
Robo-Aspirador/
├── main.py                # Script principal
├── node_red_flow.json     # Flow Node-RED
├── src/
│   ├── robot.py           # Classe do robô
│   ├── environment.py     # Ambiente PyBullet
│   ├── mapping.py         # Mapa de ocupação
│   ├── controller.py      # Navegação
│   └── node_red_client.py # Cliente HTTP
├── models/                # URDFs
└── saved_maps/            # Mapas entre execuções
```

---

## Instalação

```bash
# Clonar repositório
git clone https://github.com/JoaoLCardozo/ProjetoRobotica.git
cd ProjetoRobotica

# Instalar dependências
pip install pybullet numpy requests
```

---

## Configuração Node-RED

```bash
# Instalar Node-RED
npm install -g node-red

# Iniciar
node-red
```

1. Acesse `http://localhost:1880`
2. Menu ☰ → Import → selecione `node_red_flow.json`
3. Deploy
4. Dashboard em `http://localhost:1880/ui`

---

## ✅ Requisitos Atendidos

### Manipulador
- ✅ Manipulador planar 2-DOF
- ✅ Controle PID por junta
- ✅ Cinemática direta/inversa
- ✅ Desvio de obstáculos
- ✅ Pick-and-place
- ✅ Supervisão Node-RED

### Aspirador
- ✅ Robô móvel diferencial
- ✅ 5 sensores ultrassônicos
- ✅ Exploração autônoma
- ✅ Mapeamento de ocupação
- ✅ Aprendizado por repetição
- ✅ Supervisão Node-RED

---

## 👥 Autor

**João Luiz Ferreira Cardozo** - [@JoaoLCardozo](https://github.com/JoaoLCardozo)




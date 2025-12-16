
# 🤖 Robô Aspirador Autônomo com PyBullet e Node-RED

> Simulação de um robô aspirador autônomo utilizando PyBullet para física e Node-RED para supervisão e visualização.

---

## 📋 Índice

- [Visão Geral](#visão-geral)
- [Funcionalidades](#funcionalidades)
- [Comportamento de Aprendizado](#comportamento-de-aprendizado)
- [Métricas](#métricas)
- [Execução](#execução)
- [Estrutura](#estrutura)
- [Instalação](#instalação)
- [Configuração Node-RED](#configuração-node-red)
- [Requisitos Atendidos](#requisitos-atendidos)
- [Autor](#autor)

---

## Visão Geral

O projeto simula um robô aspirador móvel diferencial, equipado com 5 sensores ultrassônicos, capaz de navegar autonomamente, mapear ambientes e otimizar rotas a partir de execuções anteriores. A interface de supervisão e visualização é feita via Node-RED.

---

## Funcionalidades

| Funcionalidade         | Descrição                                      |
|------------------------|------------------------------------------------|
| Navegação Autônoma     | Varredura sistemática (boustrophedon)          |
| Evasão de Obstáculos   | 5 sensores ultrassônicos                       |
| Mapeamento 2D          | Grid de ocupação construído em tempo real      |
| Aprendizado            | Usa mapa anterior para otimizar rotas          |

---

## Comportamento de Aprendizado

| Execução | Comportamento                                 |
|----------|-----------------------------------------------|
| 1ª       | Exploração completa, cria mapa                |
| 2ª       | Usa mapa salvo, evita áreas já limpas         |
| 3ª+      | Otimização refinada                           |

---

## Métricas

| Métrica       | Objetivo    |
|---------------|-------------|
| Cobertura (%) | Maximizar   |
| Tempo         | Minimizar   |
| Energia (J)   | Minimizar   |
| Colisões      | Minimizar   |

---

## Execução

```bash
cd Robo-Aspirador
python main.py
```

| Argumento      | Descrição             | Padrão |
|----------------|----------------------|--------|
| `--executions` | Número de execuções  | 3      |
| `--time`       | Tempo máximo (s)     | 90     |
| `--no-gui`     | Sem interface        | False  |

---

## Estrutura

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

- ✅ Robô móvel diferencial
- ✅ 5 sensores ultrassônicos
- ✅ Exploração autônoma
- ✅ Mapeamento de ocupação
- ✅ Aprendizado por repetição
- ✅ Supervisão Node-RED

---

## 👥 Autor

**João Luiz Ferreira Cardozo** - [@JoaoLCardozo](https://github.com/JoaoLCardozo)




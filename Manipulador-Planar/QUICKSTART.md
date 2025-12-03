# Como executar o Projeto Robotica

## ✓ Todos os testes passaram!

## Opção 1: Testar sem GUI (recomendado primeiro)
```bash
cd "c:\Users\joluc\OneDrive\Documents\GitHub\ProjetoRobotica\Manipulador-Planar"
python test_basic.py
```

Você verá:
- ✓ Todos os módulos carregando corretamente
- ✓ Cinemática FK/IK funcionando
- ✓ PID computado corretamente
- ✓ Payload de perturbação funcionando
- ✓ Detecção de colisão ativa

## Opção 2: Executar simulação com GUI (modo interativo)
```bash
cd "c:\Users\joluc\OneDrive\Documents\GitHub\ProjetoRobotica\Manipulador-Planar"
python projeto_robotica_q1.py
```

**Importante**: O GUI do PyBullet abre em janela separada. Você pode:
- Controlar a câmera com mouse/scroll
- Usar os sliders para ajustar Kp, Ki, Kd das juntas
- Clicar em botões (`grasp_toggle`, `release`, `autotune`)
- Pausar a simulação com espaço

## Opção 3: Testar com payload (perturbação)
Edite `projeto_robotica_q1.py` linha 21:
```python
ENABLE_PERTURBATION_TEST = False  # Mude para True
```

Depois execute:
```bash
python projeto_robotica_q1.py
```

O braço será simulado com 0.3 kg de carga. O controle PID se ajustará automaticamente.

## Troubleshooting

**Problema**: "ModuleNotFoundError: No module named 'pybullet'"
```bash
python -m pip install -r requirements.txt
```

**Problema**: GUI não abre
- PyBullet GUI requer display gráfico (X11 no Linux, DirectX no Windows)
- Use `test_basic.py` para testar sem GUI
- Ou rode em um computador com display

**Problema**: Erro de colisão/IK
- Todos os testes de unidade passam em `test_basic.py`
- Se `projeto_robotica_q1.py` falhar, verifique se `planar_arm.urdf` existe

## Estrutura dos Arquivos

```
projeto_robotica_q1.py    - Script principal (com GUI)
test_basic.py             - Testes rápidos (sem GUI)
pid.py                    - Controlador PID
robot_arm.py              - Cinemática + colisão
planner.py                - Planejamento (evasão)
perturbation.py           - Teste de perturbação (payload)
logger_node_red.py        - Log para MQTT/HTTP
planar_arm.urdf           - Descrição do braço (auto-gerado)
requirements.txt          - Dependências Python
README.md                 - Documentação completa
QUICKSTART.md             - Este arquivo
```

## Próximos Passos

1. **Rodar teste básico** (sem GUI):
   ```bash
   python test_basic.py
   ```

2. **Rodar simulação** (com GUI):
   ```bash
   python projeto_robotica_q1.py
   ```

3. **Ajustar parâmetros** na janela do PyBullet:
   - Sliders de Kp, Ki, Kd
   - Botões de grasp/release
   - Ativar autotune

4. **Verificar logs**:
   - Console mostra erros de colisão detectados
   - Logs MQTT enviados (se broker configurado)

---

✓ Projeto funcional! Pronto para usar.

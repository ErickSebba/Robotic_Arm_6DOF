# BIMO – simulação no Wokwi

Simula o ESP32 com o firmware real, 6 drivers de passo, 6 motores e 6 encoders MT6701.
A pinagem vem de `netlist.py`, a mesma fonte que gera o esquemático do Fusion.

## Como abrir

1. Acesse wokwi.com, clique em **New Project** e escolha **ESP32**.
2. Apague o conteúdo do `sketch.ino` e do `diagram.json` padrão.
3. Crie os arquivos abaixo e cole o conteúdo de cada um. Use o botão "+" / "New file…" ao lado das abas.
   - `sketch.ino`
   - `diagram.json`
   - `bimo_fw.cpp`
   - `bimo_core.h`
   - `pinout.h`
   - `mt6701.chip.c`
   - `mt6701.chip.json`
4. Clique em **Play**. O Wokwi compila o chip customizado e o firmware na nuvem.

## Testar (Serial Monitor, 115200)

```
S                          status
M 30 -10 45 5 -60 90       move as 6 juntas (graus), perfil sincronizado
MODE SEQ                   uma junta por vez (J1..J6)
M 0 0 0 0 0 0
```

- No encoder de uma junta, aumente o controle **slip** ("Passos perdidos por 1000") para
  injetar perda de passo. A malha fechada tem de corrigir o erro. Se o erro passar de
  2 passos inteiros, a junta entra em falha. Para limpar a falha, envie `CLR`.
- Os motores mostram o ângulo **do eixo do motor**, não o da junta. Para a junta, divida pela
  relação de redução (38,4:1).

## Diferenças para o hardware real

| Wokwi | Bancada |
|---|---|
| A4988 (MS1..MS3 em nível alto = 1/16) | DRV8825 (jumper **só em M2** = 1/16) |
| Encoder ideal: conta os pulsos STEP/DIR | MT6701 real: ímã no eixo traseiro, precisa de calibração (`ZERO`) |
| Sem corrente, sem torque, sem folga | Ajustar Vref (I = 2 × Vref), começar em ~0,6 V |
| Wi-Fi `Wokwi-GUEST` | Trocar `WIFI_SSID`/`WIFI_PASS` em `bimo_fw.cpp` |

Antes do primeiro teste físico:

- Medir a relação real das EBA-17-S (J4 a J6). A contagem de dentes deu 35,444, e não 38,4.
  Corrija `GEAR[]` em `bimo_core.h`.
- Conferir o sentido de cada junta. Ajuste em `DIR_SIGN[]` e `ENC_POL[]` em `bimo_fw.cpp`.

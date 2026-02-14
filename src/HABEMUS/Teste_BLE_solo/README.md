# Testes para conexão Bluetooth
Este repositório contém os testes realizados para estabelecer a conexão Bluetooth entre um dispositivo celular ou PC e um microcontrolador. O objetivo é criar uma comunicação eficiente e confiável para troca de dados entre os dois dispositivos.

OBS: Este código foi testado e funciona tanto com o Python (BT.py) quanto com o aplicativo LightBlue para celular.

## Estrutura do código

BT.py - Contém as funções para estabelecer a conexão Bluetooth, enviar e receber dados no PC.

## Bibliotecas externas
- Para o python:

```bash
import asyncio
import csv
from datetime import datetime
from bleak import BleakClient, BleakScanner, BleakError
```

- Todas as [libs para o BLE](https://github.com/espressif/arduino-esp32/tree/ee021855a156847dfcf75a9aeb8d585213f09e42/libraries/BLE) são do Espressif Systems.


## Histórico de modificações do código

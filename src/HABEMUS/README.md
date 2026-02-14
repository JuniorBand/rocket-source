## Estrutura de Pastas
O código foi dividido em arquivos, de acordo com sua implementação com ou sem Bluetooth (Low Energy ou Classic), se utiliza 1 ou 2 sensores para cada medida e etc:

    HABEMUS/
    ├── src/
    │ └── HABEMUS/
    │ ├── HABEMUS_BLE/ 
    │ │ └── Functions.h -> Funções para o BLE
    │ │ └── HABEMUS_BLE.ino -> Código Fonte (BT Low Energy + 2 sensores)
    │ ├── HABEMUS_classic_bluetooth/
    │ │ └── HABEMUS_classic_bluetooth.ino -> (BT Classic + 2 sensores)
    │ └── HABEMUS_no_bluetooth/
    │ │ └── HABEMUS_no_bluetooth.ino -> (Sem BT + 2 sensores)
    │ └── Teste_BLE_solo/
    │ │ └── BT.py -> Código para o PC (BT Low Energy)
    │ │ └── Functions.h -> Funções para o BLE
    │ │ └── Teste_BLE_solo.ino -> (BT Low Energy)
    │ ├── CODIGO_HABEMUS_1_2/
    │ │ └── CODIGO_HABEMUS_1_2.ino
    │ ├── CODIGO_IMPROVISADO_PARA_TESTE_COM_DISPLAY_BMP180_HABEMUS_1_2/
    │ │ └── CODIGO_IMPROVISADO_PARA_TESTE_COM_DISPLAY_BMP180_HABEMUS_1_2.ino
    │ ├── Kalman/
    │ │ └── Kalman.ino -> Código com filtro de Kalman Unscented
    ├── Random_Access_Codes/
    ├── .gitignore
    └── README.md

Cada código tem o seu próprio README.md que facilita a leitura individual.
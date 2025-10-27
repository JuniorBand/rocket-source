# rocket-source
//Descreva o foguete aqui (sintaxe do .md: [(https://encurtador.com.br/iRkUC)](https://encurtador.com.br/iRkUC))

## Estrutura de Pastas
O código foi dividido em arquivos, de acordo com sua implementação com ou sem Bluetooth (Low Energy ou Classic), se utiliza 1 ou 2 sensores para cada medida e etc:

    HABEMUS/
    ├── src/
    │ └── HABEMUS/
    │ ├── HABEMUS_BLE/ 
    │ │ └── HABEMUS_BLE.ino -> Código Fonte (BT Low Energy + 2 sensores)
    │ ├── HABEMUS_classic_bluetooth/
    │ │ └── HABEMUS_classic_bluetooth.ino -> (BT Classic + 2 sensores)
    │ └── HABEMUS_no_bluetooth/
    │ └── HABEMUS_no_bluetooth.ino -> (Sem BT + 2 sensores)
    │ └── Teste_bluetooth_solo/
    │ └── Teste_bluetooth_solo.ino -> (BT Low Energy)
    ├── .gitignore
    └── README.md

Cada código tem o seu próprio README.md que facilita a leitura individual.
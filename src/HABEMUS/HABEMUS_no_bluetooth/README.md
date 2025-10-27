# HABEMUS sem Bluetooth
<!-- (sintaxe do .md: [(https://encurtador.com.br/iRkUC)](https://encurtador.com.br/iRkUC)) -->

## Estrutura do código
O mesmo código do HABEMUS BLE, a única diferença é a não utilização do BLE (Bluetooth Low Energy), mas mantém a lógica de leitura de dois sensores ao mesmo tempo (2 barômetros/altímetros e 2 acelerômetros/giroscópios).



## Sensores utilizados
- `BMP280`-> Barômetro/Altímetro -> (m)
- `MPU6050` -> Acelerômetro/Giroscópio -> (m/s^2)/(rad/s)

## Bibliotecas externas
- Adafruit_MPU6050 (https://github.com/adafruit/Adafruit_MPU6050)
- Adafruit_BMP280_Library (https://github.com/adafruit/Adafruit_BMP280_Library/tree/master)

## Histórico de modificações do código

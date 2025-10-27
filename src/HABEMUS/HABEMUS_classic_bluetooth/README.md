# HABEMUS com Classic Bluetooth
<!-- (sintaxe do .md: [(https://encurtador.com.br/iRkUC)](https://encurtador.com.br/iRkUC)) -->


## Estrutura do código
Mesma lógica do HABEMUS BLE, mas aqui se utiliza o BT Classic, e não o Low Energy, o que elimina grande parte da verbosidade e deixa o código mais abstraído.


## Sensores utilizados
- `BMP280`-> Barômetro/Altímetro -> (m)
- `MPU6050` -> Acelerômetro/Giroscópio -> (m/s^2)/(rad/s)

## Bibliotecas externas
- Adafruit_MPU6050 (https://github.com/adafruit/Adafruit_MPU6050)
- Adafruit_BMP280_Library (https://github.com/adafruit/Adafruit_BMP280_Library/tree/master)
- BluetoothSerial para o BT Classic e todas as libs para o BLE são do Espressif Systems, baixado no Board Manager da Arduino IDE (https://github.com/espressif/arduino-esp32/tree/ee021855a156847dfcf75a9aeb8d585213f09e42/libraries/BluetoothSerial)

## Histórico de modificações do código

# HABEMUS BLE
<!-- (sintaxe do .md: [(https://encurtador.com.br/iRkUC)](https://encurtador.com.br/iRkUC)) -->

## Estrutura do código
HABEMUS BLE:
- O código possui uma lógica para dois sensores (2 barômetros/altímetros e 2 acelerômetros/giroscópios) funcionarem ao mesmo tempo.
- O código implementa Bluetooth Low Energy, o que gera um código grande e sujo.
- BLE é mais verboso e chato de configurar do que as bibliotecas do Bluetooth Classic.
- OBS: é definida somente um Characteristic no Service: FullTelemetry. Isso para transmitir todos os dados, isso com a ajuda do MTU (explanado mais abaixo); caso seja necesário (algum erro de dados truncados), talvez seja melhor definir mais Characteristics para transmissão.
- Para mais informações sobre o BT: (https://dronebotworkshop.com/esp32-bluetooth/).
- Funções:
#### 1. `void verificarAltitude(float altitude)`
#### 2. `void escanearI2C(void)`
#### 3. `void finalizarMissao(const __FlashStringHelper *razao)`
#### 4. `void verificarSensores(uint8_t ADDRESS_1, uint8_t ADDRESS_2, uint8_t ADDRESS_3, uint8_t  ADDRESS_4)`
#### 5. `SensorStatus validarSensores(float altitude_atual_1, float altitude_atual_2)`
#### 6. `void printDados(uint16_t tempoDecorrido, float altitude, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2)`
#### 7. `void acionarBuzzer(int timeOn, int timeOff)`
#### 8. `float calcularMedia(float buffer_leituras[])`
#### 9. `void enviarTabelaBluetooth(uint16_t tempoDecorrido, float altitude, float acel_x, float acel_y, float acel_z, float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado)`
#### 10. `void atualizarBuffer(float leitura)`
#### 11. `void inicializarBLE(const char* deviceName);`

- ***Lógica de configuração do BLE***:

    O sistema de telemetria do foguete utiliza o `Bluetooth Low Energy (BLE)` para transmitir dados em tempo real para um dispositivo externo (como um smartphone ou computador), permitindo o monitoramento do voo.

#### 1. Inicialização do BLE e Configuração do Servidor

Esta seção configura o ESP32 como um servidor BLE, que é o dispositivo que anuncia sua presença e fornece os dados.

+ `BLEDevice::init("HABEMUS_S3_ROCKET");`: Define o nome do dispositivo BLE que será visível para outros dispositivos durante a varredura. No seu caso, o foguete aparecerá como `"HABEMUS_S3_ROCKET"`.

+ `BLEDevice::setMTU(256);`: a transmissão de dados via BLE é otimizada para eficiência energética, o que significa que, por padrão, os pacotes de dados são pequenos (geralmente 20 bytes de dados úteis por pacote). No entanto, a telemetria completa do HABEMUS (altitude, aceleração, giroscópio, tempo e status do paraquedas) é uma string mais longa, com aproximadamente 138 caracteres.

Para garantir que todos os dados sejam transmitidos em um único pacote BLE (o que é mais eficiente e fácil de processar no lado do receptor), o ESP32 é configurado para aceitar uma MTU (Maximum Transmission Unit) maior. A MTU define o tamanho máximo de dados que pode ser transferido em uma única operação BLE.

No código do ESP32, a função `BLEDevice::setMTU()` é usada para indicar que o dispositivo pode lidar com pacotes de até 256 bytes.

+ `Importante`: O ESP32 não inicia a solicitação de um MTU maior. Essa solicitação deve ser feita pelo aplicativo cliente que se conecta ao foguete. Se o cliente não solicitar um MTU maior, a comunicação permanecerá no padrão de 20 bytes, e os dados de telemetria serão truncados.

+ `pServer = BLEDevice::createServer();`: Cria a instância do servidor BLE. Este servidor será responsável por gerenciar as conexões e os serviços BLE.

+ `pServer->setCallbacks(new MyServerCallbacks());`: Associa uma classe de callbacks (MyServerCallbacks) ao servidor. Isso permite que o código reaja a eventos importantes do BLE, como conexão e desconexão de clientes.

#### 2. Definição do Serviço BLE

Um serviço BLE é uma coleção de características que agrupam dados relacionados. Pense nele como uma "categoria" de informações que o servidor oferece.

+ ***`#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"`***: Um `UUID (Universally Unique Identifier)` é um identificador único de 128 bits para o seu serviço. É crucial que este UUID seja único para evitar conflitos com outros serviços BLE.

+ ***`BLEService *pService = pServer->createService(SERVICE_UUID);`***: Cria o serviço BLE com o UUID definido. Este serviço conterá as características que serão transmitidas.

#### 3. Definição e Propriedades das Características BLE

Características são os itens de dados reais que o servidor BLE fornece. Cada característica tem um UUID único e propriedades que definem como os dados podem ser acessados.

No projeto, as seguintes características foram definidas:

+ ***Altitude*** (`ALTITUDE_CHARACTERISTIC_UUID`): Transmite a altitude atual do foguete.

+ ***Aceleração/Giroscópio*** (`ACCEL_CHARACTERISTIC_UUID`): Transmite os dados combinados dos acelerômetros e giroscópios.

+ ***Status do Paraquedas*** (`PARACHUTE_CHARACTERISTIC_UUID`): Indica se o paraquedas foi acionado ou não.

Para cada característica, foram definidas as seguintes propriedades:

+ ***`BLECharacteristic::PROPERTY_READ`***: Permite que um dispositivo cliente (quem se conecta ao foguete) leia o valor atual da característica.

+ ***`BLECharacteristic::PROPERTY_NOTIFY`***: Permite que o servidor envie notificações automáticas para clientes inscritos sempre que o valor da característica for atualizado. Isso é fundamental para a telemetria em tempo real.

+ ***`pCharacteristic->addDescriptor(new BLE2902())`***;: Adiciona um descritor padrão BLE2902, que é necessário para características que utilizam a propriedade NOTIFY. Ele permite que o cliente habilite ou desabilite o recebimento de notificações.

#### 4. Configuração do Advertising (Anúncio)

O advertising é o processo pelo qual o servidor BLE anuncia sua presença para que outros dispositivos possam descobri-lo e se conectar.

+ ***`pAdvertising = BLEDevice::getAdvertising();`***: Obtém o objeto de advertising do dispositivo BLE.

+ ***`pAdvertising->addServiceUUID(SERVICE_UUID);`***: Inclui o UUID do serviço principal nos dados de advertising. Isso permite que os clientes saibam qual serviço o dispositivo oferece antes mesmo de se conectar.

+ ***`pAdvertising->setScanResponse(true);`***: Habilita a resposta de scan, permitindo que o dispositivo envie informações adicionais quando é detectado por um scanner.

+ ***`pAdvertising->setMinPreferred(0x06);` `pAdvertising->setMinPreferred(0x12);`***: Define intervalos de conexão preferenciais. Valores menores geralmente indicam uma preferência por conexões mais rápidas, à custa de um consumo de energia potencialmente maior.

+ ***`BLEDevice::startAdvertising()`***;: Inicia o processo de anúncio, tornando o `"HABEMUS S3 ROCKET"` detectável por outros dispositivos BLE.

#### 5. Lógica de Conexão e Desconexão (`MyServerCallbacks`)
A classe `MyServerCallbacks` é crucial para gerenciar o estado da conexão BLE:

+ ***`void onConnect(BLEServer* pServer)`***: Este método é chamado automaticamente pela biblioteca quando um cliente se conecta ao servidor. Ele atualiza a variável global `deviceConnected` para true e imprime uma mensagem no monitor serial.

+ ***`void onDisconnect(BLEServer* pServer)`***: Este método é chamado quando um cliente se desconecta. Ele atualiza `deviceConnected` para false e, muito importante, chama `pServer->startAdvertising()`; novamente para que o foguete possa ser redescoberto e novas conexões sejam feitas.

#### 6. Envio de Dados via BLE (`enviarTabelaBluetooth()`)

+ A função `enviarTabelaBluetooth()` é responsável por formatar os dados dos sensores e enviá-los para o cliente conectado.

+ ***Verificação de Conexão***: if (`deviceConnected`) garante que os dados só sejam enviados se houver um cliente conectado, evitando erros e consumo desnecessário de recursos.

+ ***Formatação dos Dados***: Os dados de altitude, aceleração/giroscópio e status do paraquedas são formatados em strings (e futuramente, idealmente em buffers de caracteres para otimização de memória) para serem facilmente legíveis pelo cliente.

+ ***`pCharacteristic->setValue(...)`***: Define o valor da característica com a string formatada.

+ ***`pCharacteristic->notify()`***: Dispara uma notificação para o cliente conectado, enviando o novo valor da característica em tempo real. Um pequeno delay(50) é adicionado para evitar sobrecarga no barramento BLE.

`Espaço` (ESP32S3 Dev Module):

        Sketch uses 1032054 bytes (78%) of program storage space. Maximum is 1310720 bytes. Global variables use 48344 bytes (14%) of dynamic memory, leaving 279336 bytes for local variables. Maximum is 327680 bytes.

## Apps para conexão Bluetooth
Aplicativos Recomendados para Monitoramento BLE
Para se conectar ao HABEMUS e visualizar a telemetria:

+ ***`nRF Connect (Recomendado)`***

    `Plataformas`: Android, iOS

    `Funcionalidade`: É a ferramenta mais robusta para depuração BLE. Ela permite que você visualize todos os serviços e características, e o mais importante, oferece a opção de solicitar uma MTU maior.

    `Como usar para MTU`: Após conectar-se ao "HABEMUS_S3_ROCKET", procure o ícone de três pontos (⋮) no canto superior direito da tela de detalhes da conexão. No menu que se abre: Configurações -> Connectivity, selecione a opção "Auto Request Maximum MTU" (ou "Request MTU" e insira um valor como 256 ou 517). Isso garantirá que você receba a string de telemetria completa.

+ ***`LightBlue`***

    `Plataformas`: Android, iOS

    `Funcionalidade`: Um aplicativo mais simples e intuitivo para explorar dispositivos BLE. Ele geralmente tenta negociar uma MTU maior automaticamente (especialmente em dispositivos iOS mais recentes), mas pode não oferecer o mesmo nível de controle explícito que o nRF Connect.

## Sensores utilizados
- `BMP280`-> Barômetro/Altímetro -> (m)
- `MPU6050` -> Acelerômetro/Giroscópio -> (m/s^2)/(rad/s)

## Bibliotecas externas
- Adafruit_MPU6050 (https://github.com/adafruit/Adafruit_MPU6050)
- Adafruit_BMP280_Library (https://github.com/adafruit/Adafruit_BMP280_Library/tree/master)
- BluetoothSerial para o BT Classic e todas as libs para o BLE são do Espressif Systems, baixado no Board Manager da Arduino IDE (https://github.com/espressif/arduino-esp32/tree/ee021855a156847dfcf75a9aeb8d585213f09e42/libraries/BluetoothSerial)


## Mudanças necessárias
- comunicação de bluetooth para checar as peças
- receber as informações dos 2 BMP e tirar as médias e no caso de só receber o sinal de 1, usa o valor do único que recebeu
- gerar a saída conforme falei, contendo todas aquelas informações

Não precisa mostrar a saída na telinha nem nada. Ele vai dar o resultado por meio de um comando bluetooth.

Posteriormente vou mandar pra vocês os módulos de cartão de memória que temos. porém precisamos fazer isso até dia 14, pois precisamos enviar um documento para a organização mostrando tudo montado.

- `(OK)` comunicação de bluetooth para checar as peças
- `(OK)` receber as informações dos 2 BMP e tirar as médias e no caso de só receber o sinal de 1, usa o valor do único que recebeu
- `(OK)` fazer o mesmo do BMP, mas para MPU6050
- `(OK)` gerar a saída conforme falei, contendo todas aquelas informações
- `(OK)` Escrever em um cartão de memória
- `(OK)` Tempo
- `(PENDENTE)` Menu de Potência do Foguete


## Histórico de modificações do código (OBS: Precisa ser Atualizado)
### 1. _Substituição de Hardware_:

    - O `servo motor` foi substituído por um `módulo relê` (mantendo o mesmo pino de controle).

    - Toda a lógica e bibliotecas relacionadas ao `cartão SD` foram removidas.

### 2. _Configuração do Sensor BMP280_:

    - Implementação da comunicação I2C utilizando os pinos `A4 (SDA)` e `A5 (SCL)`.

    - O sensor `BMP280` agora é inicializado diretamente no endereço `I2C 0x76`.

### 3. _Melhorias na Inicialização e Feedback_:

    - O sistema agora toca 1 bip ao iniciar o programa.

    - Após a tentativa de inicialização do BMP280, o sistema toca 2 bips para indicar sucesso e início da calibração.

### 4. _Lógica de Calibração e Medição de Altitude_:

    - A altitude inicial é calibrada fazendo uma média das leituras do BMP280. Este valor (`leitura_inicial_fixa`) se torna o ponto de referência "zero" para todas as altitudes subsequentes.

    - Após a calibração inicial, a altitude exibida é a 
    édia móvel das últimas leituras
    do sensor, subtraída da `leitura_inicial_fixa`, para suavizar a leitura.

### 5. _Saída do Terminal (Monitor Serial)_:

    - A saída foi simplificada para mostrar apenas a altitude de referência (zero) uma vez, e depois, continuamente, as altitudes atualizadas.

### 6. _Funções de Erro/Depuração_:

    - A função `escanearI2C()` foi mantida como uma ferramenta de depuração em caso de falha de inicialização do sensor.

### 7. _Generalização de Idioma_:

    - Todas as variáveis, funções e comentários foram traduzidos para o português.

### 8. _OTIMIZAÇÃO DE MEMÓRIA (RAM)_:

    - Todas as strings literais impressas no Serial Monitor foram movidas para a `memória Flash (PROGMEM)` usando a macro `F()`.

### 9. _UNIFICAÇÃO DO PROTOCOLO FINAL_:

    - Criada uma única função `finalizarMissao()` para centralizar todos os procedimentos de fim de voo, garantindo que o comportamento (relê e buzzer) seja idêntico em acionamento normal ou falha.

### 10. _ADAPTAÇÃO PARA FOGUETE DE ALTO DESEMPENHO_:

    - A frequência do loop foi aumentada (removido delay) e o tamanho da média móvel foi reduzido para maior reatividade.

    - A lógica de acionamento do paraquedas foi ajustada para um voo de alta performance (`apogeu - 20m`).

    - As configurações do sensor `BMP280` foram otimizadas para velocidade em vez de precisão máxima.

### 11. _REFINAMENTO DAS POLÍTICAS DE SEGURANÇA_:

    - Adicionada detecção de leituras anômalas (`salto > 75m ou 3 leituras zeradas`) como condições de falha crítica.

### 12. _IMPLEMENTAÇÃO DE DETECÇÃO DE LANÇAMENTO (CORREÇÃO DE BUG CRÍTICO)_:

    - Adicionada uma máquina de estados (`foguete_lancado`) para diferenciar as fases de "pronto na base" e "em voo".
    - A lógica de apogeu e ejeção só é "armada" após o foguete ultrapassar uma altitude de segurança (`30m`), corrigindo o acionamento prematuro que ocorria com o foguete no solo.
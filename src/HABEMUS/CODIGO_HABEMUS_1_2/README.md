# HABEMUS v1.2
<!-- (sintaxe do .md: [(https://encurtador.com.br/iRkUC)](https://encurtador.com.br/iRkUC)) -->

## Estrutura do Código
O código do computador de voo do foguete é estruturado para operar de forma eficiente em um microcontrolador Arduino, seguindo a arquitetura básica de `setup()` e `loop()`, mas com diversas otimizações para desempenho e clareza.

**Constantes e Definições (`#define` e `const`):**
No início do código, são definidas constantes em `SCREAMING_SNAKE_CASE` para os pinos de hardware (`PINO_BUZZER`, `PINO_MOSFET_1`, `PINO_MOSFET_2`, `LED_STATUS_PIN`) e para os limites e configurações de voo (`LIMITE_SALTO_ANOMALO`, `ALTITUDE_DE_LANCAMENTO`, `ALTITUDE_LIMITE_X`, `DESCIDA_MINIMA_Y`). Isso centraliza as configurações e melhora a legibilidade.

**Instanciação do Objeto do Sensor (`Adafruit_BMP280 bmp;`):**
O objeto principal para interagir com o sensor BMP280 é declarado globalmente. Embora a filosofia geral de desenvolvimento moderno tenda a evitar variáveis globais, esta é a abordagem padrão e mais prática para objetos de hardware no ambiente `setup()`/`loop()` do Arduino, garantindo que o sensor seja inicializado uma vez em `setup()` e acessível continuamente em `loop()` e suas sub-rotinas.

**Protótipos de Funções:**
Todas as funções personalizadas são prototipadas antes de `setup()`. As funções seguem a convenção `camelCase` e demonstram o uso de passagem de parâmetros por referência (`&`) para variáveis de estado (como `apogeu_ref`, `acionamento_iniciado_ref`, `foguete_lancado_ref`) e ponteiros de função (`void (*finalizar)(...)`) para maior modularidade, permitindo que as sub-rotinas alterem o estado principal do programa e chamem outras funções de forma flexível.

**`setup()` Função:**
Executada uma única vez na inicialização do Arduino.
* Configura os pinos de entrada/saída (`pinMode`).
* Inicializa a comunicação serial (`Serial.begin`) e I2C (`Wire.begin`).
* Exibe um "menu" inicial informativo no Monitor Serial.
* Realiza a inicialização e configuração do sensor BMP280, incluindo a otimização para velocidade máxima (`MODE_FORCED`, `FILTER_OFF`, `STANDBY_MS_1`).
* Contém uma lógica de verificação de erro crítico para o sensor, que, em caso de falha, aciona a `finalizarMissao()`.

**`loop()` Função:**
É a função principal do programa e é executada repetidamente após `setup()`.
* **Gerenciamento de Estado com `static` Locais:** A maioria das variáveis de estado que precisam persistir entre as iterações do `loop()` (como `apogeu`, `calibracao_inicial_concluida`, `buffer_leituras`, `foguete_lancado`) são declaradas como `static` locais. Isso mantém o escopo das variáveis dentro do `loop()`, minimizando o uso de variáveis globais e melhorando a encapsulamento do estado.
* Realiza a leitura contínua da altitude através de `lerAltitudeBmp280()`.
* Implementa a lógica de calibração inicial (média de 5 leituras).
* Contém a máquina de estados para detecção de lançamento (`foguete_lancado`).
* Calcula e exibe a altitude relativa (com média móvel).
* Monitora leituras anômalas (`NaN`, zeros consecutivos, saltos grandes) e aciona `finalizarMissao()` em caso de falha.
* Chama `verificaAltitude()` para implementar as lógicas complexas de acionamento de recuperação.
* É otimizada para velocidade, sem `delay()` bloqueantes que possam comprometer a taxa de amostragem durante o voo, permitindo o uso do sensor no modo forçado.

**Funções Auxiliares (`lerAltitudeBmp280()`, `finalizarMissao()`, `verificaAltitude()`):**
* `lerAltitudeBmp280()`: Centraliza a lógica de leitura do sensor, incluindo a solicitação de medição no modo forçado e o feedback visual através do LED embutido.
* `finalizarMissao()`: Uma função unificada para lidar com o término do voo (seja por sucesso no acionamento ou por falha crítica). Ela aciona os MOSFETs e entra em um loop infinito de sinalização visual e sonora.
* `verificaAltitude()`: Contém a lógica de negócios para decidir quando acionar o sistema de recuperação, baseada nas múltiplas regras de apogeu alto/baixo e descida mínima, recebendo os estados necessários por referência e chamando `finalizarMissao()` quando as condições são atendidas.

**Otimização de Memória (`F()` macro):**
Todas as strings literais usadas para impressão serial (`Serial.println()`, `Serial.print()`) são encapsuladas na macro `F()`. Isso as armazena na memória Flash (`PROGMEM`) do Arduino em vez da RAM, conservando recursos preciosos para as variáveis do programa.

Em suma, a estrutura do código é um equilíbrio entre a simplicidade do ambiente Arduino e a robustez necessária para uma aplicação crítica como um computador de voo de foguetes, priorizando desempenho, segurança e clareza.

---

## Sensores utilizados
O computador de voo utiliza um sensor barométrico para a medição de altitude, que é fundamental para a navegação e a detecção de eventos de voo.

**Sensor Barométrico BMP280:**
* **Função:** O BMP280 é um sensor de pressão barométrica e temperatura de alta precisão e baixo consumo de energia. Sua principal função no computador de voo é medir a pressão atmosférica, que é então convertida em altitude.
* **Comunicação:** Ele se comunica com o microcontrolador Arduino através do protocolo I2C. Os pinos específicos utilizados para essa comunicação são **A4 (SDA)** para dados e **A5 (SCL)** para o clock.
* **Modo de Operação:** No código, o BMP280 é configurado para operar no **Modo Forçado (`MODE_FORCED`)**. Este modo é escolhido por proporcionar o maior controle sobre o momento das medições, permitindo que o sistema solicite uma nova leitura assim que a anterior for concluída. Isso maximiza a frequência de amostragem e, consequentemente, a capacidade de detectar mudanças rápidas na altitude, como o apogeu.
* **Otimização de Desempenho:** As configurações de oversampling para pressão (`SAMPLING_X16`) e a desativação do filtro (`FILTER_OFF`) são aplicadas para otimizar a velocidade de resposta do sensor, crucial para aplicações dinâmicas como o voo de um foguete. O tempo de espera entre medições é minimizado para 1ms, embora no Modo Forçado, a taxa seja determinada pela velocidade de conversão e aquisição de dados.

A combinação da precisão do BMP280 com sua configuração otimizada para velocidade permite que o computador de voo monitore a trajetória do foguete com alta fidelidade, essencial para as lógicas de detecção de lançamento, apogeu e acionamento da recuperação.

---

## Bibliotecas externas
Para habilitar as funcionalidades do sensor barométrico e a comunicação I2C, o código utiliza duas bibliotecas externas principais. Essas bibliotecas simplificam a interação com o hardware, abstraindo a complexidade dos protocolos de comunicação de baixo nível.

**`Wire.h` (Biblioteca de Comunicação I2C):**
* **Propósito:** Esta é uma biblioteca padrão do Arduino que facilita a comunicação com dispositivos que utilizam o protocolo I2C (também conhecido como Two-Wire Interface ou TWI).
* **Uso no Código:** É usada implicitamente pelas bibliotecas dos sensores para estabelecer a comunicação com o BMP280. A chamada `Wire.begin()` no `setup()` inicializa o barramento I2C, permitindo que o Arduino atue como mestre na comunicação.

**`Adafruit_BMP280.h` (Biblioteca para o Sensor BMP280):**
* **Propósito:** Desenvolvida pela Adafruit, esta biblioteca fornece uma interface de alto nível para interagir com o sensor BMP280. Ela gerencia a leitura de dados brutos de pressão e temperatura e realiza os cálculos necessários para convertê-los em valores úteis, como altitude.
* **Uso no Código:**
    * Permite a criação de um objeto `Adafruit_BMP280` (`bmp;`).
    * Facilita a inicialização do sensor (`bmp.begin()`).
    * Oferece métodos para configurar o modo de operação e os parâmetros de amostragem (`bmp.setSampling()`).
    * Permite solicitar e ler medições de pressão, temperatura e altitude (`bmp.takeForcedMeasurement()`, `bmp.readAltitude()`).

A utilização dessas bibliotecas agiliza o desenvolvimento ao fornecer funções prontas para uso que interagem diretamente com o hardware complexo, permitindo que o foco do projeto seja na lógica de voo e nas políticas de segurança.

---

## Histórico de modificações do código
1.  ### _Substituição de Hardware_:
    * O `servo motor` foi substituído por um `módulo relê` (mantendo o mesmo pino de controle).
    * Toda a lógica e bibliotecas relacionadas ao `cartão SD` foram removidas.
2.  ### _Configuração do Sensor BMP280_:
    * Implementação da comunicação I2C utilizando os pinos `A4 (SDA)` e `A5 (SCL)`.
    * O sensor `BMP280` agora é inicializado diretamente no endereço `I2C 0x76`.
3.  ### _Melhorias na Inicialização e Feedback_:
    * O sistema agora toca 1 bip ao iniciar o programa.
    * Após a tentativa de inicialização do BMP280, o sistema toca 2 bips para indicar sucesso e início da calibração.
4.  ### _Lógica de Calibração e Medição de Altitude_:
    * A altitude inicial é calibrada fazendo uma média das leituras do BMP280. Este valor (`leitura_inicial_fixa`) se torna o ponto de referência "zero" para todas as altitudes subsequentes.
    * Após a calibração inicial, a altitude exibida é a média móvel das últimas leituras do sensor, subtraída da `leitura_inicial_fixa`, para suavizar a leitura.
5.  ### _Saída do Terminal (Monitor Serial)_:
    * A saída foi simplificada para mostrar apenas a altitude de referência (zero) uma vez, e depois, continuamente, as altitudes atualizadas.
6.  ### _Funções de Erro/Depuração_:
    * A função `escanearI2C()` foi mantida como uma ferramenta de depuração em caso de falha de inicialização do sensor.
7.  ### _Generalização de Idioma_:
    * Todas as variáveis, funções e comentários foram traduzidos para o português.
8.  ### _OTIMIZAÇÃO DE MEMÓRIA (RAM)_:
    * Todas as strings literais impressas no Serial Monitor foram movidas para a `memória Flash (PROGMEM)` usando a macro `F()`.
9.  ### _UNIFICAÇÃO DO PROTOCOLO FINAL_:
    * Criada uma única função `finalizarMissao()` para centralizar todos os procedimentos de fim de voo, garantindo que o comportamento (relê e buzzer) seja idêntico em acionamento normal ou falha.
10. ### _ADAPTAÇÃO PARA FOGUETE DE ALTO DESEMPENHO_:
    * A frequência do loop foi aumentada (removido delay) e o tamanho da média móvel foi reduzido para maior reatividade.
    * A lógica de acionamento do paraquedas foi ajustada para um voo de alta performance (`apogeu - 20m`).
    * As configurações do sensor `BMP280` foram otimizadas para velocidade em vez de precisão máxima.
11. ### _REFINAMENTO DAS POLÍTICAS DE SEGURANÇA_:
    * Adicionada detecção de leituras anômalas (`salto > 75m ou 3 leituras zeradas`) como condições de falha crítica.
12. ### _IMPLEMENTAÇÃO DE DETECÇÃO DE LANÇAMENTO (CORREÇÃO DE BUG CRÍTICO)_:
    * Adicionada uma máquina de estados (`foguete_lancado`) para diferenciar as fases de "pronto na base" e "em voo".
    * A lógica de apogeu e ejeção só é "armada" após o foguete ultrapassar uma altitude de segurança (`30m`), corrigindo o acionamento prematuro que ocorria com o foguete no solo.

---

## Histórico de Modificações do Código (Versão BMP280)

1.  ### _Substituição e Otimização de Hardware_:
    * O `servo motor` foi substituído por um `módulo relé` (mantendo o mesmo pino de controle, agora `PINO_MOSFET_1` e `PINO_MOSFET_2`).
    * Toda a lógica e bibliotecas relacionadas ao `cartão SD` foram removidas para simplificar e otimizar o código.

2.  ### _Configuração e Otimização do Sensor BMP280_:
    * Implementação da comunicação I2C utilizando os pinos **A4 (SDA)** e **A5 (SCL)**.
    * O sensor `BMP280` agora é inicializado com detecção automática do endereço I2C (geralmente `0x76` ou `0x77`).
    * As configurações do sensor `BMP280` foram otimizadas para velocidade máxima (modo forçado, oversampling de pressão x16, filtro desativado, tempo de espera de 1ms). Isso garante a maior taxa de leitura possível para detecção precisa do apogeu.

3.  ### _Melhorias na Inicialização e Feedback_:
    * O sistema agora toca 1 bip ao iniciar o programa, seguido por 2 bips para indicar sucesso na inicialização do BMP280 e o início da calibração.
    * **LED de Status Visual:** O LED embutido do Arduino (`LED_BUILTIN`) pisca uma vez brevemente a cada leitura de altitude bem-sucedida, fornecendo feedback visual contínuo do funcionamento do sensor.
    * **Menu Inicial Informativo:** Adicionada uma tela de abertura "COMPUTADOR DE VOO HABEMOS ROCKET v1.2" no Monitor Serial para identificação do sistema.

4.  ### _Lógica de Calibração e Medição de Altitude_:
    * A altitude inicial é calibrada fazendo uma média das 5 primeiras leituras do BMP280. Este valor (`leitura_inicial_fixa`) se torna o ponto de referência "zero" para todas as altitudes subsequentes.
    * Durante a fase de calibração, cada uma das 5 leituras individuais é exibida no Monitor Serial para maior transparência do processo.
    * Após a calibração inicial, a altitude exibida é a média móvel das últimas 5 leituras do sensor, subtraída da `leitura_inicial_fixa`, para suavizar a leitura e fornecer um valor relativo.

5.  ### _Refinamento das Políticas de Segurança e Detecção de Lançamento_:
    * **Detecção de Lançamento Implementada:** Adicionada uma máquina de estados (`foguete_lancado`) para diferenciar as fases de "pronto na base" e "em voo". A lógica de apogeu e ejeção só é "armada" após o foguete ultrapassar uma altitude de segurança (`ALTITUDE_DE_LANCAMENTO`, definida em 30m), corrigindo acionamentos prematuros no solo.
    * **Detecção de Falhas Críticas:**
        * Adicionada detecção de leituras anômalas (salto brusco na altitude > `LIMITE_SALTO_ANOMALO` de 75m) como condição de falha crítica, levando à finalização imediata da missão.
        * Implementada a detecção de 3 leituras zeradas/nulas consecutivas do sensor como condição de falha crítica, acionando a `finalizarMissao()`. Leituras zeradas individuais são ignoradas nos cálculos da média para evitar distorções.

6.  ### _Nova Lógica de Acionamento da Recuperação (Paraquedas)_:
    * A lógica de acionamento do paraquedas foi aprimorada para cobrir dois cenários principais de descida pós-lançamento:
        * **Recuperação por Apogeu Alto:** Se o foguete voou acima de `ALTITUDE_LIMITE_X` (100m) e então desceu `DESCIDA_MINIMA_Y` (10m) abaixo do apogeu, a recuperação é acionada.
        * **Recuperação por Apogeu Baixo/Falha:** Se o foguete, após ser detectado como lançado, atingiu um apogeu abaixo ou igual a `ALTITUDE_LIMITE_X` (100m) e então desceu `DESCIDA_MINIMA_Y` (10m) abaixo do apogeu, a recuperação é acionada. Isso garante a ejeção mesmo em voos com desempenho abaixo do esperado.
    * A ejeção ocorre uma única vez, e o sistema entra em um estado final de "missão finalizada".

7.  ### _Unificação e Refinamento do Protocolo Final_:
    * Criada uma única função `finalizarMissao()` para centralizar todos os procedimentos de fim de voo (acionamento dos MOSFETs, buzzer e LED), garantindo um comportamento idêntico em acionamento normal ou falha crítica.
    * A ativação dos MOSFETs (`PINO_MOSFET_1` e `PINO_MOSFET_2`) é feita com PWM máximo (`255`), permanecendo ativos por 3 segundos antes de serem desligados.
    * No estado de missão finalizada, o buzzer e o LED embutido piscam continuamente.

8.  ### _Otimização de Código e Estilo_:
    * **Otimização de Memória (RAM):** Todas as strings literais impressas no Serial Monitor foram movidas para a memória Flash (`PROGMEM`) usando a macro `F()`, economizando RAM valiosa em microcontroladores.
    * **Consistência de Nomenclatura:** Adotadas convenções de nomenclatura padronizadas: `camelCase` para funções, `snake_case` para variáveis e `SCREAMING_SNAKE_CASE` para constantes `#define` e `const float`.
    * **Modularidade e Clareza:** Uso extensivo de variáveis `static` locais em `loop()` para encapsular o estado, reduzindo o uso de variáveis globais e melhorando a organização do código. Passagem de argumentos por referência (`&`) e ponteiros de função (`*`) para modularidade.
    * **Comentários Detalhados:** Comentários abrangentes foram adicionados e atualizados em todo o código para explicar a lógica e as funcionalidades.
# rocket-source (Por enquanto, o código em desenvolvimento está na branch dev)
//Descreva o foguete aqui (sintaxe do .md: [(https://encurtador.com.br/iRkUC)](https://encurtador.com.br/iRkUC))


## Estrutura do código




## Sensores utilizados



## Bibliotecas externas

## Histórico de modificações do código
1. ### _Substituição de Hardware_:

    - O `servo motor` foi substituído por um `módulo relê` (mantendo o mesmo pino de controle).

    - Toda a lógica e bibliotecas relacionadas ao `cartão SD` foram removidas.

2. ### _Configuração do Sensor BMP280_:

    - Implementação da comunicação I2C utilizando os pinos `A4 (SDA)` e `A5 (SCL)`.

    - O sensor `BMP280` agora é inicializado diretamente no endereço `I2C 0x76`.

3. ### _Melhorias na Inicialização e Feedback_:

    - O sistema agora toca 1 bip ao iniciar o programa.

    - Após a tentativa de inicialização do BMP280, o sistema toca 2 bips para indicar sucesso e início da calibração.

4. ### _Lógica de Calibração e Medição de Altitude_:

    - A altitude inicial é calibrada fazendo uma média das leituras do BMP280. Este valor (`leitura_inicial_fixa`) se torna o ponto de referência "zero" para todas as altitudes subsequentes.

    - Após a calibração inicial, a altitude exibida é a 
    édia móvel das últimas leituras
    do sensor, subtraída da `leitura_inicial_fixa`, para suavizar a leitura.

5. ### _Saída do Terminal (Monitor Serial)_:

    - A saída foi simplificada para mostrar apenas a altitude de referência (zero) uma vez, e depois, continuamente, as altitudes atualizadas.

6. ### _Funções de Erro/Depuração_:

    - A função `escanearI2C()` foi mantida como uma ferramenta de depuração em caso de falha de inicialização do sensor.

7. ### _Generalização de Idioma_:

    - Todas as variáveis, funções e comentários foram traduzidos para o português.

8. ### _OTIMIZAÇÃO DE MEMÓRIA (RAM)_:

    - Todas as strings literais impressas no Serial Monitor foram movidas para a `memória Flash (PROGMEM)` usando a macro `F()`.

9. ### _UNIFICAÇÃO DO PROTOCOLO FINAL_:

    - Criada uma única função `finalizarMissao()` para centralizar todos os procedimentos de fim de voo, garantindo que o comportamento (relê e buzzer) seja idêntico em acionamento normal ou falha.

10. ### _ADAPTAÇÃO PARA FOGUETE DE ALTO DESEMPENHO_:

    - A frequência do loop foi aumentada (removido delay) e o tamanho da média móvel foi reduzido para maior reatividade.

    - A lógica de acionamento do paraquedas foi ajustada para um voo de alta performance (`apogeu - 20m`).

    - As configurações do sensor `BMP280` foram otimizadas para velocidade em vez de precisão máxima.

11. ### _REFINAMENTO DAS POLÍTICAS DE SEGURANÇA_:

    - Adicionada detecção de leituras anômalas (`salto > 75m ou 3 leituras zeradas`) como condições de falha crítica.

12. ### _IMPLEMENTAÇÃO DE DETECÇÃO DE LANÇAMENTO (CORREÇÃO DE BUG CRÍTICO)_:

    - Adicionada uma máquina de estados (`foguete_lancado`) para diferenciar as fases de "pronto na base" e "em voo".
    - A lógica de apogeu e ejeção só é "armada" após o foguete ultrapassar uma altitude de segurança (`30m`), corrigindo o acionamento prematuro que ocorria com o foguete no solo.



# 🚀 RAD (Random Access Drivers) - Avionics Firmware

**RAD** é o núcleo de software embarcado (Firmware) desenvolvido para o controle, telemetria e recuperação de minifoguetes experimentais. Construído para a família STM32F4 (Cortex-M4), o sistema possui uma arquitetura orientada a eventos, *non-blocking*, com foco absoluto em performance de tempo real e tolerância a falhas.

---

## 📂 Estrutura de Diretórios

O projeto está encapsulado de forma modular. Toda a lógica de voo e drivers físicos estão contidos dentro da pasta `RAD`:

```text
RAD/
├── Inc/                    # Arquivos de Cabeçalho (Interfaces e Macros)
│   ├── utils/              # Funcionalidades base e configuração central
│   │   ├── config_voo.h    # Parâmetros de voo, limiares de apogeu e pinagem.
│   │   ├── prints.h        # Macros de telemetria visual (ANSI Colors).
│   │   └── utils.h         # Manipulação bare-metal de GPIO e tipos rígidos.
│   ├── flash_stm.h         # Definições do setor da flash interna.
│   ├── lkf.h               # Definições matemáticas do Filtro de Kalman.
│   ├── ms5611.h            # Registradores e structs do barômetro.
│   ├── usb_com.h           # Protótipos da comunicação USB CDC.
│   └── w25q.h              # Comandos SPI e gerenciamento de endereços da memória.
├── Src/                    # Código Fonte (Implementações)
│   ├── utils/              # Implementações base
│   │   └── config_voo.c    # Máquina de Estados e lógica de voo central.
│   ├── flash_stm.c         # Driver de gravação na Flash Nativa (Mock/Bancada).
│   ├── lkf.c               # Algoritmo de estimação de estado (Kalman Filter).
│   ├── ms5611.c            # Driver non-blocking do barômetro de alta precisão.
│   ├── usb_com.c           # Despachante de comandos da Estação Solo.
│   └── w25q.c              # Driver da "Caixa Preta" (Memória SPI externa).
└── README.md               # Documentação do projeto.
```

---

## 🧠 Arquitetura dos Módulos

* **`config_voo` (O Cérebro):** Gerencia a Máquina de Estados principal (Calibração > Pronto > Voo > Apogeu > Recuperação > Pousado). Processa a fusão de sensores com o Filtro de Kalman e decide o momento exato do acionamento do MOSFET de ejeção.
* **`w25q` (A Caixa Preta):** Controla a memória Flash SPI W25Q128. Utiliza técnica de *Page Buffer* para gravar logs a 100Hz sem desgastar o chip e possui um setor isolado para backup de estado (recuperação de boot em queda livre).
* **`ms5611` (O Sensor):** Lida com a leitura do barômetro em modo de resolução máxima (OSR 4096). Opera de forma assíncrona orientada a *Ticks*, garantindo que o processador nunca congele enquanto aguarda a conversão analógica-digital do chip.
* **`usb_com` (A Estação Solo):** Intercepta comandos vindos da porta Serial/USB via interrupção (IRQ segura) e roteia para execução no laço principal, evitando travamentos do microcontrolador.
* **`prints` & `utils` (Infraestrutura):** Garantem que no "Modo Voo" todo o peso de formatação de strings seja descartado pelo compilador (`Zero Overhead`), além de oferecer funções de *Bare-Metal* GPIO de altíssima velocidade (1 ciclo de clock).

---

## 🎛️ Macros de Configuração (`config_voo.h`)

O comportamento central do firmware é ditado por duas macros de pré-processamento localizadas no arquivo `config_voo.h`. Alterar estas definições muda radicalmente a compilação e a alocação de recursos do código:

* **`#define EM_VOO`**:
    * **Ativado (Descomentado):** Modo de Operação Real. Otimiza o código para o voo, desativando completamente a comunicação USB e substituindo todas as funções de `print` por instruções vazias (`do {} while(0)`). Garante que 100% dos ciclos de clock do processador sejam dedicados à física e aos sensores.
    * **Desativado (Comentado):** Modo Estação Solo/Bancada. Habilita a comunicação USB CDC, permitindo o envio de comandos e a visualização da telemetria em tempo real no terminal do PC.
* **`#define USE_W25Q`**:
    * **Ativado (Descomentado):** Direciona a gravação de dados de telemetria para a "Caixa Preta" externa (Memória Flash SPI W25Q128). Configuração obrigatória para voos reais.
    * **Desativado (Comentado):** Redireciona a gravação para um setor isolado da memória Flash interna do próprio STM32. Útil para testes de bancada rápidos de software (Mock) sem a necessidade de ter o chip SPI conectado.

---

## 💻 Comandos da Estação Solo (USB)

Através de um terminal serial (ex: PuTTY, Tera Term, HTerm, Monitor Serial), os seguintes comandos são aceitos quando o firmware está compilado em Modo Bancada (sem a macro `EM_VOO`):

### Com a Caixa Preta Externa (`USE_W25Q` ativado)
| Comando | Ação | Descrição |
| :---: | :--- | :--- |
| **`V`** | Visualizar Todos | Imprime a tabela de telemetria completa armazenada na W25Q. |
| **`U`** | Último Log | Imprime apenas o bloco de dados mais recente gravado. |
| **`I`** | Idle / Parar | Interrompe a gravação atual e descarrega o buffer da RAM. |
| **`A`** | Apagar Logs | Executa um *Smart Erase*, apagando apenas os blocos de memória sujos (< 1s). |
| **`$`** | Apagar TUDO | **CUIDADO:** Formata o chip W25Q inteiro (Processo Crítico, leva ~40s). |
| **`S`** | Simular Ao Vivo | Executa um teste SITL e printa a simulação da física no terminal a 10Hz. |
| **`M`** | Mock de Memória | Gera um voo simulado e grava os dados fisicamente na W25Q a 100Hz. |

### Sem Caixa Preta (Flash STM32 Nativa)
| Comando | Ação | Descrição |
| :---: | :--- | :--- |
| **`L`** | Log Teste (UP) | Grava um dado fictício de subida na memória interna e liga o LED PC13. |
| **`D`** | Log Teste (DOWN) | Grava um dado fictício de descida na memória interna e apaga o LED PC13. |
| **`R`** | Read (Ler) | Imprime todos os logs recuperados e validados da flash interna. |
| **`C`** | Clear (Apagar) | Apaga o setor alocado da flash interna, resetando o banco de testes. |

---

## ⚙️ Instruções de Setup (STM32CubeIDE)

Para que a compilação funcione perfeitamente sem erros de *linkagem*, certifique-se de configurar a IDE:

1. **Source Location:** Em `Properties -> C/C++ General -> Paths and Symbols -> Source Location`, adicione a pasta `RAD` (ou `RAD/Src` e `RAD/Src/utils`).
2. **Include Paths:** Em `Properties -> C/C++ Build -> Settings -> MCU GCC Compiler -> Include paths`, adicione o diretório `RAD/Inc` e `RAD/Inc/utils`.
3. **Suporte a Float:** Em `C/C++ Build -> Settings -> MCU Settings`, marque a caixa *"Use float with printf from newlib-nano (-u _printf_float)"*.

**Nota de Segurança:** Nunca inclua arquivos gerados pelo CubeMX (como `main.h`) dentro dos cabeçalhos principais do RAD para evitar dependência circular. Utilize as áreas de `USER CODE` do arquivo `main.c` para chamar `setupVoo()` e `processarLogicaVoo()`.

---
*Desenvolvido por Júnior Bandeira.*
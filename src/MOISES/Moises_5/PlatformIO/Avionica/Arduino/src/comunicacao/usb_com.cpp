/*
  ******************************************************************************
  * @file    usb_com.cpp
  * @brief   Processamento de Comandos Interativos e Intertravamentos de Segurança
  ******************************************************************************
*/

#include <comunicacao/usb_com.h>
#include <config_voo.h>
#include <flash/w25q.h>

inline static bool emVooAtivo(void) {
    return (dadosVoo.estadoAtual >= ESTADO_EM_VOO) &&
           (dadosVoo.estadoAtual <= ESTADO_RECUPERACAO);
}

void usbComImprimirBanner(void) {
    if (!Serial || Serial.availableForWrite() < 64){ return; }

    printlnLCyan("\r\n==================================================================");
    printlnLYellow("       EQUIPE SIRIUS ROCKETS - COMPUTADOR DE VOO (RAD v3)         ");
    printlnLCyan("==================================================================");
#ifdef EM_VOO
    printlnLRed("  MODO DE OPERACAO : [EM_VOO - PARAMETROS REAIS DE LANCAMENTO]");
#else
    printlnLGreen("  MODO DE OPERACAO : [EM_SOLO - BANCADA / TESTES]");
#endif
    printlnLCyan("  Envie 'H' para listar os comandos disponiveis.");
    printlnLCyan("==================================================================\r\n");
}

void usbComImprimirAjuda(void) {
    if (!Serial){ return; }

    printlnLCyan("\r\n--- COMANDOS DISPONIVEIS (USB) ---");
    printlnWhite("  [V] Visualizar logs gravados (Tabela + Grafico ASCII)");
    printlnWhite("  [C] Exportar logs gravados em formato CSV");
    printlnWhite("  [U] Visualizar ultimo registro gravado");
    printlnWhite("  [I] Inspecionar status completo do sistema e sensores");
    printlnWhite("  [R] Recalibrar sensores e rearmar sistema (se Flash livre)");
    printlnWhite("  [A] Apagar logs (Smart Erase) e rearmar sistema");
    printlnWhite("  [F] Formatar chip Flash inteiro (8 MB) e rearmar");
    printlnWhite("  [$] Gerar voo simulado na Flash (SITL 20 Hz)");
    printlnWhite("  [S] Simular perfil de voo ao vivo via USB");
    printlnWhite("  [E] Forcar estado de erro critico (teste de falha)");
#ifndef EM_VOO
    printlnLYellow("  [1] Teste bancada: pulsar Pirotecnico J2 (Deploy - PB0)");
    printlnLYellow("  [2] Teste bancada: pulsar Pirotecnico J1 (Main   - PB10)");
#endif
    printlnWhite("  [H] Exibir este menu de ajuda");
    printlnLCyan("----------------------------------\r\n");
}

void usbComImprimirStatus(void) {
    if (!Serial){ return; }

    const u32 capBytes = w25qGetCapacity();
    const u32 usadosRegs = w25qGetRecordCount();
    const u32 maxRegs = w25qGetMaxRecords();
    const float usoPct = (maxRegs > 0) ? (100.0f * usadosRegs / maxRegs) : 0.0f;
    CaixaPretaInfo_t cp = w25qLerCaixaPreta();

    u8 st = (u8)dadosVoo.estadoAtual;
    if (st > 6){ st = 6; }

    printlnLCyan("\r\n================ DIAGNOSTICO DO SISTEMA ================");
    printflnWhite("  Estado Atual     : %s", PRINT_ESTADO[st]);
    printflnWhite("  Gravacao Flash   : %s | Bloqueio Log: %s",
                  flagGravacaoParada ? "PAUSADA/ENCERRADA" : "ATIVA",
                  sistemaBloqueadoPorLog ? "SIM (Envie 'A')" : "NAO");
    printflnWhite("  BMP280           : P_atual=%.1f Pa | P_ref=%.1f Pa | Alt=%.2f m | Vel=%.2f m/s",
                  dadosVoo.pressaoPa, bmpSensor.pressaoRefPa,
                  dadosVoo.altitudeM, dadosVoo.velocidadeMs);
    printflnWhite("  MPU6050          : %s | Gyro Calibrado: %s",
                  mpuSensor.online ? "ONLINE" : "OFFLINE",
                  mpuSensor.gyroCalibrated ? "SIM" : "NAO");
    printflnWhite("  Pirotecnicos     : J2(Deploy)=%s | J1(Main)=%s",
                  seguroVoo.deployAcionado ? "ACIONADO" : "ARMADO",
                  seguroVoo.mainAcionado ? "ACIONADO" : "ARMADO");
    printflnWhite("  Flash W25Q64     : %lu / %lu registros (%.2f%% de %lu KB)",
                  (unsigned long)usadosRegs, (unsigned long)maxRegs,
                  usoPct, (unsigned long)(capBytes / 1024));
    printflnWhite("  Caixa-Preta (S0) : Header=%s | StatusVoo=0x%08lX",
                  cp.headerValido ? "VALIDO (FLT1)" : "VAZIO/INVALIDO",
                  (unsigned long)cp.statusVoo);
    printlnLCyan("========================================================\r\n");
}

void usbComProcessar(void) {
    if (!Serial || Serial.available() <= 0){ return; }

    while (Serial.available() > 0) {
        const char cmd = (char)Serial.read();
        if (cmd == '\r' || cmd == '\n' || cmd == ' '){ continue; }

        switch (cmd) {
            case 'V':
            case 'v':
                w25qVisualizarLogsTabela();
                break;

            case 'C':
            case 'c':
                w25qVisualizarLogsCSV();
                break;

            case 'U':
            case 'u':
                w25qVisualizarUltimoLog();
                break;

            case 'I':
            case 'i':
                usbComImprimirStatus();
                break;

            case 'H':
            case 'h':
            case '?':
                usbComImprimirAjuda();
                break;

            case 'R':
            case 'r':
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Nao e possivel recalibrar durante o voo!");
                } else if (sistemaBloqueadoPorLog) {
                    printlnLYellow("[BLOQUEADO] Existe um voo gravado na Flash. Envie 'A' para apagar antes de rearmar.");
                } else {
                    recalibrarERearmarSistema();
                }
                break;

            case 'A':
            case 'a':
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Comando de apagamento bloqueado durante o voo!");
                } else if (w25qApagarLogs()) {
                    recalibrarERearmarSistema();
                }
                break;

            case 'F':
            case 'f':
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Formatacao bloqueada durante o voo!");
                } else if (w25qApagarTudo()) {
                    recalibrarERearmarSistema();
                }
                break;

            case '$':
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Simulacao bloqueada durante o voo!");
                } else {
                    w25qGerarVooSimulado();
                    sistemaBloqueadoPorLog = true;
                    flagGravacaoParada     = true;
                }
                break;

            case 'S':
            case 's':
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Simulacao ao vivo bloqueada durante o voo!");
                } else {
                    simularVooAoVivoUSB();
                }
                break;

            case 'E':
            case 'e':
                forcarErroSistema("COMANDO MANUAL DE ABORTAGEM VIA USB");
                break;

            #ifndef EM_VOO
                case '1':
                    if (!emVooAtivo()) {
                        printlnLYellow("[TESTE BANCADA] Pulsando Canal J2 (Deploy - PB0) por 500 ms...");
                        digitalWrite(PIN_PYRO_DEPLOY, HIGH);
                        delay(500);
                        digitalWrite(PIN_PYRO_DEPLOY, LOW);
                        printlnLGreen("[OK] Pulso J2 concluido.");
                    }
                    break;

                case '2':
                    if (!emVooAtivo()) {
                        printlnLYellow("[TESTE BANCADA] Pulsando Canal J1 (Main - PB10) por 500 ms...");
                        digitalWrite(PIN_PYRO_MAIN, HIGH);
                        delay(500);
                        digitalWrite(PIN_PYRO_MAIN, LOW);
                        printlnLGreen("[OK] Pulso J1 concluido.");
                    }
                    break;
            #endif

            default:
                printflnLYellow("[AVISO] Comando '%c' desconhecido. Envie 'H' para ajuda.", cmd);
                break;
        }
    }
}
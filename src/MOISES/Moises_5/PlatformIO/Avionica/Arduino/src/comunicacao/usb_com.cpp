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

static void aguardarEspacoUSB(void) {
    if (!Serial) { return; }
    const u32 t0 = millis();
    while (Serial.availableForWrite() < 64 && (millis() - t0) < 10UL) {
        feedWatchdog();
    }
}

static void notificarComandoValido(char cmd, const char *descricao) {
    aguardarEspacoUSB();
    printflnLCyan("\r\n[CMD] Comando '%c' digitado -> %s", cmd, descricao);
    aguardarEspacoUSB();
}

void usbComImprimirBanner(void) {
    if (!Serial || Serial.availableForWrite() < 64) { return; }

    printlnLCyan("\r\n==================================================================");
    aguardarEspacoUSB();
    printlnLYellow("       EQUIPE SIRIUS ROCKETS - COMPUTADOR DE VOO (RAD v3)         ");
    aguardarEspacoUSB();
    printlnLCyan("==================================================================");
    aguardarEspacoUSB();
#ifdef EM_VOO
    printlnLRed("  MODO DE OPERACAO : [EM_VOO - PARAMETROS REAIS DE LANCAMENTO]");
#else
    printlnLGreen("  MODO DE OPERACAO : [EM_SOLO - BANCADA / TESTES]");
#endif
    aguardarEspacoUSB();
    printlnLCyan("  Envie 'H' para listar os comandos disponiveis.");
    aguardarEspacoUSB();
    printlnLCyan("==================================================================\r\n");
}

void usbComImprimirAjuda(void) {
    if (!Serial) { return; }

    printlnLCyan("\r\n--- COMANDOS DISPONIVEIS (USB) ---");
    aguardarEspacoUSB();
    printlnWhite("  [V] Visualizar logs gravados (Tabela + Grafico ASCII)");
    aguardarEspacoUSB();
    printlnWhite("  [C] Exportar logs gravados em formato CSV");
    aguardarEspacoUSB();
    printlnWhite("  [U] Visualizar ultimo registro gravado");
    aguardarEspacoUSB();
    printlnWhite("  [I] Inspecionar status completo do sistema e sensores");
    aguardarEspacoUSB();
    printlnWhite("  [R] Recalibrar sensores e rearmar sistema (se Flash livre)");
    aguardarEspacoUSB();
    printlnWhite("  [A] Apagar logs (Smart Erase) e rearmar sistema");
    aguardarEspacoUSB();
    printlnWhite("  [F] Formatar chip Flash inteiro (8 MB) e rearmar");
    aguardarEspacoUSB();
    printlnWhite("  [$] Gerar voo simulado na Flash (SITL 20 Hz)");
    aguardarEspacoUSB();
    printlnWhite("  [S] Simular perfil de voo ao vivo via USB");
    aguardarEspacoUSB();
    printlnWhite("  [E] Forcar estado de erro critico (teste de falha)");
    aguardarEspacoUSB();
    printlnWhite("  [P] Encerrar voo manualmente (marcar pouso e salvar log)");
    aguardarEspacoUSB();
#ifndef EM_VOO
    printlnLYellow("  [1] Teste bancada: pulsar Pirotecnico J2 (Deploy - PA0)");
    aguardarEspacoUSB();
    printlnLYellow("  [2] Teste bancada: pulsar Pirotecnico J1 (Main   - PB0)");
    aguardarEspacoUSB();
#endif
    printlnWhite("  [H] Exibir este menu de ajuda");
    aguardarEspacoUSB();
    printlnLCyan("----------------------------------\r\n");
}

void usbComImprimirStatus(void) {
    if (!Serial) { return; }

    const u32 capBytes   = w25qGetCapacity();
    const u32 usadosRegs = w25qGetRecordCount();
    const u32 maxRegs    = w25qGetMaxRecords();
    const float usoPct   = (maxRegs > 0) ? (100.0f * usadosRegs / maxRegs) : 0.0f;
    CaixaPretaInfo_t cp  = w25qLerCaixaPreta();

    u8 st = (u8)dadosVoo.estadoAtual;
    if (st > 6) { st = 6; }

    printlnLCyan("\r\n================ DIAGNOSTICO DO SISTEMA ================");
    aguardarEspacoUSB();
    printflnWhite("  Estado Atual     : %s", PRINT_ESTADO[st]);
    aguardarEspacoUSB();
    printflnWhite("  Gravacao Flash   : %s | Bloqueio Log: %s",
                  flagGravacaoParada ? "PAUSADA/ENCERRADA" : "ATIVA",
                  sistemaBloqueadoPorLog ? "SIM (Envie 'A')" : "NAO");
    aguardarEspacoUSB();
    printflnWhite("  BMP280           : P_atual=%.1f Pa | P_ref=%.1f Pa | Alt=%.2f m | Vel=%.2f m/s",
                  dadosVoo.pressaoPa, bmpSensor.pressaoRefPa,
                  dadosVoo.altitudeM, dadosVoo.velocidadeMs);
    aguardarEspacoUSB();
    printflnWhite("  MPU6050          : %s | Gyro Calibrado: %s",
                  mpuSensor.online ? "ONLINE" : "OFFLINE",
                  mpuSensor.gyroCalibrated ? "SIM" : "NAO");
    aguardarEspacoUSB();
    printflnWhite("  Pirotecnicos     : J2(Deploy)=%s | J1(Main)=%s",
                  seguroVoo.deployAcionado ? "ACIONADO" : "ARMADO",
                  seguroVoo.mainAcionado ? "ACIONADO" : "ARMADO");
    aguardarEspacoUSB();
    printflnWhite("  Flash W25Q     : %lu / %lu registros (%.2f%% de %lu KB)",
                  (unsigned long)usadosRegs, (unsigned long)maxRegs,
                  usoPct, (unsigned long)(capBytes / 1024));
    aguardarEspacoUSB();
    printflnWhite("  Caixa-Preta (S0) : Header=%s | StatusVoo=0x%08lX",
                  cp.headerValido ? "VALIDO (FLT1)" : "VAZIO/INVALIDO",
                  (unsigned long)cp.statusVoo);
    aguardarEspacoUSB();
    printlnLCyan("========================================================\r\n");
}

void usbComProcessar(void) {
    if (!Serial || Serial.available() <= 0) { return; }

    while (Serial.available() > 0) {
        const char cmd = (char)Serial.read();
        if (cmd == '\r' || cmd == '\n' || cmd == ' ') { continue; }

        switch (cmd) {
            case 'V':
            case 'v':
                notificarComandoValido(cmd, "Visualizar logs em tabela");
                w25qVisualizarLogsTabela();
                break;

            case 'C':
            case 'c':
                notificarComandoValido(cmd, "Exportar logs em CSV");
                w25qVisualizarLogsCSV();
                break;

            case 'U':
            case 'u':
                notificarComandoValido(cmd, "Visualizar ultimo registro");
                w25qVisualizarUltimoLog();
                break;

            case 'I':
            case 'i':
                notificarComandoValido(cmd, "Inspecionar status do sistema");
                usbComImprimirStatus();
                break;

            case 'H':
            case 'h':
            case '?':
                notificarComandoValido(cmd, "Exibir menu de ajuda");
                usbComImprimirAjuda();
                break;

            case 'R':
            case 'r':
                notificarComandoValido(cmd, "Recalibrar sensores e rearmar");
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
                notificarComandoValido(cmd, "Apagar logs (Smart Erase) e rearmar");
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Comando de apagamento bloqueado durante o voo!");
                } else if (w25qGetCapacity() == 0) {
                    printlnRed("[ERRO] Memoria Flash W25Q offline.");
                } else if (w25qApagarLogs()) {
                    recalibrarERearmarSistema();
                }
                break;

            case 'F':
            case 'f':
                notificarComandoValido(cmd, "Formatar chip Flash inteiro e rearmar");
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Formatacao bloqueada durante o voo!");
                } else if (w25qGetCapacity() == 0) {
                    printlnRed("[ERRO] Memoria Flash W25Q offline.");
                } else if (w25qApagarTudo()) {
                    recalibrarERearmarSistema();
                }
                break;

            case '$':
                notificarComandoValido(cmd, "Gerar voo simulado na Flash");
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Simulacao bloqueada durante o voo!");
                } else if (w25qGetCapacity() == 0) {
                    printlnRed("[ERRO] Memoria Flash W25Q offline.");
                } else {
                    w25qGerarVooSimulado();
                    sistemaBloqueadoPorLog = true;
                    flagGravacaoParada     = true;
                }
                break;

            case 'S':
            case 's':
                notificarComandoValido(cmd, "Simular voo ao vivo via USB");
                if (emVooAtivo()) {
                    printlnRed("[BLOQUEADO] Simulacao ao vivo bloqueada durante o voo!");
                } else {
                    simularVooAoVivoUSB();
                }
                break;

            case 'E':
            case 'e':
                notificarComandoValido(cmd, "Forcar estado de erro critico");
                forcarErroSistema("COMANDO MANUAL DE ABORTAGEM VIA USB");
                break;

            case 'P':
            case 'p':
                notificarComandoValido(cmd, "Encerrar voo manualmente (pouso)");
                forcarPousoManual();
                break;
                
            #ifndef EM_VOO
                case '1':
                    notificarComandoValido(cmd, "Teste bancada Pirotecnico J2 (Deploy - PA0)");
                    if (!emVooAtivo()) {
                        printlnLYellow("[TESTE BANCADA] Pulsando Canal J2 (Deploy - PA0) por 500 ms...");
                        digitalWrite(PIN_PYRO_DEPLOY, HIGH);
                        delay(500);
                        digitalWrite(PIN_PYRO_DEPLOY, LOW);
                        printlnLGreen("[OK] Pulso J2 concluido.");
                    }
                    break;

                case '2':
                    notificarComandoValido(cmd, "Teste bancada Pirotecnico J1 (Main - PB0)");
                    if (!emVooAtivo()) {
                        printlnLYellow("[TESTE BANCADA] Pulsando Canal J1 (Main - PB0) por 500 ms...");
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
// ============================================================
// ALTÍMETRO / DATALOGGER DE VOO - Black Pill STM32F411CE
// BMP280 (SPI3) + W25Q64 (SPI1) + buzzer + LED
//
// Funções principais:
//  - Telemetria ao vivo na serial em todos os estados (inclusive gravando)
//  - Detecção de lançamento com confirmação por amostras consecutivas
//  - Buffer pré-trigger: grava os ~2 s anteriores ao lançamento
//  - Grava pressão, temperatura e altitude relativa com CRC por registro
//  - Retoma a gravação só se o watchdog reiniciar a placa em voo
//    (ligar a placa normalmente nunca retoma: o log fica protegido)
//  - Watchdog (IWDG)
//  - Grava até receber S ou a Flash encher
//
// Comandos pela serial (só 3):
//   V  mostra os dados do voo (CSV)
//   A  apaga o voo e arma para o próximo
//   S  encerra a gravação (envie depois do pouso)
// ============================================================

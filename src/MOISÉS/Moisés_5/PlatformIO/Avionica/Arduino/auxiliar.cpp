
// ============================================================
// FUNÇÕES AUXILIARES
// ============================================================

void feedWatchdog(void);

u8 crc8(const u8 *data, size_t length);

bool timeReached(u32 now, u32 target);

// ============================================================
// BUZZER NÃO BLOQUEANTE (sequência de passos liga/desliga)
// ============================================================

struct BeepStep {
    u16 onMs;
    u16 offMs;
};


void beeperClear(void);

bool beeperAdd(u16 onMs, u16 offMs);

void beeperStart(void);
void beeperUpdate(void);

void playBeeps(u8 count, u16 onMs, u16 offMs);
// Dois bipes curtos e um longo.
void playLaunchPattern(void);

// Bipe bloqueante: só para inicialização e erros críticos.
void beepBlocking(u16 durationMs, u8 repetitions = 1);
void haltWithError(FlashStringPtr message, u16 durationMs,
                   u8 repetitions, u16 pauseMs);
void updateLed(u32 now);

// ============================================================
// FUNÇÕES AUXILIARES
// ============================================================

void feedWatchdog(void) {
    if (watchdogStarted) {
        IWatchdog.reload();
    }
}

u8 crc8(const u8 *data, size_t length) {
    u8 crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (u8 bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<u8>((crc << 1) ^ 0x07)
                               : static_cast<u8>(crc << 1);
        }
    }
    return crc;
}



bool timeReached(u32 now, u32 target) {
    return static_cast<int32>(now - target) >= 0;
}

FlashStringPtr stateName(EstadoSistema_t state) {
    switch (state) {
        case STATE_IDLE:      return F("ARMADO  ");
        case STATE_RECORDING: return F("GRAVANDO");
        default:              return F("PARADO  ");
    }
}

// ============================================================
// BUZZER NÃO BLOQUEANTE (sequência de passos liga/desliga)
// ============================================================

static const u8 BEEPER_MAX_STEPS = 64;
BeepStep beeperSteps[BEEPER_MAX_STEPS];
u8  beeperCount = 0;
u8  beeperIndex = 0;
bool     beeperActive = false;
bool     beeperOn = false;
u32 beeperNextChange = 0;

void beeperClear(void) {
    beeperActive = false;
    beeperOn = false;
    beeperCount = 0;
    beeperIndex = 0;
    digitalWrite(BUZZER_PIN, LOW);
}

bool beeperAdd(u16 onMs, u16 offMs) {
    if (beeperCount >= BEEPER_MAX_STEPS) {
        return false;
    }
    beeperSteps[beeperCount].onMs = onMs;
    beeperSteps[beeperCount].offMs = offMs;
    ++beeperCount;
    return true;
}

void beeperStart(void) {
    if (beeperCount == 0) {
        return;
    }
    beeperIndex = 0;
    beeperActive = true;
    beeperOn = true;
    digitalWrite(BUZZER_PIN, HIGH);
    beeperNextChange = millis() + beeperSteps[0].onMs;
}

void beeperUpdate(void) {
    if (!beeperActive) {
        return;
    }

    const u32 now = millis();
    if (!timeReached(now, beeperNextChange)) {
        return;
    }

    if (beeperOn) {
        digitalWrite(BUZZER_PIN, LOW);
        beeperOn = false;
        beeperNextChange = now + beeperSteps[beeperIndex].offMs;
    } else {
        ++beeperIndex;
        if (beeperIndex >= beeperCount) {
            beeperActive = false;
            return;
        }
        digitalWrite(BUZZER_PIN, HIGH);
        beeperOn = true;
        beeperNextChange = now + beeperSteps[beeperIndex].onMs;
    }
}

void playBeeps(u8 count, u16 onMs, u16 offMs) {
    beeperClear();
    for (u8 i = 0; i < count; ++i) {
        beeperAdd(onMs, (i + 1 < count) ? offMs : 0);
    }
    beeperStart();
}

// Dois bipes curtos e um longo.
void playLaunchPattern(void) {
    beeperClear();
    beeperAdd(100, 120);
    beeperAdd(100, 120);
    beeperAdd(500, 0);
    beeperStart();
}

// Bipe bloqueante: só para inicialização e erros críticos.
// u8 repetitions = 1 por definicao
void beepBlocking(u16 durationMs, u8 repetitions) {
    beeperClear();
    for (u8 i = 0; i < repetitions; ++i) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i + 1 < repetitions) {
            delay(80);
        }
        feedWatchdog();
    }
}

void haltWithError(FlashStringPtr message, u16 durationMs,
                   u8 repetitions, u16 pauseMs) {
    while (true) {
        Serial.print(F("[ERRO CRITICO] "));
        Serial.println(message);
        beepBlocking(durationMs, repetitions);
        delay(pauseMs);
    }
}



// ============================================================
// LED DE ESTADO
// ============================================================

void updateLed(u32 now) {
    bool on = false;
    switch (currentState) {
        case STATE_IDLE:      on = (now % 1000) < 50;  break;  // piscada curta: armado
        case STATE_RECORDING: on = true;               break;  // aceso: gravando
        default:              on = false;              break;  // apagado: parado
    }
    digitalWrite(LED_PIN, on ? LOW : HIGH);
}
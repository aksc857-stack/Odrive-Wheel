// Phase 2c — Bridge FFB ⇄ ODrive v0.5.6 stock.
// Este é o UNICO arquivo que inclui odrive_main.h fora da árvore ODrive — evita
// colisão entre nomes "Axis" (ODrive) e "Axis" (OpenFFBoard FFB stack).

#include "odrive_bridge.h"
#include "odrive_main.h"

extern "C" void odrive_bridge_init(void) {
    // v0.5.6 stock: motor.setup() + axis.start_thread() já correm pelo
    // odrive_main() normal. TIM3 encoder quadrature está ativo. Nada a fazer.
}

// ===================== Option A — thread de leitura do encoder ===============
// Objetivo: tirar a leitura SPI do MT6835 do ISR prio-0 (sampling_cb) pra uma
// thread dedicada, eliminando a fome de USB/FFB. ETAPA 1: só o esqueleto (loop
// idle). Não mexe na leitura ainda — serve pra confirmar que adicionar a thread
// não quebra a enumeração USB, antes de migrar a leitura na etapa 2.
static void enc_spi_thread(void* arg) {
    (void)arg;
    Encoder& e = axes[0].encoder_;

    // Tick do FreeRTOS = 1 kHz → osDelay/osDelayUntil têm granularidade de 1 ms.
    // Pra ler acima de 1 kHz fazemos N leituras por tick, igualmente espaçadas
    // por busy-wait curto medido no DWT_CYCCNT (contador de ciclos, ~6 ns
    // @168 MHz). Durante cada espera cedemos a CPU (osThreadYield) pras threads
    // de mesma prioridade (USB tusb também é Normal) — não bloqueamos o
    // barramento (o SPI só é tocado nas ~9 µs de cada leitura). O ISR (8 kHz)
    // consome pos_abs_ em cache; a PLL interpola. NÃO usar busy-wait sem ceder/
    // dormir: o thread DEVE liberar a CPU pra não estrangular USB/FFB nem segurar
    // a SPI3 (compartilhada com o DRV8301).
    const uint32_t reads_per_tick = 8;                          // 8 leituras/ms = 8 kHz (= taxa do loop de controle)
    const uint32_t slot = (168000000UL / 1000UL) / reads_per_tick;  // 125 µs em ciclos

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t tick = osKernelSysTick();
    for (;;) {
        // Pausa pós Program-EEPROM: datasheet 7.6.6 proíbe QUALQUER operação
        // SPI por >= 6 s após o comando. sys.mteeprom! arma este tick.
        bool spi_paused =
            (int32_t)(osKernelSysTick() - e.mt6835_spi_pause_until_tick_) < 0;
        if (!spi_paused && e.mode_ == Encoder::MODE_SPI_ABS_MT6835) {
            uint32_t t0 = DWT->CYCCNT;                           // base do tick
            for (uint32_t i = 0; i < reads_per_tick; ++i) {
                if (i > 0) {                                     // espaça a leitura i no slot i*125µs
                    while ((uint32_t)(DWT->CYCCNT - t0) < i * slot) {
                        osThreadYield();                         // cede a USB/iguais durante a espera
                    }
                }
                e.abs_spi_start_transaction();                  // polled read + abs_spi_cb → pos_abs_
            }
        }
        osDelayUntil(&tick, 1);                                 // próximo tick → N leituras/ms
    }
}

extern "C" void odrive_bridge_start_enc_thread(void) {
    osThreadDef(encSpiThread, enc_spi_thread, osPriorityNormal, 0, 1024 / sizeof(StackType_t));
    osThreadCreate(osThread(encSpiThread), NULL);
}

extern "C" float odrive_bridge_get_pos_turns(void) {
    // shadow_count_ é int32 acumulado pelo axis thread (não wrappa em 16-bit
    // como o timer CNT direto). turns = shadow_count / cpr.
    int32_t cnt = axes[0].encoder_.shadow_count_;
    int32_t cpr = axes[0].encoder_.config_.cpr;
    if (cpr <= 0) cpr = 10000;
    return (float)cnt / (float)cpr;
}

extern "C" void odrive_bridge_set_input_torque(float nm) {
    // Efetivo apenas se axis está em CLOSED_LOOP_CONTROL + control_mode=TORQUE
    // + input_mode=PASSTHROUGH. Fora desses estados, escrita é ignorada pelo
    // Controller::update(). Defaults bakados no firmware atendem (control_mode=1,
    // input_mode=1) mas state precisa ser setado via ASCII ou pela UI HTML.
    axes[0].controller_.input_torque_ = nm;
}

// Bus current/voltage readouts — usados pelo peak tracker do ffb_task.
// Centralizados aqui pra não precisar incluir odrive_main.h em outros .cpp
// (colide com class Axis do OpenFFBoard).
extern "C" float odrive_bridge_get_ibus(void)        { return odrv.ibus_; }
extern "C" float odrive_bridge_get_vbus(void)        { return odrv.vbus_voltage_; }
extern "C" float odrive_bridge_get_motor_ibus(void)  { return axes[0].motor_.I_bus_; }
extern "C" int   odrive_bridge_motor_is_armed(void)  { return axes[0].motor_.is_armed_ ? 1 : 0; }

// Telemetria 1 kHz pro HID input report (axes não usados Y, Z, Dial).
// vel_estimate em turns/s (PLL filtrado), Iq_measured em Ampères,
// torque_output em Nm. Tudo em float, escala aplicada no caller pra int16.
// vel/Iq usam .present().value_or(0) porque são OutputPort<float> (podem
// não ter valor se ainda nem rodou um ciclo do controller — raro mas
// possível durante boot).
extern "C" float odrive_bridge_get_vel_estimate(void) {
    return axes[0].encoder_.vel_estimate_.any().value_or(0.0f);
}
extern "C" float odrive_bridge_get_iq_measured(void) {
    return axes[0].motor_.current_control_.Iq_measured_;
}
extern "C" float odrive_bridge_get_torque_output(void) {
    return axes[0].controller_.torque_output_.any().value_or(0.0f);
}
extern "C" float odrive_bridge_get_brake_resistor_current(void) {
    return odrv.brake_resistor_current_;
}

// Snapshot dos contadores de SPI do encoder + último raw recebido.
extern "C" void odrive_bridge_enc_get_raw(struct encraw_snap_t *snap) {
    auto& e = axes[0].encoder_;
    snap->ok_count    = e.abs_spi_ok_count_;
    snap->fail_parity = e.abs_spi_fail_parity_;
    snap->fail_ef     = e.abs_spi_fail_ef_;
    snap->fail_xfer   = e.abs_spi_fail_xfer_;
    snap->last_rx     = e.abs_spi_last_rx_;
    snap->pos_abs     = e.pos_abs_;
}

extern "C" void odrive_bridge_enc_get_magnet(struct magnet_snap_t *snap) {
    auto& e = axes[0].encoder_;
    uint16_t raw       = e.diaagc_raw_;
    snap->raw          = raw;
    snap->update_count = e.diaagc_update_count_;
    snap->agc          = raw & 0xFF;
    snap->lf           = (raw >> 8) & 1;
    snap->cof          = (raw >> 9) & 1;
    snap->magh         = (raw >> 10) & 1;
    snap->magl         = (raw >> 11) & 1;
}

// Anticogging calibration trigger — equivalente a chamar a função RPC
// `start_anticogging_calibration()` via Fibre. Necessário porque o campo
// `axis0.controller.config.anticogging.calib_anticogging` é marcado como
// readonly bool no YAML do ODrive (Property<const bool> no autogen), o
// que faz `w` via ASCII responder "not implemented". O cmdparser
// OpenFFBoard expõe isto via `axis.anticogcal!`.
//
// Pré-requisito: motor já calibrado, encoder pronto, control_mode setado
// pra POSITION_CONTROL, axis em CLOSED_LOOP_CONTROL. start_anticogging
// força mode=POSITION durante o procedimento (controller.cpp:86,94).
extern "C" int odrive_bridge_start_anticogcal(void) {
    // Reflete o axis.error_ check feito em start_anticogging_calibration:
    // se há erros, a função simplesmente não inicia (axis_->error_ ==
    // ERROR_NONE é a guarda).
    if (axes[0].error_ != Axis::ERROR_NONE) return 0;
    axes[0].controller_.start_anticogging_calibration();
    return 1;
}

// ===================== MT6835 register access (cmd_table) ====================
// Wrappers C pros command handlers (sys.mtread/mtwrite/mtzero/mteeprom/
// mtstatus) — cmd_table.cpp não pode incluir odrive_main.h (colisão de Axis).
// Todos retornam erro se o encoder não está em MODE_SPI_ABS_MT6835 (261).

extern "C" int odrive_bridge_mt6835_read_reg(int addr) {
    if (addr < 0 || addr > 0xFFF) return -1;
    uint8_t val = 0;
    if (!axes[0].encoder_.mt6835_read_reg((uint16_t)addr, &val)) return -1;
    return (int)val;
}

extern "C" int odrive_bridge_mt6835_write_reg(int addr, int val) {
    if (addr < 0 || addr > 0xFFF || val < 0 || val > 0xFF) return 0;
    return axes[0].encoder_.mt6835_write_reg((uint16_t)addr, (uint8_t)val) ? 1 : 0;
}

// Auto-set zero: grava a posição atual em ZERO_POS (volátil). Recusa com motor
// armado — mudar o zero do encoder debaixo do FOC deslocaria a referência de
// comutação (phase_offset ficaria inválido).
extern "C" int odrive_bridge_mt6835_set_zero(void) {
    if (axes[0].motor_.is_armed_) return 0;
    return axes[0].encoder_.mt6835_auto_set_zero() ? 1 : 0;
}

// Program EEPROM: persiste TODO o register map (ZERO_POS, HYST, BW, ABZ...).
// Recusa com motor armado. Pausa as leituras de ângulo da thread por 6.5 s
// (datasheet 7.6.6: nenhuma operação SPI por >= 6 s) — pos_abs_ congela nesse
// intervalo, por isso a guarda de desarme é obrigatória, não cosmética.
extern "C" int odrive_bridge_mt6835_program_eeprom(void) {
    auto& e = axes[0].encoder_;
    if (axes[0].motor_.is_armed_) return 0;
    // Arma a pausa ANTES do comando: nenhuma leitura pode se intrometer
    // entre o comando e o fim da janela de programação.
    e.mt6835_spi_pause_until_tick_ = osKernelSysTick() + 6500;
    // Deixa uma rajada de leituras em curso da thread terminar (a thread lê
    // em bursts de 8/ms; 3 ticks de margem cobrem o pior caso + preempção).
    osDelay(3);
    if (!e.mt6835_program_eeprom()) {
        e.mt6835_spi_pause_until_tick_ = osKernelSysTick();  // comando falhou — despausa
        return 0;
    }
    return 1;
}

// Snapshot de diagnóstico pro sys.mtstatus?.
extern "C" void odrive_bridge_mt6835_get_status(struct mt6835_snap_t *snap) {
    auto& e = axes[0].encoder_;
    uint8_t st         = e.mt6835_status_;
    snap->is_mt6835    = (e.mode_ == Encoder::MODE_SPI_ABS_MT6835) ? 1 : 0;
    snap->overspeed    = (st >> 0) & 1;
    snap->weak_field   = (st >> 1) & 1;
    snap->undervolt    = (st >> 2) & 1;
    // Status da User Auto-Calibration (datasheet 9.2): reg 0x113[7:6].
    // 0=sem cal, 1=rodando, 2=falhou, 3=sucesso. -1 = leitura SPI falhou.
    uint8_t cal_reg = 0;
    snap->cal_state = e.mt6835_read_reg(0x113, &cal_reg) ? ((cal_reg >> 6) & 0x3) : -1;
}

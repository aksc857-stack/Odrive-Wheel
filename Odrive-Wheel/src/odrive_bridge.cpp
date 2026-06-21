// Phase 2c — Bridge FFB ⇄ ODrive v0.5.6 stock.
// Este é o UNICO arquivo que inclui odrive_main.h fora da árvore ODrive — evita
// colisão entre nomes "Axis" (ODrive) e "Axis" (OpenFFBoard FFB stack).

#include "odrive_bridge.h"
#include "odrive_main.h"

extern "C" void odrive_bridge_init(void) {
    // v0.5.6 stock: motor.setup() + axis.start_thread() já correm pelo
    // odrive_main() normal. TIM3 encoder quadrature está ativo. Nada a fazer.
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

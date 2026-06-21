// Phase 5 — Bridge minimal pros internals da ODrive.
// Isola odrive_main.h (que define sua propria classe Axis) do resto do codigo
// FFB, que usa a classe Axis stub de OpenFFBoard. Evita colisao de nomes.

#ifndef ODRIVE_BRIDGE_H_
#define ODRIVE_BRIDGE_H_

#ifdef __cplusplus
extern "C" {
#endif

// Inicia o quadrature decoder do encoder (HAL_TIM_Encoder_Start em axes[0]).
// Necessario porque motor.setup() — que normalmente faz isso — foi deferido.
void odrive_bridge_init(void);

// Posicao absoluta do encoder em voltas. 0.0 se estimativa indisponivel.
float odrive_bridge_get_pos_turns(void);

// Escreve setpoint de torque (Nm) em axes[0].controller_.input_torque_.
// Sem motor armado a ODrive ignora — seguro chamar sempre.
void odrive_bridge_set_input_torque(float nm);

// Telemetry — leituras de bus/motor pra peak tracker do ffb_task. Não pode
// ler odrv.ibus_ direto do ffb_task porque odrive_main.h colide com Axis.h.
float odrive_bridge_get_ibus(void);
float odrive_bridge_get_vbus(void);
float odrive_bridge_get_motor_ibus(void);
int   odrive_bridge_motor_is_armed(void);

// Telemetria 1 kHz embedded no HID input report (Tier 1 task 1).
// vel_estimate em turns/s (PLL filtrado), Iq em A, torque_output em Nm.
float odrive_bridge_get_vel_estimate(void);
float odrive_bridge_get_iq_measured(void);
float odrive_bridge_get_torque_output(void);

// Corrente no resistor de freio (regen). Já existe vbus/ibus acima — esta
// completa a tríade pro overlay HID-only computar P_brake real a 1 kHz.
float odrive_bridge_get_brake_resistor_current(void);

// Snapshot dos contadores de SPI ABS do encoder pra debugar AS5047.
// Preenche o struct com: ok_count, fail_parity, fail_ef, fail_xfer, last_rx.
struct encraw_snap_t {
    unsigned int ok_count;
    unsigned int fail_parity;
    unsigned int fail_ef;
    unsigned int fail_xfer;
    unsigned int last_rx;       // raw 16-bit recebido na última transação
    unsigned int pos_abs;       // último pos_abs (que só atualiza quando validação passa)
};
void odrive_bridge_enc_get_raw(struct encraw_snap_t *snap);

// Snapshot do registro DIAAGC do AS5047 (atualizado a cada 256 transações ≈ 31 Hz).
struct magnet_snap_t {
    unsigned int raw;           // raw 14-bit (MAGL|MAGH|COF|LF|AGC[7:0])
    unsigned int update_count;  // # de leituras DIAAGC bem-sucedidas desde boot
    unsigned int agc;           // 0-255 (ideal ~128)
    unsigned int magl;          // 1 = magneto longe/fraco
    unsigned int magh;          // 1 = magneto perto/forte
    unsigned int cof;           // 1 = CORDIC overflow (ângulo inválido)
    unsigned int lf;            // 1 = loop finished (offset compensation OK)
};
void odrive_bridge_enc_get_magnet(struct magnet_snap_t *snap);

#ifdef __cplusplus
}
#endif

#endif

/*===========================================================================
 * kron_pi.h  --  KronEditor Global Process Image & HAL Driver Interface
 *
 * This is the neutral shared-memory contract between:
 *   - HAL drivers  (KronEthercatMaster, KronCANopen, etc.) — write inputs, read outputs
 *   - Axis engine  (KronMotion / kron_axis.c)               — read inputs, write outputs
 *   - PLC Logic    (KronLogic / user program)               — read/write DI/DO/AI/AO
 *
 * Dependency: stdint.h, stdbool.h only. No fieldbus headers.
 *
 * Layer diagram:
 *
 *   ┌─────────────────────────────────────────────────────┐
 *   │  KronLogic / User PLC Program                       │
 *   │  KronMotion MC_xxx function blocks                  │  ← consumes kron_pi.h
 *   └───────────────────────┬─────────────────────────────┘
 *                           │  AXIS_REF + KRON_MOTION_REQ
 *   ┌───────────────────────▼─────────────────────────────┐
 *   │  Axis engine  (kron_axis.c)                         │
 *   │  trajectory core, state diagram, CiA402             │  ← consumes kron_pi.h
 *   └───────────────────────┬─────────────────────────────┘
 *                           │  KRON_PROCESS_IMAGE
 *   ┌───────────────────────▼─────────────────────────────┐
 *   │  KRON_HAL_Driver  (implements read/write)           │
 *   │  KronEthercatMaster / KronCANopen / KronModbus      │  ← implements kron_pi.h
 *   └─────────────────────────────────────────────────────┘
 *
 * All three run in one task, in that order, once per cycle.
 *
 *===========================================================================*/

#ifndef KRON_PI_H
#define KRON_PI_H

#include <stdint.h>
#include <stdbool.h>

/* ── Encoder feedback type ───────────────────────────────────────────────── */
typedef enum {
    KRON_ENC_INCREMENTAL = 0,   /* Requires homing after every power-on    */
    KRON_ENC_ABSOLUTE_ST = 1,   /* Single-turn absolute (0..360 deg)       */
    KRON_ENC_ABSOLUTE_MT = 2,   /* Multi-turn absolute (full travel range) */
} KRON_ENCODER_TYPE;

/* ── Capacity limits ─────────────────────────────────────────────────────── */
#define KRON_MAX_SERVO_AXES   16
#define KRON_MAX_DI           256
#define KRON_MAX_DO           256
#define KRON_MAX_AI           64
#define KRON_MAX_AO           64

/* ── Atomic helpers ──────────────────────────────────────────────────────── */
/* Kept for drivers that exchange the image with a task of their own; the    */
/* motion layer itself is single task and needs none of them.                */
/* Requires GCC or Clang (standard for Linux PLC targets).                   */
/* Fall back to volatile cast for other toolchains (baremetal, MSVC).        */
#if defined(__GNUC__) || defined(__clang__)
#  define KRON_LOAD_ACQ_U16(ptr)         __atomic_load_n((ptr),  __ATOMIC_ACQUIRE)
#  define KRON_STORE_REL_U16(ptr, val)   __atomic_store_n((ptr), (val), __ATOMIC_RELEASE)
#  define KRON_FETCH_ADD_U16(ptr, val)   __atomic_fetch_add((ptr), (val), __ATOMIC_RELEASE)
#  define KRON_MEMORY_BARRIER()          __atomic_thread_fence(__ATOMIC_SEQ_CST)
#else
#  define KRON_LOAD_ACQ_U16(ptr)         (*(volatile uint16_t *)(ptr))
#  define KRON_STORE_REL_U16(ptr, val)   (*(volatile uint16_t *)(ptr) = (val))
#  define KRON_FETCH_ADD_U16(ptr, val)   (*(volatile uint16_t *)(ptr) += (val))
#  define KRON_MEMORY_BARRIER()          do {} while (0)
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * KRON_SERVO_SLOT
 *
 * One slot per physical servo drive in the process image.
 * Fieldbus driver writes the "inputs" section each cycle.
 * The axis engine writes the "outputs" section each cycle.
 * CiA402 (DS-402) fieldnames used for fieldbus-agnostic naming.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* ── Inputs: written by HAL driver (fieldbus → image) ── */
    int32_t  actual_pos_raw;       /* Encoder counts (position actual value)    */
    int32_t  actual_vel_raw;       /* Velocity actual value (counts/s)          */
    int16_t  actual_torque_raw;    /* Torque actual value (per-mille of rated)  */
    int16_t  following_error_raw;  /* Following error (counts)                  */
    uint16_t status_word;          /* CiA402 statusword (object 0x6041)         */
    uint8_t  mode_display;         /* Modes of operation display (0x6061)       */

    /* ── Outputs: written by the axis engine (image → fieldbus via HAL) ── */
    int32_t  target_pos_raw;       /* Target position (counts) (0x607A)         */
    int32_t  target_vel_raw;       /* Target velocity (counts/s) (0x60FF)       */
    /* Note: vel_raw_per_unit = counts_per_unit (both in counts/s per u/s)     */
    int16_t  target_torque_raw;    /* Target torque (per-mille) (0x6071)        */
    uint16_t control_word;         /* CiA402 controlword (0x6040)               */
    uint8_t  mode_of_operation;    /* Modes of operation (0x6060)               */

    /* ── Scaling (set once at init by fieldbus config) ── */
    float    counts_per_unit;      /* Encoder counts per user unit [u]          */
    float    vel_raw_per_unit;     /* Raw velocity unit per [u/s]               */

    /* ── Encoder configuration ── */
    KRON_ENCODER_TYPE encoder_type;        /* Feedback encoder type                     */
    uint32_t          enc_single_turn_bits;/* Bits per single turn (e.g. 13 = 8192 cts) */
    uint32_t          enc_multi_turn_bits; /* Multi-turn range bits (e.g. 12 = 4096 rev)*/

    /* ── Presence ── */
    bool     present;              /* TRUE if this slot is connected to a drive */
} KRON_SERVO_SLOT;

/* ═══════════════════════════════════════════════════════════════════════════
 * KRON_PROCESS_IMAGE
 *
 * Global shared memory. All layers read/write through this structure.
 * No fieldbus-specific types appear here.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    KRON_SERVO_SLOT  servo[KRON_MAX_SERVO_AXES];  /* Servo drive images       */
    bool             di[KRON_MAX_DI];              /* Digital inputs           */
    bool             dout[KRON_MAX_DO];            /* Digital outputs          */
    int32_t          ai_raw[KRON_MAX_AI];          /* Analog inputs  (raw ADC) */
    int32_t          ao_raw[KRON_MAX_AO];          /* Analog outputs (raw DAC) */
} KRON_PROCESS_IMAGE;

/* ═══════════════════════════════════════════════════════════════════════════
 * KRON_HAL_Driver
 *
 * A fieldbus driver registers itself by filling this struct and calling
 * KRON_HAL_Register().  KronMotion_ReadInputs() / KronMotion_WriteOutputs()
 * call through these pointers — the motion layer never knows which fieldbus
 * is underneath.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* Called at start of every cycle: fieldbus PDO → process image */
    void        (*read_inputs)(KRON_PROCESS_IMAGE *pi);

    /* Called at end of every cycle: process image → fieldbus PDO */
    void        (*write_outputs)(KRON_PROCESS_IMAGE *pi);

    /* Optional lifecycle hooks */
    int         (*init)(KRON_PROCESS_IMAGE *pi);   /* Returns 0 on success */
    void        (*close)(KRON_PROCESS_IMAGE *pi);

    const char  *name;    /* "EtherCAT", "CANopen", "Modbus", etc.  */
    bool         ready;   /* Set true by driver after successful init */
} KRON_HAL_Driver;

/* ── Global singletons (defined in kron_pi.c or the generated plc.c) ─────── */
extern KRON_PROCESS_IMAGE  Kron_PI;      /* The global process image          */
extern KRON_HAL_Driver    *Kron_HAL;     /* Active HAL driver (set at boot)   */

/* ── Inline wrappers called by the motion task ───────────────────────────── */
static inline void HAL_Read_Inputs(void) {
    if (Kron_HAL && Kron_HAL->read_inputs && Kron_HAL->ready)
        Kron_HAL->read_inputs(&Kron_PI);
}

static inline void HAL_Write_Outputs(void) {
    if (Kron_HAL && Kron_HAL->write_outputs && Kron_HAL->ready)
        Kron_HAL->write_outputs(&Kron_PI);
}

/* Register a driver and run its init.  Returns 0 on success. */
static inline int KRON_HAL_Register(KRON_HAL_Driver *drv) {
    if (!drv) return -1;
    Kron_HAL = drv;
    if (drv->init) {
        int r = drv->init(&Kron_PI);
        if (r != 0) return r;
    }
    drv->ready = true;
    return 0;
}

#endif /* KRON_PI_H */

// --------------------------------------------------------------------------------------
// Module     : SHM
// Author     : N. El-Fata
//            : R. Leslie
// --------------------------------------------------------------------------------------
// Pulse Innovation inc. www.pulseinnovation.com
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------

#ifndef SHM_H
#define SHM_H

/* Defining all shared memory structures and data here */

// Note: the "IN" states are transitions into the named state
typedef enum
{
    W5D_STATE_DISABLED,
    W5D_STATE_DISABLED_IN,      
    W5D_STATE_FORCE_CONTROL,
    W5D_STATE_FORCE_CONTROL_IN,
    W5D_STATE_TORQUE_CONTROL,
    W5D_STATE_TORQUE_CONTROL_IN
} w5d_state_t;

typedef enum
{
    W5D_UNHOMED,
    W5D_HOMED
} w5d_homed_t;

typedef enum
{
    W5D_NO_DOHOME,
    W5D_DOHOME
} w5d_dohome_t;

typedef struct
{
    w5d_state_t w5d_state;
    w5d_homed_t w5d_homed;
    w5d_dohome_t dohome;
    uint32_t home_offset[8];
    float force_setpoint[6];
    float torque_setpoint[6];
    float q[8];
    float qd[8];
    float pos[3];
    float orr[9];
    float posdot[3];
    float omega[3];
    uint32_t w5d_faultvec;
    int32_t w5d_freq;
} shm_struct_t;


int shm_init(void);
int shm_get(uint8_t *dst, uint8_t *src, uint32_t len);
int shm_put(uint8_t *dst, uint8_t *src, uint32_t len);

extern volatile shm_struct_t shm_struct;

#endif // SHM_H

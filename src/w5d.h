/// --------------------------------------------------------------------------------------
// Module     : W5D
// Description: High Level Functions for the W5D
// Author     : R. Leslie
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------
//
// References:
// 
//

#ifndef W5D_H
#define W5D_H

#include "eapi_packets.h"

// comment out to remove debug messages
//#define W5D_DEBUG

// comment out to remove safety safety monitor
#define W5D_SAFETY_MONITOR

// comment out to remove torque controller access
#define W5D_TAU_ACCESS

// comment out to remove encoder access
#define W5D_ENC_ACCESS

#define DEV_NAME_LEN	(32)

/* PI (might be in math.h) */
#define PI (3.14159265)

/* Link Lengths (mm) */
#define L1	(74.0)
#define L2	(46.5)
#define L3	(8.0)
#define L4	(160.0)

/* Home Offset (rads) */
#define phi	(PI/4)

/* K encoder 0 thru 5 (revs/count) */
#define K_ENC05	(2*PI/5000)

/* K encoder 6 (revs/count) */
#define K_ENC6	(2*PI/1440)

/* Encoder Direction Multiplier */
#if defined(W5D_LEFT)	    // Left Handed W5D
    #define E1_DIR  (1)
    #define E2_DIR  (1)
    #define E3_DIR  (1)
    #define E4_DIR  (-1)
    #define E5_DIR  (-1)
    #define E6_DIR  (1)
    #define E7_DIR  (-1)
    #define E8_DIR  (1)
#elif defined(W5D_RIGHT)     // Right Handed W5D
    #define E1_DIR  (1)
    #define E2_DIR  (1)
    #define E3_DIR  (-1)
    #define E4_DIR  (-1)
    #define E5_DIR  (-1)
    #define E6_DIR  (-1)
    #define E7_DIR  (-1)
    #define E8_DIR  (1)
#else
        #error define as left or right build (ex, -DW5D_LEFT)
#endif

#define RADTODEG (180/PI)

/* RT frequency saturation (carried out in message handler) Hz */
#define W5D_FREQ_MAX		(5000)
#define W5D_FREQ_DEFAULT	(1000)
#define W5D_FREQ_MIN		(100)
#define W5D_FREQ_VELSAMPLE	(50)

/* Safety Monitor defines */
// N.mm
#define W5D_TAU_MAX             (4000.0)
// rad/s
#define W5D_VEL_MAX             (120.00)

/* Torque Direction Multiplier */
#if defined(W5D_LEFT)	    // Left Handed W5D
    #define T1_DIR  (1)
    #define T2_DIR  (1)
    #define T3_DIR  (1)
    #define T4_DIR  (-1)
    #define T5_DIR  (-1)
    #define T6_DIR  (1)
#elif defined(W5D_RIGHT)     // Right Handed W5D
    #define T1_DIR  (1)
    #define T2_DIR  (1)
    #define T3_DIR  (-1)
    #define T4_DIR  (-1)
    #define T5_DIR  (-1)
    #define T6_DIR  (-1)
#else
        #error define as left or right build (ex, -DW5D_LEFT)
#endif

int w5d_init(void);
int w5d_uninit(void);
int w5d_real_time(void);
int w5d_msg_handler(udp_pkt_t *packet);

#endif // W5D_H

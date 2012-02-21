// --------------------------------------------------------------------------------------
// Module     : W5D
// Description: High Level Functions for the W5D
// Author     : R. Leslie
// --------------------------------------------------------------------------------------
// Entact Robotics Inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------
//
// References:
// 
//

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "w5d.h"
#include "spi.h"
#include "mem.h"
#include "shm.h"
#include "udp.h"

//TODO
// - implement fault detection and handling system
// - code optimizations for increased performance (possibly a linux configuraton issue also)
// - add acknowledgment packets for setMode, setFreq, and state changes

shm_struct_t rt_loc; // RT-thread local storage buffer 
shm_struct_t mh_loc; // MH-thread local storage buffer

int display_encoders = 0; // Outputs encoder position to a terminal window (if 1)
float seconds_counter = 0;

int w5d_init(void)
{
	w5d_state_t state = W5D_STATE_DISABLED_IN;
	w5d_homed_t homed = W5D_UNHOMED;
	w5d_dohome_t dohome = W5D_NO_DOHOME;
	int32_t freq = W5D_FREQ_DEFAULT;
	
	if (shm_put((void*)&shm_struct.w5d_state, (void*)&state, sizeof(shm_struct.w5d_state))==-1) return(-1);
	if (shm_put((void*)&shm_struct.w5d_homed, (void*)&homed, sizeof(shm_struct.w5d_homed))==-1) return(-1);
	if (shm_put((void*)&shm_struct.dohome, (void*)&dohome, sizeof(shm_struct.dohome))==-1) return(-1);	
	if (shm_put((void*)&shm_struct.w5d_freq, (void*)&freq, sizeof(shm_struct.w5d_freq))==-1) return(-1);	
	
	mem_led_init();
	return(0);
}

int w5d_uninit(void)
{
	mem_led_set(0);
	return(0);
}

int w5d_kinematics(void)
{
	static float kineqn[18]; // add more elements later on when velocity calcs are included
	static float tau[6];
	static float T[3];
	static float F[3];
	static float q[8];
	
	// read in current force setpoint from the shared memory
	if(shm_get((void*)rt_loc.force_setpoint, (void*)shm_struct.force_setpoint, sizeof(shm_struct.force_setpoint))==-1) return(-1);
	
	// current joint position will have been read in using the w5d_read_joints() function
	
	//debug (force setpoint printout)
	//printf("\rForce Command = [%3.1f, %3.1f, %3.1f, %3.1f, %3.1f]              ", f[0], f[1], f[2], f[3], f[4]);
	//end debug
	
	// this code solves the issue of ignorning the z torque command
	q[0] = rt_loc.q[0];
	q[1] = rt_loc.q[1];
	q[2] = rt_loc.q[2];
	q[3] = rt_loc.q[3];
	q[4] = rt_loc.q[4];
	q[5] = rt_loc.q[5];
	q[6] = rt_loc.q[6];
	q[7] = rt_loc.q[7];
	
	F[0] = rt_loc.force_setpoint[0];
	F[1] = rt_loc.force_setpoint[1];
	F[2] = rt_loc.force_setpoint[2];
	T[0] = rt_loc.force_setpoint[3];
	T[1] = rt_loc.force_setpoint[4];
	T[2] = rt_loc.force_setpoint[5];
	
	#if defined(W5D_LEFT)	    // Left Handed W5D
	
	float t1 = cos(q[5]);
	float t2 = q[3] + phi;
	float t3 = sin(t2);
	float t4 = t3 * L4;
	float t5 = q[4] + phi;
	float t6 = cos(t5);
	float t7 = t6 * L4;
	float t8 = -L3 + t4 + t7;
	float t9 = t1 * t8;
	float t11 = sin(q[5]);
	float t12 = t11 * L2;
	float t14 = cos(q[2]);
	float t15 = q[0] + phi;
	float t16 = sin(t15);
	float t17 = t16 * L4;
	float t18 = q[1] + phi;
	float t19 = cos(t18);
	float t20 = t19 * L4;
	float t21 = -L3 + t17 + t20;
	float t22 = t14 * t21;
	float t24 = sin(q[2]);
	float t25 = t24 * L2;
	float t28 = cos(t2);
	float t29 = t28 * L4;
	float t31 = sin(t5);
	float t32 = t31 * L4;
	float t34 = cos(t15);
	float t35 = t34 * L4;
	float t37 = sin(t18);
	float t38 = t37 * L4;
	float t41 = t11 * t8;
	float t43 = t1 * L2;
	float t45 = t24 * t21;
	float t47 = t14 * L2;
	float t50 = q[6] + phi;
	float t51 = cos(t50);
	float t52 = t22 - t25 - t9 - t12;
	float t53 = t52 * t52;
	float t54 = -t35 + t38 + t29 - t32;
	float t55 = t54 * t54;
	float t57 = (double) (2 * L1) - t45 - t47 + t41 - t43;
	float t58 = t57 * t57;
	float t59 = t53 + t55 + t58;
	float t60 = sqrt(t59);
	float t61 = 0.1e1 / t60;
	float t62 = t31 * t61;
	float t64 = t11 * t6;
	float t65 = t61 * t54;
	float t67 = t62 * t57 + t64 * t65;
	float t69 = sin(t50);
	float t70 = t1 * t6;
	float t73 = t70 * t65 - t52 * t62;
	float t75 = t61 * t57;
	float t76 = t61 * t52;
	float t79 = -t64 * t76 - t70 * t75;
	float t81 = t65 * t73 - t75 * t79;
	float t90 = t75 * t67 - t76 * t73;
	float t99 = t76 * t79 - t65 * t67;
	float t107 = 0.1e1 / ((double) L1 - L2) / 0.2e1;
	float t108 = 0.1e1 / t59;
	float t109 = t108 * t52;
	float t114 = t108 * t54;
	float t115 = t57 * T[2];
	float t118 = (T[1] - t109 * t54 * T[0] - t108 * t55 * T[1] - t114 * t115) * t61;
	float t127 = (T[2] - t109 * t57 * T[0] - t114 * t57 * T[1] - t108 * t58 * T[2]) * t61;
	float t129 = t118 * t57 - t127 * t54;
	float t131 = 0.5e0 * F[0];
	float t132 = t107 * t129 + t131;
	float t133 = L4 * t132;
	float t142 = (T[0] - t108 * t53 * T[0] - t109 * t54 * T[1] - t109 * t115) * t61;
	float t144 = t127 * t52 - t142 * t57;
	float t146 = 0.5e0 * F[1];
	float t147 = t107 * t144 + t146;
	float t152 = t142 * t54 - t118 * t52;
	float t154 = 0.5e0 * F[2];
	float t155 = t107 * t152 + t154;
	float t156 = L4 * t155;
	float t172 = -t107 * t129 + t131;
	float t173 = L4 * t172;
	float t176 = -t107 * t144 + t146;
	float t180 = -t107 * t152 + t154;
	float t181 = L4 * t180;
	kineqn[0] = 0.5e0 * t9 + 0.5e0 * t12 + 0.5e0 * t22 - 0.5e0 * t25;
	kineqn[1] = -0.5e0 * t29 + 0.5e0 * t32 - 0.5e0 * t35 + 0.5e0 * t38;
	kineqn[2] = -0.5e0 * t41 + 0.5e0 * t43 - 0.5e0 * t45 - 0.5e0 * t47;
	kineqn[3] = t51 * t67 + t69 * t81;
	kineqn[4] = -t69 * t67 + t51 * t81;
	kineqn[5] = t76;
	kineqn[6] = t51 * t79 + t69 * t90;
	kineqn[7] = -t69 * t79 + t51 * t90;
	kineqn[8] = t65;
	kineqn[9] = t51 * t73 + t69 * t99;
	kineqn[10] = -t69 * t73 + t51 * t99;
	kineqn[11] = t75;
	kineqn[12] = t14 * t34 * t133 + t17 * t147 - t24 * t34 * t156;
	kineqn[13] = -t14 * t37 * t133 + t20 * t147 + t24 * t37 * t156;
	kineqn[14] = (-t45 - t47) * t132 + (-t22 + t25) * t155;
	kineqn[15] = t1 * t28 * t173 + t4 * t176 - t11 * t28 * t181;
	kineqn[16] = -t1 * t31 * t173 + t7 * t176 + t11 * t31 * t181;
	kineqn[17] = (-t41 + t43) * t172 + (-t9 - t12) * t180;
	
	#elif defined(W5D_RIGHT)     // Right Handed W5D
	
	float t1 = cos(q[5]);
	float t2 = -q[3] + phi;
	float t3 = sin(t2);
	float t4 = t3 * L4;
	float t5 = -q[4] + phi;
	float t6 = cos(t5);
	float t7 = t6 * L4;
	float t8 = -L3 + t4 + t7;
	float t9 = t1 * t8;
	float t11 = sin(q[5]);
	float t12 = t11 * L2;
	float t14 = cos(q[2]);
	float t15 = -q[0] + phi;
	float t16 = sin(t15);
	float t17 = t16 * L4;
	float t18 = -q[1] + phi;
	float t19 = cos(t18);
	float t20 = t19 * L4;
	float t21 = -L3 + t17 + t20;
	float t22 = t14 * t21;
	float t24 = sin(q[2]);
	float t25 = t24 * L2;
	float t28 = cos(t2);
	float t29 = t28 * L4;
	float t31 = sin(t5);
	float t32 = t31 * L4;
	float t34 = cos(t15);
	float t35 = t34 * L4;
	float t37 = sin(t18);
	float t38 = t37 * L4;
	float t41 = t11 * t8;
	float t43 = t1 * L2;
	float t45 = t24 * t21;
	float t47 = t14 * L2;
	float t50 = q[6] + phi;
	float t51 = cos(t50);
	float t52 = q[4] + phi;
	float t53 = sin(t52);
	float t54 = t22 - t25 - t9 - t12;
	float t55 = t54 * t54;
	float t56 = t35 - t38 - t29 + t32;
	float t57 = t56 * t56;
	float t59 = (double) (2 * L1) - t45 - t47 + t41 - t43;
	float t60 = t59 * t59;
	float t61 = t55 + t57 + t60;
	float t62 = sqrt(t61);
	float t63 = 0.1e1 / t62;
	float t64 = t53 * t63;
	float t66 = cos(t52);
	float t67 = t11 * t66;
	float t68 = t63 * t56;
	float t70 = t64 * t59 + t67 * t68;
	float t72 = sin(t50);
	float t73 = t1 * t66;
	float t76 = t73 * t68 - t64 * t54;
	float t78 = t63 * t59;
	float t79 = t63 * t54;
	float t82 = -t67 * t79 - t73 * t78;
	float t84 = t68 * t76 - t78 * t82;
	float t93 = t78 * t70 - t79 * t76;
	float t102 = t79 * t82 - t68 * t70;
	float t110 = 0.1e1 / ((double) L1 - L2) / 0.2e1;
	float t111 = 0.1e1 / t61;
	float t112 = t111 * t54;
	float t117 = t111 * t56;
	float t118 = t59 * T[2];
	float t121 = (T[1] - t112 * t56 * T[0] - t111 * t57 * T[1] - t117 * t118) * t63;
	float t130 = (T[2] - t112 * t59 * T[0] - t117 * t59 * T[1] - t111 * t60 * T[2]) * t63;
	float t132 = t121 * t59 - t130 * t56;
	float t134 = 0.5e0 * F[0];
	float t135 = t110 * t132 + t134;
	float t136 = L4 * t135;
	float t145 = (T[0] - t111 * t55 * T[0] - t112 * t56 * T[1] - t112 * t118) * t63;
	float t147 = t130 * t54 - t145 * t59;
	float t149 = 0.5e0 * F[1];
	float t150 = t110 * t147 + t149;
	float t155 = t145 * t56 - t121 * t54;
	float t157 = 0.5e0 * F[2];
	float t158 = t110 * t155 + t157;
	float t159 = L4 * t158;
	float t175 = -t110 * t132 + t134;
	float t176 = L4 * t175;
	float t179 = -t110 * t147 + t149;
	float t183 = -t110 * t155 + t157;
	float t184 = L4 * t183;
	kineqn[0] = 0.5e0 * t9 + 0.5e0 * t12 + 0.5e0 * t22 - 0.5e0 * t25;
	kineqn[1] = 0.5e0 * t29 - 0.5e0 * t32 + 0.5e0 * t35 - 0.5e0 * t38;
	kineqn[2] = -0.5e0 * t41 + 0.5e0 * t43 - 0.5e0 * t45 - 0.5e0 * t47;
	kineqn[3] = t51 * t70 + t72 * t84;
	kineqn[4] = -t72 * t70 + t51 * t84;
	kineqn[5] = t79;
	kineqn[6] = t51 * t82 + t72 * t93;
	kineqn[7] = -t72 * t82 + t51 * t93;
	kineqn[8] = t68;
	kineqn[9] = t51 * t76 + t72 * t102;
	kineqn[10] = -t72 * t76 + t51 * t102;
	kineqn[11] = t78;
	kineqn[12] = -t14 * t34 * t136 + t17 * t150 + t24 * t34 * t159;
	kineqn[13] = t14 * t37 * t136 + t20 * t150 - t24 * t37 * t159;
	kineqn[14] = (-t45 - t47) * t135 + (-t22 + t25) * t158;
	kineqn[15] = -t1 * t28 * t176 + t4 * t179 + t11 * t28 * t184;
	kineqn[16] = t1 * t31 * t176 + t7 * t179 - t11 * t31 * t184;
	kineqn[17] = (-t41 + t43) * t175 + (-t9 - t12) * t183;
		
	#else
		#error define as left or right build (ex, -DW5D_LEFT)
	#endif

	// torque direction correction
	tau[0] = T1_DIR*kineqn[12];
	tau[1] = T2_DIR*kineqn[13];
	tau[2] = T3_DIR*kineqn[14];
	tau[3] = T4_DIR*kineqn[15];
	tau[4] = T5_DIR*kineqn[16];
	tau[5] = T6_DIR*kineqn[17];

	// torque saturation (part of the safety monitor)
	#ifdef W5D_SAFETY_MONITOR
	if (tau[0]>W5D_TAU_MAX) { tau[0] = W5D_TAU_MAX; } else { if (tau[0]<-W5D_TAU_MAX) tau[0] = -W5D_TAU_MAX;}
	if (tau[1]>W5D_TAU_MAX) { tau[1] = W5D_TAU_MAX; } else { if (tau[1]<-W5D_TAU_MAX) tau[1] = -W5D_TAU_MAX;}
	if (tau[2]>W5D_TAU_MAX) { tau[2] = W5D_TAU_MAX; } else { if (tau[2]<-W5D_TAU_MAX) tau[2] = -W5D_TAU_MAX;}
	if (tau[3]>W5D_TAU_MAX) { tau[3] = W5D_TAU_MAX; } else { if (tau[3]<-W5D_TAU_MAX) tau[3] = -W5D_TAU_MAX;}
	if (tau[4]>W5D_TAU_MAX) { tau[4] = W5D_TAU_MAX; } else { if (tau[4]<-W5D_TAU_MAX) tau[4] = -W5D_TAU_MAX;}
	if (tau[5]>W5D_TAU_MAX) { tau[5] = W5D_TAU_MAX; } else { if (tau[5]<-W5D_TAU_MAX) tau[5] = -W5D_TAU_MAX;}
	#endif

	//debug (force setpoint printout)
	//printf("\rTorque Command = [%3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f]                ", tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);
	//end debug

	#ifdef W5D_TAU_ACCESS	
	if (spi_torque_command(tau)==-1) {printf("w5d.c: w5d_kinematics(): torque set failed \n"); return(-1);}
	#endif
	
	// DEBUG
	
	rt_loc.orr[0] = kineqn[3]; rt_loc.orr[1] = kineqn[4]; rt_loc.orr[2] = kineqn[5];
	rt_loc.orr[3] = kineqn[6]; rt_loc.orr[4] = kineqn[7]; rt_loc.orr[5] = kineqn[8];
	rt_loc.orr[6] = kineqn[9]; rt_loc.orr[7] = kineqn[10]; rt_loc.orr[8] = kineqn[11];
	
	rt_loc.pos[0] = kineqn[0]; rt_loc.pos[1] = kineqn[1]; rt_loc.pos[2] = kineqn[2];
	
	//rt_loc.orr[0] = 1; rt_loc.orr[1] = 0; rt_loc.orr[2] = 0;
	//rt_loc.orr[3] = 0; rt_loc.orr[4] = 1; rt_loc.orr[5] = 0;
	//rt_loc.orr[6] = 0; rt_loc.orr[7] = 0; rt_loc.orr[8] = 1;
	
	//rt_loc.pos[0] = 210; rt_loc.pos[1] = 200; rt_loc.pos[2] = 400;
	
	// writing the orientation matrix to shared memory
	if(shm_put((void*)shm_struct.orr, (void*)rt_loc.orr, sizeof(shm_struct.orr))==-1) return(-1);
	
	// writing the position to shared memory
	if(shm_put((void*)shm_struct.pos, (void*)rt_loc.pos, sizeof(shm_struct.pos))==-1) return(-1);
	
	// END DEBUG
	
	return(0);
}
int w5d_read_joints(void)
{
	static int32_t e[8];
	static float qprev[8];
	static int32_t loop_ctr = 1;
	static int32_t led_ctr = 1;
	static int32_t led_state = 1;
	static int32_t n_loops;
	static float tSamp;
	
	// read in the current encoder values
	if(mem_encoder_read_all(e)==-1) { printf("w5d.c: w5d_real_time(): encoder read failed \n"); return(-1); }
	
	// converting encoder counts to radian joint positions
	rt_loc.q[0] = E1_DIR*e[0]*K_ENC05; 
	rt_loc.q[1] = E2_DIR*e[1]*K_ENC05;
	rt_loc.q[2] = E3_DIR*e[2]*K_ENC05;
	rt_loc.q[3] = E4_DIR*e[3]*K_ENC05;
	rt_loc.q[4] = E5_DIR*e[4]*K_ENC05;
	rt_loc.q[5] = E6_DIR*e[5]*K_ENC05;
	rt_loc.q[6] = E7_DIR*e[6]*K_ENC6;
	rt_loc.q[7] = E8_DIR*e[7];

	// read in the loop frequency
	if(shm_get((void*)&rt_loc.w5d_freq, (void*)&shm_struct.w5d_freq,  sizeof(shm_struct.w5d_freq))==-1) return(-1);
	
	// update velocity value (if on the proper loop_ctr value)
	n_loops = (int32_t) floor(rt_loc.w5d_freq/W5D_FREQ_VELSAMPLE);
	if (loop_ctr >= n_loops)
	{
		tSamp = ((float)loop_ctr)/((float)rt_loc.w5d_freq);
		rt_loc.qd[0] = (rt_loc.q[0] - qprev[0])/tSamp;
		rt_loc.qd[1] = (rt_loc.q[1] - qprev[1])/tSamp;
		rt_loc.qd[2] = (rt_loc.q[2] - qprev[2])/tSamp;
		rt_loc.qd[3] = (rt_loc.q[3] - qprev[3])/tSamp;
		rt_loc.qd[4] = (rt_loc.q[4] - qprev[4])/tSamp;
		rt_loc.qd[5] = (rt_loc.q[5] - qprev[5])/tSamp;
		rt_loc.qd[6] = (rt_loc.q[6] - qprev[6])/tSamp;
		rt_loc.qd[7] = (rt_loc.q[7] - qprev[7])/tSamp;
		qprev[0] = rt_loc.q[0];
		qprev[1] = rt_loc.q[1];
		qprev[2] = rt_loc.q[2];
		qprev[3] = rt_loc.q[3];
		qprev[4] = rt_loc.q[4];
		qprev[5] = rt_loc.q[5];
		qprev[6] = rt_loc.q[6];
		qprev[7] = rt_loc.q[7];
		
		// over-velocity fault (ignoring handle joint and spare encoder channel) (part of the safety monitor)
		#ifdef W5D_SAFETY_MONITOR
		int do_disable = 0;
		if (rt_loc.qd[0]>W5D_VEL_MAX) { do_disable = 1; } else { if (rt_loc.qd[0]<-W5D_VEL_MAX) do_disable = 1;}
		if (rt_loc.qd[1]>W5D_VEL_MAX) { do_disable = 1; } else { if (rt_loc.qd[1]<-W5D_VEL_MAX) do_disable = 1;}
		if (rt_loc.qd[2]>W5D_VEL_MAX) { do_disable = 1; } else { if (rt_loc.qd[2]<-W5D_VEL_MAX) do_disable = 1;}
		if (rt_loc.qd[3]>W5D_VEL_MAX) { do_disable = 1; } else { if (rt_loc.qd[3]<-W5D_VEL_MAX) do_disable = 1;}
		if (rt_loc.qd[4]>W5D_VEL_MAX) { do_disable = 1; } else { if (rt_loc.qd[4]<-W5D_VEL_MAX) do_disable = 1;}
		if (rt_loc.qd[5]>W5D_VEL_MAX) { do_disable = 1; } else { if (rt_loc.qd[5]<-W5D_VEL_MAX) do_disable = 1;}
		if (do_disable)
		{
			rt_loc.w5d_state = W5D_STATE_DISABLED_IN;
			if (shm_put((void*)&shm_struct.w5d_state, (void*)&rt_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);	
		}
		#endif
		
		// display encoder position to the terminal (if desired)
		if (display_encoders) printf("\r q = %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f                    ", rt_loc.q[0]*RADTODEG, rt_loc.q[1]*RADTODEG, rt_loc.q[2]*RADTODEG, rt_loc.q[3]*RADTODEG, rt_loc.q[4]*RADTODEG, rt_loc.q[5]*RADTODEG, rt_loc.q[6]*RADTODEG, rt_loc.q[7]*RADTODEG);
		
		//printf("\r qd = %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f                    ", qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qd[6], qd[7]);
		//printf("\r q = %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f                    ", rt_loc.q[0], rt_loc.q[1], rt_loc.q[2], rt_loc.q[3], rt_loc.q[4], rt_loc.q[5], rt_loc.q[6], rt_loc.q[7]);
		//printf("\r q = %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f                    ", qprev[0], qprev[1], qprev[2], qprev[3], qprev[4], qprev[5], qprev[6], qprev[7]);
		//printf("\r tSamp %1.5f, loop_ctr %d, loop_freq %d    ", tSamp, loop_ctr, loop_freq);
		loop_ctr = 1;
		
		// handling control of the blinking panel mount LED here
		// piggybacking off of the same calculations used for encoder velocity
		if ((rt_loc.w5d_state==W5D_STATE_FORCE_CONTROL)||(rt_loc.w5d_state==W5D_STATE_TORQUE_CONTROL))
		{
			mem_led_set(1);
		}
		else
		{
			led_ctr++;
			if (led_ctr >= (W5D_FREQ_VELSAMPLE/2))
			{
				led_state ^= 1; // toggle LED
				mem_led_set(led_state);
				led_ctr = 1;
			}
		}
	}
	else
	{
		loop_ctr++;
	}
	
	
	// writing the joint position values to the shared memory
	if(shm_put((void*)shm_struct.q, (void*)rt_loc.q, sizeof(shm_struct.q))==-1) return(-1);

	// writing the joint velocity values to the shared memory
	if(shm_put((void*)shm_struct.qd, (void*)rt_loc.qd, sizeof(shm_struct.qd))==-1) return(-1);
	
	// extra code to keep a seconds counter (for printf and whatnot)
	seconds_counter+=(1/((float)rt_loc.w5d_freq));
	
	return(0);
}

int w5d_real_time(void)
{
	int rc = 0;
	
	#ifdef W5D_ENC_ACCESS
	static int32_t e[8];
	#endif
		
	#ifdef W5D_TAU_ACCESS
	static float tau[6];
	#endif
		
	#ifdef W5D_ENC_ACCESS
	// update the current joint position and velocity values
	w5d_read_joints();
	#endif
	
	// read the devices current state in from shared memory
	if(shm_get((void*)&rt_loc.w5d_state, (void*)&shm_struct.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
	
	switch(rt_loc.w5d_state)
	{
		case W5D_STATE_FORCE_CONTROL:
			if(w5d_kinematics()==-1)
			{
				printf("w5d.c: w5d_real_time(): kinematics call failed \n"); 
				return(rc);
			}
			break;
		
		case W5D_STATE_TORQUE_CONTROL:
			// read in current force setpoint from the shared memory
			#ifdef W5D_TAU_ACCESS
			if (shm_get((void*)tau, (void*)shm_struct.torque_setpoint, sizeof(shm_struct.torque_setpoint))==-1) return(-1);
			
			// torque direction correction
			tau[0] = T1_DIR*tau[0];
			tau[1] = T2_DIR*tau[1];
			tau[2] = T3_DIR*tau[2];
			tau[3] = T4_DIR*tau[3];
			tau[4] = T5_DIR*tau[4];
			tau[5] = T6_DIR*tau[5];
			
			if (spi_torque_command(tau)==-1) {printf("w5d.c: torque set failed \n"); return(-1);}
			#endif
			//debug (torque setpoint printout)
			//printf("\rTorque Command = [%3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f]                ", tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);
			//end debug
			break;
			
		case W5D_STATE_DISABLED:
			if(shm_get((void*)&rt_loc.dohome, (void*)&shm_struct.dohome, sizeof(shm_struct.dohome))==-1) return(-1);
			if (rt_loc.dohome==W5D_DOHOME)
			{
				#ifdef W5D_ENC_ACCESS
				// clear/read current encoder count values
				if(mem_encoder_clear_all(e)==-1) { printf("w5d.c: w5d_real_time(): encoder clear failed \n"); return(-1); }
				// write home offset to shmem
				if (shm_put((void*)shm_struct.home_offset, (void*)e, sizeof(shm_struct.home_offset))==-1) return(-1);	
				#endif
				// update the devices homed status
				rt_loc.dohome = W5D_NO_DOHOME;
				rt_loc.w5d_homed = W5D_HOMED;
				if (shm_put((void*)&shm_struct.dohome, (void*)&rt_loc.dohome, sizeof(shm_struct.dohome))==-1) return(-1);
				if (shm_put((void*)&shm_struct.w5d_homed, (void*)&rt_loc.w5d_homed, sizeof(shm_struct.w5d_homed))==-1) return(-1);
				
				#ifdef W5D_DEBUG
				printf("\nw5d.c: w5d_real_time(): device homed \n");
				#endif
			}
			break;
			
		case W5D_STATE_DISABLED_IN:
			#ifdef W5D_TAU_ACCESS	
			tau[0] = 0; tau[1] = 0; tau[2] = 0; tau[3] = 0; tau[4] = 0; tau[5] = 0;
			if (spi_torque_command(tau)==-1) {printf("w5d.c: w5d_real_time(): torque zeroing failed \n"); return(-1);}
			mem_slave_disable();
			#endif
			
			rt_loc.w5d_state = W5D_STATE_DISABLED;
			if (shm_put((void*)&shm_struct.w5d_state, (void*)&rt_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			
			#ifdef W5D_DEBUG
			printf("\nw5d.c: w5d_real_time(): entering disabled state \n");
			#endif
			break;
			
		case W5D_STATE_FORCE_CONTROL_IN:
			rt_loc.w5d_state = W5D_STATE_FORCE_CONTROL;
			if (shm_put((void*)&shm_struct.w5d_state, (void*)&rt_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			
			memset(rt_loc.force_setpoint, 0, sizeof(rt_loc.force_setpoint));
			if (shm_put((void*)shm_struct.force_setpoint, (void*)rt_loc.force_setpoint, sizeof(shm_struct.force_setpoint))==-1) return(-1);
			
			#ifdef W5D_TAU_ACCESS
			mem_slave_enable();
			usleep(10000);
			tau[0] = 0; tau[1] = 0; tau[2] = 0; tau[3] = 0; tau[4] = 0; tau[5] = 0;
			if (spi_torque_command(tau)==-1) {printf("w5d.c: w5d_real_time(): torque zeroing failed \n"); return(-1);}
			#endif
			
			#ifdef W5D_DEBUG
			printf("\nw5d.c: w5d_real_time(): entering force control state \n");
			#endif
			break;
		
		case W5D_STATE_TORQUE_CONTROL_IN:
			rt_loc.w5d_state = W5D_STATE_TORQUE_CONTROL;
			if (shm_put((void*)&shm_struct.w5d_state, (void*)&rt_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			
			memset(rt_loc.torque_setpoint, 0, sizeof(rt_loc.torque_setpoint));
			if (shm_put((void*)shm_struct.torque_setpoint, (void*)rt_loc.torque_setpoint, sizeof(shm_struct.torque_setpoint))==-1) return(-1);
			
			#ifdef W5D_TAU_ACCESS
			mem_slave_enable();
			usleep(10000);
			tau[0] = 0; tau[1] = 0; tau[2] = 0; tau[3] = 0; tau[4] = 0; tau[5] = 0;
			if (spi_torque_command(tau)==-1) {printf("w5d.c: w5d_real_time(): torque zeroing failed \n"); return(-1);}
			#endif
			
			#ifdef W5D_DEBUG
			printf("\nw5d.c: w5d_real_time(): entering torque control state \n");
			#endif
			break;
			
		default:
			printf("w5d.c: w5d_real_time(): device set to invalid state (disabling) \n");
			rt_loc.w5d_state = W5D_STATE_DISABLED_IN;
			if (shm_put((void*)&shm_struct.w5d_state, (void*)&rt_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			return(-1);
			break;
	}

	/*
		Hardware Fault Detection Code Here 
		- implement in a future revision
	*/

	return(rc);
}

int w5d_send_nack(uint8_t nack_code)
{
	int rc = 0;
	int size;
	udp_pkt_t packet;
	
	packet.cmd = EAPI_OUTPKT_NACK;
	packet.len = 1;
	packet.payload[0] = nack_code;
	size = sizeof(packet) - sizeof(packet.payload) + packet.len;
	
	rc = udp_send(&packet, size);
	if (rc==-1) { printf("w5d.c: w5d_send_nack(): udp send (nack packet) failed \n"); return(rc); }
	
	return (rc);
}

int w5d_send_ack(uint8_t ack_code)
{
	int rc = 0;
	int size;
	udp_pkt_t packet;
	
	packet.cmd = EAPI_OUTPKT_ACK;
	packet.len = 1;
	packet.payload[0] = ack_code;
	size = sizeof(packet) - sizeof(packet.payload) + packet.len;
	
	rc = udp_send(&packet, size);
	if (rc==-1) { printf("w5d.c: w5d_send_ack(): udp send (ack packet) failed \n"); return(rc); }
	
	return (rc);
}

int w5d_read_devicefiles(int *serial_no, char *device_type, char *device_name)
{
	FILE *serial_file;
	FILE *device_file;
	FILE *name_file;
	
	// product name read
    device_file = fopen("product.txt","rb");
    if (device_file == NULL)
    {
        printf("ERR: failed to open product.txt \n");
		return (-1);
    }
    else
    {
        if(fgets(device_type, DEV_NAME_LEN, device_file) != NULL)
		{
			fclose(device_file);
		}
		else
		{
			printf("ERR: name.txt improperly formatted \n");
			return (-1);
		}
    }
	
	// serial number read
    serial_file = fopen("serial.txt","rb");
    if (serial_file == NULL)
    {
        printf("ERR: failed to open serial.txt \n");
		return (-1);
    }
    else
    {
        if(fscanf(serial_file, "%d", serial_no)==1)
		{
			fclose(serial_file);
		}
		else
		{
			printf("ERR: serial.txt improperly formatted \n");
			return (-1);
		}
    }
    
    // name read
    name_file = fopen("name.txt","rb");
    if (name_file == NULL)
    {
        printf("ERR: failed to open name.txt \n");
		return (-1);
    }
    else
    {
        if(fgets(device_name, DEV_NAME_LEN, name_file) != NULL)
		{
			fclose(name_file);
		}
		else
		{
			printf("ERR: name.txt improperly formatted \n");
			return (-1);
		}
    }
    
	return (0);
}

int w5d_msg_handler(udp_pkt_t *packet)
{
	int rc = 0;

	static char device_type[DEV_NAME_LEN];
	static char device_name[DEV_NAME_LEN];
	static int serial_no;
	
	static udp_pkt_t send_packet;
	static udp_status_pkt_t statuspkt;
	static udp_outdatafc_pkt_t fcdatapkt;
	static udp_outdatatc_pkt_t tcdatapkt;
	
	static int size;
	
	// Determine the devices state, fault and homed status
	if (shm_get((void*)&mh_loc.w5d_state, (void*)&shm_struct.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
	if (shm_get((void*)&mh_loc.w5d_homed, (void*)&shm_struct.w5d_homed, sizeof(shm_struct.w5d_homed))==-1) return(-1);
	if (shm_get((void*)&mh_loc.w5d_faultvec, (void*)&shm_struct.w5d_faultvec, sizeof(shm_struct.w5d_faultvec))==-1) return(-1);
	
	
	// Switch through all possible commands
	switch((*packet).cmd)
	{
		// Force Control Data Packet (this is by far the most commonly received packet)
		case EAPI_INPKT_DATA_FC: 
			if ((*packet).len != 24) { w5d_send_nack(NACK_LENGTH_ERROR); return(0);}
			
			// verify we're in FC state
			if (mh_loc.w5d_state != W5D_STATE_FORCE_CONTROL) { w5d_send_nack(NACK_STATE_CHANGE_REQ); return(0); }
			
			// Copy in the force setpoint from received packet.
			memcpy((void*)mh_loc.force_setpoint, (void*) (*packet).payload, sizeof(mh_loc.force_setpoint)); 
			if (shm_put((void*)shm_struct.force_setpoint, (void*)mh_loc.force_setpoint, sizeof(shm_struct.force_setpoint))==-1) return(-1);
			
			// Copy out the task position for a return packet.
			if(shm_get((void*)mh_loc.pos, (void*)shm_struct.pos, sizeof(shm_struct.pos))==-1) return(-1);
			
			// Copy out the task orientation for a return packet.
			if(shm_get((void*)mh_loc.orr, (void*)shm_struct.orr, sizeof(shm_struct.orr))==-1) return(-1);
			
			// Copy out the task velocity for a return packet.
			if(shm_get((void*)mh_loc.posdot, (void*)shm_struct.posdot, sizeof(shm_struct.posdot))==-1) return(-1);
			
			// Copy out the angular velocity for a return packet.
			if(shm_get((void*)mh_loc.omega, (void*)shm_struct.omega, sizeof(shm_struct.omega))==-1) return(-1);
			
			fcdatapkt.cmd = EAPI_OUTPKT_DATA;
			fcdatapkt.len = sizeof(mh_loc.pos) + sizeof(mh_loc.orr) + sizeof(mh_loc.posdot) + sizeof(mh_loc.omega);
			memcpy((void*)&fcdatapkt.pos, (void*)mh_loc.pos, sizeof(mh_loc.pos));
			memcpy((void*)&fcdatapkt.orr, (void*)mh_loc.orr, sizeof(mh_loc.orr));
			memcpy((void*)&fcdatapkt.posdot, (void*)mh_loc.posdot, sizeof(mh_loc.posdot));
			memcpy((void*)&fcdatapkt.omega, (void*)mh_loc.omega, sizeof(mh_loc.omega));
			rc = udp_send(&fcdatapkt, sizeof(fcdatapkt));
			if (rc==-1) { printf("w5d.c: w5d_msg_handler(): udp send (force control data packet) failed \n"); return(rc); }
			break;
		
		// Torque Control Data Packet (commonly received packet)
		case EAPI_INPKT_DATA_TC: 
			if ((*packet).len != 24) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			// verify we're in TC state
			if (mh_loc.w5d_state != W5D_STATE_TORQUE_CONTROL) { w5d_send_nack(NACK_STATE_CHANGE_REQ); return(0); }
			
			// Copy in the force setpoint from received packet.
			memcpy((void*)mh_loc.torque_setpoint, (void*) (*packet).payload, sizeof(mh_loc.torque_setpoint)); 
			if (shm_put((void*)shm_struct.torque_setpoint, (void*)mh_loc.torque_setpoint, sizeof(shm_struct.torque_setpoint))==-1) return(-1);
			
			// Copy out the joint angles into a return packet.
			if(shm_get((void*)mh_loc.q, (void*)shm_struct.q, sizeof(shm_struct.q))==-1) return(-1);
			
			// Copy out the joint velocities into return packet.
			if(shm_get((void*)mh_loc.qd, (void*)shm_struct.qd, sizeof(shm_struct.qd))==-1) return(-1);
			
			tcdatapkt.cmd = EAPI_OUTPKT_DATA;
			tcdatapkt.len = sizeof(mh_loc.q) + sizeof(mh_loc.qd);
			memcpy((void*)&tcdatapkt.q, (void*)mh_loc.q, sizeof(mh_loc.q));
			memcpy((void*)&tcdatapkt.qd, (void*)mh_loc.qd, sizeof(mh_loc.qd));
			rc = udp_send(&tcdatapkt, sizeof(tcdatapkt));
			if (rc==-1) { printf("w5d.c: w5d_msg_handler(): udp send (torque control data packet) failed \n"); return(rc); }
			break;
		
		// Host requesting Data (joints, joint velocities, etc..) 
		case EAPI_INPKT_GET_DATA:
			if ((*packet).len != 0) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			// Copy out the joint angles into a return packet.
			if(shm_get((void*)mh_loc.q, (void*)shm_struct.q, sizeof(shm_struct.q))==-1) return(-1);
			
			// Copy out the joint velocities into return packet.
			if(shm_get((void*)mh_loc.qd, (void*)shm_struct.qd, sizeof(shm_struct.qd))==-1) return(-1);
			
			send_packet.cmd = EAPI_OUTPKT_DATA;
			send_packet.len = sizeof(mh_loc.q) + sizeof(mh_loc.qd);
			memcpy((void*)&send_packet.payload[0], (void*)mh_loc.q, sizeof(mh_loc.q));
			memcpy((void*)&send_packet.payload[sizeof(mh_loc.q)], (void*)mh_loc.qd, sizeof(mh_loc.qd));
			size = sizeof(send_packet) - sizeof(send_packet.payload) + send_packet.len;
			
			rc = udp_send(&send_packet, size);
			if (rc==-1) { printf("w5d.c: w5d_msg_handler(): udp send (force control data packet) failed \n"); return(rc); }
			break;
		
		// Enable Force Control Mode
		case EAPI_INPKT_ENABLE_FC:	
			if ((*packet).len != 0) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			// verify we're in the disabled state
			// and the device is homed
			if ((mh_loc.w5d_state == W5D_STATE_DISABLED)&&(mh_loc.w5d_homed == W5D_HOMED))
			{
				w5d_send_ack(ACK_STATE_CHANGED);
				mh_loc.w5d_state = W5D_STATE_FORCE_CONTROL_IN;
				if (shm_put((void*)&shm_struct.w5d_state, (void*)&mh_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			}
			else if (mh_loc.w5d_state == W5D_STATE_FORCE_CONTROL)
			{
				w5d_send_ack(ACK_STATE_CHANGED);
				#ifdef W5D_DEBUG
				printf("w5d.c: w5d_msg_handler(): already in force control state \n");
				#endif
				return(0);
			}
			else
			{
				#ifdef W5D_DEBUG
				printf("w5d.c: w5d_msg_handler(): Device must be homed and in disabled state \n");
				#endif
				
				// send a nack packet
				rc = w5d_send_nack(NACK_STATE_CHANGE_REQ);
				if (rc==-1) { printf("w5d.c: w5d_msg_handler(): nack send call failed \n"); return(rc); }
				return(0);
			}
			break;
		
		// Enable Torque Control Mode
		case EAPI_INPKT_ENABLE_TC:	
			if ((*packet).len != 0) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			// verify we're in the disabled state
			if (mh_loc.w5d_state == W5D_STATE_DISABLED)
			{
				w5d_send_ack(ACK_STATE_CHANGED);
				mh_loc.w5d_state = W5D_STATE_TORQUE_CONTROL_IN;
				if (shm_put((void*)&shm_struct.w5d_state, (void*)&mh_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			}
			else if (mh_loc.w5d_state == W5D_STATE_TORQUE_CONTROL)
			{
				w5d_send_ack(ACK_STATE_CHANGED);
				#ifdef W5D_DEBUG
				printf("w5d.c: w5d_msg_handler(): already in torque control state \n");
				#endif
				return(0);
			}
			else
			{
				#ifdef W5D_DEBUG
				printf("w5d.c: w5d_msg_handler(): attempting invalid state change \n");
				#endif
				
				// send a nack packet
				rc = w5d_send_nack(NACK_STATE_CHANGE_REQ);
				if (rc==-1) { printf("w5d.c: w5d_msg_handler(): nack send call failed \n"); return(rc); }
				
				return(0);
			}
			break;
		
		// Disable Device
		case EAPI_INPKT_DISABLE:	
			if ((*packet).len != 0) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			if (mh_loc.w5d_state == W5D_STATE_DISABLED)
			{
				w5d_send_ack(ACK_STATE_CHANGED);
				#ifdef W5D_DEBUG
				printf("w5d.c: w5d_msg_handler(): already in disabled state \n");
				#endif
				return(0);
			}
			
			w5d_send_ack(ACK_STATE_CHANGED);
			mh_loc.w5d_state = W5D_STATE_DISABLED_IN;
			if (shm_put((void*)&shm_struct.w5d_state, (void*)&mh_loc.w5d_state, sizeof(shm_struct.w5d_state))==-1) return(-1);
			break;
		
		// Get Device Status
		case EAPI_INPKT_STATUS:
			#ifdef W5D_DEBUG
			printf("w5d.c: w5d_msg_handler(): %3.3f: status request  \n", seconds_counter);
			#endif			
			if ((*packet).len != 0) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			if (w5d_read_devicefiles(&serial_no, device_type, device_name)==-1)
			{
				serial_no = -1;
				memcpy(device_type, "unknown\n", 8);
				memcpy(device_name, "unknown\n", 8);
			}
			
			// filling a packet full of system information
			statuspkt.cmd = EAPI_OUTPKT_STATUS;
			statuspkt.len = 0;
			memcpy((void*) &statuspkt.device_type, (void*) &device_type, sizeof(device_type));
			memcpy((void*) &statuspkt.device_name, (void*) &device_name, sizeof(device_name));
			statuspkt.serial_no = serial_no;
			statuspkt.state = (uint32_t) mh_loc.w5d_state;
			statuspkt.homed = (uint32_t) mh_loc.w5d_homed;
			statuspkt.faultvector = (uint32_t) mh_loc.w5d_faultvec;
			size = sizeof(statuspkt);
			rc = udp_send(&statuspkt, size);
			if (rc==-1) { printf("w5d.c: w5d_msg_handler(): udp send (status packet) failed \n"); return(rc); }
			break;
		
		// Home the Device
		case EAPI_INPKT_HOME:	
			if ((*packet).len != 0) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			// verify we're in the disabled state
			if (mh_loc.w5d_state == W5D_STATE_DISABLED)
			{
				w5d_send_ack(ACK_HOMED);
				mh_loc.dohome = W5D_DOHOME;
				if (shm_put((void*)&shm_struct.dohome, (void*)&mh_loc.dohome, sizeof(shm_struct.dohome))==-1) return(-1);
			}
			else
			{
				#ifdef W5D_DEBUG
				printf("w5d.c: w5d_msg_handler(): must be in disabled state to home device \n");
				#endif
				
				// send a nack packet
				rc = w5d_send_nack(NACK_STATE_CHANGE_REQ);
				if (rc==-1) { printf("w5d.c: w5d_msg_handler(): nack send call failed  \n"); return(rc); }
			}
			break;
		
		// Set the real-time loop frequency
		case EAPI_INPKT_SET_FREQ:
			// on unsigned intefer
			if ((*packet).len != 4) { w5d_send_nack(NACK_LENGTH_ERROR); return(0); }
			
			// read in the frequency
			memcpy((void*) &mh_loc.w5d_freq, (void*)(*packet).payload, sizeof(mh_loc.w5d_freq)); 
			
			w5d_send_ack(ACK_FREQ_CHANGED);
			
			// verify the frequency is in range
			if (mh_loc.w5d_freq < W5D_FREQ_MIN) mh_loc.w5d_freq = W5D_FREQ_MIN;
			if (mh_loc.w5d_freq > W5D_FREQ_MAX) mh_loc.w5d_freq = W5D_FREQ_MAX;
			
			// update frequency in shared memory 
			if (shm_put((void*)&shm_struct.w5d_freq, (void*)&mh_loc.w5d_freq, sizeof(shm_struct.w5d_freq))==-1) return(-1);
			
			printf("w5d.c: w5d_msg_handler(): frequency set to %d \n", mh_loc.w5d_freq);
			break;
		
		// Host PC attempting to discover the IP for this device 
		case EAPI_INPKT_DISCOVER:
			printf("w5d.c: w5d_msg_handler(): received a network discover packet\n");
			
			//reply with a handshaking packet
			send_packet.cmd = EAPI_OUTPKT_DISCOVER;
			send_packet.len = 0;
			size = sizeof(send_packet) - sizeof(send_packet.payload) + send_packet.len;
			rc = udp_send(&send_packet, size);
			if (rc==-1) { printf("w5d.c: w5d_msg_handler(): udp send (discover handshake packet) failed \n"); return(rc); }
			break;
		
		// Host PC would like encoder positions outputted to the terminal 
		case EAPI_INPKT_DISPLAY_ENC:
			// toggles on and off
			display_encoders^=1;
			break;
		
		// Invalid command
		default:
			printf("w5d.c: received invalid packet identifier\n");
			// send a nack packet
			rc = w5d_send_nack(NACK_UNKNOWN_PACKET);
			if (rc==-1) { printf("w5d.c: w5d_msg_handler(): nack send call failed \n"); return(rc); }
	        break;
	} 
	
	return(rc);
}


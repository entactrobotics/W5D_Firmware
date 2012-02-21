// --------------------------------------------------------------------------------------
// Module     : PROFILING_MAIN
// Description: Testing code used to evaluate computational timing of calls
// Author     : R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------
//

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <stdint.h>

#include "mem.h"
#include "spi.h"
#include "udp.h"
#include "shm.h"
#include "w5d.h"

/* PI (might be in math.h) */
#define PI (3.14159265)

/* Link Lengths */
#define L1	(67.5)
#define L2	(45.0)
#define L3	(12.5)
#define L4	(160.0)
#define L5	(14.0)

/* Home Offset */
#define phi	 (0.78539816) // pi/4

/* K encoder 0 thru 5 */
#define K_ENC05	(2*PI/5000)

void signal_handler(int signal);
int kincode(float *q, float *f, float *tau);

static volatile int loop_done = 0;
char *build_date = __DATE__;
char *build_time = __TIME__;

int main(int argc, char **argv)
{
    int rc;
    struct sched_param sp;
    struct timespec start_time, end_time;
    double diff_time; 
	int nLoops = 0;
	
	
	float tau[6] = {100,0,0,0,0,0};
    int32_t e[8];
	float q[8], f[6];
	
    printf("\n\nEntact Robotics\n\n");
    printf("W5D Haptics Firmware\n\n");

    printf("Build time: %s\n", build_time);
    printf("Build date: %s\n\n", build_date);

    // Init the data structures
    memset(&sp, 0, sizeof(sp));
    memset(&start_time, 0, sizeof(start_time));
    memset(&end_time, 0, sizeof(end_time));
    
    // Check to see if we can run with the current priviledges
    rc = (int)geteuid();
    if(rc != 0)
    {
        fprintf(stderr, "ERR: try running with sudo\n");
        return(-1);
    }

    // Handle CTRL-C
    signal(SIGINT, signal_handler);
    
    // Set max priority level
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    rc = sched_setscheduler(0, SCHED_FIFO, &sp);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: failed to change priority %s\n",  strerror(errno));
        return(-1);
    }
    
    // Lock the process memory to avoid paging
    rc = mlockall(MCL_CURRENT|MCL_FUTURE);
    if(rc != 0)
    {
        fprintf(stderr, "ERR: mlockall() %d\n", rc);
        perror("errno");
        return(-1);
    }
    
    printf("main.c: configuring embedded system \n");
    mem_init();
	mem_slave_disable();
    spi_init();
    shm_init();
    udp_init();
    w5d_init();
	
    printf("main.c: loop testing the function call \n");
    
	
	mem_slave_enable();
	nLoops = 0;
	
	// *** KEEP FIRST
	clock_gettime(CLOCK_MONOTONIC, &start_time); // Get the start time

	while (nLoops<100000)
	{
		
		// testing encoder read latency
		//mem_encoder_read_all(e);
		//printf("\r %d %d %d %d %d %d %d %d   ",e[0],e[1],e[2],e[3],e[4],e[5],e[6],e[7]);
		
		
		// testing torque command latency
		if (spi_torque_command(tau)==-1) {printf("torque set failed \n"); return(-1);}
		
		
		// testing kinematic computational latency
		/*
		q[0] = 1e9*nLoops;	// fill with dummy data
		q[1] = 1e9*nLoops;
		q[2] = 1e9*nLoops;
		q[3] = 1e9*nLoops;
		q[4] = 1e9*nLoops;
		q[5] = 1e9*nLoops;
		q[6] = 1e9*nLoops;
		q[7] = 1e9*nLoops;
		*/
		/*
		q[0] = -e[0]*K_ENC05; 
		q[1] = -e[1]*K_ENC05;
		q[2] = -e[2]*K_ENC05;
		q[3] = e[3]*K_ENC05;
		q[4] = e[4]*K_ENC05;
		q[5] = -e[5]*K_ENC05;
		q[6] = -e[6]*K_ENC6;
		q[7] = e[7];
	
		f[0] = 1e9*nLoops;
		f[1] = 1e9*nLoops;
		f[2] = 1e9*nLoops;
		f[3] = 1e9*nLoops;
		f[4] = 1e9*nLoops;
		f[5] = 1e9*nLoops;
		if (!kincode(q, f, tau)) { printf("kincode call fail \n");}
		*/
		
		
		/*
		// testing shared memory access latency 
		rc = shm_get((void*)f, (void*)shm_struct.force_setpoint, sizeof(shm_struct.force_setpoint));
		if (rc==-1) { printf("shared memory get (force setpoint) failed \n"); return(rc); }
		*/
		
		nLoops++;
	}
	
	// *** KEEP LAST
	clock_gettime(CLOCK_MONOTONIC, &end_time); // Get the end time
	diff_time = ((end_time.tv_sec*1e9) + end_time.tv_nsec) - ((start_time.tv_sec*1e9) + start_time.tv_nsec); // elapsed time in nano-seconds
	
	printf("Loop Time: %lf s\n", diff_time/1e9/nLoops);
	
    printf("main.c: application done\n");
	mem_slave_disable();
	mem_uninit();
    return(0);
}

int kincode(float *q, float *f, float *tau)
{
		
	// calculating the required motor torques given the joint
	// angles q[] and the desired force f[]
	float t1 = cos(q[2]);
	float t2 = q[0] + phi;
	float t3 = cos(t2);
	float t4 = t1 * t3;
	float t5 = L4 * f[0];
	float t8 = sin(t2);
	float t9 = t8 * L4;
	float t12 = sin(q[2]);
	float t13 = t12 * t3;
	float t14 = L4 * f[2];
	float t18 = q[1] + phi;
	float t19 = cos(t18);
	float t20 = t19 * L4;
	float t21 = -L3 + t9 + t20;
	float t22 = t12 * t21;
	float t23 = t1 * L2;
	float t24 = sin(q[5]);
	float t25 = q[3] + phi;
	float t26 = sin(t25);
	float t27 = t26 * L4;
	float t28 = q[4] + phi;
	float t29 = cos(t28);
	float t30 = t29 * L4;
	float t31 = -L3 + t27 + t30;
	float t32 = t24 * t31;
	float t33 = cos(q[5]);
	float t34 = t33 * L2;
	float t35 = (float) (2 * L1) - t22 - t23 + t32 - t34;
	float t36 = 0.1e1 / t35;
	float t39 = sin(t18);
	float t41 = cos(t25);
	float t43 = sin(t28);
	float t45 = -t3 * L4 + t39 * L4 + t41 * L4 - t43 * L4;
	float t46 = t35 * t35;
	float t47 = 0.1e1 / t46;
	float t48 = t45 * t47;
	float t49 = t13 * L4;
	float t52 = t45 * t45;
	float t55 = 0.1e1 / (0.1e1 + t52 * t47);
	float t58 = L4 * t36;
	float t60 = t1 * t21;
	float t61 = t12 * L2;
	float t62 = t33 * t31;
	float t63 = t24 * L2;
	float t64 = t60 - t61 - t62 - t63;
	float t65 = t64 * t47;
	float t68 = t64 * t64;
	float t71 = 0.1e1 / (0.1e1 + t68 * t47);
	float t75 = t1 * t39;
	float t80 = t12 * t39;
	float t84 = t80 * L4;
	float t103 = -t60 + t61;
	float t114 = t33 * t41;
	float t119 = t24 * t41;
	float t123 = t119 * L4;
	float t134 = t33 * t43;
	float t139 = t24 * t43;
	float t143 = t139 * L4;
	float t162 = t62 + t63;
	
	tau[0] = 0.5e0 * t4 * t5 + 0.5e0 * t9 * f[1] - 0.5e0 * t13 * t14 - (t9 * t36 + t48 * t49) * t55 * f[3] + (t4 * t58 + t65 * t49) * t71 * f[4];
	tau[1] = -0.5e0 * t75 * t5 + 0.5e0 * t20 * f[1] + 0.5e0 * t80 * t14 - (t20 * t36 - t48 * t84) * t55 * f[3] + (-t75 * t58 - t65 * t84) * t71 * f[4];
	tau[2] = (-0.5e0 * t22 - 0.5e0 * t23) * f[0] + (-0.5e0 * t60 + 0.5e0 * t61) * f[2] + t48 * t103 * t55 * f[3] + ((-t22 - t23) * t36 - t65 * t103) * t71 * f[4];
	tau[3] = 0.5e0 * t114 * t5 + 0.5e0 * t27 * f[1] - 0.5e0 * t119 * t14 - (-t27 * t36 - t48 * t123) * t55 * f[3] + (-t114 * t58 - t65 * t123) * t71 * f[4];
	tau[4] = -0.5e0 * t134 * t5 + 0.5e0 * t30 * f[1] + 0.5e0 * t139 * t14 - (-t30 * t36 + t48 * t143) * t55 * f[3] + (t134 * t58 + t65 * t143) * t71 * f[4];
	tau[5] = (-0.5e0 * t32 + 0.5e0 * t34) * f[0] + (-0.5e0 * t62 - 0.5e0 * t63) * f[2] + t48 * t162 * t55 * f[3] + ((t32 - t34) * t36 - t65 * t162) * t71 * f[4];	
	
	return (1);
}

void signal_handler(int signal)
{
    printf("\nmain.c: signal_handler: control-c shutdown in progress \n");
    // Perform device shutdown/disable and cleanup
	mem_slave_disable();
	mem_uninit();
}



// --------------------------------------------------------------------------------------
// Module     : MAIN
// Description: Initialization and  Real-Time loop
// Author     : N. El-Fata
//            : Revised by R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// Pulse Innovation inc. www.pulseinnovation.com
// --------------------------------------------------------------------------------------
//
// References:
// http://export.writer.zoho.com/public/rreginelli/Chapter-5---High-Resolution-Timers-Final1/fullpage
//

#include <stdio.h>
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

#define FREQ_MIN (100)
#define FREQ_MAX (5000)

void signal_handler(int signal);

static volatile int loop_done = 0;

int main(int argc, char **argv)
{
    int rc;
    struct sched_param sp;
    struct timespec start_time, end_time, sleep_time;
    double diff_time, loop_period; 
    int32_t loop_freq = FREQ_MIN;
    int32_t serial_no = -1;
    char name[32];
    
    printf("\n\nEntact Robotics\n\n");
    printf("W5D Haptics Firmware\n\n");

    // Init the data structures
    memset(&sp, 0, sizeof(sp));
    memset(&start_time, 0, sizeof(start_time));
    memset(&end_time, 0, sizeof(end_time));
    memset(&sleep_time, 0, sizeof(sleep_time));
    
    // serial number printout
    FILE *serial_file = fopen("serial.txt","rb");
    if (serial_file == NULL)
    {
        printf("ERR: failed to get serial");
    }
    else
    {
        if(fscanf(serial_file, "%d", &serial_no)==1) printf("main.c: device serial number: %d \n", serial_no);
        fclose(serial_file);
    }
    
    // name printout
    FILE *name_file = fopen("name.txt","rb");
    if (name_file == NULL)
    {
        printf("ERR: failed to get name");
    }
    else
    {
        if (fgets(name, sizeof(name), name_file) != NULL) printf("main.c: device name: %s \n", name);
        //if(fscanf(name_file, "%s", name)==1) printf("main.c: name %s \n", name);
        fclose(name_file);
    }

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
    spi_init();
    shm_init();
    udp_init();
    w5d_init();

    printf("main.c: initializing real-time thread \n");
    while(loop_done == 0)
    {
        // *** KEEP FIRST
        clock_gettime(CLOCK_MONOTONIC, &start_time); // Get the start time

        // call w5d realtime handler
        if (w5d_real_time()==-1)
        {
            printf("main.c: real-time-loop: real-time loop failure (terminating thread)\n");
            return(-1);
        }

        // *** KEEP 2ND LAST - update loop period (usually it just stays the same)
        rc = shm_get((void*)&loop_freq, (void*)&shm_struct.w5d_freq,  sizeof(loop_freq));
		if (rc==-1) { printf("w5d.c: realtime thread: shared memory get (freq) failed \n"); return(rc); }
        if (loop_freq < FREQ_MIN) loop_freq = FREQ_MIN;
		if (loop_freq > FREQ_MAX) loop_freq = FREQ_MAX;
        loop_period = 1e9*(1/((double)loop_freq));
        
        // *** KEEP LAST
        clock_gettime(CLOCK_MONOTONIC, &end_time); // Get the end time
        diff_time = ((end_time.tv_sec*1e9) + end_time.tv_nsec) - ((start_time.tv_sec*1e9) + start_time.tv_nsec); // elapsed time in nano-seconds
        
        //printf("\r freq = %d   ", loop_freq);
        if(diff_time > loop_period)
        {
	    #ifdef W5D_DEBUG
            printf("main.c: realtime thread: realtime overrun\n");
	    #endif
        }
        else
        {
            sleep_time.tv_sec = 0;
            sleep_time.tv_nsec = loop_period - diff_time;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
            //printf("\rSleep time (ms) = %3.2f                                 ", (LOOP_TIME_NS - diff_time)/1000000);
        }
    }

    printf("main.c: haptics application done\n");
    return(0);
}

void signal_handler(int signal)
{
    printf("\nmain.c: signal_handler: control-c shutdown in progress \n");
    // Perform device shutdown/disable and cleanup
    loop_done = 1;
    udp_uninit();
    mem_slave_disable();
	w5d_uninit();
}



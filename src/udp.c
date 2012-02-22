// --------------------------------------------------------------------------------------
// Module     : UDP
// Description: UDP utilities
// Author     : R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------

#include <stdint.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/stat.h>
#include <pthread.h>
#include "udp.h"
#include "w5d.h"
#include "eapi_packets.h"

static volatile int    thread_done      = 0;
static volatile int    udp_client_valid = 0;

static volatile int port = UDP_PORT;            // The port used by the W5D to listen for packets
static volatile struct sockaddr_in w5d_addr;    // W5D device address structure
static volatile struct sockaddr_in pc_addr;     // remote PC address structure 
static volatile int w5d_sd;                     // w5d socket descriptor  


void *udp_thread_recv(void *arg);

int udp_init(void)
{
    int rc;
    pthread_t thread_pthread;
    pthread_attr_t thread_attr;
    int thread_id;
    int arg = 0; // potential argument to be passed into new thread

    // Initializing UDP Listen Thread
    printf("udp.c: initializing message handler thread \n");
    rc = pthread_attr_init(&thread_attr); // Get default thread attributes
    if(rc != 0) { printf("udp.c: error pthread_attr_init failed \n"); return(-1);}

    // Create thread
    thread_id = pthread_create(&thread_pthread, &thread_attr, &udp_thread_recv, (void *)arg);
    if(thread_id < 0) { printf("udp.c: error pthread_createpthread_attr_init failed \n"); return(-1); }
    
    return(0);
}

int udp_uninit(void)
{
    printf("udp.c: exiting message handler thread\n");
    thread_done = 1;
    usleep(10000);  // wait for thread to complete
    
    return(0);
}

// UDP Message Handling Thread
void *udp_thread_recv(void *arg)
{
    int msg_in_size;
    int sockaddr_len;
    udp_pkt_t  inpacket;    
    int packet_valid;
    
    udp_client_valid = 0;
    thread_done = 0;
    memset((void *)&w5d_addr, 0, sizeof(w5d_addr));    
    
    w5d_addr.sin_family         = AF_INET;
    w5d_addr.sin_addr.s_addr    = INADDR_ANY;
    w5d_addr.sin_port           = htons(port);

    if((w5d_sd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    { printf("udp.c: udp_create() create new UDP socket failed \n"); exit(1); }

    if(bind(w5d_sd, (struct sockaddr *) &w5d_addr, sizeof(struct sockaddr_in)) != 0)
    { fprintf(stderr, "udp.c: udp_create(): bind to port error \n"); exit(1); }
    
    printf("udp.c: udp_create(): creating socket on port %d\n", ntohs(w5d_addr.sin_port));
    

    while(thread_done == 0)
    {
        #ifdef UDP_DEBUG   
        printf("udp.c: udp_thread_recv(): waiting for message...\n");
        #endif
        
        // Wait to receive packet header portion, only works when no errors with MSG_WAITALL flag
        msg_in_size = recvfrom(w5d_sd, &inpacket, sizeof(inpacket), MSG_WAITALL, (struct sockaddr *)&pc_addr, (socklen_t *)&sockaddr_len);
        
        // validate the receive size of the packet
        packet_valid = 1; // innocent until proven guilty
        if (msg_in_size!=(inpacket.len + 4))
        {
            if (msg_in_size < 0) { printf("udp.c: udp_thread_recv(): recvfrom error\n"); }
            else { printf("udp.c: udp_thread_recv(): size mismatch for received packet\n"); }
            packet_valid = 0;
        }

        if (packet_valid)
        {
            // let the udp_send know that the client has connected so sending is allowed
            udp_client_valid = 1;
            
            #ifdef UDP_DEBUG    
            printf("udp.c: udp_thread_recv(): received message: port(%d) IP(%s)\n", ntohs(pc_addr.sin_port), inet_ntoa(pc_addr.sin_addr));
            printf("udp.c: udp_thread_recv(): message size %d\n", msg_in_size);
            #endif
            
            if (w5d_msg_handler(&inpacket)==-1) { printf("udp.c: udp_thread_recv(): message handler failed\n"); }
        }
    }
    pthread_exit(NULL);
}

int udp_send(void *buf, int len)
{
    int rc;

    // Note this function is called in w5d.c from the message handler function
    
    // client did not connect yet
    if(udp_client_valid == 0) return(-1);
    
    rc = sendto(w5d_sd, buf, len, 0, (struct sockaddr *)&pc_addr, sizeof(pc_addr));
    if(rc == -1) { printf("udp.c: udp_send(): sendto error \n"); return(-1); }
    if(rc != len) { printf("udp.c: udp_send(): sent improper number of bytes \n"); return(-1); }

    return(0);
}




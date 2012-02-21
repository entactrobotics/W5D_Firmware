// --------------------------------------------------------------------------------------
// Module     : SHM
// Description: Shared memory access functions
// Author     : N. El-Fata
// --------------------------------------------------------------------------------------
// Pulse Innovation, Inc. www.pulseinnovation.com
// --------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <errno.h>
#include "shm.h"

volatile shm_struct_t shm_struct;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

int shm_init(void)
{
    memset((void *)&shm_struct, 0, sizeof(shm_struct));

    pthread_mutexattr_t mutexAttr;
    pthread_mutexattr_settype(&mutexAttr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&mutex, &mutexAttr);

    return(0);
}

//
// Copy part of the shm_struct from source (address within shm_struct) into destination
//
int shm_get(uint8_t *dst, uint8_t *src, uint32_t len)
{
    int rc;
    uint32_t src_addr    = (uint32_t)src;
    uint32_t struct_addr = (uint32_t)&shm_struct;

    if(len < 1)
    {   // nothing to be done
        return 0;
    }
    // Make sure src is within the shm_struct address range
    if(src_addr < struct_addr)
    {
        printf("ERR: shm_get src address out of range\n");
        return -1;
    }
    if((src_addr + len) > (struct_addr + sizeof(shm_struct)))
    {
        printf("ERR: shm_get len generates out of range address\n");
        return -1;
    }

    //
    // Critical section start
    //
    rc = pthread_mutex_lock(&mutex);
    if(rc != 0)
    {
        printf("ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }

    memcpy((void *)dst, (void *)src, len);

    //
    // Critical section end
    //
    rc = pthread_mutex_unlock(&mutex);
    if(rc != 0)
    {
        printf("ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }

    return(0);
}

//
// Copy part of the shm_struct from source into destination (address within shm_struct)
//
int shm_put(uint8_t *dst, uint8_t *src, uint32_t len)
{
    int rc;
    uint32_t dst_addr    = (uint32_t)dst;
    uint32_t struct_addr = (uint32_t)&shm_struct;

    if(len < 1)
    {   // nothing to be done
        return 0;
    }
    // Make sure dst is within the shm_struct address range
    if(dst_addr < struct_addr)
    {
        printf("ERR: shm_get src address out of range\n");
        return -1;
    }
    if((dst_addr + len) > (struct_addr + sizeof(shm_struct)))
    {
        printf("ERR: shm_get len generates out of range address\n");
        return -1;
    }
    //
    // Critical section start
    //
    rc = pthread_mutex_lock(&mutex);
    if(rc != 0)
    {
        printf("ERR: pthread_mutex_lock: %s\n",  strerror(errno));
        exit(1);
    }

    memcpy((void *)dst, (void *)src, len);

    //
    // Critical section end
    //
    rc = pthread_mutex_unlock(&mutex);
    if(rc != 0)
    {
        printf("ERR: pthread_mutex_unlock: %s\n",  strerror(errno));
        exit(1);
    }

    return(0);
}



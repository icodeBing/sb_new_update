#include "create_threads.h"
#include <time.h>
#include <unistd.h>
#include <error.h>
#include <stdio.h>
#include <pthread.h>
//#include <cstdint>

bool ecat_mode = true;

bool flag = true;

uint64_t sync_t = 0;

void setSyncClock()
{
    sync_t++;
}

long getSyncClock()
{
    return sync_t;
}

op_ op0, op1;

bool ethercat_mode(int xml_mode)
{
    if (xml_mode == 0 || xml_mode == 1){
        ecat_mode = true;
        return true;
    }
    else if (xml_mode == 2){
        ecat_mode = false;
        return false;
    }
}

void cal_cycle(struct timespec *st)
{
    if (st->tv_nsec >= 1000000000){
        st->tv_nsec = 0;
        st->tv_sec++;
    }
}

void *spi_clock(void *clock_cycle)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(1, &mask);
	pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

    long *cycle = clock_cycle;
    struct timespec now, interal;
    interal.tv_nsec = *cycle;
    interal.tv_sec = 0;

    __RT(clock_gettime(CLOCK_MONOTONIC, &now));
    now.tv_nsec += interal.tv_nsec;
    now.tv_sec += interal.tv_sec;
    cal_cycle(&now);

    while (flag)
    {
        __RT(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &now, NULL));
        setSyncClock();
        now.tv_nsec += interal.tv_nsec;
        now.tv_sec += interal.tv_sec;
        cal_cycle(&now);
    }
}

void callback_spi_0(int cb())
{
    op0.Function = cb;
    // op0.eth = dd;
}

void callback_spi_1(int cb())
{
    op1.Function = cb;
    // op1.eth = dd;
}

void *running0()
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
	pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

    spi_sync lstSync = 0;
    spi_sync curSync = 0;
    while (flag)
    {
        if (ecat_mode)
            curSync = getsync();
        else
            curSync = getSyncClock();
        if(lstSync == curSync)
        {
            __RT(usleep(50));
            continue;
        }
        lstSync = curSync;
        
        //
        if (op0.Function()){
            __RT(printf("Please check SPI_0 device: No connect SPI_0 or SPI_0 is damaged!\n"));
            break;
        }
    }
}

void *running1()
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
	pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

    spi_sync lstSync = 0;
    spi_sync curSync = 0;
    while (flag)
    {
        if (ecat_mode)
            curSync = getsync();
        else
            curSync = getSyncClock();
        if(lstSync == curSync)
        {
            __RT(usleep(50));
            continue;
        }
        lstSync = curSync;

        //
        if (op1.Function()){
            __RT(printf("Please check SPI_1 device: No connect SPI_1 or SPI_1 is damaged!\n"));
            break;
        }
    }
}

static pthread_t thread0, thread1, thread2;

void spi_create_pthread(int priority, char io, long clock_cycle)
{   
    int status;
    pthread_attr_t attr;
    
    struct sched_param p;

    status = pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	p.sched_priority = priority; //
	pthread_attr_setschedparam(&attr, &p);

    if (io == 0){
	    status = __RT(pthread_create(&thread0, &attr, running0, NULL));
        if (status)
            printf("Fail to create thread SPI_0!\n");
        __RT(pthread_setname_np(thread0, "SPI-0"));
    }
    if (io == 1){
	    status = __RT(pthread_create(&thread1, &attr, running1, NULL));
        if (status)
            printf("Fail to create thread SPI_1!\n");
        __RT(pthread_setname_np(thread1, "SPI-1"));
    }
    if (io == 2){
	    status = __RT(pthread_create(&thread2, &attr, spi_clock, &clock_cycle));
        if (status)
            printf("Fail to create thread SPI!\n");
        __RT(pthread_setname_np(thread2, "SPI"));
    }
}

void spi_close_pthread(char io)
{
    if (io == 0)
        __RT(pthread_join(thread0, NULL));
    if (io == 1)
        __RT(pthread_join(thread1, NULL)); 
    if (io == 2)
        __RT(pthread_join(thread2, NULL)); 
}

void spi_stop()
{
    flag = false;
}

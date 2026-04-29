#ifndef __CREATE_THREADS_H
#define __CREATE_THREADS_H

#include "ec_task.h"
#include <getopt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>
// #include <pthread.h>

typedef unsigned long spi_sync;

typedef struct {
    int (* Function)();
    // spi_sync *eth;
} op_;

void callback_spi_0(int cb());

void callback_spi_1(int cb());

static void *running0();

static void *running1();

void spi_create_pthread(int priority, char io, long clock_cycle);

void spi_close_pthread(char io);

bool ethercat_mode(int xml_mode);

void *spi_clock();

void spi_stop();

#endif
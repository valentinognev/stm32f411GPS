
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdlib.h>
#include <string.h>
#include "main.h"

#define NBUFF 40
#define QUEUELEN NMEA_GPRMC_SENTENCE_SIZE

typedef struct
{
    char data[NBUFF][QUEUELEN];
    int count;
} NMEAQueue;

void NMEAQueue_init(NMEAQueue *buf);
int NMEAQueue_empty(NMEAQueue *buf);
int NMEAQueue_full(NMEAQueue *buf);
int NMEAQueue_push(NMEAQueue *buf, const char *data);
int NMEAQueue_pop(NMEAQueue *buf, char *data);

#endif // CIRCULAR_BUFFER_H
#include "NMEAQueue.h"


void NMEAQueue_init(NMEAQueue *buf)
{
    memset(buf->data, 0, sizeof(buf->data));
    buf->count = 0;
}

int NMEAQueue_empty(NMEAQueue *buf)
{
    return buf->count == 0;
}

int NMEAQueue_full(NMEAQueue *buf)
{
    return buf->count == NBUFF;
}

int NMEAQueue_push(NMEAQueue *buf, const char *data)
{
    if (NMEAQueue_full(buf))
    {
        return -1;
    }
    strncpy(buf->data[buf->count], data, QUEUELEN - 1);
    buf->count++;
    return 0;
}

int NMEAQueue_pop(NMEAQueue *buf, char *data)
{
    if (NMEAQueue_empty(buf))
    {
        return -1;
    }
    strncpy(data, buf->data[0], QUEUELEN - 1);
    buf->count--;
    memmove(buf->data[0], buf->data[1], (buf->count) * QUEUELEN);
    memset(buf->data[buf->count], 0, QUEUELEN);
    return 0;
}
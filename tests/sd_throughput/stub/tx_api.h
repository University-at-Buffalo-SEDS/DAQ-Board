/* Host model of the ThreadX services used by the SD writer. Single CPU,
 * virtual time: see tests/sd_throughput/harness.c. Not ThreadX. */
#ifndef TX_API_H
#define TX_API_H
#include <stdint.h>
#include <stddef.h>
#ifndef VOID
#define VOID void
typedef char CHAR;
typedef char BOOL;
typedef unsigned char UCHAR;
typedef int INT;
typedef unsigned int UINT;
typedef long LONG;
typedef unsigned long ULONG;
typedef short SHORT;
typedef unsigned short USHORT;
#endif
#ifndef ULONG64_DEFINED
#define ULONG64_DEFINED
typedef unsigned long long ULONG64;
#endif
#define TX_SUCCESS 0x00U
#define TX_POOL_ERROR 0x02U
#define TX_NO_MEMORY 0x10U
#define TX_QUEUE_EMPTY 0x0AU
#define TX_QUEUE_FULL 0x0BU
#define TX_NO_EVENTS 0x07U
#define TX_QUEUE_ERROR 0x09U
#define TX_GROUP_ERROR 0x06U
#define TX_NO_WAIT 0UL
#define TX_WAIT_FOREVER 0xFFFFFFFFUL
#define TX_OR 0U
#define TX_OR_CLEAR 1U
#define TX_1_ULONG 1U
#define TX_AUTO_START 1U
#define TX_NULL ((void *)0)
#define TX_TIMER_TICKS_PER_SECOND 1000UL
typedef struct { int unused; } TX_THREAD;
typedef struct { int unused; } TX_BYTE_POOL;
typedef struct {
  ULONG *tx_queue_start, *tx_queue_end, *tx_queue_read, *tx_queue_write;
  UINT tx_queue_message_size, tx_queue_capacity, tx_queue_enqueued;
} TX_QUEUE;
typedef struct {
  UCHAR *tx_block_pool_start; ULONG tx_block_pool_size, tx_block_pool_block_size;
  UINT tx_block_pool_total, tx_block_pool_available;
  UCHAR **free_list; UINT free_top;
} TX_BLOCK_POOL;
typedef struct { ULONG flags; } TX_EVENT_FLAGS_GROUP;
UINT tx_queue_create(TX_QUEUE *q, CHAR *name, UINT size, VOID *start, ULONG bytes);
UINT tx_queue_send(TX_QUEUE *q, VOID *message, ULONG wait);
UINT tx_queue_receive(TX_QUEUE *q, VOID *message, ULONG wait);
UINT tx_block_pool_create(TX_BLOCK_POOL *p, CHAR *name, ULONG block, VOID *start, ULONG bytes);
UINT tx_block_allocate(TX_BLOCK_POOL *p, VOID **block, ULONG wait);
UINT tx_block_release(VOID *block);
UINT tx_event_flags_create(TX_EVENT_FLAGS_GROUP *g, CHAR *name);
UINT tx_event_flags_set(TX_EVENT_FLAGS_GROUP *g, ULONG flags, UINT option);
UINT tx_event_flags_get(TX_EVENT_FLAGS_GROUP *g, ULONG requested, UINT option, ULONG *actual, ULONG wait);
UINT tx_thread_sleep(ULONG ticks);
ULONG tx_time_get(void);
TX_THREAD *tx_thread_identify(void);
#endif

#ifndef MAVLINK_DATA_QUEUE_H
#define MAVLINK_DATA_QUEUE_H

#include <mavlink/common/mavlink.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>

// Constants
#define MAVLINK_DATA_QUEUE_SIZE 256  // Size of MAVLink data queue

// MAVLink message data structure for queue
typedef struct {
    char msg_type[64];              // Message type name
    mavlink_message_t msg;           // MAVLink message
    double timestamp;                // Reception timestamp
} mavlink_data_item_t;

// Thread-safe MAVLink data queue
typedef struct {
    mavlink_data_item_t items[MAVLINK_DATA_QUEUE_SIZE];
    volatile size_t head;  // Write index (producer)
    volatile size_t tail;  // Read index (consumer)
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;  // Signal when queue has items
    pthread_cond_t not_full;   // Signal when queue has space
} mavlink_data_queue_t;

// Queue functions
int mavlink_data_queue_init(mavlink_data_queue_t* queue);
void mavlink_data_queue_cleanup(mavlink_data_queue_t* queue);
int mavlink_data_queue_enqueue(mavlink_data_queue_t* queue, const mavlink_data_item_t* item);
int mavlink_data_queue_dequeue(mavlink_data_queue_t* queue, mavlink_data_item_t* item, int timeout_ms);
size_t mavlink_data_queue_size(mavlink_data_queue_t* queue);

#endif // MAVLINK_DATA_QUEUE_H

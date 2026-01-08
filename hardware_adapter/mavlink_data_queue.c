#define _POSIX_C_SOURCE 200809L
#include "mavlink_data_queue.h"
#include <string.h>
#include <errno.h>

// Initialize MAVLink data queue
int mavlink_data_queue_init(mavlink_data_queue_t* queue) {
    if (queue == NULL) {
        return -1;
    }
    queue->head = 0;
    queue->tail = 0;
    if (pthread_mutex_init(&queue->mutex, NULL) != 0) {
        return -1;
    }
    if (pthread_cond_init(&queue->not_empty, NULL) != 0) {
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }
    if (pthread_cond_init(&queue->not_full, NULL) != 0) {
        pthread_cond_destroy(&queue->not_empty);
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }
    return 0;
}

// Cleanup MAVLink data queue
void mavlink_data_queue_cleanup(mavlink_data_queue_t* queue) {
    if (queue == NULL) {
        return;
    }
    pthread_cond_destroy(&queue->not_full);
    pthread_cond_destroy(&queue->not_empty);
    pthread_mutex_destroy(&queue->mutex);
}

// Enqueue a MAVLink data item (thread-safe)
int mavlink_data_queue_enqueue(mavlink_data_queue_t* queue, const mavlink_data_item_t* item) {
    if (queue == NULL || item == NULL) {
        return -1;
    }
    
    pthread_mutex_lock(&queue->mutex);
    
    // Check if queue is full
    size_t next_head = (queue->head + 1) % MAVLINK_DATA_QUEUE_SIZE;
    if (next_head == queue->tail) {
        // Queue is full, wait for space
        pthread_cond_wait(&queue->not_full, &queue->mutex);
        // After wait, check again
        next_head = (queue->head + 1) % MAVLINK_DATA_QUEUE_SIZE;
        if (next_head == queue->tail) {
            pthread_mutex_unlock(&queue->mutex);
            return -1;  // Still full (shouldn't happen normally)
        }
    }
    
    // Enqueue item
    queue->items[queue->head] = *item;
    queue->head = next_head;
    
    // Signal that queue is not empty
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
    
    return 0;
}

// Dequeue a MAVLink data item (thread-safe)
// timeout_ms: 0 = non-blocking, >0 = blocking with timeout
int mavlink_data_queue_dequeue(mavlink_data_queue_t* queue, mavlink_data_item_t* item, int timeout_ms) {
    if (queue == NULL || item == NULL) {
        return -1;
    }
    
    pthread_mutex_lock(&queue->mutex);
    
    // Check if queue is empty
    if (queue->head == queue->tail) {
        if (timeout_ms <= 0) {
            // Non-blocking: return immediately
            pthread_mutex_unlock(&queue->mutex);
            return -1;
        }
        
        // Blocking: wait for data with timeout
        struct timespec timeout;
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += timeout_ms / 1000;
        timeout.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (timeout.tv_nsec >= 1000000000) {
            timeout.tv_sec++;
            timeout.tv_nsec -= 1000000000;
        }
        
        int result = pthread_cond_timedwait(&queue->not_empty, &queue->mutex, &timeout);
        if (result != 0 || queue->head == queue->tail) {
            // Timeout or still empty
            pthread_mutex_unlock(&queue->mutex);
            return -1;
        }
    }
    
    // Dequeue item
    *item = queue->items[queue->tail];
    queue->tail = (queue->tail + 1) % MAVLINK_DATA_QUEUE_SIZE;
    
    // Signal that queue is not full
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    
    return 0;
}

// Get current queue size (thread-safe)
size_t mavlink_data_queue_size(mavlink_data_queue_t* queue) {
    if (queue == NULL) {
        return 0;
    }
    
    pthread_mutex_lock(&queue->mutex);
    size_t size = (queue->head >= queue->tail) ? 
                  (queue->head - queue->tail) : 
                  (MAVLINK_DATA_QUEUE_SIZE - queue->tail + queue->head);
    pthread_mutex_unlock(&queue->mutex);
    
    return size;
}

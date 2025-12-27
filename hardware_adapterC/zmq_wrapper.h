#ifndef ZMQ_WRAPPER_H
#define ZMQ_WRAPPER_H

#include <zmq.h>
#include <stdint.h>
#include <stdbool.h>

// ZMQ context (global, initialized once)
void zmq_wrapper_init(void);
void zmq_wrapper_cleanup(void);
void* zmq_wrapper_get_context(void);

// Publisher functions
void* zmq_publisher_create(int port);
void zmq_publisher_destroy(void* socket);
int zmq_publisher_send(void* socket, const char* topic, const void* data, size_t data_len);

// Subscriber functions
void* zmq_subscriber_create(int port);
void zmq_subscriber_destroy(void* socket);
int zmq_subscriber_subscribe(void* socket, const char* topic);
int zmq_subscriber_receive(void* socket, char* topic_buffer, size_t topic_buffer_size, 
                          void* data_buffer, size_t data_buffer_size, int timeout_ms);

#endif // ZMQ_WRAPPER_H


#ifndef SINE_TEST_MODULE_H
#define SINE_TEST_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

// Default sine test parameters
#define SINE_TEST_DEFAULT_CMD_FREQUENCY      (100.0)    // Command generation frequency (Hz)
#define SINE_TEST_DEFAULT_DURATION           (30.0)    // Test duration in seconds (0 = infinite)
#define SINE_TEST_DEFAULT_VX_AMPLITUDE        (3.0)    // Velocity X amplitude (m/s)
#define SINE_TEST_DEFAULT_VX_FREQUENCY        (1.0)    // Velocity X frequency (Hz)
#define SINE_TEST_DEFAULT_VY_AMPLITUDE        (1.0)    // Velocity Y amplitude (m/s)
#define SINE_TEST_DEFAULT_VY_FREQUENCY        (0.5)    // Velocity Y frequency (Hz)
#define SINE_TEST_DEFAULT_VZ_AMPLITUDE        (0.5)    // Velocity Z amplitude (m/s)
#define SINE_TEST_DEFAULT_VZ_FREQUENCY        (0.3)    // Velocity Z frequency (Hz)
#define SINE_TEST_DEFAULT_YAWRATE_AMPLITUDE   (1.5)    // Yaw rate amplitude (rad/s)
#define SINE_TEST_DEFAULT_YAWRATE_FREQUENCY   (1.0)    // Yaw rate frequency (Hz)

// Sine test configuration structure
typedef struct {
    bool enabled;                    // Enable/disable sine test mode
    
    // Command generation parameters
    double cmd_frequency;            // Frequency of command generation (Hz)
    double duration;                 // Duration of test sequence (seconds, 0 = infinite)
    
    // Sine wave parameters for velocity X, Y, Z
    double vx_amplitude;             // Amplitude for velocity X (m/s)
    double vx_frequency;              // Frequency for velocity X (Hz)
    double vy_amplitude;             // Amplitude for velocity Y (m/s)
    double vy_frequency;              // Frequency for velocity Y (Hz)
    double vz_amplitude;             // Amplitude for velocity Z (m/s)
    double vz_frequency;              // Frequency for velocity Z (Hz)
    
    // Sine wave parameters for yaw rate
    double yaw_rate_amplitude;       // Amplitude for yaw rate (rad/s)
    double yaw_rate_frequency;        // Frequency for yaw rate (Hz)
    
    // Internal state (do not modify directly)
    void* adapter;                    // Hardware adapter instance (for direct command injection)
    void* mavlink_connection;        // MAVLink connection (extracted from adapter for direct access)
    bool running;                     // Test is running
    struct timespec start_time;       // Test start time
    int sent_count;                   // Number of commands sent
} sine_test_config_t;

// Initialize sine test module
// adapter: Pointer to hardware adapter instance (required for direct command injection)
// Returns 0 on success, -1 on error
int sine_test_init(sine_test_config_t* config, void* adapter);

// Start sine test (runs in current thread, blocks until duration expires or stopped)
// Returns 0 on success, -1 on error
int sine_test_run(sine_test_config_t* config);

// Stop sine test
void sine_test_stop(sine_test_config_t* config);

// Cleanup sine test module
void sine_test_cleanup(sine_test_config_t* config);

#endif // SINE_TEST_MODULE_H

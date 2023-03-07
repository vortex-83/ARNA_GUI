#include <cstddef>
#include <cstdint>
#include <cstdlib>  
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>

#include <iostream>
#include<atomic>

struct listener_args {
    int fd;
    std::atomic_int16_t* axis;
    std::atomic_int* connected;
};

struct axis_state {
    int16_t axis0;
    int16_t axis1;
    int16_t axis2;
};

enum joystick_status: int {
    joy_stat_connected = 1,
    joy_stat_disconnected = 0
};

class joystick_listener {
public:
    std::atomic_int16_t axis[3];
    std::atomic_int connected;
    int fd;
    pthread_t listener_pt;

    joystick_listener();
    ~joystick_listener();
    int get_axis_state(axis_state& ret);
    int connect();
    int disconnect();
    int status();
    
};

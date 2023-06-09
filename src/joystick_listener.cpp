#include "joystick_listener.hpp"

void* joystick_listener_thread(void* args) {
    listener_args l_args = *(listener_args*)args;
    free(args);
    int fd = l_args.fd;
    std::atomic_int16_t* axis = l_args.axis;
    
    js_event event;
    int ret = 1;

    while (1) {
        
    	/* TODO maybe make this better with poll, case statement, etc... */
    	ret = read(fd, &event, sizeof(event));

    	//std::cout << "type: " << (int)event.type << std::endl;
    	
    	if (ret < 0) {

    	    //read error
    	    l_args.connected->store(joy_stat_disconnected);
    	    return NULL;
    	}

    	if (event.type == 2) {
    	    if (event.number == 0) {
    		    axis[0].store(event.value);
    	    } else if (event.number == 1) {
    		    axis[1].store(event.value);
    	    } else if(event.number == 2) {
    		    axis[2].store(event.value);
    	    }
    	}
    }
}

joystick_listener::joystick_listener() {
    connected.store(joy_stat_disconnected);
    axis[0].store(0);
    axis[1].store(0);
    axis[2].store(0);
    fd = -1;
}

joystick_listener::~joystick_listener() {
    if (status() == joy_stat_connected) {
	   disconnect();
    }
}

int joystick_listener::get_axis_state(axis_state &ret) {
    if (connected.load() == joy_stat_connected) {
	    ret = {axis[0].load(), axis[1].load(), axis[2].load()};
      	return joy_stat_connected;
    } else {
        return joy_stat_disconnected;
    }
}

int joystick_listener::connect() {
    connected.store(joy_stat_disconnected);

    fd = open("/dev/input/js0", O_RDONLY);

    if (fd <= 0) { 
        return joy_stat_disconnected;
    }
    
    connected.store(joy_stat_connected);
    
    listener_args* l_args = new listener_args;
    l_args->fd = fd;
    l_args->axis = axis;
    l_args->connected = &connected;

    int pt_num = pthread_create(&listener_pt, NULL, joystick_listener_thread, (void*)l_args);

    if (connected.load() == joy_stat_connected) {
	    return joy_stat_connected;
    } else {
	    return joy_stat_disconnected;
    }
}

int joystick_listener::disconnect() {
    close(fd);
    pthread_join(listener_pt, NULL);

    if (connected.load() == joy_stat_connected) {
	    return joy_stat_connected;
    } else {
	    return joy_stat_disconnected;
    }
}

int joystick_listener::status() {
    int status = connected.load();

    if (fd <= 0 || status == joy_stat_disconnected) {
	    return joy_stat_disconnected;
    } else {
	    return joy_stat_connected;
    }
}

#include <bits/types/sigset_t.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <netdb.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>

#include "json.hpp"

#include <functional>
#include <algorithm>
#include <csignal>
#include <cstddef>
#include <cstdlib>
#include <mutex>
#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <atomic>
#include <ostream>
#include <sstream>
#include <list>
#include <cstdint>
#include <tuple>

namespace rosbridge_lib{

    enum listener_command: int32_t{
	lcom_shutdown = -1
    };

    enum listener_status: int32_t{
	lstat_connected = 1,
	lstat_starting = 0,
	lstat_disconnected = -1
    };

    enum send_return: int{
	seret_sent = 1,
	seret_not_sent = 0,
	seret_disconnected = -1
    };

    enum spin_return: int{
	spret_spun = 1,
	spret_not_spun = 0,
	spret_disconnected = -1
    };

    enum conn_return: int{
	cnret_succes = 1,
	cnret_no_socket = -1,
	cnret_no_dns = -2,
	cnret_no_conn = -3,
	cnret_no_pipe = -4
    };
    
    const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    const int default_send_buf_size = 16384;
    const int default_recv_buf_size = 16384;
    
    //
    // Listener Thread
    //
    struct listener_args{
	int socket_fd;
	int pipe_fd;
	char* recv_buf;
	int recv_buf_size;
	std::atomic_int32_t* status_code_;
	std::mutex* queue_mutex_;
	std::list<nlohmann::json>* msg_queue_;
	
    };

    void* listener_thread(void* args);

    //
    // Rosbridge Client Class
    //
    class rosbridge_client {
    public:

	//networking
	int port;
	std::string address;
	int socket_fd = -1;

	//synchronization
	std::mutex in_msg_queue_mutex;
	std::atomic_int32_t listener_status;
	pthread_t listener_pt;
	int listener_pipe_fd[2] = {-1};
  
	//msg queues
	std::list<nlohmann::json> in_msg_queue;
	std::list<nlohmann::json> out_msg_queue;

	//buffers
	char* send_buffer = NULL;
	int send_buffer_size = -1;
	char* recv_buffer = NULL;
	int recv_buffer_size = -1;

	//ROS vectors
	std::vector<std::tuple<std::string, std::function<void(nlohmann::json&)>>> subscribed_topics;
	std::vector<std::string> advertised_topics;

	rosbridge_client();
	~rosbridge_client();
	void cleanup();
	int connect(int port_arg, std::string address_arg);
	std::vector<std::string> get_advertised_topics();
	std::vector<std::string> get_subscribed_topics();
	int advertise(std::string topic, std::string type);
	int unadvertise(std::string topic);
	int publish(std::string topic, nlohmann::json msg);
	int subscribe(std::string topic, std::function<void(nlohmann::json&)> callback);
	int unsubscribe(std::string topic);
	int send_queue();
	int handle_msg(std::string topic, nlohmann::json &j);
	int spin_once();
    };

    //
    // Base64 Utilities
    //  
    std::vector<char> base64_decode(std::string_view encoded_string);

}


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
#include <sys/select.h>
#include <pthread.h>

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

    //
    // Listener Thread
    //
    struct listener_args{
	int socket_fd;
	int buffer_size;
	std::mutex* p_queue_mutex;
	std::list<nlohmann::json>* p_in_msg_queue;
	std::atomic_bool* p_shutdown_flag;
	std::atomic_bool* p_listening_flag;
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
  int socket_fd;

  //synchronization
  std::mutex queue_mutex;
  std::atomic_bool shutdown_flag;
  std::atomic_bool listening_flag;
  pthread_t listener_pt;
  
  //msg queues
  std::list<nlohmann::json> in_msg_queue;
  std::list<nlohmann::json> out_msg_queue;

  char send_buffer[16384];

  //ROS vectors
  std::vector<std::tuple<std::string, std::function<void(nlohmann::json&)>>> subscribed_topics;
  std::vector<std::string> advertised_topics;

  rosbridge_client();
  ~rosbridge_client();
  void cleanup();
  int rosbridge_connect(int port_arg, std::string address_arg);
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
 std::vector<char> base64_decode(std::string const& encoded_string);

}


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

const unsigned int buffer_size = 1<<20;

typedef nlohmann::json json;
typedef std::string string;
typedef unsigned char BYTE;


//
// Listener Thread
//
typedef struct{
  int socket_fd;
  std::mutex* p_queue_mutex;
  std::list<json>* p_in_msg_queue;
  std::atomic_bool* p_shutdown_flag;
} listener_args_struct;

void* listener_thread(void* args);

//
// Rosbridge Client Class
//
class rosbridge_client {
public:

  //networking
  int port;
  string address;
  int socket_fd;

  //synchronization
  std::mutex queue_mutex;
  std::atomic_bool shutdown_flag;
  pthread_t listener_pt;
  bool listener_start;

  //msg queues
  std::list<json> in_msg_queue;
  std::list<json> out_msg_queue;

  char send_buffer[buffer_size];

  //ROS vectors
  std::vector<std::tuple<string, std::function<void(json)>>> subscribed_topics;
  std::vector<string> advertised_topics;

  rosbridge_client();
  ~rosbridge_client();
  void cleanup();
  int rosbridge_connect(int port_arg, string address_arg);
  std::vector<string> get_advertised_topics();
  std::vector<string> get_subscribed_topics();
  int advertise(string topic, string type);
  int unadvertise(string topic);
  int publish(string topic, json msg);
  int subscribe(string topic, std::function<void(json)> callback);
  int unsubscribe(string topic, std::function<void(json)> callback);
  int send_queue();
  int handle_msg(string topic, json &j);
  int spin_once();
};

//
// Base64 Utilities
//
static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                        "abcdefghijklmnopqrstuvwxyz"
                                        "0123456789+/";

static inline bool is_base64(BYTE c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

std::vector<BYTE> base64_decode(std::string const& encoded_string);

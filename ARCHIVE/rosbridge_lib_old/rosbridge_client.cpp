#include "rosbridge_client.hpp"
#include <fcntl.h>
#include <sys/select.h>

void* listener_thread(void* args) {
  std::cout << "listener_start" << std::endl;

  //cast args
  listener_args_struct listener_args = *(listener_args_struct*)args;

  char read_buffer[buffer_size];
  
  //listener loop
  int ret;
  int bracket_count = 0;
  std::stringstream sstream;
  while(listener_args.p_shutdown_flag->load() == false){
    //read data from socket
    ret = read(listener_args.socket_fd, read_buffer, 2048);
    if( ret < 1){ std::cout << "disconnected" << std::endl; }
    else{
      //loop through read buffer
      for (int i = 0; i < ret; i++) {
	//add char to stringstream, there is definitly a more efficient way of doing this but this is easy
	sstream << read_buffer[i];
	//keep track of how many brackets deep we are
	if(read_buffer[i] == '{'){ bracket_count++; }
	else if(read_buffer[i] == '}'){ bracket_count--; }
	//if we are at 0 brackets deep then we are at the end of a json so push it
	if(bracket_count == 0){
	  //lock the mutex
	  string s = sstream.str();
	  listener_args.p_queue_mutex->lock();
	  listener_args.p_in_msg_queue->push_back(json::parse(s));
	  listener_args.p_queue_mutex->unlock();
      //clear sstream
      sstream.str(string());
	}
      }
    }
    usleep(5000);
  }
  std::cout << "listener shutdown" << std::endl;
  return nullptr;
}

  
rosbridge_client::rosbridge_client(){}

rosbridge_client::~rosbridge_client(){cleanup();}

  //tell listener thread to shutdown, close socket
void rosbridge_client::cleanup(){
    std::cout << "cleaning" << std::endl;
    shutdown_flag.store(true);
    if (socket_fd >= 0){ shutdown(socket_fd, SHUT_RDWR); close(socket_fd); }
    if (listener_start == 1)pthread_join(listener_pt, NULL);
    std::cout << "cleaned" << std::endl;
}

int rosbridge_client::rosbridge_connect(int port_arg, string address_arg) {
    //create socket
    port = port_arg;
    address = address_arg;

    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if( socket_fd < 0){}

    //dns lookup info
    struct addrinfo *dns_result, hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    
    //dns lookup
    int sys_result = getaddrinfo(address.c_str(), std::to_string(port).c_str(), &hints, &dns_result);
    if (sys_result != 0) {return -1;}

    //connect to server
    sys_result = connect(socket_fd, dns_result->ai_addr, dns_result->ai_addrlen);

    shutdown_flag.store(false);
    
    //listener thread args
    listener_args_struct l_args;
    l_args.p_queue_mutex = &queue_mutex;
    l_args.p_in_msg_queue = &in_msg_queue;
    l_args.p_shutdown_flag = &shutdown_flag;
    l_args.socket_fd = socket_fd;

    //start listener thread
    int listener_pt_num = pthread_create(&listener_pt, NULL, listener_thread, (void*)&l_args);
    listener_start = 1;
    
    //dont know why I need this
    usleep(20000);

    return 0;
}

std::vector<string> rosbridge_client::get_advertised_topics(){return advertised_topics; }

std::vector<string> rosbridge_client::get_subscribed_topics(){
    std::vector<string> tmp(subscribed_topics.size());
    for (std::tuple<string, std::function<void(json)>> i: subscribed_topics) {
      tmp.push_back(std::get<0>(i));
    }
    return tmp;
  }
  
int rosbridge_client::advertise(string topic, string type){
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);
    if(found == advertised_topics.end())
    {
      json j;
      j["op"] = "advertise";
      j["topic"] = topic;
      j["type"] = type;
      advertised_topics.push_back(topic);
      out_msg_queue.push_back(j);
      return 0;
    }
    else{ return -1;}   
  }

int rosbridge_client::unadvertise(string topic){
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);
    if(found == advertised_topics.end()){return -1;}
    else
    {
      json j;
      j["op"] = "unadvertise";
      j["topic"] = topic;
      out_msg_queue.push_back(j);
      advertised_topics.erase(found);
      return 0;
    }
  }
  
int rosbridge_client::publish(string topic, json msg){
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);
    if(found == advertised_topics.end()){return -1;}
    else{
      json j;
      j["op"] = "publish";
      j["topic"] = topic;
      j["msg"] = msg;
      out_msg_queue.push_back(j);
      return 0;
    }
  }

int rosbridge_client::subscribe(string topic, std::function<void(json)> callback){
    auto topic_match = [&topic](std::tuple<string, std::function<void(json)>> i){ return topic == std::get<0>(i);};
    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);
    if(found == subscribed_topics.end())
    {
      json j;
      j["op"] = "subscribe";
      j["topic"] = topic;
      out_msg_queue.push_back(j);
      subscribed_topics.push_back(std::tuple<string, std::function<void(json)>>(topic, callback));
      return 0;
    }
    else{return -1; }
  }

int rosbridge_client::unsubscribe(string topic, std::function<void(json)> callback){
    auto topic_match = [&topic](std::tuple<string, std::function<void(json)>> i){ return topic == std::get<0>(i);};
    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);
    if(found == subscribed_topics.end()){return -1;}
    else
    {
      json j;
      j["op"] = "unsubscribe";
      j["topic"] = topic;
      out_msg_queue.push_back(j);
      subscribed_topics.erase(found);
      return 0;
    }
  }

  //send all queued messages in out_message_queue
int rosbridge_client::send_queue(){
    for (auto const &j: out_msg_queue) {
      string j_string = j.dump();
      int j_length = j_string.size();
      memcpy(send_buffer, j_string.c_str(), j_length);
      
      int sent = 0;
      int ret;
      while(sent < j_length){
        ret = send(socket_fd, send_buffer, j_length, 0);
        if(ret < 0){std::cout << "send_queue error" << std::endl;}
        else{sent += ret;}
      }
    }
    while(!out_msg_queue.empty()){out_msg_queue.pop_front();};
    return 0;
  }

  //invoke a subscribed topics function
int rosbridge_client::handle_msg(string topic, json &j){
    auto topic_match = [&topic](std::tuple<string, std::function<void(json)>> i){ return topic == std::get<0>(i);};
    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);
    if(found == subscribed_topics.end()){return -1;}
    else
    {
      (std::get<1>(*found)(j));
      return 0;
    }
  }

  // handle queued messages
int rosbridge_client::spin_once() {
    queue_mutex.lock();
    while (!in_msg_queue.empty()) {
      json j = in_msg_queue.front();
      in_msg_queue.pop_front();
      string op = j["op"];
      if (op == "publish") {
        handle_msg(j["topic"], j);
      } else if (op == "service") { /*TODO implement service*/
      }
    }
    queue_mutex.unlock();
    return 0;
  }

std::vector<BYTE> base64_decode(std::string const& encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  BYTE char_array_4[4], char_array_3[3];
  std::vector<BYTE> ret;

  while (in_len-- && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_]; in_++;
    if (i ==4) {
      for (i = 0; i <4; i++)
        char_array_4[i] = base64_chars.find(char_array_4[i]);

      char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++)
          ret.push_back(char_array_3[i]);
      i = 0;
    }
  }

  if (i) {
    for (j = i; j <4; j++)
      char_array_4[j] = 0;

    for (j = 0; j <4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);

    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (j = 0; (j < i - 1); j++) ret.push_back(char_array_3[j]);
  }

  return ret;
}

//example
/*
void do_nothing(json j){
  string s = j["msg"]["data"].dump();
  int l = s.size();
  s = s.substr(1,l-2); //terribly innificient
  std::vector<BYTE> decoded = base64_decode(s); 
  cv::Mat img = cv::imdecode(decoded, 0);
  cv::imshow("image", img);
  cv::waitKey(80);
}

int main(){

  int port = 17647;
  string ip = "8.tcp.ngrok.io";
  rosbridge_client ros(port, ip);
  
  
  usleep(50000);

  ros.subscribe("/images/compressed", do_nothing);

  ros.send_queue();
  

  usleep(50000);

  cv::namedWindow( "image", cv::WINDOW_AUTOSIZE );
  
  while(true){
    ros.spin_once();
    usleep(5000);
  }
  
  ros.cleanup();
  
  std::cout << "shutdown succesfully" << std::endl;
  return 0;
}
*/

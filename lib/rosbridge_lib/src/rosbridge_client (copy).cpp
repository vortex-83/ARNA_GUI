#include "rosbridge_client.hpp"

using namespace rosbridge_lib;

typedef std::string string;
typedef nlohmann::json json;

void* start_listener_thread(void* args) {

    //cast args
    listener_args l_args = *(listener_args*)args;

    //begin listening
    l_args.p_listening_flag->store(true);
  
    char in_buf[l_args.buffer_size];
    int bytes_read = 0;
    int bracket_count = 0;
    std::stringstream sstream; /* TODO add hadling for sstream overflow */
  
    while(l_args.p_shutdown_flag->load() == false){
    
	bytes_read = read(l_args.socket_fd, in_buf, l_args.buffer_size);
	if(bytes_read < 1){
	    std::cout << "disconnected" << std::endl; /* TODO handle socket fail to reads*/
	    l_args.p_shutdown_flag->store(true);
	}
	else{
	    char cur;
	    for (int i = 0; i < bytes_read; i++) {
	      
		cur = in_buf[i];
		sstream << cur;
		if(cur  == '{'){ bracket_count++; }
		else if(cur == '}'){ bracket_count--; }
		if(bracket_count == 0){
		    //lock the mutex
		    string s = sstream.str();
		    l_args.p_queue_mutex->lock();
		    l_args.p_in_msg_queue->push_back(json::parse(s)); /* TODO handle json fail to parse */
		    l_args.p_queue_mutex->unlock();
		    //unlock and clear sstream
		    sstream.str(string());
		}
        
	    }
	}
      
    }

    //shutdown
    l_args.p_listening_flag->store(false);
    return nullptr;
}


rosbridge_client::rosbridge_client(){}

rosbridge_client::~rosbridge_client(){cleanup();}

//cleanup resources
void rosbridge_client::cleanup(){
    if (listening_flag.load() == true){
	shutdown_flag.store(true);
	pthread_join(listener_pt, NULL);
    }

    if (socket_fd >= 0){ shutdown(socket_fd, SHUT_RDWR); close(socket_fd); }
}

int rosbridge_client::rosbridge_connect(int port_arg, string address_arg) {
    //create socket
    port = port_arg;
    address = address_arg;

    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if( socket_fd < 0){ /* TODO handle failure to acquire socket */ }

    //dns lookup
    struct addrinfo *dns_result, hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    
    int sys_result = getaddrinfo(address.c_str(), std::to_string(port).c_str(), &hints, &dns_result);
    if (sys_result < 0) { std::cout << "DNS lookup failure" << std::endl; exit(-1); /* TODO handle dns lookup fail */ }

    //connect to server TODO make this have a timeout
    sys_result = connect(socket_fd, dns_result->ai_addr, dns_result->ai_addrlen);
    if (sys_result < 0) { std::cout << "connection fauilure" << std::endl; exit(-1); /* TODO handle failure to connect, might need to make this non blocking */ }
    
    //start listener thread
    shutdown_flag.store(false);
    
    listener_args l_args;
    l_args.p_queue_mutex = &queue_mutex;
    l_args.p_in_msg_queue = &in_msg_queue;
    l_args.p_shutdown_flag = &shutdown_flag;
    l_args.socket_fd = socket_fd;
    l_args.buffer_size = 16384;

    int listener_pt_num = pthread_create(&listener_pt, NULL, start_listener_thread, (void*)&l_args);
    
    //dont know why I need this
    usleep(20000);

    return 0;
}

std::vector<string> rosbridge_client::get_advertised_topics(){return advertised_topics; }

std::vector<string> rosbridge_client::get_subscribed_topics(){
    std::vector<string> tmp(subscribed_topics.size());
    for (std::tuple<string, std::function<void(json&)>> i: subscribed_topics) {
	tmp.push_back(std::get<0>(i));
    }
    return tmp;
}
  
int rosbridge_client::advertise(string topic, string type){
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);
    if(found == advertised_topics.end()){
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
    else{
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

int rosbridge_client::subscribe(string topic, std::function<void(json&)> callback){
    auto topic_match = [&topic](std::tuple<string, std::function<void(json&)>> i){ return topic == std::get<0>(i);};
    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);
    if(found == subscribed_topics.end()){
	json j;
	j["op"] = "subscribe";
	j["topic"] = topic;
	out_msg_queue.push_back(j);
	subscribed_topics.push_back(std::tuple<string, std::function<void(json&)>>(topic, callback));
	return 0;
    }
    else{return -1; }
}

int rosbridge_client::unsubscribe(string topic){
    auto topic_match = [&topic](std::tuple<string, std::function<void(json&)>> i){ return topic == std::get<0>(i);};
    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);
    if(found == subscribed_topics.end()){return -1;}
    else{
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
    if(listening_flag.load() == false){
	return -1;
	cleanup();
    }

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
    auto topic_match = [&topic](std::tuple<string, std::function<void(json&)>> i){ return topic == std::get<0>(i);};
    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);
    if(found == subscribed_topics.end()){return -1;}
    else{
	(std::get<1>(*found)(j));
	return 0;
    }
}

// handle queued messages
int rosbridge_client::spin_once() {
    if(listening_flag.load() == false){
	return -1;
	cleanup();
    }

    //TODO swap with empty queue instead of executing callbacks with this locked
    queue_mutex.lock();
    while (!in_msg_queue.empty()) {
	json j = in_msg_queue.front();
	in_msg_queue.pop_front();
	string op = j["op"];
	if (op == "publish") {
	    handle_msg(j["topic"], j);
	} else if (op == "service"){
	    /*TODO implement service*/
	}
    }
    queue_mutex.unlock();
    return 0;
}

static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::vector<char> rosbridge_lib::base64_decode(std::string const& encoded_string) {
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    char char_array_4[4], char_array_3[3];
    std::vector<char> ret;

    while (in_len-- && ( encoded_string[in_] != '=')) {
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



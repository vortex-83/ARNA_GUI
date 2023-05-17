#include "rosbridge_client.hpp"

typedef std::string string;
typedef nlohmann::json json;

void* start_listener_thread(void* args) {    
    
    //cast args
    rosbridge_lib::listener_args l_args = *(rosbridge_lib::listener_args*)args;
    free(args);
    char* recv_buf = l_args.recv_buf;
    
    //parsing
    int bytes_read = 0;
    int bracket_count = 0;
    std::stringstream sstream; /* TODO optimize this */

    //poll setup
    int poll_return;
    int nfds = 2;
    pollfd pfds[2];
    
    pollfd& sock_pfd = pfds[0];
    sock_pfd.fd = l_args.socket_fd;
    sock_pfd.events = POLLIN;
    
    pollfd& pipe_pfd = pfds[1];
    pipe_pfd.fd = l_args.pipe_fd;
    pipe_pfd.events = POLLIN;
    
    //shutdown utility lambda
    auto listener_shutdown = [stat_reg = l_args.status_code_] () {
		stat_reg->store(rosbridge_lib::listener_status::lstat_disconnected);
		pthread_exit(NULL);
    };
    
    //set status code to 1 to indicate listening
    l_args.status_code_->store(rosbridge_lib::listener_status::lstat_connected);
    
    while(1) {

		sock_pfd.revents = 0;
		pipe_pfd.revents = 0;
		
	        poll_return = poll(pfds, nfds, -1);
		
		//poll error
		if (poll_return < 0) {
		    listener_shutdown();
		}
		
		//socket has data
		if (sock_pfd.revents & POLLIN) {

		    /* TODO optimize this (goes with removing the sstream)*/
		    bytes_read = read(l_args.socket_fd, recv_buf, l_args.recv_buf_size);

		    if (bytes_read < 1) {

				//server disconnected or error
				listener_shutdown();
		    } else {

				//we recieved data
				char cur;

				for (int i = 0; i < bytes_read; i++) {
				    cur = recv_buf[i];
				    sstream << cur;

				    //update bracket count
				    if (cur  == '{') { bracket_count++; }
				    else if (cur == '}') { bracket_count--; }

				    //if json is completed add it to in_queue
				    if(bracket_count == 0) {
						string s = sstream.str();
						l_args.queue_mutex_->lock();
						l_args.msg_queue_->emplace_back(json::parse(s)); /* TODO handle json fail to parse */
						l_args.queue_mutex_->unlock();

						//clear sstream
						sstream.str(string());
				    }
				}
		    }
		}

		//socket error
		else if (sock_pfd.revents) {
		    listener_shutdown();
		}

		//command pipe has data
		if (pipe_pfd.revents & POLLIN) {
		    int32_t command;
		    bytes_read = read(l_args.pipe_fd, &command, 4);

		    if (command == rosbridge_lib::listener_command::lcom_shutdown) {
				//recieved command to disconnect
				l_args.status_code_->store(rosbridge_lib::listener_status::lstat_disconnected);
				pthread_exit(NULL);
		    }
		}

		//pipe error
		else if (pipe_pfd.revents) {
		    listener_shutdown();
		}	
    }
}


rosbridge_lib::rosbridge_client::rosbridge_client() {
    listener_status.store(rosbridge_lib::listener_status::lstat_disconnected);
    socket_fd = -1;
    listener_pipe_fd[0] = -1;
    recv_buffer = 0;
    send_buffer = 0;
}

rosbridge_lib::rosbridge_client::~rosbridge_client() { cleanup(); }

//cleanup resources
void rosbridge_lib::rosbridge_client::cleanup() {

    //join listener thread
    int32_t status = listener_status.load();

    if (status == rosbridge_lib::listener_status::lstat_connected ||
	    status == rosbridge_lib::listener_status::lstat_starting)
    {
		int32_t command = rosbridge_lib::listener_command::lcom_shutdown;
		write(listener_pipe_fd[1], &command, 4);
		pthread_join(listener_pt, NULL);
    }

    //shutdown socket
    if (socket_fd >= 0) {
		shutdown(socket_fd, SHUT_RDWR);
		close(socket_fd);
    }

    //close pipe
    if (listener_pipe_fd[0] >= 0) {
		close(listener_pipe_fd[0]);
		close(listener_pipe_fd[1]);
    }

    //free buffers
    if(recv_buffer) {
		free(recv_buffer);
    }

    if(send_buffer) {
		free(send_buffer);
    }
}

/* TODO maybe rename this */
int rosbridge_lib::rosbridge_client::connect(int port_arg, string address_arg) {

    //allocate buffers
    /* TODO have different buffer sizes */
    recv_buffer_size = default_recv_buf_size;
    send_buffer_size = default_send_buf_size;

    send_buffer = (char*)malloc(recv_buffer_size);
    recv_buffer = (char*)malloc(send_buffer_size);
    
    //create socket
    port = port_arg;
    address = address_arg;

    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_fd < 0) {
		return rosbridge_lib::conn_return::cnret_no_socket;
    }

    //dns lookup
    struct addrinfo *dns_result, hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    
    int sys_result = getaddrinfo(address.c_str(), std::to_string(port).c_str(), &hints, &dns_result);

    if (sys_result < 0) {
		return rosbridge_lib::conn_return::cnret_no_dns;
    }

    //connect to server
    /* TODO make this have a timeout */
    sys_result = ::connect(socket_fd, dns_result->ai_addr, dns_result->ai_addrlen);

    if (sys_result < 0) {
		return rosbridge_lib::conn_return::cnret_no_conn;
    }

    //create pipe
    sys_result = pipe(listener_pipe_fd);

    if (sys_result < 0){
		return rosbridge_lib::conn_return::cnret_no_pipe;
    }
    
    listener_args* l_args = new listener_args;
    l_args->socket_fd = socket_fd;
    l_args->pipe_fd = listener_pipe_fd[0];
    l_args->recv_buf = recv_buffer;
    l_args->recv_buf_size = recv_buffer_size;
    l_args->status_code_ = &listener_status;
    l_args->queue_mutex_ = &in_msg_queue_mutex;
    l_args->msg_queue_ = &in_msg_queue;

    //set status code to startup
    listener_status.store(rosbridge_lib::listener_status::lstat_starting);

    int listener_pt_num = pthread_create(&listener_pt, NULL, start_listener_thread, (void*)l_args);
    
    return rosbridge_lib::conn_return::cnret_succes;
}

std::vector<string> rosbridge_lib::rosbridge_client::get_advertised_topics() { return advertised_topics; }

std::vector<string> rosbridge_lib::rosbridge_client::get_subscribed_topics() {
    std::vector<string> tmp(subscribed_topics.size());

    for (std::tuple<string, std::function<void(json&)>> i: subscribed_topics) {
		tmp.push_back(std::get<0>(i));
    }

    return tmp;
}
  
int rosbridge_lib::rosbridge_client::advertise(string topic, string type) {
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);

    if (found == advertised_topics.end()) {
		json j;

		j["op"] = "advertise";
		j["topic"] = topic;
		j["type"] = type;

		advertised_topics.push_back(topic);
		out_msg_queue.push_back(j);

		return 0;
    } else {
    	return -1;
    }
    
}

int rosbridge_lib::rosbridge_client::unadvertise(string topic) {
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);

    if (found == advertised_topics.end()) {
    	return -1;
    } else {
		json j;

		j["op"] = "unadvertise";
		j["topic"] = topic;

		out_msg_queue.push_back(j);
		advertised_topics.erase(found);

		return 0;
    }
}
  
int rosbridge_lib::rosbridge_client::publish(string topic, json msg) {
    auto found = std::find(advertised_topics.begin(), advertised_topics.end(), topic);

    if (found == advertised_topics.end()) {
    	return -1;
    } else {
		json j;

		j["op"] = "publish";
		j["topic"] = topic;
		j["msg"] = msg;

		out_msg_queue.push_back(j);

		return 0;
    }
}

int rosbridge_lib::rosbridge_client::subscribe(string topic, std::function<void(json&)> callback) {
    auto topic_match = [&topic] (std::tuple<string, std::function<void(json&)>> i) { 
    	return topic == std::get<0>(i);
    };

    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);

    if (found == subscribed_topics.end()) {
		json j;

		j["op"] = "subscribe";
		j["topic"] = topic;

		out_msg_queue.push_back(j);
		subscribed_topics.push_back(std::tuple<string, std::function<void(json&)>>(topic, callback));

		return 0;
    } else {
    	return -1; 
    }
}

int rosbridge_lib::rosbridge_client::unsubscribe(string topic) {
    auto topic_match = [&topic] (std::tuple<string, std::function<void(json&)>> i) { 
    	return topic == std::get<0>(i);
    };

    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);

    if (found == subscribed_topics.end()) {
    	return -1;
    } else {
		json j;

		j["op"] = "unsubscribe";
		j["topic"] = topic;

		out_msg_queue.push_back(j);
		subscribed_topics.erase(found);

		return 0;
    }
}

// send all queued messages in out_message_queue
/* TODO maybe make this asynchronous */
int rosbridge_lib::rosbridge_client::send_queue() {
    int32_t status = listener_status.load();

    if (status == rosbridge_lib::listener_status::lstat_disconnected) {
		return rosbridge_lib::send_return::seret_disconnected;
		cleanup();
    } else if (status == rosbridge_lib::listener_status::lstat_starting) {
		return rosbridge_lib::send_return::seret_not_sent;
    }

    for (auto const &j: out_msg_queue) {
		string j_string = j.dump();
		int j_length = j_string.size();

		memcpy(send_buffer, j_string.c_str(), j_length);
	      
		int sent = 0;
		int ret;

		while(sent < j_length) {
		    ret = send(socket_fd, send_buffer, j_length, 0);

		    if (ret < 0) {
		    	std::cout << "send_queue error" << std::endl;
		    } else {
		    	sent += ret;
		    }
		}
    }

    while (!out_msg_queue.empty()) {
    	out_msg_queue.pop_front();
    }

    return rosbridge_lib::send_return::seret_sent;
}

// invoke a subscribed topics function
int rosbridge_lib::rosbridge_client::handle_msg(string topic, json &j) {
    auto topic_match =  [&topic] (std::tuple<string, std::function<void(json&)>> i) {
    	return topic == std::get<0>(i);
    };

    auto found = std::find_if(subscribed_topics.begin(), subscribed_topics.end(), topic_match);

    if (found == subscribed_topics.end()) {
    	return -1;
    } else {
		(std::get<1>(*found)(j));

		return 0;
    }
}

// handle queued messages
/* Handle failed handle_msg calls */
int rosbridge_lib::rosbridge_client::spin_once() {
    	int32_t status = listener_status.load();

    if (status == rosbridge_lib::listener_status::lstat_disconnected) {
		return rosbridge_lib::spin_return::spret_disconnected;
		cleanup();
    } else if (status == rosbridge_lib::listener_status::lstat_starting) {
		return rosbridge_lib::spin_return::spret_not_spun;
    }

    std::list<nlohmann::json> tmp_queue;
    in_msg_queue_mutex.lock();
    std::swap(tmp_queue, in_msg_queue);
    in_msg_queue_mutex.unlock();

    while (!tmp_queue.empty()) {
		json j = tmp_queue.front();
		tmp_queue.pop_front();
		string op = j["op"];

		if (op == "publish") {
		    handle_msg(j["topic"], j);
		} else if (op == "service"){
		    /*TODO implement services*/
		}
    }
    
    return rosbridge_lib::spin_return::spret_spun;
}

// credit to stack overflow
/* TODO find faster implemntation with LUT */
std::vector<char> rosbridge_lib::base64_decode(std::string_view encoded_string) {
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    char char_array_4[4], char_array_3[3];
    std::vector<char> ret;

    while (in_len-- && (encoded_string[in_] != '=')) {
		char_array_4[i++] = encoded_string[in_]; in_++;
		if (i == 4) {
		    for (i = 0; i < 4; i++)
			char_array_4[i] = rosbridge_lib::base64_chars.find(char_array_4[i]);

		    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
		    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
		    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

		    for (i = 0; i < 3; i++)
			ret.push_back(char_array_3[i]);
		    i = 0;
		}
    }

    if (i) {
		for (j = i; j < 4; j++)
		    char_array_4[j] = 0;

		for (j = 0; j < 4; j++)
		    char_array_4[j] = base64_chars.find(char_array_4[j]);

		char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
		char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
		char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

		for (j = 0; j < i - 1; j++)
			ret.push_back(char_array_3[j]);
    }

    return ret;
}


#include "ARNA_theora.hpp"
#include <opencv2/core/mat.hpp>
#include <string>

typedef std::string string;
typedef nlohmann::json json;

//GTK widgets
GtkWidget *window;
GtkWidget *layout;
GdkPixbuf *pbp;

GtkWidget *left_button;
GtkWidget *up_button;
GtkWidget *right_button;
GtkWidget *down_button;
GtkWidget *clockwise_button;
GtkWidget *cclockwise_button;

GtkWidget *connect_button;
GtkWidget *connect_add_entry;
GtkWidget *connect_port_entry;

std::vector<image_group> image_groups;

// lazy
char move_vec[3] = {0};

//networking
rosbridge_lib::rosbridge_client *ros;
int connected = -1;


//
// generic image update
//
//updates GtkImage with data from cv::Mat
void update_img(cv::Mat& mat, GtkWidget* target) {
    if(!mat.empty()){
	pbp = gdk_pixbuf_new_from_data(mat.data, GDK_COLORSPACE_RGB, false, 8, mat.cols, mat.rows, mat.cols * 3, NULL, NULL);
	gtk_image_set_from_pixbuf(GTK_IMAGE(target), pbp);
    }
}

//
// theora image decode
//
// TODO optimize this with passing the json as a reference
// remember to free data in packet
void theora_json_to_oggpacket(json& j, ogg_packet &ogg)
{
    /* TODO look at optimizing this */
    string s = j["msg"]["data"].dump();
    //std::string_view data = s;
    std::string_view sv = std::string_view(s).substr(1,s.size()-2);
    std::vector<char> data = rosbridge_lib::base64_decode(sv); /* TODO optimize this, too many copies */
    int bytes = sv.size();

    ogg.bytes      = bytes;
    ogg.b_o_s      = j["msg"]["b_o_s"];
    ogg.e_o_s      = j["msg"]["e_o_s"];
    ogg.granulepos = j["msg"]["granulepos"];
    ogg.packetno   = j["msg"]["packetno"];
    ogg.packet = new unsigned char[ogg.bytes]; //caller responsible for freeing this
    memcpy(ogg.packet, data.data(), ogg.bytes);
}

//copies decoded contents of oggpacket to target, returns 1 if new frame, 0 if no new frame, -1 if error
int theora_oggpacket_to_cvmat(theora_context& decode, ogg_packet& oggpacket, cv::Mat& target)
{
    // Beginning of logical stream flag means we're getting new headers
    if (oggpacket.b_o_s == 1) {
	decode.reset();
    }

    // Decode header packets until we get the first video packet
    if (decode.received_header_ == false) {
	int rval = th_decode_headerin(&decode.header_info_, &decode.header_comment_, &decode.setup_info_, &oggpacket);
	switch (rval) {
	case 0:
	    // We've received the full header; this is the first video packet.
	    decode.decoding_context_ = th_decode_alloc(&decode.header_info_, decode.setup_info_);
	    if (!decode.decoding_context_) {
		//std::cout <<  "[theora] Decoding parameters were invalid" << std::endl;
		return -1;
	    }
	    decode.received_header_ = true;
	    decode.update_pp_level();
	    break; // Continue on the video decoding
	case TH_EFAULT:
	    //RCLCPP_WARN(logger_, "[theora] EFAULT when processing header packet");
	    return -1;
	case TH_EBADHEADER:
	    //RCLCPP_WARN(logger_, "[theora] Bad header packet");
	    return -1;
	case TH_EVERSION:
	    //RCLCPP_WARN(logger_, "[theora] Header packet not decodable with this version of libtheora");
	    return -1;
	case TH_ENOTFORMAT:
	    //RCLCPP_WARN(logger_, "[theora] Packet was not a Theora header");
	    return -1;
	default:
	    // If rval > 0, we successfully received a header packet.
	    if (rval < 0)
		//RCLCPP_WARN(logger_, "[theora] Error code %d when processing header packet", rval);
		return -1;
	}
    }

    //wait for keyframe
    decode.received_keyframe_ = decode.received_keyframe_ || (th_packet_iskeyframe(&oggpacket) == 1);
    if (!decode.received_keyframe_) { return 0; }

    //decode video packet
    int rval = th_decode_packetin(decode.decoding_context_, &oggpacket, NULL);
    switch (rval) {
    case 0:
	break;
    case TH_DUPFRAME:
	//RCLCPP_DEBUG(logger_, "[theora] Got a duplicate frame");
	//if (!mat_img.empty()) {
	//    show_mat_img(); //TODO stop using show_mat_img
	//}
	return 0;
    case TH_EFAULT:
	//RCLCPP_WARN(logger_, "[theora] EFAULT processing video packet");
	return -1;
    case TH_EBADPACKET:
	//std::cout <<  "[theora] Packet does not contain encoded video data") << std::endl;
	return -1;
    case TH_EIMPL:
	//RCLCPP_WARN(logger_, "[theora] The video data uses bitstream features not supported by this version of libtheora");
	return -1;
    default:
	//RCLCPP_WARN(logger_, "[theora] Error code %d when decoding video packet", rval);
	return -1;
    }

    /* TODO optimize this!!!!! */
    //decode new frame
    th_ycbcr_buffer ycbcr_buffer;
    th_decode_ycbcr_out(decode.decoding_context_, ycbcr_buffer);
  
    // Wrap YCbCr channel data into OpenCV format
    th_img_plane &y_plane = ycbcr_buffer[0], &cb_plane = ycbcr_buffer[1], &cr_plane = ycbcr_buffer[2];
    cv::Mat y(y_plane.height, y_plane.width, CV_8UC1, y_plane.data, y_plane.stride);
    cv::Mat cb_sub(cb_plane.height, cb_plane.width, CV_8UC1, cb_plane.data, cb_plane.stride);
    cv::Mat cr_sub(cr_plane.height, cr_plane.width, CV_8UC1, cr_plane.data, cr_plane.stride);

    // Upsample chroma channels
    cv::Mat cb, cr;
    cv::pyrUp(cb_sub, cb);
    cv::pyrUp(cr_sub, cr);

    // Merge into interleaved image. Note OpenCV uses YCrCb, so we swap the chroma channels.
    cv::Mat ycrcb, channels[] = {y, cr, cb};
    cv::merge(channels, 3, ycrcb);

    // Convert to BGR color
    cv::Mat bgr, bgr_padded;
    cv::cvtColor(ycrcb, bgr_padded, cv::COLOR_YCrCb2RGB);
    // Pull out original (non-padded) image region
    bgr = bgr_padded(cv::Rect(decode.header_info_.pic_x, decode.header_info_.pic_y, decode.header_info_.pic_width, decode.header_info_.pic_height));

    //optimize this!!!!
    //this should deep copy
    swap(target, bgr);
    return 1;
}

//TODO make this generic, not just theora
void theora_image_callback(theora_stream* stream_p, json j) {
    theora_stream& stream_info = *stream_p;
    ogg_packet oggpacket;
    
    theora_json_to_oggpacket(j, oggpacket);
    int new_frame = theora_oggpacket_to_cvmat(stream_info.decode_function_context, oggpacket, stream_info.buffer);
    if(new_frame == 1){
	update_img(stream_info.buffer, stream_info.target);
    }
    
    //free oggpacket data
    free(oggpacket.packet);
}

//
// movement
//
int publish_move_vec() {
    //publish joystick data
    json j;
    j["axes"] = {move_vec[0]*150, move_vec[1]*150, move_vec[2]*150};
    ros->publish("/phy_joy_topic", j);
    return ros->send_queue();
}

void press_Button(GtkWidget *widget, gpointer data) {
    if (widget == left_button){ move_vec[0] = -1; }
    else if(widget == right_button){ move_vec[0] = 1; }
    else if(widget == up_button){ move_vec[1] = 1;}
    else if(widget == down_button){ move_vec[1] = -1; }
    else if(widget == cclockwise_button){ move_vec[2] = -1; }
    else if(widget == clockwise_button){ move_vec[2] = 1; }
    else{}

    connected = publish_move_vec();
}

void release_Button(GtkWidget *widget, gpointer data) {
    move_vec[0] = 0;
    move_vec[1] = 0;
    move_vec[2] = 0;

    connected = publish_move_vec();
}


//
// ROS
//
gint spin_ROS(gpointer data) {
    connected = ros->spin_once();
    return 1;
}

gint setup_ROS(gpointer data) {
    
    //parse entries
    char* port_str = (char*)gtk_entry_get_text(GTK_ENTRY(connect_port_entry));
    char* p;
    int port = strtol ( port_str, &p, 10);
    if ( * p != 0 ) {std::cout << "invalid port" << std::endl;}

    string address = string((char*)gtk_entry_get_text(GTK_ENTRY(connect_add_entry)));

    //startup rosbridge_client
    ros = new rosbridge_lib::rosbridge_client();
    int result = ros->connect(port, address);
    if(result < 0) {
	ros->cleanup();
	connected = -1;
	std::cout << "failed to connect" << std::endl;
	return 0;
    }

    connected = 0;
    std::cout << "connected!" << std::endl;

   
    //subscribe to video topic
    ros->advertise("/phy_joy_topic", "sensor_msgs/Joy");
    connected = ros->send_queue() >= 0;

    std::cout << "connection status" << connected << std::endl;
    
    
    //add network read to event loop
    /* TODO do this with a pipe and gtk polling instead of just spinning every 3ms */
    g_timeout_add (3, spin_ROS, NULL);
    
    return 1;
}

gint connect_stream(GtkWidget* button, gpointer data) {

    if(!connected){ return 0; }

    image_group* ig = nullptr;
    for( int i = 0; i < image_groups.size(); i++){
	if(image_groups[i].button == button){
	    ig = &image_groups[i];
	}
    }
    if (ig == nullptr){return 0;}
    
    string new_topic = string((char*)gtk_entry_get_text(GTK_ENTRY(ig->entry))); //get topic from entry

    if(ig->stream_ == nullptr){ //image group does not have theora stream, create one
	std::cout << "creating stream" << std::endl;
	theora_stream* stream_p = new theora_stream();
	ig->stream_ = stream_p;
	ig->stream_->decode_function_context.reset();
	ig->stream_->target = ig->image;
    }
    else{ //image group does have stream so unsub and free old stream
	std::cout << "reseting stream" << std::endl;
	ros->unsubscribe(ig->topic);
	ig->stream_->decode_function_context.reset();
    }

    ig->topic = new_topic;
    
    //bind new callback
    std::function<void(json&)> binded = [stream_pointer = ig->stream_](json& j){
	theora_image_callback(stream_pointer, j);
    };

    //subscribe to new topic
    ros->subscribe(new_topic, binded);
    ros->send_queue();
    return 1;    
}

//
// GTK setup
//
void quit_app(GtkWidget *widget, gpointer data) {
    if(connected) {ros->cleanup(); }
    gtk_main_quit();
}

void setup_gtk(int argc, char** argv) {

    //init
    gtk_init(&argc, &argv);

    //create window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), 1700, 580);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);

    //add layout
    layout = gtk_layout_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(window), layout);
    gtk_widget_show(layout);

    auto register_image_group_helper = [](image_group& ig,string default_topic, int x, int y){

	ig.image = gtk_image_new();
	gtk_layout_put(GTK_LAYOUT(layout), ig.image, 0 + x,0 + y);

	ig.button = gtk_button_new_with_label("Begin Stream");
	gtk_layout_put(GTK_LAYOUT(layout), ig.button, 50+x, 500+y);
	gtk_widget_set_size_request(ig.button, 100, 30);
	g_signal_connect(G_OBJECT(ig.button), "clicked", G_CALLBACK(connect_stream), NULL);

	ig.entry = gtk_entry_new();
	gtk_layout_put(GTK_LAYOUT(layout), ig.entry, 155+x, 500+y);
	gtk_entry_set_width_chars(GTK_ENTRY(ig.entry), 30);
	gtk_entry_set_text(GTK_ENTRY(ig.entry), default_topic.c_str());
    };

    image_groups.emplace_back();
    register_image_group_helper(image_groups[0], "/cam0/image_raw/theora", 0,0);

    image_groups.emplace_back();
    register_image_group_helper(image_groups[1], "/cam1/image_raw/theora", 1100, 0);

    //add connection button and entries
    connect_button = gtk_button_new_with_label("Connect");
    gtk_layout_put(GTK_LAYOUT(layout), connect_button, 700, 50);
    gtk_widget_set_size_request(connect_button, 80, 30);
    g_signal_connect(G_OBJECT(connect_button), "clicked", G_CALLBACK(setup_ROS), NULL);

    connect_add_entry = gtk_entry_new();
    gtk_layout_put(GTK_LAYOUT(layout), connect_add_entry, 790, 50);
    gtk_entry_set_width_chars(GTK_ENTRY(connect_add_entry), 20);
    gtk_entry_set_text(GTK_ENTRY(connect_add_entry), "localhost");

    connect_port_entry = gtk_entry_new();
    gtk_layout_put(GTK_LAYOUT(layout), connect_port_entry, 975, 50);
    gtk_entry_set_width_chars(GTK_ENTRY(connect_port_entry), 10);
    gtk_entry_set_text(GTK_ENTRY(connect_port_entry), "9090");



    //add move buttons + signals
    auto register_move_button_helper = [](GtkWidget*& target, string text, int x, int y) {
	target = gtk_button_new_with_label(text.c_str());
	gtk_layout_put(GTK_LAYOUT(layout), target, x, y);
	gtk_widget_set_size_request(target, 80, 35);
	g_signal_connect(G_OBJECT(target), "pressed", G_CALLBACK(press_Button), NULL);
	g_signal_connect(G_OBJECT(target), "released", G_CALLBACK(release_Button), NULL);
    };
    
    register_move_button_helper(left_button, "left", 700, 300);
    register_move_button_helper(right_button, "right", 900, 300);
    register_move_button_helper(up_button, "up", 800, 250);
    register_move_button_helper(down_button, "down", 800, 350);
    register_move_button_helper(clockwise_button, "clockwise", 900, 250);
    register_move_button_helper(cclockwise_button, "cclockwise", 700, 250);

    //add exit signal
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(quit_app), NULL);

    //show window
    gtk_widget_show_all(window);
}

//
// main
//
int main(int argc, char **argv) {

    setup_gtk(argc, argv);
  
    gtk_main();

    return 0;
}

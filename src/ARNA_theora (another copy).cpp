#include <gtkmm-3.0/gtkmm.h>
#include <gdkmm-3.0/gdkmm.h>

#include "rosbridge_client.hpp"

#include <ogg/ogg.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

#include <string_view>
#include <vector>

typedef std::string string;
typedef nlohmann::json json;

//GTK widgets
GtkWidget *window;
GtkWidget *layout;
GtkWidget *image;
GdkPixbuf *pbp;
cv::Mat mat_img;

GtkWidget *left_button;
GtkWidget *up_button;
GtkWidget *right_button;
GtkWidget *down_button;
GtkWidget *clockwise_button;
GtkWidget *cclockwise_button;

GtkWidget *connect_button;
GtkWidget *connect_add_entry;
GtkWidget *connect_port_entry;

// lazy
char move_vec[3] = {0};

//networking
rosbridge_lib::rosbridge_client *ros;
int connected = -1;

//
// image decode
//
void msgToOggPacket(json j, ogg_packet &ogg)
{
  string s = j["msg"]["data"].dump();
  //std::string_view data = s;
  s = s.substr(1,s.size()-2); //TODO replace this with a string_view
  std::vector<char> data = rosbridge_lib::base64_decode(s);
  int bytes = s.size();

  //std::cout << j << std::endl;
  
  ogg.bytes      = bytes;
  ogg.b_o_s      = j["msg"]["b_o_s"];
  ogg.e_o_s      = j["msg"]["e_o_s"];
  ogg.granulepos = j["msg"]["granulepos"];
  ogg.packetno   = j["msg"]["packetno"];
  ogg.packet = new unsigned char[ogg.bytes]; //caller responsible for freeing this
  memcpy(ogg.packet, data.data(), ogg.bytes);

  //std::cout << "bytes: " << ogg.bytes << std::endl;
  //std::cout << "eos: " << ogg.e_o_s << std::endl;
  //std::cout << "bos: " << ogg.b_o_s << std::endl;
  //std::cout << "granulepos: " << ogg.granulepos << std::endl;
  //std::cout << "packetno: " << ogg.packetno << std::endl;
}

void show_mat_img() {
  if(!mat_img.empty()){
	pbp = gdk_pixbuf_new_from_data(mat_img.data, GDK_COLORSPACE_RGB, false, 8, mat_img.cols, mat_img.rows, mat_img.cols * 3, NULL, NULL);
	gtk_image_set_from_pixbuf(GTK_IMAGE(image), pbp);
    }
}

////ripped from github, fix it later////

int pplevel_; // Post-processing level
bool received_header_;
bool received_keyframe_;
th_dec_ctx* decoding_context_;
th_info header_info_;
th_comment header_comment_;
th_setup_info* setup_info_;

int updatePostProcessingLevel(int level)
{
  int pplevel_max;
  int err = th_decode_ctl(decoding_context_, TH_DECCTL_GET_PPLEVEL_MAX, &pplevel_max, sizeof(int));
  if (err) {
      //RCLCPP_WARN(logger_, "Failed to get maximum post-processing level, error code %d", err);
  } else if (level > pplevel_max) {
      //RCLCPP_WARN(logger_, "Post-processing level %d is above the maximum, clamping to %d", level, pplevel_max);
    level = pplevel_max;
  }

  err = th_decode_ctl(decoding_context_, TH_DECCTL_SET_PPLEVEL, &level, sizeof(int));
  if (err) {
      //RCLCPP_ERROR(logger_, "Failed to set post-processing level, error code %d", err);
    return pplevel_; // old value
  }
  return level;
}

void handle_theora_msg(ogg_packet & oggpacket)
{
  /// @todo Break this function into pieces
  std::unique_ptr<unsigned char[]> packet_guard(oggpacket.packet); // Make sure packet memory gets deleted

  // Beginning of logical stream flag means we're getting new headers
  if (oggpacket.b_o_s == 1) {
    // Clear all state, everything we knew is wrong
    received_header_ = false;
    received_keyframe_ = false;
    if (decoding_context_) {
      th_decode_free(decoding_context_);
      decoding_context_ = NULL;
    }
    th_setup_free(setup_info_);
    setup_info_ = NULL;
    th_info_clear(&header_info_);
    th_info_init(&header_info_);
    th_comment_clear(&header_comment_);
    th_comment_init(&header_comment_);
    //latest_image_.reset();
  }

  // Decode header packets until we get the first video packet
  if (received_header_ == false) {
    int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    switch (rval) {
      case 0:
        // We've received the full header; this is the first video packet.
        decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
        if (!decoding_context_) {
	    std::cout <<  "[theora] Decoding parameters were invalid" << std::endl;
          return;
        }
        received_header_ = true;
        pplevel_ = updatePostProcessingLevel(pplevel_);
        break; // Continue on the video decoding
      case TH_EFAULT:
	  //RCLCPP_WARN(logger_, "[theora] EFAULT when processing header packet");
        return;
      case TH_EBADHEADER:
	  //RCLCPP_WARN(logger_, "[theora] Bad header packet");
        return;
      case TH_EVERSION:
	  //RCLCPP_WARN(logger_, "[theora] Header packet not decodable with this version of libtheora");
        return;
      case TH_ENOTFORMAT:
	  //RCLCPP_WARN(logger_, "[theora] Packet was not a Theora header");
        return;
      default:
        // If rval > 0, we successfully received a header packet.
        if (rval < 0)
	    //RCLCPP_WARN(logger_, "[theora] Error code %d when processing header packet", rval);
        return;
    }
  }

  // Wait for a keyframe if we haven't received one yet - delta frames are useless to us in that case
  received_keyframe_ = received_keyframe_ || (th_packet_iskeyframe(&oggpacket) == 1);
  if (!received_keyframe_)
    return;

  // We have a video packet we can handle, let's decode it
  int rval = th_decode_packetin(decoding_context_, &oggpacket, NULL);
  switch (rval) {
  case 0:
      break; // Yay, we got a frame. Carry on below.
  case TH_DUPFRAME:
      // Video data hasn't changed, so we update the timestamp and reuse the last received frame.
      //RCLCPP_DEBUG(logger_, "[theora] Got a duplicate frame");
      if (!mat_img.empty()) {
	  show_mat_img();
      }
      return;
  case TH_EFAULT:
      //RCLCPP_WARN(logger_, "[theora] EFAULT processing video packet");
      return;
  case TH_EBADPACKET:
      //std::cout <<  "[theora] Packet does not contain encoded video data") << std::endl;
      return;
  case TH_EIMPL:
      //RCLCPP_WARN(logger_, "[theora] The video data uses bitstream features not supported by this version of libtheora");
      return;
  default:
      //RCLCPP_WARN(logger_, "[theora] Error code %d when decoding video packet", rval);
      return;
  }

  // We have a new decoded frame available
  th_ycbcr_buffer ycbcr_buffer;
  th_decode_ycbcr_out(decoding_context_, ycbcr_buffer);

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
  cv::cvtColor(ycrcb, bgr_padded, cv::COLOR_YCrCb2BGR);
  // Pull out original (non-padded) image region
  bgr = bgr_padded(cv::Rect(header_info_.pic_x, header_info_.pic_y, header_info_.pic_width, header_info_.pic_height));
  mat_img = bgr;
  show_mat_img();
}

void image_callback(json j) {
    
    ogg_packet oggpacket;
    msgToOggPacket(j, oggpacket);
    handle_theora_msg(oggpacket);
    
}

//
// movement
//
int publish_move_vec() {
    //publish joystick data
    json j;
    j["axes"] = {move_vec[0]*150, move_vec[1]*150, move_vec[2]*150};
    //axis_list = [2*axis_state['x'], -2*axis_state['y'], axis_state['z']]
    //{"op": "publish", "msg": {"axes": axis_list}, "topic": "phy_joy_topic"}
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

void setup_ROS(GtkWidget* widget, gpointer data) {
    
    //parse entries
    char* port_str = (char*)gtk_entry_get_text(GTK_ENTRY(connect_port_entry));
    char* p;
    int port = strtol ( port_str, &p, 10);
    if ( * p != 0 ) {std::cout << "invalid port" << std::endl;}

    string address = string((char*)gtk_entry_get_text(GTK_ENTRY(connect_add_entry)));

    //startup rosbridge_client
    ros = new rosbridge_lib::rosbridge_client();
    int result = ros->rosbridge_connect(port, address);
    if(result < 0) {
	ros->cleanup();
	connected = -1;
	std::cout << "failed to connect" << std::endl;
	return;
    }

    connected = 0;
    std::cout << "connected!" << std::endl;

    //subscribe to video topic
    ros->subscribe("/camera/color/image_raw/theora", image_callback);
    ros->advertise("/phy_joy_topic", "sensor_msgs/Joy");
    ros->send_queue();

    //add network read to event loop
    g_timeout_add (3, spin_ROS, NULL);
}

//
// GTK setup
//
void quit_app(GtkWidget *widget, gpointer data) {
    if(connected == 0) {ros->cleanup(); }
    gtk_main_quit();
}

void setup_gtk(int argc, char** argv) {

    //init
    gtk_init(&argc, &argv);

    //create window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), 1100, 480);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);

    //add layout
    layout = gtk_layout_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(window), layout);
    gtk_widget_show(layout);

    //add image for displaying video stream
    image = gtk_image_new();
    gtk_layout_put(GTK_LAYOUT(layout), image, 0,0);

    //add connection button and entries
    connect_button = gtk_button_new_with_label("Connect");
    gtk_layout_put(GTK_LAYOUT(layout), connect_button, 700, 50);
    gtk_widget_set_size_request(connect_button, 80, 30);
    g_signal_connect(G_OBJECT(connect_button), "clicked", G_CALLBACK(setup_ROS), NULL);

    connect_add_entry = gtk_entry_new();
    gtk_layout_put(GTK_LAYOUT(layout), connect_add_entry, 790, 50);
    gtk_entry_set_width_chars(GTK_ENTRY(connect_add_entry), 20);

    connect_port_entry = gtk_entry_new();
    gtk_layout_put(GTK_LAYOUT(layout), connect_port_entry, 975, 50);
    gtk_entry_set_width_chars(GTK_ENTRY(connect_port_entry), 10);

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

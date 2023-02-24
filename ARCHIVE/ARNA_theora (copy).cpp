#include <gtkmm-3.0/gtkmm.h>
#include <gdkmm-3.0/gdkmm.h>

#include "rosbridge_client.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

#include <string_view>

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

/*
void msgToOggPacket(json j, ogg_packet &ogg)
{
  string s = j["msg"]["bytes"].dump();
  std::string_view data = s;
  data = s.substr(1,s.size()-2);
  rosbridge_lib::base64_decode(data);
  int bytes = data.size();


  ogg.bytes      = bytes;
  ogg.b_o_s      = msg.b_o_s;
  ogg.e_o_s      = msg.e_o_s;
  ogg.granulepos = msg.granulepos;
  ogg.packetno   = msg.packetno;
  ogg.packet = new unsigned char[ogg.bytes];
  memcpy(ogg.packet, &msg.data[0], ogg.bytes);
}
*/

void image_callback(json j) {
    
    string s = j["msg"]["data"].dump();
    int l = s.size();
    s = s.substr(1,l-2); //terribly inneficient
    std::vector<char> decoded = rosbridge_lib::base64_decode(s); 
    mat_img = cv::imdecode(decoded, cv::IMREAD_COLOR);
    cv::cvtColor(mat_img, mat_img, cv::COLOR_BGR2RGB);
    
    if(!mat_img.empty()){
	pbp = gdk_pixbuf_new_from_data(mat_img.data, GDK_COLORSPACE_RGB, false, 8, mat_img.cols, mat_img.rows, mat_img.cols * 3, NULL, NULL);
	gtk_image_set_from_pixbuf(GTK_IMAGE(image), pbp);
    }
}

gint spin_ROS(gpointer data) {
    connected = ros->spin_once();
    return 1;
}

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
    ros->subscribe("/camera/color/image_raw/compressed", image_callback);
    ros->advertise("/phy_joy_topic", "sensor_msgs/Joy");
    ros->send_queue();

    //add network read to event loop
    g_timeout_add (3, spin_ROS, NULL);
}

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

int main(int argc, char **argv) {

    setup_gtk(argc, argv);
  
    gtk_main();

    return 0;
}

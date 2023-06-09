#include <cstdlib>
#include <gtkmm-3.0/gtkmm.h>
#include "json.hpp"
#include "rosbridge_client.hpp"


//typedefs
typedef nlohmann::json json;
typedef std::string string;


//consts
const string move_vec_topic = "/my_gen3/in/cartesian_velocity/";

const int linear_buttondIDX_to_vecIDX[6] = {2, 2, 1, 1, 0, 0};
const int linear_buttondIDX_to_vecCOEF[6] = {50, -50, 50, -50, 50, -50};

const int angular_buttondIDX_to_vecIDX[6] = {0, 0, 1, 1, 2, 2};
const int angular_buttondIDX_to_vecCOEF[6] = {50, -50, -50, 50, 50, -50};

//GTK stuff
GtkWidget* window;
GtkWidget* layout;

std::vector<GtkWidget*> linear_buttons(6);
std::vector<GtkWidget*> angular_buttons(6);

GtkWidget* connect_button;
GtkWidget* connect_entry[2];


//move vectors
int linear_move[3] = {0};
int angular_move[3] = {0};


// ROS globals
rosbridge_lib::rosbridge_client ros;
int connected = 0;


//publish move vecs
gint publish_move_vec(gpointer data) {
    json j;
    json twist;

    twist["linear_x"] = linear_move[0];
    twist["linear_y"] = linear_move[1];
    twist["linear_z"] = linear_move[2];
    twist["angular_x"] = 0;
    twist["angular_y"] = 0;
    twist["angular_z"] = 0;

    // j["data"] = {, linear_move[1], linear_move[2],
    //	 angular_move[0], angular_move[1], angular_move[2]};
    j["twist"] = twist;

    ros.publish(move_vec_topic, j);

    return ros.send_queue() != rosbridge_lib::seret_disconnected;
}

//GTK callback - connect to rosbridge server
gint setup_ROS(gpointer data) {
    
    //parse entries
    char* port_str = (char*)gtk_entry_get_text(GTK_ENTRY(connect_entry[1]));
    char* p;
    int port = strtol ( port_str, &p, 10);

    if (*p != 0) {
        std::cout << "invalid port" << std::endl;
    }

    string address = string((char*)gtk_entry_get_text(GTK_ENTRY(connect_entry[0])));

    //connect rosbridge_client
    int result = ros.connect(port, address);

    if (result != rosbridge_lib::conn_return::cnret_succes) {
    	ros.cleanup();
    	connected = 0;
    	std::cout << "failed to connect" << std::endl;

    	return 0;
    }
    
    connected = 1;
    std::cout << "connected!" << std::endl;
   
    //subscribe to arm topic
    ros.advertise(move_vec_topic, "kortex_driver/TwistCommand");
    connected = ros.send_queue() != rosbridge_lib::send_return::seret_disconnected;

    //add publish_move_vec to event loop
    g_timeout_add (100, publish_move_vec, NULL);

    return 1;
}

//find index of button in list
int find_index(std::vector<GtkWidget*> arr, GtkWidget* widget) {
    int idx = -1;

    for(int i = 0; i < arr.size(); i++){
    	if (arr[i] == widget) {
    	    idx = i;
    	    break;
    	}
    }

    return idx;
}

//GTK callback - respond to move button pressed
void press_Button(GtkWidget *widget, gpointer data) {
    int idx = -1;
    
    //find index of button in linear buttons
    idx = find_index(linear_buttons, widget);

    if (idx != -1) {
    	linear_move[linear_buttondIDX_to_vecIDX[idx]] = linear_buttondIDX_to_vecCOEF[idx];
    	return;
    }

    idx = find_index(angular_buttons, widget);

    if (idx != -1) {
    	angular_move[angular_buttondIDX_to_vecIDX[idx]] = angular_buttondIDX_to_vecCOEF[idx];
    	return;
    } else {
        // TODO-low handle error (button not found)
    }
}

//GTK callback - reset all buttons
void release_Button(GtkWidget *widget, gpointer data) {
    linear_move[0] = 0;
    linear_move[1] = 0;
    linear_move[2] = 0;
    
    angular_move[0] = 0;
    angular_move[1] = 0;
    angular_move[2] = 0;
}

//GTK callback - quit application
void quit_app(GtkWidget *widget, gpointer data) {
    if (connected) {
	   ros.cleanup();
    }

    gtk_main_quit();

    //exit(EXIT_SUCCESS);
}

//GTK - setup gtk window
void setup_gtk() {

    //create window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(window), 600, 580);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);

    //add layout
    layout = gtk_layout_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(window), layout);
    gtk_widget_show(layout);

    //register move buttons
    auto register_move_button_helper = [] (GtkWidget*& target, string text, int x, int y) {
    	target = gtk_button_new_with_label(text.c_str());
    	gtk_layout_put(GTK_LAYOUT(layout), target, x, y);
    	gtk_widget_set_size_request(target, 80, 35);
    	g_signal_connect(G_OBJECT(target), "pressed", G_CALLBACK(press_Button), NULL);
    	g_signal_connect(G_OBJECT(target), "released", G_CALLBACK(release_Button), NULL);
    };

    register_move_button_helper(linear_buttons[0], "up", 200, 250);
    register_move_button_helper(linear_buttons[1], "down", 200, 350);
    register_move_button_helper(linear_buttons[2], "left", 100, 300);
    register_move_button_helper(linear_buttons[3], "right", 300, 300);
    register_move_button_helper(linear_buttons[4], "forward", 400, 250);
    register_move_button_helper(linear_buttons[5], "backward", 400, 350);

    //add connection button and entries
    connect_button = gtk_button_new_with_label("Connect");
    gtk_layout_put(GTK_LAYOUT(layout), connect_button, 100, 50);
    gtk_widget_set_size_request(connect_button, 80, 30);
    g_signal_connect(G_OBJECT(connect_button), "clicked", G_CALLBACK(setup_ROS), NULL);

    connect_entry[0] = gtk_entry_new();
    gtk_layout_put(GTK_LAYOUT(layout), connect_entry[0], 190, 50);
    gtk_entry_set_width_chars(GTK_ENTRY(connect_entry[0]), 20);
    gtk_entry_set_text(GTK_ENTRY(connect_entry[0]), "localhost");

    connect_entry[1] = gtk_entry_new();
    gtk_layout_put(GTK_LAYOUT(layout), connect_entry[1], 375, 50);
    gtk_entry_set_width_chars(GTK_ENTRY(connect_entry[1]), 10);
    gtk_entry_set_text(GTK_ENTRY(connect_entry[1]), "9090");

    //add exit signal
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(quit_app), NULL);
    
    //show window
    gtk_widget_show_all(window);
}

//
// main
//
int main(int argc, char **argv) {

    //init
    gtk_init(&argc, &argv);

    //setup
    setup_gtk();

    //begin main loop
    gtk_main();

    return 0;
}

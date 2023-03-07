#include "rosbridge_client.hpp"
#include "joystick_listener.hpp"

#include <ogg/ogg.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

#include <gtkmm-3.0/gtkmm.h>
#include <gdkmm-3.0/gdkmm.h>

#include <string_view>
#include <vector>


//ripped from github
class theora_context {
public:
    int pplevel_; 
    bool received_header_;
    bool received_keyframe_;

    th_info header_info_;
    th_comment header_comment_;

    th_setup_info* setup_info_;
    th_dec_ctx* decoding_context_;

    theora_context(){
	pplevel_ = 0;
	received_header_ = false;
        received_keyframe_ = false;
	header_info_ = {0};
	header_comment_ = {0};
	setup_info_ = 0;
	decoding_context_ = 0;
    }

    ~theora_context(){
	th_setup_free(setup_info_);
	th_decode_free(decoding_context_);
    }

    void reset(){
	pplevel_ = 0;
	received_header_ = false;
	received_keyframe_ = false;

	if (decoding_context_) {
	    th_decode_free(decoding_context_);
	    decoding_context_ = NULL;
	}

	if (setup_info_){
	    th_setup_free(setup_info_);
	    setup_info_ = NULL;
	}
	

	th_info_clear(&header_info_);
	th_info_init(&header_info_);

	th_comment_clear(&header_comment_);
	th_comment_init(&header_comment_);

    }

    void update_pp_level(){
	int pplevel_max;
	int err = th_decode_ctl(decoding_context_, TH_DECCTL_GET_PPLEVEL_MAX, &pplevel_max, sizeof(int));
	if (err) {
	    //RCLCPP_WARN(logger_, "Failed to get maximum post-processing level, error code %d", err);
	} else if (pplevel_ > pplevel_max) {
	    //RCLCPP_WARN(logger_, "Post-processing level %d is above the maximum, clamping to %d", level, pplevel_max);
	    pplevel_ = pplevel_max;
	}

	err = th_decode_ctl(decoding_context_, TH_DECCTL_SET_PPLEVEL, &pplevel_, sizeof(int));
	if (err) {
	    //RCLCPP_ERROR(logger_, "Failed to set post-processing level, error code %d", err);
	}
    }
};

struct theora_stream {
    theora_context decode_function_context; //replace this with stream_context interface
    cv::Mat buffer;
    GtkWidget* target;
};

struct image_group {
    GtkWidget* image;
    GtkWidget* entry;
    GtkWidget* button;
    std::string topic;
    theora_stream* stream_;
};

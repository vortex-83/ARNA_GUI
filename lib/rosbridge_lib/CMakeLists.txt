cmake_minimum_required(VERSION 3.10)
project (rosbridge_lib)

add_library(rosbridge_lib SHARED src/rosbridge_client.cpp)
target_include_directories(rosbridge_lib PUBLIC inc)


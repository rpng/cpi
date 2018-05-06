cmake_minimum_required(VERSION 2.8.8)

# Project name
project(cpi_comm)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs nav_msgs sensor_msgs std_msgs)

# Add what messages should be generated
add_message_files(
        DIRECTORY msg
        FILES
        FeatureMeasurement.msg
        CameraMeasurement.msg
)

# Generate our messages
generate_messages(DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs)


# Describe catkin project
catkin_package(
        DEPENDS
        CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs sensor_msgs std_msgs
        INCLUDE_DIRS
)










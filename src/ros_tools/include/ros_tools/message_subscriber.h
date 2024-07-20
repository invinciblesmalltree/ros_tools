#ifndef MESSAGE_SUBSCRIBER_H
#define MESSAGE_SUBSCRIBER_H

#include <ros/ros.h>

template <typename T> void msg_cb(T &msg_var, const typename T::ConstPtr &msg) {
    msg_var = *msg;
}

template <typename T>
ros::Subscriber subscribe(ros::NodeHandle &nh, const std::string &topic,
                          T &msg_var) {
    auto wrapper_handler = [&msg_var](const typename T::ConstPtr &msg) {
        msg_cb(msg_var, msg);
    };

    return nh.subscribe<T>(topic, 10, wrapper_handler);
}

#endif // MESSAGE_SUBSCRIBER_H

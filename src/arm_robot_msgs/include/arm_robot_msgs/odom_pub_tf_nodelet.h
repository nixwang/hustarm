//
// Created by hustac on 18-10-22.
//

#ifndef PROJECT_NODELET_ODOM_PUB_TF_H
#define PROJECT_NODELET_ODOM_PUB_TF_H

#include <nodelet/nodelet.h>

namespace arm_robot_msgs {

class odom_pub_tf_nodelet : nodelet::Nodelet {
public:
    virtual void onInit();
};

}

#endif //PROJECT_NODELET_ODOM_PUB_TF_H

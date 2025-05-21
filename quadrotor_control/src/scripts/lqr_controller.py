#!/usr/bin/env python3

import rospy


class DroneLQR:
    def __init__(self):
        rospy.init_node('quadrotor_lqr', anonymous=True)


if __name__ == "__main__":
    try:
        drone = DroneLQR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
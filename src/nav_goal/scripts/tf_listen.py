#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion

class CoordinatePrinter:
    def __init__(self):
        rospy.init_node('coordinate_printer', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(15)  # 10Hz refresh rate

    def run(self):
        while not rospy.is_shutdown():
            try:
                # Get transform between map and base_link
                (trans, rot) = self.tf_listener.lookupTransform(
                    'map', 'base_link', rospy.Time(0))
                
                # Convert quaternion to yaw angle
                _, _, yaw = euler_from_quaternion(rot)
                
                # Print formatted coordinates
                print "\rX: {0:.3f}m | Y: {1:.3f}m | Yaw: {2:.3f}rad".format(trans[0], trans[1], yaw),
                
            except (tf.LookupException, 
                    tf.ConnectivityException, 
                    tf.ExtrapolationException):
                print "\rWaiting for TF data...",
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        printer = CoordinatePrinter()
        printer.run()
    except rospy.ROSInterruptException:
        pass

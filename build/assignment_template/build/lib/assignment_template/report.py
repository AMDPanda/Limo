import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import MarkerArray

class potholes_report(Node):
    def __init__(self,mapflag = True):
        super().__init__('pothole_report_generator')
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            '/limo/pothole_location', 
            self.marker_callback,
            15
        )
        self.mapflag = mapflag
        self.map = self.load_background_image()
    def load_background_image(self):
        if self.mapflag:
            map = cv2.imread('src/Limo/assignment_template/assignment_template/background_potholes_simple.png')
            if map is None:
                print("Error1: Unable to load background image.")
            return map
            # cv2.imwrite('pothole_map.png', map)
        else:
            return cv2.imread('background_potholes.png')
        
    def marker_callback(self, msg):
        print(msg.markers.ns)
        self.map = self.load_background_image() 
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            print("coordinate",x,y)
            x_,y_ = self.real_to_image(x,y)
            print('new coor',x_,y_)
            size  = float(marker.ns)/200 * 10 
            cv2.circle(self.map, (int(x_),int( y_)), int(size), (255,255,0))
        text = "NUMBER of POTHOLES:"+ str(len(msg.markers))
        cv2.putText(self.map, str(text), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imwrite('src/Limo/assignment_template/report/pothole_map.png', self.map)
        print("Image saved area_scaledas 'pothole_map.png'")


    def real_to_image(self,x,y):
        # move the centroid to top left
        x = x+1.5
        y = -(y-0.2)
        height, width, _ = self.map.shape
        print('height',height)
        heightppixcel = height/886
        widthppixcel = width/1772
        x_pic = (x/3)*width
        y_pic = (y/1.5)*height
        return[x_pic,y_pic]
def main():
    rclpy.init()
    report = potholes_report()
    	# Simulate receiving marker data)
    rclpy.spin(report)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

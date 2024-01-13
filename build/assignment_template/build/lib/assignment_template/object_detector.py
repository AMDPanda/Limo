# Python libs
from stringprep import in_table_c22
import rclpy
from rclpy.node import Node
from rclpy import qos
import numpy as np
# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose

class ObjectDetector(Node):
    camera_model = None
    image_depth_ros = None
    
    # visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = 1.0 # for a simulated camera

    def __init__(self):    

        super().__init__('image_projection_3')
        self.bridge = CvBridge()
        self.id = 0
        self.mapflag = False
        self.potholes_rviz = MarkerArray()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback, 
                                                qos_profile=qos.qos_profile_sensor_data)
        
        # self.object_location_pub = self.create_publisher(PoseStamped, '/limo/object_location', 10)

        self.pothole_location = self.create_publisher(MarkerArray, '/limo/pothole_location', 10)
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)
        if self.mapflag == True:
            # convert to HSV
            hsv_image = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)  
            # define range of hsv  
            lower_hsv = np.array([140,50,30])
            upper_hsv = np.array([180,255,255])

            pothole_mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
            contours, _ = cv2.findContours(pothole_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            

            if len(contours) == 0:
                print('No pink object detected.')
                return

            cv2.drawContours(image_color, contours, -1, (0,255,0), 3) # To visualize the pothole in the robot image
            cv2.imshow('Potholes_pink', image_color)
            cv2.waitKey(1)
            self.get_location(contours,data)
        else:
            # scenario of realworld simulation
            # edge detection + contour analysis
            grayscale= cv2.cvtColor(image_color,cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(grayscale, (5, 5), 0)
            canny_1 = cv2.Canny(grayscale, 0, 135)
            canny_2 = cv2.Canny(grayscale, 0, 28) 
            edges = cv2.subtract(canny_2, canny_1)
            # edges = cv2.Canny(blurred,50,150)
            kernel = np.ones((10,10), np.uint8)
            dilated = cv2.dilate(edges, kernel, iterations=1)
            contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            new_contours = []
            # Filter and analyze contours
            for contour in contours:
                
                if cv2.contourArea(contour) < 500:  # The area threshold may need tuning
                    
                    continue
                peri = cv2.arcLength(contour, True)
                epsilon = 0.1 * peri
                approx = cv2.approxPolyDP(contour, epsilon, True)
                        # Assuming potholes are roughly circular, we might expect a higher number of approximation points
                # if len(approx) < 3:
                #     continue
                x, y, w, h = cv2.boundingRect(contour)
                if 4>np.abs(x-w)/np.abs(y-h)>0.25 :
                    new_contours.append(contour)
                    # Visualize the contour
                    cv2.drawContours(image_color, [contour], -1, (0, 255, 0), 2)

            # Show the output image
            cv2.imshow('Detected Potholes', image_color)
            cv2.waitKey(1)
            self.get_location(new_contours,data)
            

            return
        

    def get_location(self, contours, data):
        print('Number of contours found:', len(contours))
        image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros,"32FC1") 
        for contour in contours:
            
            area = cv2.contourArea(contour) 
            print('area', area)
            # Calculate contour area and ignore small areas
            # if cv2.contourArea(contour) < 10:  # The area threshold will need to be adjusted
            #     continue

            # calculate moments of the binary image
            M = cv2.moments(contour)

            if M["m00"] == 0:
                print('No object detected.')
                return

            # calculate the y,x centroid
            image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
            # "map" from color to depth image
            depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
                  image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
            # # get the depth reading at the centroid location
            depth_y, depth_x = int(depth_coords[0]), int(depth_coords[1])
            if depth_x >= image_depth.shape[1]:
                 depth_x = image_depth.shape[1]-1
            if depth_y >= image_depth.shape[0]:
                 depth_y = image_depth.shape[0]-1

            depth_value = image_depth[depth_y,depth_x] # you might need to do some boundary checking first!

            # print('image coords: ', image_coords)
            # print('depth coords: ', depth_coords)
            # print('depth value: ', depth_value)

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

            print('camera coords: ', camera_coords)

            #define a point in camera coordinates
            object_location = PoseStamped()
            object_location.header.frame_id = "depth_link"
            object_location.pose.orientation.w = 1.0
            object_location.pose.position.x = camera_coords[0]
            object_location.pose.position.y = camera_coords[1]
            object_location.pose.position.z = camera_coords[2]

            # publish so we can see that in rviz
            # self.object_location_pub.publish(object_location)        

            # print out the coordinates in the odom frame
            try:

                transform = self.get_tf_transform('map', 'depth_link')
                p_camera = do_transform_pose(object_location.pose, transform)
            except Exception as e:
                self.get_logger().error(f"Failed to transform pose: {e}")
                return
            p_exist = self.potholes_rviz.markers.copy()
            for m in p_exist:
                if np.abs(p_camera.position.x - m.pose.position.x)< 0.15 and np.abs(p_camera.position.y - m.pose.position.y)< 0.15:
                    print('repeated')
                    return
                
            # if not aleady published add to the array
            pothole_marker = Marker()
            pothole_marker.header.frame_id = "map"  # Use the correct frame_id here
            pothole_marker.header.stamp = self.get_clock().now().to_msg()
            pothole_marker.ns = "potholes"
            pothole_marker.id = self.id
            pothole_marker.type = Marker.SPHERE  # Use SPHERE_LIST for multiple points
            pothole_marker.action = 0
            pothole_marker.scale.x = 0.15  # Size of the marker
            pothole_marker.scale.y = 0.15
            pothole_marker.scale.z = 0.15
            pothole_marker.color.a = 1.0  # Alpha (transparency)
            pothole_marker.color.r = 1.0  # Red
            pothole_marker.color.g = 0.0  # Green
            pothole_marker.color.b = 0.0  # Blue
            pothole_marker.pose.position.x = p_camera.position.x # The pose of the marker
            pothole_marker.pose.position.y = p_camera.position.y
            pothole_marker.pose.position.z = p_camera.position.z
            pothole_marker.pose.orientation.x = p_camera.orientation.x
            pothole_marker.pose.orientation.y = p_camera.orientation.y
            pothole_marker.pose.orientation.z = p_camera.orientation.z
            pothole_marker.pose.orientation.w = p_camera.orientation.w
            self.potholes_rviz.markers.append(pothole_marker)   
            self.id += 1
            print('odom coords: ', p_camera.position)
        self.pothole_location.publish(self.potholes_rviz)

            # if self.visualisation:

            #     # draw circles
            #     cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
            #     cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)

            #     #resize and adjust for visualisation
            #     # image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
            #     image_depth *= 1.0/10.0 # scale for visualisation (max range 10.0 m)

            #     cv2.imshow("image depth", image_depth)
            #     cv2.imshow("image color", image_color)
            #     cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_projection = ObjectDetector()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

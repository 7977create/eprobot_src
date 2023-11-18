#!/usr/bin/env python
#EPRobot 
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
 
 
class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_callback, queue_size=10)
    self.image_pub = rospy.Publisher('/image_hsv', Image, queue_size=10)
    self.image_compressed_pub = rospy.Publisher('/image_hsv/compressed', CompressedImage, queue_size=10)
    self.image_bin_pub = rospy.Publisher('/image_hsv/bin', Image, queue_size=10)
    self.image_bin_compressed_pub = rospy.Publisher('/image_hsv/bin/compressed', CompressedImage, queue_size=10)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    self.twist = Twist()


    self.forward_velocity = rospy.get_param('~forward_velocity', 0.2)
    self.scale_diversion = rospy.get_param('~scale_diversion', 0.4)

 
  def image_callback(self, msg):
    #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #lower_line = numpy.array([ 0,  43,  46]) #red
    #upper_line = numpy.array([10, 255, 255]) #red
    lower_line = numpy.array([ 0,  0,  0]) #black and gray
    upper_line = numpy.array([180, 255, 95]) #black and gray
    mask = cv2.inRange(hsv, lower_line, upper_line) #
    mask_pub  = cv2.inRange(hsv, lower_line, upper_line) #
    
    # Astra Pro image offset
    img_offset = 60

    # BEGIN CROP
    h, w, d = image.shape
    search_top = 2*h/3
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    # END CROP
    # BEGIN FINDER
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        cv2.circle(mask_pub, (cx, cy), 20, (0,0,0), -1)
        vtherror = cx - w/2 - img_offset
        print cx,cy
        self.twist.linear.x = self.forward_velocity
        self.twist.angular.z = -float(vtherror) / 300 * self.scale_diversion   # 400: 0.1, 300: 0.15, 250, 0.2
        self.cmd_vel_pub.publish(self.twist)
    else:
        print 'not found line'
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
    
    cv2.line(image, (0, search_top), (w,search_top), (0,0,255), 1)
    cv2.line(image, (0, search_bot), (w,search_bot), (0,0,255), 1)
    cv2.line(image, (w/2 + img_offset, 0), (w/2 + img_offset, h), (0,0,255), 1)
    
    cv2.line(mask_pub, (0, search_top), (w,search_top), (255,255,255), 1)
    cv2.line(mask_pub, (0, search_bot), (w,search_bot), (255,255,255), 1)
    cv2.line(mask_pub, (w/2 + img_offset, 0), (w/2 + img_offset, h), (255,255,255), 1)

    hsv_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
    hsv_image_bin = self.bridge.cv2_to_imgmsg(mask_pub)
    hsv_iamge_compressed = self.bridge.cv2_to_compressed_imgmsg(image)
    hsv_iamge_bin_compressed = self.bridge.cv2_to_compressed_imgmsg(mask_pub)
    
    self.image_pub.publish(hsv_image)
    self.image_bin_pub.publish(hsv_image_bin)
    self.image_compressed_pub.publish(hsv_iamge_compressed)
    self.image_bin_compressed_pub.publish(hsv_iamge_bin_compressed)


 
rospy.init_node('follower')
follower = Follower()
rospy.spin()

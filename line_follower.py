#!/usr/bin/python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

rospy.init_node('follower')

last_time=rospy.Time.now()
time=rospy.Time.now()
e_last=0
time_diff=0
cy_last=0
cy_total=0

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        #certain bots like turtlebot 2 has this same publisher, while some have it as only 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.twist = Twist()
        # kp and kd may vary according to the bot used, so tune accordingly 
        self.kp=0.1
        self.kd=0.5
        self.time=rospy.Time.now()
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #using HSV for the robot to work in varied lighting
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 10, 10])  #lower limit of yellow line
        upper_yellow = numpy.array([255, 255, 250]) #upper limit of yellow line
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = self.image.shape
        search_bot = 3*h/4 + 50
        #limiting the image feed so as to decrease proccesing and lag
        mask[search_bot:h, 0:w]=0
        M = cv2.moments(mask)
        if M['m00'] != 0:
            self.cx = int(M['m10']/M['m00'])
            self.cy = int(M['m01']/M['m00'])
            cv2.circle(self.image, (self.cx, self.cy), 20, (0,0,255), -1)
            err =self.cx - w/2
            self.pd(err)
        #robot may stop few cms befor the end of staright line, so this following code helps in traversing it
        else:
            self.twist.angular.z=0
            if rospy.Time.now().to_sec()<=(self.time.to_sec()+30): #this number i.e '30' may change according to bot used, so tune accordingly
                self.twist.linear.x=0.2
                self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", self.image)
                cv2.waitKey(3)
            else:
                self.twist.linear.x=0
                self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", self.image)
                cv2.waitKey(3)

    def pd(self, e):
        #this is the code for the pd controller
        global e_last
        global last_time
        global time_diff
        global time
        global cy_last
        global cy_total
        if e_last==0:
            last_time=rospy.Time.now()
            self.twist.angular.z=-((self.kp*e)/100)

        else:
            self.time=rospy.Time.now()
            time_diff=self.time-last_time
            last_time=self.time
            self.twist.angular.z=-((self.kp*e-abs(self.kd*((e-e_last)/(time_diff.to_sec()))))/100)
        #linear velocity shd be low when angular velocity is high and vice versa, so the following code is used
        self.twist.linear.x=abs(0.2-abs(self.twist.angular.z)/40)
        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", self.image)
        cv2.waitKey(3)

follower = Follower()
rospy.spin()

#!/usr/bin/env python3
import rospy 
import tf
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import PositionTarget,GlobalPositionTarget
from sensor_msgs.msg import Range
from precision_landing.srv import SelectTag,SelectTagResponse 

#define class apriltagcontrol 
class ApriltagControl(object):
    def __init__(self):
        rospy.init_node('ApriltagControl',anonymous=True)
        self.transformation_listener = tf.TransformListener()
        self.position_target_publish = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=10) #AprilTags position
        self.new_GPS_data_recovery = rospy.Publisher('/mavros/setpoint_raw/global',GlobalPositionTarget,queue_size=10) #GPS data that input after not detect AprilTags
        self.gps_position_subscribe = rospy.Subscriber('/mavros/global_position/rel_alt',Float64,self.Get_Position) #Real altitude from pixhawk
        self.rangefinder_positiom_subscribe = rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub',Range,self.range_altitude) #Altitude from rangefinder
        self.select_bundle = rospy.Service('/apriltag/tag_select',SelectTag,self.Go_To_Tags) #Detected Apriltags
        self.rate = rospy.Rate(1) 
        #self.process_time_xy = 20.0 
        #self.process_time_z = 10000.0
        self.position = None
        self.orientation = None 
        rospy.spin()

    #define get_position function
    # Function that get altitude from pixhawk 
    def Get_Position(self,data): 
        self.pose_z = data.data

    #define range_altitude dunction
    #Function that get altitude from rangefinder
    def range_altitude(self,data):
        self.pose_z_range = data.range

    #define new_GPS_data function 
    #Function that publish Apriltags GPS data after not detect tags 
    def New_GPS_Data(self,position):
        new_GPS_data = GlobalPositionTarget()
        new_GPS_data.coordinate_frame = 6
        new_GPS_data.type_mask = 3576
        new_GPS_data.latitude = position[0]*0.0000001
        new_GPS_data.longitude = position[1]*0.0000001
        new_GPS_data.altitude = position[2]
        #new_GPS_data.pose.orientation.x = 0
        #new_GPS_data.pose.orientation.y = 0
        #new_GPS_data.pose.orientation.z = 0
        #new_GPS_data.pose.orientation.w = 1
        #publish GPS position recovery to vechicle
        self.new_GPS_data_recovery.publish(new_GPS_data)
        self.rate.sleep()

    #define setpoint_publish function 
    #Function that publish Apriltags posistion after detected 
    def Setpoint_Publish(self,type_mask,pose,orientation,velocity,accel):
        position_target = PositionTarget()
        position_target.coordinate_frame = 9
        position_target.type_mask = type_mask
        position_target.position.x = pose[0]
        position_target.position.y = pose[1]
        position_target.position.z = pose[2] 
        position_target.velocity.x = velocity[0]
        position_target.velocity.y = velocity[1]
        position_target.velocity.z = velocity[2]
        position_target.acceleration_or_force.x = accel[0]
        position_target.acceleration_or_force.y = accel[1]
        position_target.acceleration_or_force.z = accel[2]
        euler_yaw = tf.transformations.euler_from_quaternion(orientation)
        position_target.yaw = -euler_yaw[2]
        #publish position targrt to vechicle
        self.position_target_publish.publish(position_target)
        self.rate.sleep()

    def add_two_list(self, list1, list2):
        product = []
        if len(list1) == len(list2):
            for n in range(len(list1)):
                result = float(list1[n]) + float(list2[n])
                product.append(result)
        else:
            print('error')
        return product

    #define Go_To_Tags function 
    def Go_To_Tags(self,data):
        #print('get request')
        bundle_no = '/' + data.tag_no
        transition = None
        rotation = None
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                #subscribe tf of apriltag
                self.transformation_listener.waitForTransform('/usb_cam1',bundle_no,now,rospy.Duration(1.0))
                (transition_apriltag,rotation_apriltag) = self.transformation_listener.lookupTransform('/usb_cam1',bundle_no,now) 
                self.transformation_listener.waitForTransform('/usb_cam1','/beacon',now,rospy.Duration(1.0))
                (transition_beacon,rotation_beacon) = self.transformation_listener.lookupTransform('/usb_cam1','/beacon',now)
                product1 = [x * 0.6 for x in transition_apriltag] 
                product2 = [x * 0.4 for x in transition_beacon]
                transition = self.add_two_list(product1, product2)
                rotation = rotation_apriltag
                if data.altitude == 0.0:
                    #change heading
                    type_mask = 2503
                    pose = [0,0,data.altitude]
                    orientation = [rotation[0],rotation[1],rotation[2],rotation[3]]
                    velocity = [0,0,0]
                    accel = [0,0,0] 
                    print('Change vehicle heading')
                    #Go to tags with holds altitude
                    self.Setpoint_Publish(type_mask,pose,orientation,velocity,accel)
                    time.sleep(2)
                    type_mask = 3576  
                    pose = [-transition[1],-transition[0],data.altitude]
                    orientation = [0,0,0,1]
                    velocity = [0,0,0]
                    accel = [0,0,0]
                #print('Go to tags with holds altitude')
                #print('pose = ' , pose)
                #print('velocity = ',velocity)
                #print('accel =',accel)
                    print('Go to tag')
                    self.Setpoint_Publish(type_mask,pose,orientation,velocity,accel)                       
                else: 
                    type_mask = 3576
                    pose = [-transition[1],-transition[0],data.altitude]
                    orientation = [0,0,0,1]
                    velocity = [0,0,0]
                    accel = [0,0,0]
                    print('Go to tags with decent altitude')
                #print('pose = ' , pose)
                #print('velocity = ',velocity)
                #print('accel =',accel)
                #delete pose z 
                    n = self.pose_z_range+data.altitude
                    self.Setpoint_Publish(type_mask,pose,orientation,velocity,accel)   
                    while True:
                        if self.pose_z_range-n >0.5:
                            self.altitude_sub = rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub',Range,self.range_altitude)
                        else:
                            break
                #change altitude
                if self.pose_z_range <= 2 and bundle_no == '/tag_77':
                    #print('return true')
                    return SelectTagResponse(True)
                if self.pose_z_range <=0.55 and bundle_no == '/tag_61':
                    #print('return true')
                    return SelectTagResponse(True)
                else:
                    #print('return false')
                    return SelectTagResponse(False)
            except Exception as e:
                print(e)
                if self.pose_z_range >=3:
                    print("Can't see any tags, Please Input New GPS Data")
                    lat = float(input('Input New latitude: '))
                    long = float(input('Input New longitude: '))
                    pose = [lat,long,0.0]
                    self.New_GPS_Data(pose)
                    #print('return false')
                    time.sleep(5)
                    #return SelectTagResponse(False)
                    continue
                else:
                    print("Can't see any tags, increase altitude ")
                    altitude_recovery = self.pose_z_range
                    print('altitude_recovery = ', altitude_recovery)
                    type_mask = 3576
                    pose = [0,0,altitude_recovery]
                    orientation = [0,0,0,1]
                    velocity = [0,0,0]
                    accel = [0,0,0] 
                    n = 2*altitude_recovery
                    self.Setpoint_Publish(type_mask,pose,orientation,velocity,accel)
                    while True:
                        if n-self.pose_z_range > 0.5:
                            self.altitude_sub = rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub',Range,self.range_altitude)
                        else:
                            break
            
                    #print('return false')       
                    return SelectTagResponse(False)

#main
if __name__ == '__main__':
    try :
        ApriltagControl()
    except rospy.ROSInternalException:
        pass


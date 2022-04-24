import rospy
from visualization.msg import DoorSensor
import random

def publisher():
    pub = rospy.Publisher('sensor_data',  DoorSensor)
    rospy.init_node('Sensor_writter', anonymous=True)
    r = rospy.Rate(30)
    
    now = rospy.Time.now()
  


    while not rospy.is_shutdown():
        msg = DoorSensor()
        msg.current_time = rospy.Time.now() - now
       
        msg.tof  = 120 + random.randint(0, 100) - 50
        msg.fsr1 = 110 + random.randint(0, 100) - 50 
        msg.fsr2 = 100 + random.randint(0, 100) - 50
        msg.fsr3 = 90 + random.randint(0, 100) - 50
        msg.fsr4 = 80 + random.randint(0, 100) - 50
        msg.fsr5 = 70 + random.randint(0, 100) - 50
        msg.fsr6 = 60 + random.randint(0, 100) - 50
        msg.fsr7 = 50 + random.randint(0, 100) - 50
        msg.fsr8 = 40 + random.randint(0, 100) - 50
        msg.fsr9 = 30 + random.randint(0, 100) - 50
        msg.fsr10 = 20 + random.randint(0, 100) - 50
        msg.fsr11 = 10 + random.randint(0, 100) - 50
        msg.fsr12 = 0 + random.randint(0, 100) - 50
        msg.fsr_contact_1
        msg.fsr_contact_2
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    publisher()
#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import Bool

class Disarmed(smach.State):
    def __init__(self, input_data):
        smach.State.__init__(self, outcomes=['armed', 'full_stop'])
        self.input_data = input_data

    def execute(self, userdata):
        rospy.loginfo('State is DISARMED')
        self.input_data.arm_signal = False
        while(not self.input_data.arm_signal):
            rospy.sleep(1)
        return 'armed'

class Armed(smach.State):
    def __init__(self, input_data):
        smach.State.__init__(self, outcomes=['disarmed','acceleration', 'emergency_stop'])
        self.input_data = input_data

    def execute(self, userdata):
        rospy.loginfo('State is ARMED')
        self.input_data.launch_signal = False
        while(not self.input_data.launch_signal and not self.input_data.e_stop_signal):
            rospy.sleep(1)
        if self.input_data.launch_signal:
            return 'acceleration'
        else:
            return 'emergency_stop'

class Acceleration(smach.State):
    def __init__(self, input_data):
        smach.State.__init__(self, outcomes=['cruise','braking','emergency_stop'])
        self.input_data = input_data

    def execute(self, userdata):
        rospy.loginfo('State is ACCELERATION')
        return 'cruise'

class Cruise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['braking','emergency_stop'])

    def execute(self, userdata):
        rospy.loginfo('State is CRUISE')
        return 'braking'

class Braking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['secondary_brake','emergency_stop'])

    def execute(self, userdata):
        rospy.loginfo('State is BRAKING')
        return 'secondary_brake'

class SecondaryBrake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['full_stop','emergency_stop'])

    def execute(self, userdata):
        rospy.loginfo('State is SECOND BRAKE')
        return 'full_stop'

class FullStop(smach.State):
    def __init__(self, input_data):
        smach.State.__init__(self, outcomes=['disarmed','emergency_stop'])
        self.input_data = input_data

    def execute(self, userdata):
        rospy.loginfo('State is FULL STOP')
        self.input_data.e_stop_signal = True
        return 'emergency_stop'

class EmergenecyStop(smach.State):
    def __init__(self, input_data):
        smach.State.__init__(self, outcomes=['disarmed'])
        self.input_data = input_data

    def execute(self, userdata):
        rospy.loginfo('State is EMERGENCY STOP')
        while(self.input_data.e_stop_signal):
            rospy.sleep(1)
        return 'disarmed'
        

class Inputs():
    def __init__(self):
        self.estop_sub = rospy.Subscriber("/e_stop", Bool, self.e_stop_cb)
        self.arm_sub = rospy.Subscriber("/arm", Bool, self.arm_cb)
        self.launch_sub = rospy.Subscriber("/launch", Bool, self.launch_cb)
        self.e_stop_signal = True
        self.arm_signal = False
        self.launch_signal = False

    def e_stop_cb(self, msg):
        self.e_stop_signal = msg.data
        return
    def arm_cb(self, msg):
        self.arm_signal = msg.data
        return
    def launch_cb(self, msg):
        print("Got Launch Signal")
        self.launch_signal = msg.data
        return

# main
def main():
    rospy.init_node('HLSM')
    rate = rospy.Rate(1)

    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
    input_data = Inputs()

    with sm:
        smach.StateMachine.add('DISARMED', Disarmed(input_data), 
                               transitions={'armed':'ARMED', 
                                            'full_stop':'FULLSTOP'})
        smach.StateMachine.add('ARMED', Armed(input_data), 
                               transitions={'disarmed':'DISARMED', 
                                            'acceleration':'ACCELERATION',
                                            'emergency_stop': 'EMERGENCYSTOP'})
        smach.StateMachine.add('ACCELERATION', Acceleration(input_data), 
                               transitions={'cruise':'CRUISE', 
                                            'braking':'BRAKING',
                                            'emergency_stop': 'EMERGENCYSTOP'})
        smach.StateMachine.add('CRUISE', Cruise(), 
                               transitions={'braking':'BRAKING', 
                                            'emergency_stop':'EMERGENCYSTOP'})
        smach.StateMachine.add('BRAKING', Braking(), 
                               transitions={'secondary_brake':'SECONDARYBRAKE', 
                                            'emergency_stop':'EMERGENCYSTOP'})
        smach.StateMachine.add('SECONDARYBRAKE', SecondaryBrake(), 
                               transitions={'full_stop':'FULLSTOP', 
                                            'emergency_stop':'EMERGENCYSTOP'})
        smach.StateMachine.add('FULLSTOP', FullStop(input_data), 
                               transitions={'disarmed':'DISARMED', 
                                            'emergency_stop':'EMERGENCYSTOP'})
        smach.StateMachine.add('EMERGENCYSTOP', EmergenecyStop(input_data), 
                               transitions={'disarmed':'DISARMED'})

    while not rospy.is_shutdown():
        outcome = sm.execute()
        rate.sleep()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

from std_msgs.msg import String

# Define the WaitForCall state
class WaitForCall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_called', 'called'])
        self.rate = rospy.Rate(1)
        self.call_received = False

    def execute(self, userdata):
        rospy.loginfo('Waiting for call...')
        #while not self.call_received:
        #    if rospy.is_shutdown():
        #        return 'not_called'
        #    self.rate.sleep()
        return 'called'

# Define the TakeOrder state
class TakeOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], 
                             input_keys=['order'], output_keys=['order'])
        
    def execute(self, userdata):
        # 注文を取る処理
        #order = take_order()  # 関数 take_order() は注文を取り、リストで返す想定
        order = ['Tea','Cola'] 
        # 注文を次のステートに渡す
        userdata.order = order
        
        return 'success'

# Define the DeliverOrder state
class DeliverOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], 
                             input_keys=['order'], output_keys=['order'])
        
    def execute(self, userdata):
        # 注文を受け取る処理
        order = userdata.order
        
        # 注文を配達する処理
        #deliver_order(order)
        
        return 'success'


# Create a SMACH state machine
def main():
    rospy.init_node('restaurant_fsm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT_FOR_CALL', WaitForCall(), 
                               transitions={'called': 'TAKE_ORDER_1',
                                            'not_called': 'WAIT_FOR_CALL'})
        smach.StateMachine.add('TAKE_ORDER_1', TakeOrder(), 
                               transitions={'success': 'DELIVER_ORDER_1'}, 
                               remapping={'order': 'order_1'})
        smach.StateMachine.add('DELIVER_ORDER_1', DeliverOrder(), 
                               transitions={'success': 'WAIT_FOR_CALL_2'},
                               remapping={'order': 'order_1'})
        smach.StateMachine.add('WAIT_FOR_CALL_2', WaitForCall(), 
                               transitions={'called': 'TAKE_ORDER_2',
                                            'not_called': 'WAIT_FOR_CALL'})
        smach.StateMachine.add('TAKE_ORDER_2', TakeOrder(), 
                               transitions={'success': 'DELIVER_ORDER_2'}, 
                               remapping={'order': 'order_2'})
        smach.StateMachine.add('DELIVER_ORDER_2', DeliverOrder(), 
                               transitions={'success': 'finish'},
                               remapping={'order': 'order_2'})

    # Execute the state machine
    outcome = sm.execute()


if __name__ == '__main__':
    main()

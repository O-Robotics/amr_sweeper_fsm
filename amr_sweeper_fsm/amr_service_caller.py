#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from amr_sweeper_fsm.srv import SetFSMState
from std_msgs.msg import UInt8

class AMRStateMachine(Node):
    # State definitions
    INITIALIZING = 0
    RUNNING = 1
    CHARGING = 2
    IDLE = 3
    FAULT = 4
    
    STATE_NAMES = {
        0: "INITIALIZING",
        1: "RUNNING",
        2: "CHARGING",
        3: "IDLE",
        4: "FAULT"
    }
    
    def __init__(self):
        super().__init__('amr_state_machine')
        
        # Initialize state
        self.current_state = self.INITIALIZING
        
        # Create service
        self.srv = self.create_service(
            SetFSMState,
            'amr/set_state',
            self.handle_state_change
        )
        
        # Create publisher for current state
        self.state_publisher = self.create_publisher(
            UInt8,
            'amr/current_state',
            10
        )
        
        # Timer to publish state periodically (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('AMR State Machine initialized in INITIALIZING state')
    
    def handle_state_change(self, request, response):
        """Handle service requests to change state"""
        target_state = request.target_state
        
        # Validate state value
        if target_state not in range(5):
            response.success = False
            response.message = f"Invalid state: {target_state}. Must be 0-4."
            response.current_state = self.current_state
            self.get_logger().warn(f"Invalid state request: {target_state}")
            return response
        
        # Check if transition is valid
        if self.is_valid_transition(self.current_state, target_state):
            old_state = self.current_state
            self.current_state = target_state
            response.success = True
            response.message = (
                f"Transitioned from {self.STATE_NAMES[old_state]} "
                f"to {self.STATE_NAMES[target_state]}"
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                f"Invalid transition from {self.STATE_NAMES[self.current_state]} "
                f"to {self.STATE_NAMES[target_state]}"
            )
            self.get_logger().warn(response.message)
        
        response.current_state = self.current_state
        return response
    
    def is_valid_transition(self, from_state, to_state):
        """Define valid state transitions"""
        valid_transitions = {
            self.INITIALIZING: [self.RUNNING, self.FAULT],
            self.RUNNING: [self.CHARGING, self.IDLE, self.FAULT],
            self.CHARGING: [self.IDLE, self.RUNNING, self.FAULT],
            self.IDLE: [self.RUNNING, self.CHARGING, self.FAULT],
            self.FAULT: [self.INITIALIZING]
        }
        
        # Allow staying in same state
        if from_state == to_state:
            return True
            
        return to_state in valid_transitions.get(from_state, [])
    
    def publish_state(self):
        """Publish current state periodically"""
        msg = UInt8()
        msg.data = self.current_state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AMRStateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

from ..base_behavior import BaseBehavior
import py_trees
from py_trees.common import Status

class MoveTurtle(BaseBehavior):
    """turtle is moving"""    
    def __init__(self, name: str):
        super(MoveTurtle, self).__init__(name)
        self.disconnect_manual_mode_after_this_amount_of_ticks = 10
    
    def setup(self, **kwargs) -> None:
        """Set up publishers for motor control"""
        # Here you would set up publishers to control motors, servos, etc.
        # Example:
        # self.motor_publisher = self.node.create_publisher(MotorCommand, '/motor_control', 10)
        self.node.get_logger().info("Manual boat movement initialized")
        return True
    
    def update(self):
        """Update manual control behavior"""
        if self.disconnect_manual_mode_after_this_amount_of_ticks < 0:
            self.node.get_logger().info("Manual control time limit reached, exiting manual mode")
            return py_trees.common.Status.SUCCESS
        
        # Here you would implement the actual manual control logic
        # Example:
        # cmd = MotorCommand()
        # cmd.throttle = 0.5  # 50% throttle
        # self.motor_publisher.publish(cmd)
        
        self.node.get_logger().debug(f"Manual control active, {self.disconnect_manual_mode_after_this_amount_of_ticks} ticks remaining")
        self.disconnect_manual_mode_after_this_amount_of_ticks -= 1
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status: Status) -> None:
        """Reset the behavior when terminated"""
        self.disconnect_manual_mode_after_this_amount_of_ticks = 10
        self.node.get_logger().info(f"Manual movement terminated with status: {new_status}")
        
        # Here you would implement cleanup logic like stopping motors
        # Example:
        # cmd = MotorCommand()
        # cmd.throttle = 0.0  # Stop motors
        # self.motor_publisher.publish(cmd)
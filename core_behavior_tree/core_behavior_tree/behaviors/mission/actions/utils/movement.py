"""
Movement utility module for vehicle control.
Provides modular functions for basic movements like straight, turn, stop, etc.
"""

from std_msgs.msg import Float64
from typing import Optional


class MovementController:
    """
    Utility class for controlling vehicle movements.
    Provides methods for basic movement commands.
    """
    
    def __init__(self, yaw_effort_pub, speed_effort_pub, logger=None):
        """
        Initialize the movement controller.
        
        Args:
            yaw_effort_pub: Publisher for yaw effort commands
            speed_effort_pub: Publisher for speed effort commands
            logger: Optional logger for debugging
        """
        self.yaw_effort_pub = yaw_effort_pub
        self.speed_effort_pub = speed_effort_pub
        self.logger = logger
    
    def _log(self, message: str):
        """Log a message if logger is available."""
        if self.logger:
            self.logger.info(message)
    
    def stop(self):
        """
        Stop all movement by setting both yaw and speed efforts to zero.
        """
        self._log("[MovementController] Stopping vehicle")
        self.yaw_effort_pub.publish(Float64(data=0.0))
        self.speed_effort_pub.publish(Float64(data=0.0))
    
    def go_straight(self, effort: float):
        """
        Move straight forward or backward.
        
        Args:
            effort: Speed effort value (positive for forward, negative for backward)
        """
        self._log(f"[MovementController] Going straight with effort: {effort}")
        self.yaw_effort_pub.publish(Float64(data=0.0))
        self.speed_effort_pub.publish(Float64(data=effort))
    
    def go_forward(self, effort: float):
        """
        Move forward.
        
        Args:
            effort: Speed effort value (should be positive)
        """
        self._log(f"[MovementController] Going forward with effort: {effort}")
        self.go_straight(abs(effort))
    
    def go_backward(self, effort: float):
        """
        Move backward.
        
        Args:roller] Turning left with effort: {effort}")
        self.yaw_effort_pub.publish(Float64(data=abs(effort)))
        self.speed_effort_pub.publish(Float64(data=0.0))
    
            effort: Speed effort value (will be negated)
        """
        self._log(f"[MovementController] Going backward with effort: {effort}")
        self.go_straight(-abs(effort))
    
    def turn_left(self, effort: float):
        """
        Turn left (counterclockwise).
        
        Args:
            effort: Yaw effort value (positive for left turn)
        """
        self._log(f"[MovementController] Turning left with effort: {effort}")
        self.yaw_effort_pub.publish(Float64(data=abs(effort)))
        self.speed_effort_pub.publish(Float64(data=0.0))
    
    def turn_right(self, effort: float):
        """
        Turn right (clockwise).opic.tuning_mission.createSubscriber(
            self.node,
            self._mission_cb
        )        
        self.effort_sub = Topic.tuning_effort_st.createSubscriber(
            self.node,
            self._effort_cb
        )        

        # Setup publishers
        yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
        speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
        # Initialize movement controller
        
        Args:
            effort: Yaw effort value (negative for right turn)
        """
        self._log(f"[MovementController] Turning right with effort: {effort}")
        self.yaw_effort_pub.publish(Float64(data=-abs(effort)))
        self.speed_effort_pub.publish(Float64(data=0.0))
    
    def turn(self, effort: float):
        """
        Turn with specified effort.
        
        Args:
            effort: Yaw effort value (positive=left, negative=right)
        """
        direction = "left" if effort > 0 else "right"
        self._log(f"[MovementController] Turning {direction} with effort: {effort}")
        self.yaw_effort_pub.publish(Float64(data=effort))
        self.speed_effort_pub.publish(Float64(data=0.0))
    
    def move(self, yaw_effort: float, speed_effort: float):
        """
        Move with custom yaw and speed efforts.
        Allows combined movements (e.g., turning while moving forward).
        
        Args:
            yaw_effort: Yaw effort value
            speed_effort: Speed effort value
        """
        self._log(f"[MovementController] Moving with yaw: {yaw_effort}, speed: {speed_effort}")
        self.yaw_effort_pub.publish(Float64(data=yaw_effort))
        self.speed_effort_pub.publish(Float64(data=speed_effort))
    
    def arc_left(self, yaw_effort: float, speed_effort: float):
        """
        Move in a left arc (forward while turning left).
        
        Args:
            yaw_effort: Yaw effort value (positive)
            speed_effort: Speed effort value
        """
        self._log(f"[MovementController] Arc left - yaw: {yaw_effort}, speed: {speed_effort}")
        self.move(abs(yaw_effort), speed_effort)
    
    def arc_right(self, yaw_effort: float, speed_effort: float):
        """
        Move in a right arc (forward while turning right).
        
        Args:
            yaw_effort: Yaw effort value (will be negated)
            speed_effort: Speed effort value
        """
        self._log(f"[MovementController] Arc right - yaw: {yaw_effort}, speed: {speed_effort}")
        self.move(-abs(yaw_effort), speed_effort)


# Standalone functions for direct use without class instantiation

def publish_stop(yaw_effort_pub, speed_effort_pub):
    """Stop the vehicle."""
    yaw_effort_pub.publish(Float64(data=0.0))
    speed_effort_pub.publish(Float64(data=0.0))


def publish_straight(yaw_effort_pub, speed_effort_pub, effort: float):
    """Move straight with given effort."""
    yaw_effort_pub.publish(Float64(data=0.0))
    speed_effort_pub.publish(Float64(data=effort))


def publish_turn(yaw_effort_pub, speed_effort_pub, effort: float):
    """Turn with given yaw effort."""
    yaw_effort_pub.publish(Float64(data=effort))
    speed_effort_pub.publish(Float64(data=0.0))


def publish_move(yaw_effort_pub, speed_effort_pub, yaw_effort: float, speed_effort: float):
    """Move with custom yaw and speed efforts."""
    yaw_effort_pub.publish(Float64(data=yaw_effort))
    speed_effort_pub.publish(Float64(data=speed_effort))

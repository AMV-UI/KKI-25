#!/usr/bin/env python3

from core_msgs.msg import Controller, AutoControl, Option, KillSwitch
from core.utils.config import RemoteState


def controllerToString(joy):
    """
    Convert Controller.msg to string
    """
    m_joy = {
        Controller.FORWARD: "FORWARD",
        Controller.BACKWARD: "BACKWARD",
        Controller.TURN_LEFT: "TURN_LEFT",
        Controller.TURN_RIGHT: "TURN_RIGHT",
        Controller.UP: "UP",
        Controller.DOWN: "DOWN",
        Controller.MODE_TOGGLE: "MODE_TOGGLE",
        Controller.IDLE: "IDLE"
    }

    return m_joy[joy.data]

def autocontrolToString(mode):
    autocontrol_mode = {
        AutoControl.MANUAL: "MANUAL",
        AutoControl.MISSION_FIND_STEP_ONE: "MISSION_FIND_STEP_ONE",
        AutoControl.MISSION_STEP_ONE: "MISSION_STEP_ONE", 
        AutoControl.MISSION_FIND_STEP_TWO: "MISSION_FIND_STEP_TWO",
        AutoControl.MISSION_STEP_TWO: "MISSION_STEP_TWO",
        AutoControl.MISSION_FIND_STEP_THREE: "MISSION_FIND_STEP_THREE",
        AutoControl.MISSION_STEP_THREE: "MISSION_STEP_THREE",
        AutoControl.MISSION_GREEN_BOX: "MISSION_GREEN_BOX",
        AutoControl.MISSION_BLUE_BOX: "MISSION_BLUE_BOX",
        AutoControl.MISSION_DOCKING: "MISSION_DOCKING", 
        AutoControl.MISSION_DONE: "MISSION_DONE",
    }

    return autocontrol_mode[mode.data]

def optionToString(strat):
    strat_map = {
        Option.SKIP: "Skip",
        Option.COLOR: "Contour Detection",
        Option.OBJECT:"Object Detection"
    }

def killswitchToString(stat):
    killswitch_stat = {
        KillSwitch.HARDWARE_OFF: "HARDWARE_OFF",
        KillSwitch.HARDWARE_ON: "HARDWARE_ON"
    }

    return killswitch_stat[stat.data]


def remoteStateToString(stat):
    remote_stat = {
        RemoteState.TBS_MANUAL: "Manual",
        RemoteState.TBS_AUTO: "Auto"
    }
    return remote_stat[stat.data]

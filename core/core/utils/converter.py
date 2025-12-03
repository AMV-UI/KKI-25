#!/usr/bin/env python3

from core_msgs.msg import Controller, AutoControl, Option, KillSwitch
from core.utils.config import RemoteState

def remoteStateToString(stat):
    remote_stat = {
        RemoteState.TBS_MANUAL: "Manual",
        RemoteState.TBS_AUTO: "Auto"
    }
    return remote_stat[stat.data]

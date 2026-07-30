import datetime

from google.protobuf import timestamp_pb2 as _timestamp_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    MANUAL: _ClassVar[Mode]
    AUTONOMOUS: _ClassVar[Mode]

class Side(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    A: _ClassVar[Side]
    B: _ClassVar[Side]
    C: _ClassVar[Side]
    D: _ClassVar[Side]
    NOT_FOUND: _ClassVar[Side]
MANUAL: Mode
AUTONOMOUS: Mode
A: Side
B: Side
C: Side
D: Side
NOT_FOUND: Side

class telemetryRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class telemetryResponse(_message.Message):
    __slots__ = ("mode", "battery", "timestamp", "qr_side", "depth", "fc_status", "sensor_status", "rollspeed", "pitchspeed", "yawspeed", "roll", "pitch", "yaw", "forward_rc", "lateral_rc", "vertical_rc", "yaw_rc", "mot1_eff", "mot2_eff", "mot3_eff", "mot4_eff", "mot5_eff", "mot6_eff")
    MODE_FIELD_NUMBER: _ClassVar[int]
    BATTERY_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    QR_SIDE_FIELD_NUMBER: _ClassVar[int]
    DEPTH_FIELD_NUMBER: _ClassVar[int]
    FC_STATUS_FIELD_NUMBER: _ClassVar[int]
    SENSOR_STATUS_FIELD_NUMBER: _ClassVar[int]
    ROLLSPEED_FIELD_NUMBER: _ClassVar[int]
    PITCHSPEED_FIELD_NUMBER: _ClassVar[int]
    YAWSPEED_FIELD_NUMBER: _ClassVar[int]
    ROLL_FIELD_NUMBER: _ClassVar[int]
    PITCH_FIELD_NUMBER: _ClassVar[int]
    YAW_FIELD_NUMBER: _ClassVar[int]
    FORWARD_RC_FIELD_NUMBER: _ClassVar[int]
    LATERAL_RC_FIELD_NUMBER: _ClassVar[int]
    VERTICAL_RC_FIELD_NUMBER: _ClassVar[int]
    YAW_RC_FIELD_NUMBER: _ClassVar[int]
    MOT1_EFF_FIELD_NUMBER: _ClassVar[int]
    MOT2_EFF_FIELD_NUMBER: _ClassVar[int]
    MOT3_EFF_FIELD_NUMBER: _ClassVar[int]
    MOT4_EFF_FIELD_NUMBER: _ClassVar[int]
    MOT5_EFF_FIELD_NUMBER: _ClassVar[int]
    MOT6_EFF_FIELD_NUMBER: _ClassVar[int]
    mode: Mode
    battery: float
    timestamp: _timestamp_pb2.Timestamp
    qr_side: Side
    depth: float
    fc_status: bool
    sensor_status: bool
    rollspeed: float
    pitchspeed: float
    yawspeed: float
    roll: float
    pitch: float
    yaw: float
    forward_rc: int
    lateral_rc: int
    vertical_rc: int
    yaw_rc: int
    mot1_eff: int
    mot2_eff: int
    mot3_eff: int
    mot4_eff: int
    mot5_eff: int
    mot6_eff: int
    def __init__(self, mode: _Optional[_Union[Mode, str]] = ..., battery: _Optional[float] = ..., timestamp: _Optional[_Union[datetime.datetime, _timestamp_pb2.Timestamp, _Mapping]] = ..., qr_side: _Optional[_Union[Side, str]] = ..., depth: _Optional[float] = ..., fc_status: _Optional[bool] = ..., sensor_status: _Optional[bool] = ..., rollspeed: _Optional[float] = ..., pitchspeed: _Optional[float] = ..., yawspeed: _Optional[float] = ..., roll: _Optional[float] = ..., pitch: _Optional[float] = ..., yaw: _Optional[float] = ..., forward_rc: _Optional[int] = ..., lateral_rc: _Optional[int] = ..., vertical_rc: _Optional[int] = ..., yaw_rc: _Optional[int] = ..., mot1_eff: _Optional[int] = ..., mot2_eff: _Optional[int] = ..., mot3_eff: _Optional[int] = ..., mot4_eff: _Optional[int] = ..., mot5_eff: _Optional[int] = ..., mot6_eff: _Optional[int] = ...) -> None: ...

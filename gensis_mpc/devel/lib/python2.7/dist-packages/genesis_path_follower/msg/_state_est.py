# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from genesis_path_follower/state_est.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class state_est(genpy.Message):
  _md5sum = "9c920bd35ee9bfa5fb5330660c621c0a"
  _type = "genesis_path_follower/state_est"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header

float64 lat      # latitude (deg)
float64 lon      # longitude (deg)

float64 x        # x coordinate (m)
float64 y        # y coordinate (m)
float64 psi      # yaw angle (rad)
float64 v        # speed (m/s)

float64 v_long   # longitidunal velocity (m/s)
float64 v_lat    # lateral velocity (m/s)
float64 yaw_rate # w_z, yaw rate (rad/s)

float64 a_long   # longitudinal acceleration (m/s^2)
float64 a_lat    # lateral acceleration (m/s^2)
float64 df       # front steering angle (rad)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','lat','lon','x','y','psi','v','v_long','v_lat','yaw_rate','a_long','a_lat','df']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,lat,lon,x,y,psi,v,v_long,v_lat,yaw_rate,a_long,a_lat,df

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(state_est, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.lat is None:
        self.lat = 0.
      if self.lon is None:
        self.lon = 0.
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.psi is None:
        self.psi = 0.
      if self.v is None:
        self.v = 0.
      if self.v_long is None:
        self.v_long = 0.
      if self.v_lat is None:
        self.v_lat = 0.
      if self.yaw_rate is None:
        self.yaw_rate = 0.
      if self.a_long is None:
        self.a_long = 0.
      if self.a_lat is None:
        self.a_lat = 0.
      if self.df is None:
        self.df = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.lat = 0.
      self.lon = 0.
      self.x = 0.
      self.y = 0.
      self.psi = 0.
      self.v = 0.
      self.v_long = 0.
      self.v_lat = 0.
      self.yaw_rate = 0.
      self.a_long = 0.
      self.a_lat = 0.
      self.df = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_12d().pack(_x.lat, _x.lon, _x.x, _x.y, _x.psi, _x.v, _x.v_long, _x.v_lat, _x.yaw_rate, _x.a_long, _x.a_lat, _x.df))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.lat, _x.lon, _x.x, _x.y, _x.psi, _x.v, _x.v_long, _x.v_lat, _x.yaw_rate, _x.a_long, _x.a_lat, _x.df,) = _get_struct_12d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_12d().pack(_x.lat, _x.lon, _x.x, _x.y, _x.psi, _x.v, _x.v_long, _x.v_lat, _x.yaw_rate, _x.a_long, _x.a_lat, _x.df))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.lat, _x.lon, _x.x, _x.y, _x.psi, _x.v, _x.v_long, _x.v_lat, _x.yaw_rate, _x.a_long, _x.a_lat, _x.df,) = _get_struct_12d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_12d = None
def _get_struct_12d():
    global _struct_12d
    if _struct_12d is None:
        _struct_12d = struct.Struct("<12d")
    return _struct_12d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I

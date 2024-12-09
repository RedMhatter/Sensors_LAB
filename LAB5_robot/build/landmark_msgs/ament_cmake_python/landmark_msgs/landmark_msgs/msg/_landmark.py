# generated from rosidl_generator_py/resource/_idl.py.em
# with input from landmark_msgs:msg/Landmark.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Landmark(type):
    """Metaclass of message 'Landmark'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('landmark_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'landmark_msgs.msg.Landmark')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__landmark
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__landmark
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__landmark
            cls._TYPE_SUPPORT = module.type_support_msg__msg__landmark
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__landmark

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'HAMMING__DEFAULT': 0,
            'GOODNESS__DEFAULT': 1.0,
            'DECISION_MARGIN__DEFAULT': 1.0,
        }

    @property
    def HAMMING__DEFAULT(cls):
        """Return default value for message field 'hamming'."""
        return 0

    @property
    def GOODNESS__DEFAULT(cls):
        """Return default value for message field 'goodness'."""
        return 1.0

    @property
    def DECISION_MARGIN__DEFAULT(cls):
        """Return default value for message field 'decision_margin'."""
        return 1.0


class Landmark(metaclass=Metaclass_Landmark):
    """Message class 'Landmark'."""

    __slots__ = [
        '_id',
        '_hamming',
        '_goodness',
        '_decision_margin',
        '_range',
        '_bearing',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'hamming': 'int32',
        'goodness': 'float',
        'decision_margin': 'float',
        'range': 'float',
        'bearing': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.hamming = kwargs.get(
            'hamming', Landmark.HAMMING__DEFAULT)
        self.goodness = kwargs.get(
            'goodness', Landmark.GOODNESS__DEFAULT)
        self.decision_margin = kwargs.get(
            'decision_margin', Landmark.DECISION_MARGIN__DEFAULT)
        self.range = kwargs.get('range', float())
        self.bearing = kwargs.get('bearing', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.hamming != other.hamming:
            return False
        if self.goodness != other.goodness:
            return False
        if self.decision_margin != other.decision_margin:
            return False
        if self.range != other.range:
            return False
        if self.bearing != other.bearing:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def hamming(self):
        """Message field 'hamming'."""
        return self._hamming

    @hamming.setter
    def hamming(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hamming' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'hamming' field must be an integer in [-2147483648, 2147483647]"
        self._hamming = value

    @builtins.property
    def goodness(self):
        """Message field 'goodness'."""
        return self._goodness

    @goodness.setter
    def goodness(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goodness' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'goodness' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._goodness = value

    @builtins.property
    def decision_margin(self):
        """Message field 'decision_margin'."""
        return self._decision_margin

    @decision_margin.setter
    def decision_margin(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'decision_margin' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'decision_margin' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._decision_margin = value

    @builtins.property  # noqa: A003
    def range(self):  # noqa: A003
        """Message field 'range'."""
        return self._range

    @range.setter  # noqa: A003
    def range(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'range' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._range = value

    @builtins.property
    def bearing(self):
        """Message field 'bearing'."""
        return self._bearing

    @bearing.setter
    def bearing(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bearing' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bearing' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bearing = value

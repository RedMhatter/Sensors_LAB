// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from landmark_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice
#include "landmark_msgs/msg/detail/landmark__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "landmark_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "landmark_msgs/msg/detail/landmark__struct.h"
#include "landmark_msgs/msg/detail/landmark__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Landmark__ros_msg_type = landmark_msgs__msg__Landmark;

static bool _Landmark__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Landmark__ros_msg_type * ros_message = static_cast<const _Landmark__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: hamming
  {
    cdr << ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr << ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr << ros_message->decision_margin;
  }

  // Field name: range
  {
    cdr << ros_message->range;
  }

  // Field name: bearing
  {
    cdr << ros_message->bearing;
  }

  return true;
}

static bool _Landmark__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Landmark__ros_msg_type * ros_message = static_cast<_Landmark__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: hamming
  {
    cdr >> ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr >> ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr >> ros_message->decision_margin;
  }

  // Field name: range
  {
    cdr >> ros_message->range;
  }

  // Field name: bearing
  {
    cdr >> ros_message->bearing;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_landmark_msgs
size_t get_serialized_size_landmark_msgs__msg__Landmark(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Landmark__ros_msg_type * ros_message = static_cast<const _Landmark__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hamming
  {
    size_t item_size = sizeof(ros_message->hamming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goodness
  {
    size_t item_size = sizeof(ros_message->goodness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name decision_margin
  {
    size_t item_size = sizeof(ros_message->decision_margin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name range
  {
    size_t item_size = sizeof(ros_message->range);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bearing
  {
    size_t item_size = sizeof(ros_message->bearing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Landmark__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_landmark_msgs__msg__Landmark(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_landmark_msgs
size_t max_serialized_size_landmark_msgs__msg__Landmark(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: hamming
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goodness
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: decision_margin
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: range
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bearing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = landmark_msgs__msg__Landmark;
    is_plain =
      (
      offsetof(DataType, bearing) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Landmark__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_landmark_msgs__msg__Landmark(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Landmark = {
  "landmark_msgs::msg",
  "Landmark",
  _Landmark__cdr_serialize,
  _Landmark__cdr_deserialize,
  _Landmark__get_serialized_size,
  _Landmark__max_serialized_size
};

static rosidl_message_type_support_t _Landmark__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Landmark,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, landmark_msgs, msg, Landmark)() {
  return &_Landmark__type_support;
}

#if defined(__cplusplus)
}
#endif

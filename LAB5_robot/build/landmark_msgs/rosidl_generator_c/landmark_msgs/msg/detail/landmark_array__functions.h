// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from landmark_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__FUNCTIONS_H_
#define LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "landmark_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "landmark_msgs/msg/detail/landmark_array__struct.h"

/// Initialize msg/LandmarkArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * landmark_msgs__msg__LandmarkArray
 * )) before or use
 * landmark_msgs__msg__LandmarkArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
bool
landmark_msgs__msg__LandmarkArray__init(landmark_msgs__msg__LandmarkArray * msg);

/// Finalize msg/LandmarkArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
void
landmark_msgs__msg__LandmarkArray__fini(landmark_msgs__msg__LandmarkArray * msg);

/// Create msg/LandmarkArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * landmark_msgs__msg__LandmarkArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
landmark_msgs__msg__LandmarkArray *
landmark_msgs__msg__LandmarkArray__create();

/// Destroy msg/LandmarkArray message.
/**
 * It calls
 * landmark_msgs__msg__LandmarkArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
void
landmark_msgs__msg__LandmarkArray__destroy(landmark_msgs__msg__LandmarkArray * msg);

/// Check for msg/LandmarkArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
bool
landmark_msgs__msg__LandmarkArray__are_equal(const landmark_msgs__msg__LandmarkArray * lhs, const landmark_msgs__msg__LandmarkArray * rhs);

/// Copy a msg/LandmarkArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
bool
landmark_msgs__msg__LandmarkArray__copy(
  const landmark_msgs__msg__LandmarkArray * input,
  landmark_msgs__msg__LandmarkArray * output);

/// Initialize array of msg/LandmarkArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * landmark_msgs__msg__LandmarkArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
bool
landmark_msgs__msg__LandmarkArray__Sequence__init(landmark_msgs__msg__LandmarkArray__Sequence * array, size_t size);

/// Finalize array of msg/LandmarkArray messages.
/**
 * It calls
 * landmark_msgs__msg__LandmarkArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
void
landmark_msgs__msg__LandmarkArray__Sequence__fini(landmark_msgs__msg__LandmarkArray__Sequence * array);

/// Create array of msg/LandmarkArray messages.
/**
 * It allocates the memory for the array and calls
 * landmark_msgs__msg__LandmarkArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
landmark_msgs__msg__LandmarkArray__Sequence *
landmark_msgs__msg__LandmarkArray__Sequence__create(size_t size);

/// Destroy array of msg/LandmarkArray messages.
/**
 * It calls
 * landmark_msgs__msg__LandmarkArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
void
landmark_msgs__msg__LandmarkArray__Sequence__destroy(landmark_msgs__msg__LandmarkArray__Sequence * array);

/// Check for msg/LandmarkArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
bool
landmark_msgs__msg__LandmarkArray__Sequence__are_equal(const landmark_msgs__msg__LandmarkArray__Sequence * lhs, const landmark_msgs__msg__LandmarkArray__Sequence * rhs);

/// Copy an array of msg/LandmarkArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_landmark_msgs
bool
landmark_msgs__msg__LandmarkArray__Sequence__copy(
  const landmark_msgs__msg__LandmarkArray__Sequence * input,
  landmark_msgs__msg__LandmarkArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LANDMARK_MSGS__MSG__DETAIL__LANDMARK_ARRAY__FUNCTIONS_H_

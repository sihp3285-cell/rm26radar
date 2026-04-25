// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice
#include "tensorrt_detect_msgs/msg/detail/radar_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
tensorrt_detect_msgs__msg__RadarMap__init(tensorrt_detect_msgs__msg__RadarMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tensorrt_detect_msgs__msg__RadarMap__fini(msg);
    return false;
  }
  // blue_x
  // blue_y
  // red_x
  // red_y
  return true;
}

void
tensorrt_detect_msgs__msg__RadarMap__fini(tensorrt_detect_msgs__msg__RadarMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // blue_x
  // blue_y
  // red_x
  // red_y
}

bool
tensorrt_detect_msgs__msg__RadarMap__are_equal(const tensorrt_detect_msgs__msg__RadarMap * lhs, const tensorrt_detect_msgs__msg__RadarMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // blue_x
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->blue_x[i] != rhs->blue_x[i]) {
      return false;
    }
  }
  // blue_y
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->blue_y[i] != rhs->blue_y[i]) {
      return false;
    }
  }
  // red_x
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->red_x[i] != rhs->red_x[i]) {
      return false;
    }
  }
  // red_y
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->red_y[i] != rhs->red_y[i]) {
      return false;
    }
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__RadarMap__copy(
  const tensorrt_detect_msgs__msg__RadarMap * input,
  tensorrt_detect_msgs__msg__RadarMap * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // blue_x
  for (size_t i = 0; i < 6; ++i) {
    output->blue_x[i] = input->blue_x[i];
  }
  // blue_y
  for (size_t i = 0; i < 6; ++i) {
    output->blue_y[i] = input->blue_y[i];
  }
  // red_x
  for (size_t i = 0; i < 6; ++i) {
    output->red_x[i] = input->red_x[i];
  }
  // red_y
  for (size_t i = 0; i < 6; ++i) {
    output->red_y[i] = input->red_y[i];
  }
  return true;
}

tensorrt_detect_msgs__msg__RadarMap *
tensorrt_detect_msgs__msg__RadarMap__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__RadarMap * msg = (tensorrt_detect_msgs__msg__RadarMap *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__RadarMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tensorrt_detect_msgs__msg__RadarMap));
  bool success = tensorrt_detect_msgs__msg__RadarMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tensorrt_detect_msgs__msg__RadarMap__destroy(tensorrt_detect_msgs__msg__RadarMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tensorrt_detect_msgs__msg__RadarMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tensorrt_detect_msgs__msg__RadarMap__Sequence__init(tensorrt_detect_msgs__msg__RadarMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__RadarMap * data = NULL;

  if (size) {
    data = (tensorrt_detect_msgs__msg__RadarMap *)allocator.zero_allocate(size, sizeof(tensorrt_detect_msgs__msg__RadarMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tensorrt_detect_msgs__msg__RadarMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tensorrt_detect_msgs__msg__RadarMap__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tensorrt_detect_msgs__msg__RadarMap__Sequence__fini(tensorrt_detect_msgs__msg__RadarMap__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tensorrt_detect_msgs__msg__RadarMap__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tensorrt_detect_msgs__msg__RadarMap__Sequence *
tensorrt_detect_msgs__msg__RadarMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__RadarMap__Sequence * array = (tensorrt_detect_msgs__msg__RadarMap__Sequence *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__RadarMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tensorrt_detect_msgs__msg__RadarMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tensorrt_detect_msgs__msg__RadarMap__Sequence__destroy(tensorrt_detect_msgs__msg__RadarMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tensorrt_detect_msgs__msg__RadarMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tensorrt_detect_msgs__msg__RadarMap__Sequence__are_equal(const tensorrt_detect_msgs__msg__RadarMap__Sequence * lhs, const tensorrt_detect_msgs__msg__RadarMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tensorrt_detect_msgs__msg__RadarMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__RadarMap__Sequence__copy(
  const tensorrt_detect_msgs__msg__RadarMap__Sequence * input,
  tensorrt_detect_msgs__msg__RadarMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tensorrt_detect_msgs__msg__RadarMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tensorrt_detect_msgs__msg__RadarMap * data =
      (tensorrt_detect_msgs__msg__RadarMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tensorrt_detect_msgs__msg__RadarMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tensorrt_detect_msgs__msg__RadarMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tensorrt_detect_msgs__msg__RadarMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

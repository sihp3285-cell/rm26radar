// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice
#include "tensorrt_detect_msgs/msg/detail/world_target_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `targets`
#include "tensorrt_detect_msgs/msg/detail/world_target__functions.h"

bool
tensorrt_detect_msgs__msg__WorldTargetArray__init(tensorrt_detect_msgs__msg__WorldTargetArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tensorrt_detect_msgs__msg__WorldTargetArray__fini(msg);
    return false;
  }
  // targets
  if (!tensorrt_detect_msgs__msg__WorldTarget__Sequence__init(&msg->targets, 0)) {
    tensorrt_detect_msgs__msg__WorldTargetArray__fini(msg);
    return false;
  }
  return true;
}

void
tensorrt_detect_msgs__msg__WorldTargetArray__fini(tensorrt_detect_msgs__msg__WorldTargetArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // targets
  tensorrt_detect_msgs__msg__WorldTarget__Sequence__fini(&msg->targets);
}

bool
tensorrt_detect_msgs__msg__WorldTargetArray__are_equal(const tensorrt_detect_msgs__msg__WorldTargetArray * lhs, const tensorrt_detect_msgs__msg__WorldTargetArray * rhs)
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
  // targets
  if (!tensorrt_detect_msgs__msg__WorldTarget__Sequence__are_equal(
      &(lhs->targets), &(rhs->targets)))
  {
    return false;
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__WorldTargetArray__copy(
  const tensorrt_detect_msgs__msg__WorldTargetArray * input,
  tensorrt_detect_msgs__msg__WorldTargetArray * output)
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
  // targets
  if (!tensorrt_detect_msgs__msg__WorldTarget__Sequence__copy(
      &(input->targets), &(output->targets)))
  {
    return false;
  }
  return true;
}

tensorrt_detect_msgs__msg__WorldTargetArray *
tensorrt_detect_msgs__msg__WorldTargetArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__WorldTargetArray * msg = (tensorrt_detect_msgs__msg__WorldTargetArray *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__WorldTargetArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tensorrt_detect_msgs__msg__WorldTargetArray));
  bool success = tensorrt_detect_msgs__msg__WorldTargetArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tensorrt_detect_msgs__msg__WorldTargetArray__destroy(tensorrt_detect_msgs__msg__WorldTargetArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tensorrt_detect_msgs__msg__WorldTargetArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__init(tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__WorldTargetArray * data = NULL;

  if (size) {
    data = (tensorrt_detect_msgs__msg__WorldTargetArray *)allocator.zero_allocate(size, sizeof(tensorrt_detect_msgs__msg__WorldTargetArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tensorrt_detect_msgs__msg__WorldTargetArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tensorrt_detect_msgs__msg__WorldTargetArray__fini(&data[i - 1]);
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
tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__fini(tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * array)
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
      tensorrt_detect_msgs__msg__WorldTargetArray__fini(&array->data[i]);
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

tensorrt_detect_msgs__msg__WorldTargetArray__Sequence *
tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * array = (tensorrt_detect_msgs__msg__WorldTargetArray__Sequence *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__WorldTargetArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__destroy(tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__are_equal(const tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * lhs, const tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tensorrt_detect_msgs__msg__WorldTargetArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__copy(
  const tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * input,
  tensorrt_detect_msgs__msg__WorldTargetArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tensorrt_detect_msgs__msg__WorldTargetArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tensorrt_detect_msgs__msg__WorldTargetArray * data =
      (tensorrt_detect_msgs__msg__WorldTargetArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tensorrt_detect_msgs__msg__WorldTargetArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tensorrt_detect_msgs__msg__WorldTargetArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tensorrt_detect_msgs__msg__WorldTargetArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tensorrt_detect_msgs:msg/WorldTarget.idl
// generated code does not contain a copyright notice
#include "tensorrt_detect_msgs/msg/detail/world_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tensorrt_detect_msgs__msg__WorldTarget__init(tensorrt_detect_msgs__msg__WorldTarget * msg)
{
  if (!msg) {
    return false;
  }
  // idx
  // class_id
  // team_id
  // score
  // valid
  // world_x
  // world_y
  // world_z
  // bbox_x
  // bbox_y
  // bbox_w
  // bbox_h
  return true;
}

void
tensorrt_detect_msgs__msg__WorldTarget__fini(tensorrt_detect_msgs__msg__WorldTarget * msg)
{
  if (!msg) {
    return;
  }
  // idx
  // class_id
  // team_id
  // score
  // valid
  // world_x
  // world_y
  // world_z
  // bbox_x
  // bbox_y
  // bbox_w
  // bbox_h
}

bool
tensorrt_detect_msgs__msg__WorldTarget__are_equal(const tensorrt_detect_msgs__msg__WorldTarget * lhs, const tensorrt_detect_msgs__msg__WorldTarget * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // idx
  if (lhs->idx != rhs->idx) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // team_id
  if (lhs->team_id != rhs->team_id) {
    return false;
  }
  // score
  if (lhs->score != rhs->score) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // world_x
  if (lhs->world_x != rhs->world_x) {
    return false;
  }
  // world_y
  if (lhs->world_y != rhs->world_y) {
    return false;
  }
  // world_z
  if (lhs->world_z != rhs->world_z) {
    return false;
  }
  // bbox_x
  if (lhs->bbox_x != rhs->bbox_x) {
    return false;
  }
  // bbox_y
  if (lhs->bbox_y != rhs->bbox_y) {
    return false;
  }
  // bbox_w
  if (lhs->bbox_w != rhs->bbox_w) {
    return false;
  }
  // bbox_h
  if (lhs->bbox_h != rhs->bbox_h) {
    return false;
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__WorldTarget__copy(
  const tensorrt_detect_msgs__msg__WorldTarget * input,
  tensorrt_detect_msgs__msg__WorldTarget * output)
{
  if (!input || !output) {
    return false;
  }
  // idx
  output->idx = input->idx;
  // class_id
  output->class_id = input->class_id;
  // team_id
  output->team_id = input->team_id;
  // score
  output->score = input->score;
  // valid
  output->valid = input->valid;
  // world_x
  output->world_x = input->world_x;
  // world_y
  output->world_y = input->world_y;
  // world_z
  output->world_z = input->world_z;
  // bbox_x
  output->bbox_x = input->bbox_x;
  // bbox_y
  output->bbox_y = input->bbox_y;
  // bbox_w
  output->bbox_w = input->bbox_w;
  // bbox_h
  output->bbox_h = input->bbox_h;
  return true;
}

tensorrt_detect_msgs__msg__WorldTarget *
tensorrt_detect_msgs__msg__WorldTarget__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__WorldTarget * msg = (tensorrt_detect_msgs__msg__WorldTarget *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__WorldTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tensorrt_detect_msgs__msg__WorldTarget));
  bool success = tensorrt_detect_msgs__msg__WorldTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tensorrt_detect_msgs__msg__WorldTarget__destroy(tensorrt_detect_msgs__msg__WorldTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tensorrt_detect_msgs__msg__WorldTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tensorrt_detect_msgs__msg__WorldTarget__Sequence__init(tensorrt_detect_msgs__msg__WorldTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__WorldTarget * data = NULL;

  if (size) {
    data = (tensorrt_detect_msgs__msg__WorldTarget *)allocator.zero_allocate(size, sizeof(tensorrt_detect_msgs__msg__WorldTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tensorrt_detect_msgs__msg__WorldTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tensorrt_detect_msgs__msg__WorldTarget__fini(&data[i - 1]);
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
tensorrt_detect_msgs__msg__WorldTarget__Sequence__fini(tensorrt_detect_msgs__msg__WorldTarget__Sequence * array)
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
      tensorrt_detect_msgs__msg__WorldTarget__fini(&array->data[i]);
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

tensorrt_detect_msgs__msg__WorldTarget__Sequence *
tensorrt_detect_msgs__msg__WorldTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__WorldTarget__Sequence * array = (tensorrt_detect_msgs__msg__WorldTarget__Sequence *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__WorldTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tensorrt_detect_msgs__msg__WorldTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tensorrt_detect_msgs__msg__WorldTarget__Sequence__destroy(tensorrt_detect_msgs__msg__WorldTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tensorrt_detect_msgs__msg__WorldTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tensorrt_detect_msgs__msg__WorldTarget__Sequence__are_equal(const tensorrt_detect_msgs__msg__WorldTarget__Sequence * lhs, const tensorrt_detect_msgs__msg__WorldTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tensorrt_detect_msgs__msg__WorldTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__WorldTarget__Sequence__copy(
  const tensorrt_detect_msgs__msg__WorldTarget__Sequence * input,
  tensorrt_detect_msgs__msg__WorldTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tensorrt_detect_msgs__msg__WorldTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tensorrt_detect_msgs__msg__WorldTarget * data =
      (tensorrt_detect_msgs__msg__WorldTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tensorrt_detect_msgs__msg__WorldTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tensorrt_detect_msgs__msg__WorldTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tensorrt_detect_msgs__msg__WorldTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

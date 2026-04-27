// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tensorrt_detect_msgs:msg/DetectionBox.idl
// generated code does not contain a copyright notice
#include "tensorrt_detect_msgs/msg/detail/detection_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tensorrt_detect_msgs__msg__DetectionBox__init(tensorrt_detect_msgs__msg__DetectionBox * msg)
{
  if (!msg) {
    return false;
  }
  // idx
  // confidence
  // x
  // y
  // width
  // height
  // armor_color
  // car_x
  // car_y
  // car_width
  // car_height
  // world_x
  // world_y
  // fps
  return true;
}

void
tensorrt_detect_msgs__msg__DetectionBox__fini(tensorrt_detect_msgs__msg__DetectionBox * msg)
{
  if (!msg) {
    return;
  }
  // idx
  // confidence
  // x
  // y
  // width
  // height
  // armor_color
  // car_x
  // car_y
  // car_width
  // car_height
  // world_x
  // world_y
  // fps
}

bool
tensorrt_detect_msgs__msg__DetectionBox__are_equal(const tensorrt_detect_msgs__msg__DetectionBox * lhs, const tensorrt_detect_msgs__msg__DetectionBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // idx
  if (lhs->idx != rhs->idx) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // armor_color
  if (lhs->armor_color != rhs->armor_color) {
    return false;
  }
  // car_x
  if (lhs->car_x != rhs->car_x) {
    return false;
  }
  // car_y
  if (lhs->car_y != rhs->car_y) {
    return false;
  }
  // car_width
  if (lhs->car_width != rhs->car_width) {
    return false;
  }
  // car_height
  if (lhs->car_height != rhs->car_height) {
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
  // fps
  if (lhs->fps != rhs->fps) {
    return false;
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__DetectionBox__copy(
  const tensorrt_detect_msgs__msg__DetectionBox * input,
  tensorrt_detect_msgs__msg__DetectionBox * output)
{
  if (!input || !output) {
    return false;
  }
  // idx
  output->idx = input->idx;
  // confidence
  output->confidence = input->confidence;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // armor_color
  output->armor_color = input->armor_color;
  // car_x
  output->car_x = input->car_x;
  // car_y
  output->car_y = input->car_y;
  // car_width
  output->car_width = input->car_width;
  // car_height
  output->car_height = input->car_height;
  // world_x
  output->world_x = input->world_x;
  // world_y
  output->world_y = input->world_y;
  // fps
  output->fps = input->fps;
  return true;
}

tensorrt_detect_msgs__msg__DetectionBox *
tensorrt_detect_msgs__msg__DetectionBox__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__DetectionBox * msg = (tensorrt_detect_msgs__msg__DetectionBox *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__DetectionBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tensorrt_detect_msgs__msg__DetectionBox));
  bool success = tensorrt_detect_msgs__msg__DetectionBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tensorrt_detect_msgs__msg__DetectionBox__destroy(tensorrt_detect_msgs__msg__DetectionBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tensorrt_detect_msgs__msg__DetectionBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tensorrt_detect_msgs__msg__DetectionBox__Sequence__init(tensorrt_detect_msgs__msg__DetectionBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__DetectionBox * data = NULL;

  if (size) {
    data = (tensorrt_detect_msgs__msg__DetectionBox *)allocator.zero_allocate(size, sizeof(tensorrt_detect_msgs__msg__DetectionBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tensorrt_detect_msgs__msg__DetectionBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tensorrt_detect_msgs__msg__DetectionBox__fini(&data[i - 1]);
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
tensorrt_detect_msgs__msg__DetectionBox__Sequence__fini(tensorrt_detect_msgs__msg__DetectionBox__Sequence * array)
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
      tensorrt_detect_msgs__msg__DetectionBox__fini(&array->data[i]);
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

tensorrt_detect_msgs__msg__DetectionBox__Sequence *
tensorrt_detect_msgs__msg__DetectionBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tensorrt_detect_msgs__msg__DetectionBox__Sequence * array = (tensorrt_detect_msgs__msg__DetectionBox__Sequence *)allocator.allocate(sizeof(tensorrt_detect_msgs__msg__DetectionBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tensorrt_detect_msgs__msg__DetectionBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tensorrt_detect_msgs__msg__DetectionBox__Sequence__destroy(tensorrt_detect_msgs__msg__DetectionBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tensorrt_detect_msgs__msg__DetectionBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tensorrt_detect_msgs__msg__DetectionBox__Sequence__are_equal(const tensorrt_detect_msgs__msg__DetectionBox__Sequence * lhs, const tensorrt_detect_msgs__msg__DetectionBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tensorrt_detect_msgs__msg__DetectionBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tensorrt_detect_msgs__msg__DetectionBox__Sequence__copy(
  const tensorrt_detect_msgs__msg__DetectionBox__Sequence * input,
  tensorrt_detect_msgs__msg__DetectionBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tensorrt_detect_msgs__msg__DetectionBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tensorrt_detect_msgs__msg__DetectionBox * data =
      (tensorrt_detect_msgs__msg__DetectionBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tensorrt_detect_msgs__msg__DetectionBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tensorrt_detect_msgs__msg__DetectionBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tensorrt_detect_msgs__msg__DetectionBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

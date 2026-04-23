#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "tensorrt_detect_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__DetectionBox() -> *const std::ffi::c_void;
}

#[link(name = "tensorrt_detect_msgs__rosidl_generator_c")]
extern "C" {
    fn tensorrt_detect_msgs__msg__DetectionBox__init(msg: *mut DetectionBox) -> bool;
    fn tensorrt_detect_msgs__msg__DetectionBox__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectionBox>, size: usize) -> bool;
    fn tensorrt_detect_msgs__msg__DetectionBox__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectionBox>);
    fn tensorrt_detect_msgs__msg__DetectionBox__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectionBox>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectionBox>) -> bool;
}

// Corresponds to tensorrt_detect_msgs__msg__DetectionBox
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectionBox {

    // This member is not documented.
    #[allow(missing_docs)]
    pub idx: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub width: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub height: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub armor_color: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub car_x: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub car_y: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub car_width: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub car_height: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub world_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub world_y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fps: f32,

}



impl Default for DetectionBox {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tensorrt_detect_msgs__msg__DetectionBox__init(&mut msg as *mut _) {
        panic!("Call to tensorrt_detect_msgs__msg__DetectionBox__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectionBox {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__DetectionBox__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__DetectionBox__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__DetectionBox__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectionBox {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectionBox where Self: Sized {
  const TYPE_NAME: &'static str = "tensorrt_detect_msgs/msg/DetectionBox";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__DetectionBox() }
  }
}


#[link(name = "tensorrt_detect_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__DetectionArray() -> *const std::ffi::c_void;
}

#[link(name = "tensorrt_detect_msgs__rosidl_generator_c")]
extern "C" {
    fn tensorrt_detect_msgs__msg__DetectionArray__init(msg: *mut DetectionArray) -> bool;
    fn tensorrt_detect_msgs__msg__DetectionArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectionArray>, size: usize) -> bool;
    fn tensorrt_detect_msgs__msg__DetectionArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectionArray>);
    fn tensorrt_detect_msgs__msg__DetectionArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectionArray>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectionArray>) -> bool;
}

// Corresponds to tensorrt_detect_msgs__msg__DetectionArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DetectionBox>,

}



impl Default for DetectionArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tensorrt_detect_msgs__msg__DetectionArray__init(&mut msg as *mut _) {
        panic!("Call to tensorrt_detect_msgs__msg__DetectionArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectionArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__DetectionArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__DetectionArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__DetectionArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectionArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectionArray where Self: Sized {
  const TYPE_NAME: &'static str = "tensorrt_detect_msgs/msg/DetectionArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__DetectionArray() }
  }
}


#[link(name = "tensorrt_detect_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__WorldTarget() -> *const std::ffi::c_void;
}

#[link(name = "tensorrt_detect_msgs__rosidl_generator_c")]
extern "C" {
    fn tensorrt_detect_msgs__msg__WorldTarget__init(msg: *mut WorldTarget) -> bool;
    fn tensorrt_detect_msgs__msg__WorldTarget__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<WorldTarget>, size: usize) -> bool;
    fn tensorrt_detect_msgs__msg__WorldTarget__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<WorldTarget>);
    fn tensorrt_detect_msgs__msg__WorldTarget__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<WorldTarget>, out_seq: *mut rosidl_runtime_rs::Sequence<WorldTarget>) -> bool;
}

// Corresponds to tensorrt_detect_msgs__msg__WorldTarget
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WorldTarget {

    // This member is not documented.
    #[allow(missing_docs)]
    pub idx: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub class_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub score: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub world_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub world_y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub world_z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bbox_x: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bbox_y: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bbox_w: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bbox_h: i32,

}



impl Default for WorldTarget {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tensorrt_detect_msgs__msg__WorldTarget__init(&mut msg as *mut _) {
        panic!("Call to tensorrt_detect_msgs__msg__WorldTarget__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for WorldTarget {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__WorldTarget__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__WorldTarget__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__WorldTarget__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for WorldTarget {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for WorldTarget where Self: Sized {
  const TYPE_NAME: &'static str = "tensorrt_detect_msgs/msg/WorldTarget";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__WorldTarget() }
  }
}


#[link(name = "tensorrt_detect_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__WorldTargetArray() -> *const std::ffi::c_void;
}

#[link(name = "tensorrt_detect_msgs__rosidl_generator_c")]
extern "C" {
    fn tensorrt_detect_msgs__msg__WorldTargetArray__init(msg: *mut WorldTargetArray) -> bool;
    fn tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<WorldTargetArray>, size: usize) -> bool;
    fn tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<WorldTargetArray>);
    fn tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<WorldTargetArray>, out_seq: *mut rosidl_runtime_rs::Sequence<WorldTargetArray>) -> bool;
}

// Corresponds to tensorrt_detect_msgs__msg__WorldTargetArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WorldTargetArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub targets: rosidl_runtime_rs::Sequence<super::super::msg::rmw::WorldTarget>,

}



impl Default for WorldTargetArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tensorrt_detect_msgs__msg__WorldTargetArray__init(&mut msg as *mut _) {
        panic!("Call to tensorrt_detect_msgs__msg__WorldTargetArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for WorldTargetArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tensorrt_detect_msgs__msg__WorldTargetArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for WorldTargetArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for WorldTargetArray where Self: Sized {
  const TYPE_NAME: &'static str = "tensorrt_detect_msgs/msg/WorldTargetArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tensorrt_detect_msgs__msg__WorldTargetArray() }
  }
}



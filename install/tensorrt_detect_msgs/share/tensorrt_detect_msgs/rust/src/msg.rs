#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to tensorrt_detect_msgs__msg__DetectionBox

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DetectionBox::default())
  }
}

impl rosidl_runtime_rs::Message for DetectionBox {
  type RmwMsg = super::msg::rmw::DetectionBox;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        idx: msg.idx,
        confidence: msg.confidence,
        x: msg.x,
        y: msg.y,
        width: msg.width,
        height: msg.height,
        armor_color: msg.armor_color,
        car_x: msg.car_x,
        car_y: msg.car_y,
        car_width: msg.car_width,
        car_height: msg.car_height,
        world_x: msg.world_x,
        world_y: msg.world_y,
        fps: msg.fps,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      idx: msg.idx,
      confidence: msg.confidence,
      x: msg.x,
      y: msg.y,
      width: msg.width,
      height: msg.height,
      armor_color: msg.armor_color,
      car_x: msg.car_x,
      car_y: msg.car_y,
      car_width: msg.car_width,
      car_height: msg.car_height,
      world_x: msg.world_x,
      world_y: msg.world_y,
      fps: msg.fps,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      idx: msg.idx,
      confidence: msg.confidence,
      x: msg.x,
      y: msg.y,
      width: msg.width,
      height: msg.height,
      armor_color: msg.armor_color,
      car_x: msg.car_x,
      car_y: msg.car_y,
      car_width: msg.car_width,
      car_height: msg.car_height,
      world_x: msg.world_x,
      world_y: msg.world_y,
      fps: msg.fps,
    }
  }
}


// Corresponds to tensorrt_detect_msgs__msg__DetectionArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: Vec<super::msg::DetectionBox>,

}



impl Default for DetectionArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DetectionArray::default())
  }
}

impl rosidl_runtime_rs::Message for DetectionArray {
  type RmwMsg = super::msg::rmw::DetectionArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detections: msg.detections
          .into_iter()
          .map(|elem| super::msg::DetectionBox::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detections: msg.detections
          .iter()
          .map(|elem| super::msg::DetectionBox::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detections: msg.detections
          .into_iter()
          .map(super::msg::DetectionBox::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to tensorrt_detect_msgs__msg__WorldTarget

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::WorldTarget::default())
  }
}

impl rosidl_runtime_rs::Message for WorldTarget {
  type RmwMsg = super::msg::rmw::WorldTarget;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        idx: msg.idx,
        class_id: msg.class_id,
        score: msg.score,
        valid: msg.valid,
        world_x: msg.world_x,
        world_y: msg.world_y,
        world_z: msg.world_z,
        bbox_x: msg.bbox_x,
        bbox_y: msg.bbox_y,
        bbox_w: msg.bbox_w,
        bbox_h: msg.bbox_h,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      idx: msg.idx,
      class_id: msg.class_id,
      score: msg.score,
      valid: msg.valid,
      world_x: msg.world_x,
      world_y: msg.world_y,
      world_z: msg.world_z,
      bbox_x: msg.bbox_x,
      bbox_y: msg.bbox_y,
      bbox_w: msg.bbox_w,
      bbox_h: msg.bbox_h,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      idx: msg.idx,
      class_id: msg.class_id,
      score: msg.score,
      valid: msg.valid,
      world_x: msg.world_x,
      world_y: msg.world_y,
      world_z: msg.world_z,
      bbox_x: msg.bbox_x,
      bbox_y: msg.bbox_y,
      bbox_w: msg.bbox_w,
      bbox_h: msg.bbox_h,
    }
  }
}


// Corresponds to tensorrt_detect_msgs__msg__WorldTargetArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WorldTargetArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub targets: Vec<super::msg::WorldTarget>,

}



impl Default for WorldTargetArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::WorldTargetArray::default())
  }
}

impl rosidl_runtime_rs::Message for WorldTargetArray {
  type RmwMsg = super::msg::rmw::WorldTargetArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        targets: msg.targets
          .into_iter()
          .map(|elem| super::msg::WorldTarget::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        targets: msg.targets
          .iter()
          .map(|elem| super::msg::WorldTarget::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      targets: msg.targets
          .into_iter()
          .map(super::msg::WorldTarget::from_rmw_message)
          .collect(),
    }
  }
}



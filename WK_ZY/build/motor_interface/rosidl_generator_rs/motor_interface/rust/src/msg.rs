#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to motor_interface__msg__JointMotor

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointMotor {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_names: [std::string::String; 23],


    // This member is not documented.
    #[allow(missing_docs)]
    pub kp: [f64; 23],


    // This member is not documented.
    #[allow(missing_docs)]
    pub kd: [f64; 23],


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: [f64; 23],


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: [f64; 23],


    // This member is not documented.
    #[allow(missing_docs)]
    pub effort: [f64; 23],

}



impl Default for JointMotor {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::JointMotor::default())
  }
}

impl rosidl_runtime_rs::Message for JointMotor {
  type RmwMsg = super::msg::rmw::JointMotor;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        joint_names: msg.joint_names
          .map(|elem| elem.as_str().into()),
        kp: msg.kp,
        kd: msg.kd,
        position: msg.position,
        velocity: msg.velocity,
        effort: msg.effort,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        joint_names: msg.joint_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect::<Vec<_>>()
          .try_into()
          .unwrap(),
        kp: msg.kp,
        kd: msg.kd,
        position: msg.position,
        velocity: msg.velocity,
        effort: msg.effort,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      joint_names: msg.joint_names
        .map(|elem| elem.to_string()),
      kp: msg.kp,
      kd: msg.kd,
      position: msg.position,
      velocity: msg.velocity,
      effort: msg.effort,
    }
  }
}



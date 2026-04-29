#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "motor_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__motor_interface__msg__JointMotor() -> *const std::ffi::c_void;
}

#[link(name = "motor_interface__rosidl_generator_c")]
extern "C" {
    fn motor_interface__msg__JointMotor__init(msg: *mut JointMotor) -> bool;
    fn motor_interface__msg__JointMotor__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointMotor>, size: usize) -> bool;
    fn motor_interface__msg__JointMotor__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointMotor>);
    fn motor_interface__msg__JointMotor__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointMotor>, out_seq: *mut rosidl_runtime_rs::Sequence<JointMotor>) -> bool;
}

// Corresponds to motor_interface__msg__JointMotor
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointMotor {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_names: [rosidl_runtime_rs::String; 23],


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !motor_interface__msg__JointMotor__init(&mut msg as *mut _) {
        panic!("Call to motor_interface__msg__JointMotor__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointMotor {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_interface__msg__JointMotor__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_interface__msg__JointMotor__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_interface__msg__JointMotor__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointMotor {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointMotor where Self: Sized {
  const TYPE_NAME: &'static str = "motor_interface/msg/JointMotor";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__motor_interface__msg__JointMotor() }
  }
}



#include "DeviceDriver/EteeDriver.h"

#include <algorithm>
#include <utility>

#include "Util/DriverLog.h"
#include "Util/Math.h"

EteeDeviceDriver::EteeDeviceDriver(
    VRDeviceConfiguration configuration, std::unique_ptr<BoneAnimator>&& boneAnimator, std::function<void(const DeviceEvent& deviceEvent)> deviceEventCallback)
    : m_configuration(configuration),
      m_boneAnimator(std::move(boneAnimator)),
      m_driverId(-1),
      m_isActive(false),
      m_lastInput(false),
      m_touchState(0),
      m_deviceEventCallback(std::move(deviceEventCallback)),
      m_controllerPose(std::make_unique<ControllerPose>(m_configuration.pose)) {}

bool EteeDeviceDriver::IsRightHand() const {
  return m_configuration.role == vr::TrackedControllerRole_RightHand;
}

std::string EteeDeviceDriver::GetSerialNumber() {
  return IsRightHand() ? "ETEE-7NQD0123R" : "ETEE-7NQD0123L";
};

bool EteeDeviceDriver::IsActive() {
  return m_isActive;
};

const VRDeviceConfiguration& EteeDeviceDriver::GetConfiguration() {
  return m_configuration;
};

vr::EVRInitError EteeDeviceDriver::Activate(uint32_t unObjectId) {
  DriverLog("Starting activation of id: %i", unObjectId);
  const bool isRightHand = IsRightHand();

  m_driverId = unObjectId;

  m_props = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_driverId);

  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_SerialNumber_String, GetSerialNumber().c_str());
  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_WillDriftInYaw_Bool, false);
  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_DeviceIsWireless_Bool, true);
  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_DeviceIsCharging_Bool, false);

  vr::HmdMatrix34_t l_matrix = {-1.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f};
  vr::VRProperties()->SetProperty(m_props, vr::Prop_StatusDisplayTransform_Matrix34, &l_matrix, sizeof(vr::HmdMatrix34_t), vr::k_unHmdMatrix34PropertyTag);

  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_Firmware_UpdateAvailable_Bool, false);
  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_DeviceCanPowerOff_Bool, true);

  vr::VRProperties()->SetInt32Property(m_props, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_Controller);
  vr::VRProperties()->SetBoolProperty(m_props, vr::Prop_Identifiable_Bool, true);
//  vr::VRProperties()->SetInt32Property(m_props, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad); // Overridden
//  vr::VRProperties()->SetInt32Property(m_props, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
  vr::VRProperties()->SetInt32Property(
      m_props, vr::Prop_ControllerRoleHint_Int32, IsRightHand() ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
  vr::VRProperties()->SetInt32Property(m_props, vr::Prop_ControllerHandSelectionPriority_Int32, (int32_t)7002);
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_ModelNumber_String, IsRightHand() ? "etee right" : "etee left"); // Overridden
  vr::VRProperties()->SetStringProperty(
      m_props, vr::Prop_RenderModelName_String, IsRightHand() ? "{etee}/rendermodels/etee_controller_right" : "{etee}/rendermodels/etee_controller_left");
//  vr::VRProperties()->SetStringProperty( // Overridden
//      m_props,
//      vr::Prop_ManufacturerName_String,
//      "etee");  // anything other than TG0 (what the manufacturer of the tracker is, as we use this to get the pose)
  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_ResourceRoot_String, "etee_controller");
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_InputProfilePath_String, "{etee}/input/etee_controller_profile.json"); // Overridden
//  vr::VRProperties()->SetInt32Property(m_props, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_ControllerType_String, "etee_controller"); // Overridden

//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_TrackingSystemName_String, "indexcontroller");
  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_ManufacturerName_String, "Valve");
  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_ModelNumber_String, IsRightHand() ? "Knuckles (Right Controller)" : "Knuckles (Left Controller)");
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_RenderModelName_String, IsRightHand() ? "{indexcontroller}valve_controller_knu_1_0_left" : "{indexcontroller}valve_controller_knu_1_0_right");
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_SerialNumber_String, "ALVR Remote Controller");
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_AttachedDeviceId_String, "ALVR Remote Controller");
//  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_RegisteredDeviceType_String, IsRightHand() ? "valve/index_controllerLHR-E217CD00_Right" : "valve/index_controllerLHR-E217CD00_Left");
//  uint64_t supportedButtons = 0xFFFFFFFFFFFFFFFFULL;
//  vr::VRProperties()->SetUint64Property(m_props, vr::Prop_SupportedButtons_Uint64, supportedButtons);
  vr::VRProperties()->SetInt32Property(m_props, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick);
  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_ControllerType_String, "knuckles");
  vr::VRProperties()->SetStringProperty(m_props, vr::Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");

  // Inputs | These interact without the specified input profile so the paths need to be equivalent

  // System Info
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/system/click", &m_inputComponentHandles[ComponentIndex::SYSTEM_CLICK]);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/tracker_connection/click", &m_inputComponentHandles[ComponentIndex::TRACKERCONNECTION_CLICK]);

  // Buttons
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/a/click", &m_inputComponentHandles[ComponentIndex::PINCH_A_CLICK]);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/a/touch", &m_inputComponentHandles[ComponentIndex::PINCH_A_VALUE]);

  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/b/click", &m_inputComponentHandles[ComponentIndex::PINCH_B_CLICK]);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/b/touch", &m_inputComponentHandles[ComponentIndex::PINCH_B_VALUE]);

  // Thumbstick
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/thumbstick/x", &m_inputComponentHandles[ComponentIndex::TRACKPAD_X], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/thumbstick/y", &m_inputComponentHandles[ComponentIndex::TRACKPAD_Y], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/thumbstick/click", &m_inputComponentHandles[ComponentIndex::TRACKPAD_CLICK]);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/thumbstick/touch", &m_inputComponentHandles[ComponentIndex::TRACKPAD_TOUCH]);

  // Trackpad
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/trackpad/x", &m_inputComponentHandles[ComponentIndex::SLIDER_SIMULATED_X], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/trackpad/y", &m_inputComponentHandles[ComponentIndex::SLIDER_VALUE], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/trackpad/touch", &m_inputComponentHandles[ComponentIndex::SLIDER_TOUCH]);
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/trackpad/force", &m_inputComponentHandles[ComponentIndex::SLIDER_CLICK], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

  // Fingers
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/finger/index", &m_inputComponentHandles[ComponentIndex::INDEX_PULL], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/trigger/value", &m_inputComponentHandles[ComponentIndex::INDEX_FORCE], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/trigger/click", &m_inputComponentHandles[ComponentIndex::INDEX_CLICK]);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/trigger/touch", &m_inputComponentHandles[ComponentIndex::INDEX_TOUCH]);

  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/finger/middle", &m_inputComponentHandles[ComponentIndex::MIDDLE_PULL], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/finger/ring", &m_inputComponentHandles[ComponentIndex::RING_PULL], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/finger/pinky", &m_inputComponentHandles[ComponentIndex::PINKY_PULL], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

  // Grip Gesture
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/grip/value", &m_inputComponentHandles[ComponentIndex::GRIP_VALUE], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
  vr::VRDriverInput()->CreateScalarComponent(
      m_props, "/input/grip/force", &m_inputComponentHandles[ComponentIndex::GRIP_FORCE], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
  vr::VRDriverInput()->CreateBooleanComponent(m_props, "/input/grip/touch", &m_inputComponentHandles[ComponentIndex::GRIP_TOUCH]);

  // Haptics
  vr::VRDriverInput()->CreateHapticComponent(m_props, "/output/haptic", &m_inputComponentHandles[ComponentIndex::HAPTIC]);

  // Icons
  vr::VRProperties()->SetStringProperty(
      m_props, vr::Prop_NamedIconPathDeviceOff_String, IsRightHand() ? "{etee}/icons/right_controller_status_off.png" : "{etee}/icons/left_controller_status_off.png");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceSearching_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_searching.gif" : "{etee}/icons/left_controller_status_searching.gif");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceSearchingAlert_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_searching_alert.gif" : "{etee}/icons/left_controller_status_searching_alert.gif");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceReady_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_ready.png" : "{etee}/icons/left_controller_status_ready.png");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceReadyAlert_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_ready_alert.png" : "{etee}/icons/left_controller_status_ready_alert.png");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceNotReady_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_error.png" : "{etee}/icons/left_controller_status_error.png");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceStandby_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_off.png" : "{etee}/icons/left_controller_status_off.png");
  vr::VRProperties()->SetStringProperty(
      m_props,
      vr::Prop_NamedIconPathDeviceAlertLow_String,
      IsRightHand() ? "{etee}/icons/right_controller_status_ready_low.png" : "{etee}/icons/left_controller_status_ready_low.png");

  {
    vr::VRBoneTransform_t gripLimitTransforms[NUM_BONES];
    m_boneAnimator->LoadGripLimitSkeletonByHand(gripLimitTransforms, isRightHand);
    // Skeleton
    vr::EVRInputError error = vr::VRDriverInput()->CreateSkeletonComponent(
        m_props,
        isRightHand ? "/input/skeleton/right" : "/input/skeleton/left",
        isRightHand ? "/skeleton/hand/right" : "/skeleton/hand/left",
        "/pose/raw",
        vr::VRSkeletalTracking_Partial,
        gripLimitTransforms,
        NUM_BONES,
        &m_inputComponentHandles[ComponentIndex::SKELETON]);

    if (error != vr::VRInputError_None) {
      DriverLog("CreateSkeletonComponent failed.  Error: %s\n", error);
    }
  }

  StartDevice();

  m_boneAnimator->LoadDefaultSkeletonByHand(m_handTransforms, isRightHand);

  // SteamVR wants input at startup for skeletons
  vr::VRDriverInput()->UpdateSkeletonComponent(
      m_inputComponentHandles[ComponentIndex::SKELETON], vr::VRSkeletalMotionRange_WithoutController, m_handTransforms, NUM_BONES);
  vr::VRDriverInput()->UpdateSkeletonComponent(m_inputComponentHandles[ComponentIndex::SKELETON], vr::VRSkeletalMotionRange_WithController, m_handTransforms, NUM_BONES);

  m_isActive = true;

  return vr::VRInitError_None;
}

void EteeDeviceDriver::StartDevice() {
  DriverLog("Etee controller successfully initialised");

  vr::DriverPose_t pose = m_controllerPose->GetStatusPose();
  vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_driverId, pose, sizeof(vr::DriverPose_t));

  m_poseUpdateThread = std::thread(&EteeDeviceDriver::PoseUpdateThread, this);

  m_inputUpdateThread = std::thread(&EteeDeviceDriver::InputUpdateThread, this);
}

void EteeDeviceDriver::OnInputUpdate(VRCommInputData_t data) {
  // System Info
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::SYSTEM_CLICK], data.system.systemClick, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::TRACKERCONNECTION_CLICK], data.system.trackerConnection, 0);

  // Touchpad Logic
  #define DEG_TO_RAD (3.1415926535897932384626433832795028841971693993751058209749445923078164062/180)
  #define TOUCHPAD_ACTIVE_FORCE 0.02
  #define TOUCHPAD_INACTIVE_FORCE 0.01
  #define TOUCHPAD_CLICK_FORCE 0.10
  #define TOUCHPAD_RELEASE_FORCE 0.05
  #define TRACKPAD_LOWER_ANGLE 310
  #define THUMBSTICK_THRESHOLD_VALUE (1/3)
  #define BUTTONS_START_ANGLE 130
  #define BUTTONS_B_TO_BOTH 170
  #define BUTTONS_BOTH_TO_A 190
  #define BUTTONS_END_ANGLE 230
  #define SYSTEM_END_ANGLE 270
  #define TRACKPAD_UPPER_ANGLE 50
  #define TRACKPAD_LOWER_ANGLE 310

  float touchAngle = atan2(data.thumbpad.y, IsRightHand() ? data.thumbpad.x : -data.thumbpad.x);
  float touchValue = data.thumbpad.touch ? sqrt(pow(data.thumbpad.x, 2) + pow(data.thumbpad.y, 2)) : 0;

  float thumbstick_x = 0;
  float thumbstick_y = 0;
  bool thumbstick_click = false;
  bool thumbstick_touch = false;

  float trackpad_x = 0;
  float trackpad_y = 0;
  bool trackpad_touch = false;
  float trackpad_force = 0;

  bool button_a_click = false;
  bool button_a_touch = false;

  bool button_b_click = false;
  bool button_b_touch = false;

  bool system_click = false;

  // touchState Touched zone
  if (data.thumbpad.force > TOUCHPAD_ACTIVE_FORCE && (m_touchState & 1) == 0) { // Lock to zone on press
    if (touchValue < THUMBSTICK_THRESHOLD_VALUE) { // Thumbstick
      m_touchState |= 1 << 1 | 1;
    }
    else if (touchAngle < TRACKPAD_UPPER_ANGLE * DEG_TO_RAD || touchAngle > TRACKPAD_LOWER_ANGLE * DEG_TO_RAD) { // Trackpad
      m_touchState |= 2 << 1 | 1;
    }
    else if (touchAngle > BUTTONS_START_ANGLE * DEG_TO_RAD && touchAngle < BUTTONS_END_ANGLE * DEG_TO_RAD) { // Buttons
      m_touchState |= 3 << 1 | 1;
    }
    else if (touchAngle > BUTTONS_END_ANGLE * DEG_TO_RAD && touchAngle < SYSTEM_END_ANGLE * DEG_TO_RAD) { // System
      m_touchState |= 4 << 1 | 1;
    }
    else { // Invalid zone
      m_touchState |= 1;
    }
  }
  else if (data.thumbpad.force < TOUCHPAD_INACTIVE_FORCE && (m_touchState & 1) == 1) { // Unlock zone
    m_touchState ^= m_touchState & 15;
  }

  // touchState Clicked
  if (data.thumbpad.force > TOUCHPAD_CLICK_FORCE && (m_touchState & 16) == 0) { // Click latch
    m_touchState |= 16;
  }
  else if (data.thumbpad.force < TOUCHPAD_RELEASE_FORCE && (m_touchState & 16) == 16) {
    m_touchState ^= m_touchState & 16;
  }

  bool touchClick = (m_touchState & 16) == 16 ? true : false;

  // Force active zone
  switch ((m_touchState >> 1) & 15) { // Limit values for different ranges
    case 2: // Trackpad
      touchValue = touchValue < THUMBSTICK_THRESHOLD_VALUE ? THUMBSTICK_THRESHOLD_VALUE : touchValue;
      touchAngle = touchAngle < 180 ? // Clamp to Trackpad range
        (touchAngle > TRACKPAD_UPPER_ANGLE ? TRACKPAD_UPPER_ANGLE : touchAngle)
        : (touchAngle < TRACKPAD_LOWER_ANGLE ? TRACKPAD_LOWER_ANGLE : touchAngle);
      break;
    case 3: // Buttons
      touchValue = touchValue < THUMBSTICK_THRESHOLD_VALUE ? THUMBSTICK_THRESHOLD_VALUE : touchValue;
      touchAngle = touchAngle > BUTTONS_START_ANGLE ? // Clamp to Buttons range
        (touchAngle < BUTTONS_END_ANGLE ? touchValue : BUTTONS_END_ANGLE)
        : BUTTONS_START_ANGLE;
      break;
    case 4: // System
      touchValue = touchValue < THUMBSTICK_THRESHOLD_VALUE ? THUMBSTICK_THRESHOLD_VALUE : touchValue;
      touchAngle = touchAngle > BUTTONS_END_ANGLE ? // Clamp to System range
        (touchAngle < SYSTEM_END_ANGLE ? touchValue : SYSTEM_END_ANGLE)
        : BUTTONS_END_ANGLE;
      break;
  }

  // Set values for active zone
  if ((m_touchState & 15) == (1 << 1 | 1)) { // Thumbstick
    thumbstick_x = data.thumbpad.x;
    thumbstick_y = data.thumbpad.y;
    thumbstick_click = touchClick;
    thumbstick_touch = true;
  }
  else if (touchValue < THUMBSTICK_THRESHOLD_VALUE) { // Thumbstick, not active
    thumbstick_touch = data.thumbpad.touch;
  }
  else if (touchAngle <= TRACKPAD_UPPER_ANGLE * DEG_TO_RAD || touchAngle >= TRACKPAD_LOWER_ANGLE * DEG_TO_RAD) { // Trackpad
    float trackValue = IsRightHand() ? touchValue : -touchValue; // (THUMBSTICK_THRESHOLD_VALUE, 1)
    trackpad_x = (trackValue - THUMBSTICK_THRESHOLD_VALUE) / (1 - THUMBSTICK_THRESHOLD_VALUE) * 2 - 1; // Normalize to (-1, 1) range
    float trackAngle = (touchAngle >= 0 ? // Convert touchAngle to (0, total of TRACKPAD_ANGLE)
      touchAngle + ((360 - TRACKPAD_LOWER_ANGLE) * DEG_TO_RAD)
      : touchAngle - TRACKPAD_LOWER_ANGLE * DEG_TO_RAD);
    trackpad_y = trackAngle / ((TRACKPAD_UPPER_ANGLE + (360 - TRACKPAD_LOWER_ANGLE)) * DEG_TO_RAD) * 2 - 1; // Normalize to (-1, 1) range
    trackpad_touch = true;
    trackpad_force = data.thumbpad.force;
  }
  else if (touchAngle > BUTTONS_START_ANGLE * DEG_TO_RAD && touchAngle < BUTTONS_B_TO_BOTH * DEG_TO_RAD) { // B only
    button_b_touch = true;
    button_b_click = touchClick;
  }
  else if (touchAngle > BUTTONS_B_TO_BOTH * DEG_TO_RAD && touchAngle < BUTTONS_BOTH_TO_A * DEG_TO_RAD) { // A and B
    button_a_touch = true;
    button_a_click = touchClick;
    button_b_touch = true;
    button_b_click = touchClick;
  }
  else if (touchAngle > BUTTONS_BOTH_TO_A * DEG_TO_RAD && touchAngle < BUTTONS_END_ANGLE * DEG_TO_RAD) { // A only
    button_a_touch = true;
    button_a_click = touchClick;
  }
  else if (touchAngle > BUTTONS_END_ANGLE * DEG_TO_RAD && touchAngle < SYSTEM_END_ANGLE * DEG_TO_RAD) { // System
    system_click = touchClick;
  }

  // Thumbstick
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::TRACKPAD_X], thumbstick_x, 0);
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::TRACKPAD_Y], thumbstick_y, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::TRACKPAD_TOUCH], thumbstick_touch, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::TRACKPAD_CLICK], thumbstick_click, 0);

  // Trackpad
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::SLIDER_VALUE], trackpad_y, 0);
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::SLIDER_SIMULATED_X], trackpad_x, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::SLIDER_TOUCH], trackpad_touch, 0);
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::SLIDER_CLICK], trackpad_force, 0);

  // Buttons
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::PINCH_A_CLICK], button_a_click, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::PINCH_A_VALUE], button_a_touch, 0);

  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::PINCH_B_CLICK], button_b_click, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::PINCH_B_VALUE], button_b_touch, 0);

  // System
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::SYSTEM_CLICK], system_click, 0);

  // Fingers
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::INDEX_PULL], data.fingers[1].pull, 0);
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::INDEX_FORCE], data.fingers[1].force, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::INDEX_CLICK], data.fingers[1].click, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::INDEX_TOUCH], data.fingers[1].touch, 0);

  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::MIDDLE_PULL], data.fingers[2].pull, 0);
//  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::MIDDLE_FORCE], data.fingers[2].force, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::MIDDLE_CLICK], data.fingers[2].click, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::MIDDLE_TOUCH], data.fingers[2].touch, 0);

  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::RING_PULL], data.fingers[3].pull, 0);
//  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::RING_FORCE], data.fingers[3].force, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::RING_CLICK], data.fingers[3].click, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::RING_TOUCH], data.fingers[3].touch, 0);

  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::PINKY_PULL], data.fingers[4].pull, 0);
//  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::PINKY_FORCE], data.fingers[4].force, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::PINKY_CLICK], data.fingers[4].click, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::PINKY_TOUCH], data.fingers[4].touch, 0);

  // Grip gesture
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::GRIP_VALUE], data.gesture.gripPull, 0);
  vr::VRDriverInput()->UpdateScalarComponent(m_inputComponentHandles[ComponentIndex::GRIP_FORCE], data.gesture.gripForce, 0);
  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::GRIP_TOUCH], data.gesture.gripTouch, 0);
//  vr::VRDriverInput()->UpdateBooleanComponent(m_inputComponentHandles[ComponentIndex::GRIP_CLICK], data.gesture.gripClick, 0);

  // Battery percentage
  vr::VRProperties()->SetFloatProperty(m_props, vr::Prop_DeviceBatteryPercentage_Float, data.system.battery);

  if (m_lastInput.system.trackerConnection != data.system.trackerConnection) {
    m_controllerPose->SetEteeTrackerIsConnected(data.system.trackerConnection);
  }

  {
    std::lock_guard<std::mutex> lock(m_inputMutex);

    m_lastInput = data;
  }

#ifdef _DEBUG
  {
    const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    if (now > m_startUpdateCountTime + std::chrono::milliseconds(1000)) {
      std::chrono::milliseconds milliSinceLastUpdate = now - m_startUpdateCountTime;
      // DebugDriverLog("%s Hand updated %i times last second", IsRightHand() ? "Right" : "Left", m_updateCount);

      m_updateCount = 0;
      m_startUpdateCountTime = now;
    } else
      m_updateCount++;
  }
#endif
}

void EteeDeviceDriver::InputUpdateThread() {
  while (m_isActive) {
    {
      std::lock_guard<std::mutex> lock(m_inputMutex);
      if (m_lastInput.isValid) {
        m_boneAnimator->ComputeSkeletonTransforms(m_handTransforms, m_lastInput);
      }
    }

    vr::VRDriverInput()->UpdateSkeletonComponent(
        m_inputComponentHandles[ComponentIndex::SKELETON], vr::VRSkeletalMotionRange_WithoutController, m_handTransforms, NUM_BONES);
    vr::VRDriverInput()->UpdateSkeletonComponent(
        m_inputComponentHandles[ComponentIndex::SKELETON], vr::VRSkeletalMotionRange_WithController, m_handTransforms, NUM_BONES);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void EteeDeviceDriver::OnStateUpdate(VRControllerState data) {
  if (data == m_deviceState || (data == VRControllerState::ready && m_deviceState == VRControllerState::streaming)) return;

  DriverLog(
      "%s hand updated. Connected: %s Streaming: %s",
      IsRightHand() ? "Right" : "Left",
      data != VRControllerState::disconnected ? "Yes" : "No",
      data == VRControllerState::streaming ? "Yes" : "No");

  m_deviceState = data;

  m_controllerPose->SetDeviceState(m_deviceState);

  // If we're active let's also update the status pose. We might get events before we've activated
  if (!IsActive()) return;

  // then update the pose to reflect status
  vr::DriverPose_t pose = m_controllerPose->GetStatusPose();
  vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_driverId, pose, sizeof(vr::DriverPose_t));
}

// I've never seen this function called, but will add an empty pose for good measure.
vr::DriverPose_t EteeDeviceDriver::GetPose() {
  return m_controllerPose->UpdatePose();
}

void EteeDeviceDriver::PoseUpdateThread() {
  while (m_isActive) {
    vr::DriverPose_t pose = m_controllerPose->UpdatePose();
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_driverId, pose, sizeof(vr::DriverPose_t));

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  DriverLog("Closing pose thread");
}

void EteeDeviceDriver::OnVREvent(const vr::VREvent_t& vrEvent) {
  switch (vrEvent.eventType) {
    case vr::VREvent_Input_HapticVibration: {
      if (vrEvent.data.hapticVibration.componentHandle == m_inputComponentHandles[ComponentIndex::HAPTIC] && m_configuration.haptic.enabled) {
        const HapticEventData data = OnHapticEvent(vrEvent.data.hapticVibration);
        m_deviceEventCallback({DeviceEventType::HAPTIC_EVENT, data});

        break;
      }
    }
  }
}

HapticEventData EteeDeviceDriver::OnHapticEvent(const vr::VREvent_HapticVibration_t& hapticEvent) {
  // No haptics if amplitude or frequency is 0.
  if (hapticEvent.fAmplitude <= 0.f || hapticEvent.fFrequency <= 0.f) {
    return {0, 0, 0};
  }

  const float duration = std::clamp(hapticEvent.fDurationSeconds, 0.f, 10.f);
  const float frequency = std::clamp(hapticEvent.fFrequency, 1000000.f / 65535.f, 1000000.f / 300.f);
  const float amplitude = std::clamp(hapticEvent.fAmplitude, 0.f, 1.f);
  DebugDriverLog("Received haptic vibration: freq: %f amp: %f dur: %f", frequency, amplitude, duration);

  if (duration <= 0.f) {
    const int onDurationUs = (int)(amplitude * 4000.f);
    return {
        onDurationUs,
        onDurationUs,
        1,
    };
  }

  const float periodSeconds = 1 / frequency;
  const float maxPulseDurationSeconds = std::min(0.003999f, 0.5f * periodSeconds);
  const float onDurationSeconds = Lerp(0.000080f, maxPulseDurationSeconds, amplitude);

  const int pulseCount = (int)std::clamp(duration * frequency, 1.f, 65535.f);
  const int onDurationUs = (int)std::clamp(1000000.f * onDurationSeconds, 0.f, 65535.f);
  const int offDurationUs = (int)std::clamp(1000000.f * (periodSeconds - onDurationSeconds), 0.f, 65535.f);

  return {
      onDurationUs,
      offDurationUs,
      pulseCount,
  };
}

vr::ETrackedControllerRole EteeDeviceDriver::GetDeviceRole() const {
  return m_configuration.role;
}

void EteeDeviceDriver::SetTrackerId(short deviceId, bool isRightHand) {
  m_controllerPose->SetShadowEteeTracker(deviceId, isRightHand);
}

VRControllerState EteeDeviceDriver::GetControllerState() {
  return m_deviceState;
}

void EteeDeviceDriver::Deactivate() {
  if (m_isActive.exchange(false)) {
    m_poseUpdateThread.join();
    m_inputUpdateThread.join();
  }
}

void* EteeDeviceDriver::GetComponent(const char* pchComponentNameAndVersion) {
  return nullptr;
}

void EteeDeviceDriver::EnterStandby() {
  m_deviceEventCallback({DeviceEventType::STANDBY, {}});

  m_controllerPose->SetDeviceState(VRControllerState::disconnected);
}

void EteeDeviceDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
  if (unResponseBufferSize >= 1) {
    pchResponseBuffer[0] = 0;
  }
}

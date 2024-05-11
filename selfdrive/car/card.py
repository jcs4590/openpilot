#!/usr/bin/env python3
import os
import time

import cereal.messaging as messaging

from cereal import car, log

from panda import ALTERNATIVE_EXPERIENCE

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL

from openpilot.selfdrive.boardd.boardd import can_list_to_can_capnp
from openpilot.selfdrive.car.car_helpers import get_car, get_one_can
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel

REPLAY = "REPLAY" in os.environ

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


class CarD:
  CI: CarInterfaceBase

  def __init__(self, CI=None):
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'liveParameters', 'liveTorqueParameters', 'longitudinalPlan', 'modelV2'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput'])

    self.can_rcv_timeout_counter = 0  # consecutive timeout count
    self.can_rcv_cum_timeout_counter = 0  # cumulative timeout count

    self.CC_prev = car.CarControl.new_message()

    self.last_actuators = None

    self.params = Params()

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)

      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)
      experimental_long_allowed = self.params.get_bool("ExperimentalLongitudinalEnabled")
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
    else:
      self.CI, self.CP = CI, CI.CP

    # set alternative experiences from parameters
    disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
    self.CP.alternativeExperience = 0
    if not disengage_on_accelerator:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS

    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")

    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly

    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = car.CarParams.SafetyConfig.new_message()
      safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    self.params.put_nonblocking("CarParamsCache", cp_bytes)
    self.params.put_nonblocking("CarParamsPersistent", cp_bytes)

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)

    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)

  def initialize(self):
    """Initialize CarInterface, once controls are ready"""
    self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])

  def state_update(self) -> car.CarState:
    """carState update loop, driven by can"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC_prev, can_strs)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_timeout_counter += 1
      self.can_rcv_cum_timeout_counter += 1
    else:
      self.can_rcv_timeout_counter = 0

    self.can_rcv_timeout = self.can_rcv_timeout_counter >= 5

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.state_publish(CS)

    return CS

  def state_publish(self, CS: car.CarState):
    """carState and carParams publish loop"""

    # carState
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    self.pm.send('carState', cs_send)

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = True
    if self.last_actuators is not None:
      co_send.carOutput.actuatorsOutput = self.last_actuators
    self.pm.send('carOutput', co_send)

  def state_control(self, CS: car.CarState) -> car.CarControl:
    """Given the state, this function returns a CarControl packet"""

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['controlsState'].enabled

    # Check which actuators can be enabled
    # TODO: move events here too
    standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    CC.latActive = self.sm['controlsState'].active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.joystick_mode)
    CC.longActive = self.sm['controlsState'].enabled and not self.events.contains(ET.OVERRIDE_LONGITUDINAL) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    # State specific actions

    # TODO: synced both functions up to here so far

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset(v_pid=CS.vEgo)

    if not self.joystick_mode:
      # accel PID loop
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
      t_since_plan = (self.sm.frame - self.sm.recv_frame['longitudinalPlan']) * DT_CTRL
      actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)

      # Steering PID loop and lateral MPC
      self.desired_curvature = clip_curvature(CS.vEgo, self.desired_curvature, model_v2.action.desiredCurvature)
      actuators.curvature = self.desired_curvature
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                                             self.steer_limited, self.desired_curvature,
                                                                             self.sm['liveLocationKalman'])
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if self.sm.recv_frame['testJoystick'] > 0:
        # reset joystick if it hasn't been received in a while
        should_reset_joystick = (self.sm.frame - self.sm.recv_frame['testJoystick']) * DT_CTRL > 0.2
        if not should_reset_joystick:
          joystick_axes = self.sm['testJoystick'].axes
        else:
          joystick_axes = [0.0, 0.0]

        if CC.longActive:
          actuators.accel = 4.0 * clip(joystick_axes[0], -1, 1)

        if CC.latActive:
          steer = clip(joystick_axes[1], -1, 1)
          # max angle is 45 for angle-based cars, max curvature is 0.02
          actuators.steer, actuators.steeringAngleDeg, actuators.curvature = steer, steer * 90., steer * -0.02

        lac_log.active = self.active
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = actuators.steer
        lac_log.saturated = abs(actuators.steer) >= 0.9

    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame) * DT_CTRL < 2.0

    # Send a "steering required alert" if saturation count has reached the limit
    if lac_log.active and not recent_steer_pressed and not self.CP.notCar:
      if self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
        undershooting = abs(lac_log.desiredLateralAccel) / abs(1e-3 + lac_log.actualLateralAccel) > 1.2
        turning = abs(lac_log.desiredLateralAccel) > 1.0
        good_speed = CS.vEgo > 5
        max_torque = abs(self.sm['carOutput'].actuatorsOutput.steer) > 0.99
        if undershooting and turning and good_speed and max_torque:
          lac_log.active and self.events.add(EventName.steerSaturated)
      elif lac_log.saturated:
        # TODO probably should not use dpath_points but curvature
        dpath_points = model_v2.position.y
        if len(dpath_points):
          # Check if we deviated from the path
          # TODO use desired vs actual curvature
          if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            steering_value = actuators.steeringAngleDeg
          else:
            steering_value = actuators.steer

          left_deviation = steering_value > 0 and dpath_points[0] < -0.20
          right_deviation = steering_value < 0 and dpath_points[0] > 0.20

          if left_deviation or right_deviation:
            self.events.add(EventName.steerSaturated)

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    # decrement personality on distance button press
    if self.CP.openpilotLongitudinalControl:
      if any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents):
        self.personality = (self.personality - 1) % 3
        self.params.put_nonblocking('LongitudinalPersonality', str(self.personality))

    return CC, lac_log


  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    # send car controls over can
    now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
    self.last_actuators, can_sends = self.CI.apply(CC, now_nanos)
    self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

    self.CC_prev = CC

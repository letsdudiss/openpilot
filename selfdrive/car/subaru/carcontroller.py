from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import DBC, PREGLOBAL_CARS, CarControllerParams
from selfdrive.car.subaru.values import REDUCED_TORQUE_CARS
from opendbc.can.packer import CANPacker
import time



class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_accel_cnt = -1
    self.es_lkas_cnt = -1
    self.fake_button_prev = 0
    self.steer_rate_limited = False

    #SUBARU ENGINE AUTO START-STOP flags and vars
    self.dashlights_cnt = -1
    self.has_set_auto_ss = False

    #SUBARU STOP AND GO flags and vars
    self.throttle_cnt = -1
    self.prev_cruise_state = -1
    self.cruise_state_change_time = -1
    self.sng_throttle_tap_cnt = 0
    self.sng_resume_acc = False
    self.sng_has_recorded_distance = False
    self.sng_distance_threshold = CarControllerParams.SNG_DISTANCE_LIMIT

    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line):

    can_sends = []

    # *** steering ***
    if (frame % CarControllerParams.STEER_STEP) == 0:
      #Override STEER_MAX if car is IMPREZA 2021 due to reduced torque limit
      if CS.CP.carFingerprint in REDUCED_TORQUE_CARS:
        apply_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX_REDUCED))
      else:
        apply_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))

      # limits due to driver torque

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, CarControllerParams)
      self.steer_rate_limited = new_steer != apply_steer

      #@letdudiss 18 Nov 2020 Work around for steerWarning to
      #Avoids LKAS and ES fault when OP apply a steer value exceed what ES allows
      #set Steering value to 0 when a steer Warning is present
      if CS.out.steerWarning and CS.CP.carFingerprint in REDUCED_TORQUE_CARS:
        apply_steer = 0

      if not enabled:
        apply_steer = 0

      if CS.CP.carFingerprint in PREGLOBAL_CARS:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, apply_steer, frame, CarControllerParams.STEER_STEP))
      else:
        can_sends.append(subarucan.create_steering_control(self.packer, apply_steer, frame, CarControllerParams.STEER_STEP))

      self.apply_steer_last = apply_steer

    #--------------------Engine Auto Start-Stop----------------------
    if CarControllerParams.FEATURE_NO_ENGINE_STOP_START:
      #GLOBAL only
      if CS.CP.carFingerprint not in PREGLOBAL_CARS:
        #If Auto Stop Start has gone to state 3 at least once, it means either we have successfully turn off autoStopStart
        #or driver manually turn it off before we got to it
        if CS.autoStopStartDisabled:
          self.has_set_auto_ss = True

        #Send message to press AutoSS button, only do it once, when car starts up, after that, driver can turn it back on if they want
        if self.dashlights_cnt != CS.dashlights_msg["Counter"] and not self.has_set_auto_ss:
          can_sends.append(subarucan.create_dashlights(self.packer, CS.dashlights_msg, True))
          self.dashlights_cnt = CS.dashlights_msg["Counter"]
    #----------------------------------------------------------------

    #----------------------Subaru STOP AND GO------------------------
    if CS.CP.carFingerprint in PREGLOBAL_CARS:
      throttle_cmd = -1
      #PREGLOBAL SUBARU STOP AND GO
      #NOTE: I will not add PRE_GLOBAL support to this repo, but I have added it on mlp's fork
    else:  
      #GLOBAL SUBARU STOP AND GO
      #Car can only be in HOLD state (3) if it is standing still
      # => if not in HOLD state car has to be moving or driver has taken action
      if CS.cruise_state != 3:
        self.sng_throttle_tap_cnt = 0           #Reset throttle tap message count when car starts moving
        self.sng_resume_acc = False             #Cancel throttle tap when car starts moving
        self.sng_has_recorded_distance = False  #Reset has_recorded_distance flag once car started moving

      #Reset SNG distance threshold to limit value if we havent recorded a reference distance threshold
      #This is to make sure car will always move forward when lead car moves before SnG reference distance
      #threshold is recorded
      if not self.sng_has_recorded_distance:
        self.sng_distance_threshold = CarControllerParams.SNG_DISTANCE_LIMIT

      #Record the time at which CruiseState change to HOLD (3)
      if self.prev_cruise_state != 3 and CS.cruise_state == 3:
        self.cruise_state_change_time = time.time_ns()

      #While in HOLD, wait <ES_CLOSE_DISTANCE_SETTLE_TIME> nanoseconds (since Cruise state changes to HOLD)
      #before recording SnG lead car reference distance
      if (enabled
          and CS.cruise_state == 3                #in HOLD state
          and not self.sng_has_recorded_distance  #has not recorded reference distance
          and time.time_ns() > self.cruise_state_change_time + CarControllerParams.ES_CLOSE_DISTANCE_SETTLE_TIME): #wait 200ms before recording reference distance
        self.sng_distance_threshold = CS.close_distance
        self.sng_has_recorded_distance = True     #Set flag to true so sng_distance_threshold wont be recorded again until car moves
        #Limit lead car reference distance to <SNG_DISTANCE_LIMIT> or when Wipers are ON as Wipers affect CloseDistance reading
        if self.sng_distance_threshold > CarControllerParams.SNG_DISTANCE_LIMIT or CS.wipers_activated:
          self.sng_distance_threshold = CarControllerParams.SNG_DISTANCE_LIMIT

      #Trigger THROTTLE TAP when in hold and close_distance increases > SNG distance threshold (with deadband)
      #false positives caused by pedestrians/cyclists crossing the street in front of car
      self.sng_resume_acc = False
      if (enabled
          and CS.cruise_state == 3 #cruise state == 3 => ACC HOLD state
          and CS.close_distance > self.sng_distance_threshold + CarControllerParams.SNG_DISTANCE_DEADBAND #lead car distance is within SnG operating range
          and CS.close_distance < 255
          and CS.car_follow == 1):
        self.sng_resume_acc = True

      #Send a throttle tap to resume ACC
      throttle_cmd = -1 #normally, just forward throttle msg from ECU
      if self.sng_resume_acc:
        #Send Maximum <THROTTLE_TAP_LIMIT> to get car out of HOLD. NOTE: this portion of code should not trigger
        #as car should RESUME after 1 message, but incase ES is stubborn and wont release HOLD, this should do the trick
        if self.sng_throttle_tap_cnt < CarControllerParams.THROTTLE_TAP_LIMIT:
          throttle_cmd = CarControllerParams.THROTTLE_TAP_LEVEL
          self.sng_throttle_tap_cnt += 1
        else:
          self.sng_throttle_tap_cnt = -1
          self.sng_resume_acc = False

      #Send Throttle message
      if self.throttle_cnt != CS.throttle_msg["Counter"]:
        can_sends.append(subarucan.create_throttle(self.packer, CS.throttle_msg, throttle_cmd))
        self.throttle_cnt = CS.throttle_msg["Counter"]

      #TODO: Send cruise throttle to get car up to speed. There is a 2-3 seconds delay after
      # throttle tap is sent and car start moving. EDIT: This is standard with Toyota OP's SnG
      #pseudo: !!!WARNING!!! Dangerous, proceed with CARE
      #if sng_resume_acc is True && has been 1 second since sng_resume_acc turns to True && current ES_Throttle < 2000
      #    send ES_Throttle = 2000

      #Update prev values
      self.prev_cruise_state = CS.cruise_state
    #------------------------------------------------------------------

    # *** alerts and pcm cancel ***

    if CS.CP.carFingerprint in PREGLOBAL_CARS:
      if self.es_accel_cnt != CS.es_accel_msg["Counter"]:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          fake_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          fake_button = 1
        else:
          fake_button = CS.button

        # unstick previous mocked button press
        if fake_button == 1 and self.fake_button_prev == 1:
          fake_button = 0
        self.fake_button_prev = fake_button

        can_sends.append(subarucan.create_es_throttle_control(self.packer, fake_button, CS.es_accel_msg))
        self.es_accel_cnt = CS.es_accel_msg["Counter"]

    else:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        can_sends.append(subarucan.create_es_lkas(self.packer, CS.es_lkas_msg, visual_alert, left_line, right_line))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]

    return can_sends

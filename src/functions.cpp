#include <FlexCAN_T4.h>
#include <Arduino.h>
#include "ecu_functions.h"
#include "ecu_defines.h"
// -------------------------------------------------------------
void Status_Print() {
  Serial.println("=====================system status=======================");
  Serial.print("PedalThrottle: ");
  Serial.print(PedalThrottle);
  Serial.println(" [%]");
  Serial.print("PedalBrake: ");
  Serial.print(PedalBrake);
  Serial.println(" [%]");
  Serial.print("current: ");
  Serial.print(IvtsCurrent);
  Serial.println(" [mA]");
  Serial.print("Voltage measure 1: ");
  Serial.print(IvtsVoltage);
  Serial.println(" [mV]");
  Serial.print("Temperature measure: ");
  Serial.print(IvtsTemperature);
  Serial.println(" [C]");
  Serial.print("Power measure: ");
  Serial.print(IvtsPower);
  Serial.println(" [W]");
  Serial.print("Battery state: ");
  Serial.println(Battery_state);
  Serial.print("Battery SOC percent: ");
  Serial.print(Battery_SOC_percent);
  Serial.println(" [%]");
  Serial.print("Battery Voltage: ");
  Serial.print(Battery_Voltage);
  Serial.println(" [mA]");
  Serial.print("Charger flag: ");
  Serial.println(Charger_flags);
  
  Serial.print("Motor Torque: ");
  Serial.print(MotorTorque);
  Serial.println(" [%]");
  Serial.print("Motor on flag: ");
  Serial.println(Motor_On);
  Serial.print("Motor voltage: ");
  Serial.print(MotorVoltage);
  Serial.println(" [V]");
  Serial.print("Voltage implausibility: ");
  Serial.println(voltage_implausibility);
  Serial.println("==========================END============================");
}


void Print_CanMsg(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void CAN1_Unpack(const CAN_message_t &inMsg) {
  switch(inMsg.id){
    case PEDAL_CONTROLLER_ID:
      PedalBeat  = true;
      PedalBrake = inMsg.buf[2];

      if (inMsg.buf[0] == 0x20) {
        PedalControllerError = false;
        TPS_Implausibility = true;
        Serial.println("Implausibility");
        PedalThrottle = 0;
      }
      else if (inMsg.buf[0] == 0x30){
        PedalControllerError = true;
        TPS_Implausibility = false;
        Serial.println("PedalError");
      }
      else{
        PedalControllerError = false;
        TPS_Implausibility = false;
        PedalThrottle = inMsg.buf[1];
      }
      break;
    case IVTS_CURRENT_ID:
      IVTSBeat = true;
      IvtsCurrent = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
      NominalCurrent = CalcNominal();
      break;
    case IVTS_VOLTAGE_ID:
      IVTSBeat = true;
      IvtsVoltage = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
      break;
    case IVTS_TEMP_ID:
      IVTSBeat = true;
      IvtsTemperature = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
      IvtsTemperature = IvtsTemperature/10;
      break;
    case IVTS_POWER_ID:
      IVTSBeat = true;
      IvtsPower = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
      break;
    case AMS_ERROR_ID:
      if (inMsg.buf[2] !=0  ||  inMsg.buf[1] !=0) AMSError =true;
      else AMSError = false;
      // Check if we need Minimum cell boltage Battery SOC and etc
      break;
    case AMS_PRECHARGE_DONE_ID:
      // Check if we need Battery voltage TODO add Air+ status
      break;
    case AMS_CHARGING_ID:
      // TODO check how this masseges are sent from AMS
      break;
    case LOGGER_START_ID: 
      GPSVelocity = (inMsg.buf[3] << 24) + (inMsg.buf[2] << 16) + (inMsg.buf[1] << 8) + inMsg.buf[0];
      LoggerTemp1 = (inMsg.buf[7] << 24) + (inMsg.buf[6] << 16) + (inMsg.buf[5] << 8) + inMsg.buf[4];
      break;
    case LOGGER_END_ID:
      LoggerTemp2 = (inMsg.buf[3] << 24) + (inMsg.buf[2] << 16) + (inMsg.buf[1] << 8) + inMsg.buf[0];
      break;
    case DCDC_ON_ID:
      if ((inMsg.buf[0] & 0x01) ==1) DcdcOn = true;
      else  DcdcOn = false;
      DcdcLowCurrent = (inMsg.buf[1] << 24) + (inMsg.buf[2] << 16) + (inMsg.buf[3] << 8) + inMsg.buf[4];
      break;
    case DCDC_Meas_ID:     
      DcdcLowVoltage = (inMsg.buf[4] << 24) + (inMsg.buf[5] << 16) + (inMsg.buf[6] << 8) + inMsg.buf[7]; // TODO check if low voltage or high
      break;
  }

};

void CAN2_Unpack(const CAN_message_t &inMsg) {
  switch(inMsg.id){
    case MOTOR_THROTTLE_ID:

      break;
    case MOTOR_TORQUE_ID:

      break;
    case MOTOR_CAP_VOLTAGE_ID:

      break;
    case MOTOR_VELOCITY_ID:

      break;
  }
}

void CheckPowerAnd(){
  MilliSec = true;
}

void HeartBeatAISP(){  // AISP - AMS, IVTS, Sevcon, Pedal Controller
  if  (HeartBeatCounter != HeartBeatCounterMaxValue){
    HeartBeatCounter +=1;
    return;
  }
  HeartBeatCounter = 0;
  if (IVTSBeat && SevconBeat && AMSBeat && PedalBeat){
    HeartBeatError = false;
  }
  else{
    HeartBeatError = true;
  }
  IVTSBeat   = false;
  AMSBeat    = false;
  SevconBeat = false; 
  PedalBeat  = false;
}

void MotorControllerMB(const CAN_message_t &inMsg) {
  SevconBeat = true;
  MotorTorque = inMsg.buf[0];
  Motor_On = inMsg.buf[1];
  MotorVoltage = (inMsg.buf[2] << 8) + inMsg.buf[3];
}
void Interrupt_Routine(){
  Status_Print();
}


void Send_Torque() {
  if (FwRevCouter != TorqueDelay){
    FwRevCouter+=1;
    return;
  }
  FwRevCouter=0;
  if(abs(MotorVoltage - (IvtsVoltage/1000)) > VoltageTollerance)  
  {
    Torque_msg.buf[0] = 0;
    voltage_implausibility = 1;  
  }
  else
  {
    Torque_msg.buf[0] = PedalThrottle;
    if ((state == LIMP_STATE)||(state == REV_STATE)){
      Torque_msg.buf[0] = PedalThrottle/LIMP_DIVISION;
    }
    voltage_implausibility = 0;
  }
  if (state == BT_FW_STATE || state == BT_REV_STATE || state == ERROR_STATE){
    Torque_msg.buf[0] = 0;
  }
  Torque_msg.id = 0x111;
  Torque_msg.len = 1;
  Torque_msg.flags.extended = 0;
  Torque_msg.flags.remote   = 0;
  Torque_msg.flags.overrun  = 0;
  Torque_msg.flags.reserved = 0;
  
  Can2.write(Torque_msg);    //CANBus write command
}

int CalcNominal(){
  current_list[index_current] = IvtsCurrent;
  index_current = (index_current + 1) % NOMIMAL_NUM; // The newest measure, switchs the last measure each time
  int sum = 0;
  for (int i=0; i<NOMIMAL_NUM; i++){
    sum +=current_list[i];
  }
  return sum/NOMIMAL_NUM;

}

int LVError(){
  if (AMSError){
    state = ERROR_STATE;
  }
  if (PedalControllerError){
    state = ERROR_STATE;
  }
  if(abs(MotorVoltage - (IvtsVoltage/1000)) > VoltageTollerance) {
    state = ERROR_STATE;
  }
  return state;
}

int HVError(){
  if (AMSError){
    state = ERROR_STATE;
  }
  if (PedalControllerError){
    state = ERROR_STATE;
  }
  if (abs(MotorVoltage - (IvtsVoltage/1000)) > VoltageTollerance) {
    state = ERROR_STATE;
  }
  if (!digitalRead(shutdownFB_pin)){
    state = ERROR_STATE;
  }
  if (HeartBeatError){
    state = ERROR_STATE;
  }
  if (IvtsPower> MaxPower){
    state = ERROR_STATE;
  }
  return state;
}

int CheckHV(){
  if ((IvtsVoltage > 60) ||  (MotorVoltage > 60)){
    return HV_STATE;
  } else {
    return LV_STATE;
  } 
}

int CheckCooling(int cool){
  if (CoolButtonCounter == CoolButtonDelay){
    if (digitalRead(ForceCooling_pin)){
      CoolButtonCounter = 0;
      if (cool == CoolingOff){
        cool=ForcedCoolingVal;
      } else {
        cool=CoolingOff;
      }
    }
  } else{   //CoolButtonCounter != CoolButtonDelay
    CoolButtonCounter +=1;
  }
  if (IvtsTemperature > CoolingReqTemp){
    cool = CoolingTempHigh;
  }
  return cool;
}

void EnableCooling(int cool){
  if (cool == CoolingTempHigh){
    digitalWrite(Collingfan1_pin,HIGH);
    digitalWrite(Collingfan2_pin,HIGH);
    digitalWrite(Pump1_pin,HIGH);
    digitalWrite(Pump2_pin,HIGH);
  } else if (cool == ForcedCoolingVal){
    digitalWrite(Collingfan1_pin,LOW);
    digitalWrite(Collingfan2_pin,LOW);
    digitalWrite(Pump1_pin,HIGH);
    digitalWrite(Pump2_pin,HIGH);
  } else{
    digitalWrite(Collingfan1_pin,LOW);
    digitalWrite(Collingfan2_pin,LOW);
    digitalWrite(Pump1_pin,LOW);
    digitalWrite(Pump2_pin,LOW);
  }
}
int CheckBrackNThrottle(){
  if ((PedalBrake > HARD_BRAKE_VALUE)&&(PedalThrottle>(BT_MAX_THROTTLE)||(desired_motor_torque>BT_MAX_TOQUE))){
    bt_counter +=1;
    if  (bt_counter >= BT_MAX_COUNT ){
      if (state == REV_STATE){
          return BT_REV_STATE;
      }
      else{ // state == FW or LIMP
          return BT_REV_STATE;
      }
      
    }
  }
  //else
  return state;
}
int CheckNoThrottle(){
if ((desired_motor_torque<= MIN_MOTOR_TORQUE)&&(PedalThrottle<=MIN_TPS_THROTTLE)){
  if (state==BT_REV_STATE){
    return REV_STATE;
  }
  else{
    return FW_STATE;
  }
}
//else
return state;
}

bool AllOk(){
  if ((AMSError)||(PedalControllerError)||(IvtsVoltage>=TS_VOLTAGE_ON)||(voltage_implausibility)){
    return false;
  }
  //else
  return true;
}

void DcDcCheck(){
  if ( (DcdcLowVoltage < MAX_LOW_VOLTAGE) && (DcdcLowCurrent < MAX_LOW_CURRENT) ){ //All good
    //Send can message to DCDC Turn On
  }
  else{
    //Send can message to DCDC Turn Off
  }
}

int CheckR2D(){
  if (R2DCounter == R2DDelay){
    if (digitalRead(R2Dbutton_pin) && PedalBrake > MinBrakeR2D && !digitalRead(ForwardSwitch_pin) && !digitalRead(ReverseSwitch_pin)){
      if (air_plus && !charging){
        ready_to_drive_pressed = true;
        R2DCounter = 0;
        digitalWrite(EnableBuzzer_pin,HIGH);
      }
    }
  } else {    // R2DCounter != R2DDelay
    R2DCounter +=1;
    if (R2DCounter == R2DDelay && ready_to_drive_pressed){
      state = R2D_STATE;
    }
  }
  return state;
}

int LeaveR2D(){
  if (digitalRead(R2Dbutton_pin) && PedalBrake > MinBrakeR2D && velocity < MIN_SPEED_TO_REV ){
    R2DCounter = R2DDelay - R2D_BUTTON_DELAY;
    state = HV_STATE;
  } else if (digitalRead(ForwardSwitch_pin)){
    state = FW_STATE;
  } else if (digitalRead(ReverseSwitch_pin) && velocity < MIN_SPEED_TO_REV){
    state = REV_STATE;
  }
  return state;
}

int CheckLimp(){
  if (Battery_Percent < MIN_BATTERY || NominalCurrent > MAX_NOMIMAL_CURRENT || IvtsPower > MaxPower || IvtsTemperature > MAX_TEMPERATURE){
    state = LIMP_STATE;
  }
  return state;
}

bool WaitRelay(){
  bool ready = false;
  if (relay_counter != RELAY_DELAY_ERROR){
    relay_counter +=1;
  } else {
    relay_counter = 0;
    ready = true;
  }
  return ready;
}
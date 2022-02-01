#include <FlexCAN_T4.h>
#include <Arduino.h>
#include "ecu_functions.h"
#include "ecu_defines.h"
// -------------------------------------------------------------
void Status_Print() {
  Serial.println("=====================system status=======================");
  Serial.print("Throttle: ");
  Serial.print(Throttle);
  Serial.println(" [%]");
  Serial.print("Brake: ");
  Serial.print(Brake);
  Serial.println(" [%]");
  Serial.print("current: ");
  Serial.print(Current_meas);
  Serial.println(" [mA]");
  Serial.print("Voltage measure 1: ");
  Serial.print(Voltage_meas1);
  Serial.println(" [mV]");
  Serial.print("Voltage measure 2: ");
  Serial.print(Voltage_meas2);
  Serial.println(" [mV]");
  Serial.print("Voltage measure 3: ");
  Serial.print(Voltage_meas3);
  Serial.println(" [mV]");
  Serial.print("Temperature measure: ");
  Serial.print(Temperature_meas);
  Serial.println(" [C]");
  Serial.print("Power measure: ");
  Serial.print(Power_meas);
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
  
  Serial.print("Motor torqe: ");
  Serial.print(Motor_Torqe);
  Serial.println(" [%]");
  Serial.print("Motor on flag: ");
  Serial.println(Motor_On);
  Serial.print("Motor voltage: ");
  Serial.print(Motor_Voltage);
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
void PedalControllerMB(const CAN_message_t &inMsg) {
  PedalBeat  = true;
  Brake = inMsg.buf[2];

  if (inMsg.buf[0] == 0x20) {
    PedalControllerError = false;
    TPS_Implausibility = true;
    Serial.println("Implausibility");
    Throttle = 0;
  }
  else if (inMsg.buf[0] == 0x30){
    PedalControllerError = true;
    TPS_Implausibility = false;
    Serial.println("PedalError");
  }
  else{
    PedalControllerError = false;
    TPS_Implausibility = false;
    Throttle = inMsg.buf[1];
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

void CurrentMeasMB(const CAN_message_t &inMsg) {
  IVTSBeat = true;
  Current_meas = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
  nominal_current = CalcNonimal(Current_meas);
}
void VoltageMeasure1MB(const CAN_message_t &inMsg) {
  IVTSBeat = true;
  Voltage_meas1 = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
}
void VoltageMeasure2MB(const CAN_message_t &inMsg) {
  IVTSBeat = true;
  Voltage_meas2 = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
}
void VoltageMeasure3MB(const CAN_message_t &inMsg) {
  IVTSBeat = true;
  Voltage_meas3 = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
}
void TemperatureMeasureMB(const CAN_message_t &inMsg) {
  IVTSBeat = true;
  Temperature_meas = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
  Temperature_meas = Temperature_meas/10;
}
void PowerMeasure(const CAN_message_t &inMsg) {
  IVTSBeat = true;
  Power_meas = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
}
void BatteryStateMB(const CAN_message_t &inMsg) {
  Battery_state = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
}
void BatterySOCPercentMB(const CAN_message_t &inMsg) {
  Battery_SOC_percent = inMsg.buf[1];
}
void BatteryVoltageMB(const CAN_message_t &inMsg) {
  Battery_Voltage = (inMsg.buf[1] << 24) + (inMsg.buf[2] << 16) + (inMsg.buf[3] << 8) + inMsg.buf[4];
  
}
void ChargerFlagsMB(const CAN_message_t &inMsg) {
  Charger_flags = inMsg.buf[4];
}
void MotorControllerMB(const CAN_message_t &inMsg) {
  SevconBeat = true;
  Motor_Torqe = inMsg.buf[0];
  Motor_On = inMsg.buf[1];
  Motor_Voltage = (inMsg.buf[2] << 8) + inMsg.buf[3];
}
void Interrupt_Routine(){
  Status_Print();
}


void Send_Tourqe() {
  if (FwRevCouter != TorqueDelay){
    FwRevCouter+=1;
    return;
  }
  FwRevCouter=0;
  if(abs(Motor_Voltage - (Voltage_meas1/1000)) > VoltageTollerance)  
  {
    torqe_msg.buf[0] = 0;
    voltage_implausibility = 1;  
  }
  else
  {
    torqe_msg.buf[0] = Throttle;
    voltage_implausibility = 0;
  }
  if (state == BT_FW_STATE || state == BT_REV_STATE || state == ERROR_STATE){
    torqe_msg.buf[0] = 0;
  }
  torqe_msg.id = 0x81;
  torqe_msg.len = 1;
  torqe_msg.flags.extended = 0;
  torqe_msg.flags.remote   = 0;
  torqe_msg.flags.overrun  = 0;
  torqe_msg.flags.reserved = 0;
  
  Can1.write(torqe_msg);    //CANBus write command
}

int CalcNonimal(uint32_t Current_meas){
  current_list[index] = Current_meas;
  index = (index + 1) % NOMIMAL_NUM; // The newest measure, switchs the last measure each time
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
  if(abs(Motor_Voltage - (Voltage_meas1/1000)) > VoltageTollerance) {
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
  if (abs(Motor_Voltage - (Voltage_meas1/1000)) > VoltageTollerance) {
    state = ERROR_STATE;
  }
  if (!digitalRead(shutdownFB_pin)){
    state = ERROR_STATE;
  }
  if (HeartBeatError){
    state = ERROR_STATE;
  }
  if (Power_meas> MaxPower){
    state = ERROR_STATE;
  }
  return state;
}

int CheckHV(){
  if ((Voltage_meas1 > 60) ||  (Motor_Voltage > 60)){
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
  if (Temperature_meas > CoolingReqTemp){
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
void DcDcCheck(){
  if ( (low_voltage < MAX_LOW_VOLTAGE) && (low_current < MAX_LOW_CURRENT) ){ //All good
    //Send can message to DCDC Turn On
  }
  else{
    //Send can message to DCDC Turn Off
  }
}

int CheckR2D(){
  if (R2DCounter == R2DDelay){
    if (digitalRead(R2Dbutton_pin) && Brake > MinBrakeR2D && !digitalRead(ForwardSwitch_pin) && !digitalRead(ReverseSwitch_pin)){
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
  if (digitalRead(R2Dbutton_pin) && Brake > MinBrakeR2D && velocity < MIN_SPEED_TO_REV ){
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
  if (Battery_Percent < MIN_BATTERY || nominal_current > MAX_NOMIMAL_CURRENT || Power_meas > MaxPower || Temperature_meas > MAX_TEMPERATURE){
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
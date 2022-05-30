//hall and api

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
  Serial.println(" [A]");
  Serial.print("IvtsVoltage: ");
  Serial.print(IvtsVoltage);
  Serial.println(" [V]");
  Serial.print("Temperature measure: ");
  Serial.print(IvtsTemperature);
  Serial.println(" [C]");
  Serial.print("Power measure: ");
  Serial.print(IvtsPower);
  Serial.println(" [W]");
  Serial.print("State:  ");
  Serial.println(state);
  Serial.print("StateUsage:  ");
  for (int i;i<=8;i++){
    Serial.print(stateUsage[i]);
    Serial.print(" ");
  }
  Serial.println(stateUsage[9]);

  

  /*
  Serial.print("Battery state: ");
  Serial.println(Battery_state);
  Serial.print("Battery SOC percent: ");
  Serial.print(Battery_SOC_percent);
  Serial.println(" [%]");
  Serial.print("Battery Voltage: ");
  Serial.print(AMSBatteryVoltage);
  Serial.println(" [mA]");
  Serial.print("Charger flag: ");
  Serial.println(Charger_flags);
  */
  Serial.print("Motor Torque: ");
  Serial.print(SevconActualTorque);
  Serial.println(" [%]");
  Serial.print("Motor voltage: ");
  Serial.print(SEVCON_SCALE_VOLTAGE*SevconCapVoltage);
  Serial.println(" [V]");
  Serial.println("==========================END============================");
}


void Print_CanMsg(const CAN_message_t &msg) {
  Serial.print("MB ");Can1.events();
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

      if (inMsg.buf[0] == 0x02) {
        PedalControllerError = false;
        TPS_Implausibility = true;
        Serial.println("Implausibility");
        PedalThrottle = 0;
      }
      else if (inMsg.buf[0] == 0x03){
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
      IvtsCurrent = int(IVTS_SCALE_CURRENT*((inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5]));
      NominalCurrent = CalcNominal();
      break;
    case IVTS_VOLTAGE_ID:
      IVTSBeat = true;
      IvtsVoltage = int(((inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5])*IVTS_SCALE_VOLTAGE);
      break;
    case IVTS_TEMP_ID: 
      IVTSBeat = true;
      IvtsTemperature = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
      IvtsTemperature = int (IvtsTemperature/10); // not sure about this line
      break;
    case IVTS_POWER_ID:
      IVTSBeat = true;
      IvtsPower = (inMsg.buf[2] << 24) + (inMsg.buf[3] << 16) + (inMsg.buf[4] << 8) + inMsg.buf[5];
      break;
    case AMS_ERROR_ID:
      AMSBeat    = true;
      if (inMsg.buf[2] !=0  ||  inMsg.buf[1] !=0) AMSError =true;
      else AMSError = false;
      // Check if we need Minimum cell boltage Battery SOC and etc
      break;
    case AMS_PRECHARGE_DONE_ID:
      AMSBeat    = true;
      AMSBatteryVoltage = (inMsg.buf[0] << 24) + (inMsg.buf[1] << 16) + (inMsg.buf[2] << 8) + inMsg.buf[3];
      air_plus = inMsg.buf[5]&0x1;
      // Check if we need Battery voltage TODO add Air+ status
      break;
    case AMS_CHARGING_ID:
      AMSBeat    = true;
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
  SevconBeat = true; //  All Can2 Masseges comes from SEVCON
  switch(inMsg.id){
    case SEVCON_THROTTLE_ID:
      SevconThrottle = (inMsg.buf[0] << 8) + inMsg.buf[1];  // 0.1%
      SevconActualTorque = (inMsg.buf[2] << 8) + inMsg.buf[3];  // 0.1%
      break;
    case SEVCON_TORQUE_ID:
      SevconDesiredTorque = (inMsg.buf[2] << 8) + inMsg.buf[3];  // 0.1% 
      SevconDesiredTorqueNM = SEVCON_TORQUE_PRE_TO_NM*SevconDesiredTorque;
      SevconActualTorqueNM = (inMsg.buf[4] << 8) + inMsg.buf[5];  // 0.0625[NM]
      SevconTemperature = (inMsg.buf[6] << 8) + inMsg.buf[7];  // 1[C]  
      break;
    case SEVCON_CAP_VOLTAGE_ID:
      SevconCapVoltage = ((inMsg.buf[1] << 8) + inMsg.buf[0]);  // 0.0625[V]
      SevconHeatSink   = inMsg.buf[2];  // 0.0625 1[C]
      break;
    case SEVCON_VELOCITY_ID:
      SevconVelocity = (inMsg.buf[0] << 24) + (inMsg.buf[1] << 16) + (inMsg.buf[2] << 8) + inMsg.buf[3];  // 1[RPM]
      SevconSpeed = (inMsg.buf[4] << 8) + inMsg.buf[5];  // 0.0625[kph]
      SevconReciveFWRV = inMsg.buf[7];
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
int SerialCount = 0;
void Interrupt_Routine(){
  if (SerialCount%50 == 0)
    Status_Print();
  SerialCount +=1;
}


void Send_Torque() {
  if (FwRevCouter < TorqueDelay){
    FwRevCouter+=1;
    return;
  }
  FwRevCouter=1;
  if(abs(SevconCapVoltage*SEVCON_SCALE_VOLTAGE - IvtsVoltage) > VoltageTollerance)  
  {
    Torque_msg.buf[0] = 0;
    Torque_msg.buf[1] = 0;
  }
  else
  {
    Torque_msg.buf[0] = int((TPS_2_SEVCON_SCALE_Lin + TPS_2_SEVCON_SCALE*PedalThrottle))%256; // Sevcon scale is 0.1% Pedal Controller scale is 1%
    Torque_msg.buf[1] = (TPS_2_SEVCON_SCALE_Lin + TPS_2_SEVCON_SCALE*PedalThrottle)/256; // Sevcon scale is 0.1% Pedal Controller scale is 1%
    if ((state == LIMP_STATE)||(state == REV_STATE)){
      Torque_msg.buf[0] = int(( TPS_2_SEVCON_SCALE_Lin+ TPS_2_SEVCON_SCALE*PedalThrottle/LIMP_DIVISION))%256; // Sevcon scale is 0.1% Pedal Controller scale is 1%, Limp division avoids high power consumption
      Torque_msg.buf[1] = 0;
    }
  }
  if (state == BT_FW_STATE || state == BT_REV_STATE || state == ERROR_STATE || state ==R2D_STATE){
    Torque_msg.buf[1] = 0;
  }
  Torque_msg.id = 0x111;
  Torque_msg.len = 2;
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
  if (ImpCouter < IMPLAUSIBILITY_COUNTER){
    ImpCouter+=1;
    return state;
  }
  if (abs(SEVCON_SCALE_VOLTAGE*SevconCapVoltage - IvtsVoltage) > VoltageTollerance) {
    if (FirstTimeImpV){
      ImpCouter=1;
      FirstTimeImpV=false;
    } else {
      state = ERROR_STATE;
      FirstTimeImpV=true;
      ImpCouter=IMPLAUSIBILITY_COUNTER;
    }

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
  if (!digitalRead(shutdownFB_pin)){
    state = ERROR_STATE;
  }
  if (HeartBeatError){
    state = ERROR_STATE;
  }
  if (IvtsPower> MaxPower){
    state = ERROR_STATE;
  }
  if (ImpCouter < IMPLAUSIBILITY_COUNTER){
    ImpCouter+=1;
    return state;
  }
  if (abs(SEVCON_SCALE_VOLTAGE*SevconCapVoltage - IvtsVoltage) > VoltageTollerance) {
    if (FirstTimeImpV){
      ImpCouter=1;
      FirstTimeImpV=false;
    } else {
      state = ERROR_STATE;
      FirstTimeImpV=true;
      ImpCouter=IMPLAUSIBILITY_COUNTER;
    }

  }
  return state;
}

int CheckHV(){
  if ((IvtsVoltage > TS_VOLTAGE_ON) ||  (SEVCON_SCALE_VOLTAGE*SevconCapVoltage > TS_VOLTAGE_ON)){  // If any of the IVTS voltage measurment or the Sevcon Capacitor voltage higher the 60v
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
int CalcKWTorque(){
  return B_AND_T_NM_RPM_TO_50KW*SevconDesiredTorqueNM*SevconVelocity;
}
int CheckBrakeNThrottle(){
  if ((PedalBrake > HARD_BRAKE_VALUE)&&(PedalThrottle>(BT_MAX_THROTTLE)||(CalcKWTorque()>BT_MAX_TOQUE))){
    bt_counter +=1;
    if  (bt_counter >= BT_MAX_COUNT ){
      if (state == REV_STATE){
          return BT_REV_STATE;
      } else { // state == FW or LIMP
          return BT_FW_STATE;
      }
      
    }
  }
  //else
  return state;
}
int CheckNoThrottle(){
if ((CalcKWTorque()<= MIN_MOTOR_TORQUE)&&(PedalThrottle<=MIN_TPS_THROTTLE)){
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
  if ((AMSError)||(PedalControllerError)||(IvtsVoltage>=TS_VOLTAGE_ON)||((SEVCON_SCALE_VOLTAGE*SevconCapVoltage )>= TS_VOLTAGE_ON)){
    return false;
  }
  //else
  FirstTimeImpV=true;
  ImpCouter=IMPLAUSIBILITY_COUNTER;
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
  if (AMSBatteryVoltage < MIN_BATTERY || NominalCurrent > MAX_NOMIMAL_CURRENT || IvtsPower > MaxPower || IvtsTemperature > MAX_TEMPERATURE){
    state = LIMP_STATE;
  }else{
    state = FW_STATE;
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

void PatchForTorqueTest(){
   Send_Torque();
}
int8_t LoggerCouter = 1;
void SendToLogger(){
   if (LoggerCouter < LOGGER_DELAY){
    LoggerCouter+=1;
    return;
  }
  LoggerCouter=1;
  //LoggerMsg1 
  // Torqe
  LoggerMsg1.buf[1] = (SevconActualTorque%256)/10;       //LSB
  LoggerMsg1.buf[0] = (SevconActualTorque/256);  //MSB
  //Motor Temperature
  LoggerMsg1.buf[3] = SevconTemperature%256;    //LSB
  LoggerMsg1.buf[2] = (SevconTemperature/256);  //MSB
  // Sevcon Voltage
  LoggerMsg1.buf[5] = SevconCapVoltage%256;    //LSB
  LoggerMsg1.buf[4] = (SevconCapVoltage/256);  //MSB
  // Sevcon HeatSink
  LoggerMsg1.buf[6] = SevconHeatSink;      // Check if LSB or MSB
  
  LoggerMsg1.id = 0x50;
  LoggerMsg1.len = 8;
  LoggerMsg1.flags.extended = 0;
  LoggerMsg1.flags.remote   = 0;
  LoggerMsg1.flags.overrun  = 0;
  LoggerMsg1.flags.reserved = 0;
    //LoggerMsg1 
  // Torqe
  LoggerMsg1.buf[0] = (SevconActualTorque%256)/10;       //LSB
  LoggerMsg1.buf[1] = (SevconActualTorque/256);  //MSB
  //Motor Temperature
  LoggerMsg1.buf[2] = SevconTemperature%256;    //LSB
  LoggerMsg1.buf[3] = (SevconTemperature/256);  //MSB
  // Sevcon Voltage
  LoggerMsg1.buf[4] = SevconCapVoltage%256;    //LSB
  LoggerMsg1.buf[5] = (SevconCapVoltage/256);  //MSB
  // Sevcon HeatSink
  LoggerMsg1.buf[6] = SevconHeatSink;      // Check if LSB or MSB
  
  LoggerMsg1.id = 0x50;
  LoggerMsg1.len = 8;
  LoggerMsg1.flags.extended = 0;
  LoggerMsg1.flags.remote   = 0;
  LoggerMsg1.flags.overrun  = 0;
  LoggerMsg1.flags.reserved = 0;
    //LoggerMsg2
  // Velocity
  LoggerMsg2.buf[0] = (SevconVelocity >> 24 )%256;       //MSB
  LoggerMsg2.buf[1] = (SevconVelocity >> 16 )%256;          
  LoggerMsg2.buf[2] = (SevconVelocity >> 8 )%256;             
  LoggerMsg2.buf[3] = (SevconVelocity%256);           //LSB
  // Sevcon Voltage
  LoggerMsg2.buf[5] = SevconSpeed%256;    //LSB
  LoggerMsg2.buf[4] = (SevconSpeed/256);  //MSB
  // TODO change the status - status
  LoggerMsg2.buf[7] =SevconReciveFWRV;

  // To AMS Flow 
  LoggerMsg2.buf[6] = capacitor_high; // is capacitor charged AMS needs to get air+
  if (state <R2D_STATE || state==ERROR_STATE){
    LoggerMsg2.buf[6] &=  ~0x02; // the car not at driving states
  }else{
    LoggerMsg2.buf[6] |= 0x02; // the car at driving states
  }
  if (digitalRead(shutdownFB_pin)){
    LoggerMsg2.buf[6] |=  0x04; // ShutDown close
  }else{
    LoggerMsg2.buf[6] &=  ~0x04; // ShutDown open
  }

  LoggerMsg2.id = 0x51;
  LoggerMsg2.len = 8;
  LoggerMsg2.flags.extended = 0;
  LoggerMsg2.flags.remote   = 0;
  LoggerMsg2.flags.overrun  = 0;
  LoggerMsg2.flags.reserved = 0;

  Can1.write(LoggerMsg1);    //CANBus write command 
  Can1.write(LoggerMsg2);    //CANBus write command 
  
}
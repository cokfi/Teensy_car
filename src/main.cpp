//cpp program for ECU - main by Nimrod and Kfir

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <TimerOne.h>
#include "ecu_functions.h"
#include "ecu_defines.h"


//init
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2  ;

//static CAN_message_t msg;
CAN_message_t Torque_msg;
IntervalTimer myTimer1;                      // Create an IntervalTimer1 object 
int state = LV_STATE;
uint8_t HeartBeatCounter = 0, FwRevCouter = 0, CoolButtonCounter = CoolButtonDelay, relay_counter = 0;
uint8_t DcdcLowCurrent =0, DcdcLowVoltage=0;
uint8_t PedalThrottle = 0, PedalBrake = 0, Battery_Percent, TS_voltage, TS_current, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
uint8_t Charger_flags, voltage_implausibility;
//////////// Sevcon/////////////
uint8_t SevconHeatSink;
uint16_t  SevconTemperature, SevconCapVoltage, SevconSpeed;
int16_t SevconActualTorqueNM, SevconActualTorque,SevconDesiredTorque,SevconThrottle;
int32_t SevconVelocity;
/////////////////////////////
uint16_t R2DCounter = R2DDelay, velocity = 0, NominalCurrent = 0;
bool AMSError = false, PedalControllerError = false, IVTSBeat = false, SevconBeat = false, AMSBeat= false, PedalBeat = false, HeartBeatError = false, TPS_Implausibility = false, MilliSec = true;
uint32_t IvtsPower, IvtsTemperature, IvtsCurrent, IvtsVoltage, AMSBatteryVoltage, MotorTorque, Motor_On, MotorVoltage;
uint32_t GPSVelocity, LoggerTemp1, LoggerTemp2;
CAN_message_t msg ;
bool init_skip = false , air_plus = false, charging = false, ready_to_drive_pressed = false, DcdcOn = false; // first time entering LV state
int cool = 0 , current_list[NOMIMAL_NUM], index_current = 0;
int prev_cool =0, bt_counter=0,desired_motor_torque=0;
bool capacitor_high = false ; // true when capacitor voltage is higher than 95%
bool enable_dcdc = true ; // 
bool open_relay = false;


void setup(void)
{
  Serial.begin(115200);
  int iSerialTimeout = 1000000;
  delay(100);
  while (!Serial && (iSerialTimeout-- != 0));
  ////// Can2 Configurations /////
  Can2.begin();
  Can2.setBaudRate(500000);
  Can2.enableFIFO();
	Can2.enableFIFOInterrupt();
	Can2.setFIFOFilter(REJECT_ALL);
  Can2.setFIFOFilterRange(0, SEVCON_THROTTLE_ID,SEVCON_VELOCITY_ID, STD);
	Can2.onReceive(FIFO, CAN2_Unpack);
  ////// Can1 Configurations /////
  Can1.begin();
  Can1.setBaudRate(500000);// I like to set the baud rates just to be on the safe side
  Can1.enableFIFO();
	Can1.enableFIFOInterrupt();
  Can1.setFIFOFilter(REJECT_ALL);  // For filter Config
	Can1.setFIFOFilter(0, PEDAL_CONTROLLER_ID, STD);   // PedalController
  Can1.setFIFOFilterRange(1, IVTS_CURRENT_ID,IVTS_VOLTAGE_ID, STD);  // IVTS current and voltage meas 
  Can1.setFIFOFilterRange(2, IVTS_TEMP_ID,IVTS_POWER_ID, STD);  // IVTS Temperature and power meas
  Can1.setFIFOFilterRange(3, AMS_ERROR_ID,AMS_CHARGING_ID, STD);  // AMS masseges
  Can1.setFIFOFilterRange(4, LOGGER_START_ID,LOGGER_END_ID, STD);  // DataLogger masseges
  Can1.setFIFOFilterRange(5, DCDC_ON_ID,DCDC_Meas_ID, STD);  // DCDC masseges
	Can1.onReceive(FIFO, CAN1_Unpack);

  pinMode(13,OUTPUT);
      // initialize the digital pin as an output.
  pinMode(AvoidDischarge_pin, OUTPUT);
  pinMode(Pump1_pin, OUTPUT);
  pinMode(Pump2_pin, OUTPUT);
  pinMode(TsoffLed_pin, OUTPUT);
  pinMode(ForwardMotor_pin, OUTPUT);
  pinMode(ReverseMotor_pin, OUTPUT);
  pinMode(EnableBuzzer_pin, OUTPUT);
  pinMode(Collingfan1_pin, OUTPUT);
  pinMode(Collingfan2_pin, OUTPUT);
  pinMode(Ecufault_pin, OUTPUT);

  
  pinMode(ForwardSwitch_pin, INPUT);
  pinMode(ReverseSwitch_pin, INPUT);
  pinMode(R2Dbutton_pin, INPUT);
  pinMode(ForceCooling_pin, INPUT);
  pinMode(shutdownFB_pin, INPUT);

  myTimer1.begin(CheckPowerAnd, DelayMs); 
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(Interrupt_Routine); 
  
}

void loop() {
  Can1.events();
  if (MilliSec){
    PatchForTorqueTest();
    if (false){
    HeartBeatAISP();
    switch (state)
      {
      case LV_STATE:
          // init
          if (!init_skip){
            digitalWrite(TsoffLed_pin,HIGH);
            digitalWrite(AvoidDischarge_pin,LOW);
            init_skip = true;
          }
          // Ts off led
          if ((air_plus) ||(digitalRead(shutdownFB_pin))||(state==HV_STATE)) { //air+ rellay is closed OR Shutdown circut is closed 
            digitalWrite(TsoffLed_pin,LOW);
          }
          // cooling
          cool = CheckCooling(cool);//TODO pushbutton function
          // change state
          state = CheckHV();//check if high voltage
          state = LVError();// check if low voltage error
          if (state!=LV_STATE){ 
            init_skip = false;
          } 
          break;
      
      case HV_STATE:
          // init
          if (!init_skip){
            digitalWrite(TsoffLed_pin,LOW);
            digitalWrite(AvoidDischarge_pin,LOW);
            init_skip = true;
          }
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // capacitor
          if (MotorVoltage>CAP_CHARGED){ // capacitor is charged to 95% or higher voltage
            capacitor_high =true;
          }
          // DC-DC  
          DcDcCheck();
          // change state
          state = CheckR2D() ; // check if ready 2 drive
          state = HVError() ; // check if high voltage error
          if (state!=HV_STATE){ 
            init_skip = false;
          } 
          break;

      case R2D_STATE:
          // init
          if (!init_skip){
            digitalWrite(TsoffLed_pin,HIGH);
            digitalWrite(AvoidDischarge_pin,LOW);
            init_skip = true;
          }
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;

          // change state
          state = LeaveR2D(); // TODO create function
          state = HVError() ; // check if high voltage error
          if (state!=HV_STATE){ 
            R2DCounter = R2DDelay;
            digitalWrite(EnableBuzzer_pin,LOW);
            ready_to_drive_pressed = false;
            init_skip = false;
          } 
          break;
      case FW_STATE:
          // init
          if (!init_skip){
            digitalWrite(ForwardMotor_pin,HIGH);
            init_skip = true;
          }
          //Send_Torque
          Send_Torque();
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckLimp(); // TODO create function
          state = CheckBrackNThrottle(); // TODO create function
          if (!digitalRead(ForwardSwitch_pin)){ // if forward ==0
              state = R2D_STATE;
          }
          state = HVError() ; // check if high voltage error
          if (state!=FW_STATE){ 
            digitalWrite(ForwardMotor_pin,LOW);
            init_skip = false;
          }         
          break;

      case REV_STATE:
          // init
          if (!init_skip){
            digitalWrite(ReverseMotor_pin,HIGH);
            init_skip = true;
          }
          //Send_Torque
          Send_Torque();
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckBrackNThrottle(); // TODO create function
          if (state == BT_FW_STATE){
            state = BT_REV_STATE;
          }
          
          if (!digitalRead(ReverseSwitch_pin)){ // if forward ==0
              state = R2D_STATE;
          }
          state = HVError() ; // check if high voltage error
          if (state!=REV_STATE){ 
            digitalWrite(ReverseMotor_pin,LOW);
            init_skip = false;
          }       
          break;

      case BT_REV_STATE:
          // init
          if (!init_skip){
            //Send Throttle = 0, the function checks the current state
            Send_Torque();
            init_skip = true;
          }
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckNoThrottle(); // TODO create function
          if (state == FW_STATE){
            state = REV_STATE;
          }
          
          if (!digitalRead(ReverseSwitch_pin)){ // if Reverse ==0
              state = R2D_STATE;
          }
          state = HVError() ; // check if high voltage error
          break;

      case BT_FW_STATE:
          //Send Throttle = 0, the function checks the current state
          Send_Torque();
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckNoThrottle(); // TODO create function
          
          if (!digitalRead(ForwardSwitch_pin)){ // if Forward ==0
              state = R2D_STATE;
          }
          state = HVError() ; // check if high voltage error
          break;

      case ERROR_STATE:
          // init
          if (!init_skip){
            //Send Throttle = 0, the function checks the current state
            Send_Torque();
            if (TS_voltage>=TS_VOLTAGE_ON){
              digitalWrite(TsoffLed_pin,LOW);
            }
            init_skip = true;
          }
          open_relay=WaitRelay(); // true after 250 milli seconds
          //open shutdown circut if allowed
          if (open_relay || digitalRead(shutdownFB_pin)){
            digitalWrite(Ecufault_pin,LOW); // TODO check if it is normally open
          //discharge if allowed
            if (velocity < MIN_SPEED_FOR_DISCHARGE) {
              digitalWrite(AvoidDischarge_pin,LOW);
              if (TS_voltage<TS_VOLTAGE_ON){
                digitalWrite(TsoffLed_pin,HIGH);
          //change state
                if(AllOk()){ // TODO function
                  state = LV_STATE;
                }
              }

            }
          }
          if (state!=ERROR_STATE){ 
            init_skip = false;
          }      
          break;

      case LIMP_STATE: 
          // init
          if (!init_skip){
            digitalWrite(ForwardMotor_pin,HIGH);
            init_skip = true;
          }
          //Send Limped Torque, the function checks the current state
          Send_Torque();
          // cooling
          cool= CheckCooling(cool); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckLimp(); // TODO create function
          state = CheckBrackNThrottle(); // TODO create function
          if (!digitalRead(ForwardSwitch_pin)){ // if forward ==0
              state = R2D_STATE;
          }
          state = HVError() ; // check if high voltage error
          if (state!=LIMP_STATE){ 
            digitalWrite(ForwardMotor_pin,LOW);
            init_skip = false;
          }       
          break;


      };
    }
  } 
}
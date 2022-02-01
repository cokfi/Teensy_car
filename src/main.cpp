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
CAN_message_t torqe_msg;
IntervalTimer myTimer1;                      // Create an IntervalTimer1 object 
int state = LV_STATE;
uint8_t HeartBeatCounter = 0, FwRevCouter = 0, CoolButtonCounter = CoolButtonDelay;
uint8_t low_current =0, low_voltage=0;
uint8_t Throttle = 0, Brake = 0, Battery_Percent, TS_voltage, TS_current, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
uint8_t Charger_flags, voltage_implausibility;
uint16_t R2DCounter = R2DDelay;
bool AMSError = false, PedalControllerError = false, IVTSBeat = false, SevconBeat = false, AMSBeat= false, PedalBeat = false, HeartBeatError = false, TPS_Implausibility = false, MilliSec = true;
uint32_t Power_meas, Temperature_meas, Current_meas, Voltage_meas1, Voltage_meas2, Voltage_meas3, Battery_Voltage, Motor_Torqe, Motor_On, Motor_Voltage;
static CAN_message_t msg ;
bool init_skip = false , air_plus = false, charging = false; // first time entering LV state
int cool = 0 ;
int prev_cool =0;
bool capacitor_high = false ; // true when capacitor voltage is higher than 95%
bool enabe_dcdc = true ; // 
bool open_relay = false;

void setup(void)
{
  Serial.begin(115200);
  int iSerialTimeout = 1000000;
  delay(100);
  while (!Serial && (iSerialTimeout-- != 0));
  Can1.begin();
  Can2.begin();
  Can1.setBaudRate(500000);// I like to set the baud rates just to be on the safe side
  Can2.setBaudRate(500000);
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
  // Mailbox setup
  Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); //Configuration of all Recived MB
  Can2.setMaxMB(11); //Configuration of all Recived MB
  for (uint8_t i = 0; i < NUM_RX_MAILBOXES; i++) {
    Can1.setMB(i, RX, STD);
  }
  Can1.setMBFilter(REJECT_ALL);
  Can1.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled
  Can2.setMBFilter(REJECT_ALL);
  Can2.enableMBInterrupts(); // enables all mailboxes to be interrupt enabled

  // Set mailbox 0-11 to allow CAN IDs to be collected.
  Can1.setMBFilter(MB0, 0x40);
  Can1.setMBFilter(MB1, 0x521);
  Can1.setMBFilter(MB2, 0x522);
  Can1.setMBFilter(MB3, 0x523);
  Can1.setMBFilter(MB4, 0x524);
  Can1.setMBFilter(MB5, 0x525);
  Can1.setMBFilter(MB6, 0x526);
  Can1.setMBFilter(MB7, 0x624);
  Can1.setMBFilter(MB8, 0x324);
  Can1.setMBFilter(MB9, 0x626);
  Can1.setMBFilter(MB10, 0x627);
  Can1.setMBFilter(MB11, 0x80);



// allows mailbox messages to be received in the supplied callbacks.
  Can1.onReceive(MB0, PedalControllerMB);
  Can1.onReceive(MB1, CurrentMeasMB);
  Can1.onReceive(MB2, VoltageMeasure1MB);
  Can1.onReceive(MB3, VoltageMeasure2MB);
  Can1.onReceive(MB4, VoltageMeasure3MB);
  Can1.onReceive(MB5, TemperatureMeasureMB);
  Can1.onReceive(MB6, PowerMeasure);
  Can1.onReceive(MB7, BatteryStateMB);
  Can1.onReceive(MB8, BatterySOCPercentMB);
  Can1.onReceive(MB9, BatteryVoltageMB);
  Can1.onReceive(MB10, ChargerFlagsMB);
  Can1.onReceive(MB11, MotorControllerMB);
  Can1.mailboxStatus();
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(Interrupt_Routine); 
  
}

void loop() {
  Can1.events();
  if (MilliSec){
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
          if (air_plus) ||(digitalRead(shutdownFB_pin))||(state==HV_STATE) { //air+ rellay is closed OR Shutdown circut is closed 
            digitalWrite(TsoffLed_pin,LOW);
          }
          // cooling
          disp_hv_needed = CheckCooling();//TODO pushbutton function
          // change state
          state = CheckHV(state);//check if high voltage
          state = LVError(state);// check if low voltage error
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
          cool= CheckCooling(); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // capacitor
          if (Motor_Voltage>CAP_CHARGED){ // capacitor is charged to 95% or higher voltage
            capacitor_high =true;
          }
          // DC-DC  
          DcDcCheck();
          // change state
          state = CheckR2D(state) ; // check if ready 2 drive
          state = HVError(state) ; // check if high voltage error
          if (state!=HV_STATE){ 
            init_skip = false;
          } 
          break;

      case R2D_STATE:

          // cooling
          cool= CheckCooling(); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;

          // change state
          state = LeaveR2D; // TODO create function
          state = HVError(state) ; // check if high voltage error

          break;
      case FW_STATE:
          // init
          if (!init_skip){
            digitalWrite(ForwardMotor_pin,HIGH)
            init_skip = true;
          }
          //Send_Tourqe
          Send_Tourqe();
          // cooling
          cool= CheckCooling(); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckLimp(); // TODO create function
          state = CheckHardBrake(); // TODO create function
          if (!digitalRead(ForwardSwitch_pin)){ // if forward ==0
              state = R2D_STATE;
          }
          state = HVError(state) ; // check if high voltage error
          if (state!=FW_STATE){ 
            digitalWrite(ForwardMotor_pin,LOW);
            init_skip = false;
          }         
          break;

      case REV_STATE:
          // init
          if (!init_skip){
            digitalWrite(ReverseMotor_pin,HIGH)
            init_skip = true;
          }
          //Send_Tourqe
          Send_Tourqe();
          // cooling
          cool= CheckCooling(); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckHardBrake(); // TODO create function
          if (state == BT_FW_STATE){
            state = BT_REV_STATE;
          }
          
          if (!digitalRead(ReverseSwitch_pin)){ // if forward ==0
              state = R2D_STATE;
          }
          state = HVError(state) ; // check if high voltage error
          if (state!=REV_STATE){ 
            digitalWrite(ReverseMotor_pin,LOW);
            init_skip = false;
          }       
          break;

      case BT_REV_STATE:
          // init
          if (!init_skip){
            //Send Throttle = 0, the function checks the current state
            Send_Tourqe();
            init_skip = true;
          }
          // cooling
          cool= CheckCooling(); // TODO create function
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
          state = HVError(state) ; // check if high voltage error
          break;

      case BT_FW_STATE:
          //Send Throttle = 0, the function checks the current state
          Send_Tourqe();
          // cooling
          cool= CheckCooling(); // TODO create function
          if (cool!=prev_cool){
            EnableCooling(cool); //TODO create function
          }
          prev_cool = cool;
          // change state
          state = CheckNoThrottle(); // TODO create function
          
          if (!digitalRead(ForwardSwitch_pin)){ // if Forward ==0
              state = R2D_STATE;
          }
          state = HVError(state) ; // check if high voltage error
          break;

      case ERROR_STATE:
          // init
          if (!init_skip){
            //Send Throttle = 0, the function checks the current state
            Send_Tourqe();
            if (TS_voltage>=60){
              digitalWrite(TsoffLed_pin,LOW);
            }
            init_skip = true;
          }
          open_relay=WaitDischarge(); // true after 250 milli seconds
          //open shutdown circut if allowed
          if (open_relay || digitalRead(shutdownFB_pin)){
            digitalWrite(Ecufault_pin);
          //discharge if allowed
            if (speed<MIN_SPEED_FOR_DISCHARGE) {
              digitalWrite(AvoidDischarge_pin,LOW);
              if (TS_voltage<60){
                digitalWrite(TsoffLed_pin,HIGH);
          //change state
                if(AllOk()){ // TODO function
                  state = LV_STATE;
                }
              }

            }
          }
          break;

      case LIMP_STATE:
          // do stuff
          // maybe change state
          break;


  
    }
  } 
}
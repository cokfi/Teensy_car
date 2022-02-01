#ifndef ecu_defines_h
#define ecu_defines_h

//mailboxes
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 12
//Output
#define AvoidDischarge_pin 17
#define Pump1_pin 6
#define Pump2_pin 7
#define TsoffLed_pin  33
#define ForwardMotor_pin 31
#define ReverseMotor_pin 32
#define EnableBuzzer_pin 37
#define Collingfan1_pin 39
#define Collingfan2_pin 38
#define Ecufault_pin 14

//Input
#define ForwardSwitch_pin 12
#define ReverseSwitch_pin 24
#define R2Dbutton_pin 29
#define ForceCooling_pin 30
#define shutdownFB_pin 15

//states
#define LV_STATE 1 // low voltage state
#define HV_STATE 2 // high voltage state
#define R2D_STATE 3 // ready to drive-Neutral state
#define FW_STATE 4 // forward state
#define REV_STATE 5 // Reverse state
#define BT_REV_STATE 6 // brake and throttle-Reverse state
#define BT_FW_STATE 7 // brake and throttle-Forward state
#define ERROR_STATE 8
#define LIMP_STATE 9
// Values
#define VoltageTollerance 40
#define TorqueDelay 5   // 5 MilliSec for Each torqe send
#define DelayMs 1000    // Main delay, each ms the main loop on
#define MaxPower 10000  // 10000 Watt
#define CAP_CHARGED 330 // capacitor [V] 95% TODO change value and units 
#define HeartBeatCounterMaxValue 100    //100 ms, if no response from all unit go to error
#define CoolingReqTemp 50   
#define CoolButtonDelay 50
#define R2DDelay 2000
#define CoolingOff 0
#define CoolingTempHigh 1
#define ForcedCoolingVal 2
#define MAX_LOW_VOLTAGE 15
#define MAX_LOW_CURRENT 5
#define MinBrakeR2D 5
#define LIMP_DIVISION 5
#define BT_MAX_THROTTLE 50 //half of max throttle brake&Throttle state
#define BT_MAX_TOQUE 50 // max tourqe brake&Throttle state
#define TS_VOLTAGE_ON 60 // [volt]
#endif
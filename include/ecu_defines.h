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
#define TorqueDelay 500000
#define CheckOnDelay 100000
#define DelayMs 1000
#define MaxPower 10000
#define CAP_CHARGED 330 // capacitor [V] 95% TODO change value and units 
#endif
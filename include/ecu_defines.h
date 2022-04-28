#ifndef ecu_defines_h
#define ecu_defines_h

//Canbus ID
// Can 1
#define PEDAL_CONTROLLER_ID     0x40

#define IVTS_CURRENT_ID         0x521
#define IVTS_VOLTAGE_ID         0x522
#define IVTS_TEMP_ID            0x525
#define IVTS_POWER_ID           0x526

#define AMS_ERROR_ID            0x124
#define AMS_PRECHARGE_DONE_ID   0x125
#define AMS_CHARGING_ID         0x126

#define LOGGER_START_ID         0x300
#define LOGGER_END_ID           0x301

#define DCDC_ON_ID              0x18D
#define DCDC_Meas_ID            0x28D
// Can 2
#define 
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
#define VoltageTollerance           40      // Temporary value
#define TorqueDelay                 5       // 5 MilliSec for Each torqe send   // Temporary value
#define DelayMs                     1000    // Main delay, each ms the main loop on
#define MaxPower                    10000   // 10000 Watt   // Temporary value
#define CAP_CHARGED                 330     // capacitor [V] 95% TODO change value and units    // Temporary value
#define HeartBeatCounterMaxValue    100     //100 ms, if no response from all unit go to error  // Temporary value
#define CoolingReqTemp              50      // Temporary value
#define CoolButtonDelay             50      // Temporary value
#define R2DDelay                    2000
#define R2D_BUTTON_DELAY            100     // Temporary value
#define CoolingOff                  0    
#define CoolingTempHigh             1       // Temporary value
#define ForcedCoolingVal            2       // Temporary value
#define MAX_LOW_VOLTAGE             15      // Temporary value
#define MAX_LOW_CURRENT             5       // Temporary value
#define MinBrakeR2D                 5       // Temporary value
#define MIN_SPEED_FOR_DISCHARGE     20      // Temporary value
#define MIN_SPEED_TO_REV            10      // Temporary value
#define MIN_BATTERY                 11      // Temporary value
#define MAX_NOMIMAL_CURRENT         5       // Temporary value  
#define MAX_TEMPERATURE             60      // Temporary value
#define NOMIMAL_NUM                 10     
#define RELAY_DELAY_ERROR           250
#define LIMP_DIVISION 5
#define BT_MAX_THROTTLE 50 //half of max throttle brake&Throttle state
#define BT_MAX_TOQUE 50 // max tourqe brake&Throttle state
#define TS_VOLTAGE_ON 60 // [volt]
#define BT_MAX_COUNT 500 //ms
#define MIN_TPS_THROTTLE 5 //TEMP
#define MIN_MOTOR_TORQUE 5 //TEMP
#define HARD_BRAKE_VALUE 50 // TEMP
#endif
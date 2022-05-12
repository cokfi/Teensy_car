#ifndef ecu_functions_h
#define ecu_functions_h
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ecu_defines.h"

// -------------------------------------------------------------
void Status_Print(void) ;

void CAN1_Unpack(const CAN_message_t &msg);
void CAN2_Unpack(const CAN_message_t &msg);
void PatchForTorqueTest();
void Interrupt_Routine();
void Send_Torque();

int LVError();
int HVError();
int CheckHV();
int CheckCooling(int cool);
int CheckR2D();
int CheckBrakeNThrottle();  // 
int CheckNoThrottle();
int LeaveR2D();
int CheckLimp();
int CalcNominal();
bool WaitRelay();
void SendToLogger();

void EnableCooling(int cool);
void DcDcCheck();
void CheckPowerAnd();
void HeartBeatAISP();    // AISP - AMS, IVTS, Sevcon, Pedal Controller
bool AllOk();

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
//extern myTimer1;                      // Create an IntervalTimer1 object ;
extern CAN_message_t msg;
extern CAN_message_t Torque_msg, LoggerMsg1, LoggerMsg2;
extern int8_t state, current_list[NOMIMAL_NUM], index_current,bt_counter;

extern uint8_t HeartBeatCounter, FwRevCouter, CoolButtonCounter, R2DButtonCounter, relay_counter;
extern uint8_t DcdcLowCurrent, DcdcLowVoltage,air_plus,capacitor_high;
extern uint8_t PedalThrottle , PedalBrake , TS_voltage, TS_current, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
extern uint8_t Charger_flags;
extern int32_t IvtsPower, IvtsTemperature, AMSBatteryVoltage;
extern int32_t IvtsVoltage, IvtsCurrent;
extern bool AMSError, PedalControllerError,IVTSBeat, SevconBeat, AMSBeat, PedalBeat, HeartBeatError, TPS_Implausibility, MilliSec, charging, ready_to_drive_pressed, DcdcOn,FirstTimeImpV;
extern uint16_t R2DCounter, velocity, NominalCurrent;
extern int32_t GPSVelocity, LoggerTemp1, LoggerTemp2;
//////////////// Sevcon //////////////////
extern uint8_t SevconHeatSink,SevconReciveFWRV;
extern uint16_t SevconTemperature, SevconCapVoltage, SevconSpeed;
extern int16_t SevconActualTorqueNM, SevconActualTorque,SevconDesiredTorque,SevconThrottle, SevconDesiredTorqueNM;
extern int32_t SevconVelocity;
extern int16_t ImpCouter; // IMPLUSIbillity COuntewr

///// integration variables

extern int stateUsage[];
#endif

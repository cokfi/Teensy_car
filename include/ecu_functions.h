#ifndef ecu_functions_h
#define ecu_functions_h
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ecu_defines.h"

// -------------------------------------------------------------
void Status_Print(void) ;

void CAN1_Unpack(const CAN_message_t &msg);
void CAN2_Unpack(const CAN_message_t &msg);
/*
void Print_CanMsg(const CAN_message_t &msg);
void PedalControllerMB(const CAN_message_t &inMsg);
void CurrentMeasMB(const CAN_message_t &inMsg);
void VoltageMeasure1MB(const CAN_message_t &inMsg);
void VoltageMeasure2MB(const CAN_message_t &inMsg);
void VoltageMeasure3MB(const CAN_message_t &inMsg);
void TemperatureMeasureMB(const CAN_message_t &inMsg);
void PowerMeasure(const CAN_message_t &inMsg);
void BatteryStateMB(const CAN_message_t &inMsg);
void BatterySOCPercentMB(const CAN_message_t &inMsg);
void BatteryVoltageMB(const CAN_message_t &inMsg);
void ChargerFlagsMB(const CAN_message_t &inMsg);
void MotorControllerMB(const CAN_message_t &inMsg);
*/
void Interrupt_Routine();
void Send_Torque();

int LVError();
int HVError();
int CheckHV();
int CheckCooling(int cool);
int CheckR2D();
int CheckBrackNThrottle();  // 
int CheckNoThrottle();
int LeaveR2D();
int CheckLimp();
int CalcNominal();
bool WaitRelay();

void EnableCooling(int cool);
void DcDcCheck();
void CheckPowerAnd();
void HeartBeatAISP();    // AISP - AMS, IVTS, Sevcon, Pedal Controller
bool AllOk();

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
//extern myTimer1;                      // Create an IntervalTimer1 object ;
extern CAN_message_t msg;
extern CAN_message_t Torque_msg;
extern int state, current_list[NOMIMAL_NUM], index_current,bt_counter,desired_motor_torque;

extern uint8_t HeartBeatCounter, FwRevCouter, CoolButtonCounter, R2DButtonCounter, relay_counter;
extern uint8_t DcdcLowCurrent, DcdcLowVoltage;
extern uint8_t PedalThrottle , PedalBrake , Battery_Percent, TS_voltage, TS_current, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
extern uint8_t Charger_flags, voltage_implausibility;
extern uint32_t IvtsPower, IvtsTemperature, IvtsCurrent, IvtsVoltage, Battery_Voltage, MotorTorque, Motor_On, MotorVoltage;
extern bool AMSError, PedalControllerError,IVTSBeat, SevconBeat, AMSBeat, PedalBeat, HeartBeatError, TPS_Implausibility, MilliSec, charging, air_plus, ready_to_drive_pressed, DcdcOn;
extern uint16_t R2DCounter, velocity, NominalCurrent;
extern uint32_t GPSVelocity, LoggerTemp1, LoggerTemp2;
#endif

#ifndef ecu_functions_h
#define ecu_functions_h
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ecu_defines.h"

// -------------------------------------------------------------
void Status_Print(void) ;


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
void Interrupt_Routine();
void Send_Tourqe();

int LVError();
int HVError();
int CheckHV();
int CheckCooling(int cool);
int CheckR2D();
int CheckBrakeNThrottle();  // 
int CheckNoThrottle();
int LeaveR2D();
int CheckLimp();
int CalcNonimal();
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
extern CAN_message_t torqe_msg;
extern int state, current_list[NOMIMAL_NUM], index_current,bt_counter,desired_motor_torque;

extern uint8_t HeartBeatCounter, FwRevCouter, CoolButtonCounter, R2DButtonCounter, relay_counter;
extern uint8_t low_current, low_voltage;
extern uint8_t Throttle , Brake , Battery_Percent, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
extern uint8_t Charger_flags, voltage_implausibility;
extern uint32_t Power_meas, Temperature_meas, Current_meas, Voltage_meas1, Voltage_meas2, Voltage_meas3, Battery_Voltage, Motor_Torqe, Motor_On, Motor_Voltage;
extern bool hard_brake, AMSError, PedalControllerError,IVTSBeat, SevconBeat, AMSBeat, PedalBeat, HeartBeatError, TPS_Implausibility, MilliSec, charging, air_plus, ready_to_drive_pressed;
extern uint16_t R2DCounter, velocity, nominal_current;
#endif

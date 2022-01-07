#ifndef ecu_functions_h
#define ecu_functions_h
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ecu_defines.h"

// -------------------------------------------------------------
void Status_Print(void) ;


void Print_CanMsg(void);
void PedalControllerMB(void);
void CurrentMeasMB(void);
void VoltageMeasure1MB(void);
void VoltageMeasure2MB(void);
void VoltageMeasure3MB(void);
void TemperatureMeasureMB(void);
void PowerMeasure(void);
void BatteryStateMB(void);
void BatterySOCPercentMB(void);
void BatteryVoltageMB(void);
void ChargerFlagsMB(void);
void MotorControllerMB(void);
void Interrupt_Routine(void);
void Send_Tourqe(void);

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
//extern myTimer1;                      // Create an IntervalTimer1 object ;

extern CAN_message_t torqe_msg;

extern uint8_t Throttle , Brake , TPS_Implausibility , Battery_Percent, TS_voltage, TS_current, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
extern uint8_t Charger_flags, voltage_implausibility;
extern uint32_t Power_meas, Temperature_meas, Current_meas, Voltage_meas1, Voltage_meas2, Voltage_meas3, Battery_Voltage, Motor_Torqe, Motor_On, Motor_Voltage;
#endif

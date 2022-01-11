//cpp program for ECU - main by Nimrod and Kfir

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <TimerOne.h>
#include "ecu_functions.h"
#include "ecu_defines.h"



FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2  ;

//static CAN_message_t msg;
CAN_message_t torqe_msg;
IntervalTimer myTimer1;                      // Create an IntervalTimer1 object 
int state = LV_STATE;
uint8_t Throttle = 0, Brake = 0, Battery_Percent, TS_voltage, TS_current, Acc_temperature, AMS_Shutdown, Battery_SOC_percent, Battery_state, AMS_flag_msg;
uint8_t Charger_flags, voltage_implausibility;
bool AMSError = false, PedalControllerError = false, IVTSBeat = false, SevconBeat = false, AMSBeat= false, PedalBeat = false, HeartBeatError = false, TPS_Implausibility = false;
uint32_t Power_meas, Temperature_meas, Current_meas, Voltage_meas1, Voltage_meas2, Voltage_meas3, Battery_Voltage, Motor_Torqe, Motor_On, Motor_Voltage;
static CAN_message_t msg;
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
  pinMode(discharge_pin, OUTPUT);
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

  myTimer1.begin(Send_Tourqe, TorqueDelay);                         // Send CAN messages through SendAnalog every 500ms
  myTimer2.begin(HeartBeatAISP, CheckOnDelay);                         // AISP - AMS, IVTS, Sevcon, Pedal Controller
  myTimer3.begin(CheckPowerAnd, DelayMs); 
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
  Timer1.attachInterrupt(Interrupt_Routine); // blinkLED to run every 0.15 seconds
  Timer2.initialize(1000000); 
  Timer3.initialize(1000000);
}

void loop() {
  Can1.events();

  switch (state)
    {

    case LV_STATE:
        // do stuff
        // maybe change state
        break;

    case HV_STATE:
        // do stuff
        // maybe change state
        break;

    case R2D_STATE:
        



        // maybe change state
        break;
    case FW_STATE:
        // do stuff
        // maybe change state
        break;

    case REV_STATE:
        // do stuff
        // maybe change state
        break;

    case BT_REV_STATE:
        // do stuff
        // maybe change state
        break;
    case BT_FW_STATE:
        // do stuff
        // maybe change state
        break;

    case ERROR_STATE:
        // do stuff
        // maybe change state
        break;

    case LIMP_STATE:
        // do stuff
        // maybe change state
        break;

    // ...

    }


  /*
  // put your main code here, to run repeatedly:
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(1000);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(1000);                  // wait for a second
  */
 
}
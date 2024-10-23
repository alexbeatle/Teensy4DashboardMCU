#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1; // Using CAN1 on pins 22/rx & 23/tx

//bms status
#define BMS_Boot 0
#define BMS_Ready 1
#define BMS_Drive 2
#define BMS_Charge 3
#define BMS_Precharge 4
#define BMS_Error 5

//ibooster messages
//IBST_driverBrakeApply
#define BRAKES_NOT_APPLIED 1
#define DRIVER_APPLYING_BRAKES 2
#define FAULT 3
#define NOT_INIT_OR_OFF 0
//IBST_iBoosterStatus
#define IBOOSTER_ACTIVE_GOOD_CHECK 4
#define IBOOSTER_ACTUATION 6
#define IBOOSTER_DIAGNOSTIC 3
#define IBOOSTER_FAILURE 2
#define IBOOSTER_INIT 1
#define IBOOSTER_OFF 0
#define IBOOSTER_READY 5
//IBST_internalState
#define DIAGNOSTIC 4
#define EXTERNAL_BRAKE_REQUEST 3
#define LOCAL_BRAKE_REQUEST 2
#define NO_MODE_ACTIVE 0
#define POST_DRIVE_CHECK 6
#define PRE_DRIVE_CHECK 1
#define TRANSITION_TO_IDLE 5
#define IBST_Invalid 0xF

//drive status
#define Drive_N 0
#define Drive_F -1
#define Drive_R 1
#define Drive_Invalid 15

#define Drive_OPMODE_Off 0
#define Drive_OPMODE_Run 1
#define Drive_OPMODE_ManualRun 2
#define Drive_OPMODE_Boost 3
#define Drive_OPMODE_Buck 4
#define Drive_OPMODE_Sine 5
#define Drive_OPMODE_ACHeat 6
#define Drive_OPMODE_Invalid 15

#define Drive_Stat_None 0
#define Drive_Stat_VdcLow 1
#define Drive_Stat_VdcHigh 2
#define Drive_Stat_VdcBelowUdcSw 4
#define Drive_Stat_VdcLim 8
#define Drive_Stat_EmcyStop 16
#define Drive_Stat_MProt 32
#define Drive_Stat_PotPressed 64
#define Drive_Stat_TmpHs 128
#define Drive_Stat_WaitStart 256
#define Drive_Stat_BrakeCheck 512
#define Drive_Stat_Invalid 0xFFFF

//watchdog
uint32_t Debug_Stamp = millis();
uint32_t IBooster_watchdog = millis();
uint32_t DashUpdate_Stamp = millis();
uint32_t LED_watchdog = millis();
uint32_t LIM_watchdog = millis();
uint32_t ISA_watchdog = millis();
uint32_t BMS_watchdog = millis();
uint32_t Elcon_watchdog = millis();
uint32_t Drive_watchdog = millis();
bool flash=false;
// uint32_t Timeout = millis();

// CAN bus setup
CAN_message_t msg;

//ibooster
const uint16_t canID_stsIBooster = 0x39D;
uint8_t IBST_driverBrakeApply=IBST_Invalid;
uint8_t IBST_iBoosterStatus=IBST_Invalid;
uint8_t IBST_internalState=IBST_Invalid;
uint16_t IBST_rodLen=0xffff;

// ISA
const uint16_t  canID_ISAsoc = 0x350; // set this to match your SoC CAN bus ID
const uint16_t  canID_ISAmilliAmps = 0x521; // ISA miliAmps
const uint16_t  canID_ISAVolt1 = 0x522; // ISA voltage1
const uint16_t  canID_ISAVolt2 = 0x523; // ISA voltage1
const uint16_t  canID_ISAVolt3 = 0x525; // ISA voltage2
uint16_t ISA_BatVolt=0;                          // Battery voltage 0.1V steps up to 819V
uint16_t ISA_MainPOSVolt=0;                      // Main POS voltage battery 0.1V steps up to 818V
uint16_t ISA_MainNEGVolt=0;                      // Main POS voltage battery 0.1V steps up to 818V
uint16_t ISA_BatAmp=0;                          // Battery current 0.1A steps -819.2A to 819A
// uint16_t SOC_Bat_Act=0;                        // Battery SOC actual from ISA

//BMS
const uint16_t  canID_BMSLimits = 0x351; //LSB+MSB: 0,1-Charge Volt, 0.1V; 2,3 - Charge Cur, 0.1A; 4,5 Discharge Cur, 0.1A; Discharge Volt, 0.1A
const uint16_t  canID_BMSSOC = 0x355; //LSB+MSB: 0,1-SOC, 1%; 2,3-SOH,1%; 4,5-SOC, 0.01%
const uint16_t  canID_BMSStatus = 0x356; //LSB+MSB: 0,1-Volt, 0.01V; 2,3-Cur, 0.1A; 4,5-Temp, 0.1C
const uint16_t  canID_BMSWarnings = 0x35A; //3-undervolt; 4-overvolt; 7-overtemp; 12-undertemp
const uint16_t  canID_BMSLowHigh = 0x373; //LSB+MSB: 0,1 - Min Cell Volt, 1V; 2,3 - Max Cell Volt, 1V; 4,5 - Min Cell Temp, 1K; 6,7 - Max Cell Temp, 1K; 
const uint16_t  canID_BMSInfo = 0x379; //4 - BMS Status: .0:Boot, .1:Ready, .2:Drive, .3:Charge, .4:Precharge, .5:Error
const uint16_t  canID_BMSNumbModules = 0x372;
int BMS_State = 0; //
int BMS_CellsBal = 0xff;
uint16_t BMS_NumbModules = 0xffff;
long BMS_ChargeVoltLim=0;
long BMS_ChargeAmpLim=0;
long BMS_disChargeVoltLim=0;
long BMS_disChargeAmpLim=0;
uint16_t BMS_SOC=0;
float BMS_CellsVoltMin=0;
float BMS_CellsVoltMax=0;
float BMS_CellsTempMin=0;
float BMS_CellsTempMax=0;
uint16_t BMS_CapacityAh=0;
uint16_t BMS_CapacityWhCharge=0;
long BMS_PackVolt=0;
float BMS_AvgTemp=0;
long BMS_BatAmp=0;

//Drive
const uint16_t canID_Drive = 0x100; 
const uint16_t canID_DriveRPM = 0x101; 
int8_t Drive_Dir=Drive_Invalid;
uint8_t Drive_OpMode=Drive_OPMODE_Invalid;
uint16_t Drive_Status=Drive_Stat_Invalid;
uint16_t Drive_MotorTemp=0;
uint16_t Drive_HtSnkTemp=0;
uint16_t Drive_RPM=0xFFFF;
String strTemp, strDriveStatNone, strDriveStatVdcLow, strDriveStatVdcHigh, strDriveStatVdcBelowVdcSw, strDriveStatVdcLim, strDriveStatEStop, strDriveStatMProt, strDriveStatPotPressed, strDriveStatTmpHs, strDriveStatWaitStart, strDriveStatBrakeCheck, strDriveStatInvalid;

//Elcon 
// Charger
const uint32_t  canID_ElconChargerCtrl = 0x1806E5F4;
const uint32_t  canID_ElconChargerFback = 0x18FF50E5;
long ElconCharger_VoltOutput=0;
long ElconCharger_AmpOutput=0;
long ElconCharger_Temp=0;
uint8_t ElconCharger_HardwareError=0xff;
uint8_t ElconCharger_TempError=0xff;
uint8_t ElconCharger_InVoltError=0xff;
uint8_t ElconCharger_BatVoltError=0xff;
uint8_t ElconCharger_CommTimeout=0xff;
bool ElconCharger_msgInvalid=false;
const float ElconCharger_CurrentConst=1.6; // 32/20=1.6
// DCDC
const uint32_t  canID_ElconDCDCCtrl = 0x18008FD0;
const uint32_t  canID_ElconDCDCFback = 0x1801D08F;
long ElconDCDC_VoltOutput=0; //0.1V/bit Offset:0
long ElconDCDC_AmpOutput=0; //0.1V/bit Offset:0
long ElconDCDC_Temp=0;
uint8_t ElconDCDC_HVILError=0xff; //0:Lock accomplish,1Non-Lock
uint8_t ElconDCDC_WaterFanSts=0xff; //0:FAN OFF。1: FAN ON
uint8_t ElconDCDC_StopError=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_WaterFanSts2=0xff; //0:off。1:ON
uint8_t ElconDCDC_CommTimeout=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_HardwareError=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_Sts=0xff; //1:working, 0:stopped
uint8_t ElconDCDC_Ready=0xff; //initialisation 0:uncompleted; 1:completed
uint8_t ElconDCDC_OutOverAmp=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_InUnderVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_InOverVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_OutUnderVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_OutOverVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_OverTemp=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_HighTemp=0xff; ////1:High temp, 0:Normal temp
bool ElconDCDC_msgInvalid=false;

// LIM
uint8_t LIM_ACSE_I_Avbl_Grid;                  // E 3B4h Maximum AC current network up to 252A in 1A steps
uint16_t LIM_DCSE_I_Avbl;                    // E 29Eh Available current up to 255A in 0.1A steps
uint8_t LIM_Charger_Type;                       // E 3B4h Current charging type 0=no charging, 1=AC-Typ1, 2=AC-Typ2, 3=DC-Cahdemo, 4=DC-Typ1, 5=AC-CN, 6=AC-Combo1, 7=AC-Combo2, 8= DC-Typ2 9=DC-Combo2, A=DC-GB_T
uint16_t LIM_DCSE_I_Current;                       // E 2B2h Current current from DC EVSE up to 255A in 0.1A steps
uint16_t LIM_DCSE_V_Current;                       // E 2B2h Current voltage from DC EVSE to 600V in 0.1V steps
uint16_t LIM_DCSE_Rst_Tme_Chg;                 // E 2B2h Remaining charging time of DC EVSE in seconds
uint8_t LIM_DCSE_Rst_Tme_Chg_secs=0;
uint8_t LIM_DCSE_Rst_Tme_Chg_mins=0;
uint8_t LIM_DCSE_Rst_Tme_Chg_hrs=0;
uint8_t  ctr_mins_EOC=0;                           // End of Charge timer: Remaining charging time for the display in minutes
uint8_t  ctr_hrs_EOC=0;                           // End of Charge timer: Remaining charging time for the display in minutes

// Common
const long canSpeedChas = 500000; // Chassis CAN speed

void setup() 
  {
    // setup serial for debug
    // Serial.begin(115200);//normal port
    // while (!Serial) 
    // {
    //   ; // wait for Serial port to connect. Needed for native USB port only
    // }
    // setup serial for display
    Serial2.begin(115200);//normal port
    while (!Serial2) 
    {
      ; // wait for Serial port to connect. Needed for native USB port only
    }

    //pin13 output for LED blinking
    pinMode(LED_BUILTIN, OUTPUT);

    strDriveStatNone = String("None|");
    strDriveStatVdcLow = String("VdcLow|");
    strDriveStatVdcHigh = String("VdcHigh|");
    strDriveStatVdcBelowVdcSw = String("Vdc<VdcSw|");
    strDriveStatVdcLim = String("VdcLim|");
    strDriveStatEStop = String("EStop|");
    strDriveStatMProt = String("MProt|");
    strDriveStatPotPressed = String("PotPressed|");
    strDriveStatTmpHs = String("TmpHs|");
    strDriveStatWaitStart = String("WaitStart|");
    strDriveStatBrakeCheck = String("BrakeCheck|");
    strDriveStatInvalid = String("Invalid|");

    // setup CAN bus**************
    // Chassis
    delay(1000); // allow CAN hardware to stabilise
    Can1.begin();
    Can1.setBaudRate(canSpeedChas);
    // Can1.setMaxMB(16);
    // Can1.enableFIFO();
    // Can1.enableFIFOInterrupt();
    // Can1.onReceive(canDataReceived);
    // Can1.mailboxStatus();
    // Elcon
    // delay(1000); // allow CAN hardware to stabilise

    //Zero all CAN message values
    Set_ZeroLIM();
    // Set_ZeroISA();
    Set_ZeroBMS();
    Set_ZeroDrive();
    Set_ZeroElcon();
    Set_ZeroIBooster();
  }

// void Debug (void)
// { 
//     Serial.println();
//     Serial.println();
//     Serial.println();
//     Serial.println(); 
//     Serial.println();
//     Serial.println();
//     Serial.println();
//     Serial.println();

//     Serial.print      (" -----------------------------------------------------------Values from BMS and Drive -----------------------------------------------------");
//     Serial.println(); // Line 0  
//     if (BMS_State == BMS_Boot)         Serial.print ("BMS_Boot| ");  
//     if (BMS_State == BMS_Ready)         Serial.print ("BMS_Ready| ");  
//     if (BMS_State == BMS_Drive)         Serial.print ("BMS_Drive| ");  
//     if (BMS_State == BMS_Charge)         Serial.print ("BMS_Charge| "); 
//     if (BMS_State == BMS_Precharge)        Serial.print ("BMS_Precharge| "); 
//     if (BMS_State == BMS_Error)         Serial.print ("BMS_Error| "); 
//     Serial.println();
//     if (Drive_Dir == Drive_N)         Serial.println ("Drive_N| ");  
//     if (Drive_Dir == Drive_F)         Serial.print ("Drive_F| ");  
//     if (Drive_Dir == Drive_R)         Serial.print ("Drive_R| ");  
//     if (Drive_Dir == Drive_Invalid)   Serial.print ("Drive_Invalid| "); 
//     Serial.println();
//     // Serial.print (Drive_OpMode,BIN);  
//     if (Drive_OpMode == Drive_OPMODE_Off)         Serial.print ("Drive_OPMODE_Off| ");  
//     if (Drive_OpMode == Drive_OPMODE_ACHeat)         Serial.print ("Drive_OPMODE_ACHeat| "); 
//     if (Drive_OpMode == Drive_OPMODE_Boost)         Serial.print ("Drive_OPMODE_Boost| "); 
//     if (Drive_OpMode == Drive_OPMODE_Buck)         Serial.print ("Drive_OPMODE_Buck| "); 
//     if (Drive_OpMode == Drive_OPMODE_ManualRun)         Serial.print ("Drive_OPMODE_ManualRun| "); 
//     if (Drive_OpMode == Drive_OPMODE_Run)         Serial.print ("Drive_OPMODE_Run| "); 
//     if (Drive_OpMode == Drive_OPMODE_Sine)         Serial.print ("Drive_OPMODE_Sine| "); 
//     if (Drive_OpMode == Drive_OPMODE_Invalid)         Serial.print ("Drive_OPMODE_Invalid| "); 
//     Serial.println();
//     // Serial.print (Drive_Status,BIN);  
//     if (Drive_Status == Drive_Stat_None)         Serial.print ("Drive_Stat_None| ");
//     if ((Drive_Status>>0 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcLow| ");  
//     if ((Drive_Status>>1 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcHigh| ");
//     if ((Drive_Status>>2 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcBelowUdcSw| ");  
//     if ((Drive_Status>>3 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcLim| ");
//     if ((Drive_Status>>4 & 0x1) == 1)         Serial.print ("Drive_Stat_EmcyStop| ");  
//     if ((Drive_Status>>5 & 0x1) == 1)         Serial.print ("Drive_Stat_MProt| ");
//     if ((Drive_Status>>6 & 0x1) == 1)         Serial.print ("Drive_Stat_PotPressed| ");  
//     if ((Drive_Status>>7 & 0x1) == 1)         Serial.print ("Drive_Stat_TmpHs| ");
//     if ((Drive_Status>>8 & 0x1) == 1)         Serial.print ("Drive_Stat_WaitStart| ");  
//     if ((Drive_Status>>9 & 0x1) == 1)         Serial.print ("Drive_Stat_BrakeCheck| ");
//     if (Drive_Status == Drive_Stat_Invalid)         Serial.print ("Drive_Stat_Invalid| ");  
//     Serial.println();
//     // Serial.print(Drive_MotorTemp,100.0f,2);
//     Serial.print(Drive_MotorTemp/100);
//     Serial.print      ("C, Drive Motor Temp| ");
//     Serial.println();
//     // Serial.print(Drive_HtSnkTemp,100.0f,2);
//     Serial.print(Drive_HtSnkTemp/100);
//     Serial.print      ("C, Drive HeatSink Temp| ");

//     Serial.println(); // Line 4
//     Serial.print      (" | ");
//     Serial.print(BMS_CellsVoltMax/1000);
//     Serial.print      ("V ->Cell voltage Max.   | ");
//     Serial.print(BMS_CellsVoltMin/1000);
//     Serial.print      ("V ->Cell voltage Min.   | ");
//     Serial.print(BMS_PackVolt/100);
//     Serial.print      ("V ->Pack   | ");
//     Serial.print(BMS_BatAmp/100);
//     Serial.print      ("A ->Pack   | ");
//     // Serial.print(BMS_CellsTempMax-273.15,1.0f,3);
//     Serial.print(BMS_CellsTempMax-273.15);
//     Serial.print      ("*C ->Cell temp Max.   | ");
//     // Serial.print(BMS_CellsTempMin-273.15,1.0f,3);
//     Serial.print(BMS_CellsTempMin-273.15);
//     Serial.print      ("*C ->Cell temp Min.   | ");
//     Serial.print(BMS_AvgTemp);
//     Serial.print      ("*C ->Cell temp Avg.   | ");
//     Serial.print(BMS_CapacityAh);
//     Serial.print      ("Ah ->Bat. capacity   | ");
//     Serial.print(BMS_CapacityWhCharge);
//     Serial.print      ("kWh ->Bat. capacity   | ");
//     // Serial.print(LIM_SM,1.0f,0);
//     // Serial.print      ("     ->Charge SM Step         | ");
//     Serial.print(BMS_SOC/10);
//     Serial.print      ("% ->SOC from BMS           | "); 
//     Debug_Stamp = millis();
// }

void Set_ZeroElcon(void)
  {
    ElconCharger_VoltOutput=0;
    ElconCharger_AmpOutput=0;
    ElconCharger_Temp=0;
    ElconCharger_HardwareError=0xff;
    ElconCharger_TempError=0xff;
    ElconCharger_InVoltError=0xff;
    ElconCharger_BatVoltError=0xff;
    ElconCharger_CommTimeout=0xff;
    ElconCharger_msgInvalid=false;
    ElconDCDC_VoltOutput=0; //0.1V/bit Offset:0
    ElconDCDC_AmpOutput=0; //0.1V/bit Offset:0
    ElconDCDC_HVILError=0xff; //0:Lock accomplish,1Non-Lock
    ElconDCDC_WaterFanSts=0xff; //0:FAN OFF。1: FAN ON
    ElconDCDC_StopError=0xff; //1:Error, 0:No error
    ElconDCDC_WaterFanSts2=0xff; //0:off。1:ON
    ElconDCDC_CommTimeout=0xff; //1:Error, 0:No error
    ElconDCDC_HardwareError=0xff; //1:Error, 0:No error
    ElconDCDC_Sts=0xff; //1:working, 0:stopped
    ElconDCDC_Ready=0xff; //initialisation 0:uncompleted; 1:completed
    ElconDCDC_OutOverAmp=0xff; //1:Error, 0:No error
    ElconDCDC_InUnderVolt=0xff; //1:Error, 0:No error
    ElconDCDC_InOverVolt=0xff; //1:Error, 0:No error
    ElconDCDC_OutUnderVolt=0xff; //1:Error, 0:No error
    ElconDCDC_OutOverVolt=0xff; //1:Error, 0:No error
    ElconDCDC_OverTemp=0xff; //1:Error, 0:No error
    ElconDCDC_HighTemp=0xff; ////1:High temp, 0:Normal temp
    ElconDCDC_msgInvalid=false;
    ElconDCDC_Temp=0;
  }
void Set_ZeroLIM(void)
  {         
    LIM_ACSE_I_Avbl_Grid=0xFF;     
    LIM_DCSE_I_Avbl=0xFFFF;
    LIM_Charger_Type=0xFF;                
    LIM_DCSE_I_Current=0xFFFF;
    LIM_DCSE_V_Current=0xFFFF;
    LIM_DCSE_Rst_Tme_Chg=0xFFFF;
  }
void Set_ZeroIBooster(void)
{
  IBST_driverBrakeApply=IBST_Invalid;
  IBST_iBoosterStatus=IBST_Invalid;
  IBST_internalState=IBST_Invalid;
  IBST_rodLen=0xffff;
}

void Set_ZeroISA(void)
  {
    ISA_BatVolt = 0xFFFF; 
    ISA_MainPOSVolt = 0xFFFF;
    ISA_BatAmp = 0xFFFF;
    ISA_MainNEGVolt=0;
  }

void Set_ZeroBMS (void)
{
  BMS_State = BMS_Error; //unknown state
  BMS_ChargeVoltLim=0;
  BMS_ChargeAmpLim=0;
  BMS_disChargeVoltLim=0;
  BMS_disChargeAmpLim=0;
  BMS_SOC=0;
  BMS_CapacityAh=0;
  BMS_CapacityWhCharge=0;
  BMS_CellsVoltMin = 0;
  BMS_CellsVoltMax = 0;
  BMS_CellsTempMin = 0;
  BMS_CellsTempMax = 0;
  BMS_PackVolt=0;
  BMS_AvgTemp=0;
  BMS_BatAmp=0;
  BMS_CellsBal=0xff;
  BMS_NumbModules = 0xffff;
}
void Set_ZeroDrive(void)
{
  Drive_Dir=Drive_Invalid;
  Drive_OpMode=Drive_OPMODE_Invalid;
  Drive_Status=Drive_Stat_Invalid;
  Drive_MotorTemp=0xFFFF;
  Drive_HtSnkTemp=0xFFFF;
  Drive_RPM=0xFFFF;
}

void Check_BMS (CAN_message_t incoming)
{
  switch (incoming.id)
     {
      case canID_BMSInfo:
        {    
        uint8_t readingBMSInfo=0;
        uint16_t readingBMSCap=0;
        int readingBMSCellsBal=0xFF;

        readingBMSCap=(long)((incoming.buf[1] << 8) | (incoming.buf[0]));
        if (readingBMSCap > 0) BMS_CapacityAh=readingBMSCap;
        else BMS_CapacityAh = 0;

        readingBMSInfo = incoming.buf[4]; 
        if (readingBMSInfo > 0) BMS_State=readingBMSInfo;
        else BMS_State = 0;

        readingBMSCellsBal = incoming.buf[5]; 
        BMS_CellsBal=readingBMSCellsBal;

        break;
        }

      case canID_BMSNumbModules:
        {    
        uint16_t readingNumbModules=0;
        
        readingNumbModules = (uint16_t) ((incoming.buf[1]<<8) | (incoming.buf[0])); 
        BMS_NumbModules = readingNumbModules;

        break;
        }
      // case canID_BMSLimits:
      //   {    
      //   long readingBMS_ChargeVoltLim = 0; 
      //   long readingBMS_ChargeAmpLim = 0; 
      //   long readingBMS_disChargeVoltLim = 0; 
      //   long readingBMS_disChargeAmpLim = 0; 

      //   readingBMS_ChargeVoltLim = (long)((incoming.buf[1] << 8) | (incoming.buf[0]));
      //   readingBMS_ChargeVoltLim = readingBMS_ChargeVoltLim/10; 
      //   if (readingBMS_ChargeVoltLim > 0) BMS_ChargeVoltLim=readingBMS_ChargeVoltLim;
      //   else BMS_ChargeVoltLim = 0;
      //   BMS_CapacityWhCharge=BMS_CapacityAh*BMS_ChargeVoltLim;

      //   readingBMS_ChargeAmpLim = (long)((incoming.buf[3] << 8) | (incoming.buf[2]));
      //   readingBMS_ChargeAmpLim = readingBMS_ChargeAmpLim/10; 
      //   if (readingBMS_ChargeAmpLim > 0) BMS_ChargeAmpLim=readingBMS_ChargeAmpLim;
      //   else BMS_ChargeAmpLim = 0;

      //   readingBMS_disChargeVoltLim = (long)((incoming.buf[7] << 8) | (incoming.buf[6]));
      //   readingBMS_disChargeVoltLim = readingBMS_disChargeVoltLim/10; 
      //   if (readingBMS_disChargeVoltLim > 0) BMS_disChargeVoltLim=readingBMS_disChargeVoltLim;
      //   else BMS_disChargeVoltLim = 0;

      //   readingBMS_disChargeAmpLim = (long)((incoming.buf[5] << 8) | (incoming.buf[4]));
      //   readingBMS_disChargeAmpLim = readingBMS_disChargeAmpLim/10; 
      //   if (readingBMS_disChargeAmpLim > 0) BMS_disChargeAmpLim=readingBMS_disChargeAmpLim;
      //   else BMS_disChargeAmpLim = 0;

      //   break;
      //   }

      case canID_BMSSOC:
        {    
        uint16_t readingBMS_SOC = 0; 

        readingBMS_SOC = (uint16_t)((incoming.buf[5] << 8) | (incoming.buf[4]));
        if (readingBMS_SOC > 0) BMS_SOC=readingBMS_SOC;
        else BMS_SOC = 0;

        break;
        }
      case canID_BMSLowHigh:
        {    
        float readingBMS_MaxCellsVolt = 0; 
        float readingBMS_MinCellsVolt = 0; 
        float readingBMS_MaxCellsTemp = 0; 
        float readingBMS_MinCellsTemp = 0; 

        readingBMS_MinCellsVolt = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
        // readingBMS_MinCellsVolt = readingBMS_MinCellsVolt/1000;
        if (readingBMS_MinCellsVolt > 0) BMS_CellsVoltMin=readingBMS_MinCellsVolt;
        else BMS_CellsVoltMin = 0;

        readingBMS_MaxCellsVolt = (uint16_t)((incoming.buf[3] << 8) | (incoming.buf[2]));
        // readingBMS_MaxCellsVolt = readingBMS_MaxCellsVolt/1000;
        if (readingBMS_MaxCellsVolt > 0) BMS_CellsVoltMax=readingBMS_MaxCellsVolt;
        else BMS_CellsVoltMax = 0;

        readingBMS_MinCellsTemp = (uint16_t) incoming.buf[5] << 8 | incoming.buf[4];
        readingBMS_MinCellsTemp = readingBMS_MinCellsTemp-273.15;
        if (readingBMS_MinCellsTemp > 0) BMS_CellsTempMin=readingBMS_MinCellsTemp;
        else BMS_CellsTempMin = 0;

        readingBMS_MaxCellsTemp = (uint16_t) incoming.buf[7] << 8 | incoming.buf[6];
        readingBMS_MaxCellsTemp = readingBMS_MaxCellsTemp-273.15;
        if (readingBMS_MaxCellsTemp > 0) BMS_CellsTempMax=readingBMS_MaxCellsTemp;
        else BMS_CellsTempMax = 0;

        break;
        }
      case canID_BMSStatus:
        {    
          uint16_t readingBMS_PackVoltage = 0; 
          uint16_t readingBMS_Current = 0; 
          float readingBMS_AvgTemp = 0; 

          readingBMS_PackVoltage = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
          if (readingBMS_PackVoltage > 0) BMS_PackVolt=readingBMS_PackVoltage;
          else BMS_PackVolt = 0;

          readingBMS_Current = (long)((incoming.buf[3] << 8) | (incoming.buf[2]));
          // readingBMS_Current=readingBMS_Current/100; //miliamps to amps in 0.1A steps
          // readingBMS_Current=8192-readingBMS_Current;
          BMS_BatAmp=(long)readingBMS_Current;
          if (readingBMS_Current > 0x7ffffff) BMS_BatAmp=-BMS_BatAmp;

          readingBMS_AvgTemp = (float)((incoming.buf[5] << 8) | (incoming.buf[4]));
          BMS_AvgTemp=readingBMS_AvgTemp;

          break;
        }
     }
  BMS_watchdog=millis();
}
void Check_Drive (CAN_message_t incoming)
{
 switch (incoming.id)
    {
      case canID_Drive:
      {
        int8_t readingDriveDir=Drive_Invalid;
        uint8_t readingDrive_OpMode=Drive_OPMODE_Invalid;
        uint16_t readingDrive_Status=Drive_Stat_Invalid;
        Drive_MotorTemp=0xFFFF;
        Drive_HtSnkTemp=0xFFFF;
        
        readingDrive_OpMode = (uint8_t)(incoming.buf[0]);
        if (readingDrive_OpMode<=6) Drive_OpMode=readingDrive_OpMode;
        else Drive_OpMode=Drive_OPMODE_Invalid;

        readingDrive_Status = (uint16_t)((incoming.buf[2] << 8) | (incoming.buf[1]));
        if ((readingDrive_Status>>10)==0) Drive_Status=readingDrive_Status;
        else Drive_Status=Drive_Stat_Invalid;

        readingDriveDir = (int8_t)(incoming.buf[3]);
        if (readingDriveDir==Drive_N ||readingDriveDir==Drive_F || readingDriveDir==Drive_R) Drive_Dir=readingDriveDir;
        else Drive_Dir=Drive_Invalid;

        Drive_HtSnkTemp=(uint16_t)((incoming.buf[5] << 8) | (incoming.buf[4])); // scale 1/100
        Drive_MotorTemp=(uint16_t)((incoming.buf[7] << 8) | (incoming.buf[6])); // scale 1/100
        break;
      }
      case canID_DriveRPM:
      {
        Drive_RPM=(uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0])); // 
        break;
      }
    }
  Drive_watchdog=millis();
}

void Check_IBooster(CAN_message_t incoming)
{
switch (incoming.id)
    {
      case canID_stsIBooster:
      {
        uint8_t readingIBST_driverBrakeApply=IBST_Invalid;
        uint8_t readingIBST_iBoosterStatus=IBST_Invalid;
        uint8_t readingIBST_internalState=IBST_Invalid;
        uint16_t readingIBST_RodLen=0xffff;
        
        readingIBST_driverBrakeApply = (uint8_t)(incoming.buf[2] & 0x03);
        if (readingIBST_driverBrakeApply==BRAKES_NOT_APPLIED || readingIBST_driverBrakeApply==DRIVER_APPLYING_BRAKES || readingIBST_driverBrakeApply==FAULT || readingIBST_driverBrakeApply==NOT_INIT_OR_OFF) 
          {
            IBST_driverBrakeApply=readingIBST_driverBrakeApply;
          }
        else 
          {
            IBST_driverBrakeApply=IBST_Invalid;
          }

        readingIBST_iBoosterStatus = (uint8_t)((incoming.buf[1] >> 4) & 0x07);
        if (readingIBST_iBoosterStatus==IBOOSTER_ACTIVE_GOOD_CHECK || readingIBST_iBoosterStatus==IBOOSTER_ACTUATION || readingIBST_iBoosterStatus==IBOOSTER_DIAGNOSTIC || readingIBST_iBoosterStatus==IBOOSTER_FAILURE ||readingIBST_iBoosterStatus==IBOOSTER_INIT || readingIBST_iBoosterStatus==IBOOSTER_OFF || readingIBST_iBoosterStatus==IBOOSTER_READY) 
          {
            IBST_iBoosterStatus=readingIBST_iBoosterStatus;
          }
        else 
          {
            IBST_iBoosterStatus=IBST_Invalid;
          }
        readingIBST_internalState = (uint8_t)((incoming.buf[2] >> 2) & 0x07);
        if (readingIBST_internalState==DIAGNOSTIC || readingIBST_internalState==EXTERNAL_BRAKE_REQUEST || readingIBST_internalState==LOCAL_BRAKE_REQUEST || readingIBST_internalState==NO_MODE_ACTIVE || readingIBST_internalState==POST_DRIVE_CHECK || readingIBST_internalState==PRE_DRIVE_CHECK || readingIBST_internalState==TRANSITION_TO_IDLE) 
          {
            IBST_internalState=readingIBST_internalState;
          }
        else 
          {
            IBST_internalState=IBST_Invalid;
          }
        readingIBST_RodLen = (uint16_t)((((incoming.buf[4]) & 0x01)<<11) | ((incoming.buf[3])<<3) | ((incoming.buf[2] >> 5) & 0x07));
        IBST_rodLen = readingIBST_RodLen;
        break;
      }
    }
  IBooster_watchdog=millis();
}
// Decoding checks
void Check_ISA(CAN_message_t incoming)
  {
  switch (incoming.id)
      {
        case canID_ISAmilliAmps:
        {
          long readingISAamps=0;
          // bigE
          readingISAamps = (long)((incoming.buf[2] << 24) | (incoming.buf[3] << 16) | (incoming.buf[4] << 8) | (incoming.buf[5]));
          // littleE
          // readingISAamps = (long)((incoming.buf[5] << 24) | (incoming.buf[4] << 16) | (incoming.buf[3] << 8) | (incoming.buf[2]));
          readingISAamps=readingISAamps/100; //miliamps to amps in 0.1A steps
          readingISAamps=8192-readingISAamps;
          if (readingISAamps > 0) ISA_BatAmp=(uint16_t)readingISAamps;
          else ISA_BatAmp = 0;
          // ISA_BatAmp=(uint16_t)readingISAamps;
          break;
        }
      
      }
    ISA_watchdog=millis();
  }
void Check_Elcon(CAN_message_t incoming)
  {
  switch (incoming.id)
      {
        case canID_ElconChargerFback:
          {    
            long readingElconChargerVolt=0;
            long readingElconChargerAmp=0;
            long readingElconChargerTemp=0;
            uint8_t readingElconChargerStatus=0;

            readingElconChargerVolt = (long)((incoming.buf[0] << 8) | (incoming.buf[1]));
            // readingElconChargerVolt = readingElconChargerVolt/10; 
            if (readingElconChargerVolt > 0) ElconCharger_VoltOutput=readingElconChargerVolt;
            else ElconCharger_VoltOutput = 0;

            readingElconChargerAmp = (long)((incoming.buf[2] << 8) | (incoming.buf[3]));
            // readingElconChargerAmp = readingElconChargerAmp/10; 
            if (readingElconChargerAmp > 0) ElconCharger_AmpOutput=readingElconChargerAmp;
            else ElconCharger_AmpOutput = 0;

            readingElconChargerStatus = (uint8_t)(incoming.buf[4]);
            if (readingElconChargerStatus >= 0) 
              {
                ElconCharger_msgInvalid=false;
                ElconCharger_HardwareError=(readingElconChargerStatus>>0)&0x01;
                ElconCharger_TempError=(readingElconChargerStatus>>1)&0x01;
                ElconCharger_InVoltError=(readingElconChargerStatus>>2)&0x01;
                ElconCharger_BatVoltError=(readingElconChargerStatus>>3)&0x01;
                ElconCharger_CommTimeout=(readingElconChargerStatus>>4)&0x01;
              }
            else ElconCharger_msgInvalid = true;

            readingElconChargerTemp = (long)(incoming.buf[5]);
            // readingElconChargerTemp = readingElconChargerAmp/10; 
            ElconCharger_Temp=readingElconChargerTemp;
            break;
          }
        case canID_ElconDCDCFback:
          {    
            long readingElconDCDCVolt=0;
            long readingElconDCDCAmp=0;
            long readingElconDCDCTemp=0;
            uint16_t readingElconDCDCStatus=0;

            readingElconDCDCVolt = (long)((incoming.buf[0] << 8) | (incoming.buf[1]));
            // readingElconDCDCVolt = readingElconDCDCVolt/10; 
            if (readingElconDCDCVolt > 0) ElconDCDC_VoltOutput=readingElconDCDCVolt;
            else ElconDCDC_VoltOutput = 0;

            readingElconDCDCAmp = (long)((incoming.buf[2] << 8) | (incoming.buf[3]));
            // readingElconDCDCAmp = readingElconDCDCAmp/10; 
            if (readingElconDCDCAmp > 0) ElconDCDC_AmpOutput=readingElconDCDCAmp;
            else ElconDCDC_AmpOutput = 0;

            readingElconDCDCStatus = (uint16_t)((incoming.buf[4] << 8) | (incoming.buf[5]));
            if (readingElconDCDCStatus >= 0) 
            {
              ElconDCDC_msgInvalid=false;
              ElconDCDC_HVILError=(readingElconDCDCStatus>>15)&0x01; //0:Lock accomplish,1Non-Lock
              ElconDCDC_WaterFanSts=(readingElconDCDCStatus>>14)&0x01; //0:FAN OFF。1: FAN ON
              ElconDCDC_StopError=(readingElconDCDCStatus>>13)&0x01; //1:Error, 0:No error
              ElconDCDC_WaterFanSts2=(readingElconDCDCStatus>>12)&0x01; //0:off。1:ON
              ElconDCDC_CommTimeout=(readingElconDCDCStatus>>11)&0x01; //1:Error, 0:No error
              ElconDCDC_HardwareError=(readingElconDCDCStatus>>10)&0x01; //1:Error, 0:No error
              ElconDCDC_Sts=(readingElconDCDCStatus>>9)&0x1;//1:working, 0:stopped
              ElconDCDC_Ready=(readingElconDCDCStatus>>8)&0x1; //initialisation 0:uncompleted; 1:completed
              ElconDCDC_OutOverAmp=(readingElconDCDCStatus>>6)&0x1; //1:Error, 0:No error
              ElconDCDC_OutUnderVolt=(readingElconDCDCStatus>>5)&0x1; //1:Error, 0:No error
              ElconDCDC_OutOverVolt=(readingElconDCDCStatus>>4)&0x1; //1:Error, 0:No error
              ElconDCDC_InUnderVolt=(readingElconDCDCStatus>>3)&0x1; //1:Error, 0:No error
              ElconDCDC_InOverVolt=(readingElconDCDCStatus>>2)&0x1; //1:Error, 0:No error
              ElconDCDC_OverTemp=(readingElconDCDCStatus>>1)&0x1; //1:Error, 0:No error
              ElconDCDC_HighTemp=(readingElconDCDCStatus)&0x1; ////1:High temp, 0:Normal temp
            }
            else ElconDCDC_msgInvalid = true;

            readingElconDCDCTemp = (long)(incoming.buf[7]);
            readingElconDCDCTemp=readingElconDCDCTemp-32;
            readingElconDCDCTemp=readingElconDCDCTemp*5;
            readingElconDCDCTemp=readingElconDCDCTemp/9;
            ElconDCDC_Temp=readingElconDCDCTemp;
            break;
          }
      }
    Elcon_watchdog=millis();
  }
void Check_LIM(CAN_message_t incoming)
  {
    //Serial.println(incoming.id);
    switch (incoming.id)
      {
        case 0x3B4:    
          LIM_ACSE_I_Avbl_Grid = incoming.buf[0];    
          LIM_Charger_Type = incoming.buf[6];      
        break;
        case 0x2B2:
          {    
            uint16_t LIM_DCSE_Rst_Tme_Chg_temp=0;
            LIM_DCSE_Rst_Tme_Chg = (uint16_t)((incoming.buf[7] << 8) | (incoming.buf[6]));
            LIM_DCSE_Rst_Tme_Chg_temp=LIM_DCSE_Rst_Tme_Chg;
            //seconds to hh:mm:ss
            LIM_DCSE_Rst_Tme_Chg_secs=LIM_DCSE_Rst_Tme_Chg_temp % 60;
            LIM_DCSE_Rst_Tme_Chg_temp = (LIM_DCSE_Rst_Tme_Chg_temp - LIM_DCSE_Rst_Tme_Chg_secs)/60;
            LIM_DCSE_Rst_Tme_Chg_mins = LIM_DCSE_Rst_Tme_Chg_temp % 60;
            LIM_DCSE_Rst_Tme_Chg_temp = (LIM_DCSE_Rst_Tme_Chg_temp - LIM_DCSE_Rst_Tme_Chg_mins)/60;
            LIM_DCSE_Rst_Tme_Chg_hrs=LIM_DCSE_Rst_Tme_Chg_temp;
            LIM_DCSE_I_Current = (uint16_t)((incoming.buf[3] << 8) | (incoming.buf[2]));
            LIM_DCSE_V_Current = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
          }
        case 0x29E:    
          LIM_DCSE_I_Avbl = (uint16_t)((incoming.buf[4] << 8) | (incoming.buf[3]));
        break;
      }
    LIM_watchdog=millis();
  }

void Chg_Timers(void)
  {
    float temp_ctr_mins_EOC=0.0;

    //timer left to full charge in minutes
    temp_ctr_mins_EOC=(float) (BMS_SOC/1000.0); //BMS_SOC is in 0.1% scale, hence /1000% not /100%
    temp_ctr_mins_EOC=(float) (1-temp_ctr_mins_EOC);
    temp_ctr_mins_EOC=(float) (BMS_CapacityAh*temp_ctr_mins_EOC);
    temp_ctr_mins_EOC=(float) (temp_ctr_mins_EOC*60.0); //hrs to mins *60
    temp_ctr_mins_EOC=(float) (temp_ctr_mins_EOC*10.0/ElconCharger_AmpOutput); //Elcon charger amps are in 0.1A scale, hence *10
    if (temp_ctr_mins_EOC>60) 
      {
        ctr_hrs_EOC=(uint8_t) (temp_ctr_mins_EOC/60.0);       //ex. 277.55minutes = 4hrs; 
        temp_ctr_mins_EOC=(float)((temp_ctr_mins_EOC*1.66)-ctr_hrs_EOC*100.0);
        temp_ctr_mins_EOC=(float)(temp_ctr_mins_EOC/1.66);
        ctr_mins_EOC=(uint8_t) temp_ctr_mins_EOC;
      }
    else 
      {
        ctr_mins_EOC= (uint8_t) temp_ctr_mins_EOC;
        ctr_hrs_EOC=0;
      }
  }
void dashupdate()
  {
    //additional stat font=small
    if((BMS_State==BMS_Ready || BMS_State==BMS_Charge || BMS_State==BMS_Drive) && Drive_OpMode!=Drive_OPMODE_Run) Serial2.print("stat.font=0");
    else Serial2.print("stat.font=1");
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);

    //error or boot colour = flashing red
    if ((BMS_State==BMS_Error || BMS_State==BMS_Boot) && flash)
      {
        Serial2.print("stat.pco=63488"); //text colour = red
        Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial2.write(0xff);
        Serial2.write(0xff);
      }
    else
      {
        Serial2.print("stat.pco=34815"); // text colour neutral
        Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial2.write(0xff);
        Serial2.write(0xff);
      }

    Serial2.print("stat.txt=");
    Serial2.write(0x22);
    switch (BMS_State)
      {
        case (BMS_Boot):
          {
            Serial2.print("Boot");
            break;
          }
        case (BMS_Ready):
          {
            if (Drive_OpMode==Drive_OPMODE_Run)
              {
                Serial2.print("Ready");
              }
            else
              {
                if (Drive_Status<0xffff)
                  {
                  if ((Drive_Status>>8 & 0x1) == 1)         
                    {
                      Serial2.print ("press"); 
                      Serial2.println(); 
                      Serial2.print ("start");  
                    }
                  else if ((Drive_Status>>9 & 0x1) == 1) 
                    {        
                      Serial2.print ("test");
                      Serial2.println(); 
                      Serial2.print ("brake");
                    }
                  else 
                    {
                      Serial2.print ("check");
                      Serial2.println(); 
                      Serial2.print ("drive");
                    }
                  }
                else
                  {
                      Serial2.print ("check");
                      Serial2.println(); 
                      Serial2.print ("drive");
                  }
              }
            break;
          }

        // case (Precharge):
        //   Serial2.print("Precharge");
        //   break;

        case (BMS_Drive):
          {
            if (Drive_OpMode==Drive_OPMODE_Run)
              {
                Serial2.print("Drive");
              }
            else
              {
                if (Drive_Status<0xffff)
                  {
                  if ((Drive_Status>>8 & 0x1) == 1)         
                    {
                      Serial2.print ("press"); 
                      Serial2.println(); 
                      Serial2.print ("start");  
                    }
                  else if ((Drive_Status>>9 & 0x1) == 1) 
                    {        
                      Serial2.print ("test");
                      Serial2.println(); 
                      Serial2.print ("brake");
                    }
                  else 
                    {
                      Serial2.print ("check");
                      Serial2.println(); 
                      Serial2.print ("drive");
                    }
                  }
                else
                  {
                      Serial2.print ("check");
                      Serial2.println(); 
                      Serial2.print ("drive");
                  }
              }
            break;
          }
        case (BMS_Charge):
          {
            if (Drive_OpMode==Drive_OPMODE_Run)
              {
                Serial2.print("Charge");
              }
            else
              {
                if (Drive_Status<0xffff)
                  {
                  if ((Drive_Status>>8 & 0x1) == 1)         
                    {
                      Serial2.print ("press"); 
                      Serial2.println(); 
                      Serial2.print ("start");  
                    }
                  else if ((Drive_Status>>9 & 0x1) == 1) 
                    {        
                      Serial2.print ("test");
                      Serial2.println(); 
                      Serial2.print ("brake");
                    }
                  else 
                    {
                      Serial2.print ("check");
                      Serial2.println(); 
                      Serial2.print ("drive");
                    }
                  }
                else
                  {
                      Serial2.print ("check");
                      Serial2.println(); 
                      Serial2.print ("drive");
                  }
              }
            break;
          }
        case (BMS_Error):
          Serial2.print("Error");
          break;
      }
    Serial2.write(0x22);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);

    Serial2.print("bms_stat.txt=");
    Serial2.write(0x22);
    switch (BMS_State)
      {
        case (BMS_Boot):
          Serial2.print("Boot");
          break;

        case (BMS_Ready):
          Serial2.print("Ready");
          break;

        // case (Precharge):
        //   Serial2.print("Precharge");
        //   break;

        case (BMS_Drive):
          Serial2.print("Drive");
          break;

        case (BMS_Charge):
          Serial2.print("Charge");
          break;

        case (BMS_Error):
          Serial2.print("Error");
          break;
      }
    Serial2.write(0x22);
    Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial2.write(0xff);
    Serial2.write(0xff);

    //DNR font sizes
    if (BMS_State!=BMS_Drive) 
      {
        //make DNR font small
        Serial2.print("drive_n.font=1");
        // Serial2.write(0x22);
        Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial2.write(0xff);
        Serial2.write(0xff);
        Serial2.print("drive_r.font=1");
        // Serial2.write(0x22);
        Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial2.write(0xff);
        Serial2.write(0xff);
        Serial2.print("drive_f.font=1");
        // Serial2.write(0x22);
        Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial2.write(0xff);
        Serial2.write(0xff);
      }
    else 
      {
        switch(Drive_Dir)
          {
            case Drive_N:
              {
                //make N large font everything else small
                Serial2.print("drive_n.font=2");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                Serial2.print("drive_r.font=1");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                Serial2.print("drive_f.font=1");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                break;
              }
            case Drive_R:
              {
                //make R large font everything else small
                Serial2.print("drive_n.font=1");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                Serial2.print("drive_r.font=2");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                Serial2.print("drive_f.font=1");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                break;
              }
            case Drive_F:
              {
                //make F large font everything else small
                Serial2.print("drive_n.font=1");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                Serial2.print("drive_r.font=1");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                Serial2.print("drive_f.font=2");
                Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial2.write(0xff);
                Serial2.write(0xff);
                break;
              }
          }
      }
    
  // DNR font colours
    switch(Drive_Dir)
      {
        case Drive_N:
          {
            //make N blue everything else white
            Serial2.print("drive_n.pco=34815");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_r.pco=31695");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_f.pco=31695");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_dir.txt=");
            Serial2.write(0x22);
            Serial2.print("N");
            Serial2.write(0x22);
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            break;
          }
        case Drive_R:
          {
            //make R blue everything else white
            Serial2.print("drive_n.pco=31695");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_r.pco=34815");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_f.pco=31695");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_dir.txt=");
            Serial2.write(0x22);
            Serial2.print("R");
            Serial2.write(0x22);
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            break;
          }
        case Drive_F:
          {
            //make F blue everything else white
            Serial2.print("drive_n.pco=31695");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_r.pco=31695");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_f.pco=34815");
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            Serial2.print("drive_dir.txt=");
            Serial2.write(0x22);
            Serial2.print("F");
            Serial2.write(0x22);
            Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial2.write(0xff);
            Serial2.write(0xff);
            break;
          }
      }
      Serial2.print("soc.val=");
      // Serial2.write(0x22);
      Serial2.print(BMS_SOC/10);
      // Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      //colourcode SOC
      if(BMS_SOC<=100) Serial2.print("soc.pco=63488"); //text colour = red
      else if (BMS_SOC>100 && BMS_SOC<=300) Serial2.print("soc.pco=65504"); //text colour = yellow
      else Serial2.print("soc.pco=2016"); //text colour = green
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("soc1.val=");
      // Serial2.write(0x22);
      Serial2.print(BMS_SOC/10);
      // Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("current.txt=");
      Serial2.write(0x22);
      // if (BMS_BatAmp<=0x7fff) Serial2.print((float)((BMS_BatAmp)/10.0),2);
      // else Serial2.print((float)((BMS_BatAmp)/(-10.0)),2);
      Serial2.print((float) ((8192-ISA_BatAmp)/10.0f),2);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("b_temp.txt=");
      Serial2.write(0x22);
      Serial2.print((long) (BMS_AvgTemp/10));
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      //colourcode bat temp
      if(BMS_AvgTemp<=100) Serial2.print("b_temp.pco=34815"); //text colour = blue=cold
      else if (BMS_AvgTemp>100 && BMS_AvgTemp<=450) Serial2.print("b_temp.pco=2016"); //text colour = green
      else Serial2.print("b_temp.pco=63488"); //text colour = red
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("m_temp.txt=");
      Serial2.write(0x22);
      Serial2.print(Drive_MotorTemp/100);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      //colourcode motor temp
      if(Drive_MotorTemp<=6000) Serial2.print("m_temp.pco=2016"); //text colour = green
      else if (Drive_MotorTemp>6000 && Drive_MotorTemp<=7000) Serial2.print("m_temp.pco=65504"); //text colour = yellow
      else Serial2.print("m_temp.pco=63488"); //text colour = red
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("hs_temp.txt=");
      Serial2.write(0x22);
      Serial2.print(Drive_HtSnkTemp/100);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      //colourcode HS temp
      if(Drive_HtSnkTemp<=10000) Serial2.print("hs_temp.pco=2016"); //text colour = green
      else if (Drive_HtSnkTemp>10000 && Drive_HtSnkTemp<=12000) Serial2.print("hs_temp.pco=65504"); //text colour = yellow
      else Serial2.print("hs_temp.pco=63488"); //text colour = red
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("b_temp_low.txt=");
      Serial2.write(0x22);
      Serial2.print(BMS_CellsTempMin,0);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      //colourcode bat temp low
      if(BMS_CellsTempMin<=10) Serial2.print("b_temp_low.pco=34815"); //text colour = blue=cold
      else if (BMS_CellsTempMin>10 && BMS_CellsTempMin<=45) Serial2.print("b_temp_low.pco=2016"); //text colour = green
      else Serial2.print("b_temp_low.pco=63488"); //text colour = red
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("b_temp_high.txt=");
      Serial2.write(0x22);
      Serial2.print(BMS_CellsTempMax,0);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      //colourcode bat temp high
      if(BMS_CellsTempMax<=10) Serial2.print("b_temp_high.pco=34815"); //text colour = blue=cold
      else if (BMS_CellsTempMax>10 && BMS_CellsTempMax<=45) Serial2.print("b_temp_high.pco=2016"); //text colour = green
      else Serial2.print("b_temp_high.pco=63488"); //text colour = red
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("volt.txt=");
      Serial2.write(0x22);
      Serial2.print(BMS_PackVolt/100);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("v_lowcell.txt=");
      Serial2.write(0x22);
      Serial2.print((float)(BMS_CellsVoltMin/1000.0),2);// * 1000);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("v_highcell.txt=");
      Serial2.write(0x22);
      Serial2.print((float)(BMS_CellsVoltMax/1000.0),2);// * 1000);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      // // Serial2.print("firm.txt=");
      // // Serial2.print(firmver);
      // // Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      // // Serial2.write(0xff);
      // // Serial2.write(0xff);
      Serial2.print("volt_delta.txt=");
      Serial2.write(0x22);
      Serial2.print((float) ((BMS_CellsVoltMax - BMS_CellsVoltMin) / 1.0),0);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("cellbal.txt=");
      Serial2.write(0x22);
      Serial2.print(BMS_CellsBal);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("rpm.txt=");
      Serial2.write(0x22);
      Serial2.print(Drive_RPM);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("mph.txt=");
      Serial2.write(0x22);
      Serial2.print((uint16_t) (Drive_RPM/120.24));
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print("kmph.txt=");
      Serial2.write(0x22);
      Serial2.print((uint16_t) (Drive_RPM/74.68));
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("m_opmode.txt=");
      Serial2.write(0x22);
      switch (Drive_OpMode)
      {
        case Drive_OPMODE_Off:
          {
          Serial2.print ("Off");
          break;
          } 
        case Drive_OPMODE_ACHeat:
          {
          Serial2.print ("ACHeat");
          break;
          } 
        case Drive_OPMODE_Boost:
          {
          Serial2.print ("Boost");
          break;
          } 
        case Drive_OPMODE_Buck:
        {
          Serial2.print ("Buck");
          break;
        } 
        case Drive_OPMODE_ManualRun:
          {
          Serial2.print ("ManualRun");
          break;
          } 
        case Drive_OPMODE_Run:
          {
          Serial2.print ("Run");
          break;
          } 
        case Drive_OPMODE_Sine:
          {
          Serial2.print ("Sine");
          break;
          } 
        case Drive_OPMODE_Invalid:
          {
          Serial2.print ("Invalid");
          break;
          } 
        default:
          {
          Serial2.print ("None");
          break;
          } 
        }
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      // strTemp = String(""); //clear message
      // if (Drive_Status == Drive_Stat_None)      strTemp+=strDriveStatNone;
      // if ((Drive_Status>>0 & 0x1) == 1)         strTemp+=strDriveStatVdcLow;  
      // if ((Drive_Status>>1 & 0x1) == 1)         strTemp+=strDriveStatVdcHigh;
      // if ((Drive_Status>>2 & 0x1) == 1)         strTemp+=strDriveStatVdcBelowVdcSw;  
      // if ((Drive_Status>>3 & 0x1) == 1)         strTemp+=strDriveStatVdcLim;
      // if ((Drive_Status>>4 & 0x1) == 1)         strTemp+=strDriveStatEStop;  
      // if ((Drive_Status>>5 & 0x1) == 1)         strTemp+=strDriveStatMProt;
      // if ((Drive_Status>>6 & 0x1) == 1)         strTemp+=strDriveStatPotPressed;  
      // if ((Drive_Status>>7 & 0x1) == 1)         strTemp+=strDriveStatTmpHs;
      // if ((Drive_Status>>8 & 0x1) == 1)         strTemp+=strDriveStatWaitStart;  
      // if ((Drive_Status>>9 & 0x1) == 1)         strTemp+=strDriveStatBrakeCheck;
      // if (Drive_Status == Drive_Stat_Invalid)         strTemp+=strDriveStatInvalid;  
      // Serial2.print ("m_stat.txt=");
      // Serial2.write(0x22);
      // Serial2.print(strTemp);
      // Serial2.write(0x22);
      // Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      // Serial2.write(0xff);
      // Serial2.write(0xff);
      Serial2.print ("m_stat.txt=");
      Serial2.write(0x22);
      if (Drive_Status == Drive_Stat_None)      Serial2.print (strDriveStatNone);
      if ((Drive_Status>>0 & 0x1) == 1)         Serial2.print (strDriveStatVdcLow);  
      if ((Drive_Status>>1 & 0x1) == 1)         Serial2.print (strDriveStatVdcHigh);
      if ((Drive_Status>>2 & 0x1) == 1)         Serial2.print (strDriveStatVdcBelowVdcSw);  
      if ((Drive_Status>>3 & 0x1) == 1)         Serial2.print (strDriveStatVdcLim);
      if ((Drive_Status>>4 & 0x1) == 1)         Serial2.print (strDriveStatEStop);  
      if ((Drive_Status>>5 & 0x1) == 1)         Serial2.print (strDriveStatMProt);
      if ((Drive_Status>>6 & 0x1) == 1)         Serial2.print (strDriveStatPotPressed);  
      if ((Drive_Status>>7 & 0x1) == 1)         Serial2.print (strDriveStatTmpHs);
      if ((Drive_Status>>8 & 0x1) == 1)         Serial2.print (strDriveStatWaitStart);  
      if ((Drive_Status>>9 & 0x1) == 1)         Serial2.print (strDriveStatBrakeCheck);
      if (Drive_Status == Drive_Stat_Invalid)         Serial2.print (strDriveStatInvalid);  
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("ib_brk_app.txt=");
      Serial2.write(0x22);
      if (IBST_driverBrakeApply==BRAKES_NOT_APPLIED)      Serial2.print ("BRAKES_NOT_APPLIED");
      if (IBST_driverBrakeApply==DRIVER_APPLYING_BRAKES)  Serial2.print ("DRIVER_APPLYING_BRAKES");
      if (IBST_driverBrakeApply==FAULT)                   Serial2.print ("FAULT");
      if (IBST_driverBrakeApply==NOT_INIT_OR_OFF)         Serial2.print ("NOT_INIT_OR_OFF");
      if (IBST_driverBrakeApply==IBST_Invalid)            Serial2.print ("IBST_Invalid");
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("ib_stat.txt=");
      Serial2.write(0x22);
      if (IBST_iBoosterStatus==IBOOSTER_ACTIVE_GOOD_CHECK)         Serial2.print ("IBOOSTER_ACTIVE_GOOD_CHECK");
      if (IBST_iBoosterStatus==IBOOSTER_ACTUATION)                 Serial2.print ("IBOOSTER_ACTUATION");
      if (IBST_iBoosterStatus==IBOOSTER_DIAGNOSTIC)                Serial2.print ("IBOOSTER_DIAGNOSTIC");
      if (IBST_iBoosterStatus==IBOOSTER_FAILURE)                   Serial2.print ("IBOOSTER_FAILURE");
      if (IBST_iBoosterStatus==IBOOSTER_INIT)                      Serial2.print ("IBOOSTER_INIT");
      if (IBST_iBoosterStatus==IBOOSTER_OFF)                       Serial2.print ("IBOOSTER_OFF");
      if (IBST_iBoosterStatus==IBOOSTER_READY)                     Serial2.print ("IBOOSTER_READY");
      if (IBST_iBoosterStatus==IBST_Invalid)                       Serial2.print ("IBST_Invalid");
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("ib_int_stat.txt=");
      Serial2.write(0x22);
      if (IBST_internalState==DIAGNOSTIC)                     Serial2.print ("DIAGNOSTIC");
      if (IBST_internalState==EXTERNAL_BRAKE_REQUEST)         Serial2.print ("EXTERNAL_BRAKE_REQUEST");
      if (IBST_internalState==LOCAL_BRAKE_REQUEST)            Serial2.print ("LOCAL_BRAKE_REQUEST");
      if (IBST_internalState==NO_MODE_ACTIVE)                 Serial2.print ("NO_MODE_ACTIVE");
      if (IBST_internalState==POST_DRIVE_CHECK)               Serial2.print ("POST_DRIVE_CHECK");
      if (IBST_internalState==PRE_DRIVE_CHECK)                Serial2.print ("PRE_DRIVE_CHECK");
      if (IBST_internalState==TRANSITION_TO_IDLE)             Serial2.print ("TRANSITION_TO_IDLE");
      if (IBST_internalState==IBST_Invalid)                   Serial2.print ("IBST_Invalid");
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("ib_rod_len.txt=");
      Serial2.write(0x22);
      if(IBST_rodLen<0xffff) Serial2.print((double) ((IBST_rodLen*0.015625d)-5.0d),4);
      else Serial2.print("NA ");
      Serial2.print ("mm");
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("numbModules.txt=");
      Serial2.write(0x22);
      Serial2.print(BMS_NumbModules);
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);
      Serial2.print ("dcdc_error.txt=");
      Serial2.write(0x22);
      if (ElconDCDC_msgInvalid)
        {
          Serial2.print ("Invalid CAN message");
        }
      else
        {
          if (ElconDCDC_HVILError==1)                     Serial2.print ("HVILError;");
          if (ElconDCDC_StopError==1)                     Serial2.print ("StopError;");
          if (ElconDCDC_CommTimeout==1)                     Serial2.print ("CommTimeout;");
          if (ElconDCDC_HardwareError==1)                     Serial2.print ("HardwareError;");      
          if (ElconDCDC_Ready==0)                     Serial2.print ("Not Ready;");
          if (ElconDCDC_OutOverAmp==1)                     Serial2.print ("OutOverAmp;");
          if (ElconDCDC_InUnderVolt==1)                     Serial2.print ("InUnderVolt;");
          if (ElconDCDC_InOverVolt==1)                     Serial2.print ("InOverVolt;");
          if (ElconDCDC_OutUnderVolt==1)                     Serial2.print ("OutUnderVolt;");
          if (ElconDCDC_OutOverVolt==1)                     Serial2.print ("OutOverVolt;");
          if (ElconDCDC_OverTemp==1)                     Serial2.print ("OverTemp;");
          if (ElconDCDC_HighTemp==1)                     Serial2.print ("HighTemp;");
          if (ElconDCDC_HVILError==0 && ElconDCDC_StopError==0 && ElconDCDC_CommTimeout==0 && ElconDCDC_HardwareError==0 && ElconDCDC_Ready==1 && ElconDCDC_OutOverAmp==0 && ElconDCDC_InOverVolt==0 && ElconDCDC_OutUnderVolt==0 && ElconDCDC_OutOverVolt==0 && ElconDCDC_OverTemp==0 && ElconDCDC_HighTemp==0) 
            {Serial2.print ("Ok");}
        }
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);

      Serial2.print ("dcdc_stat.txt=");
      Serial2.write(0x22);
      if (ElconDCDC_msgInvalid)
        {
          Serial2.print ("Invalid CAN message");
        }
      else
        {
          if (ElconDCDC_Sts==1)       Serial2.print ("ON ");
          else if (ElconDCDC_Sts==0)  Serial2.print ("OFF ");
          Serial2.print ((float) (ElconDCDC_VoltOutput/10.0),2);
          Serial2.print ("V ");
          Serial2.print ((float) (ElconDCDC_AmpOutput/10.0),2);
          Serial2.print ("A ");
          Serial2.print ((float) (ElconDCDC_Temp/1.0),2);
          Serial2.print ("C ");
        }
      Serial2.write(0x22);
      Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial2.write(0xff);
      Serial2.write(0xff);

      if (LIM_Charger_Type==0x04 || LIM_Charger_Type==0x08 || LIM_Charger_Type==0x09) //DC Charging info
        {
          Serial2.print ("charger_error.txt=");
          Serial2.write(0x22);
          Serial2.print ("TBD");
          // if (ElconCharger_msgInvalid)
          //   {
          //     Serial2.print ("Invalid CAN message");
          //   }
          // else
          //   {
          //     if (ElconCharger_CommTimeout==1)                     Serial2.print ("CommTimeout;");
          //     if (ElconCharger_HardwareError==1)                     Serial2.print ("HardwareError;");      
          //     if (ElconCharger_TempError==1)                     Serial2.print ("OverTemp;");
          //     if (ElconCharger_BatVoltError==1)                     Serial2.print ("BatteryVoltError;");
          //     if (ElconCharger_InVoltError==1)                     Serial2.print ("InputVoltError;");
          //     if (ElconCharger_CommTimeout==0 && ElconCharger_HardwareError==0 && ElconCharger_TempError==0 && ElconCharger_BatVoltError==0 && ElconCharger_InVoltError==0) 
          //       {Serial2.print ("Ok");}
          //   }
          Serial2.write(0x22);
          Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
          Serial2.write(0xff);
          Serial2.write(0xff);

          Serial2.print ("charger_stat.txt=");
          Serial2.write(0x22);
          if (LIM_Charger_Type == 4) Serial2.print ("CCS1: ");
          else if (LIM_Charger_Type == 8) Serial2.print ("CCS1C: ");
          else if (LIM_Charger_Type == 9) Serial2.print ("CCS2C: ");
          else Serial2.print ("DC???: ");
          Serial2.print ((float) (LIM_DCSE_V_Current/10.0),1);
          Serial2.print ("V ");
          Serial2.print ((float) (LIM_DCSE_I_Current/10.0),1);
          Serial2.print (" / ");
          if (BMS_State==BMS_Charge && LIM_DCSE_I_Avbl < 2550) Serial2.print ((float) (LIM_DCSE_I_Avbl/10.0),1);
          else Serial2.print ("0");
          Serial2.print ("A");
          Serial2.write(0x22);
          Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
          Serial2.write(0xff);
          Serial2.write(0xff);

          Serial2.print ("charger_time.txt=");
          Serial2.write(0x22);
          if (BMS_State==BMS_Charge)
            {

              if (LIM_DCSE_Rst_Tme_Chg_hrs<=0 && LIM_DCSE_Rst_Tme_Chg_mins<=0)
                {
                  Serial2.print ("less than a minute to complete");  
                }
              else
                {
                  if(LIM_DCSE_Rst_Tme_Chg_hrs>0)
                    {
                      Serial2.print (LIM_DCSE_Rst_Tme_Chg_hrs);
                      Serial2.print ("hrs ");
                    }
                  Serial2.print (LIM_DCSE_Rst_Tme_Chg_mins);
                  Serial2.print ("mins to complete");                 
                }
            }
          else
            {
              Serial2.print ("");
            }
          Serial2.write(0x22);
          Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
          Serial2.write(0xff);
          Serial2.write(0xff);
        }
      else //AC Charging info
        {
          Serial2.print ("charger_error.txt=");
          Serial2.write(0x22);
          if (ElconCharger_msgInvalid)
            {
              Serial2.print ("Invalid CAN message");
            }
          else
            {
              if (ElconCharger_CommTimeout==1)                     Serial2.print ("CommTimeout;");
              if (ElconCharger_HardwareError==1)                     Serial2.print ("HardwareError;");      
              if (ElconCharger_TempError==1)                     Serial2.print ("OverTemp;");
              if (ElconCharger_BatVoltError==1)                     Serial2.print ("BatteryVoltError;");
              if (ElconCharger_InVoltError==1)                     Serial2.print ("InputVoltError;");
              if (ElconCharger_CommTimeout==0 && ElconCharger_HardwareError==0 && ElconCharger_TempError==0 && ElconCharger_BatVoltError==0 && ElconCharger_InVoltError==0) 
                {Serial2.print ("Ok");}
            }
          Serial2.write(0x22);
          Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
          Serial2.write(0xff);
          Serial2.write(0xff);

          Serial2.print ("charger_stat.txt=");
          Serial2.write(0x22);
          if (ElconCharger_msgInvalid)
            {
              Serial2.print ("Invalid CAN message");
            }
          else
            {
              // if (ElconCharger_Sts==1)       Serial2.print ("ON ");
              // else if (ElconCharger_Sts==0)  Serial2.print ("OFF ");
              Serial2.print ("DC:");
              Serial2.print ((float) (ElconCharger_VoltOutput/10.0),1);
              Serial2.print ("V");
              Serial2.print ((float) (ElconCharger_AmpOutput/10.0),1);
              Serial2.print ("A AC:");

              Serial2.print ((float) (ElconCharger_AmpOutput*ElconCharger_CurrentConst/10.0),0);
              Serial2.print ("/");
              if (BMS_State==BMS_Charge && LIM_ACSE_I_Avbl_Grid < 253) Serial2.print (LIM_ACSE_I_Avbl_Grid);
              else Serial2.print ("0");
              Serial2.print ("A");
            }
          Serial2.write(0x22);
          Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
          Serial2.write(0xff);
          Serial2.write(0xff);

          Serial2.print ("charger_time.txt=");
          Serial2.write(0x22);
          if (BMS_State==BMS_Charge)
            {
              if(ctr_hrs_EOC>0)
                {
                  Serial2.print (ctr_hrs_EOC);
                  Serial2.print ("hrs ");
                }
              Serial2.print (ctr_mins_EOC);
              Serial2.print ("mins to complete");
            }
          else
            {
              Serial2.print ("");
            }
          Serial2.write(0x22);
          Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
          Serial2.write(0xff);
          Serial2.write(0xff);
        }
      DashUpdate_Stamp = millis();
  }


void loop() {
  // if (millis() > (Debug_Stamp + 1000)) {Debug ();}
  if (millis() > (DashUpdate_Stamp + 1000)) {flash=!flash;dashupdate ();}
  if (millis() > (BMS_watchdog + 2000)) Set_ZeroBMS ();
  if (millis() > (Drive_watchdog + 1000)) Set_ZeroDrive ();
  if (millis() > (IBooster_watchdog + 2000)) Set_ZeroIBooster();
  if (millis() > (ISA_watchdog + 1000)) {Set_ZeroISA();}
  if (millis() > (Elcon_watchdog + 2000)) Set_ZeroElcon();
  if (millis() > (LED_watchdog + 1000)) {digitalToggle(LED_BUILTIN);LED_watchdog=millis();}
  //monitor CANbus messages
  if (Can1.read(msg))
    {
      // if (msg.id==0x272 || msg.id==0x2EF || msg.id==0x2B2 || msg.id==0x29E || msg.id==0x390 || msg.id==0x337 || msg.id==0x3B4) Check_LIM (msg);  
      if (msg.id==0x2B2 || msg.id==0x29E || msg.id==0x3B4) Check_LIM (msg); 
      if (msg.id==canID_BMSLimits || msg.id==canID_BMSInfo || msg.id==canID_BMSLowHigh || msg.id==canID_BMSSOC || msg.id==canID_BMSStatus || msg.id==canID_BMSWarnings || msg.id==canID_BMSNumbModules) Check_BMS (msg);
      if (msg.id>=0x521 && msg.id<=0x524) Check_ISA (msg);
      if (msg.id==canID_Drive || msg.id==canID_DriveRPM) Check_Drive (msg);
      if (msg.id==canID_stsIBooster) Check_IBooster(msg);
      if (msg.id==canID_ElconChargerFback || msg.id==canID_ElconDCDCFback) Check_Elcon(msg); 
    }
  if (BMS_State==BMS_Charge){Chg_Timers();}
  else{ctr_mins_EOC=0;ctr_hrs_EOC=0;}
}

#pragma once
#include <stdint.h>

/* 
 * -------------------------------------------------------------------------
 * DIRECTINPUT / HID FFB QUIRKS & KNOWLEDGE
 * -------------------------------------------------------------------------
 * - Windows expects all of the I/O/F reports to be wrapped in an application 
 *   collection; otherwise, the device won't be registered as capable of 
 *   force-feedback. Linux is fine either way.
 * - hid-pidff.c in the Linux kernel will "oops" due to a null-pointer deref
 *   if any of its "optional" reports are not present. These include:
 *   Set Envelope, Set Condition, Set Periodic, Set Constant, Set Ramp.
 * - The HID spec specifies directions should be Joystick usages, but Windows 
 *   expects Ordinal.
 * -------------------------------------------------------------------------
 */

// =========================================================================
// REPORT IDs
// =========================================================================
enum HID_Report_IDs : uint8_t {
    REPORT_ID_OUTPUT_SET_EFFECT          = 1,
    REPORT_ID_INPUT_PID_STATUS           = 2,
    REPORT_ID_OUTPUT_SET_ENVELOPE        = 2,
    REPORT_ID_OUTPUT_SET_CONDITION       = 3,
    REPORT_ID_OUTPUT_SET_PERIODIC        = 4,
    REPORT_ID_OUTPUT_SET_CONSTANT_FORCE  = 5,
    REPORT_ID_FEATURE_CREATE_NEW_EFFECT  = 5,
    REPORT_ID_OUTPUT_SET_RAMP_FORCE      = 6,
    REPORT_ID_FEATURE_PID_BLOCK_LOAD     = 6,
    REPORT_ID_OUTPUT_CUSTOM_FORCE_DATA   = 7,
    REPORT_ID_FEATURE_PID_POOL           = 7,
    REPORT_ID_OUTPUT_DOWNLOAD_FORCE      = 8,
    REPORT_ID_OUTPUT_EFFECT_OPERATION    = 10,
    REPORT_ID_OUTPUT_PID_BLOCK_FREE      = 11,
    REPORT_ID_OUTPUT_PID_DEVICE_CONTROL  = 12,
    REPORT_ID_OUTPUT_DEVICE_GAIN         = 13,
    REPORT_ID_OUTPUT_SET_CUSTOM_FORCE    = 14
};

// =========================================================================
// PID FFB EFFECT STRUCTS (Strictly Packed)
// =========================================================================

/// Device --> Host

typedef struct __attribute__((packed)) { // PID state
    uint8_t reportId;         // =2
    uint8_t status;           // Bits: 0=Device Paused, 1=Actuators Enabled, 2=Safety Switch, 3=Actuator Override Switch, 4=Actuator Power
    uint8_t effectBlockIndex; // Bit7=Effect Playing, Bit0..7=EffectId (1..40)
} USB_FFBReport_PIDStatus_Input_Data_t;


/// Host --> Device

typedef struct __attribute__((packed)) { // FFB: Set Effect Output Report
    uint8_t  reportId;              // =1
    uint8_t  effectBlockIndex;      // 1..40
    uint8_t  effectType;            // 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
    uint16_t duration;              // 0..32767 ms
    uint16_t triggerRepeatInterval; // 0..32767 ms
    uint16_t samplePeriod;          // 0..32767 ms
    uint8_t  gain;                  // 0..255 (physical 0..10000)
    uint8_t  triggerButton;         // button ID (0..8)
    uint8_t  enableAxis;            // bits: 0=X, 1=Y, 2=DirectionEnable
    uint8_t  directionX;            // angle (0=0 .. 255=360deg)
    uint8_t  directionY;            // angle (0=0 .. 255=360deg)
} USB_FFBReport_SetEffect_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set Envelope Output Report
    uint8_t  reportId;          // =2
    uint8_t  effectBlockIndex;  // 1..40
    uint16_t attackLevel;
    uint16_t fadeLevel;
    uint32_t attackTime;        // ms
    uint32_t fadeTime;          // ms
} USB_FFBReport_SetEnvelope_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set Condition Output Report
    uint8_t  reportId;             // =3
    uint8_t  effectBlockIndex;     // 1..40
    uint8_t  parameterBlockOffset; // bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
    int16_t  cpOffset;             // 0..255
    int16_t  positiveCoefficient;  // -128..127
    int16_t  negativeCoefficient;  // -128..127
    uint16_t positiveSaturation;   // -128..127
    uint16_t negativeSaturation;   // -128..127
    uint16_t deadBand;             // 0..255
} USB_FFBReport_SetCondition_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set Periodic Output Report
    uint8_t  reportId;          // =4
    uint8_t  effectBlockIndex;  // 1..40
    uint16_t magnitude;
    int16_t  offset;
    uint16_t phase;             // 0..255 (=0..359, exp-2)
    uint32_t period;            // 0..32767 ms
} USB_FFBReport_SetPeriodic_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set ConstantForce Output Report
    uint8_t  reportId;          // =5
    uint8_t  effectBlockIndex;  // 1..40
    int16_t  magnitude;         // -255..255
} USB_FFBReport_SetConstantForce_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set RampForce Output Report
    uint8_t  reportId;          // =6
    uint8_t  effectBlockIndex;  // 1..40
    int16_t  startMagnitude;
    int16_t  endMagnitude;
} USB_FFBReport_SetRampForce_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set CustomForceData Output Report
    uint8_t  reportId;          // =7
    uint8_t  effectBlockIndex;  // 1..40
    uint16_t dataOffset;
    int8_t   data[12];
} USB_FFBReport_SetCustomForceData_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set DownloadForceSample Output Report
    uint8_t  reportId;          // =8
    int8_t   x;
    int8_t   y;
} USB_FFBReport_SetDownloadForceSample_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set EffectOperation Output Report
    uint8_t  reportId;          // =10
    uint8_t  effectBlockIndex;  // 1..40
    uint8_t  operation;         // 1=Start, 2=StartSolo, 3=Stop
    uint8_t  loopCount;
} USB_FFBReport_EffectOperation_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Block Free Output Report
    uint8_t  reportId;          // =11
    uint8_t  effectBlockIndex;  // 1..40
} USB_FFBReport_BlockFree_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Device Control Output Report
    uint8_t  reportId;          // =12
    uint8_t  control;           // 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
} USB_FFBReport_DeviceControl_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: DeviceGain Output Report
    uint8_t  reportId;          // =13
    uint8_t  gain;
} USB_FFBReport_DeviceGain_Output_Data_t;

typedef struct __attribute__((packed)) { // FFB: Set Custom Force Output Report
    uint8_t  reportId;          // =14
    uint8_t  effectBlockIndex;  // 1..40
    uint8_t  sampleCount;
    uint16_t samplePeriod;      // 0..32767 ms
} USB_FFBReport_SetCustomForce_Output_Data_t;

/// Feature

typedef struct __attribute__((packed)) { // FFB: Create New Effect Feature Report
    uint8_t  reportId;          // =5
    uint8_t  effectType;        // Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
    uint16_t byteCount;         // 0..511
} USB_FFBReport_CreateNewEffect_Feature_Data_t;

typedef struct __attribute__((packed)) { // FFB: PID Block Load Feature Report
    uint8_t  reportId;          // =6
    uint8_t  effectBlockIndex;  // 1..40
    uint8_t  loadStatus;        // 1=Success, 2=Full, 3=Error
    uint16_t ramPoolAvailable;  // =0 or 0xFFFF?
} USB_FFBReport_PIDBlockLoad_Feature_Data_t;

typedef struct __attribute__((packed)) { // FFB: PID Pool Feature Report
    uint8_t  reportId;               // =7
    uint16_t ramPoolSize;            // 
    uint8_t  maxSimultaneousEffects; // 40
    uint8_t  memoryManagement;       // Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
} USB_FFBReport_PIDPool_Feature_Data_t;


// =========================================================================
// HID REPORT DESCRIPTOR
// =========================================================================

const uint8_t hid_report_descriptor[] = {
  // PID State Report
  0x05, 0x0F,          // USAGE_PAGE (Physical Interface)
  0x09, 0x92,          // USAGE (PID State Report)
  0xA1, 0x02,          // COLLECTION (Logical)
	0x85, 0x02,          // REPORT_ID (02)
	0x09, 0x9F,          // USAGE (Device Paused)
	0x09, 0xA0,          // USAGE (Actuators Enabled)
	0x09, 0xA4,          // USAGE (Safety Switch)
	0x09, 0xA5,          // USAGE (Actuator Override Switch)
	0x09, 0xA6,          // USAGE (Actuator Power)
	0x15, 0x00,          // LOGICAL_MINIMUM (00)
	0x25, 0x01,           //  Logical Maximum (1)
	0x35, 0x00,           //  Physical Minimum (0)
	0x45, 0x01,           //  Physical Maximum (1)
	0x75, 0x01,           //  Report Size (1)
	0x95, 0x05,           //  Report Count (5)
	0x81, 0x02,           //  Input (variable,absolute)
	0x95, 0x03,           //  Report Count (3)
	0x81, 0x03,           //  Input (Constant, Variable)
	0x09, 0x94,           //  Usage (Effect Playing)
	0x15, 0x00,           //  Logical Minimum (0)
	0x25, 0x01,           //  Logical Maximum (1)
	0x35, 0x00,           //  Physical Minimum (0)
	0x45, 0x01,           //  Physical Maximum (1)
	0x75, 0x01,           //  Report Size (1)
	0x95, 0x01,           //  Report Count (1)
	0x81, 0x02,           //  Input (variable,absolute)
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //  Logical Minimum (1)
	0x25, 0x28,           //  Logical Maximum (40)
	0x35, 0x01,           //  Physical Minimum (1)
	0x45, 0x28,           //  Physical Maximum (40)
	0x75, 0x07,           //  Report Size (7)
	0x95, 0x01,           //  Report Count (1)
	0x81, 0x02,           //  Input (variable,absolute)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  //================================OutputReport======================================//

  // SetEffectReport
  0x09, 0x21,           //Usage (Set Effect Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x01,           //Report ID 1
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x25,           //  Usage (Effect Type)
	0xA1, 0x02,           //    Collection Datalink (Logical)
	  0x09, 0x26,           // USAGE (26)
	  0x09, 0x27,           // USAGE (27)
	  0x09, 0x30,           // USAGE (30)
	  0x09, 0x31,           // USAGE (31)
	  0x09, 0x32,           // USAGE (32)
	  0x09, 0x33,           // USAGE (33)
	  0x09, 0x34,           // USAGE (34)
	  0x09, 0x40,           // USAGE (40)
	  0x09, 0x41,           // USAGE (41)
	  0x09, 0x42,           // USAGE (42)
	  0x09, 0x43,           // USAGE (43)
	  0x09, 0x28,           // USAGE (28)
	  0x09, 0x28,           // Usage (ET Custom Force Data)
	  0x15, 0x01,           //       Logical Minimum (1)
	  0x25, 0x0C,           //       Logical Maximum (12)
	  0x35, 0x01,           //       Physical Minimum (1)
	  0x45, 0x0C,           //       Physical Maximum (12)
	  0x75, 0x08,           //       Report Size (8)
	  0x95, 0x01,           //       Report Count (1)
	  0x91, 0x00,           //       Output (Data)
	0xC0,                 //    End Collection Datalink (Logical)
	0x09, 0x50,           //    Usage (Duration)
	0x09, 0x54,           //    Usage (Trigger Repeat Interval)
	0x09, 0x51,           //    Usage (Sample Period)
	0x15, 0x00,           //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,     //     Logical Maximum (32767)
	0x35, 0x00,           //     Physical Minimum (1)
	0x46, 0xFF, 0x7F,     //     Physical Maximum (32767)
	0x66, 0x03, 0x10,     //     Unit (4099)
	0x55, 0xFD,           //     Unit Exponent (253)
	0x75, 0x10,           //     Report Size (16)
	0x95, 0x03,           //     Report Count (3)
	0x91, 0x02,           //     Output (Data,Var,Abs)
	0x55, 0x00,           //     Unit Exponent (0)
	0x66, 0x00, 0x00,     //     Unit (0)
	0x09, 0x52,           //    Usage (Gain)
	0x15, 0x00,           //     Logical Minimum (0)
	0x26, 0xFF, 0x00,     //     Logical Maximum (255)
	0x35, 0x00,           //     Physical Minimum (1)
	0x46, 0x10, 0x27,     //     Physical Maximum (10000)
	0x75, 0x08,           //     Report Size (8)
	0x95, 0x01,           //     Report Count (1)
	0x91, 0x02,           //     Output (Data,Var,Abs)
	0x09, 0x53,           //    Usage (Trigger Button)
	0x15, 0x01,           //     Logical Minimum (1)
	0x25, 0x08,           //     Logical Maximum (8)
	0x35, 0x01,           //     Physical Minimum (1)
	0x45, 0x08,           //     Physical Maximum (8)
	0x75, 0x08,           //     Report Size (8)
	0x95, 0x01,           //     Report Count (1)
	0x91, 0x02,           //     Output (Data,Var,Abs)
	0x09, 0x55,           //    Usage (Axes Enable)
	0xA1, 0x02,           //      Collection Datalink (Logical)
	  0x05, 0x01,           //        Usage Page (Generic Desktop)
	  0x09, 0x30,           //        Usage (X)//
	  0x09, 0x31,           //        Usage (Y)//
	  0x15, 0x00,           //        Logical Minimum (0)
	  0x25, 0x01,           //        Logical Maximum (1)
	  0x75, 0x01,           //        Report Size (1)
	  0x95, 0x02,           //        Report Count (2)
	  0x91, 0x02,           //        Output (Data,Var,Abs)
	0xC0,                 //      End Collection Datalink (Logical)

	0x05, 0x0F,           //    Usage Page (Physical Interface)
	0x09, 0x56,           //      Usage (Direction Enable)
	0x95, 0x01,           //        Report Count (1)
	0x91, 0x02,           //        Output (Data,Var,Abs)
	0x95, 0x05,           //        Report Count (5)
	0x91, 0x03,           //        Output (Constant, Variable)
	0x09, 0x57,           //      Usage (Direction)
	0xA1, 0x02,           //        Collection Datalink (Logical)
	  0x0B, 0x01, 0, 0x0A, 0,  //          Usage (Ordinals: Instance 1)
	  0x0B, 0x02, 0, 0x0A, 0,  //          Usage (Ordinals: Instance 2)
	  0x66, 0x14, 0x00,     //          Unit (20)
	  0x55, 0xFE,           //          Unit Exponent (254)
	  0x15, 0x00,           //          Logical Minimum (0)
	  0x26, 0xFF, 0x00,     //          Logical Maximum (255)
	  0x35, 0x00,           //          Physical Minimum (1)
	  0x47, 0xA0, 0x8C, 0, 0, //          Physical Maximum (36000)
	  0x66, 0x00, 0x00,     //          Unit (0)
	  0x75, 0x08,           //          Report Size (8)
	  0x95, 0x02,           //          Report Count (2)
	  0x91, 0x02,           //          Output (Data,Var,Abs)
	  0x55, 0x00,           //          Unit Exponent (0)
	  0x66, 0x00, 0x00,     //          Unit (0)
	0xC0,                 //        End Collection Datalink (Logical)


	0x05, 0x0F,           //    Usage Page (Physical Interface)
	0x09, 0x58,           //      Usage (Type Specific Block Offset)
	0xA1, 0x02,           //        Collection (Logical)
	  0x0B, 0x01, 0, 0x0A, 0,  //          Usage (Ordinals: Instance 1)
	  0x0B, 0x02, 0, 0x0A, 0,  //          Usage (Ordinals: Instance 2)
	  0x26, 0xFD, 0x7F,     //          Logical Maximum (32765)
	  0x75, 0x10,           //          Report Size (16)
	  0x95, 0x02,           //          Report Count (2)
	  0x91, 0x02,           //          Output (Data,Var,Abs)
	0xC0,                 //        End Collection (Logical)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // SetEnvelopeReport
  0x09, 0x5A,           //Usage (Set Envelope Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x02,           //Report ID 2
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x5B,           //  Usage (Attack Level)
	0x09, 0x5D,           //  Usage (Fade Level)
	0x16, 0x00, 0x00,     //   Logical Minimum (0)
	0x26, 0x10, 0x27,     //   Logical Maximum (10000)
	0x36, 0x00, 0x00,     //   Physical Minimum (0)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x02,           //   Report Count (2)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x5C,           //  Usage (Attack Time)
	0x09, 0x5E,           //  Usage (Fade Time)
	0x66, 0x03, 0x10,     //   Unit (1003h) English Linear, Seconds
	0x55, 0xFD,           //   Unit Exponent (FDh) (X10^-3 ==> Milisecond)
	0x27, 0xFF, 0x7F, 0, 0, //   Logical Maximum (4294967295)
	0x47, 0xFF, 0x7F, 0, 0, //   Physical Maximum (4294967295)
	0x75, 0x20,           //   Report Size (16)
	0x95, 0x02,           //   Report Count (2)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x45, 0x00,           //   Physical Maximum (0)
	0x66, 0x00, 0x00,     //   Unit (0)
	0x55, 0x00,           //   Unit Exponent (0)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // SetConditionReport
  0x09, 0x5F,           //Usage (Set Condition Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x03,           //Report ID 3
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x23,           //  Usage (Parameter Block Offset)
	0x15, 0x00,           //   Logical Minimum (0)
	0x25, 0x03,           //   Logical Maximum (3)
	0x35, 0x00,           //   Physical Minimum (0)
	0x45, 0x03,           //   Physical Maximum (3)
	0x75, 0x04,           //   Report Size (4)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x58,           //  Usage (Type Specific Block Off...)
	0xA1, 0x02,           //  Collection Datalink (Logical)
	  0x0B, 0x01, 0, 0x0A, 0,  //    Usage (Ordinals: Instance 1)
	  0x0B, 0x02, 0, 0x0A, 0,  //    Usage (Ordinals: Instance 2)
	  0x75, 0x02,           //     Report Size (2)
	  0x95, 0x02,           //     Report Count (2)
	  0x91, 0x02,           //     Output (Data,Var,Abs)
	0xC0,                 //  End Collection Datalink (Logical)
	0x16, 0xF0, 0xD8,       //  Logical Minimum (-10000)
	0x26, 0x10, 0x27,       //  Logical Maximum (10000)
	0x36, 0xF0, 0xD8,       //  Physical Minimum (-10000)
	0x46, 0x10, 0x27,       //  Physical Maximum (10000)
	0x09, 0x60,           //  Usage (CP Offset)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x36, 0xF0, 0xD8,       //  Physical Minimum (-10000)
	0x46, 0x10, 0x27,       //  Physical Maximum (10000)
	0x09, 0x61,           //  Usage (Positive Coefficient)
	0x09, 0x62,           //  Usage (Negative Coefficient)
	0x95, 0x02,           //   Report Count (2)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x16, 0x00, 0x00,       //   Logical Minimum (0)
	0x26, 0x10, 0x27,       //   Logical Maximum (10000)
	0x36, 0x00, 0x00,       //   Physical Minimum (0)
	0x46, 0x10, 0x27,       //   Physical Maximum (10000)
	0x09, 0x63,           //  Usage (Positive Saturation)
	0x09, 0x64,           //  Usage (Negative Saturation)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x02,           //   Report Count (2)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x65,           //  Usage (Dead Band)
	0x46, 0x10, 0x27,       //   Physical Maximum (10000)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // SetPeriodicReport
  0x09, 0x6E,           //Usage (Set Periodic Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x04,           //Report ID 4
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x70,           //  Usage (Magnitude)
	0x16, 0x00, 0x00,     //   Logical Minimum (0)
	0x26, 0x10, 0x27,     //   Logical Maximum (10000)
	0x36, 0x00, 0x00,     //   Physical Minimum (0)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x6F,           //  Usage (Offset)
	0x16, 0xF0, 0xD8,     //   Logical Minimum (-10000)
	0x26, 0x10, 0x27,     //   Logical Maximum (10000)
	0x36, 0xF0, 0xD8,     //   Physical Minimum (-10000)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x95, 0x01,           //   Report Count (1)
	0x75, 0x10,           //   Report Size (16)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x71,           //  Usage (Phase)
	0x66, 0x14, 0x00,     //   Unit (14h) (Eng Rotation, Degrees)
	0x55, 0xFE,           //   Unit Exponent (FEh) (X10^-2)
	0x15, 0x00,           //   Logical Minimum (0)
	0x27, 0x9F, 0x8C, 0, 0, //   Logical Maximum (35999)
	0x35, 0x00,           //   Physical Minimum (0)
	0x47, 0x9F, 0x8C, 0, 0, //   Physical Maximum (35999)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x72,           //  Usage (Period)
	0x15, 0x00,           //   Logical Minimum (0)
	0x27, 0xFF, 0x7F, 0, 0, //   Logical Maximum (32K)
	0x35, 0x00,           //   Physical Minimum (0)
	0x47, 0xFF, 0x7F, 0, 0, //   Physical Maximum (32K)
	0x66, 0x03, 0x10,     //   Unit (1003h) (English Linear, Seconds)
	0x55, 0xFD,           //   Unit Exponent (FDh) (X10^-3 ==> Milisecond)
	0x75, 0x20,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x66, 0x00, 0x00,     //  Unit (0)
	0x55, 0x00,           //  Unit Exponent (0)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // SetConstantForceReport
  0x09, 0x73,           //Usage (Set Constant Force Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x05,           // Report ID 5
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x70,           //  Usage (Magnitude)
	0x16, 0xF0, 0xD8,     //   Logical Minimum (-10000)
	0x26, 0x10, 0x27,     //   Logical Maximum (10000)
	0x36, 0xF0, 0xD8,     //   Physical Minimum (-10000)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // SetRampForceReport
  0x09, 0x74,           //Usage (Set Ramp Force Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x06,           // Report ID 6
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x75,           //  Usage (Ramp Start)
	0x09, 0x76,           //  Usage (Ramp End)
	0x16, 0xF0, 0xD8,     //   Logical Minimum (-10000)
	0x26, 0x10, 0x27,     //   Logical Maximum (10000)
	0x36, 0xF0, 0xD8,     //   Physical Minimum (-10000)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x02,           //   Report Count (2)
	0x91, 0x02,           //   Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // CustomForceDataReport
  0x09, 0x68,           //Usage (Custom Force Data Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x07,           // Report ID 7
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x6C,           //  Usage (Custom Force Data Offset)
	0x15, 0x00,           //   Logical Minimum (0)
	0x26, 0x10, 0x27,     //   Logical Maximum (10000)
	0x35, 0x00,           //   Physical Minimum (0)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x69,           //  Usage (Custom Force Data)
	0x15, 0x81,           //   Logical Minimum (-127)
	0x25, 0x7F,           //   Logical Maximum (127)
	0x35, 0x00,           //   Physical Minimum (0)
	0x46, 0xFF, 0x00,     //   Physical Maximum (255)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x0C,           //   Report Count (12)
	0x92, 0x02, 0x01,     //   Output (Variable, Buffered)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // DownloadForceSample
  0x09, 0x66,           //Usage (Download Force Sample)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x08,           //Report ID 8
	0x05, 0x01,           //  Usage Page (Generic Desktop)
	0x09, 0x30,           //    Usage (X)
	0x09, 0x31,           //    Usage (Y)
	0x15, 0x81,           //     Logical Minimum (-127)
	0x25, 0x7F,           //     Logical Maximum (127)
	0x35, 0x00,           //     Physical Minimum (0)
	0x46, 0xFF, 0x00,     //     Physical Maximum (255)
	0x75, 0x08,           //     Report Size (8)
	0x95, 0x01,           //     Report Count (2)
	0x91, 0x02,           //     Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // EffectOperationReport
  0x05, 0x0F,           //Usage Page (Physical Interface)
  0x09, 0x77,           //Usage (Effect Operation Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x0A,          //Report ID 10
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x78,           //  Usage (Effect Operation)
	0xA1, 0x02,           //  Collection Datalink (Logical)
	  0x09, 0x79,           //    Usage (Op Effect Start)
	  0x09, 0x7A,           //    Usage (Op Effect Start Solo)
	  0x09, 0x7B,           //    Usage (Op Effect Stop)
	  0x15, 0x01,           //     Logical Minimum (1)
	  0x25, 0x03,           //     Logical Maximum (3)
	  0x75, 0x08,           //     Report Size (8)
	  0x95, 0x01,           //     Report Count (1)
	  0x91, 0x00,           //     Output (Data)
	0xC0,                 //  End Collection Datalink (Logical)
	0x09, 0x7C,           //  Usage (Loop Count)
	0x15, 0x00,           //   Logical Minimum (0)
	0x26, 0xFF, 0x00,     //   Logical Maximum (255)
	0x35, 0x00,           //   Physical Minimum (0)
	0x46, 0xFF, 0x00,     //   Physical Maximum (255)
	0x91, 0x02,           //   Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // PIDBlockFreeReport
  0x09, 0x90,           //Usage (PID Block Free Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x0B,           // Report ID 11
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)
  //PIDDeviceControl
  0x09, 0x96,           //Usage (PID Device Control)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x0C,           // Report ID 12
	0x09, 0x97,           //  Usage (DC Enable Actuators)
	0x09, 0x98,           //  Usage (DC Disable Actuators)
	0x09, 0x99,           //  Usage (DC Stop All Effects)
	0x09, 0x9A,           //  Usage (DC Device Reset)
	0x09, 0x9B,           //  Usage (DC Device Pause)
	0x09, 0x9C,           //  Usage (DC Device Continue)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x06,           //   Logical Maximum (6)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x00,           //   Output (Data)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  // DeviceGainReport
  0x09, 0x7D,           //Usage (Device Gain Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x0D,           //Report ID 13
	0x09, 0x7E,           //  Usage (Device Gain)
	0x15, 0x00,           //   Logical Minimum (0)
	0x26, 0xFF, 0x00,     //   Logical Maximum (255)
	0x35, 0x00,           //   Physical Minimum (0)
	0x46, 0x10, 0x27,     //   Physical Maximum (10000)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
  0xC0,                 //End Collection Datalink (Logical) (OK)

  //SetCustomForceReport
  0x09, 0x6B,           //Usage (Set Custom Force Report)
  0xA1, 0x02,           //Collection Datalink (Logical)
	0x85, 0x0E,           // Report ID 14
	0x09, 0x22,           //  Usage (Effect Block Index)
	0x15, 0x01,           //   Logical Minimum (1)
	0x25, 0x28,           //   Logical Maximum (40)
	0x35, 0x01,           //   Physical Minimum (1)
	0x45, 0x28,           //   Physical Maximum (40)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x6D,           //  Usage (Sample Count)
	0x15, 0x00,           //   Logical Minimum (0)
	0x26, 0xFF, 0x00,     //   Logical Maximum (255)
	0x35, 0x00,           //   Physical Minimum (0)
	0x46, 0xFF, 0x00,     //   Physical Maximum (255)
	0x75, 0x08,           //   Report Size (8)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x09, 0x51,           //  Usage (Sample Period)
	0x66, 0x03, 0x10,     //   Unit 4099
	0x55, 0xFD,           //   Unit (Exponent 253)
	0x15, 0x00,           //   Logical Minimum (0)
	0x26, 0xFF, 0x7F,     //   Logical Maximum (32767)
	0x35, 0x00,           //   Physical Minimum (0)
	0x46, 0xFF, 0x7F,     //   Physical Maximum (32767)
	0x75, 0x10,           //   Report Size (16)
	0x95, 0x01,           //   Report Count (1)
	0x91, 0x02,           //   Output (Data,Var,Abs)
	0x55, 0x00,           //   Unit (Exponent 0)
	0x66, 0x00, 0x00,     //   Unit 0
  0xC0,                 //End Collection Datalink (Logical) (OK)

  //=========================================FeatureReport======================================//

  //CreateNewEffectReport
  0x09, 0xAB, // USAGE (Create New Effect Report)
  0xA1, 0x02, // COLLECTION (Logical)
	0x85, 0x05, // REPORT_ID (05)
	0x09, 0x25, // USAGE (Effect Type)
	0xA1, 0x02, // COLLECTION (Logical)
	  0x09, 0x26, // USAGE (26)
	  0x09, 0x27, // USAGE (27)
	  0x09, 0x30, // USAGE (30)
	  0x09, 0x31, // USAGE (31)
	  0x09, 0x32, // USAGE (32)
	  0x09, 0x33, // USAGE (33)
	  0x09, 0x34, // USAGE (34)
	  0x09, 0x40, // USAGE (40)
	  0x09, 0x41, // USAGE (41)
	  0x09, 0x42, // USAGE (42)
	  0x09, 0x43, // USAGE (43)
	  0x09, 0x28, // USAGE (28)
	  0x25, 0x0C, // LOGICAL_MAXIMUM (0C)
	  0x15, 0x01, // LOGICAL_MINIMUM (01)
	  0x35, 0x01, // PHYSICAL_MINIMUM (01)
	  0x45, 0x0C, // PHYSICAL_MAXIMUM (0C)
	  0x75, 0x08, // REPORT_SIZE (08)
	  0x95, 0x01, // REPORT_COUNT (01)
	  0xB1, 0x00, // FEATURE (Data)
	0xC0, // END COLLECTION ()
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x3B, // USAGE (Byte Count)
	0x15, 0x00, // LOGICAL_MINIMUM (00)
	0x26, 0xFF, 0x01, // LOGICAL_MAXIMUM (511)
	0x35, 0x00, // PHYSICAL_MINIMUM (00)
	0x46, 0xFF, 0x01, // PHYSICAL_MAXIMUM (511)
	0x75, 0x0A, // REPORT_SIZE (0A)
	0x95, 0x01, // REPORT_COUNT (01)
	0xB1, 0x02, // FEATURE (Data,Var,Abs)
	0x75, 0x06, // REPORT_SIZE (06)
	0xB1, 0x01, // FEATURE (Constant,Ary,Abs)
  0xC0, // END COLLECTION ()

  // PIDBlockLoadReport
  0x05, 0x0F, // USAGE_PAGE (Physical Interface)
  0x09, 0x89, // USAGE (PID Block Load Report)
  0xA1, 0x02, // COLLECTION (Logical)
	0x85, 0x06, // REPORT_ID (06)
	0x09, 0x22, // USAGE (Effect Block Index)
	0x25, 0x28, // LOGICAL_MAXIMUM (28)
	0x15, 0x01, // LOGICAL_MINIMUM (01)
	0x35, 0x01, // PHYSICAL_MINIMUM (01)
	0x45, 0x28, // PHYSICAL_MAXIMUM (28)
	0x75, 0x08, // REPORT_SIZE (08)
	0x95, 0x01, // REPORT_COUNT (01)
	0xB1, 0x02, // FEATURE (Data,Var,Abs)
	0x09, 0x8B, // USAGE (Block Load Status)
	0xA1, 0x02, // COLLECTION (Logical)
	  0x09, 0x8C, // USAGE (Block Load Success)
	  0x09, 0x8D, // USAGE (Block Load Full)
	  0x09, 0x8E, // USAGE (Block Load Error)
	  0x25, 0x03, // LOGICAL_MAXIMUM (03)
	  0x15, 0x01, // LOGICAL_MINIMUM (01)
	  0x35, 0x01, // PHYSICAL_MINIMUM (01)
	  0x45, 0x03, // PHYSICAL_MAXIMUM (03)
	  0x75, 0x08, // REPORT_SIZE (08)
	  0x95, 0x01, // REPORT_COUNT (01)
	  0xB1, 0x00, // FEATURE (Data)
	0xC0, // END COLLECTION ()
	0x09, 0xAC, // USAGE (RAM Pool Available)
	0x15, 0x00, // LOGICAL_MINIMUM (00)
	0x27, 0xFF, 0xFF, 0x00, 0x00, // LOGICAL_MAXIMUM (00 00 FF FF)
	0x35, 0x00, // PHYSICAL_MINIMUM (00)
	0x47, 0xFF, 0xFF, 0x00, 0x00, // PHYSICAL_MAXIMUM (00 00 FF FF)
	0x75, 0x10, // REPORT_SIZE (10)
	0x95, 0x01, // REPORT_COUNT (01)
	0xB1, 0x00, // FEATURE (Data)
  0xC0, // END COLLECTION ()

  // PIDPoolReport
  0x09, 0x7F, // USAGE (PID Pool Report)
  0xA1, 0x02, // COLLECTION (Logical)
	0x85, 0x07, // REPORT_ID (07)
	0x09, 0x80, // USAGE (RAM Pool Size)
	0x75, 0x10, // REPORT_SIZE (10)
	0x95, 0x01, // REPORT_COUNT (01)
	0x15, 0x00, // LOGICAL_MINIMUM (00)
	0x35, 0x00, // PHYSICAL_MINIMUM (00)
	0x27, 0xFF, 0xFF, 0x00, 0x00, // LOGICAL_MAXIMUM (00 00 FF FF)
	0x47, 0xFF, 0xFF, 0x00, 0x00, // PHYSICAL_MAXIMUM (00 00 FF FF)
	0xB1, 0x02, // FEATURE (Data,Var,Abs)
	0x09, 0x83, // USAGE (Simultaneous Effects Max)
	0x26, 0xFF, 0x00, // LOGICAL_MAXIMUM (00 FF)
	0x46, 0xFF, 0x00, // PHYSICAL_MAXIMUM (00 FF)
	0x75, 0x08, // REPORT_SIZE (08)
	0x95, 0x01, // REPORT_COUNT (01)
	0xB1, 0x02, // FEATURE (Data,Var,Abs)
	0x09, 0xA9, // USAGE (Device Managed Pool)
	0x09, 0xAA, // USAGE (Shared Parameter Blocks)
	0x75, 0x01, // REPORT_SIZE (01)
	0x95, 0x02, // REPORT_COUNT (02)
	0x15, 0x00, // LOGICAL_MINIMUM (00)
	0x25, 0x01, // LOGICAL_MAXIMUM (01)
	0x35, 0x00, // PHYSICAL_MINIMUM (00)
	0x45, 0x01, // PHYSICAL_MAXIMUM (01)
	0xB1, 0x02, // FEATURE (Data,Var,Abs)
	0x75, 0x06, // REPORT_SIZE (06)
	0x95, 0x01, // REPORT_COUNT (01)
	0xB1, 0x03, // FEATURE ( Cnst,Var,Abs)
  0xC0, // END COLLECTION ()
0xC0 // END COLLECTION ()
};

/*
 * file: Rte_DataTypes.h
 *
 * This file contains all datatypes for programm architecture.
 *
 * $Author: $
 * $Date: $
 * $Revision: $
 *
 * Magnetic Active Suspension System.
 */
#ifndef RTE_DATA_TYPES_H
#define RTE_DATA_TYPES_H


#ifdef COMPONENT_TEST
#define STATIC_GLOBAL
#else
#define STATIC_GLOBAL static
#endif

/* Base datatypes */
#define TRUE	1
#define FALSE	0

typedef  unsigned char  boolean;
typedef  float  float32;
typedef  signed short  int16;
typedef  signed int  int32;
typedef  signed char  int8;
typedef  unsigned short  uint16;
typedef  unsigned int  uint32;
typedef  unsigned char  uint8;
#define arrAccelerationData_ELEMENTS_COUNT        14U

typedef struct
{
    uint8 Values[14];
} arrAccelerationData;


/* Rte data types */
typedef uint16               Std_ReturnType;
typedef void    *Rte_ComponentInstance;

/* Enums data types */

/* Enum Datatype : enAccelerationDriverInitState */
typedef enum 
{
    INIT_STATE_CHECK_BUSY = 0,
    INIT_STATE_WHO_AM_I = 1,
    INIT_STATE_WRITE_POWER = 2,
    INIT_STATE_WRITE_HYROSCOPE = 3,
    INIT_STATE_WRITE_ACCELEROMETER = 4,
    INIT_STATE_WRITE_OK = 5,
    INIT_STATE_FAKE_STOP = 6,
} enAccelerationDriverInitState;

#define enAccelerationDriverInitState_LOWER_LIMIT 0
#define enAccelerationDriverInitState_UPPER_LIMIT 6
#define enAccelerationDriverInitState_ELEMENTS_COUNT 7

/* Enum Datatype : enAccelerationDriverState */
typedef enum 
{
    ACC_DRIVER_STATE_INIT = 0,
    ACC_DRIVER_STATE_ERROR = 1,
    ACC_DRIVER_STATE_READ_DATA = 2,
} enAccelerationDriverState;

#define enAccelerationDriverState_LOWER_LIMIT     0
#define enAccelerationDriverState_UPPER_LIMIT     2
#define enAccelerationDriverState_ELEMENTS_COUNT  3

/* Enum Datatype : enActuatorStatus */
typedef enum 
{
    ACTUATOR_STATUS_INIT = 0,
    ACTUATOR_STATUS_OFF = 1,
    ACTUATOR_STATUS_STANDBY = 2,
    ACTUATOR_STATUS_WORKING = 3,
    ACTUATOR_STATUS_INIT_TO_WORKING = 4,
    ACTUATOR_STATUS_WORKING_TO_INIT = 5,
} enActuatorStatus;

#define enActuatorStatus_LOWER_LIMIT              0
#define enActuatorStatus_UPPER_LIMIT              5
#define enActuatorStatus_ELEMENTS_COUNT           6

/* Enum Datatype : enActuatorTesterStatus */
typedef enum 
{
    ATS_INIT = 0,
    ATS_WAITING_FOR_START = 1,
    ATS_TESTING = 2,
    ATS_TEST_FAILED = 3,
    ATS_NOT_FINISHED = 4,
    ATS_SUCCESS = 5,
} enActuatorTesterStatus;

#define enActuatorTesterStatus_LOWER_LIMIT        0
#define enActuatorTesterStatus_UPPER_LIMIT        5
#define enActuatorTesterStatus_ELEMENTS_COUNT     6

/* Enum Datatype : enAdcDriverState */
typedef enum 
{
    ADC_STATE_INIT = 0,
    ADC_STATE_WORKING = 1,
    ADC_STATE_ERROR = 2,
} enAdcDriverState;

#define enAdcDriverState_LOWER_LIMIT              0
#define enAdcDriverState_UPPER_LIMIT              2
#define enAdcDriverState_ELEMENTS_COUNT           3

/* Enum Datatype : enBatteryRelayControl */
typedef enum 
{
    BATTERY_RELAY_NO_REQUEST = 0,
    BATTERY_RELAY_REQUEST_OPEN = 1,
    BATTERY_RELAY_REQUEST_CLOSE = 2,
} enBatteryRelayControl;

#define enBatteryRelayControl_LOWER_LIMIT         0
#define enBatteryRelayControl_UPPER_LIMIT         2
#define enBatteryRelayControl_ELEMENTS_COUNT      3

/* Enum Datatype : enBatteryRelayStatus */
typedef enum 
{
    BRIDGE_RELAY_OPEN = 0,
    BRIDGE_RELAY_CLOSING_FAILED = 1,
    BRIDGE_RELAY_CLOSING_IN_PROGRESS = 2,
    BRIDGE_RELAY_CLOSED = 3,
} enBatteryRelayStatus;

#define enBatteryRelayStatus_LOWER_LIMIT          0
#define enBatteryRelayStatus_UPPER_LIMIT          3
#define enBatteryRelayStatus_ELEMENTS_COUNT       4

/* Enum Datatype : enBridgeRelayVectorIndex */
typedef enum 
{
    VECTOR_UNSET = 0,
    VECTOR_2_1 = 1,
    VECTOR_2_2 = 2,
    VECTOR_2_3 = 3,
    VECTOR_2_4 = 4,
    VECTOR_2_5 = 5,
    VECTOR_2_6 = 6,
    VECTOR_1_1 = 7,
    VECTOR_1_2 = 8,
    VECTOR_1_3 = 9,
    VECTOR_1_4 = 10,
    VECTOR_1_5 = 11,
    VECTOR_1_6 = 12,
    VECTOR_2_7 = 13,
    VECTOR_2_8 = 14,
} enBridgeRelayVectorIndex;

#define enBridgeRelayVectorIndex_LOWER_LIMIT      0
#define enBridgeRelayVectorIndex_UPPER_LIMIT      14
#define enBridgeRelayVectorIndex_ELEMENTS_COUNT   15

/* Enum Datatype : enCalibrationCommand */
typedef enum 
{
    CALIBRATION_COMMAND_NONE = 0,
    CALIBRATION_COMMAND_START = 1,
    CALIBRATION_COMMAND_BREAK = 2,
} enCalibrationCommand;

#define enCalibrationCommand_LOWER_LIMIT          0
#define enCalibrationCommand_UPPER_LIMIT          2
#define enCalibrationCommand_ELEMENTS_COUNT       3

/* Enum Datatype : enConnectionStatus */
typedef enum 
{
    CONN_STAT_INIT = 0,
    CONN_STAT_FAILED = 1,
    CONN_STAT_DISCONNECTED = 2,
    CONN_STAT_CONNECTED = 3,
} enConnectionStatus;

#define enConnectionStatus_LOWER_LIMIT            0
#define enConnectionStatus_UPPER_LIMIT            3
#define enConnectionStatus_ELEMENTS_COUNT         4

/* Enum Datatype : enCurrentSensingStatus */
typedef enum 
{
    CSS_INIT = 0,
    CSS_WORKING = 1,
    CSS_ERROR = 2,
} enCurrentSensingStatus;

#define enCurrentSensingStatus_LOWER_LIMIT        0
#define enCurrentSensingStatus_UPPER_LIMIT        2
#define enCurrentSensingStatus_ELEMENTS_COUNT     3

/* Enum Datatype : enErrorReportMessageKind */
typedef enum 
{
    ERROR_KIND_SET = 0,
    ERROR_KIND_CLEAR = 1,
} enErrorReportMessageKind;

#define enErrorReportMessageKind_LOWER_LIMIT      0
#define enErrorReportMessageKind_UPPER_LIMIT      1
#define enErrorReportMessageKind_ELEMENTS_COUNT   2

/* Enum Datatype : enExtremPointStatus */
typedef enum 
{
    EXTREME_POINT_STATUS_FORBIDDEN = 0,
    EXTREME_POINT_STATUS_ALLOWED = 1,
} enExtremPointStatus;

#define enExtremPointStatus_LOWER_LIMIT           0
#define enExtremPointStatus_UPPER_LIMIT           1
#define enExtremPointStatus_ELEMENTS_COUNT        2

/* Enum Datatype : enGduControl */
typedef enum 
{
    GDU_NO_COMMAND = 0,
    GDU_SWITCH_OFF = 1,
    GDU_SWITCH_ON = 2,
} enGduControl;

#define enGduControl_LOWER_LIMIT                  0
#define enGduControl_UPPER_LIMIT                  2
#define enGduControl_ELEMENTS_COUNT               3

/* Enum Datatype : enGduErrorState */
typedef enum 
{
    GDE_NO_ERROR = 0,
    GDE_FAULT = 1,
    GDE_OCTW = 2,
    GDE_PWRGD = 3,
} enGduErrorState;

#define enGduErrorState_LOWER_LIMIT               0
#define enGduErrorState_UPPER_LIMIT               3
#define enGduErrorState_ELEMENTS_COUNT            4

/* Enum Datatype : enGduState */
typedef enum 
{
    GDU_OFF = 0,
    GDU_INITIALIZING = 1,
    GDU_STANDBY = 2,
    GDU_ERROR_HANDLING = 3,
    GDU_ENABLED = 4,
} enGduState;

#define enGduState_LOWER_LIMIT                    0
#define enGduState_UPPER_LIMIT                    4
#define enGduState_ELEMENTS_COUNT                 5

/* Enum Datatype : enPhase */
typedef enum 
{
    PHASE_U = 0,
    PHASE_V = 1,
    PHASE_W = 2,
} enPhase;

#define enPhase_LOWER_LIMIT                       0
#define enPhase_UPPER_LIMIT                       2
#define enPhase_ELEMENTS_COUNT                    3

/* Enum Datatype : enPIDCommand */
typedef enum 
{
    PID_NO_COMMAND = 0,
    PID_STOP = 1,
    PID_START = 2,
} enPIDCommand;

#define enPIDCommand_LOWER_LIMIT                  0
#define enPIDCommand_UPPER_LIMIT                  2
#define enPIDCommand_ELEMENTS_COUNT               3

/* Enum Datatype : enPort */
typedef enum 
{
    ENUM_PORT_A = 0,
    ENUM_PORT_B = 1,
    ENUM_PORT_C = 2,
    ENUM_PORT_D = 3,
} enPort;

#define enPort_LOWER_LIMIT                        0
#define enPort_UPPER_LIMIT                        3
#define enPort_ELEMENTS_COUNT                     4

/* Enum Datatype : enPwmControl */
typedef enum 
{
    PWM_DISABLE = 0,
    PWM_ENABLE = 1,
} enPwmControl;

#define enPwmControl_LOWER_LIMIT                  0
#define enPwmControl_UPPER_LIMIT                  1
#define enPwmControl_ELEMENTS_COUNT               2

/* Enum Datatype : enPwmDriverStatus */
typedef enum 
{
    PWM_DRIVER_STATE_OFF = 0,
    PWM_DRIVER_STATE_INIT = 1,
    PWM_DRIVER_STATE_STANDBY = 2,
    PWM_DRIVER_STATE_WORKING_PWM = 3,
    PWM_DRIVER_STATE_WORKING_STEP = 4,
} enPwmDriverStatus;

#define enPwmDriverStatus_LOWER_LIMIT             0
#define enPwmDriverStatus_UPPER_LIMIT             4
#define enPwmDriverStatus_ELEMENTS_COUNT          5

/* Enum Datatype : enQualifier */
typedef enum 
{
    SEN_NOT_AVAILABLE = 0,
    SEN_NOT_RELIABLE = 1,
    SEN_RELIABLE = 2,
} enQualifier;

#define enQualifier_LOWER_LIMIT                   0
#define enQualifier_UPPER_LIMIT                   2
#define enQualifier_ELEMENTS_COUNT                3

/* Enum Datatype : enSpaceVectorDirection */
typedef enum 
{
    SV_ZM = 0,
    SV_UP = 1,
    SV_WM = 2,
    SV_VP = 3,
    SV_UM = 4,
    SV_WP = 5,
    SV_VM = 6,
    SV_ZP = 7,
} enSpaceVectorDirection;

#define enSpaceVectorDirection_LOWER_LIMIT        0
#define enSpaceVectorDirection_UPPER_LIMIT        7
#define enSpaceVectorDirection_ELEMENTS_COUNT     8

/* Enum Datatype : enTestingControl */
typedef enum 
{
    TC_UNDEFINED = 0,
    TC_STOP_TESTING = 1,
    TC_START_TESTING = 2,
} enTestingControl;

#define enTestingControl_LOWER_LIMIT              0
#define enTestingControl_UPPER_LIMIT              2
#define enTestingControl_ELEMENTS_COUNT           3

/* Simple data types */

/* Datatype : PidParameter */
#define PidParameter_UPPER_LIMIT                  3.402823e+38
#define PidParameter_LOWER_LIMIT                  -3.402823e+38
typedef  float32        PidParameter;

/* Datatype : sdtAccelerationMs2 */
#define sdtAccelerationMs2_UPPER_LIMIT            3.402823e+38
#define sdtAccelerationMs2_LOWER_LIMIT            -3.402823e+38
typedef  float32        sdtAccelerationMs2;

/* Datatype : sdtActuatorTestsResult */
#define sdtActuatorTestsResult_UPPER_LIMIT        16383
#define sdtActuatorTestsResult_LOWER_LIMIT        0
typedef  uint16         sdtActuatorTestsResult;

/* Datatype : sdtADCTriggerElement */
#define sdtADCTriggerElement_UPPER_LIMIT          1.f
#define sdtADCTriggerElement_LOWER_LIMIT          0.f
typedef  float32        sdtADCTriggerElement;

#define arrADCTriggers_ELEMENTS_COUNT             6U

typedef struct
{
    sdtADCTriggerElement Values[6];
} arrADCTriggers;

/* Datatype : sdtADCValue */
#define sdtADCValue_UPPER_LIMIT                   65535
#define sdtADCValue_LOWER_LIMIT                   0
typedef  uint16         sdtADCValue;

#define arrRawPhaseCurrent_ELEMENTS_COUNT         6U

typedef struct
{
    sdtADCValue Values[6];
} arrRawPhaseCurrent;

/* Datatype : sdtAngle */
#define sdtAngle_UPPER_LIMIT                      3.402823e+38
#define sdtAngle_LOWER_LIMIT                      -3.402823e+38
typedef  float32        sdtAngle;

/* Datatype : sdtAvalaibleDistance */
#define sdtAvalaibleDistance_UPPER_LIMIT          600.f
#define sdtAvalaibleDistance_LOWER_LIMIT          -600.f
typedef  float32        sdtAvalaibleDistance;

/* Datatype : sdtCoefficient */
#define sdtCoefficient_UPPER_LIMIT                3.402823e+38
#define sdtCoefficient_LOWER_LIMIT                -3.402823e+38
typedef  float32        sdtCoefficient;

/* Datatype : sdtCRC32 */
#define sdtCRC32_UPPER_LIMIT                      4294967295
#define sdtCRC32_LOWER_LIMIT                      0
typedef  uint32         sdtCRC32;

/* Datatype : sdtCurrent */
#define sdtCurrent_UPPER_LIMIT                    500.f
#define sdtCurrent_LOWER_LIMIT                    -500.f
typedef  float32        sdtCurrent;

#define arrMeasuredPhaseCurrents_ELEMENTS_COUNT   6U

typedef struct
{
    sdtCurrent Values[6];
} arrMeasuredPhaseCurrents;

#define arrPhaseCurrents_ELEMENTS_COUNT           3U

typedef struct
{
    sdtCurrent Values[3];
} arrPhaseCurrents;

/* Datatype : sdtDeadTimeNs */
#define sdtDeadTimeNs_UPPER_LIMIT                 10000.f
#define sdtDeadTimeNs_LOWER_LIMIT                 0.f
typedef  float32        sdtDeadTimeNs;

/* Datatype : sdtDebugInfoField */
#define sdtDebugInfoField_UPPER_LIMIT             4294967295
#define sdtDebugInfoField_LOWER_LIMIT             0
typedef  uint32         sdtDebugInfoField;

/* Datatype : sdtDutyCycle */
#define sdtDutyCycle_UPPER_LIMIT                  1.f
#define sdtDutyCycle_LOWER_LIMIT                  0.f
typedef  float32        sdtDutyCycle;

#define arrDutyCycles_ELEMENTS_COUNT              6U

typedef struct
{
    sdtDutyCycle Values[6];
} arrDutyCycles;

/* Datatype : sdtElectricalAngle */
#define sdtElectricalAngle_UPPER_LIMIT            360.f
#define sdtElectricalAngle_LOWER_LIMIT            0.f
typedef  float32        sdtElectricalAngle;

/* Datatype : sdtElementIndex */
#define sdtElementIndex_UPPER_LIMIT               4294967295
#define sdtElementIndex_LOWER_LIMIT               0
typedef  uint32         sdtElementIndex;

/* Datatype : sdtErrorID */
#define sdtErrorID_UPPER_LIMIT                    65535
#define sdtErrorID_LOWER_LIMIT                    0
typedef  uint16         sdtErrorID;

/* Datatype : sdtForce */
#define sdtForce_UPPER_LIMIT                      5000.f
#define sdtForce_LOWER_LIMIT                      -5000.f
typedef  float32        sdtForce;

/* Datatype : sdtPhaseLength */
#define sdtPhaseLength_UPPER_LIMIT                200.f
#define sdtPhaseLength_LOWER_LIMIT                0.f
typedef  float32        sdtPhaseLength;

/* Datatype : sdtProtectorIndex */
#define sdtProtectorIndex_UPPER_LIMIT             4294967295
#define sdtProtectorIndex_LOWER_LIMIT             0
typedef  uint32         sdtProtectorIndex;

/* Datatype : sdtPwmDutyCycle */
#define sdtPwmDutyCycle_UPPER_LIMIT               100
#define sdtPwmDutyCycle_LOWER_LIMIT               0
typedef  uint8          sdtPwmDutyCycle;

/* Datatype : sdtRawAccelerationData */
#define sdtRawAccelerationData_UPPER_LIMIT        10000
#define sdtRawAccelerationData_LOWER_LIMIT        0
typedef  int16          sdtRawAccelerationData;

/* Datatype : sdtRawRotationData */
#define sdtRawRotationData_UPPER_LIMIT            32767
#define sdtRawRotationData_LOWER_LIMIT            -32768
typedef  int16          sdtRawRotationData;

/* Datatype : sdtResistance */
#define sdtResistance_UPPER_LIMIT                 3.402823e+38
#define sdtResistance_LOWER_LIMIT                 0.f
typedef  float32        sdtResistance;

/* Datatype : sdtRotationRadS2 */
#define sdtRotationRadS2_UPPER_LIMIT              3.402823e+38
#define sdtRotationRadS2_LOWER_LIMIT              -3.402823e+38
typedef  float32        sdtRotationRadS2;

/* Datatype : sdtRotationSpeedRadS */
#define sdtRotationSpeedRadS_UPPER_LIMIT          3.402823e+38
#define sdtRotationSpeedRadS_LOWER_LIMIT          -3.402823e+38
typedef  float32        sdtRotationSpeedRadS;

/* Datatype : sdtSliderPositionMm */
#define sdtSliderPositionMm_UPPER_LIMIT           300.f
#define sdtSliderPositionMm_LOWER_LIMIT           -300.f
typedef  float32        sdtSliderPositionMm;

/* Datatype : sdtSliderSpeedMmS */
#define sdtSliderSpeedMmS_UPPER_LIMIT             10000.f
#define sdtSliderSpeedMmS_LOWER_LIMIT             -10000.f
typedef  float32        sdtSliderSpeedMmS;

/* Datatype : sdtTemperature */
#define sdtTemperature_UPPER_LIMIT                350.f
#define sdtTemperature_LOWER_LIMIT                -50.f
typedef  float32        sdtTemperature;

/* Datatype : sdtTime */
#define sdtTime_UPPER_LIMIT                       4294967295
#define sdtTime_LOWER_LIMIT                       0
typedef  uint32         sdtTime;

/* Datatype : sdtTimeInMs */
#define sdtTimeInMs_UPPER_LIMIT                   4294967295
#define sdtTimeInMs_LOWER_LIMIT                   0
typedef  uint32         sdtTimeInMs;

/* Datatype : sdtTimeSinceReset */
#define sdtTimeSinceReset_UPPER_LIMIT             4294967295
#define sdtTimeSinceReset_LOWER_LIMIT             0
typedef  uint32         sdtTimeSinceReset;

/* Datatype : sdtVoltage */
#define sdtVoltage_UPPER_LIMIT                    150.f
#define sdtVoltage_LOWER_LIMIT                    0.f
typedef  float32        sdtVoltage;

/* Complex data types */

/* Complex datatype : cdtAcceleration */
typedef struct
{
    sdtAccelerationMs2 X;
    sdtAccelerationMs2 Y;
    sdtAccelerationMs2 Z;
} cdtAcceleration;

/* Complex datatype : cdtADCTrigger */
typedef struct
{
    uint8 measureableInfo;
    arrADCTriggers triggerTimers;
} cdtADCTrigger;

/* Complex datatype : cdtAlphaBetaVector */
typedef struct
{
    float32 Alpha;
    float32 Beta;
} cdtAlphaBetaVector;

/* Complex datatype : cdtCurentLimitationMessage */
typedef struct
{
    sdtCurrent CurrentLimit;
    sdtProtectorIndex ProtectorIndex;
} cdtCurentLimitationMessage;

#define arrCurrentLimitCollection_ELEMENTS_COUNT  2U

typedef struct
{
    cdtCurentLimitationMessage Values[2];
} arrCurrentLimitCollection;

/* Complex datatype : cdtDebugInfo */
typedef struct
{
    sdtDebugInfoField field0;
    sdtDebugInfoField field1;
    sdtDebugInfoField field2;
} cdtDebugInfo;

/* Complex datatype : cdtDQCurrent */
typedef struct
{
    sdtCurrent D;
    sdtCurrent Q;
} cdtDQCurrent;

/* Complex datatype : cdtDQVoltage */
typedef struct
{
    sdtVoltage D;
    sdtVoltage Q;
} cdtDQVoltage;

/* Complex datatype : cdtIntegralSliderPosition */
typedef struct
{
    sdtSliderPositionMm IntegralPosition;
    enQualifier Qualifier;
} cdtIntegralSliderPosition;

/* Complex datatype : cdtMeasuredPhaseCurrents */
typedef struct
{
    uint8 count;
    arrMeasuredPhaseCurrents currents;
    enQualifier qualifier;
} cdtMeasuredPhaseCurrents;

/* Complex datatype : cdtPhaseCurrents */
typedef struct
{
    arrPhaseCurrents Currents;
    enQualifier Qualifier;
} cdtPhaseCurrents;

/* Complex datatype : cdtPidRegulatorParameters */
typedef struct
{
    float32 D;
    float32 I;
    float32 P;
} cdtPidRegulatorParameters;

/* Complex datatype : cdtQualifiedAdcValue */
typedef struct
{
    sdtADCValue adcValue;
    enQualifier qualifier;
} cdtQualifiedAdcValue;

#define arrAdcArray5_ELEMENTS_COUNT               11U

typedef struct
{
    cdtQualifiedAdcValue Values[11];
} arrAdcArray5;

/* Complex datatype : cdtQualifiedCurrent */
typedef struct
{
    sdtCurrent Current;
    enQualifier Qualifier;
} cdtQualifiedCurrent;

/* Complex datatype : cdtQualifiedElectricalAngle */
typedef struct
{
    sdtElectricalAngle Angle;
    enQualifier Qualifier;
} cdtQualifiedElectricalAngle;

/* Complex datatype : cdtQualifiedPhaseCurrent */
typedef struct
{
    sdtCurrent Current;
    enQualifier Qualifier;
} cdtQualifiedPhaseCurrent;

/* Complex datatype : cdtQualifiedSliderAcceleration */
typedef struct
{
    sdtRawAccelerationData Acceleration;
    enQualifier Qualifier;
} cdtQualifiedSliderAcceleration;

/* Complex datatype : cdtQualifiedSliderPosition */
typedef struct
{
    sdtSliderPositionMm Position;
    enQualifier Qualifier;
} cdtQualifiedSliderPosition;

/* Complex datatype : cdtQualifiedSliderSpeed */
typedef struct
{
    enQualifier Qualifier;
    sdtSliderSpeedMmS Speed;
} cdtQualifiedSliderSpeed;

/* Complex datatype : cdtQualifiedTemperature */
typedef struct
{
    enQualifier qualifier;
    sdtTemperature temperature;
} cdtQualifiedTemperature;

/* Complex datatype : cdtQualifiedVoltage */
typedef struct
{
    enQualifier Qualifier;
    sdtVoltage Voltage;
} cdtQualifiedVoltage;

/* Complex datatype : cdtRawAcceleration */
typedef struct
{
    sdtRawAccelerationData X;
    sdtRawAccelerationData Y;
    sdtRawAccelerationData Z;
} cdtRawAcceleration;

/* Complex datatype : cdtRawPhaseCurrent */
typedef struct
{
    uint8 count;
    arrRawPhaseCurrent measuredData;
    enQualifier qualifier;
} cdtRawPhaseCurrent;

/* Complex datatype : cdtRawRotation */
typedef struct
{
    sdtRawRotationData X;
    sdtRawRotationData Y;
    sdtRawRotationData Z;
} cdtRawRotation;

/* Complex datatype : cdtRotation */
typedef struct
{
    sdtRotationRadS2 X;
    sdtRotationRadS2 Y;
    sdtRotationRadS2 Z;
} cdtRotation;

/* Complex datatype : cdtSpaceVector */
typedef struct
{
    enSpaceVectorDirection Direction;
    float32 Duration;
} cdtSpaceVector;

#define arrSpaceVector_ELEMENTS_COUNT             6U

typedef struct
{
    cdtSpaceVector Values[6];
} arrSpaceVector;

/* Complex datatype : cdtErrorReportMessage */
typedef struct
{
    cdtDebugInfo debugInfo;
    sdtErrorID errorID;
    enErrorReportMessageKind messageKind;
} cdtErrorReportMessage;

/* Complex datatype : cdtQualifiedAcceleration */
typedef struct
{
    cdtAcceleration Acceleration;
    enQualifier Qualifier;
} cdtQualifiedAcceleration;

/* Complex datatype : cdtQualifiedRotation */
typedef struct
{
    enQualifier Qualifier;
    cdtRotation Rotation;
} cdtQualifiedRotation;

/* Complex datatype : cdtStatusedDQCurrentVector */
typedef struct
{
    cdtDQCurrent DQCurrent;
    enCurrentSensingStatus Status;
} cdtStatusedDQCurrentVector;

/* Datatype for AccelerationReader */
typedef struct
{
    uint32 index;
    arrAccelerationData Rte_PimField_AccelerationReader_Buffer;
    uint32 Rte_PimField_AccelerationReader_FakeStopCounter;
    sdtElementIndex Rte_PimField_AccelerationReader_I2CIndex;
    enAccelerationDriverInitState Rte_PimField_AccelerationReader_InitState;
    uint8 Rte_PimField_AccelerationReader_RepeatsCount;
    boolean Rte_PimField_AccelerationReader_RequestSent;
    enAccelerationDriverState Rte_PimField_AccelerationReader_State;
    sdtTemperature Rte_CDataField_AccelerationReader_MaxAccelerationMCUTemperature;
    sdtTemperature Rte_CDataField_AccelerationReader_MinAccelerationMCUTemperature;
    cdtQualifiedTemperature Rte_WriteField_AccTemperature_Temperature;
    enQualifier Rte_WriteField_RawData_Qualifier;
    cdtRawAcceleration Rte_WriteField_RawData_RawAcceleration;
    cdtRawRotation Rte_WriteField_RawData_RawRotation;
} AccelerationReader;

/* Datatype for ActuatorBridgeRelayTester */
typedef struct
{
    enTestingControl Rte_PimField_ActuatorBridgeRelayTester_StartTesting;
    sdtCurrent Rte_CDataField_ActuatorBridgeRelayTester_MinAllowedCurrent;
    enActuatorTesterStatus Rte_WriteField_ActuatorTesterStatus_Status;
    sdtActuatorTestsResult Rte_WriteField_ActuatorTesterStatus_TestsResults;
    cdtADCTrigger Rte_WriteField_ADCTriggerOverride_ADCTrigger;
    boolean Rte_WriteField_InverterOverrideStatus_Override;
    arrSpaceVector Rte_WriteField_SpaceVectorsOverride_SpaceVectors;
} ActuatorBridgeRelayTester;

/* Datatype for ActuatorManager */
typedef struct
{
    sdtVoltage Rte_CDataField_ActuatorManager_MinBatteryVoltage;
    sdtVoltage Rte_CDataField_ActuatorManager_StartTestingVoltage;
    enActuatorStatus Rte_WriteField_ActuatorStatus_Status;
} ActuatorManager;

/* Datatype for ADCDriver */
typedef struct
{
    uint8 Rte_PimField_ADCDriver_CurrentsMeasurementCount;
    enAdcDriverState Rte_PimField_ADCDriver_State;
    cdtQualifiedAdcValue Rte_WriteField_Raw_BatteryVoltage_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_BridgeVoltage_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_GduOOS_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Hall1_Ch1_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Hall1_Ch2_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Hall1_Ch3_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Hall2_Ch1_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Hall2_Ch2_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Hall2_Ch3_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_MCUTemperature_qualifiedADC;
    cdtQualifiedAdcValue Rte_WriteField_Raw_MosfetTemperature_qualifiedADC;
    cdtRawPhaseCurrent Rte_WriteField_Raw_PhaseUCurrent_measuredValues;
    cdtRawPhaseCurrent Rte_WriteField_Raw_PhaseVCurrent_measuredValues;
    cdtRawPhaseCurrent Rte_WriteField_Raw_PhaseWCurrent_measuredValues;
    cdtQualifiedAdcValue Rte_WriteField_Raw_Sensing5V_qualifiedADC;
} ADCDriver;

/* Datatype for AdcMedianFilter5 */
typedef struct
{
    uint32 index;
    arrAdcArray5 Rte_PimField_AdcMedianFilter5_Data;
    cdtQualifiedAdcValue Rte_WriteField_FilteredData_qualifiedADC;
} AdcMedianFilter5;

/* Datatype for BatteryCurrentProvider */
typedef struct
{
    cdtQualifiedCurrent Rte_WriteField_BatteryCurrent_Current;
} BatteryCurrentProvider;

/* Datatype for BatteryRelayDriver */
typedef struct
{
    enBatteryRelayControl Rte_PimField_BatteryRelayDriver_RequestControl;
    sdtTimeInMs Rte_CDataField_BatteryRelayDriver_MaxChargeTime;
    sdtVoltage Rte_CDataField_BatteryRelayDriver_MaxVoltageGap;
    sdtVoltage Rte_CDataField_BatteryRelayDriver_MinStartVoltage;
    enBatteryRelayStatus Rte_WriteField_BatteryRelayStatus_Status;
} BatteryRelayDriver;

/* Datatype for BatteryVoltageProvider */
typedef struct
{
    sdtVoltage Rte_CDataField_BatteryVoltageProvider_MaxDifference;
    sdtADCValue Rte_CDataField_BatteryVoltageProvider_MinAdcValue;
    cdtQualifiedVoltage Rte_WriteField_BatteryVoltage_Voltage;
    cdtQualifiedVoltage Rte_WriteField_BridgeVoltage_Voltage;
} BatteryVoltageProvider;

/* Datatype for BoardVoltageProvider */
typedef struct
{
    sdtVoltage Rte_CDataField_BoardVoltageProvider_MaxVoltageValue;
    sdtVoltage Rte_CDataField_BoardVoltageProvider_MinVoltageValue;
    cdtQualifiedVoltage Rte_WriteField_Sensing5V_Voltage;
} BoardVoltageProvider;

/* Datatype for BodyAccelerationProvider */
typedef struct
{
    cdtQualifiedAcceleration Rte_WriteField_Acceleration_Acceleration;
    cdtQualifiedRotation Rte_WriteField_Rotation_Rotation;
} BodyAccelerationProvider;

/* Datatype for CANDriver */
typedef struct
{
} CANDriver;

/* Datatype for CRCCalculator */
typedef struct
{
    uint32 index;
} CRCCalculator;

/* Datatype for CurrentLimitationCollector */
typedef struct
{
    arrCurrentLimitCollection Rte_PimField_CurrentLimitationCollector_Limitations;
    boolean Rte_PimField_CurrentLimitationCollector_MessageReceived;
    sdtCurrent Rte_WriteField_CurrentLimit_MaxCurrent;
} CurrentLimitationCollector;

/* Datatype for DataTransmitter */
typedef struct
{
    uint32 index;
} DataTransmitter;

/* Datatype for DeadTimeCompensator */
typedef struct
{
    sdtDeadTimeNs Rte_CDataField_DeadTimeCompensator_DeadTimeNs;
    cdtDQVoltage Rte_WriteField_CompensatedVoltage_Voltage;
} DeadTimeCompensator;

/* Datatype for DesiredCurrentProvider */
typedef struct
{
    cdtDQCurrent Rte_WriteField_RequiredCurrent_Current;
} DesiredCurrentProvider;

/* Datatype for DistanceProtector */
typedef struct
{
    sdtSliderPositionMm Rte_CDataField_DistanceProtector_MaxLength;
    sdtAvalaibleDistance Rte_WriteField_AvailableDistance_DistanceDown;
    sdtAvalaibleDistance Rte_WriteField_AvailableDistance_DistanceUp;
} DistanceProtector;

/* Datatype for DQCurrentProvider */
typedef struct
{
    cdtStatusedDQCurrentVector Rte_WriteField_DQCurrent_Current;
} DQCurrentProvider;

/* Datatype for ErrorsReportCollector */
typedef struct
{
} ErrorsReportCollector;

/* Datatype for ETSR */
typedef struct
{
} ETSR;

/* Datatype for ExternalControlParser */
typedef struct
{
    enConnectionStatus Rte_WriteField_ConnectionStatus_Status;
    sdtForce Rte_WriteField_DesiredForce_Force;
} ExternalControlParser;

/* Datatype for ExtremePointProtector */
typedef struct
{
    enExtremPointStatus Rte_WriteField_ExtremePointsStatus_StatusDown;
    enExtremPointStatus Rte_WriteField_ExtremePointsStatus_StatusUp;
} ExtremePointProtector;

/* Datatype for GDUDriver_DRV8301 */
typedef struct
{
    enGduControl Rte_PimField_GDUDriver_DRV8301_GduControl;
    enGduErrorState Rte_WriteField_GduErrorState_ErrorState;
    enGduState Rte_WriteField_GduState_State;
} GDUDriver_DRV8301;

/* Datatype for IntegralSliderProvider */
typedef struct
{
    cdtIntegralSliderPosition Rte_PimField_IntegralSliderProvider_IntegralPosition;
    int32 Rte_PimField_IntegralSliderProvider_PhasesCount;
    sdtSliderPositionMm Rte_PimField_IntegralSliderProvider_PositionShift;
    sdtSliderPositionMm Rte_PimField_IntegralSliderProvider_PreviousPosition;
    sdtPhaseLength Rte_CDataField_IntegralSliderProvider_PhaseLengthMm;
    cdtQualifiedElectricalAngle Rte_WriteField_FilteredElectricalAngle_ElectricalAngle;
    cdtIntegralSliderPosition Rte_WriteField_IntegralSliderPosition_IntegralPosition;
} IntegralSliderProvider;

/* Datatype for IntegralSpeedProvider */
typedef struct
{
    cdtQualifiedSliderSpeed Rte_WriteField_Speed_Speed;
} IntegralSpeedProvider;

/* Datatype for Inverter */
typedef struct
{
    float32 Rte_CDataField_Inverter_Limit3AOff;
    float32 Rte_CDataField_Inverter_Limit3AOn;
    float32 Rte_CDataField_Inverter_Limit3AS0Off;
    float32 Rte_CDataField_Inverter_Limit6AOff;
    float32 Rte_CDataField_Inverter_MaxLen3AX;
    float32 Rte_CDataField_Inverter_MinDuration3AX;
    float32 Rte_CDataField_Inverter_MinMeasureableLength;
    cdtADCTrigger Rte_WriteField_ADCTrigger_ADCTrigger;
    arrDutyCycles Rte_WriteField_PwmDutyCycles_dutyCycles;
    arrSpaceVector Rte_WriteField_PwmDutyCycles_spaceVectors;
    arrSpaceVector Rte_WriteField_SpaceVectors_SpaceVectors;
} Inverter;

/* Datatype for LedBlinker */
typedef struct
{
    uint32 index;
    sdtTime Rte_PimField_LedBlinker_BlinkPeriod;
    sdtTimeSinceReset Rte_PimField_LedBlinker_LastRunTime;
    uint16 Rte_PimField_LedBlinker_Pin;
    enPort Rte_PimField_LedBlinker_Port;
} LedBlinker;

/* Datatype for MCUClockInitializer */
typedef struct
{
} MCUClockInitializer;

/* Datatype for MCUCoreTemperature */
typedef struct
{
    float32 Rte_CDataField_MCUCoreTemperature_Avg_Slope;
    sdtTemperature Rte_CDataField_MCUCoreTemperature_MaxTemperature;
    sdtTemperature Rte_CDataField_MCUCoreTemperature_MinTemperature;
    sdtADCValue Rte_CDataField_MCUCoreTemperature_V25;
    cdtQualifiedTemperature Rte_WriteField_Temperature_Temperature;
} MCUCoreTemperature;

/* Datatype for OvertemperatureProtector */
typedef struct
{
    uint32 index;
    sdtCurrent Rte_PimField_OvertemperatureProtector_PreviousCurrentLimitation;
    boolean Rte_PimField_OvertemperatureProtector_StartupFinished;
    uint32 Rte_PimField_OvertemperatureProtector_StartupTimeoutCounter;
    sdtTemperature Rte_CDataField_OvertemperatureProtector_L1Temperature;
    sdtTemperature Rte_CDataField_OvertemperatureProtector_L2Temperature;
    sdtCurrent Rte_CDataField_OvertemperatureProtector_NominalBatteryCurrent;
    sdtProtectorIndex Rte_CDataField_OvertemperatureProtector_ProtectorIndex;
    sdtCurrent Rte_CDataField_OvertemperatureProtector_TimeBasedDegradationStep;
} OvertemperatureProtector;

/* Datatype for PhaseCurrentProvider */
typedef struct
{
    sdtADCValue Rte_CDataField_PhaseCurrentProvider_HighRawValue;
    sdtADCValue Rte_CDataField_PhaseCurrentProvider_LowRawValue;
    sdtCurrent Rte_CDataField_PhaseCurrentProvider_MaxAbsZeroCurrent;
    sdtCurrent Rte_CDataField_PhaseCurrentProvider_MaxPhaseCurrent;
    float32 Rte_CDataField_PhaseCurrentProvider_PhaseU_ACoeff;
    float32 Rte_CDataField_PhaseCurrentProvider_PhaseU_BCoeff;
    float32 Rte_CDataField_PhaseCurrentProvider_PhaseV_ACoeff;
    float32 Rte_CDataField_PhaseCurrentProvider_PhaseV_BCoeff;
    cdtPhaseCurrents Rte_WriteField_PhaseCurrents_phaseCurrents;
    cdtPhaseCurrents Rte_WriteField_ZeroVectorCurrents_phaseCurrents;
} PhaseCurrentProvider;

/* Datatype for PhaseCurrentProviderHAL */
typedef struct
{
    enCalibrationCommand Rte_PimField_PhaseCurrentProviderHAL_CalibrationCommand;
    float32 Rte_CDataField_PhaseCurrentProviderHAL_CurrentScaleFactor;
    sdtADCValue Rte_CDataField_PhaseCurrentProviderHAL_OOSLimitHigh;
    sdtADCValue Rte_CDataField_PhaseCurrentProviderHAL_OOSLimitLow;
    float32 Rte_CDataField_PhaseCurrentProviderHAL_StartupOffsetLimitHigh;
    float32 Rte_CDataField_PhaseCurrentProviderHAL_StartupOffsetLimitLow;
    cdtMeasuredPhaseCurrents Rte_WriteField_PhaseCurrentsU_currents;
    cdtMeasuredPhaseCurrents Rte_WriteField_PhaseCurrentsV_currents;
    cdtMeasuredPhaseCurrents Rte_WriteField_PhaseCurrentsW_currents;
} PhaseCurrentProviderHAL;

/* Datatype for PIDRegulator */
typedef struct
{
    enPIDCommand Rte_PimField_PIDRegulator_ControlCommand;
    sdtForce Rte_WriteField_DesiredForce_Force;
} PIDRegulator;

/* Datatype for PIDRegulatorParameterProvider */
typedef struct
{
    cdtPidRegulatorParameters Rte_WriteField_PidParameters_Parameters;
} PIDRegulatorParameterProvider;

/* Datatype for PIDVoltageRegulator */
typedef struct
{
    cdtDQVoltage Rte_WriteField_RequiredVoltage_Voltage;
} PIDVoltageRegulator;

/* Datatype for PortsLibrary */
typedef struct
{
} PortsLibrary;

/* Datatype for PWMDriver */
typedef struct
{
    enPwmDriverStatus Rte_PimField_PWMDriver_PwmEnableStatus;
    enBridgeRelayVectorIndex Rte_PimField_PWMDriver_RequiredVector;
    enPwmDriverStatus Rte_WriteField_Status_Status;
} PWMDriver;

/* Datatype for ReferenceVoltageCalculator */
typedef struct
{
    cdtAlphaBetaVector Rte_WriteField_ReferenceVoltage_Vector;
} ReferenceVoltageCalculator;

/* Datatype for ResetHandler */
typedef struct
{
} ResetHandler;

/* Datatype for RuntimeCounterLibrary */
typedef struct
{
} RuntimeCounterLibrary;

/* Datatype for RuntimeMonitor */
typedef struct
{
} RuntimeMonitor;

/* Datatype for SliderAngleProvider */
typedef struct
{
    sdtElectricalAngle Rte_CDataField_SliderAngleProvider_AngleShift;
    sdtADCValue Rte_CDataField_SliderAngleProvider_MaxAdcScattering;
    sdtSliderPositionMm Rte_CDataField_SliderAngleProvider_MaxPositionScatteringMm;
    sdtPhaseLength Rte_CDataField_SliderAngleProvider_PhaseLengthMm;
    sdtSliderPositionMm Rte_CDataField_SliderAngleProvider_SliderSensorShiftMm;
    cdtQualifiedElectricalAngle Rte_WriteField_ElectricalAngle_ElectricalAngle;
    cdtQualifiedSliderPosition Rte_WriteField_SliderPosition_Position;
} SliderAngleProvider;

/* Datatype for SliderPhaseToMmProvider */
typedef struct
{
    cdtQualifiedSliderPosition Rte_WriteField_SliderPosition_Position;
} SliderPhaseToMmProvider;

/* Datatype for SliderSpeedProvider */
typedef struct
{
    cdtQualifiedSliderSpeed Rte_WriteField_SliderSpeed_Speed;
} SliderSpeedProvider;

/* Datatype for SliderTester */
typedef struct
{
    sdtForce Rte_WriteField_DesiredForce_Force;
} SliderTester;

/* Datatype for StartupCode */
typedef struct
{
} StartupCode;

/* Datatype for TemperatureModelMotor */
typedef struct
{
    float32 Rte_CDataField_TemperatureModelMotor_HeatCapacity;
    float32 Rte_CDataField_TemperatureModelMotor_HeatTransferCoeff;
    float32 Rte_CDataField_TemperatureModelMotor_MotorMass;
    float32 Rte_CDataField_TemperatureModelMotor_SurfaceArea;
    cdtQualifiedTemperature Rte_WriteField_CoilTemperature_Temperature;
    cdtQualifiedTemperature Rte_WriteField_MagnetTemperature_Temperature;
} TemperatureModelMotor;

/* Datatype for UartReceiver */
typedef struct
{
} UartReceiver;

/* Datatype for UARTTransmitter */
typedef struct
{
    float32 Rte_PimField_UARTTransmitter_PerInstanceMemory444;
} UARTTransmitter;

/* Datatype for WatchdogHandler */
typedef struct
{
} WatchdogHandler;

/* Datatype for WheelAccelerationProvider */
typedef struct
{
    cdtQualifiedAcceleration Rte_WriteField_Acceleration_Acceleration;
    cdtQualifiedRotation Rte_WriteField_Rotation_Rotation;
} WheelAccelerationProvider;

#endif /* RTE_DATA_TYPES_H */
/* End of file */

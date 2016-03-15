/**
  **************************************************************************************
  * @file    rov2016_canbus.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V1.0
  * @date    3-February-2016
  * @brief   This file contains local variables and macros for rov2016_canbus.c
  **************************************************************************************
  */

/* Include -----------------------------------------------------------------------------*/

/* Private macro -----------------------------------------------------------------------*/
#define CAN_ID_TYPE_STD						0u
#define CAN_ID_TYPE_EXT						4u

/* ID list, RANGE = [0...0x7FF] *********************************************************/

#define TOPSIDE_BASE 					0x200
#define TOPSIDE_JOYSTICK				TOPSIDE_BASE
#define TOPSIDE_COMMANDS				(TOPSIDE_BASE + 1)

#define SENSOR_BASE 					0x300
#define SENSOR_AN_RAW					SENSOR_BASE
#define SENSOR_PROCESSED_DATA			(SENSOR_BASE + 1)
#define SENSOR_ACCELERATION				(SENSOR_BASE + 2)
#define SENSOR_DEPTH_TEMP				(SENSOR_BASE + 3)
#define SENSOR_LEAKAGE_ALARM			(SENSOR_BASE + 4)
#define SENSOR_AHRS_QUATERNIONS			(SENSOR_BASE + 5)

#define POWR_BASE 						0x400
#define POWR_STATUS						POWR_BASE
#define POWR_MOTOR_SWITCH				(POWR_BASE + 1)
#define POWR_LIGHT_SWITCH				(POWR_BASE + 2)
#define POWR_LIGHT_DIMMING 				(POWR_BASE + 3)
#define POWR_COOLING_FAN_SWITCH			(POWR_BASE + 4)
#define POWR_ENCODER_ONE				(POWR_BASE + 5)
#define POWR_ENCODER_TOW				(POWR_BASE + 6)
#define POWR_CAM_PWM					(POWR_BASE + 7)
#define POWR_PWM_MANIP_ONE				(POWR_BASE + 8)
#define POWR_PWM_MANIP_TWO				(POWR_BASE + 9)

#define ESC_BASE						0x500
#define ESC_1							ESC_BASE


/* Standard ID filters ******************************************************************/
#define CAN_RX_FILTER_NONE				0x7FF

/* Filter bank 0 */
#define CAN_RX_FILTER_0					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_1					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_2					SENSOR_PROCESSED_DATA
#define CAN_RX_FILTER_3					CAN_RX_FILTER_NONE

/* Filter bank 1 */
#define CAN_RX_FILTER_4					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_5					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_6					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_7					CAN_RX_FILTER_NONE

/* Filter bank 3 */
#define CAN_RX_FILTER_8					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_9					CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_10				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_11				CAN_RX_FILTER_NONE

/* Filter bank 4 */
#define CAN_RX_FILTER_12				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_13				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_14				CAN_RX_FILTER_NONE
#define CAN_RX_FILTER_15				CAN_RX_FILTER_NONE

/* Extended ID filters ******************************************************************/


/* Exported function prototypes --------------------------------------------------------*/
extern void CAN_init(void);
extern uint8_t CAN_getByteFromMessage(uint8_t filter_number, uint8_t byte_number);
extern uint8_t CAN_getRxMessages(void);
extern void CAN_transmitBuffer(uint32_t Id, uint8_t* buffer, uint8_t length, uint8_t Id_Type);
extern void CAN_transmitByte(uint16_t StdId, uint8_t data);

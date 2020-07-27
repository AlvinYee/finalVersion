/**
 * @file HCM_LCD.c
 * @brief Lock Conditions
 * @details This module checks the lock conditions including E2E protection and message timeouts.
 * @date 11.05.2012
 * @author ruecks
 */
/*[[[cog
print('[INFO]: handling template _HCM_LCD.c starts')
import sys
sys.path.append('../')
sys.path.append('../../')
mq_xml = '../description/Coco.xml'
CocoCfg_h = '../output/CocoCfg.h'
CocoCfg_c = '../output/CocoCfg.c'
import cog
from PkCocoConfigurator.MdCocoConfigurator import CocoConfigurator
print('[INFO]: parsing xml starts')
coco_configurator = CocoConfigurator(mq_xml)
print('[INFO]: parsing xml ends')
print('[INFO]: generate h file starts')
coco_configurator.generate_hfile(CocoCfg_h)
print('[INFO]: generate h file ends')
print('[INFO]: generate c file starts')
coco_configurator.generate_cfile(CocoCfg_c)
print('[INFO]: generate c file ends')
]]]*/
//[[[end]]]
/* Includes */
#include "mq_type.h"
#include "HCM_LCD.h"
#include "HDL_RMH.h"
#include "APP_SDA.h"
#include "DRV_CRC.h"
#include "I2CCAN_Par.h"
#include "HDL_RMH.h"

#define CRC_idx 0u
#define ALIVE_idx 1u
/*[[[cog
for ws in 'wheelSpeed_1', 'wheelSpeed_2':
    cfg = coco_configurator.get_wheel_speed_allocation(ws)
    if cfg[0]:
        cog.outl("#define FIRST_{0}_WHEEL_VALIDITY {1}".format(ws, cfg[0]))
    else:
        cog.outl("#define FIRST_{0}_WHEEL_VALIDITY NULL".format(ws))
    if cfg[1]:
        cog.outl("#define FIRST_{0}_WHEEL {1}".format(ws, cfg[1]))
    else:
        cog.outl("#define FIRST_{0}_WHEEL NULL".format(ws))
    cog.outl("#define {0}_VALID_QUANTITY {1}".format(ws, cfg[2]))
    cog.outl("#define {0}_WHEEL_QUANTITY {1}".format(ws, cfg[3]))
]]]*/
//[[[end]]]

/* polyspace<MISRA-C:12.5:not a defect:no action planned> Expression is OK without parentheses*/
#if wheelSpeed_1_VALID_QUANTITY > 0 || wheelSpeed_2_VALID_QUANTITY > 0
static uint8 HcmLcdWhlSpdVldFlag[wheelSpeed_1_VALID_QUANTITY+wheelSpeed_2_VALID_QUANTITY];
#else
static uint8 HcmLcdWhlSpdVldFlag[wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY];
#endif

/* Global variables */
uint8 HCM_LCD_can_msg_crc_failure;			    /* crc failures of the received CAN messages */
uint8 HCM_LCD_can_msg_timeout_failure;		    /* timeout failures of the received CAN messages */
uint8 HCM_LCD_can_msg_alive_cnt_failure;	    /* alive cnt failures of the received CAN messages */
uint16 HCM_LCD_can_msg_lcd_sync[msgNum];	    /* msg Lcd synchronization  */
uint16 HCM_LCD_can_msg_lcd_inv_sync[msgNum];	/* msg Lcd synchronization  */
bool  SWSW_coco_timeout_check_called;		    /* information if timeout handler was called */

static uint16	HcmLcdMsgTimeoutCnt[msgNum];
static uint16 	HcmLcdLastAliveCnt[msgNum];

#if	wheelSpeed_1_msgEnable_cfg > FALSE
#if HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD_ON_CHANGE > FALSE
static uint16 	HcmLcdLastWheelSpeed1[wheelSpeed_1_WHEEL_QUANTITY];
#endif
#endif
#if	wheelSpeed_2_msgEnable_cfg > FALSE
#if HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD_ON_CHANGE > FALSE
static uint16 	HcmLcdLastWheelSpeed2[wheelSpeed_2_WHEEL_QUANTITY];
#endif
#endif
static uint16	HCM_LCD_timeout_cnt_haco_status_u16;	/* timeout counter for HaCo status message */

static void HCM_LCD_timeout_handler(void);
static void HCM_LCD_evaluate_received_msgs(void);
static void HCM_LCD_evaluate_received_msgs_qual(void);
static void HCM_LCD_check_haco_status_msg(void);


/**
 * @brief Initialization function
 * @details
 * no message ESP_10 received yet, timeout initially set
 * initially set E2E error in ESP_10
 * the lock condition wheel signals isn't fulfilled (SDA)
 * reset qual time counter wheel signals in SDA
 * the lock condition HaCo status message isn't fulfilled (SDA)
 * reset timeout counter ESP_10 (timeout reached)
 * reset timeout counter HaCo status message (timeout reached)
 * timeout handler not yet called
 * initialize buffer for wheel signal front left in SDA
 * initialize buffer for wheel signal front right in SDA
 * initialize buffer for wheel signal rear left in SDA
 * initialize buffer for wheel signal rear right in SDA
 * set message counter ESP_10 to it's initial value
 * @ReqKey MOD_HCM_LCD-10 MOD_HCM_LCD-87 MOD_HCM_LCD-88 MOD_HCM_LCD-89 MOD_HCM_LCD-90 MOD_HCM_LCD-19
 * @ReqKey MOD_HCM_LCD-105 MOD_HCM_LCD-42 MOD_HCM_LCD-249 MOD_HCM_LCD-137 MOD_HCM_LCD-138 MOD_HCM_LCD-139
 * @ReqKey MOD_HCM_LCD-140 MOD_HCM_LCD-170
 */
void HCM_LCD_init(void)
{

	uint8 msgIdx = 0;
	uint8 whlSpdVldIdx = 0;
	uint8 whlSpdIdx = 0;

	/* no message  received yet, timeout initially set */
	for(msgIdx = 0; msgIdx < msgNum; msgIdx++)
	{
		/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
		HCM_LCD_can_msg_timeout_failure 	|= (0x01 << msgIdx); 
		/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
		HCM_LCD_can_msg_crc_failure 		|= (0x01 << msgIdx);
		/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
		HCM_LCD_can_msg_alive_cnt_failure 	|= (0x01 << msgIdx);
        HCM_LCD_can_msg_lcd_sync[msgIdx] = RESET;         
        HCM_LCD_can_msg_lcd_inv_sync[msgIdx] = RESET;

		HcmLcdMsgTimeoutCnt[msgIdx] = RESET;
		HcmLcdLastAliveCnt[msgIdx] = 0xffu;
	}
	/* polyspace<MISRA-C:12.4:not a defect:no action planned> Expression is OK without parentheses*/
#if wheelSpeed_1_VALID_QUANTITY > 0 || wheelSpeed_2_VALID_QUANTITY > 0
	for(whlSpdVldIdx = 0; whlSpdVldIdx < wheelSpeed_1_VALID_QUANTITY+wheelSpeed_2_VALID_QUANTITY; whlSpdVldIdx++)
	{
	    HcmLcdWhlSpdVldFlag[whlSpdVldIdx] = RESET;
	}
#endif
#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
	for(whlSpdIdx = 0; whlSpdIdx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; whlSpdIdx++)
	{
	    HcmLcdWhlSpdVldFlag[whlSpdIdx] = RESET;
	}
#endif
	#if wheelSpeed_1_msgEnable_cfg > FALSE
	/* the lock condition wheel signals isn't fulfilled (SDA) */
	SWSW_coco_sda_lcdwheelspeed1		= APP_SDA_LcdWheelSpeed1_FALSE;
	/* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
	SWSW_coco_sda_lcdwheelspeed1_inv	= INV_UINT16(APP_SDA_LcdWheelSpeed1_FALSE);
    #if wheelSpeed_1_qualCheck_cfg > FALSE
    /* reset qual time counter wheel signals in SDA */
    SWSW_coco_sda_lcdqualcntwheelspeed1		= (RESET | APP_SDA_LcdQualCntWheelSpeed1_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_lcdqualcntwheelspeed1_inv	= INV_UINT16(RESET | APP_SDA_LcdQualCntWheelSpeed1_ID);
    #endif
    #if WHEEL_SPEED_1_VALUE_FR_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedrr        = ((uint16)WHEEL_SPEED_1_VALUE_FR_initValue_cfg | APP_SDA_BufferWheelSpeedFR_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedrr_inv    = INV_UINT16((uint16)WHEEL_SPEED_1_VALUE_FR_initValue_cfg | APP_SDA_BufferWheelSpeedFR_ID);
    #endif
    #if WHEEL_SPEED_1_VALUE_FL_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedfl        = ((uint16)WHEEL_SPEED_1_VALUE_FL_initValue_cfg | APP_SDA_BufferWheelSpeedFL_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedfl_inv    = INV_UINT16((uint16)WHEEL_SPEED_1_VALUE_FL_initValue_cfg | APP_SDA_BufferWheelSpeedFL_ID);
    #endif
    #if WHEEL_SPEED_1_VALUE_RR_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedrr        = ((uint16)WHEEL_SPEED_1_VALUE_RR_initValue_cfg | APP_SDA_BufferWheelSpeedRR_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedrr_inv    = INV_UINT16((uint16)WHEEL_SPEED_1_VALUE_RR_initValue_cfg | APP_SDA_BufferWheelSpeedRR_ID);
    #endif
    #if WHEEL_SPEED_1_VALUE_RL_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedrl        = ((uint16)WHEEL_SPEED_1_VALUE_RL_initValue_cfg | APP_SDA_BufferWheelSpeedRL_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedrl_inv    = INV_UINT16((uint16)WHEEL_SPEED_1_VALUE_RL_initValue_cfg | APP_SDA_BufferWheelSpeedRL_ID);
    #endif
	#endif

	#if wheelSpeed_2_msgEnable_cfg > FALSE
	/* the lock condition wheel signals isn't fulfilled (SDA) */
	SWSW_coco_sda_lcdwheelspeed2		= APP_SDA_LcdWheelSpeed2_FALSE;
	/* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
	SWSW_coco_sda_lcdwheelspeed2_inv	= INV_UINT16(APP_SDA_LcdWheelSpeed2_FALSE);
    #if wheelSpeed_2_qualCheck_cfg > FALSE
    /* reset qual time counter wheel signals in SDA */
    SWSW_coco_sda_lcdqualcntwheelspeed2		= (RESET | APP_SDA_LcdQualCntWheelSpeed2_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_lcdqualcntwheelspeed2_inv	= INV_UINT16(RESET | APP_SDA_LcdQualCntWheelSpeed2_ID);
    #endif
    #if WHEEL_SPEED_2_VALUE_FR_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedrr        = ((uint16)WHEEL_SPEED_2_VALUE_FR_initValue_cfg | APP_SDA_BufferWheelSpeedFR_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedrr_inv    = INV_UINT16((uint16)WHEEL_SPEED_2_VALUE_FR_initValue_cfg | APP_SDA_BufferWheelSpeedFR_ID);
    #endif
    #if	WHEEL_SPEED_2_VALUE_FL_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedfl		= ((uint16)WHEEL_SPEED_2_VALUE_FL_initValue_cfg | APP_SDA_BufferWheelSpeedFL_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedfl_inv	= INV_UINT16((uint16)WHEEL_SPEED_2_VALUE_FL_initValue_cfg | APP_SDA_BufferWheelSpeedFL_ID);
    #endif
    #if WHEEL_SPEED_2_VALUE_RR_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedrr        = ((uint16)WHEEL_SPEED_2_VALUE_RR_initValue_cfg | APP_SDA_BufferWheelSpeedRR_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedrr_inv    = INV_UINT16((uint16)WHEEL_SPEED_2_VALUE_RR_initValue_cfg | APP_SDA_BufferWheelSpeedRR_ID);
    #endif
    #if	WHEEL_SPEED_2_VALUE_RL_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferwheelspeedrl		= ((uint16)WHEEL_SPEED_2_VALUE_RL_initValue_cfg | APP_SDA_BufferWheelSpeedRL_ID);
    /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
    SWSW_coco_sda_bufferwheelspeedrl_inv	= INV_UINT16((uint16)WHEEL_SPEED_2_VALUE_RL_initValue_cfg | APP_SDA_BufferWheelSpeedRL_ID);
    #endif
	#endif

	#if powerMode_msgEnable_cfg > FALSE
	SWSW_coco_sda_lcdpowermode						  = APP_SDA_LcdPowerMode_FALSE;
	SWSW_coco_sda_lcdpowermode_inv					  = INV_UINT16(APP_SDA_LcdPowerMode_FALSE);
    #if powerMode_qualCheck_cfg > FALSE
    SWSW_coco_sda_lcdqualcntpowermode 		          = (RESET | APP_SDA_LcdQualCntPowerMode_ID);
    SWSW_coco_sda_lcdqualcntpowermode_inv 	          = INV_UINT16(RESET | APP_SDA_LcdQualCntPowerMode_ID);
    #endif
    #if POWER_MODE_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferpowermode                     = ((uint16)POWER_MODE_initValue_cfg | APP_SDA_BufferPowerMode_ID);
    SWSW_coco_sda_bufferpowermode_inv                 = INV_UINT16((uint16)POWER_MODE_initValue_cfg | APP_SDA_BufferPowerMode_ID);
    #endif
    #if IGNITION_STATE_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferignitionstate 	              = ((uint16)IGNITION_STATE_initValue_cfg | APP_SDA_BufferIgnitionState_ID);
    SWSW_coco_sda_bufferignitionstate_inv             = INV_UINT16((uint16)IGNITION_STATE_initValue_cfg | APP_SDA_BufferIgnitionState_ID);
    #endif
    #if ENGINE_STATE_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferenginestate                   = ((uint16)ENGINE_STATE_initValue_cfg | APP_SDA_BufferEngineState_ID);
    SWSW_coco_sda_bufferenginestate_inv               = INV_UINT16((uint16)ENGINE_STATE_initValue_cfg | APP_SDA_BufferEngineState_ID);
    #endif
    #endif
	
	#if vehicleSpeed_msgEnable_cfg > FALSE
	SWSW_coco_sda_lcdvehiclespeed			          = APP_SDA_LcdVehicleSpeed_FALSE;
	SWSW_coco_sda_lcdvehiclespeed_inv		          = INV_UINT16(APP_SDA_LcdVehicleSpeed_FALSE);
    #if vehicleSpeed_qualCheck_cfg > FALSE
    SWSW_coco_sda_lcdqualcntvehiclespeed 	  		  = (RESET | APP_SDA_LcdQualCntVehicleSpeed_ID);
    SWSW_coco_sda_lcdqualcntvehiclespeed_inv          = INV_UINT16(RESET | APP_SDA_LcdQualCntVehicleSpeed_ID);
    #endif
    #if VEHICLE_SPEED_sigEnable_cfg > FALSE
    SWSW_coco_sda_buffervehiclespeed				  = ((uint16)VEHICLE_SPEED_initValue_cfg | APP_SDA_BufferVehicleSpeed_ID);
    SWSW_coco_sda_buffervehiclespeed_inv    		  = INV_UINT16((uint16)VEHICLE_SPEED_initValue_cfg | APP_SDA_BufferVehicleSpeed_ID);
    #endif
    #if ENGINE_SPEED_sigEnable_cfg > FALSE
    SWSW_coco_sda_bufferenginespeed				      = ((uint16)ENGINE_SPEED_initValue_cfg | APP_SDA_BufferEngineSpeed_ID);
    SWSW_coco_sda_bufferenginespeed_inv    		      = INV_UINT16((uint16)ENGINE_SPEED_initValue_cfg | APP_SDA_BufferEngineSpeed_ID);
    #endif
	#endif
	
	/* the lock condition HaCo status message isn't fulfilled (SDA) */
	SWSW_coco_sda_lcdhacostatus		= APP_SDA_LcdHacoStatus_FALSE;
	/* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
	SWSW_coco_sda_lcdhacostatus_inv	= INV_UINT16(APP_SDA_LcdHacoStatus_FALSE);
	/* reset timeout counter HaCo status message (timeout reached) */
	HCM_LCD_timeout_cnt_haco_status_u16 = RESET;
	/* timeout handler not yet called */
	SWSW_coco_timeout_check_called = FALSE;
}

/**
 * @brief This function handles the cyclic functionality of the lock conditions module.
 * @details
 * call function for timeout handling
 * call function for ESP_10
 * call function for qual of the lock condition wheel signals
 * call function for HaCo status message
 * @ReqKey MOD_HCM_LCD-86 MOD_HCM_LCD-84 MOD_HCM_LCD-106 MOD_HCM_LCD-160 MOD_HCM_LCD-83
 */
void HCM_LCD_cyclic(void)
{
	HCM_LCD_timeout_handler();
	/* call function for timeout handling, has to be called before the functions that handle the received messages to get the correct timeout times */
	HCM_LCD_evaluate_received_msgs();
	HCM_LCD_evaluate_received_msgs_qual();
	/* call function for HaCo status message */
	HCM_LCD_check_haco_status_msg();
}

/**
 * @brief This function checks the lock conditions contained in ESP_10.
 * @details
 * local variables
 * message ESP_10 received?
 *   message received in time?
 *     no timeout of ESP_10
 *   restart the timeout counter
 *   extract message counter
 *   E2E calculation except last byte
 *   E2E calculation last byte (Kennung)
 *   E2E checksum correct?
 *     for all wheel signals
 *       set the variables according the chosen wheel signal
 *         wheel signal front left
 *           select SDA variables
 *           select SDA ID
 *           select the variable for the received wheel signal
 *           select the bytes from the message buffer that contain the wheel signal
 *           set the distance that the wheel signal has to be shifted to fit into the word variable
 *           select the bit mask for the valid signal
 *         wheel signal front right
 *           select SDA variables
 *           select SDA ID
 *           select the variable for the received wheel signal
 *           select the bytes from the message buffer that contain the wheel signal
 *           set the distance that the wheel signal has to be shifted to fit into the word variable
 *           select the bit mask for the valid signal
 *         wheel signal rear left
 *           select SDA variables
 *           select SDA ID
 *           select the variable for the received wheel signal
 *           select the bytes from the message buffer that contain the wheel signal
 *           set the distance that the wheel signal has to be shifted to fit into the word variable
 *           select the bit mask for the valid signal
 *         wheel signal rear right
 *           select SDA variables
 *           select SDA ID
 *           select the variable for the received wheel signal
 *           select the bytes from the message buffer that contain the wheel signal
 *           set the distance that the wheel signal has to be shifted to fit into the word variable
 *           select the bit mask for the valid signal
 *         no wheel signal selected
 *           can't be reached as the for loop is limited to the number of wheel signals --> do nothing
 *       inversion is fulfilled and
 *       ID belongs to buffer wheel signal?
 *         use the buffer wheel signal
 *       SDA entry of buffer wheel signal invalid?
 *         use default value for buffer wheel signal
 *         set SDA error in SDA
 *       get lower part of wheel signal from message buffer
 *       get higher part of wheel signal from message buffer
 *       shift wheel signal so that it fits into the word variable
 *       mask the bits that don't belong to wheel signal
 *       wheel signal valid and
 *       last received wheel signal in valid range and
 *       wheel signal in valid range?
 *         wheel signal greater than or equal to last received wheel signal?
 *           calculate difference between wheel signal and old value
 *         wheel signal less than last received wheel signal?
 *           calculate difference from old value to end of range
 *           add new value
 *         difference within the range that fulfills the lock condition?
 *           increment counter of wheel signals that fulfill the condition
 *     buffer value message counter in range?
 *       preparation for calculating the difference
 *       received message counter less than buffer value?
 *         if the message counter is valid, an overflow must have happened, undo it by adding the range
 *       calculate the difference to the last received value
 *       difference within specified tolerance?
 *         no End2End error
 *         ESP_10 received in time and
 *         all wheel signals fulfill the lock condition?
 *           the lock condition wheel signals is fulfilled (SDA)
 *         timing violation ESP_10 or
 *         at least one wheel signal doesn't fulfill the lock condition?
 *           the lock condition wheel signals isn't fulfilled (SDA)
 *       difference of message counter violates the tolerance?
 *         End2End error occurred
 *         the lock condition wheel signals isn't fulfilled (SDA)
 *     buffer value message counter out of range?
 *       End2End error occurred
 *       the lock condition wheel signals isn't fulfilled (SDA)
 *     checksum correct, the received wheel signals should be stored
 *     wheel signal front left in SDA
 *     wheel signal front right in SDA
 *     wheel signal rear left in SDA
 *     wheel signal rear right in SDA
 *     store message counter
 *   E2E checksum not correct?
 *     End2End error occurred
 *     the lock condition wheel signals isn't fulfilled (SDA)
 *     checksum not correct, discard received wheel signals, init values should be stored
 *     wheel signal front left in SDA
 *     wheel signal front right in SDA
 *     wheel signal rear left in SDA
 *     wheel signal rear right in SDA
 *   interpretation of ESP_10 finished, clear indication flag
 * @ReqKey MOD_HCM_LCD-94 MOD_HCM_LCD-95 MOD_HCM_LCD-96 MOD_HCM_LCD-97 MOD_HCM_LCD-113 MOD_HCM_LCD-206
 * @ReqKey MOD_HCM_LCD-114 MOD_HCM_LCD-200 MOD_HCM_LCD-201 MOD_HCM_LCD-203 MOD_HCM_LCD-204 MOD_HCM_LCD-205
 * @ReqKey MOD_HCM_LCD-219 MOD_HCM_LCD-220 MOD_HCM_LCD-222 MOD_HCM_LCD-223 MOD_HCM_LCD-224 MOD_HCM_LCD-229
 * @ReqKey MOD_HCM_LCD-230 MOD_HCM_LCD-232 MOD_HCM_LCD-233 MOD_HCM_LCD-234 MOD_HCM_LCD-239 MOD_HCM_LCD-240
 * @ReqKey MOD_HCM_LCD-242 MOD_HCM_LCD-243 MOD_HCM_LCD-244 MOD_HCM_LCD-246 MOD_HCM_LCD-119 MOD_HCM_LCD-121
 * @ReqKey MOD_HCM_LCD-125 MOD_HCM_LCD-126 MOD_HCM_LCD-127 MOD_HCM_LCD-128 MOD_HCM_LCD-129 MOD_HCM_LCD-141
 * @ReqKey MOD_HCM_LCD-142 MOD_HCM_LCD-149 MOD_HCM_LCD-143 MOD_HCM_LCD-144 MOD_HCM_LCD-150 MOD_HCM_LCD-145
 * @ReqKey MOD_HCM_LCD-146 MOD_HCM_LCD-151 MOD_HCM_LCD-147 MOD_HCM_LCD-148 MOD_HCM_LCD-152 MOD_HCM_LCD-153
 * @ReqKey MOD_HCM_LCD-158 MOD_HCM_LCD-209 MOD_HCM_LCD-210 MOD_HCM_LCD-154 MOD_HCM_LCD-155 MOD_HCM_LCD-156
 * @ReqKey MOD_HCM_LCD-157 MOD_HCM_LCD-130 MOD_HCM_LCD-122 MOD_HCM_LCD-123 MOD_HCM_LCD-211 MOD_HCM_LCD-212
 * @ReqKey MOD_HCM_LCD-213 MOD_HCM_LCD-214 MOD_HCM_LCD-215 MOD_HCM_LCD-225 MOD_HCM_LCD-235 MOD_HCM_LCD-245
 * @ReqKey MOD_HCM_LCD-98
 */
/* polyspace<MISRA-C:3.4:Not a defect:No action planned> pragma to change optimization focus for performance reasons */
#pragma optimize=speed

static void HCM_LCD_evaluate_received_msgs(void)
{
	uint8   msgIdx		    = 0;
	uint8   signalIdx 	    = 0;
	uint8   whlSpdVldIdx    = 0;
	uint8   whlSpdIdx       = 0;
	uint8   NumOfWhlVld     = 0;
	uint8   wheel1Idx 	    = FIRST_wheelSpeed_1_WHEEL;
	uint8   wheel1VldIdx    = FIRST_wheelSpeed_1_WHEEL_VALIDITY;
	uint8   wheel2Idx 	    = FIRST_wheelSpeed_2_WHEEL;
	uint8   wheel2VldIdx    = FIRST_wheelSpeed_2_WHEEL_VALIDITY;
	bool    tempWheelSpeedLcd = FALSE;
	uint16  tempCrcResult   = 0;
	uint16  currentAliveCnt = 0;
	uint16  aliveCntDiff    = 0;
	#if HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD_ON_CHANGE > FALSE
	uint16  wheelSpeedDiff  = 0xffff;
	#endif
	uint16  currentWheelSpeedValid = HCM_LCD_SIGNAL_INVALID_VALUE_MAPPING;
	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
	uint16  currentWheelSpeed      = WHEEL_SPEED_1_VALUE_FR_initValue_cfg;

	MSG_CONFIG_Tag const *p_msg = (MSG_CONFIG_Tag const *) 0;
	SIGNAL_CONFIG_Tag const *p_sig = (SIGNAL_CONFIG_Tag const *) 0;

    #if POWER_MODE_VALIDITY_sigEnable_cfg > FALSE
	uint16  powerModeValid         = HCM_LCD_SIGNAL_INVALID_VALUE_MAPPING;
	#endif
    #if POWER_MODE_sigEnable_cfg > FALSE
	uint16  powerMode              = POWER_MODE_initValue_cfg;
	#endif
	#if IGNITION_STATE_VALIDITY_sigEnable_cfg > FALSE
	uint16  ignitionStateValid     = HCM_LCD_SIGNAL_INVALID_VALUE_MAPPING;
	#endif
	#if IGNITION_STATE_sigEnable_cfg > FALSE
	uint16  ignitionState          = IGNITION_STATE_initValue_cfg;
	#endif
	#if ENGINE_STATE_VALIDITY_sigEnable_cfg > FALSE
	uint16  engineStateValid       = HCM_LCD_SIGNAL_INVALID_VALUE_MAPPING;
	#endif
	#if ENGINE_STATE_sigEnable_cfg > FALSE
	uint16  engineState            = ENGINE_STATE_initValue_cfg;
	#endif
    #if VEHICLE_SPEED_VALIDITY_sigEnable_cfg > FALSE
	uint16  vehicleSpeedValid      = HCM_LCD_SIGNAL_INVALID_VALUE_MAPPING;
	#endif
	#if VEHICLE_SPEED_sigEnable_cfg > FALSE
	uint16  vehicleSpeed           = VEHICLE_SPEED_initValue_cfg;
    #endif
    #if ENGINE_SPEED_VALIDITY_sigEnable_cfg > FALSE
	uint16  engineSpeedValid       = HCM_LCD_SIGNAL_INVALID_VALUE_MAPPING;
    #endif
    #if ENGINE_SPEED_sigEnable_cfg > FALSE
	uint16  engineSpeed            = ENGINE_SPEED_initValue_cfg;
    #endif

	for(msgIdx = 0; msgIdx < msgNum; msgIdx++)
	{
	    p_msg = &CocoConfigurations[msgIdx];
		/* msg received? */
		/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise & is OK*/
		/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
		if(FALSE < (HDL_RMH_msg_ind_flags_u8.c & (0x01 << msgIdx)))
		{
		    /* clear msg indication bit */
			/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
			/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise &= is OK*/
			/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
			/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise ~ is OK*/
		    HDL_RMH_msg_ind_flags_u8.c &= ~(0x01 << msgIdx);
            
			/* once received, set SDA LCD flag to false, which is preventing locking */
			/* SDA LCD flag will be set at the end where all checks have been passed */
            *p_msg->msgLcdSda    = p_msg->msgLcdSdaFalse;
            *p_msg->msgLcdSdaInv = INV_UINT16(p_msg->msgLcdSdaFalse);
            
			/* CHECK 1 - timeout OK ? no longer configurable */
            /* received in time? */
            if(HcmLcdMsgTimeoutCnt[msgIdx] > RESET)
            {
                /* clear error flag */
            	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
            	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise &= is OK*/
            	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
            	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise ~ is OK*/
                HCM_LCD_can_msg_timeout_failure &= ~(0x01 << msgIdx);
                HcmLcdMsgTimeoutCnt[msgIdx] = p_msg->timeoutValue/2;
            }
            else
            {
                HcmLcdMsgTimeoutCnt[msgIdx] = p_msg->timeoutValue/2;
                /* EXIT 1: timeout */
                /* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is Ok to use continue without any risk*/
                continue;
            }
            
			/* update all signals from signal-access macro */
            switch(msgIdx)
            {
				#if	wheelSpeed_1_msgEnable_cfg > FALSE
                case wheelSpeed_1_idx:
				{
/*[[[cog
ss = ''
ss = coco_configurator.generate_signal_updating('wheelSpeed_1')
ss = ss.strip('!')
ss = ss.split('!')
for s in ss:
    cog.outl('\t'*4+s)
]]]*/
//[[[end]]]
					break;
				}
				#endif
				
				#if	wheelSpeed_2_msgEnable_cfg > FALSE
                case wheelSpeed_2_idx:
				{
/*[[[cog
ss = ''
ss = coco_configurator.generate_signal_updating('wheelSpeed_2')
ss = ss.strip('!')
ss = ss.split('!')
for s in ss:
    cog.outl('\t'*4+s)
]]]*/
//[[[end]]]
					break;
				}
				#endif
				
				#if	powerMode_msgEnable_cfg > FALSE
                case powerMode_idx:
				{
/*[[[cog
ss = ''
ss = coco_configurator.generate_signal_updating('powerMode')
ss = ss.strip('!')
ss = ss.split('!')
for s in ss:
    cog.outl('\t'*4+s)
]]]*/
//[[[end]]]
					break;
				}
				#endif
				#if	vehicleSpeed_msgEnable_cfg > FALSE
                case vehicleSpeed_idx:
				{
/*[[[cog
ss = ''
ss = coco_configurator.generate_signal_updating('vehicleSpeed')
ss = ss.strip('!')
ss = ss.split('!')
for s in ss:
    cog.outl('\t'*4+s)
]]]*/
//[[[end]]]
					break;
				}
				#endif
                default: break;
            }
             /* CHECK 2 - CRC OK ?*/
			if (FALSE < p_msg->CRCcheck)
			{
                #if SAE_J1850_CRC8_USED > FALSE
				if (p_msg->crcOffset == 0u)
				{
					tempCrcResult = Crc8_Wrapper(p_msg->actualMsg+1,p_msg->msgLength-1);
				}
				else
				{
					tempCrcResult = Crc8_Wrapper(p_msg->actualMsg,p_msg->crcOffset);
				}
                #else
				if (p_msg->crcOffset == 0u)
				{
					tempCrcResult = Crc8_Wrapper((p_msg->actualMsg)+1,p_msg->msgLength-1,CRC_START_VALUE);
				}
				else
				{
					tempCrcResult = Crc8_Wrapper(p_msg->actualMsg,p_msg->crcOffset,CRC_START_VALUE);
				}
                #endif
				p_sig = &p_msg->signalConfig[CRC_idx];
                if (tempCrcResult == *p_sig->actualSig)
                {
                	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise &= is OK*/
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise ~ is OK*/
                	HCM_LCD_can_msg_crc_failure &= ~(0x01 << msgIdx);
                }
                else
                {
                	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
                	HCM_LCD_can_msg_crc_failure |= (0x01 << msgIdx);
        			/* follow GEN3 logic, discard received signals, init values should be stored */
        			for(signalIdx = 0;signalIdx < (sizeof(p_msg->signalConfig) / sizeof(SIGNAL_CONFIG_Tag)); signalIdx++ )
        			{
        			    p_sig = &p_msg->signalConfig[signalIdx];
                        if((p_sig->sigSdaStorage != NUL) && (p_sig->sigSdaStorageInv != NUL))
                        {
                            *p_sig->sigSdaStorage = p_sig->initValue | p_sig->sigSdaID;
                            *p_sig->sigSdaStorageInv = INV_UINT16(p_sig->initValue | p_sig->sigSdaID);
                        }
        			}                 
                    /* EXIT 2: CRC fail */
					/* M328401_CR_72 start */
					#if	wheelSpeed_1_msgEnable_cfg > FALSE
					if(msgIdx == wheelSpeed_1_idx)
					{
						#if wheelSpeed_1_VALID_QUANTITY > 0
						for(wheel1VldIdx=0; wheel1VldIdx < wheelSpeed_1_VALID_QUANTITY; wheel1VldIdx++)
						{
							HcmLcdWhlSpdVldFlag[wheel1VldIdx] = RESET;
						}				
						#endif
						
						#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
						for(wheel1Idx = 0; wheel1Idx < wheelSpeed_1_WHEEL_QUANTITY; wheel1Idx++)
						{
							HcmLcdWhlSpdVldFlag[wheel1Idx] = RESET;
						}
						#endif								
					}
					#endif
					
					#if	wheelSpeed_2_msgEnable_cfg > FALSE
					if(msgIdx == wheelSpeed_2_idx)
					{
						#if wheelSpeed_2_VALID_QUANTITY > 0
						/* polyspace<MISRA-C:12.1:not a defect:no action planned>  Condition is OK without parentheses*/
						for(wheel2VldIdx=wheelSpeed_1_VALID_QUANTITY; wheel2VldIdx < wheelSpeed_1_VALID_QUANTITY + wheelSpeed_2_VALID_QUANTITY; wheel2VldIdx++)
						{
							HcmLcdWhlSpdVldFlag[wheel2VldIdx] = RESET;
						}
						#endif
						
						#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
						for(wheel2Idx = wheelSpeed_1_WHEEL_QUANTITY; wheel2Idx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; wheel2Idx++)
						{
							HcmLcdWhlSpdVldFlag[wheel2Idx] = RESET;
						}
						#endif							
					}
					#endif
					/* M328401_CR_72 end */ 
					/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is Ok to use continue without any risk*/
                    continue;
                }
			}
            
			/* CHECK 3 - ALIVE CNT OK? */
			if (FALSE < p_msg->aliveCntCheck)
			{
                if(HcmLcdLastAliveCnt[msgIdx] <= p_msg->aliveCntMax)
                {
                    if(FALSE < p_msg->CRCcheck)// if crc is enabled, alive cnt idx is 1, otherwise 0
                    {
                        p_sig = &p_msg->signalConfig[ALIVE_idx];
                    }
                    else
                    {
                        p_sig = &p_msg->signalConfig[CRC_idx];
                    }
                    currentAliveCnt = *p_sig->actualSig;
                    aliveCntDiff    = currentAliveCnt;
                    if (currentAliveCnt < HcmLcdLastAliveCnt[msgIdx])
                    {
                        /* if the message counter is valid, an overflow must have happened, undo it by adding the range */
                        aliveCntDiff += (p_msg->aliveCntMax+1);
                    }
                    /* calculate the difference to the last received value */
                    aliveCntDiff -= HcmLcdLastAliveCnt[msgIdx];
                    /* store alive cnt */
                    HcmLcdLastAliveCnt[msgIdx] = currentAliveCnt;
                    if((aliveCntDiff >= p_msg->aliveCntIncMin) && (aliveCntDiff <= p_msg->aliveCntIncMax))
                    {
                    	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
                    	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise &= is OK*/
                    	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
                    	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise ~ is OK*/
                    	HCM_LCD_can_msg_alive_cnt_failure &= ~(0x01 << msgIdx);
                    }
                    else
                    {
                    	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
                    	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
                    	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
                    	HCM_LCD_can_msg_alive_cnt_failure |= (0x01 << msgIdx);
                        /* follow GEN3 logic, if crc pass but alive cnt check fails, still update signal to SDA */
                        for(signalIdx = 0;signalIdx < (sizeof(p_msg->signalConfig) / sizeof(SIGNAL_CONFIG_Tag)); signalIdx++ )
                        {
                            p_sig = &p_msg->signalConfig[signalIdx];
                            if((p_sig->sigSdaStorage != NUL) && (p_sig->sigSdaStorageInv != NUL))
                            {
                                *p_sig->sigSdaStorage = *p_sig->actualSig | p_sig->sigSdaID;
                                *p_sig->sigSdaStorageInv = INV_UINT16(*p_sig->actualSig | p_sig->sigSdaID);
                            }
                        }
                        /* EXIT 3: ALIVE CNT fail */
						/* M328401_CR_72 start */
						#if	wheelSpeed_1_msgEnable_cfg > FALSE
						if(msgIdx == wheelSpeed_1_idx)
						{
							#if wheelSpeed_1_VALID_QUANTITY > 0
							for(wheel1VldIdx=0; wheel1VldIdx < wheelSpeed_1_VALID_QUANTITY; wheel1VldIdx++)
							{
								HcmLcdWhlSpdVldFlag[wheel1VldIdx] = RESET;
							}				
							#endif
							
							#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
							for(wheel1Idx = 0; wheel1Idx < wheelSpeed_1_WHEEL_QUANTITY; wheel1Idx++)
							{
								HcmLcdWhlSpdVldFlag[wheel1Idx] = RESET;
							}
							#endif						
						}
						#endif

						
						#if	wheelSpeed_2_msgEnable_cfg > FALSE
						if(msgIdx == wheelSpeed_2_idx)
						{
							#if wheelSpeed_2_VALID_QUANTITY > 0
							/* polyspace<MISRA-C:12.1:not a defect:no action planned>  Condition is OK without parentheses*/
							for(wheel2VldIdx=wheelSpeed_1_VALID_QUANTITY; wheel2VldIdx < wheelSpeed_1_VALID_QUANTITY + wheelSpeed_2_VALID_QUANTITY; wheel2VldIdx++)
							{
								HcmLcdWhlSpdVldFlag[wheel2VldIdx] = RESET;
							}
							#endif
							
							#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
							for(wheel2Idx = wheelSpeed_1_WHEEL_QUANTITY; wheel2Idx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; wheel2Idx++)
							{
								HcmLcdWhlSpdVldFlag[wheel2Idx] = RESET;
							}
							#endif
						}
						#endif
						/* M328401_CR_72 end */
						/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                        continue;
                    }
                }
                else
                {
                	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
                	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
                	HCM_LCD_can_msg_alive_cnt_failure |= (0x01 << msgIdx);
                    /* store alive cnt */
                    HcmLcdLastAliveCnt[msgIdx] = currentAliveCnt;
                    /* follow GEN3 logic, if crc pass but alive cnt check fails, still update signal to SDA */
                    for(signalIdx = 0;signalIdx < (sizeof(p_msg->signalConfig) / sizeof(SIGNAL_CONFIG_Tag)); signalIdx++ )
                    {
                        p_sig = &p_msg->signalConfig[signalIdx];
                        if((p_sig->sigSdaStorage != NUL) && (p_sig->sigSdaStorageInv != NUL))
                        {
                            *p_sig->sigSdaStorage = *p_sig->actualSig | p_sig->sigSdaID;
                            *p_sig->sigSdaStorageInv = INV_UINT16(*p_sig->actualSig | p_sig->sigSdaID);
                        }
                    }
                    /* EXIT 3: ALIVE CNT  fail */
					/* M328401_CR_72 start */
					#if	wheelSpeed_1_msgEnable_cfg > FALSE
					if(msgIdx == wheelSpeed_1_idx)
					{
						#if wheelSpeed_1_VALID_QUANTITY > 0
						for(wheel1VldIdx=0; wheel1VldIdx < wheelSpeed_1_VALID_QUANTITY; wheel1VldIdx++)
						{
							HcmLcdWhlSpdVldFlag[wheel1VldIdx] = RESET;
						}				
						#endif				
						#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
						for(wheel1Idx = 0; wheel1Idx < wheelSpeed_1_WHEEL_QUANTITY; wheel1Idx++)
						{
							HcmLcdWhlSpdVldFlag[wheel1Idx] = RESET;
						}
						#endif	
					}
					#endif
					
					#if	wheelSpeed_2_msgEnable_cfg > FALSE
					if(msgIdx == wheelSpeed_2_idx)
					{
						#if wheelSpeed_2_VALID_QUANTITY > 0
						/* polyspace<MISRA-C:12.1:not a defect:no action planned>  Condition is OK without parentheses*/
						for(wheel2VldIdx=wheelSpeed_1_VALID_QUANTITY; wheel2VldIdx < wheelSpeed_1_VALID_QUANTITY + wheelSpeed_2_VALID_QUANTITY; wheel2VldIdx++)
						{
							HcmLcdWhlSpdVldFlag[wheel2VldIdx] = RESET;
						}
						#endif				
						#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
						for(wheel2Idx = wheelSpeed_1_WHEEL_QUANTITY; wheel2Idx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; wheel2Idx++)
						{
							HcmLcdWhlSpdVldFlag[wheel2Idx] = RESET;
						}
						#endif					
					}
					#endif
					/* M328401_CR_72 end */
					/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                    continue;
                }
			}

			/* CHECK 4 - signal values OK? */
			switch(msgIdx)
			{
				#if	wheelSpeed_1_msgEnable_cfg > FALSE
				case wheelSpeed_1_idx:
					{
						/* iterate each wheel speed */
						/* polyspace<MISRA-C:12.7:not a defect:no action planned> Use of comma operator is OK without any risk */
						/* polyspace<MISRA-C:13.5:not a defect:no action planned> the expression meet requirement without any risk */
						for(wheel1Idx = FIRST_wheelSpeed_1_WHEEL,wheel1VldIdx =FIRST_wheelSpeed_1_WHEEL_VALIDITY; wheel1Idx < FIRST_wheelSpeed_1_WHEEL + wheelSpeed_1_WHEEL_QUANTITY; wheel1Idx++,wheel1VldIdx++)
						{
						    tempWheelSpeedLcd = FALSE;
						    NumOfWhlVld = RESET;
							
							#if wheelSpeed_1_VALID_QUANTITY > 0
                            p_sig = &p_msg->signalConfig[wheel1VldIdx];
                            currentWheelSpeedValid = *p_sig->actualSig;

                            if(currentWheelSpeedValid == HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                            {
                                HcmLcdWhlSpdVldFlag[wheel1VldIdx-FIRST_wheelSpeed_1_WHEEL_VALIDITY] = SET;
                            }
                            else
                            {
                                HcmLcdWhlSpdVldFlag[wheel1VldIdx-FIRST_wheelSpeed_1_WHEEL_VALIDITY] = RESET;
                            }

                            for(whlSpdVldIdx = 0; whlSpdVldIdx < wheelSpeed_1_VALID_QUANTITY+wheelSpeed_2_VALID_QUANTITY; whlSpdVldIdx++)
                            {
                                NumOfWhlVld += HcmLcdWhlSpdVldFlag[whlSpdVldIdx];
                            }

                            if(NumOfWhlVld < HCM_LCD_MIN_WHEEL_SPEED_VALIDITY_REQUIRED)
                            {
                                /* validity number is not enough, no need go to signal evaluation */
                            	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                                continue;// this inner for-continue, check next wheel
                            }
							#endif

                            /* special behavior only for Mahindra ESCL project*/
                            #if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
                            p_sig = &p_msg->signalConfig[wheel1Idx];
                            currentWheelSpeed = *p_sig->actualSig;
                            if( currentWheelSpeed < HCM_LCD_INVALID_WHEELSPEED_VALUE_MAPPING)
                            {
                                HcmLcdWhlSpdVldFlag[wheel1Idx-FIRST_wheelSpeed_1_WHEEL] = SET;
                            }
                            else
                            {
                                HcmLcdWhlSpdVldFlag[wheel1Idx-FIRST_wheelSpeed_1_WHEEL] = RESET;
                            }

                            for(whlSpdIdx = 0; whlSpdIdx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; whlSpdIdx++)
                            {
                                NumOfWhlVld += HcmLcdWhlSpdVldFlag[whlSpdIdx];
                            }

                            if(NumOfWhlVld < HCM_LCD_MIN_WHEEL_SPEED_VALIDITY_REQUIRED)
                            {
                                /* validity number is not enough, no need go to signal evaluation */
                                /* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                                continue;// this inner for-continue, check next wheel
                            }
                            #endif
							
                            /* get current signal value */
                            p_sig = &p_msg->signalConfig[wheel1Idx];
                            currentWheelSpeed = *p_sig->actualSig;

                            #if HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD > FALSE
                            if(currentWheelSpeed <= HCM_LCD_WHEEL_SPEED_MAX)
                            {
								/* M328401_CR_53 Start */
								/* If can go here, means at least there are HCM_LCD_MIN_WHEEL_SPEED_VALIDITY_REQUIRED tires with valid status */
                                #if wheelSpeed_1_VALID_QUANTITY > 0
								if(currentWheelSpeedValid == HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
								{
                                #endif
                                #if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
								if(currentWheelSpeed < HCM_LCD_INVALID_WHEELSPEED_VALUE_MAPPING)
								{
                                #endif                              		
									if (currentWheelSpeed >= HCM_LCD_WHEEL_SPEED_THRESHOLD_RAW_VALUE)
                                	{
                                    	/* wheel speed signal  out of requirement, exit */
                                    	break;// this inner for-break, no need to check next wheel, check next message
                                	}
                                	else
                                	{
                                    	tempWheelSpeedLcd = TRUE;
                                	}
								#if wheelSpeed_1_VALID_QUANTITY > 0 || HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
                              	}
								else
								{
									tempWheelSpeedLcd = TRUE;
								}
								#endif
								/* M328401_CR_53 End */	
                            }
                            else
                            {
                                /* wheel speed out of range, exit */
                            	/* polyspace<MISRA-C:14.6:not a defect:no action planned>it is Ok to use more break in one loop without any risk*/
                                break;// this inner for-break, no need to check next wheel, check next message
                            }
							
                            #elif HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD_ON_CHANGE > FALSE
                            /* get last wheel speed from SDA */
                            p_sig = &p_msg->signalConfig[wheel1Idx];
                            if((*p_sig->sigSdaStorage == INV_UINT16(*p_sig->sigSdaStorageInv))
                               && (p_sig->sigSdaID == (*p_sig->sigSdaStorage & APP_SDA_BITMASK_SDA_ID)))
                            {
                                HcmLcdLastWheelSpeed1[wheel1Idx-FIRST_wheelSpeed_1_WHEEL] = (*p_sig->sigSdaStorage) & APP_SDA_BITMASK_SDA_DATA;
                            }
                            else
                            {
                                HcmLcdLastWheelSpeed1[wheel1Idx-FIRST_wheelSpeed_1_WHEEL] = p_sig->initValue;
                                SWSW_coco_sda_sda_error     = APP_SDA_SDA_ERROR_TRUE;
                                SWSW_coco_sda_sda_error_inv = INV_UINT16(APP_SDA_SDA_ERROR_TRUE);
                                /* exist for sda invalid */
                                break;// this inner for-break, check next message
                            }

                            if((currentWheelSpeed <= HCM_LCD_WHEEL_SPEED_MAX) && (HcmLcdLastWheelSpeed1[wheel1Idx-FIRST_wheelSpeed_1_WHEEL] <= HCM_LCD_WHEEL_SPEED_MAX))
                            {
                                if(currentWheelSpeed >= HcmLcdLastWheelSpeed1[wheel1Idx-FIRST_wheelSpeed_1_WHEEL])
                                {
                                    wheelSpeedDiff = currentWheelSpeed - HcmLcdLastWheelSpeed1[wheel1Idx-FIRST_wheelSpeed_1_WHEEL];
                                }
                                else
                                {
                                    /* calculate difference from old value to end of range */
                                    wheelSpeedDiff = HCM_LCD_WHEEL_SPEED_MAX - HcmLcdLastWheelSpeed1[wheel1Idx-FIRST_wheelSpeed_1_WHEEL];
                                    /* add step for overflow from end of range to 0 */
                                    wheelSpeedDiff++;
                                    /* add new value */
                                    wheelSpeedDiff += currentWheelSpeed;
                                }
                                if(wheelSpeedDiff > HCM_LCD_WHEEL_SPEED_THRESHOLD_ON_CHANGE_RAW_VALUE)
                                {
                                    /* wheel speed signal change out of requirement, exit */
                                    break;// this inner for-break, check next message
                                }
                                else
                                {
                                    tempWheelSpeedLcd = TRUE;
                                }
                            }
                            else
                            {
                                /* wheel speed out of range, exit */
                                break;// this inner for-break, check next message
                            }
                            #endif
						}
                        /* below indicates evaluation of all wheels is false */
                        if (tempWheelSpeedLcd == FALSE)
                        {
                        	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                        	continue;// this is outer for-continue,check next message
                        }
                        break;// this is switch-break
				    }
			    #endif
				
				#if	wheelSpeed_2_msgEnable_cfg > FALSE
				case wheelSpeed_2_idx:
					{
                        /* iterate each wheel speed */
						/* polyspace<MISRA-C:12.7:not a defect:no action planned> Use of comma operator is OK without any risk */
						/* polyspace<MISRA-C:13.5:not a defect:no action planned> the expression meet requirement without any risk */
                        for(wheel2Idx = FIRST_wheelSpeed_2_WHEEL,wheel2VldIdx =FIRST_wheelSpeed_2_WHEEL_VALIDITY; wheel2Idx < FIRST_wheelSpeed_2_WHEEL + wheelSpeed_2_WHEEL_QUANTITY; wheel2Idx++,wheel2VldIdx++)
                        {
                            tempWheelSpeedLcd = FALSE;
                            NumOfWhlVld = RESET;
							
							#if wheelSpeed_2_VALID_QUANTITY > 0
                            p_sig = &p_msg->signalConfig[wheel2VldIdx];
                            currentWheelSpeedValid = *p_sig->actualSig;

                            if(currentWheelSpeedValid == HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                            {
                                HcmLcdWhlSpdVldFlag[wheel2VldIdx-FIRST_wheelSpeed_2_WHEEL_VALIDITY+wheelSpeed_1_VALID_QUANTITY] = SET;                                }
                            else
                            {
                                HcmLcdWhlSpdVldFlag[wheel2VldIdx-FIRST_wheelSpeed_2_WHEEL_VALIDITY+wheelSpeed_1_VALID_QUANTITY] = RESET;
                            }

                            for(whlSpdVldIdx = 0; whlSpdVldIdx < wheelSpeed_1_VALID_QUANTITY+wheelSpeed_2_VALID_QUANTITY; whlSpdVldIdx++)
                            {
                                NumOfWhlVld += HcmLcdWhlSpdVldFlag[whlSpdVldIdx];
                            }

                            if(NumOfWhlVld < HCM_LCD_MIN_WHEEL_SPEED_VALIDITY_REQUIRED)
                            {
                                /* validity number is not enough, no need go to signal evaluation */
                            	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                                continue;// this inner for-continue, check next wheel
                            }
							#endif

                            /* special behavior only for Mahindra ESCL project*/
                            #if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
                            p_sig = &p_msg->signalConfig[wheel2Idx];
                            currentWheelSpeed = *p_sig->actualSig;
                            if( currentWheelSpeed < HCM_LCD_INVALID_WHEELSPEED_VALUE_MAPPING)
                            {
                                HcmLcdWhlSpdVldFlag[wheel2Idx-FIRST_wheelSpeed_2_WHEEL+wheelSpeed_1_WHEEL_QUANTITY] = SET;
                            }
                            else
                            {
                                HcmLcdWhlSpdVldFlag[wheel2Idx-FIRST_wheelSpeed_2_WHEEL+wheelSpeed_1_WHEEL_QUANTITY] = RESET;
                            }

                            for(whlSpdIdx = 0; whlSpdIdx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; whlSpdIdx++)
                            {
                                NumOfWhlVld += HcmLcdWhlSpdVldFlag[whlSpdIdx];
                            }

                            if(NumOfWhlVld < HCM_LCD_MIN_WHEEL_SPEED_VALIDITY_REQUIRED)
                            {
                                /* validity number is not enough, no need go to signal evaluation */
                                /* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                                continue;// this inner for-continue, check next wheel
                            }
                            #endif

                            /* get current signal value */
                            p_sig = &p_msg->signalConfig[wheel2Idx];
                            currentWheelSpeed = *p_sig->actualSig;

                            #if HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD > FALSE
                            if(currentWheelSpeed <= HCM_LCD_WHEEL_SPEED_MAX)
                            {
								/* M328401_CR_53 Start */
								/* If can go here, means at least there are HCM_LCD_MIN_WHEEL_SPEED_VALIDITY_REQUIRED tires with valid status */
                                #if wheelSpeed_2_VALID_QUANTITY > 0
								if(currentWheelSpeedValid == HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
								{
                                #endif
                                #if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
								if(currentWheelSpeed < HCM_LCD_INVALID_WHEELSPEED_VALUE_MAPPING)
								{
                                #endif                              		
									if (currentWheelSpeed >= HCM_LCD_WHEEL_SPEED_THRESHOLD_RAW_VALUE)
                                	{
                                    	/* wheel speed signal  out of requirement, exit */
                                    	break;// this inner for-break, no need to check next wheel, check next message
                                	}
                                	else
                                	{
                                    	tempWheelSpeedLcd = TRUE;
                                	}
								#if wheelSpeed_2_VALID_QUANTITY > 0 || HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
                              	}
								else
								{
									tempWheelSpeedLcd = TRUE;
								}
								#endif
								/* M328401_CR_53 End */	
                            }
                            else
                            {
                                /* wheel speed out of range, exit */
                            	/* polyspace<MISRA-C:14.6:not a defect:no action planned>it is Ok to use more break in one loop without any risk*/
                                break;// this inner for-break, no need to check next wheel, check next message
                            }

                            #elif HCM_LCD_CHECK_WHEEL_SPEED_THRESHOLD_ON_CHANGE > FALSE
                            p_sig = &p_msg->signalConfig[wheel2Idx];
                            /* get last wheel speed from SDA */
                            if((*p_sig->sigSdaStorage == INV_UINT16(*p_sig->sigSdaStorageInv))
                               && (p_sig->sigSdaID == (*p_sig->sigSdaStorage & APP_SDA_BITMASK_SDA_ID)))
                            {
                                HcmLcdLastWheelSpeed2[wheel2Idx-FIRST_wheelSpeed_2_WHEEL] = (*p_sig->sigSdaStorage) & APP_SDA_BITMASK_SDA_DATA;
                            }
                            else
                            {
                                HcmLcdLastWheelSpeed1[wheel2Idx-FIRST_wheelSpeed_2_WHEEL] = p_sig->initValue;
                                SWSW_coco_sda_sda_error     = APP_SDA_SDA_ERROR_TRUE;
                                SWSW_coco_sda_sda_error_inv = INV_UINT16(APP_SDA_SDA_ERROR_TRUE);
                                /* exist for sda invalid */
                                break;// this inner for-break, check next message
                            }

                            if((currentWheelSpeed <= HCM_LCD_WHEEL_SPEED_MAX) && (HcmLcdLastWheelSpeed2[wheel2Idx-FIRST_wheelSpeed_2_WHEEL] <= HCM_LCD_WHEEL_SPEED_MAX))
                            {
                                if(currentWheelSpeed >= HcmLcdLastWheelSpeed2[wheel2Idx-FIRST_wheelSpeed_2_WHEEL])
                                {
                                    wheelSpeedDiff = currentWheelSpeed - HcmLcdLastWheelSpeed2[wheel2Idx-FIRST_wheelSpeed_2_WHEEL];
                                }
                                else
                                {
                                    /* calculate difference from old value to end of range */
                                    wheelSpeedDiff = HCM_LCD_WHEEL_SPEED_MAX - HcmLcdLastWheelSpeed2[wheel2Idx-FIRST_wheelSpeed_2_WHEEL];
                                    /* add step for overflow from end of range to 0 */
                                    wheelSpeedDiff++;
                                    /* add new value */
                                    wheelSpeedDiff += currentWheelSpeed;
                                }
                                if(wheelSpeedDiff > HCM_LCD_WHEEL_SPEED_THRESHOLD_ON_CHANGE_RAW_VALUE)
                                {
                                    /* wheel speed signal change out of requirement, exit */
                                    break;// this inner for-break, check next message
                                }
                                else
                                {
                                    tempWheelSpeedLcd = TRUE;
                                }
                            }
                            else
                            {
                                /* wheel speed out of range, exit */
                                break;// this inner for-break, check next message
                            }
                            #endif
                        }
                        /* below indicates evaluation of all wheels is false */
                        if (tempWheelSpeedLcd == FALSE)
                        {
                            /* jump to the next loop of outer FOR loop */
                        	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                            continue;// this is outer for-continue,check next message
                        }
                        break;// this is switch-break
				    }
			    #endif
				
				#if	powerMode_msgEnable_cfg > FALSE
				case powerMode_idx:
					{
						#if POWER_MODE_VALIDITY_sigEnable_cfg > FALSE
					    p_sig = &p_msg->signalConfig[POWER_MODE_VALIDITY_idx];
                        powerModeValid = *p_sig->actualSig;
                        if(powerModeValid != HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                        {
                            /* validity not match, exit */
                        	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                            continue;
                        }
						#endif
						
						#if POWER_MODE_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[POWER_MODE_idx];
                        powerMode = *p_sig->actualSig;
                        if(powerMode != HCM_LCD_POWER_MODE_OFF_VALUE_MAPPING)
                        {
                            /* power mode not match, exit */
                        	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
                            continue;
                        }
						#endif
						
						#if IGNITION_STATE_VALIDITY_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[IGNITION_STATE_VALIDITY_idx];
                        ignitionStateValid = *p_sig->actualSig;
                        if(ignitionStateValid != HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                        {
                            /* validity not match, exit */
                            continue;
                        }
						#endif
						
						#if IGNITION_STATE_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[IGNITION_STATE_idx];
                        ignitionState = *p_sig->actualSig;
                        if(ignitionState != HCM_LCD_IGN_STATE_OFF_VALUE_MAPPING)
                        {
                            /* power mode not match, exit */
                            continue;
                        }
						#endif
						
						#if ENGINE_STATE_VALIDITY_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[ENGINE_STATE_VALIDITY_idx];
                        engineStateValid = *p_sig->actualSig;
                        if(engineStateValid != HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                        {
                            /* validity not match, exit */
                            continue;
                        }
						#endif
						
						#if ENGINE_STATE_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[ENGINE_STATE_idx];
                        engineState = *p_sig->actualSig;
                        if(engineState != HCM_LCD_ENGINE_STATE_STOPPED_VALUE_MAPPING)
                        {
                            /* engine state not match, exit */
                            continue;
                        }
						#endif
						
						break;
					}
				#endif
				
				#if	vehicleSpeed_msgEnable_cfg > FALSE
				case vehicleSpeed_idx:
					{
						#if VEHICLE_SPEED_VALIDITY_sigEnable_cfg > FALSE
					    p_sig = &p_msg->signalConfig[VEHICLE_SPEED_VALIDITY_idx];
                        vehicleSpeedValid = *p_sig->actualSig;
                        if(vehicleSpeedValid != HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                        {
                            /* validity not match, exit */
                            continue;
                        }
						#endif
						
						#if VEHICLE_SPEED_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[VEHICLE_SPEED_idx];
                        vehicleSpeed = *p_sig->actualSig;
                        if(vehicleSpeed > HCM_LCD_VEHICLE_SPEED_THRESHOLD_RAW_VALUE)
                        {
                            /* vehicle speed not match, exit */
                            continue;
                        }
						#endif
						
						#if ENGINE_SPEED_VALIDITY_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[ENGINE_SPEED_VALIDITY_idx];
                        engineSpeedValid = *p_sig->actualSig;
                        if(engineSpeedValid != HCM_LCD_SIGNAL_VALID_VALUE_MAPPING)
                        {
                            /* validity not match, exit */
                            continue;
                        }
						#endif
						
						#if ENGINE_SPEED_sigEnable_cfg > FALSE
                        p_sig = &p_msg->signalConfig[ENGINE_SPEED_idx];
                        engineSpeed = *p_sig->actualSig;
                        if(engineSpeed > HCM_LCD_ENGINE_SPEED_THRESHOLD_RAW_VALUE)
                        {
                            /* engine speed not match, exit */
                            continue;
                        }
						#endif
						
						break;
					}
				#endif
				default: break;
			}
            
            /* update signals value to SDA */
            for(signalIdx = 0;signalIdx < (sizeof(CocoConfigurations[msgIdx].signalConfig) / sizeof(SIGNAL_CONFIG_Tag)); signalIdx++ )
            {
                p_sig = &p_msg->signalConfig[signalIdx];
                if((p_sig->sigSdaStorage != NUL) && (p_sig->sigSdaStorageInv != NUL))
                {
                    *p_sig->sigSdaStorage = *p_sig->actualSig | p_sig->sigSdaID;
                    *p_sig->sigSdaStorageInv = INV_UINT16(*p_sig->actualSig | p_sig->sigSdaID);
                }
            }

			/* last step - set LCD TRUE to SDA */
            /* due to the CocoCfgValidator, ensured not be NUL*/
            *p_msg->msgLcdSda    = p_msg->msgLcdSdaTrue;
            *p_msg->msgLcdSdaInv = INV_UINT16(p_msg->msgLcdSdaTrue);
		}
	}
    /* the time inverval between LCD set at the beginning and the end is too long, I2C could interrupt and send unintended value */
	for(msgIdx = 0; msgIdx < msgNum; msgIdx++)
	{
	    p_msg = &CocoConfigurations[msgIdx];
        HCM_LCD_can_msg_lcd_sync[msgIdx] = *p_msg->msgLcdSda;
        HCM_LCD_can_msg_lcd_inv_sync[msgIdx] = *p_msg->msgLcdSdaInv;
	}
}
static void HCM_LCD_evaluate_received_msgs_qual()
{
	bool   tempMsgLcd[msgNum];
	uint16 tempQualCnt[msgNum];
	uint8  msgIdx = 0;
    MSG_CONFIG_Tag const *p_msg = (MSG_CONFIG_Tag const *) 0;

	for(msgIdx = 0; msgIdx < msgNum; msgIdx++)
	{
	    p_msg = &CocoConfigurations[msgIdx];
        if(p_msg->qualCheck == FALSE)
        {
        	/* polyspace<MISRA-C:14.5:not a defect:no action planned>  it is OK to use continue without any risk*/
        	continue;
        }
		/* step1- get msg Lcd flag from SDA */
        if((*p_msg->msgLcdSda == INV_UINT16(*p_msg->msgLcdSdaInv)) &&
           (p_msg->msgLcdSdaID == (*p_msg->msgLcdSda & APP_SDA_BITMASK_SDA_ID)))
        {
            tempMsgLcd[msgIdx] = (bool)(*p_msg->msgLcdSda & APP_SDA_BITMASK_SDA_BOOL);
        }
        else
        {
            /* use default value for lock condition */
            tempMsgLcd[msgIdx] = FALSE;
            /* set SDA error in SDA */
            SWSW_coco_sda_sda_error 	= APP_SDA_SDA_ERROR_TRUE;
            /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
            SWSW_coco_sda_sda_error_inv	= INV_UINT16(APP_SDA_SDA_ERROR_TRUE);
        }

		/* step2- get qual cnt from SDA */
        if((*p_msg->msgQualCntSda == INV_UINT16(*p_msg->msgQualCntSdaInv)) &&
           (p_msg->msgQualCntSdaID == (*p_msg->msgQualCntSda & APP_SDA_BITMASK_SDA_ID)))
        {
            tempQualCnt[msgIdx] = *p_msg->msgQualCntSda & APP_SDA_BITMASK_SDA_DATA;
        }
        else
        {
            /* use default value for lock condition  */
            tempQualCnt[msgIdx] = (uint16)RESET;
            /* set SDA error in SDA */
            SWSW_coco_sda_sda_error 	= APP_SDA_SDA_ERROR_TRUE;
            /* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
            SWSW_coco_sda_sda_error_inv	= INV_UINT16(APP_SDA_SDA_ERROR_TRUE);
        }

		/* step3- increase qual cnt */
		/* lock condition fulfilled? */
        if (TRUE == tempMsgLcd[msgIdx])
        {
            /* qual time not yet reached? */
            if (tempQualCnt[msgIdx] < (p_msg->qualTime/2))
            {
                /* increment qual time counter */
                tempQualCnt[msgIdx]++;
            }
        }
        /* lock condition wheel signals not fulfilled? */
        else
        {
            /* reset qual time counter */
            tempQualCnt[msgIdx] = (uint16)RESET;
        }

		/* step4- store qual cnt to SDA*/
        /* due to the CocoCfgValidator, ensured not be NUL*/
        *p_msg->msgQualCntSda    = tempQualCnt[msgIdx] | p_msg->msgQualCntSdaID;
        *p_msg->msgQualCntSdaInv = INV_UINT16(tempQualCnt[msgIdx] | p_msg->msgQualCntSdaID);
	}/* end of loop*/
}

/**
 * @brief This function checks the lock conditions contained in the status message from the HaCo.
 * @details
 * HaCo status message received?
 *   lock condition HaCo status message fulfilled (SDA)
 *   restart the timeout counter
 *   interpretation of status message finished, clear indication flag
 * @ReqKey MOD_HCM_LCD-56 MOD_HCM_LCD-57 MOD_HCM_LCD-58 MOD_HCM_LCD-59
 */
static void HCM_LCD_check_haco_status_msg(void)
{
	/* HaCo status message received? */
	if (TRUE == SWSW_coco_haco_status_ind)
	{
		/* lock condition HaCo status message fulfilled (SDA) */
		SWSW_coco_sda_lcdhacostatus		= APP_SDA_LcdHacoStatus_TRUE;
		/* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
		SWSW_coco_sda_lcdhacostatus_inv	= INV_UINT16(APP_SDA_LcdHacoStatus_TRUE);

		/* restart the timeout counter */
		HCM_LCD_timeout_cnt_haco_status_u16 = HCM_LCD_TIMEOUT_STATUS_HACO;

		/* interpretation of status message finished, clear indication flag */
		SWSW_coco_haco_status_ind = FALSE;
	}
}

/**
 * @brief This function checks if a timeout of one of the messages from the HaCo occurred.
 * @details
 * ESP_10 not yet in timeout?
 *   decrement timeout counter
 * timeout ESP_10 reached?
 *   set timeout ESP_10
 *   the lock condition wheel signals isn't fulfilled (SDA)
 *   reset qual time counter wheel signals in SDA
 * status message from HaCo not yet in timeout?
 *   decrement timeout counter
 * timeout status message from HaCo reached?
 *   the lock condition HaCo status message isn't fulfilled (SDA)
 * set signal indicating that timeout handler was executed
 * @ReqKey MOD_HCM_LCD-107 MOD_HCM_LCD-108 MOD_HCM_LCD-109 MOD_HCM_LCD-110 MOD_HCM_LCD-111 MOD_HCM_LCD-112
 * @ReqKey MOD_HCM_LCD-72 MOD_HCM_LCD-73 MOD_HCM_LCD-74 MOD_HCM_LCD-75 MOD_HCM_LCD-247
 */
static void HCM_LCD_timeout_handler(void)
{
    
	uint8   msgIdx = 0;
	uint8   wheel1Idx = 0;
	uint8   wheel2Idx = 0;
	uint8   wheel1VldIdx    = FIRST_wheelSpeed_1_WHEEL_VALIDITY;
	uint8   wheel2VldIdx    = FIRST_wheelSpeed_2_WHEEL_VALIDITY;
	 MSG_CONFIG_Tag const *p_msg = (MSG_CONFIG_Tag const *) 0;

	for(msgIdx = 0; msgIdx < msgNum; msgIdx++)
	{
	    p_msg = &CocoConfigurations[msgIdx];
		/* timeout check is no longer configurable, enabled always */
        /* timeout yet? initially yes */
        if (HcmLcdMsgTimeoutCnt[msgIdx] > FALSE)
        {
            /* if not, decrease timeout cnt */
            HcmLcdMsgTimeoutCnt[msgIdx]--;
        }
        /* if timeout ?*/
        if(RESET == HcmLcdMsgTimeoutCnt[msgIdx])
        {
            /* set timeout error flag */
        	/* polyspace<MISRA-C:10.1:not a defect:no action planned> Conversion is OK */
        	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise << is OK*/
        	/* polyspace<MISRA-C:12.7:not a defect:no action planned> using bitwise |= is OK*/
            HCM_LCD_can_msg_timeout_failure   |= (0x01 << msgIdx);
            /* lcd is not fulfilled in SDA */
            *p_msg->msgLcdSda    = p_msg->msgLcdSdaFalse;
            *p_msg->msgLcdSdaInv = INV_UINT16(p_msg->msgLcdSdaFalse);
            if(p_msg->qualCheck > FALSE)
            {
                /* reset qualification time */
                *p_msg->msgQualCntSda    = RESET | p_msg->msgQualCntSdaID;
                *p_msg->msgQualCntSdaInv = INV_UINT16(RESET | p_msg->msgQualCntSdaID);
            }
			
			/* M328401_CR_72 start */
			#if	wheelSpeed_1_msgEnable_cfg > FALSE
			if(msgIdx == wheelSpeed_1_idx)
			{
				#if wheelSpeed_1_VALID_QUANTITY > 0
				for(wheel1VldIdx=0; wheel1VldIdx < wheelSpeed_1_VALID_QUANTITY; wheel1VldIdx++)
				{
					HcmLcdWhlSpdVldFlag[wheel1VldIdx] = RESET;
				}				
				#endif
				#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
				for(wheel1Idx = 0; wheel1Idx < wheelSpeed_1_WHEEL_QUANTITY; wheel1Idx++)
				{
					HcmLcdWhlSpdVldFlag[wheel1Idx] = RESET;
				}
				#endif				
			}
			#endif
			
			#if	wheelSpeed_2_msgEnable_cfg > FALSE
			if(msgIdx == wheelSpeed_2_idx)
			{
				#if wheelSpeed_2_VALID_QUANTITY > 0
				/* polyspace<MISRA-C:12.1:not a defect:no action planned>  Condition is OK without parentheses*/
				for(wheel2VldIdx=wheelSpeed_1_VALID_QUANTITY; wheel2VldIdx < wheelSpeed_1_VALID_QUANTITY + wheelSpeed_2_VALID_QUANTITY; wheel2VldIdx++)
				{
					HcmLcdWhlSpdVldFlag[wheel2VldIdx] = RESET;
				}
				#endif	
				#if HCM_LCD_CHECK_WHEELSPEED_VALIDITY_BY_VALUE > FALSE
				for(wheel2Idx = wheelSpeed_1_WHEEL_QUANTITY; wheel2Idx < wheelSpeed_1_WHEEL_QUANTITY+wheelSpeed_2_WHEEL_QUANTITY; wheel2Idx++)
				{
					HcmLcdWhlSpdVldFlag[wheel2Idx] = RESET;
				}
				#endif				
			}
			#endif
			/* M328401_CR_72 end */			
        }
	}

	/* status message from HaCo not yet in timeout? */
	if (HCM_LCD_timeout_cnt_haco_status_u16 > RESET)
	{
		/* decrement timeout counter */
		HCM_LCD_timeout_cnt_haco_status_u16--;
	}
	/* timeout status message from HaCo reached? */
	if (RESET == HCM_LCD_timeout_cnt_haco_status_u16)
	{
		/* the lock condition HaCo status message isn't fulfilled (SDA) */
		SWSW_coco_sda_lcdhacostatus		= APP_SDA_LcdHacoStatus_FALSE;
		/* polyspace<MISRA-C:19.7:Not a defect:No action planned> macro provided by standard header mq_type.h */
		SWSW_coco_sda_lcdhacostatus_inv	= INV_UINT16(APP_SDA_LcdHacoStatus_FALSE);
	}

	/* set signal indicating that timeout handler was executed */
	SWSW_coco_timeout_check_called = TRUE;
}
/*[[[cog
print('[INFO]: handling template _HCM_LCD.c ends')
]]]*/
//[[[end]]]

/**
 **************************************************************************************
 * @file    rov2016_canbus.c
 * @author  Sivert Sliper, Stian Soerensen
 * @version V1.0
 * @date    3-February-2016
 * @brief   This file contains all the functions for the CAN peripheral.
 **************************************************************************************
 */

/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "rov2016_canbus.h"
#include "stm32f30x_can.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Static Function declarations --------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/
static CanRxMsg RxMsg;
static CanTxMsg TxMsg = {0};
static uint8_t rx_messages = 0; // Counter for the number of new messages received.
static uint8_t TransmitMailbox = 0; // Used for transmitting messages.
static uint8_t FMI_counter = 0;

/* Array for incoming messages, messages are stored in a row according to filter match
 * indicator(FMI). 
 */
uint8_t Rx_Array[16][8];

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Configures the CAN-Controller peripheral for 500 kbps communication.
 * 		   Also configures Rx filters according to ID's specified in "rov2016_canbus.h"
 * @param  None
 * @retval None
 */
void CAN_init(void){
	GPIO_InitTypeDef  		GPIO_InitStructure;
	CAN_InitTypeDef       	CAN_InitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;


	/* Enable clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	/* CAN GPIOs configuration *********************************************************/

	/* Connect CAN pins to AF7, ref. User manual UM1581 p. 111*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_7); //D , PINSOURCE 0, CAN1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_7); //D , PINSOURCE 1, CAN1

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //set to alternate function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* CAN configuration ***************************************************************/

	/* CAN register init */
	CAN_DeInit(CAN1);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_4tq;

	/* CAN Baudrate = 500 kbps (CAN clocked at 9 MHz)
	 * se kapittel 4.1.3 i oppgaven og ref.manual RM0316 seksjon 31.7 */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_Prescaler = 4; // 36MHz/4 = 9 MHz

	/* Initalize the CAN controller */
	CAN_Init(CAN1, &CAN_InitStructure);

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	/* NVIC configuration **************************************************************/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Initialize a standard transmit message */
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.StdId = 0x301;
	TxMsg.ExtId = 0x00;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.DLC = 1;
	TxMsg.Data[0] = 0x0;
}

/**
 * @brief  	This function handles CAN1 RX0 request by storing the received message
 * 			in an array according to the filter match index and message size.
 *
 * 			Received messages are indicated by flipping LED 3.
 *
 * 			The interrupt pending bit is automatically cleared by hardware.
 * @param  	None
 * @retval 	None
 */
void USB_LP_CAN1_RX0_IRQHandler(void){
	CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);
	uint8_t bytes = RxMsg.DLC;

	if (bytes == 0) return; // Return if message is empty.

	do{
		Rx_Array[RxMsg.FMI][bytes-1] = RxMsg.Data[bytes-1];
	}while(--bytes>0);

	/* Increment message received counter */
	rx_messages++;

	/* Indicating message received.*/
	GPIOE->ODR ^= CAN_RX_LED << 8; // Flip receive-LED.
}

/**
 * @brief  	Adds a standard identifier to the receive filter bank.
 * @param  	Standard identifier(11 bit) to be added to the filter bank.
 * @retval 	The filter match index given to the added identifier.
 * 			Returns 255 if filter bank is full.
 */
extern uint8_t CAN_addRxFilter(uint16_t StdId){
	if(FMI_counter > 55) return 255;

	CAN_FilterInitTypeDef FilterStruct;

	/* Setting hardware filtering of incomming messages. Using 16-bit list mode
	 * allows 4 11-bit ID's to be compared. The mask-filters are also used as
	 *  ID's.
	 *
	 * => 4 ID's per filter => maximum 4x14 = 56 message ID's total.
	 *
	 * Mapping: StdID[10:0]-RTR-IDE-EXID[17-15] ref. figure 391 RM0316.
	 * */

	FilterStruct.CAN_FilterMode = CAN_FilterMode_IdList;
	FilterStruct.CAN_FilterScale = CAN_FilterScale_16bit;
	FilterStruct.CAN_FilterFIFOAssignment = CAN_FIFO0;
	FilterStruct.CAN_FilterActivation = ENABLE;
	FilterStruct.CAN_FilterNumber = FMI_counter/4;

	uint8_t f_index = FMI_counter % 4;
	switch(f_index){
	case 0:
		FilterStruct.CAN_FilterIdLow = 		  (StdId << 5) \
											| (CAN_RTR_DATA << 4)	 \
											| (CAN_ID_STD << 3);
		break;
	case 1:
		FilterStruct.CAN_FilterMaskIdLow =    (StdId << 5) \
											| (CAN_RTR_DATA << 4)	 \
											| (CAN_ID_STD << 3);
		break;
	case 2:
		FilterStruct.CAN_FilterIdHigh =		  (StdId << 5) \
											| (CAN_RTR_DATA << 4)	 \
											| (CAN_ID_STD << 3);
		break;
	case 3:
		FilterStruct.CAN_FilterMaskIdHigh =   (StdId << 5) \
											| (CAN_RTR_DATA << 4)	 \
											| (CAN_ID_STD << 3);
		break;
	}

	CAN_FilterInit(&FilterStruct);
	FMI_counter++;
	return FMI_counter - 1;
}

/**
 * @brief  Returns the number of unprocessed messages.
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
extern uint8_t CAN_getRxMessages(void){
	return rx_messages;
}

/**
 * @brief  	Returns a pointer to a specified message.
 * @param  	uint8_t filter_number: 	Specify which filter match index the wanted
 * 									message belongs to.
 * @retval  Pointer to the specified messagebuffer.
 */
extern uint8_t* CAN_getMessagePointer(uint8_t filter_number){
	return &Rx_Array[filter_number][0];
}

/**
 * @brief  	Returns the specified byte from the Rx array.
 * @param  	uint8_t filter_number: 	Specify which filter match index the wanted
 * 									message belongs to.
 * @param	uint8_t byte_number:	Specify where the wanted byte is in the
 * 									data field of the received message.
 * @retval 	The specified byte.
 */
extern uint8_t CAN_getByteFromMessage(uint8_t filter_number, uint8_t byte_number){
	return Rx_Array[filter_number][byte_number];
}

/**
 * @brief  Transmit byte
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
extern void CAN_transmitByte(uint16_t StdId, uint8_t data){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = StdId;
	TxMsg.ExtId = 0x00;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.DLC = 1;
	TxMsg.Data[0] = data;

	/* Put message in Tx Mailbox and store the mailbox number.
	 * Stall for a while (<1 second) if no mailbox is available.
	 */
	TransmitMailbox = CAN_TxStatus_NoMailBox;
	volatile uint32_t watchdog = 200000;
	while(1){
		TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);
		if(TransmitMailbox != CAN_TxStatus_NoMailBox){
			break;
		} else if(watchdog--<10){
			/* Return if no mailbox available within a few 100's of milliseconds. */
			return;
		}
	}
}


/**
 * @brief  	Transmit buffer
 * @param  	Id 		- ID of message to be sent.
 * 			buffer 	- Pointer to a buffer containing bytes to be sent.
 * 			length	- Length of packet to be sent. (in bytes).
 * 			Id_Type - Identifier type, can be:
 * 							CAN_ID_STD - Standard 11 bit identifier.
 * 							CAN_ID_EXT - Extended 31 bit identifier.
 * @retval 	None
 */
extern void CAN_transmitBuffer(uint32_t Id, uint8_t* buffer, uint8_t length, uint8_t Id_Type){
	/* Parameter check. */
	if(length > 8){
		return;
	}

	//	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Prepare message to be sent */
	TxMsg.IDE = Id_Type;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = length;

	/* Load ID into message. */
	if(Id_Type == CAN_ID_EXT){
		TxMsg.StdId = (uint16_t)(Id >> 18); // Top 11 bits of the 31 bit identifier
		TxMsg.ExtId = (uint32_t)(Id & 0x00003FFFF); // Bottom 18 bits of the identifier.
	} else {
		TxMsg.StdId = Id;
		TxMsg.ExtId = 0;
	}

	/* Load data into message. */
	volatile uint8_t i;
	for(i=0; i<length; i++){
		TxMsg.Data[i] = buffer[i];
	}

	/* Put message in Tx Mailbox and store the mailbox number.
	 * Stall for a while (<1 second) if no mailbox is available.
	 */
	TransmitMailbox = CAN_TxStatus_NoMailBox;
	volatile uint32_t watchdog = 200000;
	while(1){
		TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);
		if(TransmitMailbox != CAN_TxStatus_NoMailBox){
			break;
		} else if(watchdog--<10){
			/* Return if no mailbox available within a few 100's of milliseconds. */
			return;
		}
	}

}


/**
 * @brief  Transmit byte with extended identifier.
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
extern void CAN_transmitByte_EID(uint32_t EID, uint8_t data){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = 0;
	TxMsg.ExtId = EID;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.IDE = CAN_ID_EXT;
	TxMsg.DLC = 1;
	TxMsg.Data[0] = data;

	/* Put message in Tx Mailbox and store the mailbox number.
	 * Stall for a while (<1 second) if no mailbox is available.
	 */
	TransmitMailbox = CAN_TxStatus_NoMailBox;
	volatile uint32_t watchdog = 200000;
	while(1){
		TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);
		if(TransmitMailbox != CAN_TxStatus_NoMailBox){
			break;
		} else if(watchdog--<10){
			/* Return if no mailbox available within a few 100's of milliseconds. */
			return;
		}
	}
}

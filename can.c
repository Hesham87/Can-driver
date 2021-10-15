#include <stdbool.h>
#include <stdint.h>
#include "can.h"
#include "tm4c123gh6pm.h"
//*************************************************************************
//*************************************************************************
// Please Read the following.
// this driver is based on the tiva ware specification driver 
// here is the link https://www.ti.com/lit/ug/spmu298e/spmu298e.pdf?ts=1631361990557&ref_url=https%253A%252F%252Fwww.google.com%252F.
// two functions have been added to configure the can driver and the test mode
//*************************************************************************
//*************************************************************************

typedef volatile uint32_t *address;
void CAN_Enable(uint32_t ui32Base)
{
	// clear INIT bit to allow the transfering and receiving of messages.
	(*((address)(ui32Base + CTL_R))) &= ~CAN_CTL_INIT;
}

void CAN_Disable(uint32_t ui32Base)
{
	// set INIT bit to stop the transfering and receiving of messages.
	(*((address)(ui32Base + CTL_R))) |= CAN_CTL_INIT;
}

void CAN_Int_Enable(uint32_t ui32Base, uint32_t IntFlag)
{
	// IntFlag is the bit mask of the interrupt sources to be enabled.
	(*((address)(ui32Base + CTL_R))) |= IntFlag;
}

void CAN_Int_Disable(uint32_t ui32Base, uint32_t IntFlag)
{
	// IntFlag is the bit mask of the interrupt sources to be disabled.
	(*((address)(ui32Base + CTL_R))) &= ~IntFlag;
}

uint32_t CAN_Int_Status(uint32_t ui32Base, tCANIntStsReg eStatusReg)
{
	// This function returns the value of one of two interrupt status registers.
	// eStatusReg indicates which interrupt status register to read
	uint32_t reg;
	
	if(eStatusReg == CAN_INT_STS_CAUSE)
  {
		 // returns the global interrupt status for the CAN controller specified by ui32Base.
		 reg = (*((address)(ui32Base + INT_R)));
  }
	else if(eStatusReg == CAN_INT_STS_OBJECT)
  {
		 // return the current message status interrupt for all messages.
		 // Read and combine both 16 bit values into one 32bit status.
		 reg = (*((address)(ui32Base + MSG1INT_R))) & CAN_MSG1INT_INTPND_M;
		 reg |= (*((address)(ui32Base + MSG2INT_R))) << 16;
	}
	else
	{
		//if unkown regester return 0.
		reg=0;
	}
  return(reg);
}

void CAN_Int_Clear(uint32_t ui32Base, uint32_t ui32IntClr)
{
	if(ui32IntClr == CAN_INT_INTID_STATUS)
	{
		// To clear the status interrupt simply read the satuts register.
		(*((address)(ui32Base + STS_R)));
	}
	else
	{
		// Only change the interrupt pending state by setting only the CAN_IF1CMSK_CLRINTPND bit.
		(*((address)(ui32Base + IF1CMSK_R))) = CAN_IF1CMSK_CLRINTPND;
		// Send the clear pending interrupt command to the CAN controller.
		(*((address)(ui32Base + IF1CRQ_R))) = ui32IntClr & CAN_IF1CRQ_MNUM_M;
	}
}

void CAN_Retry_Set(uint32_t ui32Base, bool bAutoRetry)
{
	uint32_t clt_reg = (*((address)(ui32Base + CTL_R)));
  // Conditionally set the DAR bit to enable/disable auto-bAutoRetry.
  if(bAutoRetry)
  {
		// Clearing the DAR bit tells to enable auto bAutoRetry
		clt_reg &= ~CAN_CTL_DAR;
  }
  else
  {
		// Setting the DAR bit to disable auto bAutoRetry.
		clt_reg |= CAN_CTL_DAR;
  }

  (*((address)(ui32Base + CTL_R))) = clt_reg;
}

bool CAN_Retry_Get(uint32_t ui32Base)
{
	 // Simply check the DAR bit to know if it is set or not.
	 if((*((address)(ui32Base + CTL_R))) & CAN_CTL_DAR)
	 {
		// Auto bAutoRetry is disabled.
		return(false);
	 }
	 // Else auto bAutoRetry is enabled.
	 return(true);
}

uint32_t CAN_Status_Get(uint32_t ui32Base, tCANStsReg Sts_R)
{
	uint32_t reg;

	switch(Sts_R)
	{
			// Read the full CAN controller status.
			// return the global CAN status register.
			case CAN_STS_CONTROL:
			{
					reg = (*((address)(ui32Base + STS_R)));
					break;
			}
			// Read all message objects with a transmit request set.
			// The Transmit status bits are placed into two registers and require combining into one 32bit value.
			case CAN_STS_TXREQUEST:
			{
					reg = (*((address)(ui32Base + TXRQ1_R))) | ((*((address)(ui32Base + TXRQ2_R))) << 16);
					break;
			}
			// Read all message objects with new data available.
			// The New Data status bits are placed into two registers and require combining into one 32bit value.
			case CAN_STS_NEWDAT:
			{
					reg = (*((address)(ui32Base + NWDA1_R))) | ((*((address)(ui32Base + NWDA2_R))) << 16);
					break;
			}
			// Read all message objects that are enabled.
			// The Message valid status bits are placed into two registers and require combining into one 32bit value.
			case CAN_STS_MSGVAL:
			{
					reg = (*((address)(ui32Base + MSG1VAL_R))) | ((*((address)(ui32Base +MSG2VAL_R))) << 16);
					break;
			}

			// Returning 0 due to unknown status requested.
			default:
			{
					reg = 0;
					break;
			}
	}
	return(reg);	
}

bool CAN_Err_Cntr_Get(uint32_t ui32Base, uint32_t *reception_cntr, uint32_t *transmit_cntr)
{
	// Read the current count of transmit/receive errors.
	uint32_t can_error = (*((address)(ui32Base + ERR_R)));
	// Store the counter values.
	*reception_cntr = (can_error & CAN_ERR_REC_M) >> CAN_ERR_REC_S;
	*transmit_cntr = (can_error & CAN_ERR_TEC_M) >> CAN_ERR_TEC_S;
	//If number of reception or transmit errors is more than 128 then the EPASS bit in the can status register is set.  
	if(can_error & CAN_ERR_RP)
	{
			return(true);
	}
	return(false);
}	
/*************************************************
After reset, the CAN controller is left in the disabled state. However, the memory used for
message objects contains undefined values and must be cleared prior to enabling the CAN
controller the first time. This prevents unwanted transmission or reception of data before the
message objects are configured. This function must be called before enabling the controller
the first time.
*****************************************************/

void CAN_Init(uint32_t ui32Base)
{
	uint32_t ui32Msg;
	// Place CAN controller in init state, regardless of previous state.This puts controller in idle, and allow the message object RAM to be programmed.
	(*((address)(ui32Base + CTL_R))) = CAN_CTL_INIT;
	// Set the WRNRD bit to Transfer the data in the CANIFn registers to the CAN message object specified by the MNUM field in the CAN Command Request (CANIFnCRQ).
	// Set the ARB bit to Transfer ID + DIR + XTD + MSGVAL of the message object into the Interface registers.
	// Set the Control bit to Transfer control bits from the CANIFnMCTL register into the Interface registers.
	(*((address)(ui32Base + IF1CMSK_R))) = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_ARB | CAN_IF1CMSK_CONTROL);
	// Clear the message value bit in the arbitration register to indicate its not valid so as not to cause runtime errors.
	(*((address)(ui32Base + IF1ARB2_R))) = 0;
	// Reset the MCTL register
	(*((address)(ui32Base + IF1MCTL_R))) = 0;
	// Loop through to program all 32 message objects
	for(ui32Msg = 1; ui32Msg <= 32; ui32Msg++)
	{
		// Set the MNUM bits to transfer the data to the message objects.
		(*((address)(ui32Base + IF1CRQ_R))) = ui32Msg;
		// Wait for the message to be sent.
		while((*((address)(ui32Base + IF1CRQ_R))) & CAN_IF1CRQ_BUSY);
	}
	// Set the new data flag to update the message objects.
	(*((address)(ui32Base + IF1CMSK_R))) |= (CAN_IF1CMSK_NEWDAT);
}

void CAN_Bit_Timing_Get(uint32_t ui32Base, tCANBitClkParms *psClkParms)
{
	// Read the CANBIT register.
	uint32_t ui32BitReg = (*((address)(ui32Base + BIT_R)));
	// Store the synchronous jump width. add one because The actual interpretation by the hardware of this value is such that one more than the value programmed here is used.
	psClkParms->ui32SJW = ((ui32BitReg & CAN_BIT_SJW_M) >> CAN_BIT_SJW_S) + 1;
	// Store the phase 2 segment. add one because The actual interpretation by the hardware of this value is such that one more than the value programmed here is used.
	psClkParms->ui32Phase2Seg = ((ui32BitReg & CAN_BIT_TSEG2_M) >> CAN_BIT_TSEG2_S) + 1;
	// Store the phase 1 segment. add one because The actual interpretation by the hardware of this value is such that one more than the value programmed here is used.
	psClkParms->ui32SyncPropPhase1Seg = ((ui32BitReg & CAN_BIT_TSEG1_M) >> CAN_BIT_TSEG1_S) + 1;
	// Store the pre-scaler and the extension of the pre-scaler. add one because The actual interpretation by the hardware of this value is such that one more than the value programmed here is used.
	psClkParms->ui32QuantumPrescaler = ((ui32BitReg & CAN_BIT_BRP_M) | (((*((address)(ui32Base + BRPE_R))) & CAN_BRPE_BRPE_M) << 6)) + 1;
}

void CAN_Bit_Timing_Set(uint32_t ui32Base, tCANBitClkParms *psClkParms)
{
	uint32_t tempRegister;
	uint32_t tSeg1, tSeg2, BRP, SJW;
	// tTSeg2 --> Time Segment after Sample Point. 
	// tTSeg1 --> Time Segment Before Sample Point.
	// tSync --> Synchronization Jump Width. Note. 
	// BRP --> Baud Rate Prescaler
	// You should be aware that the value of the pervious three values will be decreased by one,
	// because the actual interpretation by the hardware of these values is such that one more than the value programmed here is used.
	// We have to set the init bit and the configuration change bit to be able to configer the CANBIT register
	// So we will save the CTL register state and restore it after we set the CANBIT register
	tempRegister = (*((address)(ui32Base + CTL_R)));
	(*((address)(ui32Base + CTL_R))) = tempRegister | CAN_CTL_INIT | CAN_CTL_CCE;
	// bit time = tTSeg2 + tTSeg1 + SJW + BRP
	tSeg2 = (((psClkParms->ui32Phase2Seg - 1) << CAN_BIT_TSEG2_S) & CAN_BIT_TSEG2_M);
	tSeg1 = (((psClkParms->ui32SyncPropPhase1Seg - 1) << CAN_BIT_TSEG1_S) & CAN_BIT_TSEG1_M);
	SJW = ((psClkParms->ui32SJW - 1) << CAN_BIT_SJW_S) & CAN_BIT_SJW_M;
	BRP = (psClkParms->ui32QuantumPrescaler - 1) & CAN_BIT_BRP_M;
	(*((address)(ui32Base + BIT_R))) = tSeg2 | tSeg1 | SJW | BRP;
	// Set the bits in the Prescaler Extension register.
	(*((address)(ui32Base + BRPE_R))) = ((psClkParms->ui32QuantumPrescaler - 1) >> 6) & CAN_BRPE_BRPE_M;
	// return the CTL register to its previous value
	(*((address)(ui32Base + CTL_R))) = tempRegister;
}
void CAN_Message_Clear(uint32_t ui32Base, uint32_t ui32ObjID)
{
	// Set the WRNRD to transfer the message object and set the ARB bit to Transfer MSGVAL of the message object into the Interface registers to indicate that it is not valid.
	(*((address)(ui32Base + IF1CMSK_R))) = CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_ARB;
	// clear the arbitration registers and make the message object invalid.
	(*((address)(ui32Base + IF1ARB1_R))) = 0;
	(*((address)(ui32Base + IF1ARB2_R))) = 0;
	// Initiate programming the message object
	(*((address)(ui32Base + IF1CRQ_R))) = ui32ObjID & CAN_IF1CRQ_MNUM_M;
}

void CAN_Message_Get(uint32_t ui32Base, uint32_t ui32ObjID, tCANMsgObject *psMsgObject, bool bClrPendingInt)
{
	uint16_t cmdMask_16V = 0;
	uint16_t mctl_16V = 0;
	uint16_t mask0_16V = 0, mask1_16V = 0;
	uint16_t arb0_16V = 0, arb1_16V = 0;
	// These bits will always be set when reading a message.
	// DATAA and DATAB will transfere the 8 byte date in the data register to the message object.
	// Control bit will transfer control bits from the CANIFnMCTL register into the Interface registers.
	// ARB will Transfer ID + DIR + XTD + MSGVAL of the message object into the Interface registers.
	// these functions are all neccassary to reading messages. 
	cmdMask_16V = (CAN_IF1CMSK_DATAA | CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL | CAN_IF1CMSK_MASK | CAN_IF1CMSK_ARB);
	// Clear a pending interrupt and new data in a message object.
	if(bClrPendingInt)
	{
			// Set the CLRINTPND bit in the comand mask register to clear the pending interrupt.
			cmdMask_16V |= CAN_IF1CMSK_CLRINTPND;
	}
	// Clear all the flags so that we can begin setting them.
	psMsgObject->ui32Flags = MSG_OBJ_NO_FLAGS;
	// First we will activate the above mentioned functions by setting the command mask register.
	// Then by setting the MNUM bit in the command request register we can transfer the message object. 
	(*((address)(ui32Base + IF2CMSK_R))) = cmdMask_16V;
	(*((address)(ui32Base + IF2CRQ_R))) = ui32ObjID & CAN_IF1CRQ_MNUM_M;
	// Now we will read the registers that we just transfered the data to.
	mctl_16V = (*((address)(ui32Base + IF2MCTL_R)));
	arb0_16V = (*((address)(ui32Base + IF2ARB1_R)));
	arb1_16V = (*((address)(ui32Base + IF2ARB2_R)));
	mask0_16V = (*((address)(ui32Base + IF2MSK1_R)));
	mask1_16V = (*((address)(ui32Base + IF2MSK2_R)));
	// Check if it is Extended identifier or 11-bit identifier by cheking the XTD bit.
	if(arb1_16V & CAN_IF1ARB2_XTD)
	{
			// It is a 29-bit extended identifier.
			// Message ID is [15:0] bits in arb0 register and [12:0] bits in arb1 register. 
			psMsgObject->ui32MsgID = (((arb1_16V & CAN_IF1ARB2_ID_M) << 16) | arb0_16V);
			// Set the flag for extended identifier.
			psMsgObject->ui32Flags |= MSG_OBJ_EXTENDED_ID;
	}
	else
	{
			// it is an 11 bit normal identifier.
			psMsgObject->ui32MsgID = (arb1_16V & CAN_IF1ARB2_ID_M) >> 2;
	}
	// Check if this is a remote frame by checking the TXRQST bit and DIR bit.
	if(((mctl_16V & CAN_IF1MCTL_TXRQST) && (!(arb1_16V & CAN_IF1ARB2_DIR))) || ((arb1_16V & CAN_IF1ARB2_DIR) && !(mctl_16V & CAN_IF1MCTL_TXRQST)))
	{
			psMsgObject->ui32Flags |= MSG_OBJ_REMOTE_FRAME;
	}
	// Check if mask was used for acceptance filtering.
	// If UMASK bit in the MCTL register was set it indicates that the mask was used for acceptance filtering. 
	if(mctl_16V & CAN_IF1MCTL_UMASK)
	{
			// Check if it is an extended identifier to know if the mask is 29-bit value.
			if(arb1_16V & CAN_IF1ARB2_XTD)
			{
					psMsgObject->ui32MsgIDMask =((mask1_16V & CAN_IF1MSK2_IDMSK_M) << 16) | mask0_16V;
			}
			else
			{
					// Set the mask ID for an 11-bit identifier by getting bits [12:2] from mask2 register.
					psMsgObject->ui32MsgIDMask = (mask1_16V & CAN_IF1MSK2_IDMSK_M) >> 2;
			}
			// Check direction filtering by checking the MDIR bit in the mask 2 register.
			if(mask1_16V & CAN_IF1MSK2_MDIR)
			{
					psMsgObject->ui32Flags |= MSG_OBJ_USE_DIR_FILTER;
			}
			// Check if extended bits was used in filtering by checking the MXTD int mask 2 register.
			if(mask1_16V & CAN_IF1MSK2_MXTD)
			{
					psMsgObject->ui32Flags |= MSG_OBJ_USE_EXT_FILTER;
			}
	}
	// Check if we lost some data.
	// MSGLST bit in the MCTL register indicates that the message handler stored a new message into this object when NEWDAT was set; the CPU has lost a message.
	if(mctl_16V & CAN_IF1MCTL_MSGLST)
	{
			psMsgObject->ui32Flags |= MSG_OBJ_DATA_LOST;
	}
	// Check the transmit interrupt by checking the TXIE bit in the MCTL register.
	if(mctl_16V & CAN_IF1MCTL_TXIE)
	{
			psMsgObject->ui32Flags |= MSG_OBJ_TX_INT_ENABLE;
	}
	// Check the recieve interrupt by checking the RXIE bit in the MCTL register.
	if(mctl_16V & CAN_IF1MCTL_RXIE)
	{
			psMsgObject->ui32Flags |= MSG_OBJ_RX_INT_ENABLE;
	}
	// Check if there is new data available by checking the NEWDAT bit in the MCTL register
	if(mctl_16V & CAN_IF1MCTL_NEWDAT)
	{
			// Get the amount of data needed to be read.
			psMsgObject->ui32MsgLen = (mctl_16V & CAN_IF1MCTL_DLC_M);
			// if the frame is a remote frame then we don't have to read any data as there is no valid data.
			if((psMsgObject->ui32Flags & MSG_OBJ_REMOTE_FRAME) == 0)
			{
					uint32_t counter = 0;
					// There are 4 registers that contain the data to be sent the difference between each of their offests is 0x04.
					// they begin with register IF1DA1_R and end with IF1DB2_R.
					// each register holds two bytes.
					for(uint32_t offset = IF1DA1_R; offset <= IF1DB2_R && (counter < psMsgObject->ui32MsgLen); offset+=0x04)
					{
						
						// pui8MsgData is a pointer to bytes.
						// So counter must not exceed the length of data. 
						// Anding with 0xFF to get the first byte.
						psMsgObject->pui8MsgData[counter++] = (*((address)(ui32Base + offset))) & 0xFF;
						
						if(counter < psMsgObject->ui32MsgLen)
						{
							// Anding with 0xFF00 to get the second byte.
							psMsgObject->pui8MsgData[counter++] = ((*((address)(ui32Base + offset))) & 0xFF00) >> 8;
						}
					}
					// Read out the data from the CAN registers.
					//_CANDataRegRead(psMsgObject->pui8MsgData, (uint32_t *)(ui32Base + IF2DA1_R), psMsgObject->ui32MsgLen);
			}
			// The new data bit should be cleared now.
			(*((address)(ui32Base + IF2CMSK_R))) = CAN_IF1CMSK_NEWDAT;
			// Set the MNUM to transfer the selected message numbers in ui32ObjID.
			(*((address)(ui32Base + IF2CRQ_R))) = ui32ObjID & CAN_IF1CRQ_MNUM_M;
			// Indicate that there is new data in this message.
			psMsgObject->ui32Flags |= MSG_OBJ_NEW_DATA;
	}
	else
	{
			// the amount of data should be zero sincee there is no new data.
			psMsgObject->ui32MsgLen = 0;
	}
}

/******************************************
The CAN_Message_Set function is used to configure any one of the 32 message objects in the CAN controller. A
message object can be configured to be any type of CAN message object as well as to use automatic transmission and reception. This call also allows the message object to be configured
to generate interrupts on completion of message receipt or transmission. The message object
can also be configured with a filter/mask so that actions are only taken when a message that
meets certain parameters is seen on the CAN bus.
******************************************/
void CAN_Message_Set(uint32_t ui32Base, uint32_t ui32ObjID, tCANMsgObject *psMsgObject, tMsgObjType eMsgType)
{
	bool checkTransfer = false;
	bool checkExtended = false;
	uint16_t mctl_16V = 0;
	uint16_t cmdMask_16V = 0;
	uint16_t arb0_16V = 0, arb1_16V = 0;
	uint16_t mask0_16V = 0, mask1_16V = 0;
	// These bits must be set in any mode when seting a message.
	cmdMask_16V = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_DATAA | CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL);

	switch(eMsgType)
	{
			// Transmit message object.
			case MSG_OBJ_TYPE_TX:
			{
					// Set the TXRQST bit and the reset the rest of the register.
					mctl_16V |= CAN_IF1MCTL_TXRQST;
				  // Set the DIR bit so that respective message object is transmitted as a data frame.
					// On reception of a remote frame with matching identifier
					arb1_16V = CAN_IF1ARB2_DIR;
					checkTransfer = true;
					break;
			}
			// Receive message object.
			case MSG_OBJ_TYPE_RX:
			{
					// This clears the DIR bit along with everything else.  The TXRQST
					// bit was cleared by defaulting mctl_16V to 0.
					arb1_16V = 0;
					break;
			}
			// Transmit remote request message object
			case MSG_OBJ_TYPE_TX_REMOTE:
			{
					// Set the TXRQST bit and the reset the rest of the register.
					mctl_16V |= CAN_IF1MCTL_TXRQST;
					// Clear the DIR bit so that a remote frame with the identifier of this message objectis received.
					arb1_16V = 0;
					break;
			}
			// Receive remote request message object.
			case MSG_OBJ_TYPE_RX_REMOTE:
			{
					// Set this bit to use mask for acceptance filtering.
					mctl_16V = CAN_IF1MCTL_UMASK;
					// Set the DIR bit for remote receivers.
					arb1_16V = CAN_IF1ARB2_DIR;
					// Send the mask to the message object.
					cmdMask_16V |= CAN_IF1CMSK_MASK;
					break;
			}
			// Remote frame receive remote, with auto-transmit message object.
			case MSG_OBJ_TYPE_RXTX_REMOTE:
			{
					// Set DIR bit for remote receivers.
					arb1_16V = CAN_IF1ARB2_DIR;
					// Set the RMTEN bit so that once a remote frame is recieved, the TXRQST bit in the CANIFnMCTL register is set.
					// Set the UMASK bet to search for a matching identifier.
					mctl_16V = CAN_IF1MCTL_RMTEN | CAN_IF1MCTL_UMASK;
					// The data to be returned needs to be filled in.
					checkTransfer = true;
					break;
			}
			default:
			{
					return;
			}
	}
	// See if we need to use an extended identifier or not.
	if((psMsgObject->ui32MsgID > CAN_11BIT_ID_MAX) || (psMsgObject->ui32Flags & MSG_OBJ_EXTENDED_ID))
	{
			checkExtended = true;
	}
	else
	{
			checkExtended = false;
	}
	// Configure the Mask Registers.
	if(psMsgObject->ui32Flags & MSG_OBJ_USE_ID_FILTER)
	{
			if(checkExtended)
			{
				// When using a 29-bit identifier, bits [15:0] in CANIF1MSK1 register are used for bits [15:0] of the ID.
				// The bits [12:0] in the MSK field in the CANIFnMSK2 register are used for bits [28:16] of the ID.
				mask0_16V = psMsgObject->ui32MsgIDMask & CAN_IF1MSK1_IDMSK_M;
				mask1_16V = ((psMsgObject->ui32MsgIDMask >> 16) & CAN_IF1MSK2_IDMSK_M);
			}
			else
			{
				// Lower 16 bit are unused so set them to zero.
				mask0_16V = 0;
				// When using an 11-bit identifier, MSK[12:2] in the CANIF1MSK2 register are used for bits [10:0] of the ID.
				mask1_16V = ((psMsgObject->ui32MsgIDMask << 2) & CAN_IF1MSK2_IDMSK_M);
			}
	}
	// MSG_OBJ_USE_DIR_FILTER indicates that a message object uses or is using filtering based on the direction of the transfer
	if((psMsgObject->ui32Flags & MSG_OBJ_USE_DIR_FILTER) == MSG_OBJ_USE_DIR_FILTER)
	{
			// Set MDIR bit so that the message direction bit DIR is used for acceptance filtering.
			mask1_16V |= CAN_IF1MSK2_MDIR;
	}
	// MSG_OBJ_USE_EXT_FILTER indicates that a message object uses or is using message identifier filtering based on the extended identifier.
	if((psMsgObject->ui32Flags & MSG_OBJ_USE_EXT_FILTER) == MSG_OBJ_USE_EXT_FILTER)
	{
		// Set the MXTD so that the extended identifier bit XTD is used for acceptance filtering.
			mask1_16V |= CAN_IF1MSK2_MXTD;
	}
	// If any of the pervious if conditions is true then we need to Transfer IDMASK + DIR + MXTD of the message object into the Interface registers.
	if(psMsgObject->ui32Flags & (MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_DIR_FILTER | MSG_OBJ_USE_EXT_FILTER))
	{
			// Set the UMASK bit to enable using the mask register.
			mctl_16V |= CAN_IF1MCTL_UMASK;
			// Set the MASK bit so thatIDMASK + DIR + MXTD of the message object are transfered into the Interface registers.
			cmdMask_16V |= CAN_IF1CMSK_MASK;
	}
	// Set the Arb bit so that we can Transfer ID + DIR + XTD + MSGVAL of the message object into the Interface registers as we have finished configuring them.
	cmdMask_16V |= CAN_IF1CMSK_ARB;
	// MSG_OBJ_FIFO bit indicates that this message object is part of a FIFO structure. If it is cleared then it is either means it is a single message or the last message in the FIFO structure.
	if((psMsgObject->ui32Flags & MSG_OBJ_FIFO) == 0)
	{
		// Set the EOB bit to classify the message object as part of a FIFO structure. 
			mctl_16V |= CAN_IF1MCTL_EOB;
	}
	// Configure the Arbitration registers.
	if(checkExtended)
	{
			// When using a 29-bit identifier, bits 15:0 of the CANIFnARB1 register are [15:0] of the ID, while bits 12:0 of the CANIFnARB2 register are [28:16] of the ID.
			// Set the 29 bit version of the Identifier for this message object.
			arb0_16V |= psMsgObject->ui32MsgID & CAN_IF1ARB1_ID_M;
			arb1_16V |= (psMsgObject->ui32MsgID >> 16) & CAN_IF1ARB2_ID_M;
			// Mark the message as valid and set the extended ID bit.
			arb1_16V |= CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD;
	}
	else
	{
			// Set the 11 bit version of the Identifier for this message object.
			// The lower 18 bits are set to zero.
			arb1_16V |= (psMsgObject->ui32MsgID << 2) & CAN_IF1ARB2_ID_M;
			// Mark the message as valid.
			arb1_16V |= CAN_IF1ARB2_MSGVAL;
	}
	// Set the data length in the MCTL register.
	mctl_16V |= (psMsgObject->ui32MsgLen & CAN_IF1MCTL_DLC_M);
	// MSG_OBJ_TX_INT_ENABLE bit indicates that we should enable transmit interrupts.
	if(psMsgObject->ui32Flags & MSG_OBJ_TX_INT_ENABLE)
	{
			mctl_16V |= CAN_IF1MCTL_TXIE;
	}
	// MSG_OBJ_RX_INT_ENABLE bit indicates that we should enable recieve interrupts.
	if(psMsgObject->ui32Flags & MSG_OBJ_RX_INT_ENABLE)
	{
			mctl_16V |= CAN_IF1MCTL_RXIE;
	}
	// Write the data out to the CAN Data registers if needed.
	if(checkTransfer)
	{
		uint32_t counter = 0;
		// There are 4 registers that contain the data to be sent the difference between each of their offests is 0x04.
		// they begin with register IF1DA1_R and end with IF1DB2_R.
		// each register holds two bytes.
		for(uint32_t offset = IF1DA1_R; offset <= IF1DB2_R && (counter < psMsgObject->ui32MsgLen); offset+=0x04)
		{
			// pui8MsgData is a pointer to bytes.
			// So counter must not exceed the length of data. 
			(*((address)(ui32Base + offset))) = psMsgObject->pui8MsgData[counter++];
			
			if(counter < psMsgObject->ui32MsgLen)
			{
				(*((address)(ui32Base + offset))) |= ((uint32_t)(psMsgObject->pui8MsgData[counter++]) << 8);
			}
		}
	}
	// Set the command register.
	(*((address)(ui32Base + IF1CMSK_R))) = cmdMask_16V;
	// Set the mask registers.
	(*((address)(ui32Base + IF1MSK1_R))) = mask0_16V;
	(*((address)(ui32Base + IF1MSK2_R))) = mask1_16V;
	// Set the arbitration registers.
	(*((address)(ui32Base + IF1ARB1_R))) = arb0_16V;
	(*((address)(ui32Base + IF1ARB2_R))) = arb1_16V;
	// Set the message control register.
	(*((address)(ui32Base + IF1MCTL_R))) = mctl_16V;
	// Set the MNUM to transfer the selected message numbers in ui32ObjID.
	(*((address)(ui32Base + IF1CRQ_R))) = ui32ObjID & CAN_IF1CRQ_MNUM_M;
}

void CAN_Configure(uint32_t portCode, uint32_t RX_PIN, uint32_t TX_PIN, uint32_t clockNumber)
{
	//________________________________________________________________________
	// I know its ugly but I was pressed for time sorry.
	//________________________________________________________________________
	// Enable GPIO clock
	SYSCTL_RCGCGPIO_R |= portCode;
	while ((SYSCTL_PRGPIO_R & portCode) == 0);
	// enable the alternate function for can recive and can transmit the RX and TX
	GPIO_PORTE_AFSEL_R = (RX_PIN) | (TX_PIN);
	// Since the PCTL number for can is always 8 then shift it by 4*pin number to get to the right position
	GPIO_PORTE_PCTL_R = ((uint32_t)(0x8) << (RX_PIN*4)) | ((uint32_t)(0x8) << (TX_PIN*4));
	SYSCTL_RCGC0_R |= clockNumber;
}
void CAN_Configure_Mode(uint32_t ui32Base, testingMode canMode, bool enable)
{
	// The enable parameter is used to check if you want to enable or disable the test modes
	// To use any mode the test bit in the ctl register must be enabled
	(*((address)(ui32Base + CTL_R))) |= CAN_CTL_TEST;
	switch(canMode)
		{
				case testMode:
				{
					if(!enable)
					{
						(*((address)(ui32Base + CTL_R))) &= ~CAN_CTL_TEST;
					}
					break;
				}
				case silentMode:
				{
					if(enable)
					// Enable the silent mode through the silent bit in the TST register.
						(*((address)(ui32Base + TST_R))) |= CAN_TST_SILENT;
					else
						(*((address)(ui32Base + TST_R))) &= ~CAN_TST_SILENT;
					break;
				}
				case loopbackMode:
				{
					if(enable)
						// Enable the loopback mode through the LBACK bit in TST register.
						(*((address)(ui32Base + TST_R))) |= CAN_TST_LBACK;
					else
						(*((address)(ui32Base + TST_R))) &= ~CAN_TST_LBACK;
					break;
				}
				case loopbackSilentMode:
				{
					if(enable)
						// Enable the loopbackSilentMode by combining the privous two steps.
						(*((address)(ui32Base + TST_R))) |= CAN_TST_LBACK | CAN_TST_SILENT;
					else
						(*((address)(ui32Base + TST_R))) |= ~(CAN_TST_LBACK | CAN_TST_SILENT);
					break;
				}
				case BasicMode:
				{
					if(enable)
						// Enable the basic mode through the BASIC bit in the TST register.
						(*((address)(ui32Base + TST_R))) |= CAN_TST_BASIC;
					else
						(*((address)(ui32Base + TST_R))) &= ~CAN_TST_BASIC;
					break;
				}
				
				default:
				{
					break;
				}
		}
}

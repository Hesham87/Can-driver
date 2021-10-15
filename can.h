#include <stdbool.h>
#include <stdint.h>
//_______________________________________________________________

//Flags

//________________________________________________________________


// This flag means that transmit interrupts are enabled.

#define MSG_OBJ_TX_INT_ENABLE   0x00000001

// This flag means that receive interrupts are enabled.

#define MSG_OBJ_RX_INT_ENABLE   0x00000002

// This flag means that a message object is using an extended identifier.

#define MSG_OBJ_EXTENDED_ID     0x00000004

// This flag means that filtering is used based on the object's message identifier.

#define MSG_OBJ_USE_ID_FILTER   0x00000008

// This flag means that new data was available in the message object.

#define MSG_OBJ_NEW_DATA        0x00000080

// This flag means that a new message was stored when NEWDAT was set, and the CPU has lost a message.

#define MSG_OBJ_DATA_LOST       0x00000100

// This flag means that a message object uses or is using filtering based on the direction of the transfer.

#define MSG_OBJ_USE_DIR_FILTER  (0x00000010 | MSG_OBJ_USE_ID_FILTER)

// This flag means that a message object is using message identifier filtering based on the extended identifier.

#define MSG_OBJ_USE_EXT_FILTER  (0x00000020 | MSG_OBJ_USE_ID_FILTER)

// This flag means that a message object is a remote frame.

#define MSG_OBJ_REMOTE_FRAME    0x00000040

// This flag means that this message object is part of a FIFO structure and not a single message nor the last message in FIFO.

#define MSG_OBJ_FIFO            0x00000200

// This flag means that a message object has no flags set.

#define MSG_OBJ_NO_FLAGS        0x00000000


// This define is used with the flag values to allow checking only status flags and not configuration flags.


#define MSG_OBJ_STATUS_MASK     (MSG_OBJ_NEW_DATA | MSG_OBJ_DATA_LOST)


//________________________________________________________________________________

// The following structer encapsulates nearly all features of a message object.

//________________________________________________________________________________
typedef struct
{
    // The CAN message identifier.
	
    uint32_t ui32MsgID;

    // The message identifier mask.

    uint32_t ui32MsgIDMask;

    // Holds nearly all the flags of the message object..
	
    uint32_t ui32Flags;

    // Holds the number of bytes in the message objes
	
    uint32_t ui32MsgLen;

    // A pointer that points to the bytes of data held in message objects (nearly equivelant to array of bytes).
		
    uint8_t *pui8MsgData;
}tCANMsgObject;

//___________________________________________________________________________________________________________________________

// This structure is used for encapsulating the values associated with setting up the bit timing for a CAN controller.

//____________________________________________________________________________________________________________________________
typedef struct
{
    // This value holds the sum of the Synchronization, Propagation, and Phase
    // Buffer 1 segments, measured in time quanta.  The valid values for this setting range from 2 to 16.
	
    uint32_t ui32SyncPropPhase1Seg;
	
    // This value holds the Phase Buffer 2 segment in time quanta.
	
    uint32_t ui32Phase2Seg;

    //! This value holds the Resynchronization Jump Width in time quanta.
	
    uint32_t ui32SJW;

    //! This value holds the CAN_CLK divider used to determine time quanta.
	
    uint32_t ui32QuantumPrescaler;
}tCANBitClkParms;

//_______________________________________________________________________________________________

//The following enum provides you with the types of interrupt status that you can read

//_______________________________________________________________________________________________

typedef enum
{
    // Read the CAN interrupt status information.
	
    CAN_INT_STS_CAUSE,

    // Read the status interrupt of a message object.
	
    CAN_INT_STS_OBJECT
}tCANIntStsReg;

//___________________________________________________________________________________________________

// The following enum provides you with the status of some bits in the message control register (MCTL).

//__________________________________________________________________________________________________

typedef enum
{

    // Read the full CAN controller status.

    CAN_STS_CONTROL,

    // Read all message objects with a transmit request set.

    CAN_STS_TXREQUEST,

    // Read all message objects with new data available.
	
    CAN_STS_NEWDAT,

    // Read all valid message objects.
	
    CAN_STS_MSGVAL
}tCANStsReg;

//______________________________________________________________________________________________

//Interrupt flags

//______________________________________________________________________________________________

// allows the generation of error interrupts.

#define CAN_INT_ERROR           0x00000008

// allows the generation of status interrupts.

#define CAN_INT_STATUS          0x00000004

// allows the genearation of any type of interrupt.

#define CAN_INT_MASTER          0x00000002
//______________________________________________________________________________________________

//tMsgObjType determines the type of message object.

typedef enum
{

    // Transmit message object.

    MSG_OBJ_TYPE_TX,

    // Transmit remote request message object

    MSG_OBJ_TYPE_TX_REMOTE,

    // Receive message object.

    MSG_OBJ_TYPE_RX,

    // Receive remote request message object.
	
    MSG_OBJ_TYPE_RX_REMOTE,

    // Remote frame receive remote, with auto-transmit message object.

    MSG_OBJ_TYPE_RXTX_REMOTE
}tMsgObjType;



//_______________________________________________________________________

//Modes

//_______________________________________________________________________
typedef enum
{
	// Activate test mode
	
	testMode,

    // Activate silent mode.

    silentMode,

    // Activate loopback mode.

    loopbackMode,

    // Activate loopback and silent mode.
	
    loopbackSilentMode,

    // Activate basic mode.

    BasicMode
}testingMode;


//__________________________________________________________________

//Errors

//___________________________________________________________________

// Bus Off state bit.

#define CAN_STATUS_BUS_OFF      0x00000080

// warnning error bit.

#define CAN_STATUS_EWARN        0x00000040

// Paaive error bit.

#define CAN_STATUS_EPASS        0x00000020

// Successgul recieption bit.

#define CAN_STATUS_RXOK         0x00000010

// Successful transmition bit.

#define CAN_STATUS_TXOK         0x00000008

// Mask for the last error code field.

#define CAN_STATUS_LEC_MSK      0x00000007

// No error.

#define CAN_STATUS_LEC_NONE     0x00000000

// Bit stuffing error.

#define CAN_STATUS_LEC_STUFF    0x00000001

// Formatting error.

#define CAN_STATUS_LEC_FORM     0x00000002

// Acknowledge error.

#define CAN_STATUS_LEC_ACK      0x00000003

// Bit1 error.

#define CAN_STATUS_LEC_BIT1     0x00000004

// Bit0 error.

#define CAN_STATUS_LEC_BIT0     0x00000005

// CRC error.

#define CAN_STATUS_LEC_CRC      0x00000006

// Last Error Code (LEC).

#define CAN_STATUS_LEC_MASK     0x00000007

//____________________________________________________________________

// This is the maximum number 11bit Message identifier.

#define CAN_11BIT_ID_MAX    0x7ff
//__________________________________________________________________________________________

extern void CAN_Enable(uint32_t ui32Base);
extern void CAN_Disable(uint32_t ui32Base);
extern void CAN_Bit_Timing_Get(uint32_t ui32Base, tCANBitClkParms *psClkParms);
extern void CAN_Bit_Timing_Set(uint32_t ui32Base, tCANBitClkParms *psClkParms);
extern bool CAN_Err_Cntr_Get(uint32_t ui32Base, uint32_t *pui32RxCount, uint32_t *pui32TxCount);
extern void CAN_Init(uint32_t ui32Base);
extern void CAN_Int_Clear(uint32_t ui32Base, uint32_t ui32IntClr);
extern void CAN_Int_Disable(uint32_t ui32Base, uint32_t ui32IntFlags);
extern void CAN_Int_Enable(uint32_t ui32Base, uint32_t ui32IntFlags);
extern uint32_t CAN_Int_Status(uint32_t ui32Base, tCANIntStsReg eIntStsReg);
extern void CAN_Message_Clear(uint32_t ui32Base, uint32_t ui32ObjID);
extern void CAN_Message_Get(uint32_t ui32Base, uint32_t ui32ObjID, tCANMsgObject *psMsgObject, bool bClrPendingInt);
extern void CAN_Message_Set(uint32_t ui32Base, uint32_t ui32ObjID, tCANMsgObject *psMsgObject, tMsgObjType eMsgType);
extern bool CAN_Retry_Get(uint32_t ui32Base);
extern void CAN_Retry_Set(uint32_t ui32Base, bool bAutoRetry);
extern uint32_t CAN_Status_Get(uint32_t ui32Base, tCANStsReg eStatusReg);
extern void CAN_Configure(uint32_t portCode, uint32_t RX_PIN, uint32_t TX_PIN, uint32_t clockNumber);
extern void CAN_Configure_Mode(uint32_t ui32Base, testingMode canMode);

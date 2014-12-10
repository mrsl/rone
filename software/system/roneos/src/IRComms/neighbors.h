/**
 * @file neighbors.h
 *
 * @brief Used to maintain information about network neighbors, sets up data storage, and  sets up
 * callbacks.
 *
 * Initializing neighbors initializes the neighbor period, neighbor timeout, obstacle timeout.
 * Also initializes neighborData, sets the message length, and puts the 7-bit roneID in message.
 * In addition, semaphore implementing neighborsMutex is created.
 *
 * @since March 2, 2011
 * @author James McLurkin
 */

#ifndef NEIGHBORS_H_
#define NEIGHBORS_H_

/******** Defines ********/

#define NEIGHBOR_PERIOD_DEFAULT 		500
#define NEIGHBOR_TIMEOUT_ROUNDS			4
#define OBSTACLE_TIMEOUT_ROUNDS			2
#define NEIGHBOR_MAX					15					//Defualt is 10

/*
 * TODO: IR: Do these really need to be removed???? It seems like they are kinda necessary
 * and I don't know what would replace them.
 */
//#define NEIGHBOR_MESSAGE_TIME_3BYTES	34
//#define NEIGHBOR_MESSAGE_TIME_4BYTES	51
//#define NEIGHBOR_XMIT_MIN_DELAY	    	(NEIGHBOR_MESSAGE_TIME_4BYTES + 10)
/* REMOVES END */
#define NEIGHBOR_XMIT_MIN_DELAY	    	50

#define	ROBOT_ID_NULL					0
#define	ROBOT_ID_MIN					90
#define	ROBOT_ID_MAX					173
#define	ROBOT_ID_ALL					0xFF

#define	MAX_PARENT_ID					0xFF

// nbr radio messages - low-level system that underpins nbrnbr data and nbrData messages
#define NBR_RADIO_MESSAGES_MAX   				16
#define NBR_RADIO_MESSAGES_SENDER_ID_IDX		0
#define NBR_RADIO_MESSAGES_LENGTH_IDX			1
#define NBR_RADIO_MESSAGES_DATA_IDX				2
#define NBR_RADIO_MESSAGE_DATA_LENGTH 			(RADIO_COMMAND_MESSAGE_DATA_LENGTH - NBR_RADIO_MESSAGES_DATA_IDX)

// nbr data messages
#define NBR_DATA_IR_IDX_MAX   			7
#define NBR_DATA_IR_BITS_MAX  			(IR_COMMS_MESSAGE_BIT_LENGTH_MAX)

#define NBR_DATA_RF_PACKETS_MAX   		8
#define NBR_DATA_RF_IDX_PER_PACKET		NBR_RADIO_MESSAGE_DATA_LENGTH
#define NBR_DATA_RF_IDX_MAX   			(NBR_RADIO_MESSAGE_DATA_LENGTH * NBR_DATA_RF_PACKETS_MAX)
#define NBR_DATA_RF_BITS_MAX  			(NBR_DATA_RF_IDX_MAX * 8)

#define NBR_DATA_TYPE_IR		0
#define NBR_DATA_TYPE_RF		1

#define NBR_ANGLE_HISTORY				3
#define NBR_ANGLE_MAX_DELTA_PER_ROUND	(MILLIRAD_PI / 4)

/******** Structs ********/


/**
 * @brief Linked list of data on messages.
 */
typedef struct NbrData {
	const char * name;		/**< Name for the data stored*/
	uint8 value;			/**< Value of the stored data*/
	uint8 size;				/**< Size of the list*/
	uint8 idx;				/**< Index of the data*/
	uint8 type;				/**< Type of data stored*/
	struct NbrData* nextPtr;/**< Pointer to next NbrData in linked list*/
} NbrData;



// public put in header file
typedef struct NbrDataFloat {
	NbrData msgFloat0;
	NbrData msgFloat1;
	NbrData msgFloat2;
	NbrData msgFloat3;
} NbrDataFloat;

/**
 * @brief The IR message a neighbor has.
 */
//TODO Add field documentation
typedef struct NbrMsgRadioNbrData {
	uint8 ID;									/**< */
	uint32 timeStamp;							/**< */
	uint8 length;								/**< */
	uint8 data[NBR_RADIO_MESSAGE_DATA_LENGTH];	/**< */
} NbrMsgRadioNbrData;


/**
 * @brief Linked list of a neighbor's messages.
 */
//TODO add field documentation
typedef struct NbrMsgRadio {
	const char* name;
	uint8 length;
	uint8 data[NBR_RADIO_MESSAGE_DATA_LENGTH];
	NbrMsgRadioNbrData dataNbr[NEIGHBOR_MAX];
	RadioCmd radioCmd;
	struct NbrMsgRadio* nextPtr;
} NbrMsgRadio;


/**
 * @brief Information stored on a network neighbor.
 */
//TODO Fill in missing field documentation
typedef struct Nbr {
	uint8 ID;												/**< Robot ID*/
	int16 bearing;											/**< Robot bearing*/
	int16 orientation;										/**< Robot orientation*/
	int16 range;											/**< */
	int16 bearingHistory[NBR_ANGLE_HISTORY];				/**< */
	int16 orientationHistory[NBR_ANGLE_HISTORY];			/**< */
	boolean orientationValid;								/**< */
	uint32 updateTime;										/**< */
	uint32 updateRound;										/**< Current round of the robot*/
	uint8 orientationsBitsMatrix[IR_COMMS_NUM_OF_RECEIVERS];/**< */
	uint32 rangeBits;										/**< */	//### 32-bit
	uint8 receiverBits;										/**< */
	uint8 orientationBits;									/**< */
	uint8 signalBits;										/**< */
	boolean active;											/**< */
	uint8 messages[NBR_DATA_IR_IDX_MAX];					/**< */
	NbrMsgRadioNbrData* radioMessagePtr;					/**< */
} Nbr;



/**
 * @brief Array containing data on all the network neighbors.
 */
//TODO Add field documentation
typedef struct NbrDatabase {
	Nbr nbrs[NEIGHBOR_MAX];						/**< */
	uint8 nbrsSize;								/**< */
	uint32 round;								/**< */
	char message[IR_COMMS_MESSAGE_LENGTH_MAX];	/**< */
} NbrDatabase;


//TODO: Double check brief
/**
 * @brief Array containing pointers to all the network neighbors' data.
 */
//TODO Add field documentation
typedef struct NbrList {
	Nbr* nbrs[NEIGHBOR_MAX];	/**< */
	uint8 size;					/**< */
} NbrList;


/**
 * @brief Data from IR signal
 */
//TODO add field documentation
typedef struct IRRangeData {
	uint8 orientationsBitsMatrix[IR_COMMS_NUM_OF_RECEIVERS];	/**< */
	uint32 rangeBits;				/**< */	//### 32-bit
	uint8 receiverBits;											/**< */
	uint8 rangeBitCount;										/**< */
	uint32 updateRound;											/**< */
} IRRangeData;



/******** Functions ********/


/**
 * @brief Initialize neighbors and start neighbors task.
 *
 * @param neighbor_period the neighbor period in rounds
 * @returns void
 */
void neighborsInit(uint32 neighbor_period);


/**
 * @brief Disable neighbor xmit/recv.
 *
 * @returns void
 */
void neighborsDisable(void);


/**
 * @brief Enable neighbor to transmit messages.
 *
 * @param neighbor_xmit_enable_arg a boolean that allows enable or not
 * @returns void
 */
void neighborsXmitEnable(boolean neighbor_xmit_enable_arg);


/**
 * @brief Set neighbor period, neighbor timeout, and obstacle timeout proportional to argument.
 *
 * @param neighbor_period_arg the neighbor period length in rounds
 * @returns void
 */
void neighborsSetPeriod(uint32 neighbor_period_arg);


/*
 * @brief Set time constant of neighbor range IIR filter
 *
 * @param Angle filter time constant. max is 100.  Is based on x/100.  10 is slow, 90 is fast
 * @param Range filter time constant. max is 100.  Is based on x/100.  10 is slow, 90 is fast
 * @returns void
 */
void neighborsSetFilterTimeConstants(int32 anglesTimeConstant, int32 rangeTimeConstant);


/**
 * @brief Set neighbor period, neighbor timeout, and obstacle timeout proportional to argument.
 *
 * @param timeoutRounds
 * @param minActive
 * @param maxInactive
 * @returns void
 */
void neighborsSetTimeoutRounds(uint8 timeoutRounds, uint8 minActive, uint8 maxInactive);


/**
 * @brief Get neighbor period.
 *
 * @returns void
 */
uint32 neighborsGetPeriod(void);


//TODO: Should this stay in the header file?
void neighborsAddReceiveCallback(void(*receiveCallbackArg)(NbrDatabase* ndPtr));


/**
 * @brief Get neighbors mutex.
 *
 * @returns void
 */
void neighborsGetMutex(void);


/*
 * @brief Get neighbors mutex with specified max delay.
 *
 * @returns void
 */
signed neighborsGetMutexDelay(unsigned long delay);


/**
 * @brief Put neighbors mutex.
 *
 * @returns void
 */
void neighborsPutMutex(void);


/**
 * @brief Get neighbor round from neighbor data.
 *
 * @return neighbor round
 */
uint32 neighborsGetRound(void);


/**
 * @brief Check to see if there is a new neighbor round.  Updates the variable at the pointer.
 *
 * @param roundOldPtr pointer for previous round
 * @return TRUE if the neighbor round has changed
 */
boolean neighborsNewRoundCheck(uint32* roundOldPtr);


/**
 * @brief Tries add neighborID to list of neighbors to ignore
 *
 * @param neighborID the neighbor we want to ignore (no longer monitor)
 * @returns void
 */
void neighborsIgnore(uint8 neighborID);

/**
 * @brief Get IR obstacle bits.
 *
 * @returns IR obstacle bits
 */
uint8 irObstaclesGetBits(void);


/**
 * @brief Get IR obstacle range bits.
 *
 * @returns IR obstacle range bits
 */
uint32 irObstaclesGetRangeBits(void);	//### 32-bit


/**
 * @brief Get IR obstacle bit matrix.
 *
 * @returns IR obstacle bit matrix
 */
uint8* irObstaclesGetBitMatrix(void);


/**
 * @brief Get IR obstacle bearing.
 *
 * This function returns the bearing of an IR obstacle.  It looks for contiguous
 * obstacle bits, to select the largest (nearest) obstacle.  This could cause dithering for 1-bit
 * sized obstacles.
 * @returns the bump sensor bearing. Returns 0 if there is no obstacle.
 */
int16 irObstaclesGetBearing(void);



int8 irObstaclesGetBearingBitVector(void);

void obstacleExcludeNbrs(NbrList* nbrListPtr, uint8* obstacleBitsGroupPtr, uint8* obstacleBitsCountPtr, int16* obstacleBearingPtr);


/**
 * @brief Print the obstacle data from the IR sensors.
 *
 * @returns void
 */
void obstaclePrint(void);

uint32 pseudoRandNumGen(uint32 init, uint32 seed);


// basic nbr functions

/**
 * @brief Get neighbor ID.
 *
 * @param nbrPtr neighbor pointer
 * @returns ID
 */
uint8 nbrGetID(Nbr* nbrPtr);


/**
 * @brief Get neighbor bearing.
 *
 * @param nbrPtr neighbor pointer
 * @returns returns bearing. Ranges within [-pi, pi).
 */
int32 nbrGetBearing(Nbr* nbrPtr);


/**
 * @brief Get neighbor orientation.
 *
 * @param nbrPtr neighbor pointer
 * @returns orientation. Ranges within [-pi, pi).
 */
int32 nbrGetOrientation(Nbr* nbrPtr);


/**
 * @brief Get neighbor range.
 *
 * @param nbrPtr neighbor pointer
 * @returns range. Ranges within [0, 1200].  units are in mm
 */
int16 nbrGetRange(Nbr* nbrPtr);


/**
 * @brief Get neighbor range.
 *
 * @param nbrPtr neighbor pointer
 * @returns range. Ranges within [0, 1200].  units are in mm
 */
uint32 nbrGetRangeBits(Nbr* nbrPtr);	//32-bit


/**
 * @brief Get neighbor orientation valid.
 *
 * @param nbrPtr neighbor pointer
 * @returns whether orientation is valid
 */
boolean nbrGetOrientationValid(Nbr* nbrPtr);


/**
 * @brief Get neighbor range bits.
 *
 * Range bits are recieverBitCount + orientationBitCount	//### ???
 * @param nbrPtr neighbor pointer
 * @returns range bits
 */
uint8 nbrGetOrientationBitsCount(Nbr* nbrPtr);


/**
 * @brief Get neighbor receiver bits.
 *
 * Receiver bits are the actual receivers the message was received on.
 *
 * @param nbrPtr neighbor pointer
 * @returns receiver bits
 */
uint8 nbrGetReceiverBits(Nbr* nbrPtr);


/**
 * @brief Get neighbor transmitter bits.
 *
 * Receiver bits are the actual transmitter the message was received from.
 *
 * @param nbrPtr neighbor pointer
 * @returns transmitter bits
 */
uint8 nbrGetTransmitterBits(Nbr* nbrPtr);


/**
 * @brief Get neighbor update time.
 *
 * @param nbrPtr neighbor pointer
 * @returns update time
 */
uint32 nbrGetUpdateTime(Nbr* nbrPtr);


/**
 * @brief Get neighbor update round
 *
 * @param nbrPtr neighbor pointer
 * @returns the last round this neighbor was heard from
 */
uint32 nbrGetUpdateRound(Nbr* nbrPtr);


/**
 * @brief Print information on neighbor (and information of neighbor's neighbors).
 *
 * Print roneID and neighbor's ID, bear, orientation, orientation valid
 * Print name and value of each neighbor message.
 *
 * @param nbr neighbor pointer
 * @returns void
 */
void nbrPrint(Nbr* nbr);


/**
 * @brief Print header and neighbor data.
 *
 * Print header once.
 * Print id, time, round; neighbor's ID, bearing, update time; neighbor's neighbor's ID, bearing,
 * update time.
 *
 * @param nbr neighbor pointer
 * @param round the round number
 * @returns void
 */
void nbrPrintData(Nbr* nbr, uint32 round);


/**
 * @brief Returns true if this neighbor is a beacon.
 *
 * @param nbrPtr neighbor pointer
 * @returns true if the neighbor is a IR beacon
 */
boolean nbrIsRobot(Nbr* nbrPtr);


/**
 * @brief Returns true if this neighbor is a beacon.
 *
 * @param nbrPtr neighbor pointer
 * @returns true if the neighbor is a IR beacon
 */
boolean nbrIsBeacon(Nbr* nbrPtr);


//TODO: implement or delete?
boolean nbrIsCharger(Nbr* nbrPtr);

//TODO: Create nbrMsgRadio header file?
// nbr radio messages

/**
 * @brief Create a radio neighbor message.
 *
 * This function is called only once per program execution for each thing.
 *
 * @param nbrMsgRadioPtr the pointer to the radio message
 * @param name the name for the radio message
 * @returns void
 */
void nbrMsgRadioCreate(NbrMsgRadio* nbrMsgRadioPtr, const char* name);


/**
 * @brief Gets a radio neighbor message.
 *
 * This function returns a pointer to the NbrMsgRadioNbrData struct for this neighbor.
 *
 * @param nbrMsgRadioPtr the pointer to the radio message
 * @param dataPtr pointer to a char array of max length RADIO_COMMAND_MESSAGE_DATA_LENGTH
 * @param length
 * @returns pointer to data or NULL
 */
void nbrMsgRadioSet(NbrMsgRadio* nbrMsgRadioPtr, char* dataPtr, uint8 length);


/**
 * @brief Gets a radio neighbor message.
 *
 * This function returns a pointer to the NbrMsgRadioNbrData struct for this neighbor.
 *
 * @param nbrMsgRadioPtr the pointer to the radio message
 * @param nbrPtr pointer to the neighbor
 * @returns pointer to data or NULL
 */
NbrMsgRadioNbrData* nbrMsgRadioGetNbr(NbrMsgRadio* nbrMsgRadioPtr, Nbr* nbrPtr);


/**
 *  @brief Prints the message data for the input neighbor.
 *
 *  @param nbrMsgRadioPtr pointer for the linked list of a neighbor's messages
 *  @param nbrPtr pointer for the desired neighbor
 *  @returns void
 */
void nbrMsgRadioPrint(NbrMsgRadio* nbrMsgRadioPtr, Nbr* nbrPtr);


// low-level function.  Used internally TODO: so should it be removed from the header file?
Nbr* nbrsGetWithID(uint8 nbrID);

void nbrListCreate(NbrList* nbrListPtr);

void serialCmdSNFunc(char *command);
#endif /* NEIGHBORS_H_ */

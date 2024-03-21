#include "grf_ctrl.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */



/**
 *------------------------------------------------------------
 *                           VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskObj_t grfCtrlTask;

GRF_Data_t GrfDataObj;


static uint16_t* rawGRF = {0}; 	 // 0,1,2 => S2, 3,4,5 => S3, 6,7,8 => S4

// Loop Time Count //
static uint32_t grfCtrlLoopCnt;
static float grfCtrlTimeElap;

uint8_t grfCtrlState = 0;

static uint8_t grfBoardNodeID;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ------------------- INITIALIZATION ------------------- */
static int GRFNodeIDCheck(uint8_t directionSet);

/* ------------------- ROUTINE ------------------- */
//static int RunGetIMUFunction(void);

/* ------------------- STATIC FUNCTIONS ------------------- */
/* Functions for Grf Parameters */
static int GetRawGRF(GRF_Data_t* grfDataObj);
static uint16_t PositiveDef(int num);
/* ------------------- SDO CALLBACK ------------------- */
//static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(grfCtrlTask)

void InitGrfCtrl(void)
{
    InitTask(&grfCtrlTask);

	/* Start DMA ADC1 for GRF Sensor */
	if(IOIF_StartADCDMA(IOIF_ADC1, &rawGRF, IOIF_ADC1_BUFFER_LENGTH)) {
		//TODO: Error Process
	}

    /* Checking Node ID */
    grfBoardNodeID = GRFNodeIDCheck(0);			// 0 : Right, 1 : LEFT

	/* State Definition */
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
//	TASK_CREATE_ROUTINE(&grfCtrlTask, ROUTINE_ID_GRF_TOTAL_FUNCTION, 		NULL, 	RunGetIMUFunction,	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_GRF);

	// PDO
	/* For SUIT PDO setting */
	DOP_CreatePDO(TASK_ID_GRF, 	 PDO_ID_GRF_S2X,		DOP_FLOAT32,	1,    &GrfDataObj.voltS2[0]);
	DOP_CreatePDO(TASK_ID_GRF, 	 PDO_ID_GRF_S2Y,		DOP_FLOAT32,	1,    &GrfDataObj.voltS2[1]);
	DOP_CreatePDO(TASK_ID_GRF, 	 PDO_ID_GRF_S2Z,		DOP_FLOAT32,	1,    &GrfDataObj.voltS2[2]);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_GRF)
	// DOP_CreateSDO(TASK_ID_GRF, 	SDO_ID_WIDM_SET_ABSOFFSET_CMD, 	DOP_UINT8, 	SetAbsOffsetCmd);


	/* Timer Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM3) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM3, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunGrfCtrl, NULL);
}

void RunGrfCtrl(void* params)
{
	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	RunTask(&grfCtrlTask);

	/* Elapsed Time Check */
	grfCtrlTimeElap = DWT->CYCCNT / 160;	// in microsecond
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	/* Calibrate offset of the GRF sensors */
	static uint16_t calibNum = 0;
	static uint32_t ArrOffset[9] = {0};
	static uint8_t errCnt = 0;

	if (calibNum < 1000) {
		ArrOffset[0] += rawGRF[0];
		ArrOffset[1] += rawGRF[1];
		ArrOffset[2] += rawGRF[2];

		ArrOffset[3] += rawGRF[3];
		ArrOffset[4] += rawGRF[4];
		ArrOffset[5] += rawGRF[5];

		ArrOffset[6] += rawGRF[6];
		ArrOffset[7] += rawGRF[7];
		ArrOffset[8] += rawGRF[8];

		calibNum++;
	}

	else if (calibNum == 1000){
		for (uint8_t i = 0; i < 9; i++) {
			if (ArrOffset[i] / 1000 > 32768) {
				calibNum = 0;
				errCnt++;
			}
		}

		if (errCnt == 0){
			GrfDataObj.rawS2offset[0] = ArrOffset[0] / 1000;
			GrfDataObj.rawS2offset[1] = ArrOffset[1] / 1000;
			GrfDataObj.rawS2offset[2] = ArrOffset[2] / 1000;

			GrfDataObj.rawS3offset[0] = ArrOffset[3] / 1000;
			GrfDataObj.rawS3offset[1] = ArrOffset[4] / 1000;
			GrfDataObj.rawS3offset[2] = ArrOffset[5] / 1000;

			GrfDataObj.rawS4offset[0] = ArrOffset[6] / 1000;
			GrfDataObj.rawS4offset[1] = ArrOffset[7] / 1000;
			GrfDataObj.rawS4offset[2] = ArrOffset[8] / 1000;

			StateTransition(&grfCtrlTask.stateMachine, TASK_STATE_STANDBY);
		}
	}
}

static void StateStandby_Run(void)
{
	GetRawGRF(&GrfDataObj);
	grfCtrlLoopCnt = 0;
}

static void StateEnable_Ent(void)
{
	EntRoutines(&grfCtrlTask.routine);
//	StateTransition(&grfCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Run(void)
{
	RunRoutines(&grfCtrlTask.routine);
	grfCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    ExtRoutines(&grfCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Module & Mode Selection Functions */
static int GRFNodeIDCheck(uint8_t directionSet)
{
	int nodeID = 0;
	if (directionSet == 0){			// RIGHT GRF
		nodeID = 14;
	}
	else if (directionSet == 1){	// LEFT GRF
		nodeID = 15;
	}
	return nodeID;
}

static uint16_t PositiveDef(int num)
{
	if (0 < num && num < 65536) {
		return (uint16_t)num;
	}
	else {
		return (uint16_t)0;
	}
}


/* Just get 3-GRF Sensor values */
static int GetRawGRF(GRF_Data_t* grfDataObj)
{
	/* Get raw values of S2,S3,S4 */
	// S2 - x,y,z //
	grfDataObj->rawS2[0] = rawGRF[0];
	grfDataObj->rawS2[1] = rawGRF[1];
	grfDataObj->rawS2[2] = rawGRF[2];

	// S3 - x,y,z //
	grfDataObj->rawS3[0] = rawGRF[3];
	grfDataObj->rawS3[1] = rawGRF[4];
	grfDataObj->rawS3[2] = rawGRF[5];

	// S4 - x,y,z //
	grfDataObj->rawS4[0] = rawGRF[6];
	grfDataObj->rawS4[1] = rawGRF[7];
	grfDataObj->rawS4[2] = rawGRF[8];


	/* Calculate (0 ~ 3.3V) and compensate offset of S2,S3,S4 */
	// S2 - x,y,z //
	grfDataObj->voltS2[0] = (float)(PositiveDef(grfDataObj->rawS2[0] - grfDataObj->rawS2offset[0]) * 3.3 / 16383);
	grfDataObj->voltS2[1] = (float)(PositiveDef(grfDataObj->rawS2[1] - grfDataObj->rawS2offset[1]) * 3.3 / 16383);
	grfDataObj->voltS2[2] = (float)(PositiveDef(grfDataObj->rawS2[2] - grfDataObj->rawS2offset[2]) * 3.3 / 16383);

	// S3 - x,y,z //
	grfDataObj->voltS3[0] = (float)(PositiveDef(grfDataObj->rawS3[0] - grfDataObj->rawS3offset[0]) * 3.3 / 16383);
	grfDataObj->voltS3[1] = (float)(PositiveDef(grfDataObj->rawS3[1] - grfDataObj->rawS3offset[1]) * 3.3 / 16383);
	grfDataObj->voltS3[2] = (float)(PositiveDef(grfDataObj->rawS3[2] - grfDataObj->rawS3offset[2]) * 3.3 / 16383);

	// S4 - x,y,z //
	grfDataObj->voltS4[0] = (float)(PositiveDef(grfDataObj->rawS4[0] - grfDataObj->rawS4offset[0]) * 3.3 / 16383);
	grfDataObj->voltS4[1] = (float)(PositiveDef(grfDataObj->rawS4[1] - grfDataObj->rawS4offset[1]) * 3.3 / 16383);
	grfDataObj->voltS4[2] = (float)(PositiveDef(grfDataObj->rawS4[2] - grfDataObj->rawS4offset[2]) * 3.3 / 16383);

	return 0;
}





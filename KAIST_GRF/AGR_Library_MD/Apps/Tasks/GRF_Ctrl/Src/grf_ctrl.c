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


float Calib_0903[3][3] = {{1.0728, -0.00330, -0.04320},
		{0.06095, 1.10683, -0.02624},
		{0.01283, 0.01208, 1.00197}};
float Calib_0904[3][3] = {{1.06284, -0.12484, -0.01295},
		{0.12809, 1.05286, 0.05660},
		{0.00172, -0.00291, 1.00636}};
float Calib_0905[3][3] = {{1.05861, -0.02645, -0.00525},
		{-0.00134, 1.09504, 0.00598},
		{0.01172, 0.00004, 1.00222}};
float Calib_0906[3][3] = {{1.06449, -0.06041, -0.01800},
		{0.02641, 1.12496, 0.00915},
		{0.00552, -0.00136, 0.99892}};
float Calib_0907[3][3] = {{1.04797, -0.00971, -0.00643},
		{0.00106, 1.11383, -0.00350},
		{0.00880, -0.01241, 0.99442}};
float Calib_0908[3][3] = {{1.07688, -0.10547, 0.00026},
		{0.02277, 1.08720, 0.00002},
		{0.00655, 0.00128, 1.00846}};
float Calib_0909[3][3] = {{1.09278, -0.03905, 0.01048},
		{0.04087, 1.06463, 0.01664},
		{0.01168, 0.00382, 1.00054}};
float Calib_1004[3][3] = {{1.06967, -0.06493, 0.01491},
		{0.03158, 1.10222, 0.00448},
		{-0.00064, 0.02465, 1.00046}};
float Calib_1005[3][3] = {{1.03857, -0.06545, 0.00511},
		{0.08713, 1.14248, 0.02753},
		{-0.00210, 0.00409, 1.00105}};
float Calib_1006[3][3] = {{1.06996, -0.02724, -0.02762},
		{0.01611, 1.09007, -0.02448},
		{0.01165, 0.00692, 0.99176}};

float strainS2[3] = {0};
float strainS3[3] = {0};
float strainS4[3] = {0};


/* Have to change for Flash Memory */
// Right GRF //
float S2X_R_offset = 0;
float S2Y_R_offset = 0;
float S2Z_R_offset = 0;

float S3X_R_offset = 0;
float S3Y_R_offset = 0;
float S3Z_R_offset = 0;

float S4X_R_offset = 0;
float S4Y_R_offset = 0;
float S4Z_R_offset = 0;

// Left GRF //
float S2X_L_offset = 0;
float S2Y_L_offset = 0;
float S2Z_L_offset = 0;

float S3X_L_offset = 0;
float S3Y_L_offset = 0;
float S3Z_L_offset = 0;

float S4X_L_offset = 0;
float S4Y_L_offset = 0;
float S4Z_L_offset = 0;


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
static int GRFNodeIDCheck(void);

/* ------------------- ROUTINE ------------------- */
//static int RunGetIMUFunction(void);

/* ------------------- STATIC FUNCTIONS ------------------- */
/* Functions for Grf Parameters */
static int GetRawGRF(GRF_Data_t* grfDataObj);

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
    grfBoardNodeID = GRFNodeIDCheck();			// 0 : Right, 1 : LEFT

	/* State Definition */
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
//	TASK_CREATE_ROUTINE(&grfCtrlTask, ROUTINE_ID_GRF_TOTAL_FUNCTION, 		NULL, 	RunGetIMUFunction,	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_EXTDEV);

	// PDO
	/* For SUIT PDO setting */
#ifdef GRF_RIGHT
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S2X_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S2[0]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S2Y_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S2[1]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S2Z_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S2[2]);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S3X_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S3[0]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S3Y_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S3[1]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S3Z_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S3[2]);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S4X_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S4[0]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S4Y_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S4[1]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S4Z_RIGHT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S4[2]);
#endif

#ifdef GRF_LEFT
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S2X_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S2[0]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S2Y_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S2[1]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S2Z_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S2[2]);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S3X_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S3[0]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S3Y_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S3[1]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S3Z_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S3[2]);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S4X_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S4[0]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S4Y_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S4[1]);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_S4Z_LEFT,		DOP_FLOAT32,	1,    &GrfDataObj.F_S4[2]);
#endif

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_EXTDEV)
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
static int GRFNodeIDCheck(void)
{
	int nodeID = 0;
#ifdef GRF_RIGHT
	nodeID = 14;
#endif

#ifdef GRF_LEFT
	nodeID = 15;
#endif
	return nodeID;
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
	grfDataObj->voltS2[0] = (float)(((int32_t)grfDataObj->rawS2[0] - (int32_t)grfDataObj->rawS2offset[0]) * 3.3 / 16383);
	grfDataObj->voltS2[1] = (float)(((int32_t)grfDataObj->rawS2[1] - (int32_t)grfDataObj->rawS2offset[1]) * 3.3 / 16383);
	grfDataObj->voltS2[2] = (float)(((int32_t)grfDataObj->rawS2[2] - (int32_t)grfDataObj->rawS2offset[2]) * 3.3 / 16383);

	// S3 - x,y,z //
	grfDataObj->voltS3[0] = (float)(((int32_t)grfDataObj->rawS3[0] - (int32_t)grfDataObj->rawS3offset[0]) * 3.3 / 16383);
	grfDataObj->voltS3[1] = (float)(((int32_t)grfDataObj->rawS3[1] - (int32_t)grfDataObj->rawS3offset[1]) * 3.3 / 16383);
	grfDataObj->voltS3[2] = (float)(((int32_t)grfDataObj->rawS3[2] - (int32_t)grfDataObj->rawS3offset[2]) * 3.3 / 16383);

	// S4 - x,y,z //
	grfDataObj->voltS4[0] = (float)(((int32_t)grfDataObj->rawS4[0] - (int32_t)grfDataObj->rawS4offset[0]) * 3.3 / 16383);
	grfDataObj->voltS4[1] = (float)(((int32_t)grfDataObj->rawS4[1] - (int32_t)grfDataObj->rawS4offset[1]) * 3.3 / 16383);
	grfDataObj->voltS4[2] = (float)(((int32_t)grfDataObj->rawS4[2] - (int32_t)grfDataObj->rawS4offset[2]) * 3.3 / 16383);



	/* Calculate Voltage to Strain */
	strainS2[0] = grfDataObj->voltS2[0] * Volt2strain;
	strainS2[1] = grfDataObj->voltS2[1] * Volt2strain;
	strainS2[2] = grfDataObj->voltS2[2] * Volt2strain;
	strainS3[0] = grfDataObj->voltS3[0] * Volt2strain;
	strainS3[1] = grfDataObj->voltS3[1] * Volt2strain;
	strainS3[2] = grfDataObj->voltS3[2] * Volt2strain;
	strainS4[0] = grfDataObj->voltS4[0] * Volt2strain;
	strainS4[1] = grfDataObj->voltS4[1] * Volt2strain;
	strainS4[2] = grfDataObj->voltS4[2] * Volt2strain;



	/* Calculate the Force (Choose proper CalibMatrix */
	for (uint8_t i = 0; i < 3; i++){
		grfDataObj->F_S2[i] = 0;
		grfDataObj->F_S3[i] = 0;
		grfDataObj->F_S4[i] = 0;
		for (uint8_t j = 0; j < 3; j++){
			grfDataObj->F_S2[i] += Calib_0909[i][j] * strainS2[j];
			grfDataObj->F_S3[i] += Calib_0907[i][j] * strainS3[j];
			grfDataObj->F_S4[i] += Calib_0906[i][j] * strainS4[j];
		}
	}

	return 0;
}






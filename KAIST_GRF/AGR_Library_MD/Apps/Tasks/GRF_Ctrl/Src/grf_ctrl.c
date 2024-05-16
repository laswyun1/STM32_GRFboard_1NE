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


/* Force Sensor Positioning Vector Matrix*/
#ifdef GRF_LEFT

float RotMat_S2[3][3] = {{+1, +0, +0}, {+0, -1, +0}, {+0, +0, +1}};
float RotMat_S3[3][3] = {{+0, +1, +0}, {+1, +0, +0}, {+0, +0, +1}};
float RotMat_S4[3][3] = {{+0, -1, +0}, {-1, +0, +0}, {+0, +0, +1}};

#endif

#ifdef GRF_RIGHT

float RotMat_S2[3][3] = {{-1, +0, +0}, {+0, +1, +0}, {+0, +0, +1}};
float RotMat_S3[3][3] = {{+0, -1, +0}, {-1, +0, +0}, {+0, +0, +1}};
float RotMat_S4[3][3] = {{+0, +1, +0}, {+1, +0, +0}, {+0, +0, +1}};

#endif


float rx_F = 220;
float rx_B = 70;
float ry_F = 70;
float ry_B = 14;
float rz = -10.5;

/* ------------ TO BE TUNED ------------ */
float thresh_sensor = 4.5;
float thresh_vert = 7;
float thresh_norm = 15;
float thresh_RateLimiter = 15;
/* ------------------------------------- */

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
static int GetFiltGRF(GRF_Data_t* grfDataObj);

static int Force_Labeling(GRF_Data_t* grfDataObj);
static int Get_Total_GRF(GRF_Data_t* grfDataObj);
static int Get_Total_COP(GRF_Data_t* grfDataObj);
static int sign(double x);
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
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 false);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 true);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&grfCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
//	TASK_CREATE_ROUTINE(&grfCtrlTask, ROUTINE_ID_GRF_TOTAL_FUNCTION, 		NULL, 	RunGetIMUFunction,	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_EXTDEV);

	// PDO
	/* For PDO setting */
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_LOOP_CNT,			DOP_UINT32,	1,    &grfCtrlLoopCnt);

//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_MEDIAL_X,		DOP_FLOAT32,	1,    &GrfDataObj.F_FM_filt[0]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_MEDIAL_Y,		DOP_FLOAT32,	1,    &GrfDataObj.F_FM_filt[1]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_MEDIAL_Z,		DOP_FLOAT32,	    1,    &GrfDataObj.F_FM_filt[2]);
//
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_LATERAL_X,		DOP_FLOAT32,	    1,    &GrfDataObj.F_FL_filt[0]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_LATERAL_Y,		DOP_FLOAT32,	    1,    &GrfDataObj.F_FL_filt[1]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_LATERAL_Z,		DOP_FLOAT32,	    1,    &GrfDataObj.F_FL_filt[2]);
//
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_BACK_X,				DOP_FLOAT32,	    1,    &GrfDataObj.F_Ba_filt[0]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_BACK_Y,				DOP_FLOAT32,	    1,    &GrfDataObj.F_Ba_filt[1]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_BACK_Z,				DOP_FLOAT32,	    1,    &GrfDataObj.F_Ba_filt[2]);
//
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_MEDIAL_XYZ,	DOP_FLOAT32,	    3,    &GrfDataObj.F_FM_filt[0]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FRONT_LATERAL_XYZ,	DOP_FLOAT32,	    3,    &GrfDataObj.F_FL_filt[0]);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_BACK_XYZ,			DOP_FLOAT32,	    3,    &GrfDataObj.F_Ba_filt[0]);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_GRF_X,					DOP_FLOAT32,	    1,    &GrfDataObj.GRF_X);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_GRF_Y,					DOP_FLOAT32,	    1,    &GrfDataObj.GRF_Y);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_GRF_Z,					DOP_FLOAT32,	    1,    &GrfDataObj.GRF_Z);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_COP_X,					DOP_FLOAT32,	    1,    &GrfDataObj.COP_X);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_COP_Y,					DOP_FLOAT32,	    1,    &GrfDataObj.COP_Y);

/*
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

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_LOOPCNT,			DOP_UINT32,		1,    &grfCtrlLoopCnt);
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

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_LOOPCNT,			DOP_UINT32,		1,    &grfCtrlLoopCnt);
#endif
*/

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

}

static void StateStandby_Run(void)
{
//	GetRawGRF(&GrfDataObj);

	/* Calibrate offset of the GRF sensors */
//	static uint16_t calibNum = 0;
//	static uint32_t ArrOffset[9] = {0};
//	static uint8_t errCnt = 0;
//
//	if (calibNum < 2000) {
//		ArrOffset[0] += rawGRF[0];
//		ArrOffset[1] += rawGRF[1];
//		ArrOffset[2] += rawGRF[2];
//
//		ArrOffset[3] += rawGRF[3];
//		ArrOffset[4] += rawGRF[4];
//		ArrOffset[5] += rawGRF[5];
//
//		ArrOffset[6] += rawGRF[6];
//		ArrOffset[7] += rawGRF[7];
//		ArrOffset[8] += rawGRF[8];
//
//		calibNum++;
//	}
//
//	else if (calibNum == 2000){
//		for (uint8_t i = 0; i < 9; i++) {
//			if (ArrOffset[i] / 2000 > 32768) {
//				calibNum = 0;
//				errCnt++;
//			}
//		}
//
//		if (errCnt == 0){
//			GrfDataObj.rawS2offset[0] = ArrOffset[0] / 2000;
//			GrfDataObj.rawS2offset[1] = ArrOffset[1] / 2000;
//			GrfDataObj.rawS2offset[2] = ArrOffset[2] / 2000;
//
//			GrfDataObj.rawS3offset[0] = ArrOffset[3] / 2000;
//			GrfDataObj.rawS3offset[1] = ArrOffset[4] / 2000;
//			GrfDataObj.rawS3offset[2] = ArrOffset[5] / 2000;
//
//			GrfDataObj.rawS4offset[0] = ArrOffset[6] / 2000;
//			GrfDataObj.rawS4offset[1] = ArrOffset[7] / 2000;
//			GrfDataObj.rawS4offset[2] = ArrOffset[8] / 2000;
//
//			StateTransition(&grfCtrlTask.stateMachine, TASK_STATE_ENABLE);
//		}
//	}

//	StateTransition(&grfCtrlTask.stateMachine, TASK_STATE_ENABLE);
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
//	GetRawGRF(&GrfDataObj);

	static uint16_t calibNum = 0;
	static uint32_t ArrOffset[9] = {0};
	static uint8_t errCnt = 0;
	static uint8_t homingOK = 0;

	if (homingOK == 0){
		if (calibNum < 2000) {
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

		else if (calibNum == 2000){
			for (uint8_t i = 0; i < 9; i++) {
				if (ArrOffset[i] / 2000 > 32768) {
					calibNum = 0;
					errCnt++;
				}
			}

			if (errCnt == 0){
				GrfDataObj.rawS2offset[0] = ArrOffset[0] / 2000;
				GrfDataObj.rawS2offset[1] = ArrOffset[1] / 2000;
				GrfDataObj.rawS2offset[2] = ArrOffset[2] / 2000;

				GrfDataObj.rawS3offset[0] = ArrOffset[3] / 2000;
				GrfDataObj.rawS3offset[1] = ArrOffset[4] / 2000;
				GrfDataObj.rawS3offset[2] = ArrOffset[5] / 2000;

				GrfDataObj.rawS4offset[0] = ArrOffset[6] / 2000;
				GrfDataObj.rawS4offset[1] = ArrOffset[7] / 2000;
				GrfDataObj.rawS4offset[2] = ArrOffset[8] / 2000;
			}
			calibNum = 5000;
			homingOK = 1;
		}
	}



	GetRawGRF(&GrfDataObj);
	Force_Labeling(&GrfDataObj);
	GetFiltGRF(&GrfDataObj);
	Get_Total_GRF(&GrfDataObj);
	Get_Total_COP(&GrfDataObj);


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



//	/* For Low Pass Filtering */
//	grfDataObj->rawS2LPF[0] = 0.97*grfDataObj->rawS2LPF[0] + 0.03*grfDataObj->rawS2[0];
//	grfDataObj->rawS2LPF[1] = 0.97*grfDataObj->rawS2LPF[1] + 0.03*grfDataObj->rawS2[1];
//	grfDataObj->rawS2LPF[2] = 0.97*grfDataObj->rawS2LPF[2] + 0.03*grfDataObj->rawS2[2];
//
//	grfDataObj->rawS3LPF[0] = 0.97*grfDataObj->rawS3LPF[0] + 0.03*grfDataObj->rawS3[0];
//	grfDataObj->rawS3LPF[1] = 0.97*grfDataObj->rawS3LPF[0] + 0.03*grfDataObj->rawS3[0];
//	grfDataObj->rawS3LPF[2] = 0.97*grfDataObj->rawS3LPF[0] + 0.03*grfDataObj->rawS3[0];
//
//	grfDataObj->rawS4LPF[0] = 0.97*grfDataObj->rawS4LPF[0] + 0.03*grfDataObj->rawS4[0];
//	grfDataObj->rawS4LPF[1] = 0.97*grfDataObj->rawS4LPF[0] + 0.03*grfDataObj->rawS4[0];
//	grfDataObj->rawS4LPF[2] = 0.97*grfDataObj->rawS4LPF[0] + 0.03*grfDataObj->rawS4[0];
//
//	/* Calculate (0 ~ 3.3V) and compensate offset of S2,S3,S4 */
//	// S2 - x,y,z //
//	grfDataObj->voltS2[0] = (float)(((int32_t)grfDataObj->rawS2LPF[0] - (int32_t)grfDataObj->rawS2offset[0]) * 3.3 / 16383);
//	grfDataObj->voltS2[1] = (float)(((int32_t)grfDataObj->rawS2LPF[1] - (int32_t)grfDataObj->rawS2offset[1]) * 3.3 / 16383);
//	grfDataObj->voltS2[2] = (float)(((int32_t)grfDataObj->rawS2LPF[2] - (int32_t)grfDataObj->rawS2offset[2]) * 3.3 / 16383);
//
//	// S3 - x,y,z //
//	grfDataObj->voltS3[0] = (float)(((int32_t)grfDataObj->rawS3LPF[0] - (int32_t)grfDataObj->rawS3offset[0]) * 3.3 / 16383);
//	grfDataObj->voltS3[1] = (float)(((int32_t)grfDataObj->rawS3LPF[1] - (int32_t)grfDataObj->rawS3offset[1]) * 3.3 / 16383);
//	grfDataObj->voltS3[2] = (float)(((int32_t)grfDataObj->rawS3LPF[2] - (int32_t)grfDataObj->rawS3offset[2]) * 3.3 / 16383);
//
//	// S4 - x,y,z //
//	grfDataObj->voltS4[0] = (float)(((int32_t)grfDataObj->rawS4LPF[0] - (int32_t)grfDataObj->rawS4offset[0]) * 3.3 / 16383);
//	grfDataObj->voltS4[1] = (float)(((int32_t)grfDataObj->rawS4LPF[1] - (int32_t)grfDataObj->rawS4offset[1]) * 3.3 / 16383);
//	grfDataObj->voltS4[2] = (float)(((int32_t)grfDataObj->rawS4LPF[2] - (int32_t)grfDataObj->rawS4offset[2]) * 3.3 / 16383);




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



	/* Calculate the Force (Choose proper CalibMatrix) */
	if (grfBoardNodeID == 14)
	{
		for (uint8_t i = 0; i < 3 ; i++){
			grfDataObj->F_S2[i] = 0;
			grfDataObj->F_S3[i] = 0;
			grfDataObj->F_S4[i] = 0;
			for (uint8_t j = 0; j < 3; j++){
				grfDataObj->F_S2[i] += Calib_1005[i][j] * strainS2[j];
				grfDataObj->F_S3[i] += Calib_1004[i][j] * strainS3[j];
				grfDataObj->F_S4[i] += Calib_1006[i][j] * strainS4[j];
			}
		}
	}
	else if (grfBoardNodeID == 15)
	{
		for (uint8_t i = 0; i < 3 ; i++){
			grfDataObj->F_S2[i] = 0;
			grfDataObj->F_S3[i] = 0;
			grfDataObj->F_S4[i] = 0;
			for (uint8_t j = 0; j < 3; j++){
				grfDataObj->F_S2[i] += Calib_0905[i][j] * strainS2[j];
				grfDataObj->F_S3[i] += Calib_0904[i][j] * strainS3[j];
				grfDataObj->F_S4[i] += Calib_0903[i][j] * strainS4[j];
			}
		}
	}
	else
	{
		for (uint8_t i = 0; i < 3 ; i++){
			grfDataObj->F_S2[i] = 0;
			grfDataObj->F_S3[i] = 0;
			grfDataObj->F_S4[i] = 0;
		}
	}

	return 0;
}


#ifdef GRF_RIGHT
static int Force_Labeling(GRF_Data_t* grfDataObj)
{
	grfDataObj->GRF_X  = 0;
	grfDataObj->GRF_Y  = 0;
	grfDataObj->GRF_Z  = 0;

	grfDataObj->F_FM[0] = RotMat_S2[0][0]*grfDataObj->F_S2[0] + RotMat_S2[0][1]*grfDataObj->F_S2[1] + RotMat_S2[0][2]*grfDataObj->F_S2[2];
	grfDataObj->F_FL[0] = RotMat_S4[0][0]*grfDataObj->F_S4[0] + RotMat_S4[0][1]*grfDataObj->F_S4[1] + RotMat_S4[0][2]*grfDataObj->F_S4[2];
	grfDataObj->F_Ba[0] = RotMat_S3[0][0]*grfDataObj->F_S3[0] + RotMat_S3[0][1]*grfDataObj->F_S3[1] + RotMat_S3[0][2]*grfDataObj->F_S3[2];

	grfDataObj->F_FM[1] = RotMat_S2[1][0]*grfDataObj->F_S2[0] + RotMat_S2[1][1]*grfDataObj->F_S2[1] + RotMat_S2[1][2]*grfDataObj->F_S2[2];
	grfDataObj->F_FL[1] = RotMat_S4[1][0]*grfDataObj->F_S4[0] + RotMat_S4[1][1]*grfDataObj->F_S4[1] + RotMat_S4[1][2]*grfDataObj->F_S4[2];
	grfDataObj->F_Ba[1] = RotMat_S3[1][0]*grfDataObj->F_S3[0] + RotMat_S3[1][1]*grfDataObj->F_S3[1] + RotMat_S3[1][2]*grfDataObj->F_S3[2];

	grfDataObj->F_FM[2] = RotMat_S2[2][0]*grfDataObj->F_S2[0] + RotMat_S2[2][1]*grfDataObj->F_S2[1] + RotMat_S2[2][2]*grfDataObj->F_S2[2];
	grfDataObj->F_FL[2] = RotMat_S4[2][0]*grfDataObj->F_S4[0] + RotMat_S4[2][1]*grfDataObj->F_S4[1] + RotMat_S4[2][2]*grfDataObj->F_S4[2];
	grfDataObj->F_Ba[2] = RotMat_S3[2][0]*grfDataObj->F_S3[0] + RotMat_S3[2][1]*grfDataObj->F_S3[1] + RotMat_S3[2][2]*grfDataObj->F_S3[2];

	return 0;
}
#endif

#ifdef GRF_LEFT
static int Force_Labeling(GRF_Data_t* grfDataObj)
{
	grfDataObj->GRF_X  = 0;
	grfDataObj->GRF_Y  = 0;
	grfDataObj->GRF_Z  = 0;

	grfDataObj->F_FM[0] = RotMat_S2[0][0]*grfDataObj->F_S2[0] + RotMat_S2[0][1]*grfDataObj->F_S2[1] + RotMat_S2[0][2]*grfDataObj->F_S2[2];
	grfDataObj->F_FL[0] = RotMat_S3[0][0]*grfDataObj->F_S3[0] + RotMat_S3[0][1]*grfDataObj->F_S3[1] + RotMat_S3[0][2]*grfDataObj->F_S3[2];
	grfDataObj->F_Ba[0] = RotMat_S4[0][0]*grfDataObj->F_S4[0] + RotMat_S4[0][1]*grfDataObj->F_S4[1] + RotMat_S4[0][2]*grfDataObj->F_S4[2];

	grfDataObj->F_FM[1] = RotMat_S2[1][0]*grfDataObj->F_S2[0] + RotMat_S2[1][1]*grfDataObj->F_S2[1] + RotMat_S2[1][2]*grfDataObj->F_S2[2];
	grfDataObj->F_FL[1] = RotMat_S3[1][0]*grfDataObj->F_S3[0] + RotMat_S3[1][1]*grfDataObj->F_S3[1] + RotMat_S3[1][2]*grfDataObj->F_S3[2];
	grfDataObj->F_Ba[1] = RotMat_S4[1][0]*grfDataObj->F_S4[0] + RotMat_S4[1][1]*grfDataObj->F_S4[1] + RotMat_S4[1][2]*grfDataObj->F_S4[2];

	grfDataObj->F_FM[2] = RotMat_S2[2][0]*grfDataObj->F_S2[0] + RotMat_S2[2][1]*grfDataObj->F_S2[1] + RotMat_S2[2][2]*grfDataObj->F_S2[2];
	grfDataObj->F_FL[2] = RotMat_S3[2][0]*grfDataObj->F_S3[0] + RotMat_S3[2][1]*grfDataObj->F_S3[1] + RotMat_S3[2][2]*grfDataObj->F_S3[2];
	grfDataObj->F_Ba[2] = RotMat_S4[2][0]*grfDataObj->F_S4[0] + RotMat_S4[2][1]*grfDataObj->F_S4[1] + RotMat_S4[2][2]*grfDataObj->F_S4[2];

	return 0;
}
#endif


static int GetFiltGRF(GRF_Data_t* grfDataObj)
{

	for (uint8_t i = 0; i < 3; i++)
	{
		// 0.1Hz - 10Hz Bandpass Filter
/*		grfDataObj->F_FM_filt[i] = 1.938*grfDataObj->F_FM_filt[i] - 0.9385*grfDataObj->F_FM_filt_f[i] + 0.06088*grfDataObj->F_FM_f[i] - 0.06088*grfDataObj->F_FM_ff[i];
		grfDataObj->F_S3_filt[i] = 1.938*grfDataObj->F_S3_filt[i] - 0.9385*grfDataObj->F_S3_filt_f[i] + 0.06088*grfDataObj->F_S3_f[i] - 0.06088*grfDataObj->F_S3_ff[i];
		grfDataObj->F_S4_filt[i] = 1.938*grfDataObj->F_S4_filt[i] - 0.9385*grfDataObj->F_S4_filt_f[i] + 0.06088*grfDataObj->F_S4_f[i] - 0.06088*grfDataObj->F_S4_ff[i];

		grfDataObj->F_FM_filt_f[i] = grfDataObj->F_FM_filt[i];
		grfDataObj->F_FM_ff[i] = grfDataObj->F_FM_f[i];
		grfDataObj->F_FM_f[i] = grfDataObj->F_FM[i];

		grfDataObj->F_S3_filt_f[i] = grfDataObj->F_S3_filt[i];
		grfDataObj->F_S3_ff[i] = grfDataObj->F_S3_f[i];
		grfDataObj->F_S3_f[i] = grfDataObj->F_S3[i];

		grfDataObj->F_S4_filt_f[i] = grfDataObj->F_S4_filt[i];
		grfDataObj->F_S4_ff[i] = grfDataObj->F_S4_f[i];
		grfDataObj->F_S4_f[i] = grfDataObj->F_S4[i];*/

		/* Rate Limiter */
		if (abs(grfDataObj->F_FM_prev[i] - grfDataObj->F_FM[i]) > thresh_RateLimiter){
			grfDataObj->F_FM_f[i] = grfDataObj->F_FM_prev[i] - sign(grfDataObj->F_FM_prev[i] - grfDataObj->F_FM[i]) * thresh_RateLimiter;
		}
		else{
			grfDataObj->F_FM_f[i] = grfDataObj->F_FM[i];
		}
		if (abs(grfDataObj->F_FL_f[i] - grfDataObj->F_S3[i]) > thresh_RateLimiter){
			grfDataObj->F_FL_f[i] = grfDataObj->F_FL_prev[i] - sign(grfDataObj->F_FL_prev[i] - grfDataObj->F_FL[i]) * thresh_RateLimiter;
		}
		else{
			grfDataObj->F_FL_f[i] = grfDataObj->F_FL[i];
		}
		if (abs(grfDataObj->F_Ba_f[i] - grfDataObj->F_Ba[i]) > thresh_RateLimiter){
			grfDataObj->F_Ba_f[i] = grfDataObj->F_Ba_prev[i] - sign(grfDataObj->F_Ba_prev[i] - grfDataObj->F_Ba[i]) * thresh_RateLimiter;
		}
		else{
			grfDataObj->F_Ba_f[i] = grfDataObj->F_Ba[i];
		}
		grfDataObj->F_FM_prev[i] = grfDataObj->F_FM_f[i];
		grfDataObj->F_FL_prev[i] = grfDataObj->F_FL_f[i];
		grfDataObj->F_Ba_prev[i] = grfDataObj->F_Ba_f[i];

		/* Low Pass Filter */
		float fc = 4; // Cut-off frequency
		float a_LPF = 1 / (1 + 1/(2*M_PI*fc*0.001));
		grfDataObj->F_FM_LPF[i] = (1 - a_LPF)*grfDataObj->F_FM_LPF[i] + a_LPF*grfDataObj->F_FM_f[i];
		grfDataObj->F_FL_LPF[i] = (1 - a_LPF)*grfDataObj->F_FL_LPF[i] + a_LPF*grfDataObj->F_FL_f[i];
		grfDataObj->F_Ba_LPF[i] = (1 - a_LPF)*grfDataObj->F_Ba_LPF[i] + a_LPF*grfDataObj->F_Ba_f[i];
//		grfDataObj->F_FM_LPF[i] = 0.982*grfDataObj->F_FM_LPF[i] + 0.018*grfDataObj->F_FM_f[i];
//		grfDataObj->F_FL_LPF[i] = 0.982*grfDataObj->F_FL_LPF[i] + 0.018*grfDataObj->F_FL_f[i];
//		grfDataObj->F_Ba_LPF[i] = 0.982*grfDataObj->F_Ba_LPF[i] + 0.018*grfDataObj->F_Ba_f[i];


	}

	grfDataObj->F_FM_LPF_X = grfDataObj->F_FM_LPF[0]; grfDataObj->F_FM_LPF_Y = grfDataObj->F_FM_LPF[1]; grfDataObj->F_FM_LPF_Z = grfDataObj->F_FM_LPF[2];
	grfDataObj->F_FL_LPF_X = grfDataObj->F_FL_LPF[0]; grfDataObj->F_FL_LPF_Y = grfDataObj->F_FL_LPF[1]; grfDataObj->F_FL_LPF_Z = grfDataObj->F_FL_LPF[2];
	grfDataObj->F_Ba_LPF_X = grfDataObj->F_Ba_LPF[0]; grfDataObj->F_Ba_LPF_Y = grfDataObj->F_Ba_LPF[1]; grfDataObj->F_Ba_LPF_Z = grfDataObj->F_Ba_LPF[2];

	/* Force Threshold */
	grfDataObj->F_FM_norm = sqrt(grfDataObj->F_FM_LPF[0]*grfDataObj->F_FM_LPF[0] + grfDataObj->F_FM_LPF[1]*grfDataObj->F_FM_LPF[1] + grfDataObj->F_FM_LPF[2]*grfDataObj->F_FM_LPF[2]);
	grfDataObj->F_FL_norm = sqrt(grfDataObj->F_FL_LPF[0]*grfDataObj->F_FL_LPF[0] + grfDataObj->F_FL_LPF[1]*grfDataObj->F_FL_LPF[1] + grfDataObj->F_FL_LPF[2]*grfDataObj->F_FL_LPF[2]);
	grfDataObj->F_Ba_norm = sqrt(grfDataObj->F_Ba_LPF[0]*grfDataObj->F_Ba_LPF[0] + grfDataObj->F_Ba_LPF[1]*grfDataObj->F_Ba_LPF[1] + grfDataObj->F_Ba_LPF[2]*grfDataObj->F_Ba_LPF[2]);

	if (grfDataObj->F_FM_norm < thresh_sensor){
		for (uint8_t i = 0; i < 3; i++){
			grfDataObj->F_FM_filt[i] = 0;
		}
	}
	else{
		for (uint8_t i = 0; i <3; i++){
			grfDataObj->F_FM_filt[i] = grfDataObj->F_FM_LPF[i];
		}
	}
	if (grfDataObj->F_FL_norm < thresh_sensor){
		for (uint8_t i = 0; i < 3; i++){
			grfDataObj->F_FL_filt[i] = 0;
		}
	}
	else{
		for (uint8_t i = 0; i <3; i++){
			grfDataObj->F_FL_filt[i] = grfDataObj->F_FL_LPF[i];
		}
	}
	if (grfDataObj->F_Ba_norm < thresh_sensor){
		for (uint8_t i = 0; i < 3; i++){
			grfDataObj->F_Ba_filt[i] = 0;
		}
	}
	else{
		for (uint8_t i = 0; i <3; i++){
			grfDataObj->F_Ba_filt[i] = grfDataObj->F_Ba_LPF[i];
		}
	}

	return 0;
}

static int Get_Total_GRF(GRF_Data_t* grfDataObj)
{
	grfDataObj->Force_FM_X = grfDataObj->F_FM_filt[0]; grfDataObj->Force_FM_Y = grfDataObj->F_FM_filt[1]; grfDataObj->Force_FM_Z = grfDataObj->F_FM_filt[2];
	grfDataObj->Force_FL_X = grfDataObj->F_FL_filt[0]; grfDataObj->Force_FL_Y = grfDataObj->F_FL_filt[1]; grfDataObj->Force_FL_Z = grfDataObj->F_FL_filt[2];
	grfDataObj->Force_Ba_X = grfDataObj->F_Ba_filt[0]; grfDataObj->Force_Ba_Y = grfDataObj->F_Ba_filt[1]; grfDataObj->Force_Ba_Z = grfDataObj->F_Ba_filt[2];

	grfDataObj->GRF_X = grfDataObj->Force_FM_X + grfDataObj->Force_FL_X + grfDataObj->Force_Ba_X;
	grfDataObj->GRF_Y = grfDataObj->Force_FM_Y + grfDataObj->Force_FL_Y + grfDataObj->Force_Ba_Y;
	grfDataObj->GRF_Z = grfDataObj->Force_FM_Z + grfDataObj->Force_FL_Z + grfDataObj->Force_Ba_Z;

	return 0;
}

#ifdef GRF_RIGHT
static int Get_Total_COP(GRF_Data_t* grfDataObj)
{
	if ( (abs(grfDataObj->GRF_Z) > thresh_vert) && (sqrt((grfDataObj->GRF_X)*(grfDataObj->GRF_X) + (grfDataObj->GRF_Y)*(grfDataObj->GRF_Y) + (grfDataObj->GRF_Z)*(grfDataObj->GRF_Z)) > thresh_norm) )
//	if ( (abs(grfDataObj->GRF_Z) > thresh_vert) )
	{
		float c_x = rx_F * (grfDataObj->Force_FL_Z + grfDataObj->Force_FM_Z) - rx_B * (grfDataObj->Force_Ba_Z) - rz * (grfDataObj->Force_FL_X + grfDataObj->Force_FM_X + grfDataObj->Force_Ba_X);
		float c_y = ry_F * (-grfDataObj->Force_FL_Z + grfDataObj->Force_FM_Z) + ry_B * (grfDataObj->Force_Ba_Z) - rz * (grfDataObj->Force_FL_Y + grfDataObj->Force_FM_Y + grfDataObj->Force_Ba_Y);

		grfDataObj->COP_X = c_x/grfDataObj->GRF_Z;
		grfDataObj->COP_Y = c_y/grfDataObj->GRF_Z;
		grfDataObj->COP_Z = 0;
	}
	else
	{
		grfDataObj->COP_X = 0;
		grfDataObj->COP_Y = 0;
		grfDataObj->COP_Z = 0;
	}
	return 0;

}
#endif

#ifdef GRF_LEFT
static int Get_Total_COP(GRF_Data_t* grfDataObj)
{
	if ( (abs(grfDataObj->GRF_Z) > thresh_vert) && (sqrt((grfDataObj->GRF_X)*(grfDataObj->GRF_X) + (grfDataObj->GRF_Y)*(grfDataObj->GRF_Y) + (grfDataObj->GRF_Z)*(grfDataObj->GRF_Z)) > thresh_norm) )
	{
		float c_x = rx_F * (grfDataObj->Force_FL_Z + grfDataObj->Force_FM_Z) - rx_B * (grfDataObj->Force_Ba_Z) - rz * (grfDataObj->Force_FL_X + grfDataObj->Force_FM_X + grfDataObj->Force_Ba_X);
		float c_y = ry_F * (-grfDataObj->Force_FL_Z + grfDataObj->Force_FM_Z) - ry_B * (grfDataObj->Force_Ba_Z) - rz * (grfDataObj->Force_FL_Y + grfDataObj->Force_FM_Y + grfDataObj->Force_Ba_Y);

		grfDataObj->COP_X = c_x/grfDataObj->GRF_Z;
		grfDataObj->COP_Y = c_y/grfDataObj->GRF_Z;
		grfDataObj->COP_Z = 0;
	}
	else
	{
		grfDataObj->COP_X = 0;
		grfDataObj->COP_Y = 0;
		grfDataObj->COP_Z = 0;
	}
	return 0;

}
#endif


static int sign(double x) {
    if (x > 0.0) {
        return 1;
    } else if (x < 0.0) {
        return -1;
    } else {
        return 0;  // If x is zero
    }
}

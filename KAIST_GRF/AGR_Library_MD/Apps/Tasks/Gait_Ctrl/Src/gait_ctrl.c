#include "gait_ctrl.h"

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

TaskObj_t gaitCtrlTask;

// For Quaternion //
VQF_MagCalib_t vqfMagCalibObj;
VQF_t vqfObj;

float magValueCal[3];
VQF_Real_t vqfGyr[3];
VQF_Real_t vqfAcc[3];
VQF_Real_t vqfMag[3];
VQF_Real_t vqfQuat[4];
int16_t q_send[4];

float A_inv[3][3] = {  	{ 0.905e-03,  -0.0069e-03, -0.00083e-03},
						{-0.0069e-03,   0.971e-03,  -0.018e-03},
						{-0.00083e-03,   -0.0181e-03,  0.9629e-03}   };

float ironErr[3] = {0.635e+03, -0.703e+03, -1.42e+03};

float RealMagX;
float RealMagY;
float RealMagZ;

// For Triggering //
WIDM_GaitData_t		widmGaitDataObj;
WIDM_AngleData_t 	widmAngleDataObj;
WIDM_AttachCase_t	widmAttachCaseObj;

static IOIF_6AxisData_t 	imu6AxisDataObj;
static IOIF_MagData_t 		magDataObj;

static WIDM_SensorData_t 	widmSensorDataObj;
static WIDM_FuzzyData_t		widmFuzzyDataObj;
static WIDM_NormData_t 		widmNormDataObj;
static WIDM_ThresData_t		widmThresDataObj;
static WIDM_Module_t		widmModuleObj;

// Loop Time Count //
static uint32_t gaitCtrlLoopCnt;
static float gaitCtrlTimeElap;


static uint8_t testImu6AxisRes 	= IOIF_I2C_STATUS_OK;
static uint8_t testImu3AxisRes 	= IOIF_I2C_STATUS_OK;

static float wcDebug 			= 0.0;

//static uint8_t absOffsetCmd = 0;
uint8_t gaitCtrlState = 0;

// SELECTION !!!  For ALL Model //
static WIDM_AttachCase_t 	ATTACH_CASE_SEL;
static WIDM_Module_t	    MODULE_SEL;
static uint8_t 				moduleNodeID;


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
static int NodeIDCheck(uint8_t directionSet);
static void ModelSelection(uint8_t nodeID);
static void InitializeIMU(void);
static void ResetDataObj(void);
static void InitValueSetting(WIDM_FuzzyData_t* widmFuzzyData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData, WIDM_ThresData_t* widmThresData, WIDM_Module_t widmModule);
static void GetInitialAngle(WIDM_Module_t widmModule);
static void SetInitialAngle(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, float initialAngle);
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase);

/* ------------------- ROUTINE ------------------- */
static int RunGetIMUFunction(void);

// Quaternion //
static int EntGetQuaternion(void);
static int RunGetQuaternion(void);

/* ------------------- STATIC FUNCTIONS ------------------- */
/* Functions for Gait Parameters */
static void UpdateSensorRawData(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase);

// Functions for Quaternion //
static void InitMagInfo(void);

/* ------------------- SDO CALLBACK ------------------- */
//static void SetAbsOffsetCmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void SetMagInvAInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(gaitCtrlTask)

void InitGaitCtrl(void)
{
    InitTask(&gaitCtrlTask);

    /* Checking Node ID */
    moduleNodeID = NodeIDCheck(0);

	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_GAIT_TOTAL_FUNCTION, 		NULL, 	RunGetIMUFunction,	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_GAIT);

	// PDO
	/* For SUIT PDO setting */
	DOP_CreatePDO(TASK_ID_GAIT, 	 PDO_ID_GAIT_DEG,		DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
	DOP_CreatePDO(TASK_ID_GAIT, 	 PDO_ID_GAIT_VEL,		DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
	DOP_CreatePDO(TASK_ID_GAIT, 	 PDO_ID_GAIT_GYR_Z,		DOP_FLOAT32,	1,    &widmSensorDataObj.gyrZ[0]);

	// Quaternion //
	// DOP_CreatePDO(TASK_ID_GAIT, PDO_ID_QUATERNION,          DOP_INT16,   4, &q_send);
	// DOP_CreateSDO(TASK_ID_GAIT, SDO_ID_IMU_MAG_INVA,        DOP_FLOAT32, SetMagInvAInfo);
	// DOP_CreateSDO(TASK_ID_GAIT, SDO_ID_IMU_MAG_IRON_ERROR,  DOP_FLOAT32, SetMagIronErrorInfo);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_GAIT)
	// DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_WIDM_SET_ABSOFFSET_CMD, 	DOP_UINT8, 	SetAbsOffsetCmd);

	// !!! Select correct Module here //
	ModelSelection(moduleNodeID);
	widmAttachCaseObj = ATTACH_CASE_SEL;
	widmModuleObj = MODULE_SEL;

	/* Init 6axis & 3axis IMU */
	InitializeIMU();

	// Quaternion //
	// InitMagInfo();

	/* Initial stage of get angle */
	ResetDataObj();
	InitValueSetting(&widmFuzzyDataObj, &widmNormDataObj, &widmGaitDataObj, &widmThresDataObj, widmModuleObj);
	GetInitialAngle(widmModuleObj);

#ifdef QUATERNION
	// Quaternion //
	InitMagInfo();
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

	/* Timer Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM2) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM2, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunGaitCtrl, NULL);
}

void RunGaitCtrl(void* params)
{
	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	RunTask(&gaitCtrlTask);

	/* Elapsed Time Check */
	gaitCtrlTimeElap = DWT->CYCCNT / 160;	// in microsecond
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
	RunGetIMUFunction();
	gaitCtrlLoopCnt = 0;
}

static void StateEnable_Ent(void)
{
	EntRoutines(&gaitCtrlTask.routine);
//	StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Run(void)
{
	RunRoutines(&gaitCtrlTask.routine);
//	RunGetIMUFunction();
	gaitCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    ExtRoutines(&gaitCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* ------------------- ROUTINE ------------------- */


/* Just get 6Axis & 3Axis IMU values */
static int RunGetIMUFunction(void)
{
	testImu6AxisRes = IOIF_Get6AxisValue(&imu6AxisDataObj);
	testImu3AxisRes = IOIF_GetMagValue(&magDataObj);

	return 0;
}


/* Functions for ALL cases(General) */

/* Module & Mode Selection Functions */
static int NodeIDCheck(uint8_t directionSet)
{
	#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)
		int nodeID = 1;
		return nodeID;
	#endif /* WALKON5_CM_ENABLED & L30_CM_ENABLED & SUIT_MINICM_ENABLED */

	#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
		int temp1, temp2, temp3, temp4;
		temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
		temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
		temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
		temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
		return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
	#endif /* WALKON5_MD_ENABLED & L30_MD_REV06_ENABLED & L30_MD_REV07_ENABLED & L30_MD_REV08_ENABLED */

	#if defined(SUIT_MD_ENABLED)
		int temp1, temp2, temp3, temp4;
		temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
		temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
		temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
		temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
		return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
	#endif /* SUIT_MD_ENABLED */

	#if defined(KAIST_GRF_BOARD_ENABLED)
		int nodeID = 0;
		if (directionSet == 0){			// RIGHT GRF
			nodeID = 14;
		}
		else if (directionSet == 1){	// LEFT GRF
			nodeID = 15;
		}
		return nodeID;
	#endif /* KAIST_GRF_BOARD_ENABLED */
}

/* Select the Joint and Sensor method */
static void ModelSelection(uint8_t nodeID)
{
	switch (nodeID){
//		case (NODE_ID_LH_SAG):	// LH
//			ATTACH_CASE_SEL   = WIDM_H10_LEFT;
//			MODULE_SEL 		  = WIDM_IMUINC_SAM;
//			break;
//		case (NODE_ID_RH_SAG):	// RH
//			ATTACH_CASE_SEL   = WIDM_H10_RIGHT;
//			MODULE_SEL 		  = WIDM_IMUINC_SAM;
//			break;
//		case (NODE_ID_LK):		// LK
//			ATTACH_CASE_SEL   = WIDM_K10_LEFT;
//			MODULE_SEL 		  = WIDM_IMU_SAM;
//			break;
//		case (NODE_ID_RK):		// RK
//			ATTACH_CASE_SEL   = WIDM_K10_RIGHT;
//			MODULE_SEL 		  = WIDM_IMU_SAM;
//			break;
		case (NODE_ID_WIDM_R):
			ATTACH_CASE_SEL   = WIDM_RIGHT_U5;
			MODULE_SEL 		  = WIDM_IMU_U5;
			break;
		case (NODE_ID_WIDM_L):
			ATTACH_CASE_SEL   = WIDM_LEFT_U5;
			MODULE_SEL 		  = WIDM_IMU_U5;
			break;
		default:
			break;
	}
}

/* Initialize 6Axis & 3Axis IMU */
static void InitializeIMU(void)
{
	testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
	testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
}

/* Setting for Initial values of "angle" variables after get initial thigh angle */
static void SetInitialAngle(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, float initialAngle)
{
	widmAngleData->degAccFiltered 	= initialAngle;
	widmAngleData->degGyrFiltered	= 0.0;

	widmAngleData->degLPF1st[0] 	= initialAngle;
	widmAngleData->degLPF1st[1] 	= initialAngle;
	widmAngleData->degLPF2nd[0] 	= initialAngle;
	widmAngleData->degLPF2nd[1] 	= initialAngle;
	widmNormData->degOri 	 		= initialAngle;
}

/*
 *Function to calculate the initial thigh angle - IMU case
*/
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase)
{
	uint8_t tTotalSamples 	= 100;
	uint8_t tDataCheck_IMU	= 0;
	uint8_t tRealSamples	= 0;
	float tAccumulatedAngle = 0.0;
	float tInitThighAngle	= 0.0;
	float tImuAngle			= 0.0;

	for (uint8_t i = 1; i <= tTotalSamples; i++) {
        for (uint16_t j = 0; j < 30000; j++) {
        	// For delay of DMA reading
        }

        tDataCheck_IMU = IOIF_Get6AxisValue(imu6AxisData);

        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

            tImuAngle = WIDM_AttachCaseSetting(widmSensorData, widmAttachCase);
            tAccumulatedAngle += tImuAngle;

            tRealSamples++;
        }
    }

	tInitThighAngle = tAccumulatedAngle / tRealSamples;
	widmAngleData->initAngle = tInitThighAngle;
    SetInitialAngle(widmAngleData, widmNormData, tInitThighAngle);
}

static void GetInitialAngle(WIDM_Module_t widmModule)
{
	/* [IMU only] case */
	if (widmModule == WIDM_IMU_SAM || widmModule == WIDM_IMU_U5) {
		GetInitialAngle_IMU(&imu6AxisDataObj, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj);
	}
}

/* Reset Value Zero */
static void ResetDataObj(void)
{
	widmSensorDataObj 		= 	(WIDM_SensorData_t){0};
	widmFuzzyDataObj 		= 	(WIDM_FuzzyData_t){0};
	widmAngleDataObj		=   (WIDM_AngleData_t){0};
	widmNormDataObj 		= 	(WIDM_NormData_t){0};
	widmGaitDataObj 		= 	(WIDM_GaitData_t){0};
	widmThresDataObj 		= 	(WIDM_ThresData_t){0};

	wcDebug					= 	0.0;
}

/* Setting for initial parameters */
static void InitValueSetting(WIDM_FuzzyData_t* widmFuzzyData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData, WIDM_ThresData_t* widmThresData, WIDM_Module_t widmModule)
{
	widmFuzzyData->wl 				= 0.5;
	widmFuzzyData->wh 				= 10.0;
	widmFuzzyData->var[0] 			= 8.0;
	widmFuzzyData->var[1] 			= 30.0;
	widmFuzzyData->var[2] 			= 5.8;
	widmFuzzyData->var[3] 			= 320.0;

	widmNormData->ampDeg 			= 30.0; 	//30
	widmNormData->ampDegLPF[0]		= 30.0;
	widmNormData->ampDegLPF[1]		= 30.0;

	widmNormData->ampVel 			= 400.0; 	//400
	widmNormData->ampVelLPF[0]		= 400.0;
	widmNormData->ampVelLPF[1]		= 400.0;


	widmGaitData->gaitPeriod 		= 1000;
	widmGaitData->gaitPhase 		= -100.0;
	widmGaitData->gaitPhasePre   	= -100.0;

	if (widmModule == WIDM_IMU_U5){
		widmThresData->degThStart	= 5.0;
		widmThresData->velThStart	= 20.0;
		widmThresData->degThStop 	= 5.0;
		widmThresData->velThStop 	= 3.0;
	}
	else if (widmModule == WIDM_IMU_SAM) {
		widmThresData->degThStart	= 12.0;
		widmThresData->velThStart	= 20.0;
		widmThresData->degThStop 	= 3.0;
		widmThresData->velThStop 	= 12.0;
	}
	else if (widmModule == WIDM_IMUABS_SAM) {
		widmThresData->degThStart	= 10.0;		//15.0
		widmThresData->velThStart	= 40.0;		//60.0
		widmThresData->degThStop 	= 8.0;		//5.0
		widmThresData->velThStop 	= 10.0;		//25.0
	}
	else if (widmModule == WIDM_IMUINC_SAM) {
		widmThresData->degThStart	= 12.0;		//15.0
		widmThresData->velThStart	= 65.0;		//60.0
		widmThresData->degThStop 	= 8.0;		//5.0
		widmThresData->velThStop 	= 40.0;		//25.0
	}
}

/*
*The function UpdateSensorRawData updates the IMU raw values.
*/
static void UpdateSensorRawData(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase)
{
	widmSensorData->accX[0] = imu6AxisData->accX;
	widmSensorData->accY[0] = imu6AxisData->accY;
	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

	if (widmAttachCase < 8){
		widmSensorData->gyrZ[0] = (-1) * (imu6AxisData->gyrZ); 	// For Negative Gyro case (Maybe LEFT case)
	}
	else if (widmAttachCase >= 8){
		widmSensorData->gyrZ[0] = imu6AxisData->gyrZ; 			// For Positive Gyro case (Maybe RIGHT case)
	}
}


// Quaternion //
static void SetMagInvAInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&vqfMagCalibObj.a11, t_req->data, 9*4);

	vqfMagCalibObj.A_inv[0][0] = vqfMagCalibObj.a11;
	vqfMagCalibObj.A_inv[0][1] = vqfMagCalibObj.a12;
	vqfMagCalibObj.A_inv[0][2] = vqfMagCalibObj.a13;
	vqfMagCalibObj.A_inv[1][0] = vqfMagCalibObj.a21;
	vqfMagCalibObj.A_inv[1][1] = vqfMagCalibObj.a22;
	vqfMagCalibObj.A_inv[1][2] = vqfMagCalibObj.a23;
	vqfMagCalibObj.A_inv[2][0] = vqfMagCalibObj.a31;
	vqfMagCalibObj.A_inv[2][1] = vqfMagCalibObj.a32;
	vqfMagCalibObj.A_inv[2][2] = vqfMagCalibObj.a33;

   t_res->dataSize = 0;
   t_res->status = DOP_SDO_SUCC;
}

static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&vqfMagCalibObj.b1, t_req->data, 3*4);

	vqfMagCalibObj.ironErr[0] = vqfMagCalibObj.b1;
	vqfMagCalibObj.ironErr[1] = vqfMagCalibObj.b2;
	vqfMagCalibObj.ironErr[2] = vqfMagCalibObj.b3;

   t_res->dataSize = 0;
   t_res->status = DOP_SDO_SUCC;
}

static void InitMagInfo(void)
{
	vqfMagCalibObj.A_inv[0][0] = vqfMagCalibObj.a11;
	vqfMagCalibObj.A_inv[0][1] = vqfMagCalibObj.a12;
	vqfMagCalibObj.A_inv[0][2] = vqfMagCalibObj.a13;
	vqfMagCalibObj.A_inv[1][0] = vqfMagCalibObj.a21;
	vqfMagCalibObj.A_inv[1][1] = vqfMagCalibObj.a22;
	vqfMagCalibObj.A_inv[1][2] = vqfMagCalibObj.a23;
	vqfMagCalibObj.A_inv[2][0] = vqfMagCalibObj.a31;
	vqfMagCalibObj.A_inv[2][1] = vqfMagCalibObj.a32;
	vqfMagCalibObj.A_inv[2][2] = vqfMagCalibObj.a33;

	vqfMagCalibObj.ironErr[0] = vqfMagCalibObj.b1;
	vqfMagCalibObj.ironErr[1] = vqfMagCalibObj.b2;
	vqfMagCalibObj.ironErr[2] = vqfMagCalibObj.b3;
}

static int EntGetQuaternion(void)
{
	VQF_Params_t params = { 0 };
	vqfObj.params = params;

	VQF_Init(&vqfObj, 0.001, 0.001, 0.001);
	return 0;
}

static int RunGetQuaternion(void)
{
	//uint8_t t6AxisRes
	//uint8_t t3AxisRes

	IOIF_Get6AxisValue(&imu6AxisDataObj);
	IOIF_GetMagValue(&magDataObj);

	//if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { return t6AxisRes; }
	//if (t3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { return t3AxisRes; }

	float accX = imu6AxisDataObj.accX;
	float accY = imu6AxisDataObj.accY;
	float accZ = imu6AxisDataObj.accZ;
	float gyrX = imu6AxisDataObj.gyrX * 0.017453f; //degree to radian
	float gyrY = imu6AxisDataObj.gyrY * 0.017453f; //degree to radian
	float gyrZ = imu6AxisDataObj.gyrZ * 0.017453f; //degree to radian

	float magX_r = -magDataObj.magY;
	float magY_r = magDataObj.magX;
	float magZ_r = magDataObj.magZ;

	RealMagX = magX_r;
	RealMagY = magY_r;
	RealMagZ = magZ_r;

	float magValueRaw[3] = {magX_r, magY_r, magZ_r};

	for (int i = 0; i < 3; i++) {
		magValueCal[i] = 0;
		for (int j = 0; j < 3; j++) {
			magValueCal[i] += A_inv[i][j] * (magValueRaw[j] - ironErr[j]);
//			magValueCal[i] += vqfMagCalibInfo.A_inv[i][j] * (magValueRaw[j] - mag_calib_info.iron_err[j]);
		}
	}
	float magnorm = sqrt(magValueCal[0]*magValueCal[0] + magValueCal[1]*magValueCal[1] + magValueCal[2]*magValueCal[2]);
	magValueCal[0] = magValueCal[0] / magnorm;
	magValueCal[1] = magValueCal[1] / magnorm;
	magValueCal[2] = magValueCal[2] / magnorm;

	float magX = magValueCal[0];
	float magY = magValueCal[1];
	float magZ = magValueCal[2];

    VQF_Real_t gyr[3] = {gyrX, gyrY, gyrZ};
    VQF_Real_t acc[3] = {accX, accY, accZ};
    VQF_Real_t mag[3] = {magX, magY, magZ};

	VQF_UpdateWithMag(&vqfObj, gyr, acc, mag);
	VQF_GetQuat9D(&vqfObj, vqfQuat);
	int i;
	for (i = 0; i < 4; i++) { q_send[i] = (int16_t)(vqfQuat[i] * 32768); }

	return 0;
}

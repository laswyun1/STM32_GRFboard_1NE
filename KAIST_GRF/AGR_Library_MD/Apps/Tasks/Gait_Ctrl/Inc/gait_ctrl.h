#ifndef GAIT_CTRL_INC_GAIT_CTRL_H_
#define GAIT_CTRL_INC_GAIT_CTRL_H_

#include "module.h"

#include <stdbool.h>

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "error_dictionary.h"

// For IMU //
#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"
#include "widm_algorithms.h"
#include "msg_hdlr.h"

// For Quaternion //
#include "vqf.h"
//#include "spi.h"

#include "data_object_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define NORM_CUTOFF_FREQ	3

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern WIDM_GaitData_t widmGaitDataObj;
extern WIDM_AngleData_t widmAngleDataObj;
extern WIDM_AttachCase_t widmAttachCaseObj;

extern float incDeg;
extern float incVel;

extern VQF_MagCalib_t vqfMagCalibObj;

extern int16_t q_send[4];

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitCtrl(void);
void RunGaitCtrl(void* params);

#endif /* GAIT_CTRL_INC_GAIT_CTRL_H_ */

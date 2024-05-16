#ifndef GRF_CTRL_INC_GRF_CTRL_H_
#define GRF_CTRL_INC_GRF_CTRL_H_

#include "module.h"

#include <stdbool.h>
#include <math.h>

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "error_dictionary.h"

// For GRF //
#include "msg_hdlr.h"
#include "data_object_common.h"
#include "ioif_adc_common.h"

#include "ioif_flash_common.h"
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define Volt2strain    2000 * 1000 / 3 / 1100; // Gain = 1100

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef struct _GRF_Data_t {
	int32_t rawS2offset[3];
	int32_t rawS3offset[3];
	int32_t rawS4offset[3];

	int32_t rawS2[3];
	int32_t rawS3[3];
	int32_t rawS4[3];

	int32_t rawS2LPF[3];
	int32_t rawS3LPF[3];
	int32_t rawS4LPF[3];

	float voltS2[3];
	float voltS3[3];
	float voltS4[3];

	float F_S2[3];
	float F_S3[3];
	float F_S4[3];

	float F_FM_filt[3];
	float F_FL_filt[3];
	float F_Ba_filt[3];

	float F_FM_filt_f[3];
	float F_FL_filt_f[3];
	float F_Ba_filt_f[3];

	float F_FM_f[3];
	float F_FL_f[3];
	float F_Ba_f[3];

	float F_FM_prev[3];
	float F_FL_prev[3];
	float F_Ba_prev[3];

	float F_FM_LPF[3];
	float F_FL_LPF[3];
	float F_Ba_LPF[3];

	float F_FM_LPF_X;
	float F_FM_LPF_Y;
	float F_FM_LPF_Z;

	float F_FL_LPF_X;
	float F_FL_LPF_Y;
	float F_FL_LPF_Z;

	float F_Ba_LPF_X;
	float F_Ba_LPF_Y;
	float F_Ba_LPF_Z;

	float F_FM_norm;
	float F_FL_norm;
	float F_Ba_norm;

	float F_FM[3];
	float F_FL[3];
	float F_Ba[3];

	float Force_FM_X;
	float Force_FM_Y;
	float Force_FM_Z;

	float Force_FL_X;
	float Force_FL_Y;
	float Force_FL_Z;

	float Force_Ba_X;
	float Force_Ba_Y;
	float Force_Ba_Z;

	float GRF_X;
	float GRF_Y;
	float GRF_Z;

	float COP_X;
	float COP_Y;
	float COP_Z;

} GRF_Data_t;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


extern GRF_Data_t GrfDataObj;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGrfCtrl(void);
void RunGrfCtrl(void* params);

#endif /* GRF_CTRL_INC_GRF_CTRL_H_ */

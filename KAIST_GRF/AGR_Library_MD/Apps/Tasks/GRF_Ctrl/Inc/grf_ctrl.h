#ifndef GRF_CTRL_INC_GRF_CTRL_H_
#define GRF_CTRL_INC_GRF_CTRL_H_

#include "module.h"

#include <stdbool.h>

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "error_dictionary.h"

// For GRF //
#include "msg_hdlr.h"
#include "data_object_common.h"
#include "ioif_adc_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef struct _GRF_Data_t {
	uint32_t rawS2offset[3];
	uint32_t rawS3offset[3];
	uint32_t rawS4offset[3];

	uint32_t rawS2[3];
	uint32_t rawS3[3];
	uint32_t rawS4[3];

	float voltS2[3];
	float voltS3[3];
	float voltS4[3];
} GRF_Data_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */






/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGrfCtrl(void);
void RunGrfCtrl(void* params);

#endif /* GRF_CTRL_INC_GRF_CTRL_H_ */

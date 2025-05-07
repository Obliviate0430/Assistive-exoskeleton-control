/*
 * File: Tree_model_stair_initialize.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 28-Mar-2024 14:04:52
 */

/* Include Files */
#include "Tree_model_stair_initialize.h"
#include "Tree_model_stair.h"
#include "Tree_model_stair_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void Tree_model_stair_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_Tree_model_stair = true;
}

/*
 * File trailer for Tree_model_stair_initialize.c
 *
 * [EOF]
 */

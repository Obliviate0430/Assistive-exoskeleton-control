/*
 * File: CompactClassificationTree.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 28-Mar-2024 14:04:52
 */

/* Include Files */
#include "CompactClassificationTree.h"
#include "Tree_model_stair.h"
#include "rt_nonfinite.h"

/* Function Definitions */

/*
 * Arguments    : const double obj_CutPredictorIndex[189]
 *                const double obj_Children[378]
 *                const double obj_CutPoint[189]
 *                const double obj_PruneList_data[]
 *                const bool obj_NanCutPoints[189]
 *                const double obj_ClassNames[3]
 *                const double obj_Cost[9]
 *                const double obj_ClassProbability[567]
 *                const float Xin[18]
 * Return Type  : double
 */
double c_CompactClassificationTree_pre(const double obj_CutPredictorIndex[189],
  const double obj_Children[378], const double obj_CutPoint[189], const double
  obj_PruneList_data[], const bool obj_NanCutPoints[189], const double
  obj_ClassNames[3], const double obj_Cost[9], const double
  obj_ClassProbability[567], const float Xin[18])
{
  int i;
  int m;
  double x[18];
  double unusedU4[3];
  int k;
  bool exitg1;
  double ex;
  double d;
  for (i = 0; i < 18; i++) {
    x[i] = Xin[i];
  }

  m = 0;
  while (!((obj_PruneList_data[m] <= 0.0) || rtIsNaN(x[(int)
           obj_CutPredictorIndex[m] - 1]) || obj_NanCutPoints[m])) {
    if (x[(int)obj_CutPredictorIndex[m] - 1] < obj_CutPoint[m]) {
      m = (int)obj_Children[m << 1] - 1;
    } else {
      m = (int)obj_Children[(m << 1) + 1] - 1;
    }
  }

  for (i = 0; i < 3; i++) {
    unusedU4[i] = (obj_ClassProbability[m] * obj_Cost[3 * i] +
                   obj_ClassProbability[m + 189] * obj_Cost[3 * i + 1]) +
      obj_ClassProbability[m + 378] * obj_Cost[3 * i + 2];
  }

  if (!rtIsNaN(unusedU4[0])) {
    m = 1;
  } else {
    m = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaN(unusedU4[k - 1])) {
        m = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (m == 0) {
    m = 1;
  } else {
    ex = unusedU4[m - 1];
    i = m + 1;
    for (k = i; k < 4; k++) {
      d = unusedU4[k - 1];
      if (ex > d) {
        ex = d;
        m = k;
      }
    }
  }

  return obj_ClassNames[m - 1];
}

/*
 * File trailer for CompactClassificationTree.c
 *
 * [EOF]
 */

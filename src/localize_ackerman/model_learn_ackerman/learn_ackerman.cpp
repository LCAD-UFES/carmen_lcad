#include "learn_ackerman.h"

#define SQR(A) ((A)*(A))

struct TLog_struct {
  float C, D, T;
  float dist, turn;
};
typedef struct TLog_struct TLog;

FILE *outFile;


carmen_inline void LeastSquares(double array[2][2], double out[2])
{
  double x, y;

  x = ((out[0]*array[1][1]) - (out[1]*array[0][1])) / ((array[0][0]*array[1][1]) - (array[0][1]*array[1][0]));
  y = ((array[0][0]*out[1]) - (array[1][0]*out[0])) / ((array[0][0]*array[1][1]) - (array[0][1]*array[1][0]));

  out[0] = x;
  out[1] = y;
}


void Learn(TPath *truePath) 
{
  int cnt;
  double total;
  double md, mt, mC, mD, mT, vd, vt, vC, vD, vT;

  TLog log[MAX_ITERATIONS];
  double meanNormal[2][2], varNormal[2][2];
  double meanC[2], meanD[2], meanT[2], varC[2], varD[2], varT[2];

  TPath *path;

  path = truePath->next;

  // determine mean and var of all input.
  cnt = 0;
  md = mt = mC = mD = mT = 0;
  while (path != NULL) {
    log[cnt].dist = path->dist;
    log[cnt].turn = path->turn;
    log[cnt].C = path->C;
    log[cnt].D = path->D;
    log[cnt].T = path->T;

    md = md + log[cnt].dist;
    mt = mt + log[cnt].turn;
    mC = mC + log[cnt].C;
    mD = mD + log[cnt].D;
    mT = mT + log[cnt].T;

    path = path->next;
    cnt++;
    if (cnt > MAX_ITERATIONS)
      path = NULL;
  }
  total = cnt;

  mt = mt / total;
  md = md / total;
  mC = mC / total;
  mD = mD / total;
  mT = mT / total;

  cnt = 0;
  vd = vt = vC = vD = vT = 0;
  for (cnt=0; cnt < total; cnt++) {
    vd = vd + SQR(md-log[cnt].dist);
    vt = vt + SQR(mt-log[cnt].turn);
    vC = vC + SQR(mC-log[cnt].C);
    vD = vD + SQR(mD-log[cnt].D);
    vT = vT + SQR(mT-log[cnt].T);
  }
  vd = sqrt(vd / total);
  vt = sqrt(vt / total);
  vC = sqrt(vC / total);
  vD = sqrt(vD / total);
  vT = sqrt(vT / total);

  meanNormal[0][0] = 0;
  meanNormal[0][1] = 0;
  meanNormal[1][1] = 0;
  meanC[0] = meanC[1] = 0;
  meanD[0] = meanD[1] = 0;
  meanT[0] = meanT[1] = 0;

  for (cnt=0; cnt < total; cnt++) {
    meanNormal[0][0] = meanNormal[0][0] + SQR(log[cnt].dist);
    meanNormal[0][1] = meanNormal[0][1] + (log[cnt].dist*log[cnt].turn);
    meanNormal[1][1] = meanNormal[1][1] + SQR(log[cnt].turn);

    meanC[0] = meanC[0] + (log[cnt].C * log[cnt].dist);
    meanC[1] = meanC[1] + (log[cnt].C * log[cnt].turn);
    meanD[0] = meanD[0] + (log[cnt].D * log[cnt].dist);
    meanD[1] = meanD[1] + (log[cnt].D * log[cnt].turn);
    meanT[0] = meanT[0] + (log[cnt].T * log[cnt].dist);
    meanT[1] = meanT[1] + (log[cnt].T * log[cnt].turn);
  }

  meanNormal[1][0] = meanNormal[0][1];

  LeastSquares(meanNormal, meanC);
  LeastSquares(meanNormal, meanD);
  LeastSquares(meanNormal, meanT);


  /////////////////////////////////////////////////////////////////////////
  // Second Pass, to compute the variances from our just-computed means  //
  /////////////////////////////////////////////////////////////////////////

  varNormal[0][0] = 0;
  varNormal[0][1] = 0;
  varNormal[1][0] = 0;
  varNormal[1][1] = 0;
  varC[0] = 0;
  varC[1] = 0;
  varD[0] = 0;
  varD[1] = 0;
  varT[0] = 0;
  varT[1] = 0;

  for (cnt=0; cnt < total; cnt++) {
    varNormal[0][0] = varNormal[0][0] + SQR(SQR(log[cnt].dist));
    varNormal[0][1] = varNormal[0][1] + (SQR(log[cnt].dist)*SQR(log[cnt].turn));
    varNormal[1][1] = varNormal[1][1] + SQR(SQR(log[cnt].turn));

    varC[0] = varC[0] + SQR(log[cnt].C - (meanC[0]*log[cnt].dist) - (meanC[1]*log[cnt].turn)) * SQR(log[cnt].dist);
    varC[1] = varC[1] + SQR(log[cnt].C - (meanC[0]*log[cnt].dist) - (meanC[1]*log[cnt].turn)) * SQR(log[cnt].turn);

    varD[0] = varD[0] + SQR(log[cnt].D - (meanD[0]*log[cnt].dist) - (meanD[1]*log[cnt].turn)) * SQR(log[cnt].dist);
    varD[1] = varD[1] + SQR(log[cnt].D - (meanD[0]*log[cnt].dist) - (meanD[1]*log[cnt].turn)) * SQR(log[cnt].turn);

    varT[0] = varT[0] + SQR(log[cnt].T - (meanT[0]*log[cnt].dist) - (meanT[1]*log[cnt].turn)) * SQR(log[cnt].dist);
    varT[1] = varT[1] + SQR(log[cnt].T - (meanT[0]*log[cnt].dist) - (meanT[1]*log[cnt].turn)) * SQR(log[cnt].turn);
  }

  varNormal[1][0] = varNormal[0][1];

  LeastSquares(varNormal, varC);
  LeastSquares(varNormal, varD);
  LeastSquares(varNormal, varT);

  if (varC[0] < 0) varC[0] = 0;
  if (varC[1] < 0) varC[1] = 0;
  if (varD[0] < 0) varD[0] = 0;
  if (varD[1] < 0) varD[1] = 0;
  if (varT[0] < 0) varT[0] = 0;
  if (varT[1] < 0) varT[1] = 0;

  fprintf(outFile, "C: %.4f %.4f  %.4f %.4f   [%.6f %.6f]\n", meanC[0], meanC[1], sqrt(varC[0]), sqrt(varC[1]), varC[0], varC[1]);
  fprintf(outFile, "D: %.4f %.4f  %.4f %.4f   [%.6f %.6f]\n", meanD[0], meanD[1], sqrt(varD[0]), sqrt(varD[1]), varD[0], varD[1]);
  fprintf(outFile, "T: %.4f %.4f  %.4f %.4f   [%.6f %.6f]\n", meanT[0], meanT[1], sqrt(varT[0]), sqrt(varT[1]), varT[0], varT[1]);
  fprintf(outFile, "\n");

  meanC_D = meanC[0];
  meanC_T = meanC[1];
  varC_D = sqrt(varC[0]);
  varC_T = sqrt(varC[1]);
  meanD_D = meanD[0];
  meanD_T = meanD[1];
  varD_D = sqrt(varD[0]);
  varD_T = sqrt(varD[1]);
  meanT_D = meanT[0];
  meanT_T = meanT[1];
  varT_D = sqrt(varT[0]);
  varT_T = sqrt(varT[1]);
}




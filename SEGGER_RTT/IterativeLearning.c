
#include "IterativeLearning.h"
#include "algorithm.h"
#include "skeleton.h"
#include "main.h"
#include "math.h"
#include "my_math.h"
#include "motor_can.h"

// float LErrorLast[OUTPUT_LENGTH]; // 左侧ILC上一时刻误差，预留给pd+ILC控制
// float RErrorLast[OUTPUT_LENGTH];
// float LError[OUTPUT_LENGTH]; // 左侧ILC误差
// float RError[OUTPUT_LENGTH];
// float IlcOutputL[OUTPUT_LENGTH]; // 左侧ILC前馈量，单位mm，
// float IlcOutputR[OUTPUT_LENGTH];
// float LearningRate = 0.1;
// int LClearIndex, RClearIndex; // 从Index位置开始置零
// float ttt;
// float Tick; // 计算误差时的Tick
// /*
// 函数功能:初始化迭代学习控制参数
// 返回值:无
// */
// void InitILC() // 初始化迭代学习控制参数
// {
//   ;
// }

// /*
// 函数功能:更新迭代学习控制的误差
// 参数:DesireForce - 期望力
//   CurrentForce - 实际力
//   Index - 误差数组索引
// 返回值:无
// */
// void UpdateError(float DesireForce, float CurrentForce, float Index) // 更新当前期望力和实际力之间的误差
// {
//   int index;
//   index = RoundToNearest(Index);
//   if (input == LeftFlag)
//   {
//     LErrorLast[index] = LError[index];
//     LError[index] = DesireForce - CurrentForce;
//     LClearIndex = index;
//   }
//   else if (input == RightFlag)
//   {
//     RErrorLast[index] = RError[index];
//     RError[index] = DesireForce - CurrentForce;
//     RClearIndex = index;
//   }
//   else
//   {
//     ;
//   }
// }
// /*
// 函数功能:更新迭代学习控制的误差
// 参数:ForceError - 期望力与实际力的误差
//   Index - 误差数组索引
// 返回值:无
// */
// void UpdateError2(float ForceError, float Index) // 更新当前期望力和实际力之间的误差
// {
//   int index;
//   index = Floor(Index);
//   if (input == LeftFlag && Fd_Left > 0)
//   {
//     if (absoluteValue(ForceError) >= 6) // 忽略小误差
//     {
//       LError[index] = ForceError;
//     }
//     else
//     {
//       LError[index] = 0;
//     }
//     LClearIndex = index;
//   }
//   else if (input == RightFlag && Fd_Right > 0)
//   {
//     if (absoluteValue(ForceError) >= 6)
//     {
//       RError[index] = ForceError;
//     }
//     else
//     {
//       LError[index] = 0;
//     }
//     RClearIndex = index;
//   }
//   else
//   {
//     ;
//   }
// }
// /*
// 函数功能:更新迭代学习控制的单个增益
// 参数:Index - 误差数组索引
// 返回值:无
// */
// void UpdateGain(float Index)
// {
//   int index;
//   index = RoundToNearest(Index);
//   if (input == LeftFlag)
//   {
//     IlcOutputL[index] += LearningRate * LError[index];
//   }
//   else if (input == RightFlag)
//   {
//     IlcOutputR[index] += LearningRate * RError[index];
//   }
//   else
//   {
//     ;
//   }
// }
// /*
// 函数功能:更新迭代学习控制的所有增益
// 参数:无
// 返回值:无
// */
// void UpdateGain2(void)
// {
//   if (current_state == LeftAssistance) // 左侧助力切换至另一侧或不助力时，更新增益
//   {
//     for (int i = 0; i < OUTPUT_LENGTH; i++)
//     {
//       IlcOutputL[i] += LearningRate * LError[i];
//     }
//   }
//   else if (current_state == RightAssistance)
//   {
//     for (int i = 0; i < OUTPUT_LENGTH; i++)
//     {
//       IlcOutputR[i] += LearningRate * RError[i];
//     }
//   }
//   else
//     ;
// }
// /*
// 函数功能:输出迭代学习控制前馈量，计算对应鲍登线位移，转化为电机角度
// 参数:Index - 误差数组索引
// 返回值:电机角度前馈量
// */
// float OutputGain(float Index)
// {
//   int index;
//   index = RoundToNearest(Index);
//   if (input == LeftFlag)
//   {
//     return DisplacementToDegree(ForceToDisplacement(IlcOutputL[index])); // 四舍五入后转化为电机位移
//   }
//   else if (input == RightFlag)
//   {
//     return DisplacementToDegree(ForceToDisplacement(IlcOutputR[index]));
//   }
//   else
//   {
//     ;
//   }
// }

/*
函数功能:步态周期结束，更新增益，清除Index以后的误差值
参数:input - 判断清除哪侧的误差
    LClearIndex - 最后一个记录的误差值索引
返回值:无
*/
// void ClearError(void)
// {
//   int i;

//   if (input == RightFlag)
//   {
//     for (i = LClearIndex; i <= OUTPUT_LENGTH; i++)
//     {
//       LError[i] = 0;
//     }
//   }
//   else if (input == LeftFlag)
//   {
//     for (i = RClearIndex; i <= OUTPUT_LENGTH; i++)
//     {
//       RError[i] = 0;
//     }
//   }
//   else // input==NoFlag,两侧都清除
//   {
//     for (i = RClearIndex; i <= OUTPUT_LENGTH; i++)
//     {
//       RError[i] = 0;
//     }
//     for (i = LClearIndex; i <= OUTPUT_LENGTH; i++)
//     {
//       LError[i] = 0;
//     }
//   }
//   UpdateGain2(); // 更新增益
// }

/*
函数功能:采样
参数:无
返回值:无
*/
// void TimingSamplingLeft(void)
// {
//   //	float Tick;
//   Tick = (float)LtickCnt / (float)TimerLeft.Cycletime * 100;
//   if (Tick < 100)
//   {
//     UpdateError2(Fd_Left - (float)skeletonAdDmaBuf[0] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096, Tick);
//   }
//   else
//     ;
//   // UpdateError2(Fd_Left-force_reference,Tick);

//   //	UpdateGain(Tick);//更新增益
// }
// void TimingSamplingRight(void)
// {
//   //	float Tick;
//   Tick = (float)RtickCnt / (float)TimerRight.Cycletime * 100;
//   if (Tick < 100)
//   {
//     UpdateError2(Fd_Right - (float)skeletonAdDmaBuf[1] * (3.3 / 10) * 250 * 0.4536 * 9.8 / 4096, Tick);
//   }
//   else
//     ;
//   //	UpdateGain(Tick);//更新增益
// }
/*
函数功能:根据步态周期更新迭代频率
参数:无
返回值:无
*/
// void UpdateSampleRate(void)
// {
// //	float Tick;
// 	Tick = (float)TimerLeft.Cycletime*1000-1;
// 	updateTimerReloadValue((uint16_t)Tick);
// }

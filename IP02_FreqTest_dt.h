/*
 * IP02_FreqTest_dt.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "IP02_FreqTest".
 *
 * Model version              : 18.0
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Tue Mar 17 18:45:06 2026
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(t_uint64),
  sizeof(t_card),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "t_uint64",
  "t_card",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&IP02_FreqTest_B.Chirp_Vcmd), 0, 0, 6 }
  ,

  { (char_T *)(&IP02_FreqTest_DW.HILInitialize_AIMinimums[0]), 0, 0, 16 },

  { (char_T *)(&IP02_FreqTest_DW.To_Host_File_PointsWritten), 15, 0, 1 },

  { (char_T *)(&IP02_FreqTest_DW.HILInitialize_Card), 16, 0, 1 },

  { (char_T *)(&IP02_FreqTest_DW.Chirp_Vcmd_PWORK.TimePtr), 11, 0, 5 },

  { (char_T *)(&IP02_FreqTest_DW.HILInitialize_ClockModes), 6, 0, 6 },

  { (char_T *)(&IP02_FreqTest_DW.To_Host_File_SamplesCount), 7, 0, 2 },

  { (char_T *)(&IP02_FreqTest_DW.Chirp_Vcmd_IWORK.PrevIndex), 10, 0, 1 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  8U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&IP02_FreqTest_P.A_cart[0]), 0, 0, 14 },

  { (char_T *)(&IP02_FreqTest_P.MotorCommand_channels), 7, 0, 2 },

  { (char_T *)(&IP02_FreqTest_P.HILInitialize_OOTerminate), 0, 0, 18 },

  { (char_T *)(&IP02_FreqTest_P.HILInitialize_CKChannels), 6, 0, 5 },

  { (char_T *)(&IP02_FreqTest_P.HILInitialize_AIChannels[0]), 7, 0, 9 },

  { (char_T *)(&IP02_FreqTest_P.HILInitialize_Active), 8, 0, 37 },

  { (char_T *)(&IP02_FreqTest_P.To_Host_File_VarName[0]), 3, 0, 15 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  7U,
  rtPTransitions
};

/* [EOF] IP02_FreqTest_dt.h */

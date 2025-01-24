/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    debug_config.h
  * @author  MCD Application Team
  * @brief   Real Time Debug module general configuration file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_conf.h"

#if(CFG_RT_DEBUG_GPIO_MODULE == 1)

/***********************************/
/** Debug configuration selection **/
/***********************************/
/* Debug configuration for System purpose */
#define USE_RT_DEBUG_CONFIGURATION_SYSTEM                     (0)

/* Debug configuration for BLE purpose */
#define USE_RT_DEBUG_CONFIGURATION_BLE                        (0)

/* Debug configuration for MAC purpose */
#define USE_RT_DEBUG_CONFIGURATION_MAC                        (0)

/* Debug configuration for COEX purpose */
#define USE_RT_DEBUG_CONFIGURATION_COEX                       (0)

/*********************************/
/** GPIO debug signal selection **/
/*********************************/

/* System clock manager - System clock config */
#define USE_RT_DEBUG_SCM_SYSTEM_CLOCK_CONFIG                  (0)
#define GPIO_DEBUG_SCM_SYSTEM_CLOCK_CONFIG                    {GPIOA, GPIO_PIN_12}

/* System clock manager - Setup */
#define USE_RT_DEBUG_SCM_SETUP                                (0)
#define GPIO_DEBUG_SCM_SETUP                                  {GPIOA, GPIO_PIN_5}

/* System clock manager - HSE RDY interrupt handling */
#define USE_RT_DEBUG_SCM_HSERDY_ISR                           (0)
#define GPIO_DEBUG_SCM_HSERDY_ISR                             {GPIOA, GPIO_PIN_15}

#define USE_RT_DEBUG_ADC_ACTIVATION                           (0)
#define GPIO_DEBUG_ADC_ACTIVATION                             {GPIOB, GPIO_PIN_4}

#define USE_RT_DEBUG_ADC_DEACTIVATION                         (0)
#define GPIO_DEBUG_ADC_DEACTIVATION                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADC_TEMPERATURE_ACQUISITION              (0)
#define GPIO_DEBUG_ADC_TEMPERATURE_ACQUISITION                {GPIOB, GPIO_PIN_8}

#define USE_RT_DEBUG_RNG_ENABLE                               (0)
#define GPIO_DEBUG_RNG_ENABLE                                 {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RNG_DISABLE                              (0)
#define GPIO_DEBUG_RNG_DISABLE                                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RNG_GEN_RAND_NUM                         (0)
#define GPIO_DEBUG_RNG_GEN_RAND_NUM                           {GPIOB, GPIO_PIN_12}

#define USE_RT_DEBUG_LOW_POWER_STOP_MODE_ENTER                (0)
#define GPIO_DEBUG_LOW_POWER_STOP_MODE_ENTER                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LOW_POWER_STOP_MODE_EXIT                 (0)
#define GPIO_DEBUG_LOW_POWER_STOP_MODE_EXIT                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LOW_POWER_STOP_MODE_ACTIVE               (0)
#define GPIO_DEBUG_LOW_POWER_STOP_MODE_ACTIVE                 {GPIOB, GPIO_PIN_3}

#define USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ENTER             (0)
#define GPIO_DEBUG_LOW_POWER_STANDBY_MODE_ENTER               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_EXIT              (0)
#define GPIO_DEBUG_LOW_POWER_STANDBY_MODE_EXIT                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE            (0)
#define GPIO_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE              {GPIOB, GPIO_PIN_15}

#define USE_RT_DEBUG_HCI_READ_DONE                            (0)
#define GPIO_DEBUG_HCI_READ_DONE                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_HCI_RCVD_CMD                             (0)
#define GPIO_DEBUG_HCI_RCVD_CMD                               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_HCI_WRITE_DONE                           (0)
#define GPIO_DEBUG_HCI_WRITE_DONE                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_EVNT_UPDATE                       (0)
#define GPIO_DEBUG_SCHDLR_EVNT_UPDATE                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_TIMER_SET                         (0)
#define GPIO_DEBUG_SCHDLR_TIMER_SET                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_PHY_CLBR_TIMER                    (0)
#define GPIO_DEBUG_SCHDLR_PHY_CLBR_TIMER                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_EVNT_SKIPPED                      (0)
#define GPIO_DEBUG_SCHDLR_EVNT_SKIPPED                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE                    (0)
#define GPIO_DEBUG_SCHDLR_HNDL_NXT_TRACE                      {GPIOA, GPIO_PIN_12}

#define USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_DETEDTED              (0)
#define GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_DETEDTED                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_GAP_CHECK             (0)
#define GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_GAP_CHECK               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_TIME_CHECK            (0)
#define GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_TIME_CHECK              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_TRACE                 (0)
#define GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_TRACE                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_EVNT_RGSTR                        (0)
#define GPIO_DEBUG_SCHDLR_EVNT_RGSTR                          {GPIOB, GPIO_PIN_8}

#define USE_RT_DEBUG_SCHDLR_ADD_CONFLICT_Q                    (0)
#define GPIO_DEBUG_SCHDLR_ADD_CONFLICT_Q                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_HNDL_MISSED_EVNT                  (0)
#define GPIO_DEBUG_SCHDLR_HNDL_MISSED_EVNT                    {GPIOA, GPIO_PIN_5}

#define USE_RT_DEBUG_SCHDLR_UNRGSTR_EVNT                      (0)
#define GPIO_DEBUG_SCHDLR_UNRGSTR_EVNT                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE                   (0)
#define GPIO_DEBUG_SCHDLR_EXEC_EVNT_TRACE                     {GPIOA, GPIO_PIN_15}

#define USE_RT_DEBUG_SCHDLR_EXEC_EVNT_PROFILE                 (0)
#define GPIO_DEBUG_SCHDLR_EXEC_EVNT_PROFILE                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_EXEC_EVNT_ERROR                   (0)
#define GPIO_DEBUG_SCHDLR_EXEC_EVNT_ERROR                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_SCHDLR_EXEC_EVNT_WINDOW_WIDENING         (0)
#define GPIO_DEBUG_SCHDLR_EXEC_EVNT_WINDOW_WIDENING           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_CMN_CLR_ISR                        (0)
#define GPIO_DEBUG_LLHWC_CMN_CLR_ISR                          {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLWCC_CMN_HG_ISR                         (0)
#define GPIO_DEBUG_LLWCC_CMN_HG_ISR                           {GPIOA, GPIO_PIN_15}

#define USE_RT_DEBUG_LLHWC_CMN_LW_ISR                         (0)
#define GPIO_DEBUG_LLHWC_CMN_LW_ISR                           {GPIOA, GPIO_PIN_12}

#define USE_RT_DEBUG_LLHWC_CMN_CLR_TIMER_ERROR                (0)
#define GPIO_DEBUG_LLHWC_CMN_CLR_TIMER_ERROR                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_LL_ISR                             (0)
#define GPIO_DEBUG_LLHWC_LL_ISR                               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_SPLTMR_SET                         (0)
#define GPIO_DEBUG_LLHWC_SPLTMR_SET                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_SPLTMR_GET                         (0)
#define GPIO_DEBUG_LLHWC_SPLTMR_GET                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_LOW_ISR                            (0)
#define GPIO_DEBUG_LLHWC_LOW_ISR                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_STOP_SCN                           (0)
#define GPIO_DEBUG_LLHWC_STOP_SCN                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_WAIT_ENVT_ON_AIR                   (0)
#define GPIO_DEBUG_LLHWC_WAIT_ENVT_ON_AIR                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_SET_CONN_EVNT_PARAM                (0)
#define GPIO_DEBUG_LLHWC_SET_CONN_EVNT_PARAM                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_POST_EVNT                                (0)
#define GPIO_DEBUG_POST_EVNT                                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_HNDL_ALL_EVNTS                           (0)
#define GPIO_DEBUG_HNDL_ALL_EVNTS                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PROCESS_EVNT                             (0)
#define GPIO_DEBUG_PROCESS_EVNT                               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PROCESS_ISO_DATA                         (0)
#define GPIO_DEBUG_PROCESS_ISO_DATA                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ALLOC_TX_ISO_EMPTY_PKT                   (0)
#define GPIO_DEBUG_ALLOC_TX_ISO_EMPTY_PKT                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIG_FREE_EMPTY_PKTS                      (0)
#define GPIO_DEBUG_BIG_FREE_EMPTY_PKTS                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_OK                 (0)
#define GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_OK                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_CRC                (0)
#define GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_CRC                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_NoRX               (0)
#define GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_NoRX                 {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_TRACE              (0)
#define GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_TRACE                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISO_HNDL_SDU                             (0)
#define GPIO_DEBUG_ISO_HNDL_SDU                               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LL_INTF_INIT                             (0)
#define GPIO_DEBUG_LL_INTF_INIT                               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_DATA_TO_CNTRLR                           (0)
#define GPIO_DEBUG_DATA_TO_CNTRLR                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_FREE_LL_PKT_HNDLR                        (0)
#define GPIO_DEBUG_FREE_LL_PKT_HNDLR                          {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PHY_INIT_CLBR_TRACE                      (0)
#define GPIO_DEBUG_PHY_INIT_CLBR_TRACE                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PHY_RUNTIME_CLBR_TRACE                   (0)
#define GPIO_DEBUG_PHY_RUNTIME_CLBR_TRACE                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PHY_CLBR_ISR                             (0)
#define GPIO_DEBUG_PHY_CLBR_ISR                               {GPIOB, GPIO_PIN_3}

#define USE_RT_DEBUG_PHY_INIT_CLBR_SINGLE_CH                  (0)
#define GPIO_DEBUG_PHY_INIT_CLBR_SINGLE_CH                    {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PHY_CLBR_STRTD                           (0)
#define GPIO_DEBUG_PHY_CLBR_STRTD                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PHY_CLBR_EXEC                            (0)
#define GPIO_DEBUG_PHY_CLBR_EXEC                              {GPIOB, GPIO_PIN_4}

#define USE_RT_DEBUG_RCO_STRT_STOP_RUNTIME_CLBR_ACTV          (0)
#define GPIO_DEBUG_RCO_STRT_STOP_RUNTIME_CLBR_ACTV            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RCO_STRT_STOP_RUNTIME_RCO_CLBR           (0)
#define GPIO_DEBUG_RCO_STRT_STOP_RUNTIME_RCO_CLBR             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_SWT           (0)
#define GPIO_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_SWT             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_TRACE         (0)
#define GPIO_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_TRACE           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RCO_ISR_TRACE                            (0)
#define GPIO_DEBUG_RCO_ISR_TRACE                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RCO_ISR_COMPENDATE                       (0)
#define GPIO_DEBUG_RCO_ISR_COMPENDATE                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_STRT_TX                              (0)
#define GPIO_DEBUG_RAL_STRT_TX                                {GPIOA, GPIO_PIN_5}

#define USE_RT_DEBUG_RAL_ISR_TIMER_ERROR                      (0)
#define GPIO_DEBUG_RAL_ISR_TIMER_ERROR                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_ISR_TRACE                            (0)
#define GPIO_DEBUG_RAL_ISR_TRACE                              {GPIOB, GPIO_PIN_3}

#define USE_RT_DEBUG_RAL_STOP_OPRTN                           (0)
#define GPIO_DEBUG_RAL_STOP_OPRTN                             {GPIOB, GPIO_PIN_8}

#define USE_RT_DEBUG_RAL_STRT_RX                              (0)
#define GPIO_DEBUG_RAL_STRT_RX                                {GPIOB, GPIO_PIN_12}

#define USE_RT_DEBUG_RAL_DONE_CLBK_TX                         (0)
#define GPIO_DEBUG_RAL_DONE_CLBK_TX                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_DONE_CLBK_RX                         (0)
#define GPIO_DEBUG_RAL_DONE_CLBK_RX                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_DONE_CLBK_ED                         (0)
#define GPIO_DEBUG_RAL_DONE_CLBK_ED                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_ED_SCAN                              (0)
#define GPIO_DEBUG_RAL_ED_SCAN                                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ERROR_MEM_CAP_EXCED                      (0)
#define GPIO_DEBUG_ERROR_MEM_CAP_EXCED                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ERROR_COMMAND_DISALLOWED                 (0)
#define GPIO_DEBUG_ERROR_COMMAND_DISALLOWED                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PTA_INIT                                 (0)
#define GPIO_DEBUG_PTA_INIT                                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PTA_EN                                   (0)
#define GPIO_DEBUG_PTA_EN                                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_PTA_SET_EN                         (0)
#define GPIO_DEBUG_LLHWC_PTA_SET_EN                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_PTA_SET_PARAMS                     (0)
#define GPIO_DEBUG_LLHWC_PTA_SET_PARAMS                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_COEX_STRT_ON_IDLE                        (0)
#define GPIO_DEBUG_COEX_STRT_ON_IDLE                          {GPIOB, GPIO_PIN_15}

#define USE_RT_DEBUG_COEX_ASK_FOR_AIR                         (0)
#define GPIO_DEBUG_COEX_ASK_FOR_AIR                           {GPIOB, GPIO_PIN_3}

#define USE_RT_DEBUG_COEX_TIMER_EVNT_CLBK                     (0)
#define GPIO_DEBUG_COEX_TIMER_EVNT_CLBK                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_COEX_STRT_ONE_SHOT                       (0)
#define GPIO_DEBUG_COEX_STRT_ONE_SHOT                         {GPIOA, GPIO_PIN_5}

#define USE_RT_DEBUG_COEX_FORCE_STOP_RX                       (0)
#define GPIO_DEBUG_COEX_FORCE_STOP_RX                         {GPIOB, GPIO_PIN_12}

#define USE_RT_DEBUG_LLHWC_ADV_DONE                           (0)
#define GPIO_DEBUG_LLHWC_ADV_DONE                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_SCN_DONE                           (0)
#define GPIO_DEBUG_LLHWC_SCN_DONE                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_INIT_DONE                          (0)
#define GPIO_DEBUG_LLHWC_INIT_DONE                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_CONN_DONE                          (0)
#define GPIO_DEBUG_LLHWC_CONN_DONE                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_CIG_DONE                           (0)
#define GPIO_DEBUG_LLHWC_CIG_DONE                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_BIG_DONE                           (0)
#define GPIO_DEBUG_LLHWC_BIG_DONE                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_OS_TMR_CREATE                            (0)
#define GPIO_DEBUG_OS_TMR_CREATE                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_TIMEOUT_CBK                      (0)
#define GPIO_DEBUG_ADV_EXT_TIMEOUT_CBK                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_SCN_DUR_CBK                      (0)
#define GPIO_DEBUG_ADV_EXT_SCN_DUR_CBK                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_SCN_PERIOD_CBK                   (0)
#define GPIO_DEBUG_ADV_EXT_SCN_PERIOD_CBK                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_PRDC_SCN_TIMEOUT_CBK             (0)
#define GPIO_DEBUG_ADV_EXT_PRDC_SCN_TIMEOUT_CBK               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIS_SYNC_TIMEOUT_TMR_CBK                 (0)
#define GPIO_DEBUG_BIS_SYNC_TIMEOUT_TMR_CBK                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIS_TERM_TMR_CBK                         (0)
#define GPIO_DEBUG_BIS_TERM_TMR_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIS_TST_MODE_CBK                         (0)
#define GPIO_DEBUG_BIS_TST_MODE_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIS_TST_MODE_TMR_CBK                     (0)
#define GPIO_DEBUG_BIS_TST_MODE_TMR_CBK                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISO_POST_TMR_CBK                         (0)
#define GPIO_DEBUG_ISO_POST_TMR_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISO_TST_MODE_TMR_CBK                     (0)
#define GPIO_DEBUG_ISO_TST_MODE_TMR_CBK                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_CONN_POST_TMR_CBK                        (0)
#define GPIO_DEBUG_CONN_POST_TMR_CBK                          {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_EVNT_SCHDLR_TMR_CBK                      (0)
#define GPIO_DEBUG_EVNT_SCHDLR_TMR_CBK                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_HCI_POST_TMR_CBK                         (0)
#define GPIO_DEBUG_HCI_POST_TMR_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLCP_POST_TMR_CBK                        (0)
#define GPIO_DEBUG_LLCP_POST_TMR_CBK                          {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_ENRGY_DETECT_CBK                   (0)
#define GPIO_DEBUG_LLHWC_ENRGY_DETECT_CBK                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PRVCY_POST_TMR_CBK                       (0)
#define GPIO_DEBUG_PRVCY_POST_TMR_CBK                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ANT_PRPR_TMR_CBK                         (0)
#define GPIO_DEBUG_ANT_PRPR_TMR_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_COEX_TMR_FRC_STOP_AIR_GRANT_CBK          (0)
#define GPIO_DEBUG_COEX_TMR_FRC_STOP_AIR_GRANT_CBK            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MLME_RX_EN_TMR_CBK                       (0)
#define GPIO_DEBUG_MLME_RX_EN_TMR_CBK                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MLME_GNRC_TMR_CBK                        (0)
#define GPIO_DEBUG_MLME_GNRC_TMR_CBK                          {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MIB_JOIN_LST_TMR_CBK                     (0)
#define GPIO_DEBUG_MIB_JOIN_LST_TMR_CBK                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MLME_PWR_PRES_TMR_CBK                    (0)
#define GPIO_DEBUG_MLME_PWR_PRES_TMR_CBK                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PRESISTENCE_TMR_CBK                      (0)
#define GPIO_DEBUG_PRESISTENCE_TMR_CBK                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RADIO_PHY_PRDC_CLBK_TMR_CBK              (0)
#define GPIO_DEBUG_RADIO_PHY_PRDC_CLBK_TMR_CBK                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RADIO_CSMA_TMR_CBK                       (0)
#define GPIO_DEBUG_RADIO_CSMA_TMR_CBK                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RADIO_CSL_RCV_TMR_CBK                    (0)
#define GPIO_DEBUG_RADIO_CSL_RCV_TMR_CBK                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ED_TMR_CBK                               (0)
#define GPIO_DEBUG_ED_TMR_CBK                                 {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_DIO_EXT_TMR_CBK                          (0)
#define GPIO_DEBUG_DIO_EXT_TMR_CBK                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RCO_CLBR_TMR_CBK                         (0)
#define GPIO_DEBUG_RCO_CLBR_TMR_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_ADV_CBK                     (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_ADV_CBK                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_SCN_CBK                     (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_SCN_CBK                       {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_SCN_ERR_CBK                 (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_SCN_ERR_CBK                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CBK                (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CBK                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_ERR_CBK            (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_ERR_CBK              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIG_ADV_CBK                              (0)
#define GPIO_DEBUG_BIG_ADV_CBK                                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIG_ADV_ERR_CBK                          (0)
#define GPIO_DEBUG_BIG_ADV_ERR_CBK                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIG_SYNC_CBK                             (0)
#define GPIO_DEBUG_BIG_SYNC_CBK                               {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIG_SYNC_ERR_CBK                         (0)
#define GPIO_DEBUG_BIG_SYNC_ERR_CBK                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISO_CIS_PKT_TRNSM_RECEIVED_CBK           (0)
#define GPIO_DEBUG_ISO_CIS_PKT_TRNSM_RECEIVED_CBK             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISO_CIG_ERR_CBK                          (0)
#define GPIO_DEBUG_ISO_CIG_ERR_CBK                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_CONN_PKT_TRNSM_RECEIVED_CBK              (0)
#define GPIO_DEBUG_CONN_PKT_TRNSM_RECEIVED_CBK                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PRDC_CLBR_EXTRL_CBK                      (0)
#define GPIO_DEBUG_PRDC_CLBR_EXTRL_CBK                        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PTR_PRDC_ADV_SYNC_CBK                    (0)
#define GPIO_DEBUG_PTR_PRDC_ADV_SYNC_CBK                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_NCONN_SCN_CBK                            (0)
#define GPIO_DEBUG_NCONN_SCN_CBK                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_NCONN_ADV_CBK                            (0)
#define GPIO_DEBUG_NCONN_ADV_CBK                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_NCONN_INIT_CBK                           (0)
#define GPIO_DEBUG_NCONN_INIT_CBK                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ANT_RADIO_CMPLT_EVNT_CBK                 (0)
#define GPIO_DEBUG_ANT_RADIO_CMPLT_EVNT_CBK                   {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ANT_STACK_EVNT_CBK                       (0)
#define GPIO_DEBUG_ANT_STACK_EVNT_CBK                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_PROCESS_TMOUT_EVNT_CBK           (0)
#define GPIO_DEBUG_ADV_EXT_PROCESS_TMOUT_EVNT_CBK             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_SCN_DUR_EVNT                (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_SCN_DUR_EVNT                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_SCN_PERIODIC_EVNT           (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_SCN_PERIODIC_EVNT             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_TMOUT_EVNT         (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_TMOUT_EVNT           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CNCEL_EVNT         (0)
#define GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CNCEL_EVNT           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIS_MNGR_BIG_TERM_CBK                    (0)
#define GPIO_DEBUG_BIS_MNGR_BIG_TERM_CBK                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_BIS_MNGR_SYNC_TMOUT_CBK                  (0)
#define GPIO_DEBUG_BIS_MNGR_SYNC_TMOUT_CBK                    {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISOAL_MNGR_SDU_GEN                       (0)
#define GPIO_DEBUG_ISOAL_MNGR_SDU_GEN                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_ISO_MNGR_CIS_PROCESS_EVNT_CBK            (0)
#define GPIO_DEBUG_ISO_MNGR_CIS_PROCESS_EVNT_CBK              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_CONN_MNGR_PROCESS_EVNT_CLBK              (0)
#define GPIO_DEBUG_CONN_MNGR_PROCESS_EVNT_CLBK                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_CONN_MNGR_UPDT_CONN_PARAM_CBK            (0)
#define GPIO_DEBUG_CONN_MNGR_UPDT_CONN_PARAM_CBK              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_EVNT_SCHDLR_HW_EVNT_CMPLT                (0)
#define GPIO_DEBUG_EVNT_SCHDLR_HW_EVNT_CMPLT                  {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_HCI_EVENT_HNDLR                          (0)
#define GPIO_DEBUG_HCI_EVENT_HNDLR                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MLME_TMRS_CBK                            (0)
#define GPIO_DEBUG_MLME_TMRS_CBK                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_DIRECT_TX_EVNT_CBK                       (0)
#define GPIO_DEBUG_DIRECT_TX_EVNT_CBK                         {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_INDIRECT_PKT_TOUR_CBK                    (0)
#define GPIO_DEBUG_INDIRECT_PKT_TOUR_CBK                      {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RADIO_CSMA_TMR                           (0)
#define GPIO_DEBUG_RADIO_CSMA_TMR                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK                     (0)
#define GPIO_DEBUG_RAL_SM_DONE_EVNT_CBK                       {GPIOB, GPIO_PIN_4}

#define USE_RT_DEBUG_ED_TMR_HNDL                              (0)
#define GPIO_DEBUG_ED_TMR_HNDL                                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_OS_TMR_EVNT_CBK                          (0)
#define GPIO_DEBUG_OS_TMR_EVNT_CBK                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PROFILE_MARKER_PHY_WAKEUP_TIME           (0)
#define GPIO_DEBUG_PROFILE_MARKER_PHY_WAKEUP_TIME             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PROFILE_END_DRIFT_TIME                   (0)
#define GPIO_DEBUG_PROFILE_END_DRIFT_TIME                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PROC_RADIO_RCV                           (0)
#define GPIO_DEBUG_PROC_RADIO_RCV                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_EVNT_TIME_UPDT                           (0)
#define GPIO_DEBUG_EVNT_TIME_UPDT                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MAC_RECEIVE_DONE                         (0)
#define GPIO_DEBUG_MAC_RECEIVE_DONE                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_MAC_TX_DONE                              (0)
#define GPIO_DEBUG_MAC_TX_DONE                                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RADIO_APPLY_CSMA                         (0)
#define GPIO_DEBUG_RADIO_APPLY_CSMA                           {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RADIO_TRANSMIT                           (0)
#define GPIO_DEBUG_RADIO_TRANSMIT                             {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_PROC_RADIO_TX                            (0)
#define GPIO_DEBUG_PROC_RADIO_TX                              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_TX_DONE                              (0)
#define GPIO_DEBUG_RAL_TX_DONE                                {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_TX_DONE_INCREMENT_BACKOFF_COUNT      (0)
#define GPIO_DEBUG_RAL_TX_DONE_INCREMENT_BACKOFF_COUNT        {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_TX_DONE_RST_BACKOFF_COUNT            (0)
#define GPIO_DEBUG_RAL_TX_DONE_RST_BACKOFF_COUNT              {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_CONTINUE_RX                          (0)
#define GPIO_DEBUG_RAL_CONTINUE_RX                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_PERFORM_CCA                          (0)
#define GPIO_DEBUG_RAL_PERFORM_CCA                            {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_RAL_ENABLE_TRANSMITTER                   (0)
#define GPIO_DEBUG_RAL_ENABLE_TRANSMITTER                     {GPIOA, GPIO_PIN_0}

#define USE_RT_DEBUG_LLHWC_GET_CH_IDX_ALGO_2                  (0)
#define GPIO_DEBUG_LLHWC_GET_CH_IDX_ALGO_2                    {GPIOA, GPIO_PIN_0}

/* Application signal selection and GPIO assignment.
   CAN BE MODIFIED BY USER */

#define USE_RT_DEBUG_APP_APPE_INIT                            (1)
#define GPIO_DEBUG_APP_APPE_INIT                              {GPIOA, GPIO_PIN_0}

/********************************/
/** Debug configuration setup **/
/*******************************/

/*
 *
 * Debug configuration for System purpose
 *
 */
#if (USE_RT_DEBUG_CONFIGURATION_SYSTEM == 1U)
/* SCM_SETUP activation */
#undef USE_RT_DEBUG_SCM_SETUP
#define USE_RT_DEBUG_SCM_SETUP                                (1U)

/* SCM_SYSTEM_CLOCK_CONFIG activation */
#undef USE_RT_DEBUG_SCM_SYSTEM_CLOCK_CONFIG
#define USE_RT_DEBUG_SCM_SYSTEM_CLOCK_CONFIG                  (1U)

/* SCM_HSERDY_ISR activation */
#undef USE_RT_DEBUG_SCM_HSERDY_ISR
#define USE_RT_DEBUG_SCM_HSERDY_ISR                           (1U)

/* LOW_POWER_STOP_MODE_ACTIVE activation */
#undef USE_RT_DEBUG_LOW_POWER_STOP_MODE_ACTIVE
#define USE_RT_DEBUG_LOW_POWER_STOP_MODE_ACTIVE               (1U)

/* ADC_ACTIVATION activation */
#undef USE_RT_DEBUG_ADC_ACTIVATION
#define USE_RT_DEBUG_ADC_ACTIVATION                           (1U)

/* ADC_TEMPERATURE_ACQUISITION activation */
#undef USE_RT_DEBUG_ADC_TEMPERATURE_ACQUISITION
#define USE_RT_DEBUG_ADC_TEMPERATURE_ACQUISITION              (1U)

/* RNG_GEN_RAND_NUM activation */
#undef USE_RT_DEBUG_RNG_GEN_RAND_NUM
#define USE_RT_DEBUG_RNG_GEN_RAND_NUM                         (1U)

/* LOW_POWER_STANDBY_MODE_ACTIVE activation */
#undef USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE
#define USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE            (1U)

/*
 *
 * Debug configuration for BLE purpose
 *
 */
#elif (USE_RT_DEBUG_CONFIGURATION_BLE == 1U)

/* LLHWC_CMN_LW_ISR activation */
#undef USE_RT_DEBUG_LLHWC_CMN_LW_ISR
#define USE_RT_DEBUG_LLHWC_CMN_LW_ISR                         (1U)

/* LLHWC_CMN_HG_ISR activation */
#undef USE_RT_DEBUG_LLWCC_CMN_HG_ISR
#define USE_RT_DEBUG_LLWCC_CMN_HG_ISR                         (1U)

/* PHY_CLBR_EXEC activation */
#undef USE_RT_DEBUG_PHY_CLBR_EXEC
#define USE_RT_DEBUG_PHY_CLBR_EXEC                            (1U)

/* SCHDLR_EVNT_RGSTR activation */
#undef USE_RT_DEBUG_SCHDLR_EVNT_RGSTR
#define USE_RT_DEBUG_SCHDLR_EVNT_RGSTR                        (1U)

/* SCHDLR_HNDL_MISSED_EVNT activation */
#undef USE_RT_DEBUG_SCHDLR_HNDL_MISSED_EVNT
#define USE_RT_DEBUG_SCHDLR_HNDL_MISSED_EVNT                  (1U)

/* SCHDLR_HNDL_NXT_TRACE activation */
#undef USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE
#define USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE                    (1U)

/* SCHDLR_EXEC_EVNT_TRACE activation */
#undef USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE
#define USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE                   (1U)

/* PHY_CLBR_ISR activation */
#undef USE_RT_DEBUG_PHY_CLBR_ISR
#define USE_RT_DEBUG_PHY_CLBR_ISR                             (1U)

/*
 *
 * Debug configuration for MAC purpose
 *
 */
#elif (USE_RT_DEBUG_CONFIGURATION_MAC == 1U)

/* LLHWC_CMN_LW_ISR activation */
#undef USE_RT_DEBUG_LLHWC_CMN_LW_ISR
#define USE_RT_DEBUG_LLHWC_CMN_LW_ISR                         (1U)

/* LLHWC_CMN_HG_ISR activation */
#undef USE_RT_DEBUG_LLWCC_CMN_HG_ISR
#define USE_RT_DEBUG_LLWCC_CMN_HG_ISR                         (1U)

/* RAL_ISR_TRACE activation */
#undef USE_RT_DEBUG_RAL_ISR_TRACE
#define USE_RT_DEBUG_RAL_ISR_TRACE                            (1U)

/* RAL_SM_DONE_EVNT_CBK activation */
#undef USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK
#define USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK                     (1U)

/* RAL_STOP_OPRTN activation */
#undef USE_RT_DEBUG_RAL_STOP_OPRTN
#define USE_RT_DEBUG_RAL_STOP_OPRTN                           (1U)

/* RAL_STRT_RX activation */
#undef USE_RT_DEBUG_RAL_STRT_RX
#define USE_RT_DEBUG_RAL_STRT_RX                              (1U)

/* RAL_STRT_TX activation */
#undef USE_RT_DEBUG_RAL_STRT_TX
#define USE_RT_DEBUG_RAL_STRT_TX                              (1U)

/*
 *
 * Debug configuration for COEX purpose
 *
 */
#elif (USE_RT_DEBUG_CONFIGURATION_COEX == 1U)

/* COEX_ASK_FOR_AIR activation */
#undef USE_RT_DEBUG_COEX_ASK_FOR_AIR
#define USE_RT_DEBUG_COEX_ASK_FOR_AIR                         (1U)

/* COEX_FORCE_STOP_RX activation */
#undef USE_RT_DEBUG_COEX_FORCE_STOP_RX
#define USE_RT_DEBUG_COEX_FORCE_STOP_RX                       (1U)

/* COEX_STRT_ON_IDLE activation */
#undef USE_RT_DEBUG_COEX_STRT_ON_IDLE
#define USE_RT_DEBUG_COEX_STRT_ON_IDLE                        (1U)

/* COEX_STRT_ONE_SHOT activation */
#undef USE_RT_DEBUG_COEX_STRT_ONE_SHOT
#define USE_RT_DEBUG_COEX_STRT_ONE_SHOT                       (1U)

/* SCHDLR_HNDL_NXT_TRACE activation */
#undef USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE
#define USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE                    (1U)

/* SCHDLR_EXEC_EVNT_TRACE activation */
#undef USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE
#define USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE                   (1U)

/* RAL_SM_DONE_EVNT_CBK activation */
#undef USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK
#define USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK                     (1U)

/* RAL_STOP_OPRTN activation */
#undef USE_RT_DEBUG_RAL_STOP_OPRTN
#define USE_RT_DEBUG_RAL_STOP_OPRTN                           (1U)

#else
/* Nothing to do */
#endif /* (USE_RT_DEBUG_CONFIGURATION_COEX == 1U) */

#endif /* CFG_RT_DEBUG_GPIO_MODULE */

/******************************************************************/
/** Association table between general debug signal and used gpio **/
/******************************************************************/

#include "debug_signals.h"

#if(CFG_RT_DEBUG_GPIO_MODULE == 1)

#include "stm32wbaxx_hal.h"

typedef struct {
  GPIO_TypeDef* GPIO_port;
  uint16_t GPIO_pin;
} st_gpio_debug_t;

static const st_gpio_debug_t general_debug_table[] = {
#if (USE_RT_DEBUG_SCM_SYSTEM_CLOCK_CONFIG == 1)
  [RT_DEBUG_SCM_SYSTEM_CLOCK_CONFIG] = GPIO_DEBUG_SCM_SYSTEM_CLOCK_CONFIG,
#endif /* USE_RT_DEBUG_SCM_SYSTEM_CLOCK_CONFIG */

#if (USE_RT_DEBUG_SCM_SETUP == 1)
  [RT_DEBUG_SCM_SETUP] = GPIO_DEBUG_SCM_SETUP,
#endif /* USE_RT_DEBUG_SCM_SETUP */

#if (USE_RT_DEBUG_SCM_HSERDY_ISR == 1)
  [RT_DEBUG_SCM_HSERDY_ISR] = GPIO_DEBUG_SCM_HSERDY_ISR,
#endif /* USE_RT_DEBUG_SCM_HSERDY_ISR */

#if (USE_RT_DEBUG_ADC_ACTIVATION == 1)
  [RT_DEBUG_ADC_ACTIVATION] = GPIO_DEBUG_ADC_ACTIVATION,
#endif /* USE_RT_DEBUG_ADC_ACTIVATION */

#if (USE_RT_DEBUG_ADC_DEACTIVATION == 1)
  [RT_DEBUG_ADC_DEACTIVATION] = GPIO_DEBUG_ADC_DEACTIVATION,
#endif /* USE_RT_DEBUG_ADC_DEACTIVATION */

#if (USE_RT_DEBUG_ADC_TEMPERATURE_ACQUISITION == 1)
  [RT_DEBUG_ADC_TEMPERATURE_ACQUISITION] = GPIO_DEBUG_ADC_TEMPERATURE_ACQUISITION,
#endif /* USE_RT_DEBUG_ADC_TEMPERATURE_ACQUISITION */

#if (USE_RT_DEBUG_RNG_ENABLE == 1)
  [RT_DEBUG_RNG_ENABLE] = GPIO_DEBUG_RNG_ENABLE,
#endif /* USE_RT_DEBUG_RNG_ENABLE */

#if (USE_RT_DEBUG_RNG_DISABLE == 1)
  [RT_DEBUG_RNG_DISABLE] = GPIO_DEBUG_RNG_DISABLE,
#endif /* USE_RT_DEBUG_RNG_DISABLE */

#if (USE_RT_DEBUG_RNG_GEN_RAND_NUM == 1)
  [RT_DEBUG_RNG_GEN_RAND_NUM] = GPIO_DEBUG_RNG_GEN_RAND_NUM,
#endif /* USE_RT_DEBUG_RNG_GEN_RAND_NUM */

#if (USE_RT_DEBUG_LOW_POWER_STOP_MODE_ENTER == 1)
  [RT_DEBUG_LOW_POWER_STOP_MODE_ENTER] = GPIO_DEBUG_LOW_POWER_STOP_MODE_ENTER,
#endif /* USE_RT_DEBUG_LOW_POWER_STOP_MODE_ENTER */

#if (USE_RT_DEBUG_LOW_POWER_STOP_MODE_EXIT == 1)
  [RT_DEBUG_LOW_POWER_STOP_MODE_EXIT] = GPIO_DEBUG_LOW_POWER_STOP_MODE_EXIT,
#endif /* USE_RT_DEBUG_LOW_POWER_STOP_MODE_EXIT */

#if (USE_RT_DEBUG_LOW_POWER_STOP_MODE_ACTIVE == 1)
  [RT_DEBUG_LOW_POWER_STOP_MODE_ACTIVE] = GPIO_DEBUG_LOW_POWER_STOP_MODE_ACTIVE,
#endif /* USE_RT_DEBUG_LOW_POWER_STOP_MODE_ACTIVE */

#if (USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ENTER == 1)
  [RT_DEBUG_LOW_POWER_STANDBY_MODE_ENTER] = GPIO_DEBUG_LOW_POWER_STANDBY_MODE_ENTER,
#endif /* USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ENTER */

#if (USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_EXIT == 1)
  [RT_DEBUG_LOW_POWER_STANDBY_MODE_EXIT] = GPIO_DEBUG_LOW_POWER_STANDBY_MODE_EXIT,
#endif /* USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_EXIT */

#if (USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE == 1)
  [RT_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE] = GPIO_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE,
#endif /* USE_RT_DEBUG_LOW_POWER_STANDBY_MODE_ACTIVE */

#if (USE_RT_DEBUG_HCI_READ_DONE == 1)
  [RT_DEBUG_HCI_READ_DONE] = GPIO_DEBUG_HCI_READ_DONE,
#endif /* USE_RT_DEBUG_HCI_READ_DONE */

#if (USE_RT_DEBUG_HCI_RCVD_CMD == 1)
  [RT_DEBUG_HCI_RCVD_CMD] = GPIO_DEBUG_HCI_RCVD_CMD,
#endif /* USE_RT_DEBUG_HCI_RCVD_CMD */

#if (USE_RT_DEBUG_HCI_WRITE_DONE == 1)
  [RT_DEBUG_HCI_WRITE_DONE] = GPIO_DEBUG_HCI_WRITE_DONE,
#endif /* USE_RT_DEBUG_HCI_WRITE_DONE */

#if (USE_RT_DEBUG_SCHDLR_EVNT_UPDATE == 1)
  [RT_DEBUG_SCHDLR_EVNT_UPDATE] = GPIO_DEBUG_SCHDLR_EVNT_UPDATE,
#endif /* USE_RT_DEBUG_SCHDLR_EVNT_UPDATE */

#if (USE_RT_DEBUG_SCHDLR_TIMER_SET == 1)
  [RT_DEBUG_SCHDLR_TIMER_SET] = GPIO_DEBUG_SCHDLR_TIMER_SET,
#endif /* USE_RT_DEBUG_SCHDLR_TIMER_SET */

#if (USE_RT_DEBUG_SCHDLR_PHY_CLBR_TIMER == 1)
  [RT_DEBUG_SCHDLR_PHY_CLBR_TIMER] = GPIO_DEBUG_SCHDLR_PHY_CLBR_TIMER,
#endif /* USE_RT_DEBUG_SCHDLR_PHY_CLBR_TIMER */

#if (USE_RT_DEBUG_SCHDLR_EVNT_SKIPPED == 1)
  [RT_DEBUG_SCHDLR_EVNT_SKIPPED] = GPIO_DEBUG_SCHDLR_EVNT_SKIPPED,
#endif /* USE_RT_DEBUG_SCHDLR_EVNT_SKIPPED */

#if (USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE == 1)
  [RT_DEBUG_SCHDLR_HNDL_NXT_TRACE] = GPIO_DEBUG_SCHDLR_HNDL_NXT_TRACE,
#endif /* USE_RT_DEBUG_SCHDLR_HNDL_NXT_TRACE */

#if (USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_DETEDTED == 1)
  [RT_DEBUG_ACTIVE_SCHDLR_NEAR_DETEDTED] = GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_DETEDTED,
#endif /* USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_DETEDTED */

#if (USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_GAP_CHECK == 1)
  [RT_DEBUG_ACTIVE_SCHDLR_NEAR_GAP_CHECK] = GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_GAP_CHECK,
#endif /* USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_GAP_CHECK */

#if (USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_TIME_CHECK == 1)
  [RT_DEBUG_ACTIVE_SCHDLR_NEAR_TIME_CHECK] = GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_TIME_CHECK,
#endif /* USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_TIME_CHECK */

#if (USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_TRACE == 1)
  [RT_DEBUG_ACTIVE_SCHDLR_NEAR_TRACE] = GPIO_DEBUG_ACTIVE_SCHDLR_NEAR_TRACE,
#endif /* USE_RT_DEBUG_ACTIVE_SCHDLR_NEAR_TRACE */

#if (USE_RT_DEBUG_SCHDLR_EVNT_RGSTR == 1)
  [RT_DEBUG_SCHDLR_EVNT_RGSTR] = GPIO_DEBUG_SCHDLR_EVNT_RGSTR,
#endif /* USE_RT_DEBUG_SCHDLR_EVNT_RGSTR */

#if (USE_RT_DEBUG_SCHDLR_ADD_CONFLICT_Q == 1)
  [RT_DEBUG_SCHDLR_ADD_CONFLICT_Q] = GPIO_DEBUG_SCHDLR_ADD_CONFLICT_Q,
#endif /* USE_RT_DEBUG_SCHDLR_ADD_CONFLICT_Q */

#if (USE_RT_DEBUG_SCHDLR_HNDL_MISSED_EVNT == 1)
  [RT_DEBUG_SCHDLR_HNDL_MISSED_EVNT] = GPIO_DEBUG_SCHDLR_HNDL_MISSED_EVNT,
#endif /* USE_RT_DEBUG_SCHDLR_HNDL_MISSED_EVNT */

#if (USE_RT_DEBUG_SCHDLR_UNRGSTR_EVNT == 1)
  [RT_DEBUG_SCHDLR_UNRGSTR_EVNT] = GPIO_DEBUG_SCHDLR_UNRGSTR_EVNT,
#endif /* USE_RT_DEBUG_SCHDLR_UNRGSTR_EVNT */

#if (USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE == 1)
  [RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE] = GPIO_DEBUG_SCHDLR_EXEC_EVNT_TRACE,
#endif /* USE_RT_DEBUG_SCHDLR_EXEC_EVNT_TRACE */

#if (USE_RT_DEBUG_SCHDLR_EXEC_EVNT_PROFILE == 1)
  [RT_DEBUG_SCHDLR_EXEC_EVNT_PROFILE] = GPIO_DEBUG_SCHDLR_EXEC_EVNT_PROFILE,
#endif /* USE_RT_DEBUG_SCHDLR_EXEC_EVNT_PROFILE */

#if (USE_RT_DEBUG_SCHDLR_EXEC_EVNT_ERROR == 1)
  [RT_DEBUG_SCHDLR_EXEC_EVNT_ERROR] = GPIO_DEBUG_SCHDLR_EXEC_EVNT_ERROR,
#endif /* USE_RT_DEBUG_SCHDLR_EXEC_EVNT_ERROR */

#if (USE_RT_DEBUG_SCHDLR_EXEC_EVNT_WINDOW_WIDENING == 1)
  [RT_DEBUG_SCHDLR_EXEC_EVNT_WINDOW_WIDENING] = GPIO_DEBUG_SCHDLR_EXEC_EVNT_WINDOW_WIDENING,
#endif /* USE_RT_DEBUG_SCHDLR_EXEC_EVNT_WINDOW_WIDENING */

#if (USE_RT_DEBUG_LLHWC_CMN_CLR_ISR == 1)
  [RT_DEBUG_LLHWC_CMN_CLR_ISR] = GPIO_DEBUG_LLHWC_CMN_CLR_ISR,
#endif /* USE_RT_DEBUG_LLHWC_CMN_CLR_ISR */

#if (USE_RT_DEBUG_LLWCC_CMN_HG_ISR == 1)
  [RT_DEBUG_LLWCC_CMN_HG_ISR] = GPIO_DEBUG_LLWCC_CMN_HG_ISR,
#endif /* USE_RT_DEBUG_LLWCC_CMN_HG_ISR */

#if (USE_RT_DEBUG_LLHWC_CMN_LW_ISR == 1)
  [RT_DEBUG_LLHWC_CMN_LW_ISR] = GPIO_DEBUG_LLHWC_CMN_LW_ISR,
#endif /* USE_RT_DEBUG_LLHWC_CMN_LW_ISR */

#if (USE_RT_DEBUG_LLHWC_CMN_CLR_TIMER_ERROR == 1)
  [RT_DEBUG_LLHWC_CMN_CLR_TIMER_ERROR] = GPIO_DEBUG_LLHWC_CMN_CLR_TIMER_ERROR,
#endif /* USE_RT_DEBUG_LLHWC_CMN_CLR_TIMER_ERROR */

#if (USE_RT_DEBUG_LLHWC_LL_ISR == 1)
  [RT_DEBUG_LLHWC_LL_ISR] = GPIO_DEBUG_LLHWC_LL_ISR,
#endif /* USE_RT_DEBUG_LLHWC_LL_ISR */

#if (USE_RT_DEBUG_LLHWC_SPLTMR_SET == 1)
  [RT_DEBUG_LLHWC_SPLTMR_SET] = GPIO_DEBUG_LLHWC_SPLTMR_SET,
#endif /* USE_RT_DEBUG_LLHWC_SPLTMR_SET */

#if (USE_RT_DEBUG_LLHWC_SPLTMR_GET == 1)
  [RT_DEBUG_LLHWC_SPLTMR_GET] = GPIO_DEBUG_LLHWC_SPLTMR_GET,
#endif /* USE_RT_DEBUG_LLHWC_SPLTMR_GET */

#if (USE_RT_DEBUG_LLHWC_LOW_ISR == 1)
  [RT_DEBUG_LLHWC_LOW_ISR] = GPIO_DEBUG_LLHWC_LOW_ISR,
#endif /* USE_RT_DEBUG_LLHWC_LOW_ISR */

#if (USE_RT_DEBUG_LLHWC_STOP_SCN == 1)
  [RT_DEBUG_LLHWC_STOP_SCN] = GPIO_DEBUG_LLHWC_STOP_SCN,
#endif /* USE_RT_DEBUG_LLHWC_STOP_SCN */

#if (USE_RT_DEBUG_LLHWC_WAIT_ENVT_ON_AIR == 1)
  [RT_DEBUG_LLHWC_WAIT_ENVT_ON_AIR] = GPIO_DEBUG_LLHWC_WAIT_ENVT_ON_AIR,
#endif /* USE_RT_DEBUG_LLHWC_WAIT_ENVT_ON_AIR */

#if (USE_RT_DEBUG_LLHWC_SET_CONN_EVNT_PARAM == 1)
  [RT_DEBUG_LLHWC_SET_CONN_EVNT_PARAM] = GPIO_DEBUG_LLHWC_SET_CONN_EVNT_PARAM,
#endif /* USE_RT_DEBUG_LLHWC_SET_CONN_EVNT_PARAM */

#if (USE_RT_DEBUG_POST_EVNT == 1)
  [RT_DEBUG_POST_EVNT] = GPIO_DEBUG_POST_EVNT,
#endif /* USE_RT_DEBUG_POST_EVNT */

#if (USE_RT_DEBUG_HNDL_ALL_EVNTS == 1)
  [RT_DEBUG_HNDL_ALL_EVNTS] = GPIO_DEBUG_HNDL_ALL_EVNTS,
#endif /* USE_RT_DEBUG_HNDL_ALL_EVNTS */

#if (USE_RT_DEBUG_PROCESS_EVNT == 1)
  [RT_DEBUG_PROCESS_EVNT] = GPIO_DEBUG_PROCESS_EVNT,
#endif /* USE_RT_DEBUG_PROCESS_EVNT */

#if (USE_RT_DEBUG_PROCESS_ISO_DATA == 1)
  [RT_DEBUG_PROCESS_ISO_DATA] = GPIO_DEBUG_PROCESS_ISO_DATA,
#endif /* USE_RT_DEBUG_PROCESS_ISO_DATA */

#if (USE_RT_DEBUG_ALLOC_TX_ISO_EMPTY_PKT == 1)
  [RT_DEBUG_ALLOC_TX_ISO_EMPTY_PKT] = GPIO_DEBUG_ALLOC_TX_ISO_EMPTY_PKT,
#endif /* USE_RT_DEBUG_ALLOC_TX_ISO_EMPTY_PKT */

#if (USE_RT_DEBUG_BIG_FREE_EMPTY_PKTS == 1)
  [RT_DEBUG_BIG_FREE_EMPTY_PKTS] = GPIO_DEBUG_BIG_FREE_EMPTY_PKTS,
#endif /* USE_RT_DEBUG_BIG_FREE_EMPTY_PKTS */

#if (USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_OK == 1)
  [RT_DEBUG_RECOMBINE_UNFRMD_DATA_OK] = GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_OK,
#endif /* USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_OK */

#if (USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_CRC == 1)
  [RT_DEBUG_RECOMBINE_UNFRMD_DATA_CRC] = GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_CRC,
#endif /* USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_CRC */

#if (USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_NoRX == 1)
  [RT_DEBUG_RECOMBINE_UNFRMD_DATA_NoRX] = GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_NoRX,
#endif /* USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_NoRX */

#if (USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_TRACE == 1)
  [RT_DEBUG_RECOMBINE_UNFRMD_DATA_TRACE] = GPIO_DEBUG_RECOMBINE_UNFRMD_DATA_TRACE,
#endif /* USE_RT_DEBUG_RECOMBINE_UNFRMD_DATA_TRACE */

#if (USE_RT_DEBUG_ISO_HNDL_SDU == 1)
  [RT_DEBUG_ISO_HNDL_SDU] = GPIO_DEBUG_ISO_HNDL_SDU,
#endif /* USE_RT_DEBUG_ISO_HNDL_SDU */

#if (USE_RT_DEBUG_LL_INTF_INIT == 1)
  [RT_DEBUG_LL_INTF_INIT] = GPIO_DEBUG_LL_INTF_INIT,
#endif /* USE_RT_DEBUG_LL_INTF_INIT */

#if (USE_RT_DEBUG_DATA_TO_CNTRLR == 1)
  [RT_DEBUG_DATA_TO_CNTRLR] = GPIO_DEBUG_DATA_TO_CNTRLR,
#endif /* USE_RT_DEBUG_DATA_TO_CNTRLR */

#if (USE_RT_DEBUG_FREE_LL_PKT_HNDLR == 1)
  [RT_DEBUG_FREE_LL_PKT_HNDLR] = GPIO_DEBUG_FREE_LL_PKT_HNDLR,
#endif /* USE_RT_DEBUG_FREE_LL_PKT_HNDLR */

#if (USE_RT_DEBUG_PHY_INIT_CLBR_TRACE == 1)
  [RT_DEBUG_PHY_INIT_CLBR_TRACE] = GPIO_DEBUG_PHY_INIT_CLBR_TRACE,
#endif /* USE_RT_DEBUG_PHY_INIT_CLBR_TRACE */

#if (USE_RT_DEBUG_PHY_RUNTIME_CLBR_TRACE == 1)
  [RT_DEBUG_PHY_RUNTIME_CLBR_TRACE] = GPIO_DEBUG_PHY_RUNTIME_CLBR_TRACE,
#endif /* USE_RT_DEBUG_PHY_RUNTIME_CLBR_TRACE */

#if (USE_RT_DEBUG_PHY_CLBR_ISR == 1)
  [RT_DEBUG_PHY_CLBR_ISR] = GPIO_DEBUG_PHY_CLBR_ISR,
#endif /* USE_RT_DEBUG_PHY_CLBR_ISR */

#if (USE_RT_DEBUG_PHY_INIT_CLBR_SINGLE_CH == 1)
  [RT_DEBUG_PHY_INIT_CLBR_SINGLE_CH] = GPIO_DEBUG_PHY_INIT_CLBR_SINGLE_CH,
#endif /* USE_RT_DEBUG_PHY_INIT_CLBR_SINGLE_CH */

#if (USE_RT_DEBUG_PHY_CLBR_STRTD == 1)
  [RT_DEBUG_PHY_CLBR_STRTD] = GPIO_DEBUG_PHY_CLBR_STRTD,
#endif /* USE_RT_DEBUG_PHY_CLBR_STRTD */

#if (USE_RT_DEBUG_PHY_CLBR_EXEC == 1)
  [RT_DEBUG_PHY_CLBR_EXEC] = GPIO_DEBUG_PHY_CLBR_EXEC,
#endif /* USE_RT_DEBUG_PHY_CLBR_EXEC */

#if (USE_RT_DEBUG_RCO_STRT_STOP_RUNTIME_CLBR_ACTV == 1)
  [RT_DEBUG_RCO_STRT_STOP_RUNTIME_CLBR_ACTV] = GPIO_DEBUG_RCO_STRT_STOP_RUNTIME_CLBR_ACTV,
#endif /* USE_RT_DEBUG_RCO_STRT_STOP_RUNTIME_CLBR_ACTV */

#if (USE_RT_DEBUG_RCO_STRT_STOP_RUNTIME_RCO_CLBR == 1)
  [RT_DEBUG_RCO_STRT_STOP_RUNTIME_RCO_CLBR] = GPIO_DEBUG_RCO_STRT_STOP_RUNTIME_RCO_CLBR,
#endif /* USE_RT_DEBUG_RCO_STRT_STOP_RUNTIME_RCO_CLBR */

#if (USE_RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_SWT == 1)
  [RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_SWT] = GPIO_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_SWT,
#endif /* USE_RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_SWT */

#if (USE_RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_TRACE == 1)
  [RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_TRACE] = GPIO_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_TRACE,
#endif /* USE_RT_DEBUG_STRT_STOP_RUNTIME_RCO_CLBR_TRACE */

#if (USE_RT_DEBUG_RCO_ISR_TRACE == 1)
  [RT_DEBUG_RCO_ISR_TRACE] = GPIO_DEBUG_RCO_ISR_TRACE,
#endif /* USE_RT_DEBUG_RCO_ISR_TRACE */

#if (USE_RT_DEBUG_RCO_ISR_COMPENDATE == 1)
  [RT_DEBUG_RCO_ISR_COMPENDATE] = GPIO_DEBUG_RCO_ISR_COMPENDATE,
#endif /* USE_RT_DEBUG_RCO_ISR_COMPENDATE */

#if (USE_RT_DEBUG_RAL_STRT_TX == 1)
  [RT_DEBUG_RAL_STRT_TX] = GPIO_DEBUG_RAL_STRT_TX,
#endif /* USE_RT_DEBUG_RAL_STRT_TX */

#if (USE_RT_DEBUG_RAL_ISR_TIMER_ERROR == 1)
  [RT_DEBUG_RAL_ISR_TIMER_ERROR] = GPIO_DEBUG_RAL_ISR_TIMER_ERROR,
#endif /* USE_RT_DEBUG_RAL_ISR_TIMER_ERROR */

#if (USE_RT_DEBUG_RAL_ISR_TRACE == 1)
  [RT_DEBUG_RAL_ISR_TRACE] = GPIO_DEBUG_RAL_ISR_TRACE,
#endif /* USE_RT_DEBUG_RAL_ISR_TRACE */

#if (USE_RT_DEBUG_RAL_STOP_OPRTN == 1)
  [RT_DEBUG_RAL_STOP_OPRTN] = GPIO_DEBUG_RAL_STOP_OPRTN,
#endif /* USE_RT_DEBUG_RAL_STOP_OPRTN */

#if (USE_RT_DEBUG_RAL_STRT_RX == 1)
  [RT_DEBUG_RAL_STRT_RX] = GPIO_DEBUG_RAL_STRT_RX,
#endif /* USE_RT_DEBUG_RAL_STRT_RX */

#if (USE_RT_DEBUG_RAL_DONE_CLBK_TX == 1)
  [RT_DEBUG_RAL_DONE_CLBK_TX] = GPIO_DEBUG_RAL_DONE_CLBK_TX,
#endif /* USE_RT_DEBUG_RAL_DONE_CLBK_TX */

#if (USE_RT_DEBUG_RAL_DONE_CLBK_RX == 1)
  [RT_DEBUG_RAL_DONE_CLBK_RX] = GPIO_DEBUG_RAL_DONE_CLBK_RX,
#endif /* USE_RT_DEBUG_RAL_DONE_CLBK_RX */

#if (USE_RT_DEBUG_RAL_DONE_CLBK_ED == 1)
  [RT_DEBUG_RAL_DONE_CLBK_ED] = GPIO_DEBUG_RAL_DONE_CLBK_ED,
#endif /* USE_RT_DEBUG_RAL_DONE_CLBK_ED */

#if (USE_RT_DEBUG_RAL_ED_SCAN == 1)
  [RT_DEBUG_RAL_ED_SCAN] = GPIO_DEBUG_RAL_ED_SCAN,
#endif /* USE_RT_DEBUG_RAL_ED_SCAN */

#if (USE_RT_DEBUG_ERROR_MEM_CAP_EXCED == 1)
  [RT_DEBUG_ERROR_MEM_CAP_EXCED] = GPIO_DEBUG_ERROR_MEM_CAP_EXCED,
#endif /* USE_RT_DEBUG_ERROR_MEM_CAP_EXCED */

#if (USE_RT_DEBUG_ERROR_COMMAND_DISALLOWED == 1)
  [RT_DEBUG_ERROR_COMMAND_DISALLOWED] = GPIO_DEBUG_ERROR_COMMAND_DISALLOWED,
#endif /* USE_RT_DEBUG_ERROR_COMMAND_DISALLOWED */

#if (USE_RT_DEBUG_PTA_INIT == 1)
  [RT_DEBUG_PTA_INIT] = GPIO_DEBUG_PTA_INIT,
#endif /* USE_RT_DEBUG_PTA_INIT */

#if (USE_RT_DEBUG_PTA_EN == 1)
  [RT_DEBUG_PTA_EN] = GPIO_DEBUG_PTA_EN,
#endif /* USE_RT_DEBUG_PTA_EN */

#if (USE_RT_DEBUG_LLHWC_PTA_SET_EN == 1)
  [RT_DEBUG_LLHWC_PTA_SET_EN] = GPIO_DEBUG_LLHWC_PTA_SET_EN,
#endif /* USE_RT_DEBUG_LLHWC_PTA_SET_EN */

#if (USE_RT_DEBUG_LLHWC_PTA_SET_PARAMS == 1)
  [RT_DEBUG_LLHWC_PTA_SET_PARAMS] = GPIO_DEBUG_LLHWC_PTA_SET_PARAMS,
#endif /* USE_RT_DEBUG_LLHWC_PTA_SET_PARAMS */

#if (USE_RT_DEBUG_COEX_STRT_ON_IDLE == 1)
  [RT_DEBUG_COEX_STRT_ON_IDLE] = GPIO_DEBUG_COEX_STRT_ON_IDLE,
#endif /* USE_RT_DEBUG_COEX_STRT_ON_IDLE */

#if (USE_RT_DEBUG_COEX_ASK_FOR_AIR == 1)
  [RT_DEBUG_COEX_ASK_FOR_AIR] = GPIO_DEBUG_COEX_ASK_FOR_AIR,
#endif /* USE_RT_DEBUG_COEX_ASK_FOR_AIR */

#if (USE_RT_DEBUG_COEX_TIMER_EVNT_CLBK == 1)
  [RT_DEBUG_COEX_TIMER_EVNT_CLBK] = GPIO_DEBUG_COEX_TIMER_EVNT_CLBK,
#endif /* USE_RT_DEBUG_COEX_TIMER_EVNT_CLBK */

#if (USE_RT_DEBUG_COEX_STRT_ONE_SHOT == 1)
  [RT_DEBUG_COEX_STRT_ONE_SHOT] = GPIO_DEBUG_COEX_STRT_ONE_SHOT,
#endif /* USE_RT_DEBUG_COEX_STRT_ONE_SHOT */

#if (USE_RT_DEBUG_COEX_FORCE_STOP_RX == 1)
  [RT_DEBUG_COEX_FORCE_STOP_RX] = GPIO_DEBUG_COEX_FORCE_STOP_RX,
#endif /* USE_RT_DEBUG_COEX_FORCE_STOP_RX */

#if (USE_RT_DEBUG_LLHWC_ADV_DONE == 1)
  [RT_DEBUG_LLHWC_ADV_DONE] = GPIO_DEBUG_LLHWC_ADV_DONE,
#endif /* USE_RT_DEBUG_LLHWC_ADV_DONE */

#if (USE_RT_DEBUG_LLHWC_SCN_DONE == 1)
  [RT_DEBUG_LLHWC_SCN_DONE] = GPIO_DEBUG_LLHWC_SCN_DONE,
#endif /* USE_RT_DEBUG_LLHWC_SCN_DONE */

#if (USE_RT_DEBUG_LLHWC_INIT_DONE == 1)
  [RT_DEBUG_LLHWC_INIT_DONE] = GPIO_DEBUG_LLHWC_INIT_DONE,
#endif /* USE_RT_DEBUG_LLHWC_INIT_DONE */

#if (USE_RT_DEBUG_LLHWC_CONN_DONE == 1)
  [RT_DEBUG_LLHWC_CONN_DONE] = GPIO_DEBUG_LLHWC_CONN_DONE,
#endif /* USE_RT_DEBUG_LLHWC_CONN_DONE */

#if (USE_RT_DEBUG_LLHWC_CIG_DONE == 1)
  [RT_DEBUG_LLHWC_CIG_DONE] = GPIO_DEBUG_LLHWC_CIG_DONE,
#endif /* USE_RT_DEBUG_LLHWC_CIG_DONE */

#if (USE_RT_DEBUG_LLHWC_BIG_DONE == 1)
  [RT_DEBUG_LLHWC_BIG_DONE] = GPIO_DEBUG_LLHWC_BIG_DONE,
#endif /* USE_RT_DEBUG_LLHWC_BIG_DONE */

#if (USE_RT_DEBUG_OS_TMR_CREATE == 1)
  [RT_DEBUG_OS_TMR_CREATE] = GPIO_DEBUG_OS_TMR_CREATE,
#endif /* USE_RT_DEBUG_OS_TMR_CREATE */

#if (USE_RT_DEBUG_ADV_EXT_TIMEOUT_CBK == 1)
  [RT_DEBUG_ADV_EXT_TIMEOUT_CBK] = GPIO_DEBUG_ADV_EXT_TIMEOUT_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_TIMEOUT_CBK */

#if (USE_RT_DEBUG_ADV_EXT_SCN_DUR_CBK == 1)
  [RT_DEBUG_ADV_EXT_SCN_DUR_CBK] = GPIO_DEBUG_ADV_EXT_SCN_DUR_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_SCN_DUR_CBK */

#if (USE_RT_DEBUG_ADV_EXT_SCN_PERIOD_CBK == 1)
  [RT_DEBUG_ADV_EXT_SCN_PERIOD_CBK] = GPIO_DEBUG_ADV_EXT_SCN_PERIOD_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_SCN_PERIOD_CBK */

#if (USE_RT_DEBUG_ADV_EXT_PRDC_SCN_TIMEOUT_CBK == 1)
  [RT_DEBUG_ADV_EXT_PRDC_SCN_TIMEOUT_CBK] = GPIO_DEBUG_ADV_EXT_PRDC_SCN_TIMEOUT_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_PRDC_SCN_TIMEOUT_CBK */

#if (USE_RT_DEBUG_BIS_SYNC_TIMEOUT_TMR_CBK == 1)
  [RT_DEBUG_BIS_SYNC_TIMEOUT_TMR_CBK] = GPIO_DEBUG_BIS_SYNC_TIMEOUT_TMR_CBK,
#endif /* USE_RT_DEBUG_BIS_SYNC_TIMEOUT_TMR_CBK */

#if (USE_RT_DEBUG_BIS_TERM_TMR_CBK == 1)
  [RT_DEBUG_BIS_TERM_TMR_CBK] = GPIO_DEBUG_BIS_TERM_TMR_CBK,
#endif /* USE_RT_DEBUG_BIS_TERM_TMR_CBK */

#if (USE_RT_DEBUG_BIS_TST_MODE_CBK == 1)
  [RT_DEBUG_BIS_TST_MODE_CBK] = GPIO_DEBUG_BIS_TST_MODE_CBK,
#endif /* USE_RT_DEBUG_BIS_TST_MODE_CBK */

#if (USE_RT_DEBUG_BIS_TST_MODE_TMR_CBK == 1)
  [RT_DEBUG_BIS_TST_MODE_TMR_CBK] = GPIO_DEBUG_BIS_TST_MODE_TMR_CBK,
#endif /* USE_RT_DEBUG_BIS_TST_MODE_TMR_CBK */

#if (USE_RT_DEBUG_ISO_POST_TMR_CBK == 1)
  [RT_DEBUG_ISO_POST_TMR_CBK] = GPIO_DEBUG_ISO_POST_TMR_CBK,
#endif /* USE_RT_DEBUG_ISO_POST_TMR_CBK */

#if (USE_RT_DEBUG_ISO_TST_MODE_TMR_CBK == 1)
  [RT_DEBUG_ISO_TST_MODE_TMR_CBK] = GPIO_DEBUG_ISO_TST_MODE_TMR_CBK,
#endif /* USE_RT_DEBUG_ISO_TST_MODE_TMR_CBK */

#if (USE_RT_DEBUG_CONN_POST_TMR_CBK == 1)
  [RT_DEBUG_CONN_POST_TMR_CBK] = GPIO_DEBUG_CONN_POST_TMR_CBK,
#endif /* USE_RT_DEBUG_CONN_POST_TMR_CBK */

#if (USE_RT_DEBUG_EVNT_SCHDLR_TMR_CBK == 1)
  [RT_DEBUG_EVNT_SCHDLR_TMR_CBK] = GPIO_DEBUG_EVNT_SCHDLR_TMR_CBK,
#endif /* USE_RT_DEBUG_EVNT_SCHDLR_TMR_CBK */

#if (USE_RT_DEBUG_HCI_POST_TMR_CBK == 1)
  [RT_DEBUG_HCI_POST_TMR_CBK] = GPIO_DEBUG_HCI_POST_TMR_CBK,
#endif /* USE_RT_DEBUG_HCI_POST_TMR_CBK */

#if (USE_RT_DEBUG_LLCP_POST_TMR_CBK == 1)
  [RT_DEBUG_LLCP_POST_TMR_CBK] = GPIO_DEBUG_LLCP_POST_TMR_CBK,
#endif /* USE_RT_DEBUG_LLCP_POST_TMR_CBK */

#if (USE_RT_DEBUG_LLHWC_ENRGY_DETECT_CBK == 1)
  [RT_DEBUG_LLHWC_ENRGY_DETECT_CBK] = GPIO_DEBUG_LLHWC_ENRGY_DETECT_CBK,
#endif /* USE_RT_DEBUG_LLHWC_ENRGY_DETECT_CBK */

#if (USE_RT_DEBUG_PRVCY_POST_TMR_CBK == 1)
  [RT_DEBUG_PRVCY_POST_TMR_CBK] = GPIO_DEBUG_PRVCY_POST_TMR_CBK,
#endif /* USE_RT_DEBUG_PRVCY_POST_TMR_CBK */

#if (USE_RT_DEBUG_ANT_PRPR_TMR_CBK == 1)
  [RT_DEBUG_ANT_PRPR_TMR_CBK] = GPIO_DEBUG_ANT_PRPR_TMR_CBK,
#endif /* USE_RT_DEBUG_ANT_PRPR_TMR_CBK */

#if (USE_RT_DEBUG_COEX_TMR_FRC_STOP_AIR_GRANT_CBK == 1)
  [RT_DEBUG_COEX_TMR_FRC_STOP_AIR_GRANT_CBK] = GPIO_DEBUG_COEX_TMR_FRC_STOP_AIR_GRANT_CBK,
#endif /* USE_RT_DEBUG_COEX_TMR_FRC_STOP_AIR_GRANT_CBK */

#if (USE_RT_DEBUG_MLME_RX_EN_TMR_CBK == 1)
  [RT_DEBUG_MLME_RX_EN_TMR_CBK] = GPIO_DEBUG_MLME_RX_EN_TMR_CBK,
#endif /* USE_RT_DEBUG_MLME_RX_EN_TMR_CBK */

#if (USE_RT_DEBUG_MLME_GNRC_TMR_CBK == 1)
  [RT_DEBUG_MLME_GNRC_TMR_CBK] = GPIO_DEBUG_MLME_GNRC_TMR_CBK,
#endif /* USE_RT_DEBUG_MLME_GNRC_TMR_CBK */

#if (USE_RT_DEBUG_MIB_JOIN_LST_TMR_CBK == 1)
  [RT_DEBUG_MIB_JOIN_LST_TMR_CBK] = GPIO_DEBUG_MIB_JOIN_LST_TMR_CBK,
#endif /* USE_RT_DEBUG_MIB_JOIN_LST_TMR_CBK */

#if (USE_RT_DEBUG_MLME_PWR_PRES_TMR_CBK == 1)
  [RT_DEBUG_MLME_PWR_PRES_TMR_CBK] = GPIO_DEBUG_MLME_PWR_PRES_TMR_CBK,
#endif /* USE_RT_DEBUG_MLME_PWR_PRES_TMR_CBK */

#if (USE_RT_DEBUG_PRESISTENCE_TMR_CBK == 1)
  [RT_DEBUG_PRESISTENCE_TMR_CBK] = GPIO_DEBUG_PRESISTENCE_TMR_CBK,
#endif /* USE_RT_DEBUG_PRESISTENCE_TMR_CBK */

#if (USE_RT_DEBUG_RADIO_PHY_PRDC_CLBK_TMR_CBK == 1)
  [RT_DEBUG_RADIO_PHY_PRDC_CLBK_TMR_CBK] = GPIO_DEBUG_RADIO_PHY_PRDC_CLBK_TMR_CBK,
#endif /* USE_RT_DEBUG_RADIO_PHY_PRDC_CLBK_TMR_CBK */

#if (USE_RT_DEBUG_RADIO_CSMA_TMR_CBK == 1)
  [RT_DEBUG_RADIO_CSMA_TMR_CBK] = GPIO_DEBUG_RADIO_CSMA_TMR_CBK,
#endif /* USE_RT_DEBUG_RADIO_CSMA_TMR_CBK */

#if (USE_RT_DEBUG_RADIO_CSL_RCV_TMR_CBK == 1)
  [RT_DEBUG_RADIO_CSL_RCV_TMR_CBK] = GPIO_DEBUG_RADIO_CSL_RCV_TMR_CBK,
#endif /* USE_RT_DEBUG_RADIO_CSL_RCV_TMR_CBK */

#if (USE_RT_DEBUG_ED_TMR_CBK == 1)
  [RT_DEBUG_ED_TMR_CBK] = GPIO_DEBUG_ED_TMR_CBK,
#endif /* USE_RT_DEBUG_ED_TMR_CBK */

#if (USE_RT_DEBUG_DIO_EXT_TMR_CBK == 1)
  [RT_DEBUG_DIO_EXT_TMR_CBK] = GPIO_DEBUG_DIO_EXT_TMR_CBK,
#endif /* USE_RT_DEBUG_DIO_EXT_TMR_CBK */

#if (USE_RT_DEBUG_RCO_CLBR_TMR_CBK == 1)
  [RT_DEBUG_RCO_CLBR_TMR_CBK] = GPIO_DEBUG_RCO_CLBR_TMR_CBK,
#endif /* USE_RT_DEBUG_RCO_CLBR_TMR_CBK */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_ADV_CBK == 1)
  [RT_DEBUG_ADV_EXT_MNGR_ADV_CBK] = GPIO_DEBUG_ADV_EXT_MNGR_ADV_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_ADV_CBK */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_SCN_CBK == 1)
  [RT_DEBUG_ADV_EXT_MNGR_SCN_CBK] = GPIO_DEBUG_ADV_EXT_MNGR_SCN_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_SCN_CBK */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_SCN_ERR_CBK == 1)
  [RT_DEBUG_ADV_EXT_MNGR_SCN_ERR_CBK] = GPIO_DEBUG_ADV_EXT_MNGR_SCN_ERR_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_SCN_ERR_CBK */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CBK == 1)
  [RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CBK] = GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CBK */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_ERR_CBK == 1)
  [RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_ERR_CBK] = GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_ERR_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_ERR_CBK */

#if (USE_RT_DEBUG_BIG_ADV_CBK == 1)
  [RT_DEBUG_BIG_ADV_CBK] = GPIO_DEBUG_BIG_ADV_CBK,
#endif /* USE_RT_DEBUG_BIG_ADV_CBK */

#if (USE_RT_DEBUG_BIG_ADV_ERR_CBK == 1)
  [RT_DEBUG_BIG_ADV_ERR_CBK] = GPIO_DEBUG_BIG_ADV_ERR_CBK,
#endif /* USE_RT_DEBUG_BIG_ADV_ERR_CBK */

#if (USE_RT_DEBUG_BIG_SYNC_CBK == 1)
  [RT_DEBUG_BIG_SYNC_CBK] = GPIO_DEBUG_BIG_SYNC_CBK,
#endif /* USE_RT_DEBUG_BIG_SYNC_CBK */

#if (USE_RT_DEBUG_BIG_SYNC_ERR_CBK == 1)
  [RT_DEBUG_BIG_SYNC_ERR_CBK] = GPIO_DEBUG_BIG_SYNC_ERR_CBK,
#endif /* USE_RT_DEBUG_BIG_SYNC_ERR_CBK */

#if (USE_RT_DEBUG_ISO_CIS_PKT_TRNSM_RECEIVED_CBK == 1)
  [RT_DEBUG_ISO_CIS_PKT_TRNSM_RECEIVED_CBK] = GPIO_DEBUG_ISO_CIS_PKT_TRNSM_RECEIVED_CBK,
#endif /* USE_RT_DEBUG_ISO_CIS_PKT_TRNSM_RECEIVED_CBK */

#if (USE_RT_DEBUG_ISO_CIG_ERR_CBK == 1)
  [RT_DEBUG_ISO_CIG_ERR_CBK] = GPIO_DEBUG_ISO_CIG_ERR_CBK,
#endif /* USE_RT_DEBUG_ISO_CIG_ERR_CBK */

#if (USE_RT_DEBUG_CONN_PKT_TRNSM_RECEIVED_CBK == 1)
  [RT_DEBUG_CONN_PKT_TRNSM_RECEIVED_CBK] = GPIO_DEBUG_CONN_PKT_TRNSM_RECEIVED_CBK,
#endif /* USE_RT_DEBUG_CONN_PKT_TRNSM_RECEIVED_CBK */

#if (USE_RT_DEBUG_PRDC_CLBR_EXTRL_CBK == 1)
  [RT_DEBUG_PRDC_CLBR_EXTRL_CBK] = GPIO_DEBUG_PRDC_CLBR_EXTRL_CBK,
#endif /* USE_RT_DEBUG_PRDC_CLBR_EXTRL_CBK */

#if (USE_RT_DEBUG_PTR_PRDC_ADV_SYNC_CBK == 1)
  [RT_DEBUG_PTR_PRDC_ADV_SYNC_CBK] = GPIO_DEBUG_PTR_PRDC_ADV_SYNC_CBK,
#endif /* USE_RT_DEBUG_PTR_PRDC_ADV_SYNC_CBK */

#if (USE_RT_DEBUG_NCONN_SCN_CBK == 1)
  [RT_DEBUG_NCONN_SCN_CBK] = GPIO_DEBUG_NCONN_SCN_CBK,
#endif /* USE_RT_DEBUG_NCONN_SCN_CBK */

#if (USE_RT_DEBUG_NCONN_ADV_CBK == 1)
  [RT_DEBUG_NCONN_ADV_CBK] = GPIO_DEBUG_NCONN_ADV_CBK,
#endif /* USE_RT_DEBUG_NCONN_ADV_CBK */

#if (USE_RT_DEBUG_NCONN_INIT_CBK == 1)
  [RT_DEBUG_NCONN_INIT_CBK] = GPIO_DEBUG_NCONN_INIT_CBK,
#endif /* USE_RT_DEBUG_NCONN_INIT_CBK */

#if (USE_RT_DEBUG_ANT_RADIO_CMPLT_EVNT_CBK == 1)
  [RT_DEBUG_ANT_RADIO_CMPLT_EVNT_CBK] = GPIO_DEBUG_ANT_RADIO_CMPLT_EVNT_CBK,
#endif /* USE_RT_DEBUG_ANT_RADIO_CMPLT_EVNT_CBK */

#if (USE_RT_DEBUG_ANT_STACK_EVNT_CBK == 1)
  [RT_DEBUG_ANT_STACK_EVNT_CBK] = GPIO_DEBUG_ANT_STACK_EVNT_CBK,
#endif /* USE_RT_DEBUG_ANT_STACK_EVNT_CBK */

#if (USE_RT_DEBUG_ADV_EXT_PROCESS_TMOUT_EVNT_CBK == 1)
  [RT_DEBUG_ADV_EXT_PROCESS_TMOUT_EVNT_CBK] = GPIO_DEBUG_ADV_EXT_PROCESS_TMOUT_EVNT_CBK,
#endif /* USE_RT_DEBUG_ADV_EXT_PROCESS_TMOUT_EVNT_CBK */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_SCN_DUR_EVNT == 1)
  [RT_DEBUG_ADV_EXT_MNGR_SCN_DUR_EVNT] = GPIO_DEBUG_ADV_EXT_MNGR_SCN_DUR_EVNT,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_SCN_DUR_EVNT */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_SCN_PERIODIC_EVNT == 1)
  [RT_DEBUG_ADV_EXT_MNGR_SCN_PERIODIC_EVNT] = GPIO_DEBUG_ADV_EXT_MNGR_SCN_PERIODIC_EVNT,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_SCN_PERIODIC_EVNT */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_TMOUT_EVNT == 1)
  [RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_TMOUT_EVNT] = GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_TMOUT_EVNT,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_TMOUT_EVNT */

#if (USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CNCEL_EVNT == 1)
  [RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CNCEL_EVNT] = GPIO_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CNCEL_EVNT,
#endif /* USE_RT_DEBUG_ADV_EXT_MNGR_PRDC_SCN_CNCEL_EVNT */

#if (USE_RT_DEBUG_BIS_MNGR_BIG_TERM_CBK == 1)
  [RT_DEBUG_BIS_MNGR_BIG_TERM_CBK] = GPIO_DEBUG_BIS_MNGR_BIG_TERM_CBK,
#endif /* USE_RT_DEBUG_BIS_MNGR_BIG_TERM_CBK */

#if (USE_RT_DEBUG_BIS_MNGR_SYNC_TMOUT_CBK == 1)
  [RT_DEBUG_BIS_MNGR_SYNC_TMOUT_CBK] = GPIO_DEBUG_BIS_MNGR_SYNC_TMOUT_CBK,
#endif /* USE_RT_DEBUG_BIS_MNGR_SYNC_TMOUT_CBK */

#if (USE_RT_DEBUG_ISOAL_MNGR_SDU_GEN == 1)
  [RT_DEBUG_ISOAL_MNGR_SDU_GEN] = GPIO_DEBUG_ISOAL_MNGR_SDU_GEN,
#endif /* USE_RT_DEBUG_ISOAL_MNGR_SDU_GEN */

#if (USE_RT_DEBUG_ISO_MNGR_CIS_PROCESS_EVNT_CBK == 1)
  [RT_DEBUG_ISO_MNGR_CIS_PROCESS_EVNT_CBK] = GPIO_DEBUG_ISO_MNGR_CIS_PROCESS_EVNT_CBK,
#endif /* USE_RT_DEBUG_ISO_MNGR_CIS_PROCESS_EVNT_CBK */

#if (USE_RT_DEBUG_CONN_MNGR_PROCESS_EVNT_CLBK == 1)
  [RT_DEBUG_CONN_MNGR_PROCESS_EVNT_CLBK] = GPIO_DEBUG_CONN_MNGR_PROCESS_EVNT_CLBK,
#endif /* USE_RT_DEBUG_CONN_MNGR_PROCESS_EVNT_CLBK */

#if (USE_RT_DEBUG_CONN_MNGR_UPDT_CONN_PARAM_CBK == 1)
  [RT_DEBUG_CONN_MNGR_UPDT_CONN_PARAM_CBK] = GPIO_DEBUG_CONN_MNGR_UPDT_CONN_PARAM_CBK,
#endif /* USE_RT_DEBUG_CONN_MNGR_UPDT_CONN_PARAM_CBK */

#if (USE_RT_DEBUG_EVNT_SCHDLR_HW_EVNT_CMPLT == 1)
  [RT_DEBUG_EVNT_SCHDLR_HW_EVNT_CMPLT] = GPIO_DEBUG_EVNT_SCHDLR_HW_EVNT_CMPLT,
#endif /* USE_RT_DEBUG_EVNT_SCHDLR_HW_EVNT_CMPLT */

#if (USE_RT_DEBUG_HCI_EVENT_HNDLR == 1)
  [RT_DEBUG_HCI_EVENT_HNDLR] = GPIO_DEBUG_HCI_EVENT_HNDLR,
#endif /* USE_RT_DEBUG_HCI_EVENT_HNDLR */

#if (USE_RT_DEBUG_MLME_TMRS_CBK == 1)
  [RT_DEBUG_MLME_TMRS_CBK] = GPIO_DEBUG_MLME_TMRS_CBK,
#endif /* USE_RT_DEBUG_MLME_TMRS_CBK */

#if (USE_RT_DEBUG_DIRECT_TX_EVNT_CBK == 1)
  [RT_DEBUG_DIRECT_TX_EVNT_CBK] = GPIO_DEBUG_DIRECT_TX_EVNT_CBK,
#endif /* USE_RT_DEBUG_DIRECT_TX_EVNT_CBK */

#if (USE_RT_DEBUG_INDIRECT_PKT_TOUR_CBK == 1)
  [RT_DEBUG_INDIRECT_PKT_TOUR_CBK] = GPIO_DEBUG_INDIRECT_PKT_TOUR_CBK,
#endif /* USE_RT_DEBUG_INDIRECT_PKT_TOUR_CBK */

#if (USE_RT_DEBUG_RADIO_CSMA_TMR == 1)
  [RT_DEBUG_RADIO_CSMA_TMR] = GPIO_DEBUG_RADIO_CSMA_TMR,
#endif /* USE_RT_DEBUG_RADIO_CSMA_TMR */

#if (USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK == 1)
  [RT_DEBUG_RAL_SM_DONE_EVNT_CBK] = GPIO_DEBUG_RAL_SM_DONE_EVNT_CBK,
#endif /* USE_RT_DEBUG_RAL_SM_DONE_EVNT_CBK */

#if (USE_RT_DEBUG_ED_TMR_HNDL == 1)
  [RT_DEBUG_ED_TMR_HNDL] = GPIO_DEBUG_ED_TMR_HNDL,
#endif /* USE_RT_DEBUG_ED_TMR_HNDL */

#if (USE_RT_DEBUG_OS_TMR_EVNT_CBK == 1)
  [RT_DEBUG_OS_TMR_EVNT_CBK] = GPIO_DEBUG_OS_TMR_EVNT_CBK,
#endif /* USE_RT_DEBUG_OS_TMR_EVNT_CBK */

#if (USE_RT_DEBUG_PROFILE_MARKER_PHY_WAKEUP_TIME == 1)
  [RT_DEBUG_PROFILE_MARKER_PHY_WAKEUP_TIME] = GPIO_DEBUG_PROFILE_MARKER_PHY_WAKEUP_TIME,
#endif /* USE_RT_DEBUG_PROFILE_MARKER_PHY_WAKEUP_TIME */

#if (USE_RT_DEBUG_PROFILE_END_DRIFT_TIME == 1)
  [RT_DEBUG_PROFILE_END_DRIFT_TIME] = GPIO_DEBUG_PROFILE_END_DRIFT_TIME,
#endif /* USE_RT_DEBUG_PROFILE_END_DRIFT_TIME */

#if (USE_RT_DEBUG_PROC_RADIO_RCV == 1)
  [RT_DEBUG_PROC_RADIO_RCV] = GPIO_DEBUG_PROC_RADIO_RCV,
#endif /* USE_RT_DEBUG_PROC_RADIO_RCV */

#if (USE_RT_DEBUG_EVNT_TIME_UPDT == 1)
  [RT_DEBUG_EVNT_TIME_UPDT] = GPIO_DEBUG_EVNT_TIME_UPDT,
#endif /* USE_RT_DEBUG_EVNT_TIME_UPDT */

#if (USE_RT_DEBUG_MAC_RECEIVE_DONE == 1)
  [RT_DEBUG_MAC_RECEIVE_DONE] = GPIO_DEBUG_MAC_RECEIVE_DONE,
#endif /* USE_RT_DEBUG_MAC_RECEIVE_DONE */

#if (USE_RT_DEBUG_MAC_TX_DONE == 1)
  [RT_DEBUG_MAC_TX_DONE] = GPIO_DEBUG_MAC_TX_DONE,
#endif /* USE_RT_DEBUG_MAC_TX_DONE */

#if (USE_RT_DEBUG_RADIO_APPLY_CSMA == 1)
  [RT_DEBUG_RADIO_APPLY_CSMA] = GPIO_DEBUG_RADIO_APPLY_CSMA,
#endif /* USE_RT_DEBUG_RADIO_APPLY_CSMA */

#if (USE_RT_DEBUG_RADIO_TRANSMIT == 1)
  [RT_DEBUG_RADIO_TRANSMIT] = GPIO_DEBUG_RADIO_TRANSMIT,
#endif /* USE_RT_DEBUG_RADIO_TRANSMIT */

#if (USE_RT_DEBUG_PROC_RADIO_TX == 1)
  [RT_DEBUG_PROC_RADIO_TX] = GPIO_DEBUG_PROC_RADIO_TX,
#endif /* USE_RT_DEBUG_PROC_RADIO_TX */

#if (USE_RT_DEBUG_RAL_TX_DONE == 1)
  [RT_DEBUG_RAL_TX_DONE] = GPIO_DEBUG_RAL_TX_DONE,
#endif /* USE_RT_DEBUG_RAL_TX_DONE */

#if (USE_RT_DEBUG_RAL_TX_DONE_INCREMENT_BACKOFF_COUNT == 1)
  [RT_DEBUG_RAL_TX_DONE_INCREMENT_BACKOFF_COUNT] = GPIO_DEBUG_RAL_TX_DONE_INCREMENT_BACKOFF_COUNT,
#endif /* USE_RT_DEBUG_RAL_TX_DONE_INCREMENT_BACKOFF_COUNT */

#if (USE_RT_DEBUG_RAL_TX_DONE_RST_BACKOFF_COUNT == 1)
  [RT_DEBUG_RAL_TX_DONE_RST_BACKOFF_COUNT] = GPIO_DEBUG_RAL_TX_DONE_RST_BACKOFF_COUNT,
#endif /* USE_RT_DEBUG_RAL_TX_DONE_RST_BACKOFF_COUNT */

#if (USE_RT_DEBUG_RAL_CONTINUE_RX == 1)
  [RT_DEBUG_RAL_CONTINUE_RX] = GPIO_DEBUG_RAL_CONTINUE_RX,
#endif /* USE_RT_DEBUG_RAL_CONTINUE_RX */

#if (USE_RT_DEBUG_RAL_PERFORM_CCA == 1)
  [RT_DEBUG_RAL_PERFORM_CCA] = GPIO_DEBUG_RAL_PERFORM_CCA,
#endif /* USE_RT_DEBUG_RAL_PERFORM_CCA */

#if (USE_RT_DEBUG_RAL_ENABLE_TRANSMITTER == 1)
  [RT_DEBUG_RAL_ENABLE_TRANSMITTER] = GPIO_DEBUG_RAL_ENABLE_TRANSMITTER,
#endif /* USE_RT_DEBUG_RAL_ENABLE_TRANSMITTER */

#if (USE_RT_DEBUG_LLHWC_GET_CH_IDX_ALGO_2 == 1)
  [RT_DEBUG_LLHWC_GET_CH_IDX_ALGO_2] = GPIO_DEBUG_LLHWC_GET_CH_IDX_ALGO_2,
#endif /* USE_RT_DEBUG_LLHWC_GET_CH_IDX_ALGO_2 */

/************************************************/
/** Application signals in general debug table **/
/************************************************/

#if (USE_RT_DEBUG_APP_APPE_INIT == 1)
  [RT_DEBUG_APP_APPE_INIT] = GPIO_DEBUG_APP_APPE_INIT,
#endif /* USE_RT_DEBUG_OS_TMR_EVNT_CBK */
};

#endif /* CFG_RT_DEBUG_GPIO_MODULE */

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_CONFIG_H */

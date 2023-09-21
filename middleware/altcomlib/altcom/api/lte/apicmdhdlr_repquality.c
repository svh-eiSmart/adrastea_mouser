/****************************************************************************
 *
 *  (c) copyright 2019 Altair Semiconductor, Ltd. All rights reserved.
 *
 *  This software, in source or object form (the "Software"), is the
 *  property of Altair Semiconductor Ltd. (the "Company") and/or its
 *  licensors, which have all right, title and interest therein, You
 *  may use the Software only in  accordance with the terms of written
 *  license agreement between you and the Company (the "License").
 *  Except as expressly stated in the License, the Company grants no
 *  licenses by implication, estoppel, or otherwise. If you are not
 *  aware of or do not agree to the License terms, you may not use,
 *  copy or modify the Software. You may use the source code of the
 *  Software only for your internal purposes and may not distribute the
 *  source code of the Software, any part thereof, or any derivative work
 *  thereof, to any third party, except pursuant to the Company's prior
 *  written consent.
 *  The Software is the confidential information of the Company.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apicmd_repquality.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern quality_report_cb_t g_quality_callback;
extern void *g_quality_cbpriv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: repquality_job
 *
 * Description:
 *   This function is an API callback for quality report receive.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repquality_job(FAR void *arg) {
  FAR struct apicmd_cmddat_repquality_s *data;
  lte_quality_t *repdat = NULL;
  quality_report_cb_t callback;
  void *cbpriv;

  ALTCOM_GET_CALLBACK(g_quality_callback, callback, g_quality_cbpriv, cbpriv);
  if (!callback) {
    DBGIF_LOG_DEBUG("g_cellinfo_callback is not registered.\n");
    return;
  }

  data = (FAR struct apicmd_cmddat_repquality_s *)arg;

  repdat = (lte_quality_t *)BUFFPOOL_ALLOC(sizeof(lte_quality_t));
  if (!repdat) {
    DBGIF_LOG_DEBUG("report data buffer alloc error.\n");
  } else {
    repdat->valid = APICMD_REP_QUALITY_ENABLE == data->enability ? LTE_VALID : LTE_INVALID;
    if (repdat->valid) {
      repdat->rsrp = ntohs(data->rsrp);
      repdat->rsrq = ntohs(data->rsrq);
      repdat->sinr = ntohs(data->sinr);
      repdat->rssi = ntohs(data->rssi);
      if (repdat->rsrp < APICMD_REP_QUALITY_RSRP_MIN ||
          APICMD_REP_QUALITY_RSRP_MAX < repdat->rsrp) {
        DBGIF_LOG1_ERROR("repdat->rsrp error:%d\n", repdat->rsrp);
        repdat->valid = LTE_INVALID;
      } else if (repdat->rsrq < APICMD_REP_QUALITY_RSRQ_MIN ||
                 APICMD_REP_QUALITY_RSRQ_MAX < repdat->rsrq) {
        DBGIF_LOG1_ERROR("repdat->rsrq error:%d\n", repdat->rsrq);
        repdat->valid = LTE_INVALID;
      } else if (repdat->sinr < APICMD_REP_QUALITY_SINR_MIN ||
                 APICMD_REP_QUALITY_SINR_MAX < repdat->sinr) {
        DBGIF_LOG1_ERROR("repdat->sinr error:%d\n", repdat->sinr);
        repdat->valid = LTE_INVALID;
      } else {
        /* Do nothing. */
      }
    }

    callback(repdat, cbpriv);
    (void)BUFFPOOL_FREE((FAR void *)repdat);
  }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdhdlr_repquality
 *
 * Description:
 *   This function is an API command handler for quality report.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_REPORT_QUALITY,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_repquality(FAR uint8_t *evt, uint32_t evlen) {
  return apicmdhdlrbs_do_runjob(evt, APICMDID_REPORT_QUALITY, repquality_job);
}

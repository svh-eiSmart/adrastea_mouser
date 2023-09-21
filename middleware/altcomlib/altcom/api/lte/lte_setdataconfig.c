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

#include <stdint.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "apiutil.h"
#include "apicmd_setdataconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETDATACONFIG_DATA_LEN (sizeof(struct apicmd_cmddat_setdataconfig_s))
#define SETDATACONFIG_RESP_LEN (sizeof(struct apicmd_cmddat_setdataconfigres_s))
#define DATA_TYPE_MIN LTE_DATA_TYPE_USER
#define DATA_TYPE_MAX LTE_DATA_TYPE_IMS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief lte_set_dataconfig() change configration of the data transfer; There are two types of data
 * that can be specified: user data or IMS.
 *
 * Details are shown in the table below.
 *
 * general  | roaming  | data transfer(Home/Roaming)
 * -------- | -------- | -----------------------------
 * disable  | disable  | Not available / Not available
 * disable  | enable   | Not available / Not available
 * enable   | disable  | Available / Not available
 * enable   | enable   | Available / Available
 *
 * @param [in] data_type: Data type. See @ref ltedatatype
 * @param [in] general: Data transfer for general.
 * See @ref lteenableflag_e
 * @param [in] roaming: Data transfer for roaming.
 * See @ref lteenableflag_e
 *
 * @return On success, LTE_RESULT_OK is returned. On failure,
 * LTE_RESULT_ERR is returned.
 */

lteresult_e lte_set_dataconfig(ltedatatype_e data_type, lteenableflag_e general,
                               lteenableflag_e roaming) {
  int32_t ret;
  uint16_t resLen = 0;
  FAR struct apicmd_cmddat_setdataconfig_s *cmd;
  FAR struct apicmd_cmddat_setdataconfigres_s *res;

  /* Return error if parameter is invalid */
  if (DATA_TYPE_MIN != data_type && DATA_TYPE_MAX != data_type) {
    DBGIF_LOG1_ERROR("Unsupport data type. type:%lu\n", (uint32_t)data_type);
    return LTE_RESULT_ERROR;
  }

  /* Check if the library is initialized */
  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    return LTE_RESULT_ERROR;
  }

  /* Allocate API command buffer to send */
  if (altcom_generic_alloc_cmdandresbuff((FAR void **)&cmd, APICMDID_SET_DATACONFIG,
                                         SETDATACONFIG_DATA_LEN, (FAR void **)&res,
                                         SETDATACONFIG_RESP_LEN)) {
    cmd->type = LTE_DATA_TYPE_USER == data_type ? (uint8_t)APICMD_SETDATACONFIG_TYPE_USERDATA
                                                : (uint8_t)APICMD_SETDATACONFIG_TYPE_IMS;
    cmd->general = LTE_ENABLE == general ? (uint8_t)APICMD_SETDATACONFIG_GENERAL_ENABLE
                                         : (uint8_t)APICMD_SETDATACONFIG_GENERAL_DISABLE;
    cmd->roaming = LTE_ENABLE == roaming ? (uint8_t)APICMD_SETDATACONFIG_ROAMING_ENABLE
                                         : (uint8_t)APICMD_SETDATACONFIG_ROAMING_DISABLE;

    /* Send API command to modem */
    ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res, SETDATACONFIG_RESP_LEN, &resLen,
                        ALTCOM_SYS_TIMEO_FEVR);
  } else {
    DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
    return LTE_RESULT_ERROR;
  }

  if (0 > ret) {
    DBGIF_LOG1_ERROR("apicmdgw_send error: %ld\n", ret);
    ret = LTE_RESULT_ERROR;
    goto errout_with_cmdfree;
  }

  if (SETDATACONFIG_RESP_LEN != resLen) {
    DBGIF_LOG1_ERROR("Unexpected response data length: %hu\n", resLen);
    ret = LTE_RESULT_ERROR;
    goto errout_with_cmdfree;
  }

  /* Check API return code */
  ret = (int32_t)res->result;

errout_with_cmdfree:
  altcom_generic_free_cmdandresbuff(cmd, res);
  return (lteresult_e)ret;
}

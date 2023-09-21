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
#include <stdbool.h>

#include "dbg_if.h"
#include "aws/altcom_aws.h"
#include "util/apiutil.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * External Data
 ****************************************************************************/
extern bool gAwsInit;
extern MQTTSession_t *gAwsSession;
extern altcom_sys_mq_t gAwsEvtQue[];

/****************************************************************************
 * External Function
 ****************************************************************************/
extern AWSError_e awsCheckApiStatus(altcom_sys_mq_t *, int32_t);

/**
 * Name: altcom_awsPublish
 *
 *   altcom_awsPublish() publish the specific topic from aws
 *
 *   @param [in] qos: The qos of publishing message; see @ref MQTTQoS_e.
 *   @param [in] topic: A specific topic to be subscribed(must be null-terminated).
 *   @param [in] msg: The message to be published(must be null-terminated).
 *
 *   @return AWS_SUCCESS on success;  AWS_FAILURE on fail.
 *
 */

AWSError_e altcom_awsPublish(AWSQoS_e qos, const char *topic, const char *msg) {
  int ret;

  /* Check init */
  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    return AWS_FAILURE;
  }

  if (!gAwsInit) {
    DBGIF_LOG_ERROR("AWS APIs not initialized\n");
    return AWS_FAILURE;
  }

  if (!gAwsSession) {
    DBGIF_LOG_ERROR("Invalid AWS session handle\n");
    return AWS_FAILURE;
  }

  if (qos > AWS_QOS0) {
    DBGIF_LOG1_ERROR("Invalid qos, %u\n", qos);
    return AWS_FAILURE;
  }

  if (!topic || !msg) {
    DBGIF_LOG2_ERROR("Invalid topic(%p) or msg(%p)\n", topic, msg);
    return AWS_FAILURE;
  }

  ret = altcom_mqttPublish(gAwsSession, (MQTTQoS_e)qos, 0, topic, msg, strlen(msg));
  if (ret == MQTT_FAILURE) {
    DBGIF_LOG1_ERROR("altcom_mqttPublish fail, ret = %d\n", ret);
    return AWS_FAILURE;
  }

  return AWS_SUCCESS;
}

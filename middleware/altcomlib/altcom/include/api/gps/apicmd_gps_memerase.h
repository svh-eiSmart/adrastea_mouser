/****************************************************************************
 * modules/lte/altcom/include/api/gps/altcom_gps_memerase.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_GPS_APICMD_GPS_MEMERASE_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_GPS_APICMD_GPS_MEMERASE_H
#include "apicmd.h"

begin_packed_struct struct apicmd_memerase_s { int32_t bitmap; } end_packed_struct;
begin_packed_struct struct apicmd_memeraseres_s { int32_t result; } end_packed_struct;

#define APICMD_MEMERASE_RES_RET_CODE_OK (0)
#define APICMD_MEMERASE_RES_RET_CODE_ERR (-1)

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_GPS_APICMD_GPS_MEMERASE_H */

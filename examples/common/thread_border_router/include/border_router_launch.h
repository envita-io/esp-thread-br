/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "esp_event.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_rcp_update.h"
#include "openthread/dataset.h"
#include "openthread/thread.h"

ESP_EVENT_DECLARE_BASE(HYP_OTBR_EVENT);

typedef enum {
    HYP_OTBR_EVENT_READY = 1,
    HYP_OTBR_EVENT_NOT_READY,
    HYP_OTBR_EVENT_INIT_READY,
    HYP_OTBR_EVENT_INIT_NOT_READY,
    HYP_OTBR_EVENT_AUTO_START_READY,
    HYP_OTBR_EVENT_PARENT_READY,
    HYP_OTBR_EVENT_PARENT_NOT_READY,
    /* Staged OTA WP4 (F-OTA-020): the H2 RCP path has a restart to make
     * (recovery finished, or a version mismatch / RCP failure) but the running
     * app image is still PENDING_VERIFY, so it is holding for the host's
     * first-boot acceptance instead of rolling the update back (F-RCP-002).
     * OpenThread does not start on this boot; the host's gate must not count
     * Thread while this holds. Posted once per hold, with a DISABLED role. */
    HYP_OTBR_EVENT_RCP_UPDATE_HOLDING_FOR_OTA,
} hyp_otbr_event_id_t;

typedef struct {
    otDeviceRole role;
    bool ip6_enabled;
} hyp_otbr_state_event_t;

/*
 * HYP F-OT-011: the dataset a gateway forms its network with on its very first
 * boot, when no Active Operational Dataset is stored in NVS yet. Weakly defined
 * in border_router_launch.c to return false, which keeps ESP-IDF's
 * esp_openthread_auto_start() Kconfig path (channel, PAN, keys from sdkconfig,
 * Active Timestamp 1). The application overrides it to ship a chosen Active
 * Timestamp (the fleet generation) and an explicit channel mask. Called once,
 * under the OpenThread lock, on the ot_br_init task (internal-RAM stack).
 * Returns true with *out filled; false to fall back to the Kconfig path.
 */
bool hyp_otbr_initial_dataset_tlvs(otInstance *instance, otOperationalDatasetTlvs *out);

void launch_openthread_border_router(const esp_openthread_platform_config_t *config,
                                     const esp_rcp_update_config_t *update_config);

#ifdef __cplusplus
} /* extern "C" */
#endif

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 */

#include "border_router_launch.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_spinel.h"
#include "esp_openthread_types.h"
#if CONFIG_AUTO_UPDATE_RCP
#include "esp_ota_ops.h"
#include "esp_rcp_firmware.h"
#endif
#include "esp_system.h"
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif
#include "esp_rcp_update.h"
#include "esp_vfs_eventfd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "openthread/backbone_router_ftd.h"
#include "openthread/border_router.h"
#include "openthread/border_routing.h"
#include "openthread/cli.h"
#include "openthread/dataset_ftd.h"
#include "openthread/error.h"
#include "openthread/instance.h"
#include "openthread/ip6.h"
#include "openthread/logging.h"
#include "openthread/platform/radio.h"
#include "openthread/tasklet.h"
#include "openthread/thread_ftd.h"
#include "nvs.h"

#if CONFIG_OPENTHREAD_CLI_WIFI
#include "esp_ot_wifi_cmd.h"
#endif

#if CONFIG_OPENTHREAD_BR_AUTO_START
#include "esp_wifi.h"
#include "example_common_private.h"
#include "protocol_examples_common.h"
#endif

#define TAG "esp_ot_br"
#define RCP_VERSION_MAX_SIZE 100

ESP_EVENT_DEFINE_BASE(HYP_OTBR_EVENT);

static esp_openthread_platform_config_t s_openthread_platform_config;

static esp_netif_t *openthread_netif;
static bool s_ot_init_ready_reported;
static bool s_otbr_ready_reported;
static bool s_ot_parent_ready_reported;

static bool is_thread_role_attached(otDeviceRole role)
{
    return (role == OT_DEVICE_ROLE_CHILD) ||
           (role == OT_DEVICE_ROLE_ROUTER) ||
           (role == OT_DEVICE_ROLE_LEADER);
}

static bool is_otbr_operational(otInstance *instance)
{
    otDeviceRole role;

    if (instance == NULL) {
        return false;
    }

    role = otThreadGetDeviceRole(instance);

    /*
     * The application services gated by HYP_OTBR_EVENT_READY are local Thread
     * services such as CoAP. They require the OT interface to be up and attached,
     * while border-routing RUNNING can lag or stay unavailable depending on the
     * backhaul runtime. Requiring border routing here can leave CoAP/log services
     * deferred even after the device has become a Thread router/leader.
     */
    return otIp6IsEnabled(instance) &&
           is_thread_role_attached(role);
}

static bool is_ot_parent_operational(otInstance *instance)
{
    otDeviceRole role;

    if (instance == NULL || !otIp6IsEnabled(instance)) {
        return false;
    }

    role = otThreadGetDeviceRole(instance);
    return (role == OT_DEVICE_ROLE_ROUTER) || (role == OT_DEVICE_ROLE_LEADER);
}

static hyp_otbr_state_event_t get_otbr_state(otInstance *instance)
{
    hyp_otbr_state_event_t state = {
        .role = OT_DEVICE_ROLE_DISABLED,
        .ip6_enabled = false,
    };

    if (instance != NULL) {
        state.role = otThreadGetDeviceRole(instance);
        state.ip6_enabled = otIp6IsEnabled(instance);
    }

    return state;
}

static bool post_otbr_event(int32_t event_id, otInstance *instance, const char *name, const char *reason)
{
    hyp_otbr_state_event_t state = get_otbr_state(instance);
    esp_err_t err;

    ESP_LOGI(TAG,
             "OpenThread state: %s, IPv6=%s, role=%d (%s)",
             name,
             state.ip6_enabled ? "enabled" : "disabled",
             (int)state.role,
             reason ? reason : "no reason");

    err = esp_event_post(HYP_OTBR_EVENT, event_id, &state, sizeof(state), portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to post OpenThread %s event: %s", name, esp_err_to_name(err));
        return false;
    }

    return true;
}

static void post_ot_init_state_if_changed(bool ready, otInstance *instance, const char *reason)
{
    if (ready == s_ot_init_ready_reported) {
        return;
    }

    if (post_otbr_event(ready ? HYP_OTBR_EVENT_INIT_READY : HYP_OTBR_EVENT_INIT_NOT_READY,
                        instance,
                        ready ? "init-ready" : "init-not-ready",
                        reason)) {
        s_ot_init_ready_reported = ready;
    }
}

static void post_otbr_state_if_changed(bool ready, otInstance *instance, const char *reason)
{
    if (ready == s_otbr_ready_reported) {
        return;
    }

    if (post_otbr_event(ready ? HYP_OTBR_EVENT_READY : HYP_OTBR_EVENT_NOT_READY,
                        instance,
                        ready ? "attached-ready" : "attached-not-ready",
                        reason)) {
        s_otbr_ready_reported = ready;
    }
}

static void post_ot_parent_state_if_changed(bool ready, otInstance *instance, const char *reason)
{
    if (ready == s_ot_parent_ready_reported) {
        return;
    }

    if (post_otbr_event(ready ? HYP_OTBR_EVENT_PARENT_READY : HYP_OTBR_EVENT_PARENT_NOT_READY,
                        instance,
                        ready ? "parent-ready" : "parent-not-ready",
                        reason)) {
        s_ot_parent_ready_reported = ready;
    }
}

static void report_otbr_state(otInstance *instance, const char *reason)
{
    post_otbr_state_if_changed(is_otbr_operational(instance), instance, reason);
    post_ot_parent_state_if_changed(is_ot_parent_operational(instance), instance, reason);
}

static void otbr_state_changed_callback(otChangedFlags changed_flags, void *ctx)
{
    otInstance *instance = esp_openthread_get_instance();

    OT_UNUSED_VARIABLE(ctx);

    if (instance == NULL) {
        return;
    }

    if (changed_flags & (OT_CHANGED_THREAD_ROLE |
                         OT_CHANGED_THREAD_NETIF_STATE |
                         OT_CHANGED_THREAD_BACKBONE_ROUTER_STATE |
                         OT_CHANGED_THREAD_NETDATA)) {
        report_otbr_state(instance, "state-changed callback");
    }
}

#if CONFIG_AUTO_UPDATE_RCP
#define HYP_RCP_RECOVERY_PENDING_KEY "h2_rec"
#define HYP_RCP_CONFIRMED_VERSION_KEY "h2_ver"
#define HYP_RCP_UPDATE_SEQ_KEY "rcp_seq"
#define HYP_RCP_RECOVERY_INITIAL_DELAY_MS 5000
#define HYP_RCP_RECOVERY_MAX_DELAY_MS 60000

static bool s_rcp_recovery_restart_requested;

static esp_err_t open_rcp_nvs(nvs_handle_t *handle)
{
    return nvs_open("storage", NVS_READWRITE, handle);
}

static bool nvs_key_exists(const char *key)
{
    nvs_handle_t handle;
    int8_t dummy = 0;
    bool exists = false;

    if (open_rcp_nvs(&handle) == ESP_OK) {
        exists = (nvs_get_i8(handle, key, &dummy) == ESP_OK);
        nvs_close(handle);
    }
    return exists;
}

static bool rcp_recovery_pending(void)
{
    nvs_handle_t handle;
    uint8_t pending = 0;

    if (open_rcp_nvs(&handle) != ESP_OK) {
        return false;
    }
    (void)nvs_get_u8(handle, HYP_RCP_RECOVERY_PENDING_KEY, &pending);
    nvs_close(handle);
    return pending == 1;
}

static void set_rcp_recovery_pending(bool pending)
{
    nvs_handle_t handle;

    if (open_rcp_nvs(&handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS while updating H2 RCP recovery flag");
        return;
    }

    if (pending) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_u8(handle, HYP_RCP_RECOVERY_PENDING_KEY, 1));
    } else {
        esp_err_t err = nvs_erase_key(handle, HYP_RCP_RECOVERY_PENDING_KEY);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Failed to clear H2 RCP recovery flag: %s", esp_err_to_name(err));
        }
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(handle));
    nvs_close(handle);
}

static bool load_confirmed_rcp_version(char *version, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err;

    if (open_rcp_nvs(&handle) != ESP_OK) {
        return false;
    }
    err = nvs_get_str(handle, HYP_RCP_CONFIRMED_VERSION_KEY, version, &size);
    nvs_close(handle);
    return err == ESP_OK;
}

static void save_confirmed_rcp_version(const char *version)
{
    nvs_handle_t handle;

    if (version == NULL || version[0] == '\0' || open_rcp_nvs(&handle) != ESP_OK) {
        return;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_set_str(handle, HYP_RCP_CONFIRMED_VERSION_KEY, version));
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_commit(handle));
    nvs_close(handle);
}

/* F-RCP-002: never restart out from under a pending OTA verification.
 *
 * The first boot after a gateway OTA runs with the new image in
 * ESP_OTA_IMG_PENDING_VERIFY. Since staged OTA WP4 (F-OTA-020) the image is
 * marked valid by the HOST's first-boot gate (main/utils/ota_staging.c: MQTT
 * connected on either broker path and one gateway status publish acknowledged,
 * plus border router ready and CoAP serving - except while this path holds, see
 * below), no longer by the OTA agent's boot-time StartNext answer. Nothing else
 * in the firmware clears the pending state.
 *
 * Every restart in this file races that. The first boot after an update always
 * reflashes the H2, because the bundled RCP version no longer matches the one
 * confirmed in NVS (and gateways flashed before 2026-06-24 have never written a
 * confirmed version at all), and the reflash finishes in tens of seconds while
 * the StartNext answer travels over a cellular backhaul where it can be lost
 * outright and re-driven by the job-check watchdog. Restarting first makes the
 * bootloader roll the gateway back onto the firmware it just replaced, and that
 * firmware then re-downloads the same 3 MB image on its next boot - invisibly,
 * because pre-F-OTA-009 firmware cannot report a failed job.
 *
 * So wait for the verification to resolve first. A restart after the timeout is
 * exactly the old behaviour and does roll back, which is the correct outcome
 * when the cloud never confirms the update. A running image that is not pending
 * verification - every normal boot, and every factory-fresh unit - returns
 * immediately, so nothing else changes.
 *
 * Task context: all callers run on internal-RAM stacks (h2_rcp_recovery,
 * ot_br_start, ot_br_main), which is required because reading the image state
 * touches the otadata partition. */
#define HYP_RCP_RESTART_OTA_WAIT_MS 600000
#define HYP_RCP_RESTART_OTA_POLL_MS 1000
#define HYP_RCP_RESTART_OTA_LOG_EVERY_MS 30000

static void wait_for_pending_ota_verification(const char *reason)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_VALID;
    uint32_t waited_ms = 0;

    if (running == NULL || esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return;
    }

    if (state != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }

    ESP_LOGW(TAG,
             "%s, but the running OTA image is still pending verification; waiting up to %d s "
             "so the restart cannot roll the update back",
             reason ? reason : "Restart requested", HYP_RCP_RESTART_OTA_WAIT_MS / 1000);

    /* Staged OTA WP4: tell the host we are holding. OpenThread will not start on
     * this boot (this path ends in a restart), so the host's gate must not wait
     * for Thread while we wait for the host - that would be a cycle that only
     * the timeout below could break, and it breaks it with a rollback. With this
     * event the gate accepts on the cloud-side conditions alone, we see VALID
     * within a second, restart, and the next boot brings Thread up as usual.
     * No OT instance exists here, so the state carries a DISABLED role. */
    post_otbr_event(HYP_OTBR_EVENT_RCP_UPDATE_HOLDING_FOR_OTA, NULL,
                    "rcp-update-holding-for-ota", reason ? reason : "restart requested");

    while (waited_ms < HYP_RCP_RESTART_OTA_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(HYP_RCP_RESTART_OTA_POLL_MS));
        waited_ms += HYP_RCP_RESTART_OTA_POLL_MS;

        if (esp_ota_get_state_partition(running, &state) != ESP_OK) {
            return;
        }

        if (state != ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "OTA image validated after %lu s; restarting now",
                     (unsigned long)(waited_ms / 1000));
            return;
        }

        if ((waited_ms % HYP_RCP_RESTART_OTA_LOG_EVERY_MS) == 0) {
            ESP_LOGW(TAG, "Still waiting for the OTA image to be validated (%lu s)",
                     (unsigned long)(waited_ms / 1000));
        }
    }

    ESP_LOGE(TAG, "OTA image was not validated within %d s; restarting anyway - the bootloader "
                  "will roll back to the previous firmware",
             HYP_RCP_RESTART_OTA_WAIT_MS / 1000);
}

static void request_rcp_recovery_restart(const char *reason)
{
    if (s_rcp_recovery_restart_requested) {
        vTaskDelay(portMAX_DELAY);
    }

    s_rcp_recovery_restart_requested = true;
    ESP_LOGE(TAG, "H2 RCP recovery requested: %s", reason ? reason : "unknown reason");
    set_rcp_recovery_pending(true);
    wait_for_pending_ota_verification("H2 RCP recovery needs a restart");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    vTaskDelay(portMAX_DELAY);
}

static void start_rcp_recovery_if_needed(const char *reason)
{
    set_rcp_recovery_pending(true);
    ESP_LOGW(TAG, "H2 RCP recovery will run before OpenThread starts: %s", reason);
}

static esp_err_t seek_to_rcp_subfile(FILE *fp, esp_rcp_filetag_t tag, esp_rcp_subfile_info_t *found_info)
{
    if (fseek(fp, 0, SEEK_SET) != 0) {
        return ESP_FAIL;
    }

    esp_rcp_subfile_info_t subfile_info;
    if (fread(&subfile_info, 1, sizeof(subfile_info), fp) != sizeof(subfile_info)) {
        return ESP_FAIL;
    }
    if (subfile_info.tag != FILETAG_IMAGE_HEADER || subfile_info.size % sizeof(subfile_info) != 0) {
        return ESP_FAIL;
    }

    int num_subfiles = subfile_info.size / sizeof(subfile_info);
    if (num_subfiles <= 1 || num_subfiles > MAX_SUBFILE_INFO) {
        return ESP_FAIL;
    }

    for (int i = 1; i < num_subfiles; i++) {
        if (fread(&subfile_info, 1, sizeof(subfile_info), fp) != sizeof(subfile_info)) {
            return ESP_FAIL;
        }
        if (subfile_info.tag == tag) {
            *found_info = subfile_info;
            return fseek(fp, subfile_info.offset, SEEK_SET) == 0 ? ESP_OK : ESP_FAIL;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t load_rcp_version_from_slot(int8_t slot, char *version, size_t size)
{
    char fullpath[RCP_FILENAME_MAX_SIZE];
    int path_len;
    FILE *fp = NULL;
    esp_rcp_subfile_info_t version_info;
    esp_err_t err = ESP_OK;

    if (version == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    version[0] = '\0';

    path_len = snprintf(fullpath, sizeof(fullpath), "%s_%d/" ESP_RCP_IMAGE_FILENAME,
                        esp_rcp_get_firmware_dir(), slot);
    if (path_len < 0 || (size_t)path_len >= sizeof(fullpath)) {
        return ESP_ERR_INVALID_SIZE;
    }

    fp = fopen(fullpath, "rb");
    if (fp == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    err = seek_to_rcp_subfile(fp, FILETAG_RCP_VERSION, &version_info);
    if (err == ESP_OK) {
        size_t read_size = (size - 1 < version_info.size) ? size - 1 : version_info.size;
        if (fread(version, 1, read_size, fp) != read_size) {
            err = ESP_FAIL;
        } else {
            version[read_size] = '\0';
        }
    }

    fclose(fp);
    return err;
}

static esp_err_t select_next_rcp_image_if_available(const char *reason)
{
    int8_t current_seq = esp_rcp_get_update_seq();
    int8_t next_seq = esp_rcp_get_next_update_seq();
    char next_version[RCP_VERSION_MAX_SIZE] = {0};
    esp_err_t err = load_rcp_version_from_slot(next_seq, next_version, sizeof(next_version));

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s; no fallback RCP image in slot %d: %s",
                 reason ? reason : "RCP image selection requested", next_seq, esp_err_to_name(err));
        return err;
    }

    err = esp_rcp_submit_new_image();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select fallback RCP image slot %d: %s",
                 next_seq, esp_err_to_name(err));
        return err;
    }

    ESP_LOGW(TAG, "%s; selected fallback RCP image slot %d (previous slot %d), version: %s",
             reason ? reason : "RCP image selection requested", next_seq, current_seq, next_version);
    return ESP_OK;
}

static bool current_rcp_image_needs_preflight_update(void)
{
    char internal_version[RCP_VERSION_MAX_SIZE] = {0};
    char confirmed_version[RCP_VERSION_MAX_SIZE] = {0};
    esp_err_t version_err = esp_rcp_load_version_in_storage(internal_version, sizeof(internal_version));

    if (rcp_recovery_pending()) {
        ESP_LOGW(TAG, "Pending H2 RCP recovery flag found");
        if (version_err != ESP_OK) {
            (void)select_next_rcp_image_if_available("pending H2 RCP recovery points to a missing image");
        }
        return true;
    }

    if (!nvs_key_exists(HYP_RCP_UPDATE_SEQ_KEY)) {
        if (version_err != ESP_OK) {
            version_err = select_next_rcp_image_if_available("RCP sequence is unconfirmed and current image is missing");
        }
        if (version_err != ESP_OK) {
            ESP_LOGW(TAG, "RCP sequence is unconfirmed, but no bundled RCP image was found; trying current H2 image");
            return false;
        }
        start_rcp_recovery_if_needed("RCP sequence has never been confirmed");
        return true;
    }

    if (version_err != ESP_OK) {
        if (select_next_rcp_image_if_available("current RCP image slot is missing") == ESP_OK) {
            start_rcp_recovery_if_needed("selected fallback RCP image because the current slot is missing");
            return true;
        }
        ESP_LOGW(TAG, "No usable RCP firmware image found in storage; OpenThread will use the current H2 image");
        return false;
    }

    if (!load_confirmed_rcp_version(confirmed_version, sizeof(confirmed_version))) {
        start_rcp_recovery_if_needed("H2 running RCP version has never been confirmed");
        return true;
    }

    if (strcmp(internal_version, confirmed_version) != 0) {
        ESP_LOGW(TAG, "Bundled RCP changed. confirmed=%s, bundled=%s", confirmed_version, internal_version);
        start_rcp_recovery_if_needed("bundled RCP version changed");
        return true;
    }

    return false;
}

static void rcp_recovery_task(void *ctx)
{
    uint32_t delay_ms = HYP_RCP_RECOVERY_INITIAL_DELAY_MS;
    uint32_t attempt = 0;

    ESP_LOGW(TAG, "H2 RCP recovery task started; OpenThread will start after H2 is flashed successfully");

    while (true) {
        char internal_version[RCP_VERSION_MAX_SIZE] = {0};
        esp_err_t err;

        attempt++;
        err = esp_rcp_load_version_in_storage(internal_version, sizeof(internal_version));
        if (err != ESP_OK) {
            (void)select_next_rcp_image_if_available("H2 RCP recovery current image is missing");
            err = esp_rcp_load_version_in_storage(internal_version, sizeof(internal_version));
        }

        if (err == ESP_OK) {
            ESP_LOGW(TAG, "H2 RCP recovery attempt %lu, image version: %s", attempt, internal_version);
        } else {
            ESP_LOGE(TAG, "H2 RCP recovery attempt %lu: no RCP image found in storage", attempt);
            goto retry;
        }

        err = esp_rcp_update();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "H2 RCP recovery succeeded; restarting to launch OpenThread");
            esp_rcp_mark_image_verified(true);
            save_confirmed_rcp_version(internal_version);
            set_rcp_recovery_pending(false);
            /* F-RCP-002 */
            wait_for_pending_ota_verification("H2 RCP recovery finished");
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
        }

        ESP_LOGE(TAG, "H2 RCP recovery attempt %lu failed: %s; retrying in %lu ms",
                 attempt, esp_err_to_name(err), delay_ms);
retry:
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        delay_ms = delay_ms < HYP_RCP_RECOVERY_MAX_DELAY_MS / 2 ? delay_ms * 2 : HYP_RCP_RECOVERY_MAX_DELAY_MS;
    }
}

static void update_rcp(void)
{
    request_rcp_recovery_restart("running RCP version does not match bundled image");
}

static void try_update_ot_rcp(void)
{
    char internal_rcp_version[RCP_VERSION_MAX_SIZE];
    const char *running_rcp_version = otPlatRadioGetVersionString(esp_openthread_get_instance());

    if (esp_rcp_load_version_in_storage(internal_rcp_version, sizeof(internal_rcp_version)) == ESP_OK) {
        ESP_LOGI(TAG, "Internal RCP Version: %s", internal_rcp_version);
        ESP_LOGI(TAG, "Running  RCP Version: %s", running_rcp_version);
        if (strcmp(internal_rcp_version, running_rcp_version) == 0) {
            esp_rcp_mark_image_verified(true);
            save_confirmed_rcp_version(running_rcp_version);
        } else {
            update_rcp();
        }
    } else {
        ESP_LOGW(TAG, "RCP firmware not found in storage; keeping the running H2 RCP image");
        esp_rcp_mark_image_verified(true);
        save_confirmed_rcp_version(running_rcp_version);
    }
}
#endif // CONFIG_AUTO_UPDATE_RCP

static void rcp_failure_handler(void)
{
#if CONFIG_AUTO_UPDATE_RCP
    char internal_rcp_version[RCP_VERSION_MAX_SIZE];
    if (select_next_rcp_image_if_available("OpenThread reported RCP failure") == ESP_OK) {
        request_rcp_recovery_restart("OpenThread reported RCP failure; fallback RCP image selected");
    }

    if (esp_rcp_load_version_in_storage(internal_rcp_version, sizeof(internal_rcp_version)) == ESP_OK) {
        ESP_LOGI(TAG, "Internal RCP Version: %s", internal_rcp_version);
        request_rcp_recovery_restart("OpenThread reported RCP failure");
    } else {
        ESP_LOGE(TAG, "RCP firmware not found in storage after RCP failure");
        request_rcp_recovery_restart("OpenThread reported RCP failure but no bundled image was found");
    }
#endif
}

/* HYP F-OT-011: default when the application provides no first-boot dataset. */
__attribute__((weak)) bool hyp_otbr_initial_dataset_tlvs(otInstance *instance, otOperationalDatasetTlvs *out)
{
    (void)instance;
    (void)out;
    return false;
}

static void ot_br_init(void *ctx)
{
#if CONFIG_OPENTHREAD_CLI_WIFI
    ESP_ERROR_CHECK(esp_ot_wifi_config_init());
#endif
#if CONFIG_OPENTHREAD_BR_AUTO_START
    // Start OpenThread Border Router without blocking on any specific backhaul.
    // Whichever uplink (Wi‐Fi / Ethernet / PPP) becomes available will be used by the system.
    esp_openthread_lock_acquire(portMAX_DELAY);
    esp_openthread_set_backbone_netif(openthread_netif);
    ESP_ERROR_CHECK(esp_openthread_border_router_init());
    otOperationalDatasetTlvs dataset;
    otInstance *instance = esp_openthread_get_instance();
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    bool have_dataset = (error == OT_ERROR_NONE);
    if (!have_dataset) {
        /* HYP F-OT-011: first boot of this unit. The application chooses the
         * dataset (channel, Active Timestamp = fleet generation, channel mask);
         * without an override ESP-IDF forms it from Kconfig with Active
         * Timestamp 1. A stored dataset is never touched here. */
        have_dataset = hyp_otbr_initial_dataset_tlvs(instance, &dataset);
        ESP_LOGI(TAG, "No stored Thread dataset: forming the network from the %s",
                 have_dataset ? "application's first-boot dataset" : "ESP-IDF Kconfig dataset");
    }
    ESP_ERROR_CHECK(esp_openthread_auto_start(have_dataset ? &dataset : NULL));
    post_otbr_event(HYP_OTBR_EVENT_AUTO_START_READY,
                    instance,
                    "auto-start-ready",
                    "esp_openthread_auto_start returned ESP_OK");
    report_otbr_state(instance, "auto-start");
    esp_openthread_lock_release();
    ESP_LOGI(TAG, "OpenThread BR auto-started; backhaul will use whichever interface gets IP first");

    // Ethernet backhaul is owned by the application backhaul manager.
    // Do not implicitly start it here, or OTBR may take over child devices
    // before the preferred uplink runtime is actually ready.
    // OT is already started above, so Wi‐Fi-only scenarios are unblocked.
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_LOGI(TAG, "Skipping example Ethernet connect; uplink runtime is managed externally");
#endif
#endif // CONFIG_OPENTHREAD_BR_AUTO_START
    vTaskDelete(NULL);
}

static void ot_task_worker(void *ctx)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    openthread_netif = esp_netif_new(&cfg);
    s_ot_init_ready_reported = false;
    s_otbr_ready_reported = false;
    s_ot_parent_ready_reported = false;

    assert(openthread_netif != NULL);

    // Initialize the OpenThread stack
    esp_openthread_register_rcp_failure_handler(rcp_failure_handler);
    esp_openthread_set_compatibility_error_callback(rcp_failure_handler);
    esp_openthread_set_coprocessor_reset_failure_callback(rcp_failure_handler);
    esp_err_t err = esp_openthread_init(&s_openthread_platform_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OpenThread init failed before RCP became ready: %s", esp_err_to_name(err));
#if CONFIG_AUTO_UPDATE_RCP
        request_rcp_recovery_restart("OpenThread initialization failed");
#else
        ESP_ERROR_CHECK(err);
#endif
    }
#if CONFIG_AUTO_UPDATE_RCP
    try_update_ot_rcp();
#endif
    post_ot_init_state_if_changed(true,
                                  esp_openthread_get_instance(),
                                  "esp_openthread_init returned ESP_OK and RCP recovery was not requested");
    // Initialize border routing features
    esp_openthread_lock_acquire(portMAX_DELAY);
    /* HYP: registered under the OpenThread lock. The instance exists from
     * esp_openthread_init on and the application's internal-RAM workers may
     * already hold the lock (F-OT-010 / F-REPORT-014 register their own
     * state-changed callback); two unlocked registrations could pick the same
     * Notifier slot. */
    ESP_ERROR_CHECK(otSetStateChangedCallback(esp_openthread_get_instance(),
                                              otbr_state_changed_callback,
                                              NULL) == OT_ERROR_NONE ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&s_openthread_platform_config)));
#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
    esp_openthread_cli_create_task();
#endif
    esp_openthread_lock_release();

    ESP_ERROR_CHECK(xTaskCreate(ot_br_init, "ot_br_init", 6144,
                                NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    // Run the main loop
    esp_openthread_launch_mainloop();
    post_ot_parent_state_if_changed(false, esp_openthread_get_instance(), "mainloop exit");
    post_otbr_state_if_changed(false, esp_openthread_get_instance(), "mainloop exit");
    post_ot_init_state_if_changed(false, esp_openthread_get_instance(), "mainloop exit");

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();
#if CONFIG_AUTO_UPDATE_RCP
    esp_rcp_update_deinit();
#endif
    vTaskDelete(NULL);
}

void launch_openthread_border_router(const esp_openthread_platform_config_t *platform_config,
                                     const esp_rcp_update_config_t *update_config)
{
    s_openthread_platform_config = *platform_config;

#if CONFIG_AUTO_UPDATE_RCP
    ESP_ERROR_CHECK(esp_rcp_update_init(update_config));
    if (current_rcp_image_needs_preflight_update()) {
        if (xTaskCreate(rcp_recovery_task, "h2_rcp_recovery", 8192, NULL, 5, NULL) != pdPASS) {
            request_rcp_recovery_restart("failed to create H2 RCP recovery task");
        }
        return;
    }
#else
    OT_UNUSED_VARIABLE(update_config);
#endif

    /* Honour CONFIG_OPENTHREAD_TASK_PRIORITY instead of a hardcoded 5. The BR
     * builds its own mainloop task here rather than going through
     * esp_openthread_start(), which was the only consumer of that Kconfig entry,
     * so the setting had no effect on this path. The stack size stays explicit. */
    ESP_ERROR_CHECK(xTaskCreate(ot_task_worker, "ot_br_main", 8192,
                                xTaskGetCurrentTaskHandle(),
                                CONFIG_OPENTHREAD_TASK_PRIORITY, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

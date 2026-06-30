#include <stdio.h>
#include <string.h>

#include "dronecan_node.h"
#include "board_config.h"
#include "mcp2518fd.h"

#include "pico/time.h"
#include "canard.h"
#include "dronecan_msgs.h"

static CanardInstance g_canard;
static uint8_t g_memory_pool[4096];

static bool g_armed = false;
static float g_cmd_pct = 0.0f;
static uint64_t g_last_fc_update_us = 0;
static uint8_t g_local_node_id = DRONECAN_NODE_ID;
static uint8_t g_esc_index = DRONECAN_ESC_INDEX;

// Separate transfer-ID counters per transfer descriptor
static uint8_t g_tid_node_status = 0;
static uint8_t g_tid_esc_status = 0;

static void mark_fc_update(void)
{
    g_last_fc_update_us = to_us_since_boot(get_absolute_time());
}

static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;
    mark_fc_update();

    struct uavcan_equipment_esc_RawCommand msg;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &msg)) {
        printf("RawCommand decode failed\r\n");
        return;
    }

    if (msg.cmd.len > g_esc_index) {
        float norm = msg.cmd.data[g_esc_index] / 8192.0f;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        g_cmd_pct = norm * 100.0f;
    } else {
        g_cmd_pct = 0.0f;
    }
}

static void handle_ArmingStatus(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;
    mark_fc_update();

    struct uavcan_equipment_safety_ArmingStatus msg;
    if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &msg)) {
        printf("ArmingStatus decode failed\r\n");
        return;
    }

    g_armed = (msg.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED);
}

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (transfer->transfer_type != CanardTransferTypeBroadcast) {
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
        handle_RawCommand(ins, transfer);
        break;

    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
        handle_ArmingStatus(ins, transfer);
        break;

    default:
        break;
    }
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)ins;
    (void)source_node_id;

    if (transfer_type != CanardTransferTypeBroadcast) {
        return false;
    }

    switch (data_type_id) {
    case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
        return true;

    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
        return true;

    default:
        return false;
    }
}

uint8_t dronecan_node_resolve_node_id_from_board_id(uint32_t board_id)
{
    // DroneCAN v0 node IDs are 1..127. Keep the original FC-only node ID as a
    // safe fallback, but when the debug/FRAM board identifier is a small human
    // engine number, expose the controller as BASE + identifier. Example with
    // the defaults: board ID 6 -> DroneCAN node ID 46. This avoids six motors
    // all publishing uavcan.equipment.esc.Status from the same node ID.
    if (board_id >= 1u && board_id <= DRONECAN_NODE_ID_MAX_OFFSET) {
        const uint32_t node_id = (uint32_t)DRONECAN_NODE_ID_BASE + board_id;
        if (node_id >= 1u && node_id <= 127u) {
            return (uint8_t)node_id;
        }
    }

    return (uint8_t)DRONECAN_NODE_ID;
}

uint8_t dronecan_node_resolve_esc_index_from_board_id(uint32_t board_id)
{
    // PX4/DroneCAN ESC identity is NOT the node ID. PX4 sends one
    // uavcan.equipment.esc.RawCommand array where motor 1 is cmd[0], motor 2
    // is cmd[1], etc. The ESC Status message reports the same 0-based index.
    // Therefore board ID 1 -> ESC index 0, board ID 2 -> ESC index 1, ...
    if (board_id >= 1u && board_id <= ((uint32_t)DRONECAN_ESC_INDEX_MAX + 1u)) {
        return (uint8_t)(board_id - 1u);
    }

    return (uint8_t)DRONECAN_ESC_INDEX;
}

void dronecan_node_init_with_ids(uint8_t node_id, uint8_t esc_index)
{
    if (node_id == 0u || node_id > 127u) {
        node_id = (uint8_t)DRONECAN_NODE_ID;
    }

    if (esc_index > DRONECAN_ESC_INDEX_MAX) {
        esc_index = (uint8_t)DRONECAN_ESC_INDEX;
    }

    canardInit(&g_canard,
               g_memory_pool,
               sizeof(g_memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    canardSetLocalNodeID(&g_canard, node_id);

    g_local_node_id = node_id;
    g_esc_index = esc_index;
    g_armed = false;
    g_cmd_pct = 0.0f;
    g_last_fc_update_us = 0;
    g_tid_node_status = 0;
    g_tid_esc_status = 0;
}

void dronecan_node_init_with_node_id(uint8_t node_id)
{
    dronecan_node_init_with_ids(node_id, (uint8_t)DRONECAN_ESC_INDEX);
}

void dronecan_node_init(void)
{
    dronecan_node_init_with_node_id((uint8_t)DRONECAN_NODE_ID);
}

uint8_t dronecan_node_get_local_node_id(void)
{
    return g_local_node_id;
}

uint8_t dronecan_node_get_esc_index(void)
{
    return g_esc_index;
}

void dronecan_node_handle_frame(CanardCANFrame *frame, uint64_t timestamp_usec)
{
    canardHandleRxFrame(&g_canard, frame, timestamp_usec);
}

bool dronecan_node_get_armed(void)
{
    return g_armed;
}

float dronecan_node_get_cmd_pct(void)
{
    return g_cmd_pct;
}

uint32_t dronecan_node_fc_age_ms(uint64_t now_us)
{
    if (g_last_fc_update_us == 0) {
        return UINT32_MAX;
    }

    if (now_us <= g_last_fc_update_us) {
        return 0;
    }

    return (uint32_t)((now_us - g_last_fc_update_us) / 1000ULL);
}

bool dronecan_node_fc_alive(uint64_t now_us)
{
    return dronecan_node_fc_age_ms(now_us) < DRONECAN_FC_TIMEOUT_MS;
}

void dronecan_node_publish_node_status(uint32_t uptime_s, uint8_t health, uint8_t mode, uint16_t vendor_status)
{
    struct uavcan_protocol_NodeStatus pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.uptime_sec = uptime_s;
    pkt.health = health;
    pkt.mode = mode;
    pkt.sub_mode = 0;
    pkt.vendor_specific_status_code = vendor_status;

    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    const uint32_t len = uavcan_protocol_NodeStatus_encode(&pkt, buffer);

    (void)canardBroadcast(&g_canard,
                          UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                          UAVCAN_PROTOCOL_NODESTATUS_ID,
                          &g_tid_node_status,
                          CANARD_TRANSFER_PRIORITY_LOW,
                          buffer,
                          (uint16_t)len);
}

void dronecan_node_publish_esc_status(float rpm, float current_a, float temperature_c, float throttle_pct)
{
    struct uavcan_equipment_esc_Status pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.error_count = 0;
    pkt.voltage = 0.0f;
    pkt.current = current_a;
    pkt.temperature = temperature_c + 273.15f;
    pkt.rpm = rpm;
    pkt.power_rating_pct = throttle_pct;
    pkt.esc_index = g_esc_index;

    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
    const uint32_t len = uavcan_equipment_esc_Status_encode(&pkt, buffer);

    (void)canardBroadcast(&g_canard,
                          UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                          UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                          &g_tid_esc_status,
                          CANARD_TRANSFER_PRIORITY_LOW,
                          buffer,
                          (uint16_t)len);
}

void dronecan_node_process_tx(void)
{
    const CanardCANFrame* txf = NULL;

    while ((txf = canardPeekTxQueue(&g_canard)) != NULL) {
        if (mcp2518fd_transmit(txf)) {
            canardPopTxQueue(&g_canard);
        } else {
            break;
        }
    }
}

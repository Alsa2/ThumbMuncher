#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_MAX_SIZE 1
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE (0x8700F375556A8003ULL)
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID 1100

#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_DISARMED 0
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED 255

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class uavcan_equipment_safety_ArmingStatus_cxx_iface;
#endif

struct uavcan_equipment_safety_ArmingStatus {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = uavcan_equipment_safety_ArmingStatus_cxx_iface;
#endif
    uint8_t status;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _uavcan_equipment_safety_ArmingStatus_encode(struct uavcan_equipment_safety_ArmingStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_safety_ArmingStatus* msg);

static inline uint32_t uavcan_equipment_safety_ArmingStatus_encode(struct uavcan_equipment_safety_ArmingStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _uavcan_equipment_safety_ArmingStatus_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_safety_ArmingStatus* msg) {

    return _uavcan_equipment_safety_ArmingStatus_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)
static inline void __uavcan_equipment_safety_ArmingStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao);
static inline bool __uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao);
void __uavcan_equipment_safety_ArmingStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->status);
    *bit_ofs += 8;
}

/*
 decode uavcan_equipment_safety_ArmingStatus, return true on failure, false on success
*/
bool __uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->status);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_safety_ArmingStatus sample_uavcan_equipment_safety_ArmingStatus_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(uavcan_equipment_safety_ArmingStatus, UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID, UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE, UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_MAX_SIZE);
#endif
#endif

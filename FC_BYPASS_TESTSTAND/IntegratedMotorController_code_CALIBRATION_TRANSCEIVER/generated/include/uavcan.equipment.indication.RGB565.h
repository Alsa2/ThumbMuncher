#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define UAVCAN_EQUIPMENT_INDICATION_RGB565_MAX_SIZE 2
#define UAVCAN_EQUIPMENT_INDICATION_RGB565_SIGNATURE (0x58A7CEF41951EC34ULL)


struct uavcan_equipment_indication_RGB565 {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _uavcan_equipment_indication_RGB565_encode(struct uavcan_equipment_indication_RGB565* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _uavcan_equipment_indication_RGB565_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_indication_RGB565* msg);

static inline uint32_t uavcan_equipment_indication_RGB565_encode(struct uavcan_equipment_indication_RGB565* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _uavcan_equipment_indication_RGB565_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool uavcan_equipment_indication_RGB565_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_indication_RGB565* msg) {

    return _uavcan_equipment_indication_RGB565_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)
static inline void __uavcan_equipment_indication_RGB565_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_indication_RGB565* msg, bool tao);
static inline bool __uavcan_equipment_indication_RGB565_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_indication_RGB565* msg, bool tao);
void __uavcan_equipment_indication_RGB565_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_indication_RGB565* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 5, &msg->red);
    *bit_ofs += 5;
    canardEncodeScalar(buffer, *bit_ofs, 6, &msg->green);
    *bit_ofs += 6;
    canardEncodeScalar(buffer, *bit_ofs, 5, &msg->blue);
    *bit_ofs += 5;
}

/*
 decode uavcan_equipment_indication_RGB565, return true on failure, false on success
*/
bool __uavcan_equipment_indication_RGB565_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_indication_RGB565* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->red);
    *bit_ofs += 5;

    canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->green);
    *bit_ofs += 6;

    canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->blue);
    *bit_ofs += 5;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_indication_RGB565 sample_uavcan_equipment_indication_RGB565_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif

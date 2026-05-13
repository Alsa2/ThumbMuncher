#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE 7
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_SIGNATURE (0x3B131AC5EB69D2CDULL)
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_ID 10

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class uavcan_protocol_param_ExecuteOpcode_cxx_iface;
#endif

struct uavcan_protocol_param_ExecuteOpcodeResponse {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = uavcan_protocol_param_ExecuteOpcode_cxx_iface;
#endif
    int64_t argument;
    bool ok;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _uavcan_protocol_param_ExecuteOpcodeResponse_encode(struct uavcan_protocol_param_ExecuteOpcodeResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _uavcan_protocol_param_ExecuteOpcodeResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_ExecuteOpcodeResponse* msg);

static inline uint32_t uavcan_protocol_param_ExecuteOpcodeResponse_encode(struct uavcan_protocol_param_ExecuteOpcodeResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _uavcan_protocol_param_ExecuteOpcodeResponse_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool uavcan_protocol_param_ExecuteOpcodeResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_ExecuteOpcodeResponse* msg) {

    return _uavcan_protocol_param_ExecuteOpcodeResponse_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)
static inline void __uavcan_protocol_param_ExecuteOpcodeResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeResponse* msg, bool tao);
static inline bool __uavcan_protocol_param_ExecuteOpcodeResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeResponse* msg, bool tao);
void __uavcan_protocol_param_ExecuteOpcodeResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 48, &msg->argument);
    *bit_ofs += 48;
    canardEncodeScalar(buffer, *bit_ofs, 1, &msg->ok);
    *bit_ofs += 1;
}

/*
 decode uavcan_protocol_param_ExecuteOpcodeResponse, return true on failure, false on success
*/
bool __uavcan_protocol_param_ExecuteOpcodeResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_ExecuteOpcodeResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 48, true, &msg->argument);
    *bit_ofs += 48;

    canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->ok);
    *bit_ofs += 1;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_ExecuteOpcodeResponse sample_uavcan_protocol_param_ExecuteOpcodeResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif

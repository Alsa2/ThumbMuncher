#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_REQUESTVOTE_REQUEST_MAX_SIZE 9
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_REQUESTVOTE_REQUEST_SIGNATURE (0xCDDE07BB89A56356ULL)
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_REQUESTVOTE_REQUEST_ID 31

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class uavcan_protocol_dynamic_node_id_server_RequestVote_cxx_iface;
#endif

struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = uavcan_protocol_dynamic_node_id_server_RequestVote_cxx_iface;
#endif
    uint32_t term;
    uint32_t last_log_term;
    uint8_t last_log_index;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg);

static inline uint32_t uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg) {

    return _uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)
static inline void __uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, bool tao);
static inline bool __uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, bool tao);
void __uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->term);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->last_log_term);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->last_log_index);
    *bit_ofs += 8;
}

/*
 decode uavcan_protocol_dynamic_node_id_server_RequestVoteRequest, return true on failure, false on success
*/
bool __uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->term);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 32, false, &msg->last_log_term);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->last_log_index);
    *bit_ofs += 8;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest sample_uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif

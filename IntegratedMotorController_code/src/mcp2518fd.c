#include "mcp2518fd.h"
#include "board_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "canard.h"

#ifndef MCP_SPI_PORT
#define MCP_SPI_PORT spi0
#endif

#define MCP_REG_C1CON        0x000
#define MCP_REG_C1NBTCFG     0x004
#define MCP_REG_C1INT        0x01C
#define MCP_REG_C1TXQCON     0x050
#define MCP_REG_C1FIFOCON1   0x05C
#define MCP_REG_C1FIFOSTA1   0x060
#define MCP_REG_C1FIFOUA1    0x064
#define MCP_REG_C1FIFOCON2   0x068
#define MCP_REG_C1FIFOSTA2   0x06C
#define MCP_REG_C1FIFOUA2    0x070
#define MCP_REG_C1FLTCON0    0x1D0
#define MCP_REG_C1FLTOBJ0    0x1F0
#define MCP_REG_C1MASK0      0x1F4

#define MCP_REG_OSC          0xE00
#define MCP_REG_DEVID        0xE14

#define MCP_RAM_BASE         0x400u

#define MCP_C1CON_REQOP_SHIFT   24u
#define MCP_C1CON_OPMOD_SHIFT   21u
#define MCP_C1CON_BRSDIS        (1u << 12)

#define MCP_MODE_CONFIG         4u
#define MCP_MODE_NORMAL_CAN20   6u

#define MCP_OSC_OSCRDY          (1u << 10)

#define MCP_FIFOCON_UINC        (1u << 8)
#define MCP_FIFOCON_TXREQ       (1u << 9)
#define MCP_FIFOCON_FRESET      (1u << 10)
#define MCP_FIFOCON_TXEN        (1u << 7)

#define MCP_FIFOSTA_TFNRFNIF    (1u << 0)

#define MCP_RXOBJ_R1_DLC_MASK   0x0Fu
#define MCP_RXOBJ_R1_IDE        (1u << 4)
#define MCP_RXOBJ_R1_RTR        (1u << 5)
#define MCP_RXOBJ_R1_BRS        (1u << 6)
#define MCP_RXOBJ_R1_FDF        (1u << 7)

static inline void mcp_cs_select(void)   { gpio_put(PIN_CAN_CS, 0); }
static inline void mcp_cs_deselect(void) { gpio_put(PIN_CAN_CS, 1); }

static uint32_t le32_load(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void le32_store(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void mcp_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    mcp_cs_select();
    spi_write_read_blocking(MCP_SPI_PORT, tx, rx, len);
    mcp_cs_deselect();
}

static void mcp_reset(void)
{
    uint8_t tx[2] = {0x00, 0x00};
    mcp_cs_select();
    spi_write_blocking(MCP_SPI_PORT, tx, 2);
    mcp_cs_deselect();
}

static uint32_t mcp_read_sfr_u32(uint16_t addr)
{
    uint8_t tx[6] = {
        (uint8_t)((0x03u << 4) | ((addr >> 8) & 0x0Fu)),
        (uint8_t)(addr & 0xFFu),
        0, 0, 0, 0
    };
    uint8_t rx[6] = {0};
    mcp_spi_transfer(tx, rx, sizeof(tx));
    return le32_load(&rx[2]);
}

static void mcp_write_sfr_u32(uint16_t addr, uint32_t value)
{
    uint8_t tx[6] = {
        (uint8_t)((0x02u << 4) | ((addr >> 8) & 0x0Fu)),
        (uint8_t)(addr & 0xFFu),
        0, 0, 0, 0
    };
    le32_store(&tx[2], value);
    mcp_cs_select();
    spi_write_blocking(MCP_SPI_PORT, tx, sizeof(tx));
    mcp_cs_deselect();
}

static void mcp_read_ram(uint16_t ram_addr, uint8_t *dst, size_t len)
{
    uint8_t tx[2 + 64] = {0};
    uint8_t rx[2 + 64] = {0};

    if (len > 64) len = 64;
    tx[0] = (uint8_t)((0x03u << 4) | ((ram_addr >> 8) & 0x0Fu));
    tx[1] = (uint8_t)(ram_addr & 0xFFu);

    mcp_spi_transfer(tx, rx, 2 + len);
    memcpy(dst, &rx[2], len);
}

static void mcp_write_ram(uint16_t ram_addr, const uint8_t *src, size_t len)
{
    uint8_t tx[2 + 64] = {0};

    if (len > 64) len = 64;
    tx[0] = (uint8_t)((0x02u << 4) | ((ram_addr >> 8) & 0x0Fu));
    tx[1] = (uint8_t)(ram_addr & 0xFFu);
    memcpy(&tx[2], src, len);

    mcp_cs_select();
    spi_write_blocking(MCP_SPI_PORT, tx, 2 + len);
    mcp_cs_deselect();
}

static bool mcp_wait_osc_ready(uint32_t timeout_ms)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        if (mcp_read_sfr_u32(MCP_REG_OSC) & MCP_OSC_OSCRDY) {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

static bool mcp_request_mode(uint8_t mode)
{
    uint32_t c1con = mcp_read_sfr_u32(MCP_REG_C1CON);
    c1con &= ~(0x7u << MCP_C1CON_REQOP_SHIFT);
    c1con |= ((uint32_t)mode << MCP_C1CON_REQOP_SHIFT);
    mcp_write_sfr_u32(MCP_REG_C1CON, c1con);

    absolute_time_t deadline = make_timeout_time_ms(100);
    while (!time_reached(deadline)) {
        c1con = mcp_read_sfr_u32(MCP_REG_C1CON);
        const uint8_t opmod = (uint8_t)((c1con >> MCP_C1CON_OPMOD_SHIFT) & 0x7u);
        if (opmod == mode) {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

static bool mcp_configure_nominal_bitrate_20mhz(uint32_t bitrate_hz)
{
    uint32_t reg = 0;

    switch (bitrate_hz) {
    case 1000000u:
        reg = (0u  << 24) | (14u << 16) | (3u << 8) | 3u;
        break;
    case 500000u:
        reg = (0u  << 24) | (30u << 16) | (7u << 8) | 7u;
        break;
    case 250000u:
        reg = (1u  << 24) | (14u << 16) | (3u << 8) | 3u;
        break;
    case 125000u:
        reg = (3u  << 24) | (30u << 16) | (7u << 8) | 7u;
        break;
    default:
        return false;
    }

    mcp_write_sfr_u32(MCP_REG_C1NBTCFG, reg);
    return true;
}

static void mcp_fifo1_uinc(void)
{
    uint32_t v = mcp_read_sfr_u32(MCP_REG_C1FIFOCON1);
    v |= MCP_FIFOCON_UINC;
    mcp_write_sfr_u32(MCP_REG_C1FIFOCON1, v);
}

static void mcp_fifo2_uinc_txreq(void)
{
    uint32_t v = mcp_read_sfr_u32(MCP_REG_C1FIFOCON2);
    v |= (MCP_FIFOCON_UINC | MCP_FIFOCON_TXREQ);
    mcp_write_sfr_u32(MCP_REG_C1FIFOCON2, v);
}

static uint8_t mcp_dlc_to_len_classic(uint8_t dlc)
{
    static const uint8_t lut[16] = {0,1,2,3,4,5,6,7,8,8,8,8,8,8,8,8};
    return lut[dlc & 0x0Fu];
}

uint32_t mcp2518fd_read_devid(void)
{
    return mcp_read_sfr_u32(MCP_REG_DEVID);
}

bool mcp2518fd_init(void)
{
    spi_init(MCP_SPI_PORT, 1000 * 1000);
    spi_set_format(MCP_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_CAN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_CAN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CAN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CAN_CS);
    gpio_set_dir(PIN_CAN_CS, GPIO_OUT);
    gpio_put(PIN_CAN_CS, 1);

    gpio_init(PIN_CAN_INT);
    gpio_set_dir(PIN_CAN_INT, GPIO_IN);
    gpio_pull_up(PIN_CAN_INT);

    mcp_reset();
    sleep_ms(10);

    if (!mcp_wait_osc_ready(50)) {
        return false;
    }

    if (MCP_OSC_HZ != 20000000u) {
        return false;
    }

    uint32_t c1con = mcp_read_sfr_u32(MCP_REG_C1CON);
    c1con &= ~(0x7u << MCP_C1CON_REQOP_SHIFT);
    c1con |= ((uint32_t)MCP_MODE_CONFIG << MCP_C1CON_REQOP_SHIFT);
    c1con |= MCP_C1CON_BRSDIS;
    mcp_write_sfr_u32(MCP_REG_C1CON, c1con);

    if (!mcp_request_mode(MCP_MODE_CONFIG)) {
        return false;
    }

    if (!mcp_configure_nominal_bitrate_20mhz(CAN_BITRATE_HZ)) {
        return false;
    }

    mcp_write_sfr_u32(MCP_REG_C1TXQCON, 0u);

    uint32_t fifo1 =
        (0u << 29) |
        (7u << 24) |
        MCP_FIFOCON_FRESET |
        (1u << 5) |
        (1u << 0);
    mcp_write_sfr_u32(MCP_REG_C1FIFOCON1, fifo1);

    uint32_t fifo2 =
        (0u << 29) |
        (3u << 24) |
        MCP_FIFOCON_FRESET |
        MCP_FIFOCON_TXEN;
    mcp_write_sfr_u32(MCP_REG_C1FIFOCON2, fifo2);

    mcp_write_sfr_u32(MCP_REG_C1FLTOBJ0, (1u << 30));
    mcp_write_sfr_u32(MCP_REG_C1MASK0,   (1u << 30));
    mcp_write_sfr_u32(MCP_REG_C1FLTCON0, 0x81u);

    fifo1 &= ~MCP_FIFOCON_FRESET;
    fifo2 &= ~MCP_FIFOCON_FRESET;
    mcp_write_sfr_u32(MCP_REG_C1FIFOCON1, fifo1);
    mcp_write_sfr_u32(MCP_REG_C1FIFOCON2, fifo2);

    if (!mcp_request_mode(MCP_MODE_NORMAL_CAN20)) {
        return false;
    }

    return true;
}

bool mcp2518fd_receive(CanardCANFrame *out_frame)
{
    for (;;) {
        const uint32_t fsta = mcp_read_sfr_u32(MCP_REG_C1FIFOSTA1);
        if ((fsta & MCP_FIFOSTA_TFNRFNIF) == 0u) {
            return false;
        }

        const uint32_t fifoua = mcp_read_sfr_u32(MCP_REG_C1FIFOUA1);
        const uint16_t ram_addr = (uint16_t)(MCP_RAM_BASE + (fifoua & 0x0FFFu));

        uint8_t raw[20] = {0};
        mcp_read_ram(ram_addr, raw, sizeof(raw));

        const uint32_t r0 = le32_load(&raw[0]);
        const uint32_t r1 = le32_load(&raw[4]);

        const bool ide = (r1 & MCP_RXOBJ_R1_IDE) != 0u;
        const bool rtr = (r1 & MCP_RXOBJ_R1_RTR) != 0u;
        const bool fdf = (r1 & MCP_RXOBJ_R1_FDF) != 0u;
        const uint8_t dlc = (uint8_t)(r1 & MCP_RXOBJ_R1_DLC_MASK);

        mcp_fifo1_uinc();

        if (!ide || fdf || rtr) {
            continue;
        }

        const uint32_t sid = (r0 & 0x7FFu);
        const uint32_t eid = (r0 >> 11) & 0x3FFFFu;
        const uint32_t can_id = (sid << 18) | eid;

        memset(out_frame, 0, sizeof(*out_frame));
        out_frame->id = can_id | CANARD_CAN_FRAME_EFF;
        out_frame->data_len = mcp_dlc_to_len_classic(dlc);
        out_frame->iface_id = 0;
        memcpy(out_frame->data, &raw[12], out_frame->data_len);

        return true;
    }
}

bool mcp2518fd_transmit(const CanardCANFrame *in_frame)
{
    const uint32_t fsta = mcp_read_sfr_u32(MCP_REG_C1FIFOSTA2);
    if ((fsta & MCP_FIFOSTA_TFNRFNIF) == 0u) {
        return false;
    }

    const uint32_t fifoua = mcp_read_sfr_u32(MCP_REG_C1FIFOUA2);
    const uint16_t ram_addr = (uint16_t)(MCP_RAM_BASE + (fifoua & 0x0FFFu));

    uint8_t raw[16] = {0};
    uint32_t t0 = 0;
    uint32_t t1 = 0;

    const uint32_t can_id = (in_frame->id & 0x1FFFFFFFu);
    const uint32_t sid = (can_id >> 18) & 0x7FFu;
    const uint32_t eid = can_id & 0x3FFFFu;

    t0 = sid | (eid << 11);
    t1 = (uint32_t)(in_frame->data_len & 0x0Fu);
    t1 |= (1u << 4);
    if (in_frame->id & CANARD_CAN_FRAME_RTR) {
        t1 |= (1u << 5);
    }

    le32_store(&raw[0], t0);
    le32_store(&raw[4], t1);

    const uint8_t data_len = (in_frame->data_len <= 8u) ? in_frame->data_len : 8u;
    memcpy(&raw[8], in_frame->data, data_len);

    /*
     * MCP2518FD TX FIFO objects are fixed-size in RAM for the selected payload
     * size. For classic CAN with PLSIZE=0, each TX object is 16 bytes:
     *   8 bytes TX header + 8 bytes data area.
     *
     * Do not shorten this write for DLC < 8. The controller can otherwise keep
     * stale data bytes in the object, which corrupts short final frames in a
     * DroneCAN multi-frame transfer. That was the root cause of PX4 transfer
     * errors: the last 3-byte ESC Status frame reused bytes from the previous
     * 8-byte frame.
     */
    mcp_write_ram(ram_addr, raw, sizeof(raw));
    mcp_fifo2_uinc_txreq();
    return true;
}

uint32_t mcp2518fd_debug_read_osc(void)      { return mcp_read_sfr_u32(MCP_REG_OSC); }
uint32_t mcp2518fd_debug_read_c1con(void)    { return mcp_read_sfr_u32(MCP_REG_C1CON); }
uint32_t mcp2518fd_debug_read_c1int(void)    { return mcp_read_sfr_u32(MCP_REG_C1INT); }
uint32_t mcp2518fd_debug_read_fifo1sta(void) { return mcp_read_sfr_u32(MCP_REG_C1FIFOSTA1); }
uint32_t mcp2518fd_debug_read_fifo1ua(void)  { return mcp_read_sfr_u32(MCP_REG_C1FIFOUA1); }

void mcp2518fd_debug_get_modes(uint8_t *reqop, uint8_t *opmod)
{
    const uint32_t c1con = mcp_read_sfr_u32(MCP_REG_C1CON);
    if (reqop) *reqop = (uint8_t)((c1con >> MCP_C1CON_REQOP_SHIFT) & 0x7u);
    if (opmod) *opmod = (uint8_t)((c1con >> MCP_C1CON_OPMOD_SHIFT) & 0x7u);
}

bool mcp2518fd_debug_peek_fifo1(uint8_t raw20[20])
{
    const uint32_t fsta = mcp_read_sfr_u32(MCP_REG_C1FIFOSTA1);
    if ((fsta & MCP_FIFOSTA_TFNRFNIF) == 0u) {
        return false;
    }

    const uint32_t fifoua = mcp_read_sfr_u32(MCP_REG_C1FIFOUA1);
    const uint16_t ram_addr = (uint16_t)(MCP_RAM_BASE + (fifoua & 0x0FFFu));
    mcp_read_ram(ram_addr, raw20, 20);
    return true;
}
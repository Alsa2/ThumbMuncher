from machine import Pin, SPI
import time

PIN_SCK  = 2
PIN_MOSI = 3
PIN_MISO = 4
PIN_CS   = 5
PIN_INT  = 6

cs = Pin(PIN_CS, Pin.OUT, value=1)
int_pin = Pin(PIN_INT, Pin.IN, Pin.PULL_UP)

spi = SPI(
    0,
    baudrate=1_000_000,
    polarity=0,
    phase=0,
    bits=8,
    sck=Pin(PIN_SCK),
    mosi=Pin(PIN_MOSI),
    miso=Pin(PIN_MISO),
)

C1CON      = 0x000
C1NBTCFG   = 0x004
C1DBTCFG   = 0x008
C1TREC     = 0x034
C1BDIAG1   = 0x03C

C1FIFOCON1 = 0x05C
C1FIFOSTA1 = 0x060
C1FIFOUA1  = 0x064

RAM_BASE   = 0x400

def xfer(tx_bytes):
    tx = bytes(tx_bytes)
    rx = bytearray(len(tx))
    cs.value(0)
    time.sleep_us(2)
    spi.write_readinto(tx, rx)
    time.sleep_us(2)
    cs.value(1)
    return rx

def reset_chip():
    xfer([0x00, 0x00])
    time.sleep_ms(5)

def read_bytes(addr, nbytes):
    h0 = 0x30 | ((addr >> 8) & 0x0F)
    h1 = addr & 0xFF
    rx = xfer([h0, h1] + [0x00] * nbytes)
    return bytes(rx[2:])

def write_bytes(addr, data):
    h0 = 0x20 | ((addr >> 8) & 0x0F)
    h1 = addr & 0xFF
    xfer([h0, h1] + list(data))

def read32(addr):
    b = read_bytes(addr, 4)
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)

def write32(addr, value):
    write_bytes(addr, bytes([
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]))

def get_opmod():
    return (read32(C1CON) >> 21) & 0x7

def request_mode(reqop, timeout_ms=200):
    v = read32(C1CON)
    v &= ~(0x7 << 24)
    v |= (reqop & 0x7) << 24
    write32(C1CON, v)

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if get_opmod() == reqop:
            return True
        time.sleep_ms(1)
    return False

def build_tx_obj_std(sid, data8):
    if len(data8) != 8:
        raise ValueError("Need exactly 8 data bytes")

    sid &= 0x7FF

    return bytes([
        sid & 0xFF,
        (sid >> 8) & 0x07,
        0x00,
        0x00,
        0x08,
        0x00,
        0x00,
        0x00
    ]) + bytes(data8)

def can_init_external_loopback():
    reset_chip()

    if get_opmod() != 4:
        print("Did not come up in Configuration mode")
        return False

    nbtcfg = (3 << 24) | (30 << 16) | (7 << 8) | 7
    write32(C1NBTCFG, nbtcfg)

    dbtcfg = (9 << 24) | (11 << 16) | (2 << 8) | 2
    write32(C1DBTCFG, dbtcfg)

    fifocon1 = (0 << 29) | (0 << 24) | (31 << 16) | (1 << 7)
    write32(C1FIFOCON1, fifocon1)

    ok = request_mode(0b101)
    print("Requested External Loopback:", ok, "OPMOD =", get_opmod())
    return ok

def send_one_std_frame(sid, data8):
    fua = read32(C1FIFOUA1)
    ram_addr = RAM_BASE + fua

    msg = build_tx_obj_std(sid, data8)
    write_bytes(ram_addr, msg)

    v = read32(C1FIFOCON1)
    v |= (1 << 8) | (1 << 9)   # UINC + TXREQ
    write32(C1FIFOCON1, v)

    time.sleep_ms(20)

    txsta = read32(C1FIFOSTA1)
    trec = read32(C1TREC)
    bdiag1 = read32(C1BDIAG1)

    print("FIFOUA=0x{:03X} RAM=0x{:03X}".format(fua, ram_addr))
    print("TXSTA = 0x{:08X}".format(txsta))
    print("TREC  = 0x{:08X}".format(trec))
    print("BDIAG1= 0x{:08X}".format(bdiag1))
    print("INT =", int_pin.value())
    print()

if can_init_external_loopback():
    counter = 0
    while True:
        if counter & 1:
            payload = [0x55, 0xAA, 0x55, 0xAA, 0x12, 0x34, 0x56, 0x78]
        else:
            payload = [0x00, 0xFF, 0x00, 0xFF, 0xDE, 0xAD, 0xBE, 0xEF]

        print("Sending frame", counter)
        send_one_std_frame(0x123, payload)
        counter += 1
        time.sleep_ms(500)
else:
    print("CAN init failed")
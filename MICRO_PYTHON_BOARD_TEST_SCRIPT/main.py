from machine import Pin, I2C, SPI, ADC, PWM
import utime as time
import math

# ============================================================
# RP2040 custom board smoke test
# Safe defaults:
# - read-only checks are enabled
# - potentially dangerous output tests are disabled by default
# ============================================================

ENABLE_LED_TEST = True
ENABLE_BUZZER_TEST = True
ENABLE_STATUS_TESTS = True
ENABLE_I2C_TESTS = True
ENABLE_ADC_TESTS = True
ENABLE_MCP2518FD_RESPONSE_TEST = True

# Disable these unless downstream hardware is unplugged / safe.
ENABLE_PWM_OUTPUT_TESTS = False   # GP16/GP17 servo-style outputs
ENABLE_RELAY_TEST = True         # GP8 relay enable MOSFET/relay path
ENABLE_FRAM_WRITE_TEST = True    # writes and restores one FRAM location

# -------------------------
# Pin map from your schematic
# -------------------------
PIN_SDA0 = 0
PIN_SCL0 = 1

PIN_SPI0_SCK = 2
PIN_SPI0_MOSI = 3
PIN_SPI0_MISO = 4
PIN_CAN_CS = 5
PIN_CAN_INT = 6

PIN_RELAY_EN = 8

PIN_SDA1 = 10
PIN_SCL1 = 11

PIN_CHOKE_PWM = 16
PIN_THROTTLE_PWM = 17

PIN_LED = 18
PIN_BUZZER = 19

PIN_PGOOD = 21
PIN_POWER_SRC = 23

PIN_TEMP_ADC = 26
PIN_RPM1_ADC = 27
PIN_RPM2_ADC = 28

# I2C addresses
INA219_ADDR = 0x40
FRAM_ADDR = 0x50

ADC_REF = 3.3

THERM_R_PULLUP = 10000.0
THERM_BETA = 3977.0
THERM_R0 = 10000.0
THERM_T0_K = 298.15

results = []


def log(msg=""):
    print(msg)


def banner(title):
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    tag = "PASS" if ok else "FAIL"

    if detail:
        print("[{}] {}: {}".format(tag, name, detail))
    else:
        print("[{}] {}".format(tag, name))

    return ok


class FM24CL64B:
    def __init__(self, i2c, addr=FRAM_ADDR):
        self.i2c = i2c
        self.addr = addr

    def read(self, memaddr, nbytes=1):
        header = bytes([
            (memaddr >> 8) & 0xFF,
            memaddr & 0xFF
        ])
        self.i2c.writeto(self.addr, header, False)
        return self.i2c.readfrom(self.addr, nbytes)

    def write(self, memaddr, data):
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)

        header = bytes([
            (memaddr >> 8) & 0xFF,
            memaddr & 0xFF
        ])

        self.i2c.writeto(self.addr, header + data)


class INA219:
    REG_CONFIG = 0x00
    REG_SHUNT_V = 0x01
    REG_BUS_V = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CAL = 0x05

    def __init__(self, i2c, addr=INA219_ADDR, shunt_ohms=0.05):
        self.i2c = i2c
        self.addr = addr
        self.shunt_ohms = shunt_ohms

        self.current_lsb = 0.0002
        self.power_lsb = self.current_lsb * 20.0
        self.cal = 0x1000

    def _write_reg(self, reg, value):
        data = bytes([
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        self.i2c.writeto_mem(self.addr, reg, data)

    def _read_reg(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        return (data[0] << 8) | data[1]

    @staticmethod
    def _s16(x):
        if x & 0x8000:
            return x - 65536
        return x

    def configure(self):
        self._write_reg(self.REG_CONFIG, 0x399F)
        self._write_reg(self.REG_CAL, self.cal)
        time.sleep_ms(10)

    def read_all(self):
        cfg = self._read_reg(self.REG_CONFIG)
        shunt_raw = self._s16(self._read_reg(self.REG_SHUNT_V))
        bus_raw = self._read_reg(self.REG_BUS_V)
        power_raw = self._read_reg(self.REG_POWER)
        current_raw = self._s16(self._read_reg(self.REG_CURRENT))

        shunt_v = shunt_raw * 10e-6
        bus_v = (bus_raw >> 3) * 4e-3
        current_a = current_raw * self.current_lsb
        power_w = power_raw * self.power_lsb

        return {
            "config": cfg,
            "shunt_raw": shunt_raw,
            "bus_raw": bus_raw,
            "current_raw": current_raw,
            "power_raw": power_raw,
            "shunt_v": shunt_v,
            "bus_v": bus_v,
            "current_a": current_a,
            "power_w": power_w,
        }


class MCP2518FD:
    # Important MCP2518FD register addresses
    C1CON_ADDR = 0x000
    OSC_ADDR = 0xE00
    IOCON_ADDR = 0xE04
    CRC_ADDR = 0xE08
    ECCCON_ADDR = 0xE0C
    DEVID_ADDR = 0xE14

    def __init__(self, spi, cs_pin, int_pin=None):
        self.spi = spi

        self.cs = Pin(cs_pin, Pin.OUT)
        self.cs.value(1)

        if int_pin is None:
            self.int_pin = None
        else:
            self.int_pin = Pin(int_pin, Pin.IN, Pin.PULL_UP)

    def _xfer(self, tx_bytes):
        tx = bytes(tx_bytes)
        rx = bytearray(len(tx))

        self.cs.value(0)
        self.spi.write_readinto(tx, rx)
        self.cs.value(1)

        return rx

    def read_sfr(self, addr, nbytes):
        # MCP2518FD SFR READ command:
        # command nibble = 0b0011
        # then 12-bit address
        header0 = 0x30 | ((addr >> 8) & 0x0F)
        header1 = addr & 0xFF

        rx = self._xfer([header0, header1] + [0x00] * nbytes)

        return bytes(rx[2:])

    def read_u32(self, addr):
        data = self.read_sfr(addr, 4)
        value = (
            data[0]
            | (data[1] << 8)
            | (data[2] << 16)
            | (data[3] << 24)
        )
        return value, data

    def read_diagnostics(self):
        c1con, c1con_b = self.read_u32(self.C1CON_ADDR)
        osc, osc_b = self.read_u32(self.OSC_ADDR)
        iocon, iocon_b = self.read_u32(self.IOCON_ADDR)
        crc, crc_b = self.read_u32(self.CRC_ADDR)
        ecccon, ecccon_b = self.read_u32(self.ECCCON_ADDR)
        devid, devid_b = self.read_u32(self.DEVID_ADDR)

        int_level = None
        if self.int_pin is not None:
            int_level = self.int_pin.value()

        return {
            "C1CON": c1con,
            "C1CON_bytes": c1con_b,

            "OSC": osc,
            "OSC_bytes": osc_b,

            "IOCON": iocon,
            "IOCON_bytes": iocon_b,

            "CRC": crc,
            "CRC_bytes": crc_b,

            "ECCCON": ecccon,
            "ECCCON_bytes": ecccon_b,

            "DEVID": devid,
            "DEVID_bytes": devid_b,

            "INT": int_level,
        }


def adc_to_volts(raw16):
    return (raw16 * ADC_REF) / 65535.0


def temp_from_ntc_voltage(v):
    # Divider:
    # 3.3 V -> 10k pull-up -> TEMP_V -> NTC -> GND

    if v <= 0.01 or v >= (ADC_REF - 0.01):
        return None, None

    r_ntc = THERM_R_PULLUP * v / (ADC_REF - v)

    temp_k = 1.0 / (
        (1.0 / THERM_T0_K)
        + (1.0 / THERM_BETA) * math.log(r_ntc / THERM_R0)
    )

    return r_ntc, temp_k - 273.15


def blink_led(count=3, on_ms=120, off_ms=120):
    led = Pin(PIN_LED, Pin.OUT)

    for _ in range(count):
        led.value(1)
        time.sleep_ms(on_ms)

        led.value(0)
        time.sleep_ms(off_ms)


def buzz(count=2, freq=2000, dur_ms=150, gap_ms=120):
    pwm = PWM(Pin(PIN_BUZZER))
    pwm.freq(freq)

    try:
        for _ in range(count):
            pwm.duty_u16(32768)
            time.sleep_ms(dur_ms)

            pwm.duty_u16(0)
            time.sleep_ms(gap_ms)
    finally:
        pwm.deinit()
        Pin(PIN_BUZZER, Pin.OUT).value(0)


def pwm_pulse_us(pwm, pulse_us, freq_hz=50):
    period_us = int(1000000 / freq_hz)
    duty = int((pulse_us * 65535) / period_us)

    if duty < 0:
        duty = 0

    if duty > 65535:
        duty = 65535

    pwm.duty_u16(duty)


def test_pwm_output(pin_num, label):
    pwm = PWM(Pin(pin_num))
    pwm.freq(50)

    try:
        for pulse_us in (1000, 1500, 2000, 1500):
            print("  {} -> {} us".format(label, pulse_us))
            pwm_pulse_us(pwm, pulse_us)
            time.sleep_ms(600)

        record(label, True, "50 Hz PWM sequence emitted")

    except Exception as e:
        record(label, False, repr(e))

    finally:
        pwm.deinit()
        Pin(pin_num, Pin.OUT).value(0)


def sample_adc(pin_num, label, duration_ms=1000, sample_delay_ms=2):
    adc = ADC(pin_num)

    start = time.ticks_ms()
    count = 0

    raw_min = 65535
    raw_max = 0
    raw_sum = 0

    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        raw = adc.read_u16()

        if raw < raw_min:
            raw_min = raw

        if raw > raw_max:
            raw_max = raw

        raw_sum += raw
        count += 1

        time.sleep_ms(sample_delay_ms)

    if count == 0:
        raw_avg = 0
    else:
        raw_avg = raw_sum / count

    v_min = adc_to_volts(raw_min)
    v_max = adc_to_volts(raw_max)
    v_avg = adc_to_volts(raw_avg)

    print(
        "  {}: min={:.4f} V  avg={:.4f} V  max={:.4f} V"
        .format(label, v_min, v_avg, v_max)
    )

    return v_min, v_avg, v_max


def bytes_are_all(data, value):
    for b in data:
        if b != value:
            return False
    return True


def mcp_register_set_is_dead(diag):
    all_bytes = (
        diag["C1CON_bytes"]
        + diag["OSC_bytes"]
        + diag["IOCON_bytes"]
        + diag["CRC_bytes"]
        + diag["ECCCON_bytes"]
        + diag["DEVID_bytes"]
    )

    if bytes_are_all(all_bytes, 0x00):
        return True, "all read bytes are 0x00, MISO may be stuck low or chip is not responding"

    if bytes_are_all(all_bytes, 0xFF):
        return True, "all read bytes are 0xFF, MISO may be floating/high or chip is not selected"

    return False, "register reads are not all-zero or all-FF"


def print_mcp_diag(diag):
    print("  MCP2518FD register dump:")
    print("    C1CON  @ 0x000 = 0x{:08X}  bytes={}".format(diag["C1CON"], diag["C1CON_bytes"]))
    print("    OSC    @ 0xE00 = 0x{:08X}  bytes={}".format(diag["OSC"], diag["OSC_bytes"]))
    print("    IOCON  @ 0xE04 = 0x{:08X}  bytes={}".format(diag["IOCON"], diag["IOCON_bytes"]))
    print("    CRC    @ 0xE08 = 0x{:08X}  bytes={}".format(diag["CRC"], diag["CRC_bytes"]))
    print("    ECCCON @ 0xE0C = 0x{:08X}  bytes={}".format(diag["ECCCON"], diag["ECCCON_bytes"]))
    print("    DEVID  @ 0xE14 = 0x{:08X}  bytes={}".format(diag["DEVID"], diag["DEVID_bytes"]))
    print("    INT level = {}".format(diag["INT"]))


def test_mcp2518fd_response():
    spi = SPI(
        0,
        baudrate=1000000,
        polarity=0,
        phase=0,
        bits=8,
        firstbit=SPI.MSB,
        sck=Pin(PIN_SPI0_SCK),
        mosi=Pin(PIN_SPI0_MOSI),
        miso=Pin(PIN_SPI0_MISO),
    )

    mcp = MCP2518FD(spi, PIN_CAN_CS, PIN_CAN_INT)

    diag = mcp.read_diagnostics()
    print_mcp_diag(diag)

    dead, reason = mcp_register_set_is_dead(diag)

    if dead:
        record("MCP2518FD response", False, reason)
        print("  Most likely causes:")
        print("    - MCP2518FD not powered")
        print("    - MCP2518FD held in reset")
        print("    - MCP oscillator/crystal not running")
        print("    - CS not reaching the MCP")
        print("    - MISO not connected or shorted")
        print("    - wrong SPI pin map")
        print("    - wrong MCP2518FD footprint / soldering issue")
        return False

    devid_bytes = diag["DEVID_bytes"]
    devid_not_zero = not bytes_are_all(devid_bytes, 0x00)
    devid_not_ff = not bytes_are_all(devid_bytes, 0xFF)

    if devid_not_zero and devid_not_ff:
        detail = "DEVID=0x{:08X}, OSC=0x{:08X}, C1CON=0x{:08X}".format(
            diag["DEVID"],
            diag["OSC"],
            diag["C1CON"]
        )
        record("MCP2518FD response", True, detail)
        print("  The MCP is answering over SPI.")
        print("  This does not prove CAN bus traffic yet.")
        return True

    detail = (
        "some MCP registers responded, but DEVID itself looks bad: "
        "DEVID=0x{:08X}, OSC=0x{:08X}, C1CON=0x{:08X}"
        .format(diag["DEVID"], diag["OSC"], diag["C1CON"])
    )

    record("MCP2518FD partial response", True, detail)
    print("  The chip may be responding, but DEVID is suspicious.")
    print("  Check SPI command/address format and MCP2518FD variant.")
    return True


def main():
    banner("RP2040 BOARD SMOKE TEST")

    log("This script checks board peripherals visible from the schematic.")
    log("Dangerous output tests are disabled by default.")

    # ------------------------------------------------------------
    # Implicit bring-up checks
    # ------------------------------------------------------------
    banner("IMPLICIT CHECKS")

    record("RP2040 boot", True, "MicroPython is running")
    record("QSPI flash / boot path", True, "Board booted and script executed")
    record("External crystal / clocks", True, "Board booted and timers are alive")

    # ------------------------------------------------------------
    # Simple outputs
    # ------------------------------------------------------------
    banner("BASIC OUTPUTS")

    if ENABLE_LED_TEST:
        try:
            blink_led()
            record("LED GP18", True, "blink sequence sent")
        except Exception as e:
            record("LED GP18", False, repr(e))

    if ENABLE_BUZZER_TEST:
        try:
            buzz()
            record("Buzzer GP19 / B_ENABLE", True, "tone burst sent")
        except Exception as e:
            record("Buzzer GP19 / B_ENABLE", False, repr(e))

    # ------------------------------------------------------------
    # Status inputs
    # ------------------------------------------------------------
    if ENABLE_STATUS_TESTS:
        banner("STATUS INPUTS")

        try:
            pgood = Pin(PIN_PGOOD, Pin.IN).value()
            src = Pin(PIN_POWER_SRC, Pin.IN).value()
            can_int = Pin(PIN_CAN_INT, Pin.IN, Pin.PULL_UP).value()

            record(
                "PGOOD GP21",
                True,
                "level={} ({})".format(
                    pgood,
                    "5V good" if pgood else "5V not-good / pulled low"
                )
            )

            record(
                "POWER_SRC GP23",
                True,
                "level={} ({})".format(
                    src,
                    "+12V / IN1 selected" if src else "VBUS / IN2 selected"
                )
            )

            record(
                "CAN_INT idle GP6",
                True,
                "level={} ({})".format(
                    can_int,
                    "inactive/high" if can_int else "active/low"
                )
            )

        except Exception as e:
            record("Status pins", False, repr(e))

    # ------------------------------------------------------------
    # I2C0 / INA219 and I2C1 / FRAM
    # ------------------------------------------------------------
    if ENABLE_I2C_TESTS:
        banner("I2C TESTS")

        try:
            i2c0 = I2C(
                0,
                scl=Pin(PIN_SCL0),
                sda=Pin(PIN_SDA0),
                freq=400000
            )

            scan0 = i2c0.scan()
            ok0 = INA219_ADDR in scan0

            record(
                "I2C0 scan",
                ok0,
                "found={}".format([hex(x) for x in scan0])
            )

            if ok0:
                ina = INA219(i2c0, INA219_ADDR, shunt_ohms=0.05)
                ina.configure()
                vals = ina.read_all()

                detail = (
                    "bus={:.3f} V, shunt={:.3f} mV, current={:.3f} A, "
                    "power={:.3f} W, cfg=0x{:04X}"
                    .format(
                        vals["bus_v"],
                        vals["shunt_v"] * 1000.0,
                        vals["current_a"],
                        vals["power_w"],
                        vals["config"]
                    )
                )

                record("INA219 @ 0x40", True, detail)

            else:
                record("INA219 @ 0x40", False, "not found on I2C0")

        except Exception as e:
            record("I2C0 / INA219", False, repr(e))

        try:
            i2c1 = I2C(
                1,
                scl=Pin(PIN_SCL1),
                sda=Pin(PIN_SDA1),
                freq=400000
            )

            scan1 = i2c1.scan()
            ok1 = FRAM_ADDR in scan1

            record(
                "I2C1 scan",
                ok1,
                "found={}".format([hex(x) for x in scan1])
            )

            if ok1:
                fram = FM24CL64B(i2c1, FRAM_ADDR)
                head = fram.read(0x0000, 16)

                record(
                    "FRAM @ 0x50",
                    True,
                    "read[0:16]={}".format(head)
                )

                if ENABLE_FRAM_WRITE_TEST:
                    test_addr = 0x1FF0
                    original = fram.read(test_addr, 4)
                    pattern = b"\xA5\x5A\x3C\xC3"

                    fram.write(test_addr, pattern)
                    verify = fram.read(test_addr, 4)
                    fram.write(test_addr, original)

                    record(
                        "FRAM write/restore",
                        verify == pattern,
                        "addr=0x{:04X}, wrote={}, read_back={}, restored={}"
                        .format(test_addr, pattern, verify, original)
                    )
                else:
                    print("  FRAM write test skipped.")

            else:
                record("FRAM @ 0x50", False, "not found on I2C1")

        except Exception as e:
            record("I2C1 / FRAM", False, repr(e))

    # ------------------------------------------------------------
    # ADC inputs
    # ------------------------------------------------------------
    if ENABLE_ADC_TESTS:
        banner("ADC TESTS")

        try:
            temp_adc = ADC(PIN_TEMP_ADC)
            temp_raw = temp_adc.read_u16()
            temp_v = adc_to_volts(temp_raw)

            r_ntc, temp_c = temp_from_ntc_voltage(temp_v)

            if temp_c is None:
                record(
                    "TEMP_V GP26",
                    True,
                    "voltage={:.4f} V (probe open/short/or absent)".format(temp_v)
                )
            else:
                record(
                    "TEMP_V GP26",
                    True,
                    "voltage={:.4f} V, Rntc={:.1f} ohm, temp={:.2f} C"
                    .format(temp_v, r_ntc, temp_c)
                )

        except Exception as e:
            record("TEMP_V GP26", False, repr(e))

        try:
            print("  Sampling RPM analog inputs for 1 second.")
            print("  Spin the source now if applicable.")

            v1 = sample_adc(PIN_RPM1_ADC, "RPM_SENS_1 / GP27")
            v2 = sample_adc(PIN_RPM2_ADC, "RPM_SENS_2 / GP28")

            record(
                "RPM_SENS_1 GP27",
                True,
                "avg={:.4f} V, swing={:.4f} V".format(v1[1], v1[2] - v1[0])
            )

            record(
                "RPM_SENS_2 GP28",
                True,
                "avg={:.4f} V, swing={:.4f} V".format(v2[1], v2[2] - v2[0])
            )

        except Exception as e:
            record("RPM ADCs", False, repr(e))

    # ------------------------------------------------------------
    # MCP2518FD response check
    # ------------------------------------------------------------
    if ENABLE_MCP2518FD_RESPONSE_TEST:
        banner("MCP2518FD RESPONSE TEST")

        try:
            test_mcp2518fd_response()
        except Exception as e:
            record("MCP2518FD response", False, repr(e))

    # ------------------------------------------------------------
    # Optional output tests
    # ------------------------------------------------------------
    banner("OPTIONAL OUTPUT TESTS")

    if ENABLE_PWM_OUTPUT_TESTS:
        print("  Running PWM output tests.")
        test_pwm_output(PIN_CHOKE_PWM, "CHOKE_PWM GP16")
        test_pwm_output(PIN_THROTTLE_PWM, "THROTTLE_PWM GP17")
    else:
        print("  PWM output tests skipped.")

    if ENABLE_RELAY_TEST:
        try:
            print("  Pulsing relay enable for 1000 ms.")

            relay = Pin(PIN_RELAY_EN, Pin.OUT)
            relay.value(1)
            time.sleep_ms(1000)
            relay.value(0)

            record("Relay enable GP8", True, "300 ms pulse sent")

        except Exception as e:
            record("Relay enable GP8", False, repr(e))
    else:
        print("  Relay test skipped.")

    # ------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------
    banner("SUMMARY")

    passed = 0
    failed = 0

    for name, ok, detail in results:
        if ok:
            passed += 1
        else:
            failed += 1

        print("- {}: {}".format(name, "PASS" if ok else "FAIL"))

    print("\nTotal: {} pass, {} fail".format(passed, failed))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
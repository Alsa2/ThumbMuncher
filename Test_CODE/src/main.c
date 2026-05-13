#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "board_config.h"

#define INA219_ADDR                  0x40u

#define INA219_REG_CONFIG            0x00u
#define INA219_REG_SHUNT_VOLTAGE     0x01u
#define INA219_REG_BUS_VOLTAGE       0x02u

#define INA219_CONFIG_VALUE          0x3FFFu

#define INA219_SHUNT_LSB_V           0.000010f
#define INA219_BUS_LSB_V             0.004f
#define SHUNT_RESISTOR_OHM           0.050f

#define I2C_BAUD_HZ                  100000u
#define I2C_TIMEOUT_US               5000u
#define PRINT_PERIOD_MS              500u
#define ADC_AVG_N                    32u

#define ADC0_SCALE                   1.0f
#define ADC1_SCALE                   1.0f

static bool i2c_read_reg16(uint8_t addr, uint8_t reg, uint16_t *out)
{
    int wr = i2c_write_timeout_us(SENSOR_I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (wr != 1) {
        return false;
    }

    uint8_t rx[2] = {0, 0};
    int rd = i2c_read_timeout_us(SENSOR_I2C_PORT, addr, rx, 2, false, I2C_TIMEOUT_US);
    if (rd != 2) {
        return false;
    }

    *out = ((uint16_t)rx[0] << 8) | rx[1];
    return true;
}

static bool i2c_write_reg16(uint8_t addr, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xffu)
    };

    int wr = i2c_write_timeout_us(SENSOR_I2C_PORT, addr, tx, 3, false, I2C_TIMEOUT_US);
    return wr == 3;
}

static void i2c_bus_recover(void)
{
    i2c_deinit(SENSOR_I2C_PORT);

    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_SIO);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
    gpio_put(PIN_I2C_SCL, 1);
    sleep_us(10);

    for (int i = 0; i < 16; i++) {
        gpio_put(PIN_I2C_SCL, 0);
        sleep_us(10);
        gpio_put(PIN_I2C_SCL, 1);
        sleep_us(10);
    }

    gpio_set_dir(PIN_I2C_SDA, GPIO_OUT);
    gpio_put(PIN_I2C_SDA, 0);
    sleep_us(10);
    gpio_put(PIN_I2C_SCL, 1);
    sleep_us(10);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    sleep_us(10);
}

static void i2c_test_init(void)
{
    i2c_bus_recover();
    i2c_init(SENSOR_I2C_PORT, I2C_BAUD_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    sleep_ms(20);
}

static void scan_i2c(void)
{
    printf("I2C scan:");
    int found = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy = 0;
        int rd = i2c_read_timeout_us(SENSOR_I2C_PORT, addr, &dummy, 1, false, I2C_TIMEOUT_US);
        if (rd == 1) {
            printf(" 0x%02x", addr);
            found++;
        }
    }

    if (found == 0) {
        printf(" none");
    }
    printf("\n");
}

static uint16_t adc_read_avg(uint input)
{
    uint32_t sum = 0;
    adc_select_input(input);

    for (uint i = 0; i < ADC_AVG_N; i++) {
        sum += adc_read();
        sleep_us(50);
    }

    return (uint16_t)(sum / ADC_AVG_N);
}

static float adc_raw_to_v(uint16_t raw)
{
    return ((float)raw * ADC_VREF) / ADC_COUNTS_MAX;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    gpio_init(PIN_RELAY);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);
    gpio_put(PIN_RELAY, 0);

#ifdef PIN_STARTER
    gpio_init(PIN_STARTER);
    gpio_set_dir(PIN_STARTER, GPIO_OUT);
    gpio_put(PIN_STARTER, 0);
#endif

    adc_init();
    adc_gpio_init(PIN_TEMP_ADC_GPIO);
    adc_gpio_init(PIN_HALL_ADC_GPIO);

    printf("\n--- INA219 relay + voltage/current diagnostic ---\n");
    printf("Relay GPIO: %u\n", PIN_RELAY);
    printf("I2C port: %s | SDA GPIO %u | SCL GPIO %u | baud %u\n",
           SENSOR_I2C_PORT == i2c0 ? "i2c0" : "i2c1",
           PIN_I2C_SDA, PIN_I2C_SCL, I2C_BAUD_HZ);
    printf("INA219 address: 0x%02x | shunt: %.4f ohm\n", INA219_ADDR, SHUNT_RESISTOR_OHM);
    printf("Commands: 1=relay ON, 0=relay OFF, t=toggle, s=rescan, r=recover I2C\n\n");

    i2c_test_init();
    scan_i2c();

    uint16_t cfg = 0;
    if (i2c_write_reg16(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG_VALUE)) {
        sleep_ms(20);
        bool ok_cfg = i2c_read_reg16(INA219_ADDR, INA219_REG_CONFIG, &cfg);
        printf("INA219 config write ok, readback %s 0x%04x\n", ok_cfg ? "ok" : "fail", cfg);
    } else {
        printf("INA219 config write failed at 0x%02x\n", INA219_ADDR);
    }

    bool relay_on = true;
    gpio_put(PIN_RELAY, 1);
    printf("Relay turned ON.\n\n");

    uint32_t last_print = 0;

    while (true) {
        int c = getchar_timeout_us(0);

        if (c == '1') {
            relay_on = true;
            gpio_put(PIN_RELAY, 1);
            printf("relay = ON\n");
        } else if (c == '0') {
            relay_on = false;
            gpio_put(PIN_RELAY, 0);
            printf("relay = OFF\n");
        } else if (c == 't' || c == 'T') {
            relay_on = !relay_on;
            gpio_put(PIN_RELAY, relay_on ? 1 : 0);
            printf("relay = %s\n", relay_on ? "ON" : "OFF");
        } else if (c == 's' || c == 'S') {
            scan_i2c();
        } else if (c == 'r' || c == 'R') {
            printf("Recovering I2C bus...\n");
            i2c_test_init();
            scan_i2c();
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_print) >= PRINT_PERIOD_MS) {
            last_print = now;
            gpio_put(PIN_LED, relay_on ? 1 : 0);

            uint16_t cfg_raw = 0;
            uint16_t bus_raw = 0;
            uint16_t shunt_raw_u = 0;

            bool ok_cfg = i2c_read_reg16(INA219_ADDR, INA219_REG_CONFIG, &cfg_raw);
            bool ok_bus = i2c_read_reg16(INA219_ADDR, INA219_REG_BUS_VOLTAGE, &bus_raw);
            bool ok_shunt = i2c_read_reg16(INA219_ADDR, INA219_REG_SHUNT_VOLTAGE, &shunt_raw_u);

            int16_t shunt_raw = (int16_t)shunt_raw_u;
            uint16_t bus_code = bus_raw >> 3;
            bool bus_conversion_ready = (bus_raw & 0x0002u) != 0u;
            bool bus_overflow = (bus_raw & 0x0001u) != 0u;

            float bus_v = (float)bus_code * INA219_BUS_LSB_V;
            float shunt_v = (float)shunt_raw * INA219_SHUNT_LSB_V;
            float current_a = shunt_v / SHUNT_RESISTOR_OHM;

            uint16_t adc0_raw = adc_read_avg(TEMP_ADC_INPUT);
            uint16_t adc1_raw = adc_read_avg(HALL_ADC_INPUT);
            float adc0_v = adc_raw_to_v(adc0_raw);
            float adc1_v = adc_raw_to_v(adc1_raw);

            printf("t=%lu ms | relay=%d | SDA=%d SCL=%d | INA219=%s | cfg=%s 0x%04x | bus=%s raw=0x%04x code=%u %.4f V cnvr=%d ovf=%d | shunt=%s raw=%d %.3f mV | current=%.4f A | ADC0=%u %.4f V scaled=%.4f | ADC1=%u %.4f V scaled=%.4f\n",
                   now,
                   relay_on ? 1 : 0,
                   gpio_get(PIN_I2C_SDA),
                   gpio_get(PIN_I2C_SCL),
                   (ok_cfg && ok_bus && ok_shunt) ? "ok" : "fail",
                   ok_cfg ? "ok" : "--", cfg_raw,
                   ok_bus ? "ok" : "--", bus_raw, bus_code, bus_v,
                   bus_conversion_ready ? 1 : 0,
                   bus_overflow ? 1 : 0,
                   ok_shunt ? "ok" : "--", shunt_raw, shunt_v * 1000.0f,
                   current_a,
                   adc0_raw, adc0_v, adc0_v * ADC0_SCALE,
                   adc1_raw, adc1_v, adc1_v * ADC1_SCALE);
        }

        sleep_ms(10);
    }
}
/*

**Integration Test 2.4 - Completed on 13/04/25:

- CAN data TX/RX Test.
- The target speed could be sent in an intriger number (range 0 - 15, the first byte with index 0
- The digital pot could covert the speed into a value from 0 - 255 (0 - 5 V, when VCC = 5V)
- RX ID - 124, TX ID - 126

*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "mcp2515/mcp2515.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include <cstring>
#include <QuadratureEncoder.hpp>
#include "hardware/adc.h"
#include "hardware/i2c.h"

#define SDA_PIN 6
#define SCL_PIN 7
#define DS3502_ADDR 0x28 // A0/A1 = GND -> 0101 000 = 0x28

float read_voltage()
{
    adc_select_input(0);       // GPIO26
    uint16_t raw = adc_read(); // 0-4095
    return raw * 3.3f / 4095.0f;
}


void ds3502_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(&i2c1_inst, DS3502_ADDR, buf, 2, false);
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
        sleep_ms(10);
    }
    // init_spi();
    adc_init();

    i2c_init(&i2c1_inst, 400 * 1000); // 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    printf("I2C Init Completed!\n");

    printf("Scanning I2C...\n");

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_blocking(&i2c1_inst, addr, &dummy, 1, false);
        if (ret >= 0) {
            printf("Device found at 0x%02X\n", addr);
        }
        sleep_ms(10); 
    }

    printf("Done\n");

    // // sleep_ms(100);
    // ds3502_write(0x02, 0x00);
    // printf("Write Successfully!\n");
    // // sleep_ms(10)
    // ds3502_write(0x00, 0x00);
    // printf("Write Successfully!\n");
    // // sleep_ms(10)
    ds3502_write(0x02, 0x80);
    // printf("Write Successfully!\n");
    // // sleep_ms(10);
    ds3502_write(0x00, 0x40);
    // printf("Write Successfully!\n");
    // sleep_ms(10);
    // uint8_t wiper_val = 0;
    // i2c_read_blocking(I2C_PORT, 0x28, &wiper_val, 1, false);
    // sleep_ms(10);
    //
        // uint8_t reg = 0x00;
        // uint8_t val = 0;
        // //i2c_write_blocking(i2c0, 0x28, &reg, 1, true);
        // i2c_read_blocking(i2c0, 0x28, &val, 1, false);
        // printf("Read WIPER: 0x%02X\n", val);
    // printf("60 Value setting Completed!\n");

    // Listen loop
    while (true)
    {
        sleep_ms(1000);
        float voltage0 = read_voltage(); // A0
        printf("Voltage: %f. \n", voltage0);

        // printf("Init Complete\n");
        // tight_loop_contents(); // Low-power mode in main-loop
    }

    return 0;
}
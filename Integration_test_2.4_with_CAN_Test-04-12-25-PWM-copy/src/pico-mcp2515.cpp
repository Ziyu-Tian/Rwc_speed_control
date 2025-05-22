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
#include "hardware/pwm.h"


// CAN Config
MCP2515 can0;
struct can_frame rx;
struct can_frame tx_frame;
volatile uint8_t received_value = 0; // received CAN data

// Pin_A (Pin_B would be set as A_pin + 1)
#define A_pin 23  // Noted that A and B should be swapped in physical connection
#define A2_pin 21 // 12
#define A3_pin 19 //
#define ppr 600.0 // PPR
#define sampling_time 10e-3 // Sampling Time for Speed Calculation -10ms
int latest_rpm = 0;                  // Global latest_rpm
bool new_rpm_available = false;      // new_rpm_calculated_flag
volatile bool can_send_flag = false; // can_send_flag



void send_can_data()
{
    struct can_frame tx_frame;
    tx_frame.can_id = 0x126; // TX CAN ID
    tx_frame.can_dlc = 8;    

    //latest_rpm = latest_rpm + 1 % 400;

    //uint8_t *latest_rpm_bytes = (uint8_t *)&latest_rpm;

    //tx_frame.data = {latest_rpm_bytes[0], latest_rpm_bytes[1], latest_rpm_bytes[2], latest_rpm_bytes[4]};

    memset(tx_frame.data, 0, sizeof(tx_frame.data)); 


    memcpy(tx_frame.data, &latest_rpm, sizeof(int)); 

    // Send Message
    if (can0.sendMessage(&tx_frame) == MCP2515::ERROR_OK)
    {
        printf("\e[1;1H\e[2J");
        printf("CAN Successful! ID=0x%X\n", tx_frame.can_id);
        printf("Core 1 - RPM: %d\n", latest_rpm);
    }
    else
    {
        //printf("CAN Failed!\n");
    }
}

bool timerCallback(struct repeating_timer *t)
{
    can_send_flag = true; // Set send_flag
    return true;          // continue timer
}

void core0_entry()
{
    // Initialize PIO_Encoder Class
    QuadratureEncoder encoder(pio1, A_pin, ppr);
    // QuadratureEncoder encoder_2(pio0, A2_pin, ppr);
    // QuadratureEncoder encoder_3(pio1, A3_pin, ppr);

    while (true)
    {
        encoder.update(sampling_time); // Initialize starting time
        // encoder_2.update(sampling_time); // Initialize starting time 
        // encoder_3.update(sampling_time); // Initialize starting time 

        // Position: return angle (default radium)
        // Velocity: return velocity in rad/s
        // Counter: return counting number (CW++, ACW--, 1200 per round)
        auto position = encoder.get_position();
        auto velocity = encoder.get_velocity();
        auto counter = encoder.get_count();

        // auto position_2 = encoder_2.get_position();
        // auto velocity_2 = encoder_2.get_velocity();
        // auto counter_2 = encoder_2.get_count();

        // auto position_3 = encoder_3.get_position();
        // auto velocity_3 = encoder_3.get_velocity();
        // auto counter_3  = encoder_3.get_count();

        int rpm = (30 * velocity) / M_PI;
        
        // int rpm_2 = (30 * velocity_2) / M_PI;
        // int rpm_3 = (30 * velocity_3) / M_PI;
        
        // rpm = (rpm + rpm_2 + rpm_3)/2;

        uint32_t rpm_data;
        //printf("Core 1 - RPM: %d\n", rpm);
        memcpy(&rpm_data, &rpm, sizeof(int));   // change to 32 bits
        multicore_fifo_push_blocking(rpm_data); // TX RPM data

        sleep_ms(10); // Send frequency
    }
}


void core1_entry()
{
    while (true)
    {

        if (multicore_fifo_rvalid())
        {
            uint32_t rpm_data = multicore_fifo_pop_blocking(); // FIFO Read
            memcpy(&latest_rpm, &rpm_data, sizeof(int));       
            new_rpm_available = true;                          // new_rpm already being calculated
        }

        if (can_send_flag)
        {
            can_send_flag = false; // Clear flag
            send_can_data();       // CAN data send function
        }
    }
}



int main()
{
    stdio_init_all();

    // Initialize interface
    can0.reset();
    can0.setBitrate(CAN_50KBPS, MCP_16MHZ);
    // can0.setNormalMode();

    // Config Mode
    can0.setConfigMode();

    // Mask0 & Mask1 All 11-bit need to be same
    can0.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    can0.setFilterMask(MCP2515::MASK1, false, 0x7FF);

    // Filter（Only ID 0x126）
    can0.setFilter(MCP2515::RXF0, false, 0x124);
    can0.setFilter(MCP2515::RXF1, false, 0x124);
    can0.setFilter(MCP2515::RXF2, false, 0x124);
    can0.setFilter(MCP2515::RXF3, false, 0x124);
    can0.setFilter(MCP2515::RXF4, false, 0x124);
    can0.setFilter(MCP2515::RXF5, false, 0x124);

    // Change to normal mode
    can0.setNormalMode();

    struct repeating_timer timer;
    add_repeating_timer_ms(60, timerCallback, NULL, &timer); // 100 ms timer for CAN

    multicore_launch_core1(core1_entry); // Core-1
    core0_entry();                       // Core 0

    // Listen loop
    while (true)
    {
        tight_loop_contents(); // Low-power mode in main-loop
    }

    return 0;
}
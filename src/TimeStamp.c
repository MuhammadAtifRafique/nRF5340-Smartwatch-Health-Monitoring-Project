#include <Zephyr/kernel.h>
#include <stdio.h>
#include <stdint.h>

#define for_micro_second 1 // For micro second time stamp
#define for_milli_second 1000 // For micro second time stamp

void init_timeStamp(void)
{
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;  // Set the TIMER mode to Timer Mode
    NRF_TIMER0->PRESCALER = 4;                // Set the prescaler to 2^4 (16)
    NRF_TIMER0->TASKS_CLEAR = 1;              // Clear the timer
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit; // 32-bit timer mode
    NRF_TIMER0->TASKS_START = 1;             // Start the timer
} // End of void timer_init(void)
///////| (34) |///////////
uint32_t get_uTimeStamp(void)
{
    NRF_TIMER0->TASKS_CAPTURE[1] = 1;  // Trigger a capture event on CC[1]
    return NRF_TIMER0->CC[1]/for_micro_second;  // Read the captured value from CC[1]
} // End of uint32_t get_timestamp(void)
uint32_t get_mTimeStamp(void)
{
    NRF_TIMER0->TASKS_CAPTURE[1] = 1;  // Trigger a capture event on CC[1]
    return NRF_TIMER0->CC[1]/for_milli_second;  // Read the captured value from CC[1]
} // End of uint32_t get_timestamp(void)
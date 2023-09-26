/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab 5 3.1
 * Date: 2023-09-24
 * ----------------------------------------------------------------------------
 */

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.c"
#include "driverlib/pin_map.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Global values for skipping blinking
int g_led1_skip = 0;
int g_led2_skip = 0;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// The error routine that is called if the driver library
// encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
//=============================================================================
// Blink led 1 every 1s
void first_led(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    int led1_state = 0;
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 0.5s which gives a blink period of 1s
    const TickType_t wait_time = pdMS_TO_TICKS(500);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Delay 0.5s
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        // Switch state every 0.5s, to keep synchronous even when LED is turned on for 10s by button click
        led1_state = !led1_state;
        // If button has been clicked, skip actually toggling LED
        if(g_led1_skip == 0)
        {
            // Toggle led 1
            LEDWrite(1, !led1_state);
            // Print action on serial
            if (!led1_state)
            {
                UARTprintf("\nLED 1 ON\n");
            }
            else
            {
                UARTprintf("\nLED 1 OFF\n");
            }
        }
    }
}
//=============================================================================
// Blink led 2 every 2s
void second_led(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    int led2_state = 0;
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 1s which gives a blink period of 2s
    const TickType_t wait_time = pdMS_TO_TICKS(1000);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Delay 1s
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        // Switch state every 1s, to keep synchronous even when LED is turned on for 10s by button click
        led2_state = !led2_state;
        // If button has been clicked, skip actually toggling LED
        if(g_led2_skip == 0)
        {
            // Toggle led 2
            LEDWrite(2, ((!led2_state) * 2));
            // Print action on serial
            if (!led2_state)
            {
                UARTprintf("\nLED 2 ON\n");
            }
            else
            {
                UARTprintf("\nLED 2 OFF\n");
            }
        }
    }
}
//=============================================================================
// Blink led 3 every 3s
void third_led(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    int led3_state = 0;
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 1.5s which gives a blink period of 3s
    const TickType_t wait_time = pdMS_TO_TICKS(1500);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Delay 1.5s
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        led3_state = !led3_state;
        // Toggle led 3
        LEDWrite(4, ((!led3_state) * 4));
        // Print action on serial
        if (!led3_state)
        {
            UARTprintf("\nLED 3 ON\n");
        }
        else
        {
            UARTprintf("\nLED 3 OFF\n");
        }
    }
}
//=============================================================================
// Blink led 4 every 4s
void fourth_led(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    int led4_state = 0;
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 2s which gives a blink period of 4s
    const TickType_t wait_time = pdMS_TO_TICKS(2000);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Delay 2s
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        led4_state = !led4_state;
        // Toggle led 4
        LEDWrite(8, ((!led4_state) * 8));
        // Print action on serial
        if(!led4_state)
        {
            UARTprintf("\nLED 4 ON\n");
        }
        else
        {
            UARTprintf("\nLED 4 OFF\n");
        }
    }
}
//=============================================================================
// LED1 timer callback function
static void led1_timer_callback(TimerHandle_t xTimer)
{
    // 10 ms has passed, stop skipping toggling of led1
    g_led1_skip = 0;
}
//=============================================================================
// Turn on LED 1 for 10s once left button is pressed
void first_button(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    unsigned char ucDelta, ucState;
    // ms to wait, 10s
    const TickType_t wait_time = pdMS_TO_TICKS(10000);
    TimerHandle_t led1_timer;
    // Create 10ms timer
    // pdFALSE makes the timer non periodic
    led1_timer = xTimerCreate(( const char * ) "LED 1 timer", wait_time, pdFALSE, ( void * ) 0, led1_timer_callback);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        // LEFT button turns on first LED for 10 sec
        if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta) || BUTTON_RELEASED(LEFT_BUTTON, ucState, ucDelta))
        {
            // Skip toggling LED 1
            g_led1_skip = 1;
            // Turn on led 1.
            LEDWrite(1, 1);
            // Start 10ms timer
            xTimerStart(led1_timer, 0 );
            // Print action on serial
            UARTprintf("\nLED 1 ON\n");
        }
    }
}
//=============================================================================
// LED2 timer callback function
static void led2_timer_callback(TimerHandle_t xTimer)
{
    // 10 ms has passed, stop skipping toggling of led2
    g_led2_skip = 0;
}
//=============================================================================
// Turn on LED 2 for 10s once right button is pressed
void second_button(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    unsigned char ucDelta, ucState;
    // ms to wait, 10s
    const TickType_t wait_time = pdMS_TO_TICKS(10000);
    TimerHandle_t led2_timer;
    // Create 10ms timer
    // pdFALSE makes the timer non periodic
    led2_timer = xTimerCreate((const char*) "LED 2 timer", wait_time, pdFALSE, (void*) 0, led2_timer_callback);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        // Right button turns on second LED for 10 sec
        if (BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta) || BUTTON_RELEASED(RIGHT_BUTTON, ucState, ucDelta))
        {
            // Skip toggling LED 2
            g_led2_skip = 1;
            // Turn on led 2.
            LEDWrite(2, 2);
            // Start 10ms timer
            xTimerStart(led2_timer, 0 );
            // Print action on serial
            UARTprintf("\nLED 2 ON\n");
        }
    }
}
//=============================================================================
// Initialize LEDs
void init_led()
{
    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Check if the peripheral access is enabled.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
        ;
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
        ;
    }

    // Configure the device pins.
    PinoutSet(true, false);

    // Enable the GPIO pin for the LEDs (D2 on PN0, D1 on PN1, D4 on PF0, D3 on PF4).
    // Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
}
//=============================================================================
// Configure the UART.
void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}
//=============================================================================
// Main Function
int main(void)
{
    //-----------------------------------------------------------------------------
    // Run from the PLL at 120 MHz (because this is what FreeRTOS sets in it's config file).
    uint32_t systemClock;
    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Handles for the different tasks
    TaskHandle_t xHandle_led1 = NULL;
    TaskHandle_t xHandle_led2 = NULL;
    TaskHandle_t xHandle_led3 = NULL;
    TaskHandle_t xHandle_led4 = NULL;
    TaskHandle_t xHandle_button1 = NULL;
    TaskHandle_t xHandle_button2 = NULL;

    // Return values for the different tasks
    BaseType_t xReturned_led1;
    BaseType_t xReturned_led2;
    BaseType_t xReturned_led3;
    BaseType_t xReturned_led4;
    BaseType_t xReturned_button1;
    BaseType_t xReturned_button2;

    UBaseType_t led_priority = 3;
    UBaseType_t button_priority = 3;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");

    // Initialize LEDs
    init_led();

    // Initialize the button driver.
    ButtonsInit();
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create the different tasks
    xReturned_led1 = xTaskCreate(first_led, "led 1 blink", 64, (void *)systemClock, led_priority, &xHandle_led1);
    xReturned_led2 = xTaskCreate(second_led, "led 2 blink", 64, (void *)systemClock, led_priority, &xHandle_led2);
    xReturned_led3 = xTaskCreate(third_led, "led 3 blink", 64, (void *)systemClock, led_priority, &xHandle_led3);
    xReturned_led4 = xTaskCreate(fourth_led, "led 4 blink", 64, (void *)systemClock, led_priority, &xHandle_led4);
    xReturned_button1 = xTaskCreate(first_button, "button 1 press", 64, (void *)systemClock, button_priority, &xHandle_button1);
    xReturned_button2 = xTaskCreate(second_button, "button 2 press", 64, (void *)systemClock, button_priority, &xHandle_button2);

    // Make sure all tasks could be created successfully
    if ((xReturned_led1 == pdPASS) && (xReturned_led2 == pdPASS) && (xReturned_led3 == pdPASS) && (xReturned_led4 == pdPASS) && (xReturned_button1 == pdPASS) && (xReturned_button2 == pdPASS))
    {
        // Start the scheduler
        vTaskStartScheduler();

        // vTaskStartScheduler() will only return if there is insufficient RTOS heap available to create the idle or timer daemon tasks.
        // Will not get here unless there is insufficient RAM.
        UARTprintf("\n Insufficient RAM\n");
    }
    // One or more tasks could not be created successfully because of memory
    UARTprintf("\n COULD NOT ALLOCATE REQUIRED MEMORY\n");
    //-----------------------------------------------------------------------------
}
//=============================================================================

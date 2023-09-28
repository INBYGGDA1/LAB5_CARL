/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab 5 5, measuring context switch time
 * Date: 2023-09-27
 * ----------------------------------------------------------------------------
 */

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>
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
#include "semphr.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
volatile TickType_t g_entering_time = 0;
volatile TickType_t g_entered_time = 0;
volatile TickType_t g_context_switch_enter = 0;

volatile TickType_t g_exiting_time = 0;
volatile TickType_t g_exited_time = 0;
volatile TickType_t g_context_switch_exit = 0;
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
// Does work
// Work time 30000000 (3*10^7) = 2500ms = 2.5s (roughly)
void workload_function(int work_time)
{
    //-----------------------------------------------------------------------------
    volatile int i = 0;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    for(i = 0; i<work_time ;i++)
    {
        ;
    }
    //-----------------------------------------------------------------------------
}
//=============================================================================
// Timer callback function
static void timer_callback_func(TimerHandle_t xTimer)
{
    // Take time when we just entered
    g_entered_time = xTaskGetTickCount();

    // Make sure value isn't an old value from last timer event
    if(g_entered_time >= g_entering_time)
    {
        // Calculate the context switch time for entering, in ticks
        g_context_switch_enter = g_entered_time - g_entering_time;
    }
    UARTprintf("\n Entering : %d \n", g_entering_time);
    UARTprintf("\n Entered : %d \n", g_entered_time);
    UARTprintf("\n In : %d \n", g_context_switch_enter);


    // Take time just before exiting
    g_exiting_time = xTaskGetTickCount();
}
//=============================================================================
// Task function with timer
// Measures context switching time when switching into and out off a timer function, and does some busy waiting ("work")
void task_func_timer(void *temp)
{
    //-----------------------------------------------------------------------------
    // Create 1 tick timer
    TimerHandle_t handle_timer;
    // pdFALSE makes the timer non periodic
    handle_timer = xTimerCreate((const char*) "timer", 1, pdFALSE, (void*) 0, timer_callback_func);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Take time just before entering, add one since the timer time must be greater than 0 and is thus 1 tick
        g_entering_time = xTaskGetTickCount() + 1;
        // Start 1 tick timer
        xTimerStart(handle_timer, 0);
        // A value of less than 10487 will result in g_exited_time = xTaskGetTickCount(); executing before the timer has gone off
        workload_function(11000);
        // Take time when we just exited
        g_exited_time = xTaskGetTickCount();

        // Make sure value isn't an old value from last timer event
        if(g_exited_time >= g_exiting_time)
        {
            // Calculate the context switch time for exiting, in ticks
            g_context_switch_exit = g_exited_time - g_exiting_time;
        }
        UARTprintf("\n Out : %d \n", g_context_switch_exit);

        // Do some work
        workload_function(10000000);
    }
}
//=============================================================================
// Task that just does some work
void task_func(void *temp)
{
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Do some work
        workload_function(10000000);
    }
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
    TaskHandle_t handle_task1 = NULL;
    TaskHandle_t handle_task2 = NULL;
    TaskHandle_t handle_task3 = NULL;
    TaskHandle_t handle_task4 = NULL;
    TaskHandle_t handle_task5 = NULL;
    TaskHandle_t handle_task6 = NULL;
    TaskHandle_t handle_task7 = NULL;
    TaskHandle_t handle_task8 = NULL;
    TaskHandle_t handle_task9 = NULL;
    TaskHandle_t handle_task10 = NULL;
    TaskHandle_t handle_task11 = NULL;
    TaskHandle_t handle_task12 = NULL;
    TaskHandle_t handle_task13 = NULL;
    TaskHandle_t handle_task14 = NULL;
    TaskHandle_t handle_task15 = NULL;
    TaskHandle_t handle_task16 = NULL;
    TaskHandle_t handle_task17 = NULL;
    TaskHandle_t handle_task18 = NULL;
    TaskHandle_t handle_task19 = NULL;
    TaskHandle_t handle_task20 = NULL;
    TaskHandle_t handle_task21 = NULL;
    TaskHandle_t handle_task22 = NULL;
    TaskHandle_t handle_task23 = NULL;
    TaskHandle_t handle_task24 = NULL;
    TaskHandle_t handle_task25 = NULL;
    TaskHandle_t handle_task26 = NULL;
    TaskHandle_t handle_task27 = NULL;
    TaskHandle_t handle_task28 = NULL;
    TaskHandle_t handle_task29 = NULL;
    TaskHandle_t handle_task30 = NULL;
    TaskHandle_t handle_task31 = NULL;
    TaskHandle_t handle_task32 = NULL;
    TaskHandle_t handle_task33 = NULL;
    TaskHandle_t handle_task34 = NULL;
    TaskHandle_t handle_task35 = NULL;
    TaskHandle_t handle_task36 = NULL;
    TaskHandle_t handle_task37 = NULL;
    TaskHandle_t handle_task38 = NULL;
    TaskHandle_t handle_task39 = NULL;
    TaskHandle_t handle_task40 = NULL;

    // Return values for the different tasks
    // Initially set to pdPASS to make switching easy, aka incase one or more tasks aren't created
    BaseType_t returned_task1 = pdPASS;
    BaseType_t returned_task2 = pdPASS;
    BaseType_t returned_task3 = pdPASS;
    BaseType_t returned_task4 = pdPASS;
    BaseType_t returned_task5 = pdPASS;
    BaseType_t returned_task6 = pdPASS;
    BaseType_t returned_task7 = pdPASS;
    BaseType_t returned_task8 = pdPASS;
    BaseType_t returned_task9 = pdPASS;
    BaseType_t returned_task10 = pdPASS;
    BaseType_t returned_task11 = pdPASS;
    BaseType_t returned_task12 = pdPASS;
    BaseType_t returned_task13 = pdPASS;
    BaseType_t returned_task14 = pdPASS;
    BaseType_t returned_task15 = pdPASS;
    BaseType_t returned_task16 = pdPASS;
    BaseType_t returned_task17 = pdPASS;
    BaseType_t returned_task18 = pdPASS;
    BaseType_t returned_task19 = pdPASS;
    BaseType_t returned_task20 = pdPASS;
    BaseType_t returned_task21 = pdPASS;
    BaseType_t returned_task22 = pdPASS;
    BaseType_t returned_task23 = pdPASS;
    BaseType_t returned_task24 = pdPASS;
    BaseType_t returned_task25 = pdPASS;
    BaseType_t returned_task26 = pdPASS;
    BaseType_t returned_task27 = pdPASS;
    BaseType_t returned_task28 = pdPASS;
    BaseType_t returned_task29 = pdPASS;
    BaseType_t returned_task30 = pdPASS;
    BaseType_t returned_task31 = pdPASS;
    BaseType_t returned_task32 = pdPASS;
    BaseType_t returned_task33 = pdPASS;
    BaseType_t returned_task34 = pdPASS;
    BaseType_t returned_task35 = pdPASS;
    BaseType_t returned_task36 = pdPASS;
    BaseType_t returned_task37 = pdPASS;
    BaseType_t returned_task38 = pdPASS;
    BaseType_t returned_task39 = pdPASS;
    BaseType_t returned_task40 = pdPASS;

    // Priorities of tasks
    UBaseType_t priority_low = 1;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");
    //-----------------------------------------------------------------------------


    //-----------------------------------------------------------------------------
    // Create the different tasks
    returned_task1 = xTaskCreate(task_func_timer, "task 1", 128, (void *)1, priority_low, &handle_task1);
    //returned_task2 = xTaskCreate(task_func, "task 2", 128, (void *)1, priority_low, &handle_task2);
    //returned_task3 = xTaskCreate(task_func, "task 3", 128, (void *)1, priority_low, &handle_task3);
    //returned_task4 = xTaskCreate(task_func, "task 4", 128, (void *)1, priority_low, &handle_task4);
    //returned_task5 = xTaskCreate(task_func, "task 5", 128, (void *)1, priority_low, &handle_task5);
    //returned_task6 = xTaskCreate(task_func, "task 6", 128, (void *)1, priority_low, &handle_task6);
    //returned_task7 = xTaskCreate(task_func, "task 7", 128, (void *)1, priority_low, &handle_task7);
    //returned_task8 = xTaskCreate(task_func, "task 8", 128, (void *)1, priority_low, &handle_task8);
    //returned_task9 = xTaskCreate(task_func, "task 9", 128, (void *)1, priority_low, &handle_task9);
    //returned_task10 = xTaskCreate(task_func, "task 10", 128, (void *)1, priority_low, &handle_task10);
    //returned_task11 = xTaskCreate(task_func, "task 11", 128, (void *)1, priority_low, &handle_task11);
    //returned_task12 = xTaskCreate(task_func, "task 12", 128, (void *)1, priority_low, &handle_task12);
    //returned_task13 = xTaskCreate(task_func, "task 13", 128, (void *)1, priority_low, &handle_task13);
    //returned_task14 = xTaskCreate(task_func, "task 14", 128, (void *)1, priority_low, &handle_task14);
    //returned_task15 = xTaskCreate(task_func, "task 15", 128, (void *)1, priority_low, &handle_task15);
    //returned_task16 = xTaskCreate(task_func, "task 16", 128, (void *)1, priority_low, &handle_task16);
    //returned_task17 = xTaskCreate(task_func, "task 17", 128, (void *)1, priority_low, &handle_task17);
    //returned_task18 = xTaskCreate(task_func, "task 18", 128, (void *)1, priority_low, &handle_task18);
    //returned_task19 = xTaskCreate(task_func, "task 19", 128, (void *)1, priority_low, &handle_task19);
    //returned_task20 = xTaskCreate(task_func, "task 20", 128, (void *)1, priority_low, &handle_task20);
    //returned_task21 = xTaskCreate(task_func, "task 21", 128, (void *)1, priority_low, &handle_task21);
    //returned_task22 = xTaskCreate(task_func, "task 22", 128, (void *)1, priority_low, &handle_task22);
    //returned_task23 = xTaskCreate(task_func, "task 23", 128, (void *)1, priority_low, &handle_task23);
    //returned_task24 = xTaskCreate(task_func, "task 24", 128, (void *)1, priority_low, &handle_task24);
    //returned_task25 = xTaskCreate(task_func, "task 25", 128, (void *)1, priority_low, &handle_task25);
    //returned_task26 = xTaskCreate(task_func, "task 26", 128, (void *)1, priority_low, &handle_task26);
    //returned_task27 = xTaskCreate(task_func, "task 27", 128, (void *)1, priority_low, &handle_task27);
    //returned_task28 = xTaskCreate(task_func, "task 28", 128, (void *)1, priority_low, &handle_task28);
    //returned_task29 = xTaskCreate(task_func, "task 29", 128, (void *)1, priority_low, &handle_task29);
    //returned_task30 = xTaskCreate(task_func, "task 30", 128, (void *)1, priority_low, &handle_task30);
    //returned_task31 = xTaskCreate(task_func, "task 31", 128, (void *)1, priority_low, &handle_task31);
    //returned_task32 = xTaskCreate(task_func, "task 32", 128, (void *)1, priority_low, &handle_task32);
    //returned_task33 = xTaskCreate(task_func, "task 33", 128, (void *)1, priority_low, &handle_task33);
    //returned_task34 = xTaskCreate(task_func, "task 34", 128, (void *)1, priority_low, &handle_task34);
    //returned_task35 = xTaskCreate(task_func, "task 35", 128, (void *)1, priority_low, &handle_task35);
    //returned_task36 = xTaskCreate(task_func, "task 36", 128, (void *)1, priority_low, &handle_task36);
    //returned_task37 = xTaskCreate(task_func, "task 37", 128, (void *)1, priority_low, &handle_task37);
    //returned_task38 = xTaskCreate(task_func, "task 38", 128, (void *)1, priority_low, &handle_task38);
    //returned_task39 = xTaskCreate(task_func, "task 39", 128, (void *)1, priority_low, &handle_task39);
    //returned_task40 = xTaskCreate(task_func, "task 40", 128, (void *)1, priority_low, &handle_task40);

    // Make sure all tasks could be created successfully
    if ((returned_task1 == pdPASS)  && (returned_task2 == pdPASS)  && (returned_task3 == pdPASS)  && (returned_task4 == pdPASS)  && (returned_task5 == pdPASS)  &&
        (returned_task6 == pdPASS)  && (returned_task7 == pdPASS)  && (returned_task8 == pdPASS)  && (returned_task9 == pdPASS)  && (returned_task10 == pdPASS) &&
        (returned_task11 == pdPASS) && (returned_task12 == pdPASS) && (returned_task13 == pdPASS) && (returned_task14 == pdPASS) && (returned_task15 == pdPASS) &&
        (returned_task16 == pdPASS) && (returned_task17 == pdPASS) && (returned_task18 == pdPASS) && (returned_task19 == pdPASS) && (returned_task20 == pdPASS) &&
        (returned_task21 == pdPASS) && (returned_task22 == pdPASS) && (returned_task23 == pdPASS) && (returned_task24 == pdPASS) && (returned_task25 == pdPASS) &&
        (returned_task26 == pdPASS) && (returned_task27 == pdPASS) && (returned_task28 == pdPASS) && (returned_task29 == pdPASS) && (returned_task30 == pdPASS) &&
        (returned_task31 == pdPASS) && (returned_task32 == pdPASS) && (returned_task33 == pdPASS) && (returned_task34 == pdPASS) && (returned_task35 == pdPASS) &&
        (returned_task36 == pdPASS) && (returned_task37 == pdPASS) && (returned_task38 == pdPASS) && (returned_task39 == pdPASS) && (returned_task40 == pdPASS))
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

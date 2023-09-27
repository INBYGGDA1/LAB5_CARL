/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab 5 3.3, deadline miss
 * Date: 2023-09-24
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
SemaphoreHandle_t sem_handle;
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
// Task 1, periodic with 11s, priority 1 (lowest)
void task1(void *temp)
{
    //-----------------------------------------------------------------------------
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 11s
    const int period = 11000;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    int start_time, stop_time, run_time, before_sem, after_sem, sem_wait, deadline_miss_time;
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        UARTprintf("\n- Task 1 started\n");
        start_time = xTaskGetTickCount();

        before_sem = xTaskGetTickCount();
        // Try to take semaphore, block until we can
        if (xSemaphoreTake(sem_handle, (TickType_t)portMAX_DELAY) == pdTRUE)
        {
            after_sem = xTaskGetTickCount();
            // Calculate total time waited to get semaphore (delay), convert to miliseconds
            sem_wait = floor(((after_sem - before_sem)*1000.0) / configTICK_RATE_HZ);
            UARTprintf("\n- Task 1 sem take (%dms delay)\n", sem_wait);

            // DO WORK, 4s
            UARTprintf("\n- Task 1 started its workload\n");
            workload_function(48000000);

            // Release semaphore, continue until we succeed
            UARTprintf("\n- Task 1 sem give\n");
            while (xSemaphoreGive(sem_handle) != pdTRUE)
            {
                ;
            }

            stop_time = xTaskGetTickCount();
            // Calculate total runtime, convert to miliseconds
            run_time = floor(((stop_time - start_time)*1000.0) / configTICK_RATE_HZ);
            UARTprintf("\n- Task 1 finished (%dms)\n", run_time);
            // If stop time - (start time + period) > 0, then the deadline was missed by the difference
            // run time is stop time - start time converted to ms
            deadline_miss_time = run_time - period;
            if(deadline_miss_time > 0)
            {
                UARTprintf("\n- !Task 1 missed deadline by: %dms! \n", deadline_miss_time);
            }
        }
        // Unexpected behavior when trying to acquire the semaphore
        else
        {
            UARTprintf("\n !!ERROR WHEN TRYING TO ACQUIRE SEMAPHORE!! \n");
        }
    }
}
//=============================================================================
// Task 2, periodic with 13s, priority 2 (mid). Note that his task does not use the semaphore
void task2(void *temp)
{
    //-----------------------------------------------------------------------------
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 13s
    const int period = 13000;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    int start_time, stop_time, run_time, deadline_miss_time;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        UARTprintf("\n= Task 2 started\n");
        start_time = xTaskGetTickCount();

        // DO WORK, 4s
        UARTprintf("\n= Task 2 started its workload\n");
        workload_function(48000000);

        stop_time = xTaskGetTickCount();
        // Calculate total runtime, convert to miliseconds
        run_time = floor(((stop_time - start_time) * 1000.0) / configTICK_RATE_HZ);
        UARTprintf("\n= Task 2 finished (%dms)\n", run_time);
        // If stop time - (start time + period) > 0, then the deadline was missed by the difference
        // run time is stop time - start time converted to ms
        deadline_miss_time = run_time - period;
        if (deadline_miss_time > 0)
        {
            UARTprintf("\n= !Task 2 missed deadline by: %dms! \n", deadline_miss_time);
        }
    }
}
//=============================================================================
// Task 3, periodic with 6s, priority 3 (highest)
void task3(void *temp)
{
    //-----------------------------------------------------------------------------
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 6s
    const int period = 6000;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    int start_time, stop_time, run_time, before_sem, after_sem, sem_wait, deadline_miss_time;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);
        UARTprintf("\n+ Task 3 started\n");
        start_time = xTaskGetTickCount();

        before_sem = xTaskGetTickCount();
        // Try to take semaphore
        if (xSemaphoreTake(sem_handle, (TickType_t)portMAX_DELAY) == pdTRUE)
        {
            after_sem = xTaskGetTickCount();
            // Calculate total time waited to get semaphore (delay), convert to miliseconds
            sem_wait = floor(((after_sem - before_sem)*1000.0) / configTICK_RATE_HZ);
            UARTprintf("\n+ Task 3 sem take (%dms delay)\n", sem_wait);

            // DO WORK, 3s
            UARTprintf("\n+ Task 3 started its workload\n");
            workload_function(36000000);

            // Release semaphore, continue until we succeed
            UARTprintf("\n+ Task 3 sem give\n");
            while (xSemaphoreGive(sem_handle) != pdTRUE)
            {
                ;
            }

            stop_time = xTaskGetTickCount();
            // Calculate total runtime, convert to miliseconds
            run_time = floor(((stop_time - start_time)*1000.0) / configTICK_RATE_HZ);
            UARTprintf("\n+ Task 3 finished (%dms)\n", run_time);
            // If stop time - (start time + period) > 0, then the deadline was missed by the difference
            // run time is stop time - start time converted to ms
            deadline_miss_time = run_time - period;
            if (deadline_miss_time > 0)
            {
                UARTprintf("\n+ !Task 3 missed deadline by: %dms! \n", deadline_miss_time);
            }
        }
        // Unexpected behavior when trying to acquire the semaphore
        else
        {
            UARTprintf("\n !!ERROR WHEN TRYING TO ACQUIRE SEMAPHORE!! \n");
        }
    }
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
    /* Variable declarations etc */
    // Run from the PLL at 120 MHz (because this is what FreeRTOS sets in it's config file).
    uint32_t systemClock;
    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Handles for the different tasks
    TaskHandle_t task1_handle = NULL;
    TaskHandle_t task2_handle = NULL;
    TaskHandle_t task3_handle = NULL;

    // Return values for the different tasks
    BaseType_t task1_return;
    BaseType_t task2_return;
    BaseType_t task3_return;

    // Priorities of different tasks
    UBaseType_t task1_priority = 1;
    UBaseType_t task2_priority = 2;
    UBaseType_t task3_priority = 3;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    /* Initialization */
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create semaphore, handle is global
    sem_handle = xSemaphoreCreateBinary();
    // Make sure semaphore could be created successfully
    if(sem_handle == NULL)
    {
        UARTprintf(" !!SEMAPHORE COULD NOT BE ALLOCATED!! ");
    }
    // Binary semaphores need to be given instantly upon creation, they start locked
    while (xSemaphoreGive(sem_handle) != pdTRUE)
    {
        ;
    }

    // Create the different tasks
    task1_return = xTaskCreate(task1, "task 1", 128, (void *) 1, task1_priority, &task1_handle);
    task2_return = xTaskCreate(task2, "task 2", 128, (void *) 1, task2_priority, &task2_handle);
    task3_return = xTaskCreate(task3, "task 3", 128, (void *) 1, task3_priority, &task3_handle);

    // Make sure all tasks could be created successfully
    if ((task1_return == pdPASS) && (task2_return == pdPASS) && (task3_return == pdPASS))
    {
        // Start the scheduler
        vTaskStartScheduler();

        // vTaskStartScheduler() will only return if there is insufficient RTOS heap available to create the idle or timer daemon tasks.
        // Will not get here unless there is insufficient RAM.
        UARTprintf("\n !!INSUFFICENT RAM FOR SCHEDULER!! \n");
    }
    // One or more tasks could not be created successfully because of memory
    UARTprintf("\n !!COULD NOT ALLOCATE REQUIRED MEMORY FOR TASK!! \n");
    //-----------------------------------------------------------------------------
}
//=============================================================================

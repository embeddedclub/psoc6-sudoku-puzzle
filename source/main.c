/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for Sudoku Puzzle game with Capsense Buttons
*              Slider & RGB.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-YEAR Cypress Semiconductor $
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "capsense_task.h"
#include "tft_task.h"
#include "audio.h"

#define printf //

/*******************************************************************************
* Macros
********************************************************************************/

/* Priorities of user tasks in this project. configMAX_PRIORITIES is defined in
* the FreeRTOSConfig.h and higher priority numbers denote high priority tasks.
*/
#define TASK_CAPSENSE_PRIORITY      (1u)
#define TASK_TFT_PRIORITY           (1u)
#define TASK_AUD_PRIORITY           (1u)

/* Stack sizes of user tasks in this project */
#define TASK_CAPSENSE_STACK_SIZE    (1*128)
#define TASK_TFT_STACK_SIZE         (1u*512)
#define TASK_AUD_STACK_SIZE         (1*1024)

/* Queue lengths of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE        (1u)


/******************************************************************************
* Global Variables
******************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/******************************************************************************
 * Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application with BSP initilization and Tasks creation.
 */
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package */
    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0u);
    }

    /* Enable global interrupts */
    __enable_irq();

    audio_clock_init();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("*****************Sudoku Puzzle******************\r\n");
    printf("*********Play with CapSense 6x6 Puzzle**********\r\n");
    printf("************************************************\r\n\r\n");

    /* Create the queues. See the respective data-types for details of queue
    * contents
    */
    capsense_command_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
                                     sizeof(capsense_command_t));
    if(NULL == capsense_command_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }

    gui_capsense_data_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
                                     sizeof(gui_capsense_data_t));
    if(NULL == gui_capsense_data_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }

    if (pdPASS != xTaskCreate(task_audio, "Audio Task", TASK_AUD_STACK_SIZE,
                              NULL, TASK_AUD_PRIORITY, &audio_task_handle))
    {
        printf("Failed to create the audio task!\r\n");
        CY_ASSERT(0u);
    }


    if (pdPASS != xTaskCreate(task_capsense, "CapSense Task",
                              TASK_CAPSENSE_STACK_SIZE,
                              NULL, TASK_CAPSENSE_PRIORITY, NULL))
    {
        printf("Failed to create the capsense task!\r\n");
        CY_ASSERT(0u);
    }


    if (pdPASS != xTaskCreate(tft_task, "Tft Task", TASK_TFT_STACK_SIZE,
                              NULL, TASK_TFT_PRIORITY, &gui_task_handle))
    {
        printf("Failed to create the TFT task!\r\n");
        CY_ASSERT(0u);
    }


    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();

    /*~~~~~~~~~~~~~~~~~~~~~ Should never get here! ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* RTOS scheduler exited */
    /* Halt the CPU if scheduler exits */
    CY_ASSERT(0u);

}

/* [] END OF FILE */

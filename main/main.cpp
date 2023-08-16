/**
 * @file main.cpp
 * 
 * @brief Contains FreeRTOS tasks and a pthread for sending and receiving messages between WASM app and CAN controllers.
 * @todo convert unessesary ESP_LOGI messages to ESP_LOGD
*/

/** 
 * @mainpage Can Controllers Documentation
 * 
 * This is the documentation for the firmware on the esp32 boards for the NMEA T connector project. 
 * These pages contain class and function descriptions. For installation and more detailed instructions, please visit the
 * <a href="https://cyberboat.gitbook.io/cyberboat/">project wiki</a> 
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "NMEA_msg.h"
#include "esp_log.h"
#include <N2kMsg.h>
#include <NMEA2000_esp32-c6.h> 
#include <NMEA2000.h>
#include <N2kMessages.h>
#include "esp_err.h"


#include "driver/gpio.h"


#include <queue>
#include <string>
#include <iomanip>
#include <chrono>


#define MAX_DATA_LENGTH_BTYES           223
#define BUFFER_SIZE                     (10 + 223*2) //10 bytes for id, 223*2 bytes for data
#define MY_ESP_LOG_LEVEL                ESP_LOG_INFO // the log level for this file

#define STATS_TASK_PRIO     tskIDLE_PRIORITY //3
#define STATS_TICKS         pdMS_TO_TICKS(1000)
#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

// Tag for ESP logging
static const char* TAG = "FIREHOSE";
static const char* TAG_STATUS = "STATUS";

/**
 * @brief Creates a NMEA2000 Object
 * 
 * NMEA2000(TX_PIN, RX_PIN)
*/
tNMEA2000_esp32c6 NMEA2000(GPIO_NUM_22, GPIO_NUM_23);

// Task Handles
static TaskHandle_t N2K_send_task_handle = NULL;
static TaskHandle_t N2K_stats_task_handle = NULL;


static unsigned long N2kMsgSentCount=0;
static unsigned long N2kMsgFailCount=0;

//----------------------------------------------------------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------------------------------------------------------
//char * wasm_buffer = NULL;  //!< buffer allocated for wasm app, used to hold received messages so app can access them
int read_msg_count = 0; //!< Used to track messages read
int send_msg_count = 0; //!< Used to track messages sent
uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL; //!< Sets which alerts to enable for TWAI controller

// Task Counters - temporary, for debugging
int tx_task_count = 0;

int stats_task_count = 0;

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Helper Functions for Conversions
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Fills a char array with values from a uint8_t array
 * 
 * @param[in] data_uint8_arr
 * @param[out] data_char_arr 
 * 
*/
void uint8ArrayToCharrArray(uint8_t (&data_uint8_arr)[MAX_DATA_LENGTH_BTYES], unsigned char (&data_char_arr)[MAX_DATA_LENGTH_BTYES]){
    for (size_t i = 0; i < MAX_DATA_LENGTH_BTYES; ++i) {
        data_char_arr[i] = static_cast<unsigned char>(data_uint8_arr[i]);
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------

/**
 * \brief Sends a message
 * 
 * Converts NMEA-msg to the NMEA2000 Library N2kMsg format and sends the message
 * 
 * @todo update time to take time from message
 * @todo update for multiple controllers
 * 
*/
bool SendN2kMsg(NMEA_msg msg) {
  tN2kMsg N2kMsg;
  N2kMsg.Priority = msg.priority;
  N2kMsg.PGN = msg.PGN;
  N2kMsg.Source = msg.source;
  N2kMsg.Destination = 0xff; //not used

  N2kMsg.DataLen = msg.data_length_bytes;

  uint8ArrayToCharrArray(msg.data, N2kMsg.Data);

  N2kMsg.MsgTime = N2kMillis64();//TODO 

  if ( NMEA2000.SendMsg(N2kMsg) ) {
    ESP_LOGD(TAG, "sent a message \n");
    N2kMsgSentCount++;
    send_msg_count++;
  } else {
    ESP_LOGW(TAG, "failed to send a message \n");
    N2kMsgFailCount++;
  }
  return true;
}

/**
 * @brief Prints runtime and percentage for tasks and pthreads
 * 
 * To use this function, you need to configure some settings in menuconfig. 
 * 
 * You must enable FreeRTOS to collect runtime stats under 
 * Component Config -> FreeRTOS -> Kernel -> configGENERATE_RUN_TIME_STATS
 * 
 * You must also choose the clock source for run time stats configured under
 * Component Config -> FreeRTOS -> Port -> Choose the clock source for runtime stats.
 * The esp_timer should be selected by default. 
 * This option will affect the time unit resolution in which the statistics
 *  are measured with respect to.
 * 
 * 
 * 
 * @param[in] xTicksToWait
 * 
*/
static esp_err_t print_real_time_stats(TickType_t xTicksToWait)
{
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;
    esp_err_t ret;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = reinterpret_cast<TaskStatus_t*>(std::malloc(sizeof(TaskStatus_t) * start_array_size));
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        free(start_array);
        free(end_array);
        return ret;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        free(start_array);
        free(end_array);
        return ret;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = reinterpret_cast<TaskStatus_t*>(std::malloc(sizeof(TaskStatus_t) * end_array_size));
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        free(start_array);
        free(end_array);
        return ret;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        free(start_array);
        free(end_array);
        return ret;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        free(start_array);
        free(end_array);
        return ret;
    }

    printf("| Task | Run Time | Percentage\n");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }
        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);
            printf("| %s | %" PRIu32 " | %" PRIu32 "%%\n", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
        }
    }

    //Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            printf("| %s | Deleted\n", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    ret = ESP_OK;
    free(start_array);
    free(end_array);
    return ret;

}


/**
 * @brief Retrieves twai status and alerts
 * 
 * Used to display information regarding queue's filling up, and how many messages have been sent/received.
 * 
 * @param[in] TAG
*/
void GetStatus(const char* TAG){
    uint32_t alerts = 0;
    NMEA2000.ReadAlerts(alerts, pdMS_TO_TICKS(1));
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL){
        ESP_LOGW(TAG, "TWAI rx queue full");
    } 
    twai_status_info_t status;
    NMEA2000.GetTwaiStatus(status);

    // Task Counters
    ESP_LOGI(TAG, "TX task count: %d", tx_task_count);
    ESP_LOGI(TAG, "Stats task count: %d", stats_task_count);

}
/**
 * @brief Optional FreeRTOS task for printing status messages for debugging 
 * 
 * @param pvParameters
 * 
*/
static void stats_task(void *arg)
{
    //Print real time stats periodically
    while (1) {
        printf("\n\nGetting real time stats over %" PRIu32 " ticks\n", STATS_TICKS);
        if (print_real_time_stats(STATS_TICKS) == ESP_OK) {
            printf("Real time stats obtained\n");
        } else {
            printf("Error getting real time stats\n");
        }
        GetStatus(TAG_STATUS);
        stats_task_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



/**
 * @brief FreeRTOS task for processing and sending messages from CAN controller with NMEA2000 library
 * 
 * Initializes NMEA2000 Object, then sends messages in loop.
 * 
 * @todo frame buffer should be 32 - see if this works
 * @param pvParameters
*/
void N2K_send_task(void *pvParameters)
{   
    esp_log_level_set(TAG, MY_ESP_LOG_LEVEL);
    ESP_LOGI(TAG, "Starting N2k_task");
    NMEA_msg msg;
    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.EnableForward(false);               

    NMEA2000.SetMode(tNMEA2000::N2km_SendOnly);
    
    NMEA2000.Open();

    NMEA2000.ConfigureAlerts(alerts_to_enable);

    // Task Loop
    for (;;)
    {
        //NMEA_msg msg;

        msg.controller_number = 0;
        msg.PGN = 129026;
        msg.source = 15;
        msg.priority = 2;
        msg.data_length_bytes = 8;
        msg.data[0] = 0x00;
        msg.data[1] = 0x02;
        msg.data[2] = 0x04;
        msg.data[3] = 0x06;
        msg.data[4] = 0x08;
        msg.data[5] = 0x0a;
        msg.data[6] = 0x0c;
        msg.data[7] = 0x0e;
        SendN2kMsg(msg);
        ESP_LOGD(TAG, "Send task called");

        tx_task_count++;      
        vTaskDelay(10);  
    }
    vTaskDelete(NULL); // should never get here...
}




/**
 * @brief Creates a FreeRTOS task for sending and receiving to and from CAN Controller and creates a pthread to run WASM app
 * 
 * In ESP-IDF, a pthread is just a wrapper on FreeRTOS
*/
extern "C" int app_main(void)
{
    /* Status Task*/
    // Uncomment this task here if you do not want to use
    esp_err_t result = ESP_OK;
    printf( "create task");
    xTaskCreatePinnedToCore(
        &stats_task,            // Pointer to the task entry function.
        "stats_task",           // A descriptive name for the task for debugging.
        4096,                 // size of the task stack in bytes.
        NULL,                 // Optional pointer to pvParameters
        STATS_TASK_PRIO, // priority at which the task should run
        &N2K_stats_task_handle,      // Optional pass back task handle
        1
    );
    if (N2K_stats_task_handle == NULL)
    {
        printf("Unable to create task.");
        result = ESP_ERR_NO_MEM;
        goto err_out;
    }
    // end of status task

    /* Sending task */
    ESP_LOGV(TAG, "create task");
    xTaskCreatePinnedToCore(
        &N2K_send_task,            // Pointer to the task entry function.
        "Send_task",           // A descriptive name for the task for debugging.
        3072,                 // size of the task stack in bytes.
        NULL,                 // Optional pointer to pvParameters
        tskIDLE_PRIORITY, // priority at which the task should run
        &N2K_send_task_handle,      // Optional pass back task handle
        1
    );
    if (N2K_send_task_handle == NULL)
    {
        ESP_LOGE(TAG, "Unable to create task.");
        result = ESP_ERR_NO_MEM;
        goto err_out;
    }

err_out:
    if (result != ESP_OK)
    {
        if (N2K_send_task_handle != NULL)
        {
            vTaskDelete(N2K_send_task_handle);

            N2K_send_task_handle = NULL;
        }
    }

    return 0;
};
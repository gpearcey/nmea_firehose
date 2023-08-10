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
#include "esp_pthread.h"

#include "driver/gpio.h"
#include "bi-inc/attr_container.h"

#include "wasm_export.h"
#include "bh_read_file.h"
#include "bh_getopt.h"
#include "bh_platform.h"

#include <queue>
#include <string>
#include <iomanip>
#include <chrono>

//WebAssembley App
#include "nmea_attack.h" 

#define NATIVE_STACK_SIZE               (32*1024)
#define NATIVE_HEAP_SIZE                (32*1024)
#define PTHREAD_STACK_SIZE              4096
#define MAX_DATA_LENGTH_BTYES           223
#define BUFFER_SIZE                     (10 + 223*2) //10 bytes for id, 223*2 bytes for data
#define MY_ESP_LOG_LEVEL                ESP_LOG_INFO // the log level for this file

#define STATS_TASK_PRIO     tskIDLE_PRIORITY //3
#define STATS_TICKS         pdMS_TO_TICKS(1000)
#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE
#define TX_QUEUE_SIZE       100
#define RX_QUEUE_SIZE       100

// Tag for ESP logging
static const char* TAG_TWAI_TX = "TWAI_SEND";
static const char* TAG_TWAI_RX = "TWAI_RECEIVE";
static const char* TAG_WASM = "WASM";
static const char* TAG_STATUS = "STATUS";

/**
 * @brief Creates a NMEA2000 Object
 * 
 * NMEA2000(TX_PIN, RX_PIN)
*/
tNMEA2000_esp32c6 NMEA2000(GPIO_NUM_22, GPIO_NUM_23);

// Task Handles
static TaskHandle_t N2K_send_task_handle = NULL;
static TaskHandle_t N2K_receive_task_handle = NULL;
static TaskHandle_t N2K_stats_task_handle = NULL;

QueueHandle_t controller0_tx_queue; //!< Queue that stores messages to be sent out on controller 0
QueueHandle_t rx_queue; //!< Queue that stores all messages received on all controllers

static unsigned long N2kMsgSentCount=0;
static unsigned long N2kMsgFailCount=0;


//----------------------------------------------------------------------------------------------------------------------------
// Forward Declarations
//----------------------------------------------------------------------------------------------------------------------------
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
std::string nmea_to_string(NMEA_msg& msg);
void uintArrToCharrArray(uint8_t (&data_uint8_arr)[MAX_DATA_LENGTH_BTYES], unsigned char (&data_char_arr)[MAX_DATA_LENGTH_BTYES]);
//----------------------------------------------------------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------------------------------------------------------
char * wasm_buffer = NULL;  //!< buffer allocated for wasm app, used to hold received messages so app can access them
int read_msg_count = 0; //!< Used to track messages read
int send_msg_count = 0; //!< Used to track messages sent
uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL; //!< Sets which alerts to enable for TWAI controller

// Task Counters - temporary, for debugging
int rx_task_count = 0;
int tx_task_count = 0;
int wasm_pthread_count = 0;
int stats_task_count = 0;
double wasm_main_duration;
//-------------------------------------------------------------------------------------------------------------------------------
// Native Functions to Export to WASM App
//-----------------------------------------------------------------------------------------------------------------------------

/**
 * @brief prints a uint8_t array to terminal
 * 
 * Native function to be exported to WASM app. Used for debugging purposes.
 * 
 * @param exec_env
 * @param input pointer to array
 * @param length length of the array
*/
void PrintStr(wasm_exec_env_t exec_env,uint8_t* input, int32_t length){
    printf("PrintStr: ");
    for (int32_t i = 0; i < length;i++ ) {
        printf("%u ", *(input+i));
    }
    printf("\n");
    return;
}

/**
 * @brief prints a integer to terminal
 * 
 * Native function to be exported to WASM app. Used for debugging purposes.
 * 
 * @param exec_env
 * @param number integer to print
 * @param hex prints the number in hex format if hex is 1, else prints it in decimal format
*/
void PrintInt32(wasm_exec_env_t exec_env,int32_t number,int32_t hex){
    if (hex == 1){
        printf("PrintInt32: %lx \n", number);
    }
    else {
        printf("PrintInt32: %li \n", number);
    }    
    return;
}

/****************************************************************************
 * \brief Puts a message in a controller send queue
 * 
 * This function is exported to the WASM app to be called from app to send a message. 
 * Creates a NMEA_msg object and puts it into the appropriate send queue. 
 * 
 * @param exec_env
 * @param[in] controller_number 
 * @param[in] priority
 * @param[in] PGN
 * @param[in] source
 * @param[in] data
 * @param[in] data_length_bytes 
 * 
 * \return 1 if message converted successfully, 0 if not.
*/
int32_t SendMsg(wasm_exec_env_t exec_env, int32_t controller_number, int32_t priority, int32_t PGN, int32_t source, uint8_t* data, int32_t data_length_bytes ){
    ESP_LOGD(TAG_WASM, "SendMsg called \n");
    NMEA_msg msg;
    msg.controller_number = controller_number;
    msg.priority = priority;
    msg.PGN = PGN;
    msg.source = source;
    msg.data_length_bytes = data_length_bytes;

    // Copy the data bytes
    for (size_t i = 0; i < data_length_bytes; ++i) {
        uint8_t value = static_cast<uint8_t>(data[i]);
        msg.data[i] = value;
    }

    
    if (controller_number == 0){
        // Add to controller 0 queue
        ESP_LOGD(TAG_WASM,"Added a msg to ctrl0_q with PGN %u \n", msg.PGN);
        if (xQueueSendToBack(controller0_tx_queue, &msg, pdMS_TO_TICKS(10))){
            return 1;
        }
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Helper Functions for Conversions
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * \brief converts a NMEA_msg to a string
 * @todo make sure that if pgn is only 4 digits in hex it still takes 5
 * @param[in] msg reference to a NMEA_msg object
 * \return std::string representing the message
*/
std::string nmea_to_string(NMEA_msg& msg){
    std::stringstream ss;
    ss << std::hex << std::setw(1) << std::setfill('0') << static_cast<int>(msg.controller_number);
    ss << std::hex << std::setw(1) << std::setfill('0') << static_cast<int>(msg.priority);
    ss << std::hex << std::setw(5) << std::setfill('0') << msg.PGN;
    ss << std::hex << std::setw(1) << std::setfill('0') << static_cast<int>(msg.source);
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(msg.data_length_bytes);
    for (uint8_t d : msg.data){
        char hex_num[3];
        sprintf(hex_num, "%X", d);
        ss << std::setw(2) << hex_num;
    }
    const std::string s = ss.str();
    return s;
    
}

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
    ESP_LOGD(TAG_TWAI_TX, "sent a message \n");
    N2kMsgSentCount++;
    send_msg_count++;
  } else {
    ESP_LOGW(TAG_TWAI_TX, "failed to send a message \n");
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
    ESP_LOGI(TAG, "Msgs queued for transmission: %" PRIu32 " Unread messages in rx queue: %" PRIu32, status.msgs_to_tx, status.msgs_to_rx);
    ESP_LOGI(TAG, "Msgs lost due to RX FIFO overrun: %" PRIu32 "", status.rx_overrun_count);
    ESP_LOGI(TAG, "Msgs lost due to full RX queue: %" PRIu32 "", status.rx_missed_count);
    ESP_LOGI(TAG, "Messages Read: %d, Messages Sent %d", read_msg_count, send_msg_count);
    UBaseType_t msgs_in_rx_q = uxQueueMessagesWaiting(rx_queue);
    ESP_LOGI(TAG, "Received Messages queue size: %d \n", msgs_in_rx_q);
    UBaseType_t msgs_in_q = uxQueueMessagesWaiting(controller0_tx_queue);
    ESP_LOGI(TAG, "Controller 0 send queue size: %d \n", msgs_in_q);

    // Task Counters
    ESP_LOGI(TAG, "RX task count: %d", rx_task_count);
    ESP_LOGI(TAG, "TX task count: %d", tx_task_count);
    ESP_LOGI(TAG, "Wasm pthread count: %d", wasm_pthread_count);
    ESP_LOGI(TAG, "Stats task count: %d", stats_task_count);

    //Duration of the app_instance_main for the wasm pthread
    ESP_LOGI(TAG, "Duration of wasm task (ms): %f",wasm_main_duration/1000000);
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
 * @brief FreeRTOS task for receiving messages from CAN controller
 * 
 * @param pvParameters
 * 
 * NMEA2000 Library is designed so that message receiving and sending is handled within the same task. 
 * In the NMEA2000_ESP32 library, this is made possible by letting the twai rx interrupt handle receiving CAN frames. 
 * In this library, the receiving is separated from the processing and sending of messages. This is done because 
 * I was unable to trigger recieving a CAN frame from the twai rx interrupt, so CAN_read_frame() must be called explicitly here. 
*/
void N2K_receive_task(void *pvParameters){
    esp_log_level_set(TAG_TWAI_RX, MY_ESP_LOG_LEVEL);
    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.EnableForward(false);               

    NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndSend);
    
    NMEA2000.Open();

    NMEA2000.ConfigureAlerts(alerts_to_enable);

    // Task Loop
    while(1)
    {
        NMEA2000.CAN_read_frame();
        NMEA2000.ParseMessages(); // Calls message handle whenever a message is available
        rx_task_count++;

    }
    vTaskDelete(NULL); // should never get here...
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
    esp_log_level_set(TAG_TWAI_TX, MY_ESP_LOG_LEVEL);
    ESP_LOGI(TAG_TWAI_TX, "Starting N2k_task");
    NMEA_msg msg;

    // Task Loop
    for (;;)
    {
        if( xQueueReceive( controller0_tx_queue, &msg, (100 / portTICK_PERIOD_MS) ))
        {
            SendN2kMsg(msg);
        }
        ESP_LOGD(TAG_TWAI_TX, "Send task called");

        tx_task_count++;        
    }
    vTaskDelete(NULL); // should never get here...
}


/**
 * \brief Creates a NMEA_msg object and adds it to.data the received messages queue
 * 
 * @todo handle out of range data
 * \param N2kMsg Reference to the N2KMs being handled
 * \return void
 */
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  if (N2kMsg.Source == 14){
    ESP_LOGD(TAG_TWAI_RX, "source is 14");
    return;
  }
  ESP_LOGD(TAG_TWAI_RX, "Message Handler called");
  NMEA_msg msg;
  msg.controller_number = 0;
  msg.priority = N2kMsg.Priority;
  msg.PGN = N2kMsg.PGN;
  msg.source = N2kMsg.Source;
  msg.data_length_bytes = N2kMsg.DataLen;
  size_t size = sizeof(N2kMsg.Data) / sizeof(N2kMsg.Data[0]);
  // Perform the conversion with range checking
  for (size_t i = 0; i < size; i++) {
      if (N2kMsg.Data[i] <= static_cast<unsigned char>(CHAR_MAX)) {
          msg.data[i] = static_cast<signed char>(N2kMsg.Data[i]);
      } else {
          // Handle out-of-range value
          //msg.data[i] = /* Your desired behavior for out-of-range values */;
          ESP_LOGE("Message Handle", "data out of range for signed array");
      }
  }

  if(xQueueSendToBack(rx_queue, &msg, pdMS_TO_TICKS(10)) == 0){
    ESP_LOGW(TAG_TWAI_RX, "Could not add received message to RX queue");    
  }
  else{
    ESP_LOGD(TAG_TWAI_RX, " added msg to received queue");
  }
  read_msg_count++;
  
}



/**
 * @brief executes main function in wasm app
 * 
 * @param module_inst wasm module instance
*/
static void * app_instance_main(wasm_module_inst_t module_inst)
{
    const char *exception;

    wasm_application_execute_main(module_inst, 0, NULL);
    if ((exception = wasm_runtime_get_exception(module_inst)))
        ESP_LOGW(TAG_WASM,"%s\n", exception);
    return NULL;
}

/**
 * @brief WASM pthread to host wasm app
 * 
 * Sets up the wasm environment. 
 * Links native function to be exported. 
 * Calls wasm app function to link allocated wasm buffer.
 * Runs main function in wasm app.
 * 
 * @param arg unused - I don't know why this is required
*/
void * iwasm_main(void *arg)
{
    esp_log_level_set(TAG_WASM, MY_ESP_LOG_LEVEL);
    (void)arg; /* unused */
    /* setup variables for instantiating and running the wasm module */
    uint8_t *wasm_file_buf = NULL;
    unsigned wasm_file_buf_size = 0;
    wasm_exec_env_t exec_env = NULL;
    wasm_module_t wasm_module = NULL;
    wasm_module_inst_t wasm_module_inst = NULL;
    char error_buf[128];
    void *ret;
    wasm_function_inst_t func = NULL;
    RuntimeInitArgs init_args;

    uint32_t buffer_for_wasm = 0;

    /* configure memory allocation */
    memset(&init_args, 0, sizeof(RuntimeInitArgs));

    /* the native functions that will be exported to WASM app */
    static NativeSymbol native_symbols[] = {
        {
            "PrintStr", // the name of WASM function name
            reinterpret_cast<void*>(PrintStr),    // the native function pointer
            "($i)",  // the function prototype signature, avoid to use i32
            NULL        // attachment is NULL
        },
        {
            "PrintInt32",
            reinterpret_cast<void*>(PrintInt32),   
            "(ii)", 
            NULL      
        },
        {
            "SendMsg",
            reinterpret_cast<void*>(SendMsg),   
            "(iiii*~)i",
            NULL    
        }
    };
#if WASM_ENABLE_GLOBAL_HEAP_POOL == 0
    init_args.mem_alloc_type = Alloc_With_Allocator;
    init_args.mem_alloc_option.allocator.malloc_func = (void *)os_malloc;
    init_args.mem_alloc_option.allocator.realloc_func = (void *)os_realloc;
    init_args.mem_alloc_option.allocator.free_func = (void *)os_free;
#else
#error The usage of a global heap pool is not implemented yet for esp-idf.
#endif

    /* configure the native functions being exported to WASM app */
    init_args.n_native_symbols = sizeof(native_symbols) / sizeof(NativeSymbol);
    init_args.native_module_name = "env";
    init_args.native_symbols = native_symbols;


    ESP_LOGI(TAG_WASM, "Initialize WASM runtime");
    /* initialize runtime environment */
    if (!wasm_runtime_full_init(&init_args)) {
        ESP_LOGE(TAG_WASM, "Init runtime failed.");
        return NULL;
    }
    ESP_LOGI(TAG_WASM, "Run wamr with interpreter");

    wasm_file_buf = (uint8_t *)nmea_attack_wasm;
    wasm_file_buf_size = sizeof(nmea_attack_wasm);

    /* load WASM module */
    if (!(wasm_module = wasm_runtime_load(wasm_file_buf, wasm_file_buf_size,
                                          error_buf, sizeof(error_buf)))) {
        ESP_LOGE(TAG_WASM, "Error in wasm_runtime_load: %s", error_buf);
        goto fail;
    }

    ESP_LOGI(TAG_WASM, "Instantiate WASM runtime");
    if (!(wasm_module_inst =
              wasm_runtime_instantiate(wasm_module, NATIVE_STACK_SIZE, // stack size
                                       NATIVE_HEAP_SIZE,              // heap size
                                       error_buf, sizeof(error_buf)))) {
        ESP_LOGE(TAG_WASM, "Error while instantiating: %s", error_buf);
        goto fail;
    }

    
    exec_env = wasm_runtime_create_exec_env(wasm_module_inst, NATIVE_STACK_SIZE);//stack size
    if (!exec_env) {
        ESP_LOGW(TAG_WASM,"Create wasm execution environment failed.\n");
        goto fail;
    }

    ESP_LOGI(TAG_WASM, "Malloc buffer in wasm function");
    buffer_for_wasm = wasm_runtime_module_malloc(wasm_module_inst, 100, (void **)&wasm_buffer);
    if (buffer_for_wasm == 0) {
        ESP_LOGI(TAG_WASM, "Malloc failed");
        goto fail;
    }
    uint32 argv[2];
    argv[0] = buffer_for_wasm;     /* pass the buffer address for WASM space */
    argv[1] = BUFFER_SIZE;                 /* the size of buffer */
    ESP_LOGI(TAG_WASM, "Call wasm function");
    /* it is runtime embedder's responsibility to release the memory,
       unless the WASM app will free the passed pointer in its code */
    
    if (!(func = wasm_runtime_lookup_function(wasm_module_inst, "link_msg_buffer",
                                               NULL))) {
        ESP_LOGW(TAG_WASM,
            "The wasm function link_msg_buffer wasm function is not found.\n");
        goto fail;
    }

    if (wasm_runtime_call_wasm(exec_env, func, 2, argv)) {
        ESP_LOGI(TAG_WASM,"Native finished calling wasm function: link_msg_buffer, "
               "returned a formatted string: %s\n",
               wasm_buffer);
    }
    else {
        ESP_LOGW(TAG_WASM,"call wasm function link_msg_buffer failed. error: %s\n",
               wasm_runtime_get_exception(wasm_module_inst));
        goto fail;
    }

    // Task Loop
    while (true){
        ESP_LOGD(TAG_WASM, "run main() of the application");
        auto start = std::chrono::high_resolution_clock::now(); 
        NMEA_msg msg;
        if (xQueueReceive(rx_queue, &msg, (100 / portTICK_PERIOD_MS) == 1)){
            std::string str_msg = nmea_to_string(msg);
            strncpy(wasm_buffer, str_msg.c_str(), str_msg.size());
            ret = app_instance_main(wasm_module_inst);  //Call the main function 
            assert(!ret);
        } else{
            vTaskDelay(10 / portTICK_PERIOD_MS); // I don't understand why this is nessesary
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto ns_duration = duration_cast<std::chrono::nanoseconds>(end-start);
        wasm_main_duration = static_cast<double>(ns_duration.count());
        
        wasm_pthread_count++;
    }


    wasm_runtime_module_free(wasm_module_inst, buffer_for_wasm);
    
    /* destroy the module instance */
    ESP_LOGI(TAG_WASM, "Deinstantiate WASM runtime");
    wasm_runtime_deinstantiate(wasm_module_inst);

fail:
    if (exec_env)
        wasm_runtime_destroy_exec_env(exec_env);
    if (wasm_module_inst) {
        if (buffer_for_wasm){
            wasm_runtime_module_free(wasm_module_inst, buffer_for_wasm);}
        wasm_runtime_deinstantiate(wasm_module_inst);
    }
    if (wasm_module){
        /* unload the module */
        ESP_LOGI(TAG_WASM, "Unload WASM module");
        wasm_runtime_unload(wasm_module);
    }
    if (wasm_buffer)
        BH_FREE(wasm_buffer);

    /* destroy runtime environment */
    ESP_LOGI(TAG_WASM, "Destroy WASM runtime");
    wasm_runtime_destroy();
    return NULL;
}

/**
 * @brief Creates a FreeRTOS task for sending and receiving to and from CAN Controller and creates a pthread to run WASM app
 * 
 * In ESP-IDF, a pthread is just a wrapper on FreeRTOS
*/
extern "C" int app_main(void)
{
    controller0_tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(NMEA_msg));
    rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(NMEA_msg));

    /* Status Task*/
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

    /* Sending task */
    ESP_LOGV(TAG_WASM, "create task");
    xTaskCreatePinnedToCore(
        &N2K_send_task,            // Pointer to the task entry function.
        "Send_task",           // A descriptive name for the task for debugging.
        3072,                 // size of the task stack in bytes.
        NULL,                 // Optional pointer to pvParameters
        tskIDLE_PRIORITY+1, // priority at which the task should run
        &N2K_send_task_handle,      // Optional pass back task handle
        1
    );
    if (N2K_send_task_handle == NULL)
    {
        ESP_LOGE(TAG_TWAI_TX, "Unable to create task.");
        result = ESP_ERR_NO_MEM;
        goto err_out;
    }

    /* Receiving task */
    ESP_LOGV(TAG_TWAI_RX, "create task");
    xTaskCreatePinnedToCore(
        &N2K_receive_task,            // Pointer to the task entry function.
        "Receive_task",           // A descriptive name for the task for debugging.
        3072,                 // size of the task stack in bytes.
        NULL,                 // Optional pointer to pvParameters
        tskIDLE_PRIORITY+3, // priority at which the task should run
        &N2K_receive_task_handle,      // Optional pass back task handle
        0
    );
    if (N2K_receive_task_handle == NULL)
    {
        ESP_LOGE(TAG_TWAI_RX, "Unable to create task.");
        result = ESP_ERR_NO_MEM;
        goto err_out;

    }

    /* Wasm pthread */
    pthread_t t;
    int res;
    esp_pthread_cfg_t esp_pthread_cfg;



    pthread_attr_t tattr;
    pthread_attr_init(&tattr);
    pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setstacksize(&tattr, PTHREAD_STACK_SIZE);

    // Use the ESP-IDF API to change the default thread attributes
    esp_pthread_cfg = esp_pthread_get_default_config();
    ESP_LOGI(TAG_WASM, "Pthread priority: %d", esp_pthread_cfg.prio);
    ESP_LOGI(TAG_WASM, "Pthread core: %d", esp_pthread_cfg.pin_to_core);
    esp_pthread_cfg.prio = tskIDLE_PRIORITY+1; //change priority 
    esp_pthread_cfg.pin_to_core = 1; // pin to core 1
    ESP_ERROR_CHECK( esp_pthread_set_cfg(&esp_pthread_cfg) );

    res = pthread_create(&t, &tattr, iwasm_main, (void *)NULL);
    assert(res == 0);

    esp_pthread_get_cfg(&esp_pthread_cfg);
    ESP_LOGI(TAG_WASM, "Pthread priority: %d", esp_pthread_cfg.prio);
    res = pthread_join(t, NULL);
    assert(res == 0);

err_out:
    if (result != ESP_OK)
    {
        if (N2K_send_task_handle != NULL || N2K_receive_task_handle != NULL || N2K_stats_task_handle != NULL)
        {
            vTaskDelete(N2K_send_task_handle);
            vTaskDelete(N2K_receive_task_handle);
            vTaskDelete(N2K_stats_task_handle);
            N2K_send_task_handle = NULL;
            N2K_receive_task_handle = NULL;
            N2K_stats_task_handle = NULL;
        }
    }

    return 0;
};
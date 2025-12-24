// SPDX-License-Identifier: MIT
/**
 * @file servo_controller.c
 * @brief Source code for servo_controller
 * © 2025 Levi Johanon. All rights reserved.
 * See LICENSE for details.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "servo_controller.h"
#include "servo_internal.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"

#ifndef CONFIG_SERVO_TASK_STACK_SIZE
#define CONFIG_SERVO_TASK_STACK_SIZE 4096
#endif

#ifndef CONFIG_SERVO_TASK_PRIORITY
#define CONFIG_SERVO_TASK_PRIORITY 15
#endif

static const char *TAG = "servo_controller";

static const rmt_symbol_word_t pulse = {.val = 0x80040024};
static const rmt_symbol_word_t clk[17] = {{.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},
                                      {.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},
                                      {.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},
                                      {.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},{.val = 0x80010001},
                                      {.val = 0x00080000}};
static const rmt_symbol_word_t all_high = {.val = 0x00088020};



struct servo_controller_s{
    gpio_num_t gpio_array[4];   //clk,pulse,1,2
    servo_precision_t servo_pres;
    float servo_angles[32];     //Holds servo angle 0-180
    uint16_t high_cycles[32];   //Holds number of cycles each servo is high for
    uint16_t *LUT1;
    uint16_t *LUT2;
    uint8_t *streaks_buffer;
    rmt_symbol_word_t* rmt1;
    rmt_symbol_word_t* rmt2;
    size_t rmt_size_1;
    size_t rmt_size_2;
    rmt_channel_handle_t rmt_handles[4];    //clk,pulse,1,2
    rmt_sync_manager_handle_t sync_handle;
    rmt_encoder_handle_t encoder_handle;
    TaskHandle_t task_handle;
    bool running;
};

//Initialize the servo_controller
esp_err_t servo_initialize_controller(const servo_controller_config_t *servo_cfg, servo_controller_handle_t *ret_handle){

    #ifndef CONFIG_SOC_RMT_SUPPORTED
        ESP_LOGE(TAG,"RMT is not supported on this ESP model");
        return ESP_ERR_NOT_SUPPORTED
    #endif
    #ifndef CONFIG_SOC_RMT_SUPPORT_TX_LOOP_COUNT
        ESP_LOGE(TAG,"RMT Sync manager is not supported on this ESP model");
        return ESP_ERR_NOT_SUPPORTED
    #endif

    gpio_num_t gpio_array[4] = {servo_cfg->clk_gpio, servo_cfg->pulse_gpio, servo_cfg->servo1_gpio, servo_cfg->servo2_gpio};
    for(int i = 0; i<4; i++){
        ESP_RETURN_ON_FALSE(gpio_array[i] <= CONFIG_SOC_GPIO_OUT_RANGE_MAX && gpio_array[i] >= 0
                        , ESP_ERR_INVALID_ARG, TAG, "GPIO pin %d is out of range", gpio_array[i]);
    }

    ESP_RETURN_ON_FALSE(ret_handle == NULL, ESP_ERR_INVALID_STATE, TAG, "The return handle is not NULL");

    //Initialize Fixed memory
    servo_controller_handle_t handle = heap_caps_calloc(1,sizeof(struct servo_controller_s), (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    //Initialize dynamic sized memory
    handle->LUT1 = heap_caps_calloc(servo_cfg->serv_pres + 1,sizeof(uint16_t),(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    handle->LUT2 = heap_caps_calloc(servo_cfg->serv_pres + 1,sizeof(uint16_t),(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    handle->streaks_buffer = heap_caps_calloc((servo_cfg->serv_pres)*16 + 10,sizeof(uint8_t),(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    handle->rmt1 = heap_caps_calloc((servo_cfg->serv_pres)*8 + 10,sizeof(rmt_symbol_word_t),(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    handle->rmt2 = heap_caps_calloc((servo_cfg->serv_pres)*8 + 10,sizeof(rmt_symbol_word_t),(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    ESP_RETURN_ON_FALSE(handle == NULL || handle->servo_angles == NULL || handle->high_cycles == NULL ||
                        handle->rmt_handles == NULL || handle->LUT1 == NULL || handle->LUT2 == NULL ||
                        handle->streaks_buffer == NULL || handle->rmt1 == NULL || handle->rmt2 == NULL,
                        ESP_ERR_NO_MEM, TAG, "There is not enough memory, please ensure 100 bytes per precision point");

    
    //Initialization
    handle->gpio_array[0] = servo_cfg->clk_gpio;
    handle->gpio_array[1] = servo_cfg->pulse_gpio;
    handle->gpio_array[2] = servo_cfg->servo1_gpio;
    handle->gpio_array[3] = servo_cfg->servo2_gpio;
    handle->servo_pres = servo_cfg->serv_pres;
    handle->running = false;
    handle->task_handle = NULL;
    
    for(int i = 0; i<4; i++) {
        rmt_tx_channel_config_t tx_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = gpio_array[i],
            .mem_block_symbols = 64,
            .resolution_hz = precision_to_clk_freq(handle->servo_pres),
            .trans_queue_depth = 6,
            .flags.invert_out = false,
            .flags.with_dma = false
        };

        ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_config,&(handle->rmt_handles[i])), TAG,"A rmt channel failed to initialize for gpio %d", gpio_array[i]);
        ESP_RETURN_ON_ERROR(rmt_enable(handle->rmt_handles[i]), TAG,"A rmt channel failed to enable");
    }

    //Sync manager initialization
    rmt_sync_manager_config_t sync_cfg = {
        .tx_channel_array = handle->rmt_handles,
        .array_size = 4
    };
    handle->sync_handle = NULL;
    ESP_RETURN_ON_ERROR(rmt_new_sync_manager(&sync_cfg,&(handle->sync_handle)), TAG, "The sync manager failed to initialize");

    //Copy Encoder
    rmt_copy_encoder_config_t copy_cfg = {};
    handle->encoder_handle = NULL;
    rmt_new_copy_encoder(&copy_cfg, &(handle->encoder_handle));

    //Final Steps
    servo_set_all(handle,0.0);
    ESP_LOGI(TAG, "Servo_controller init: success");
    *ret_handle = handle;
    return ESP_OK;
}

//Transmit Servo Task
static void servo_task(void *arg){
    servo_controller_handle_t handle = (servo_controller_handle_t) arg;
    TickType_t last_wake = xTaskGetTickCount();

    calculate_rmt(handle);

    //Delays for known lengths
    rmt_symbol_word_t delay_1_sec = {.level0 = 0, .duration0 = 40*(handle->servo_pres-1), .level1 = 0, .duration1 = 0};
    rmt_symbol_word_t delay_18_sec = {.level0 = 0, .duration0 = 40*(18*(handle->servo_pres)-1), .level1 = 0, .duration1 = 0};

    rmt_transmit_config_t single = {.loop_count = 1, .flags.eot_level = 0, .flags.queue_nonblocking = false};
    rmt_transmit_config_t multiple = {.loop_count = handle->servo_pres+1, .flags.eot_level = 0, .flags.queue_nonblocking = false};

    if(handle->running){//rmt_handles goes clk, pulse, servo1, servo2
        //All channels go high for 1 ms
        rmt_transmit(handle->rmt_handles[0], handle->encoder_handle, clk, 17*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[1], handle->encoder_handle, &pulse, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[2], handle->encoder_handle, &all_high, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[3], handle->encoder_handle, &all_high, 1*sizeof(rmt_symbol_word_t), &single);

        //1ms delay
        rmt_transmit(handle->rmt_handles[0], handle->encoder_handle, &delay_1_sec, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[1], handle->encoder_handle, &delay_1_sec, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[2], handle->encoder_handle, &delay_1_sec, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[3], handle->encoder_handle, &delay_1_sec, 1*sizeof(rmt_symbol_word_t), &single);

        //Transmit rmt_symbols, includes all low at end
        rmt_transmit(handle->rmt_handles[0], handle->encoder_handle, clk, 17*sizeof(rmt_symbol_word_t), &multiple);
        rmt_transmit(handle->rmt_handles[1], handle->encoder_handle, &pulse, 1*sizeof(rmt_symbol_word_t), &multiple);
        rmt_transmit(handle->rmt_handles[2], handle->encoder_handle, handle->rmt1, (handle->rmt_size_1)*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[3], handle->encoder_handle, handle->rmt2, (handle->rmt_size_2)*sizeof(rmt_symbol_word_t), &single);

        //Delay 18ms
        rmt_transmit(handle->rmt_handles[0], handle->encoder_handle, &delay_18_sec, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[1], handle->encoder_handle, &delay_18_sec, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[2], handle->encoder_handle, &delay_18_sec, 1*sizeof(rmt_symbol_word_t), &single);
        rmt_transmit(handle->rmt_handles[3], handle->encoder_handle, &delay_18_sec, 1*sizeof(rmt_symbol_word_t), &single);

        calculate_rmt(handle);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
    
    handle->task_handle = NULL;
    vTaskDelete(NULL);

}

//Start servo controller
esp_err_t servo_start(servo_controller_handle_t handle){
    ESP_RETURN_ON_FALSE(handle == NULL, ESP_ERR_INVALID_ARG,TAG,"Handle is null");
    ESP_RETURN_ON_FALSE(!handle->running, ESP_ERR_INVALID_STATE,TAG,"The controller has already been started");
    
    handle->running = true;

    BaseType_t ret = xTaskCreate(servo_task,"servo_task",CONFIG_SERVO_TASK_STACK_SIZE,handle,CONFIG_SERVO_TASK_PRIORITY,&handle->task_handle);

    if(ret != pdPASS){
        handle->running = false;
        handle->task_handle = NULL;
        ESP_LOGE(TAG,"Failed to create task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Servo controller started");
    return ESP_OK;
}

//Stop the servo controller
esp_err_t servo_stop(servo_controller_handle_t handle){
    ESP_RETURN_ON_FALSE(handle == NULL, ESP_ERR_INVALID_ARG,TAG,"Handle is null");
    ESP_RETURN_ON_FALSE(handle->running, ESP_ERR_INVALID_STATE,TAG,"The controller has already been stopped");

    handle->running = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    handle->task_handle = NULL;
    return ESP_OK;
}

//Remove the servo controller
esp_err_t servo_remove_controller(servo_controller_handle_t handle){
    ESP_RETURN_ON_FALSE(handle == NULL, ESP_ERR_INVALID_ARG,TAG,"The handle is already null");
    ESP_RETURN_ON_FALSE(!handle->running, ESP_ERR_INVALID_STATE,TAG,"The controller is still started");

   
    
    free(handle->LUT1);
    free(handle->LUT2);
    free(handle->rmt1);
    free(handle->rmt2);
    free(handle->streaks_buffer);
    free(handle);
    handle = NULL;

    return ESP_OK;
}

//Set a single servo position
esp_err_t servo_set_servo(servo_controller_handle_t handle, uint8_t servo_num, float servo_position){
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "The handle is null");
    ESP_RETURN_ON_FALSE(servo_num < 32, ESP_ERR_INVALID_ARG, TAG, "%hhu is not a valid servo", servo_num);
    ESP_RETURN_ON_FALSE(servo_position >= 0 && servo_position <= 180, ESP_ERR_INVALID_ARG, TAG, "%.2f is not a valid servo position", servo_position);

    handle->servo_angles[servo_num] = servo_position;
    return ESP_OK;
}

//Set all servo positions
esp_err_t servo_set_all(servo_controller_handle_t handle, float servo_position){
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "The handle is null");
    ESP_RETURN_ON_FALSE(servo_position >= 0 && servo_position <= 180, ESP_ERR_INVALID_ARG, TAG, "%.2f is not a valid servo position", servo_position);

    for(int i = 0; i<32; i++){
        handle->servo_angles[i] = servo_position;
    }
    return ESP_OK;
}

//Calculate the clock frequency based on a level of precision
uint32_t precision_to_clk_freq(servo_precision_t servo_pres){
    return 1000 * servo_pres * 40; //1000 ms/sec * servo_pres cycles/ms * 40 ticks/cycle = servo_pres * 40k ticks/sec
}

//Get the number of cycles a servo will be high based on precision
uint16_t degrees_to_cycles(float degrees, servo_precision_t servo_pres){
    return (uint16_t) (degrees/180.0f*servo_pres + 0.5f);
}

//Calculate and set high_cycles[32] from servo_angles[32]
void calculate_high_cycles(servo_controller_handle_t handle){
    for(int i = 0; i<32; i++){
        handle->high_cycles[i] = degrees_to_cycles(handle->servo_angles[i], handle->servo_pres);
    }
}

//Calculates both LUTs from high_cycles[32], calls LUT_helper
void calculate_LUT(servo_controller_handle_t handle){
    calculate_high_cycles(handle);
    LUT_helper(handle->LUT1,handle->servo_pres,handle->high_cycles,0);
    LUT_helper(handle->LUT2,handle->servo_pres,handle->high_cycles,1);
}

//Fully calculates the lookuptable from inputs
void LUT_helper(uint16_t *LUT, servo_precision_t servo_pres, uint16_t high_cycles[32], uint8_t half){
    uint16_t line;
    for(int i = 0; i<servo_pres; i++){
        line = 0;
        for(int j = 16*half; j<(16*(half+1)); j++){ //Half = 0: 0->15, Half = 1: 16->31
            line |= ((*(high_cycles + j) >= i) ? 1 : 0) << (15-j);
        }
        LUT[i] = line;
    }
    LUT[servo_pres] = 0x0000; //At the end of the 1ms-2ms period, all servos should be 0
}

//Calculates the full RMT values for the next duty cycle, calls rmt helper for 1,2
void calculate_rmt(servo_controller_handle_t handle){
    calculate_LUT(handle);
    rmt_helper(handle->LUT1,handle->servo_pres + 1, handle->rmt1, &(handle->rmt_size_1), handle->streaks_buffer);
    rmt_helper(handle->LUT2,handle->servo_pres + 1, handle->rmt2, &(handle->rmt_size_2), handle->streaks_buffer);
}

//Fully calculates the streaks and converts it to rmt symbols
void rmt_helper(uint16_t *LUT, size_t lut_size, rmt_symbol_word_t *words, size_t *rmt_size, uint8_t *streaks_buffer){
    uint16_t buffer_ind = 0; //Index of buffer, represents streaks of bit 0 or bit 1 based on what buffer_ind % 2 is equal to
    uint8_t streak_length = 0;
    uint16_t mask = 1 << 15;

    for(int i = 0; i<lut_size; i++){
        uint16_t byte = LUT[i];
        while(mask){
            if(((byte & mask) != 0) == (buffer_ind % 2 != 0)){
                streak_length += 2;  //Each bit in the LUT is 2 ticks long
            }else{
                streaks_buffer[buffer_ind++] = streak_length;
                streak_length = 2;
            }
            mask = mask >> 1;
        }
        if(streak_length % 2 == 0){   
            streak_length += 8;     //At the end of the 16 servos, the line is held low for 8 tick
        }else{
            streaks_buffer[buffer_ind++] = streak_length;
            streak_length = 8;
        }
        mask = 1 << 15;
    }
    streaks_buffer[buffer_ind++] = streak_length;
    streaks_buffer[buffer_ind++] = 0;

    for(int i = 0; i<buffer_ind; i+=2){ //Copy streaks_buffer into words
        words[i/2].duration0 = streaks_buffer[i];
        words[i/2].duration1 = streaks_buffer[i+1];
        words[i/2].level0 = 0;
        words[i/2].level1 = 1;
    }
    *rmt_size = buffer_ind/2;
}


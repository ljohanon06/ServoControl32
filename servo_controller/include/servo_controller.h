// SPDX-License-Identifier: MIT
/**
 * @file servo_controller.h
 * @brief Public header file for servo_controller
 * © 2025 Levi Johanon. All rights reserved.
 * See LICENSE for details.
 */

#include "esp_err.h"
#include "driver/gpio.h"



/**
 * @brief Servo precision enum
 */
typedef enum {
    SERVO_PRECISION_50  = 50,       /*!< Servo can be set to 50 different angles */
    SERVO_PRECISION_100 = 100,      /*!< Servo can be set to 100 different angles */
    SERVO_PRECISION_200 = 200,      /*!< Servo can be set to 200 different angles */
    SERVO_PRECISION_500 = 500       /*!< Servo can be set to 500 different angles */
} servo_precision_t;

/**
 * @brief Servo controller config
 */
typedef struct {
    gpio_num_t clk_gpio;            /*!< GPIO that will output the clock signal */
    gpio_num_t pulse_gpio;          /*!< GPIO that will output the pulse signal */
    gpio_num_t servo1_gpio;         /*!< GPIO that will output the first 16 servo's signal */
    gpio_num_t servo2_gpio;         /*!< GPIO that will output the second 16 servo's signal */
    servo_precision_t serv_pres;    /*!< The number of discrete angles the servo can be at */
} servo_controller_config_t;

typedef struct servo_controller_s   *servo_controller_handle_t; /*!< Servo controller handle */


/**
 * @brief Initializes the servo controller
 * 
 * @param[in]   servo_cfg   Servo controller configuration
 * @param[out]  ret_handle  The returned handle for the servo controller
 * @return
 *      - ESP_ERR_NOT_SUPPORTED     rmt or rmt sync manager is not supported on this device (try S3)
 *      - ESP_ERR_INVALID_ARG       An input parameter in servo_cfg is invalid
 *      - ESP_ERR_INVALID_STATE     The handle has already been initialized
 *      - ESP_ERR_NO_MEM            Not enough memory to initialize device
 *      - ESP_OK                    The controller was initialized successfully
 */
esp_err_t servo_initialize_controller(const servo_controller_config_t *servo_cfg, servo_controller_handle_t *ret_handle);

/**
 * @brief Start the servo controller
 * 
 * @param[in]   handle      Handle for the servo controller to be started
 * 
 * @return
 *      - ESP_ERR_INVALID_ARG       The input handle is invalid
 *      - ESP_ERR_INVALID_STATE     The servo controller has already been started
 *      - ESP_FAIL                  The servo task failed to be created
 *      - ESP_OK                    The servo controller was started successfully
 */
esp_err_t servo_start(servo_controller_handle_t handle);

/**
 * @brief Stop the servo controller
 * 
 * @param[in]   handle      Handle for the servo controller to be stopped
 * 
 * @return
 *      - ESP_ERR_INVALID_ARG       The input handle is invalid
 *      - ESP_ERR_INVALID_STATE     The servo controller has already been stopped
 *      - ESP_OK                    The servo controller was stopped successfully
 */
esp_err_t servo_stop(servo_controller_handle_t handle);

/**
 * @brief Remove the servo controller
 * 
 * @param[in]   handle      Handle for the servo controller to be removed
 * 
 * @return
 *      - ESP_ERR_INVALID_ARG       The input handle is invalid
 *      - ESP_ERR_INVALID_STATE     The controller was already removed or the controller is still started
 *      - ESP_OK                    The controller was successfully removed
 */
esp_err_t servo_remove_controller(servo_controller_handle_t handle);

/**
 * @brief Set the angle of a single servo
 * @note If the servo controller is currently started, the servo will begin to during the next 20ms period.
 * 
 * @param[in]   handle          Handle for the servo controller
 * @param[in]   servo_num       Servo to be set.
 *                              Must be between 0 and 31.
 * @param[in]   servo_position  Servo position.
 *                              Will be set to nearest discrete angle based on servo precision.
 *                              Must be between 0 and 180 degrees.
 * 
 * @return
 *      - ESP_ERR_INVALID_ARG       The input is invalid or out of range.
 *      - ESP_OK                    The servo angle has been set
 */
esp_err_t servo_set_servo(servo_controller_handle_t handle, uint8_t servo_num, float servo_position);

/**
 * @brief Set the angle of all 32 servos
 * @note If the servo controller is currently started, the servos will begin to during the next 20ms period.
 * 
 * @param[in]   handle          Handle for the servo controller
 * @param[in]   servo_position  Servo position.
 *                              Will be set to nearest discrete angle based on servo precision.
 *                              Must be between 0 and 180 degrees.
 * 
 * @return
 *      - ESP_ERR_INVALID_ARG       The input is invalid or out of range.
 *      - ESP_OK                    The servo angle has been set
 */
esp_err_t servo_set_all(servo_controller_handle_t handle, float servo_position);
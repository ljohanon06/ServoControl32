// SPDX-License-Identifier: MIT
/**
 * @file servo_internal.h
 * @brief Header folder for internal functions necessary for servo controller
 * © 2025 Levi Johanon. All rights reserved.
 * See LICENSE for details.
 */

#include <stdint.h>
#include "driver/rmt_tx.h"

//Calculate the clock frequency based on a level of precision
uint32_t precision_to_clk_freq(servo_precision_t servo_pres);

//Get the number of cycles a servo will be high based on precision
uint16_t degrees_to_cycles(float degrees, servo_precision_t servo_pres);

//Calculate and set high_cycles[32] from servo_angles[32]
void calculate_high_cycles(servo_controller_handle_t handle);

//Calculates both LUTs from high_cycles[32], calls LUT_helper
void calculate_LUT(servo_controller_handle_t handle);

//Fully calculates the lookuptable from inputs
void LUT_helper(uint16_t *LUT, servo_precision_t servo_pres, uint16_t high_cycles[32], uint8_t half);

//Calculates the full RMT values for the next duty cycle, calls rmt helper for 1,2
void calculate_rmt(servo_controller_handle_t handle);

//Fully calculates the streaks and converts it to rmt symbols
void rmt_helper(uint16_t *LUT, size_t lut_size, rmt_symbol_word_t *words, size_t *rmt_size, uint8_t *streaks_buffer);

//Transmit Servo Task
static void servo_task(void *arg);


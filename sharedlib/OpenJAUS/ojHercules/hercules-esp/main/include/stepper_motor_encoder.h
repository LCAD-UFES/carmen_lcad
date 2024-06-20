/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"
#include "driver/rmt.h"
#include "driver/rmt_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stepper motor curve encoder configuration
 */
typedef struct {
    uint32_t resolution;    // Encoder resolution, in Hz
    uint32_t sample_points; // Sample points used for deceleration phase. Note: |end_freq_hz - start_freq_hz| >= sample_points
    uint32_t start_freq_hz; // Start frequency on the curve, in Hz
    uint32_t end_freq_hz;   // End frequency on the curve, in Hz
} stepper_motor_curve_encoder_config_t;

/**
 * @brief Stepper motor uniform encoder configuration
 */
typedef struct {
    uint32_t resolution; // Encoder resolution, in Hz
    uint32_t sample_points; // Sample points used for uniform phase
    uint32_t freq_hz; // Frequency of the uniform phase, in Hz
} stepper_motor_uniform_encoder_config_t;

/**
 * @brief Create stepper motor curve encoder
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating step motor encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_stepper_motor_curve_encoder(const stepper_motor_curve_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

/**
 * @brief Create RMT encoder for encoding step motor uniform phase into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating step motor encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_stepper_motor_uniform_encoder(const stepper_motor_uniform_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

typedef struct ConfigStepMotor
{
    rmt_channel_handle_t motor_chan;
    rmt_tx_channel_config_t tx_chan_config;
    stepper_motor_curve_encoder_config_t accel_encoder_config; 
    stepper_motor_uniform_encoder_config_t uniform_encoder_config;
    stepper_motor_curve_encoder_config_t decel_encoder_config;
    rmt_encoder_handle_t accel_motor_encoder;
    rmt_encoder_handle_t uniform_motor_encoder;
    rmt_encoder_handle_t decel_motor_encoder;
    rmt_transmit_config_t tx_config;
} ConfigStepMotor;

#ifdef __cplusplus
}
#endif

/**
 ******************************************************************************
 * @file    vl53l5cx_plugin_motion_indicator.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 November 2021
 * @brief   Implementation of the VL53L5CX APIs for motion indicator.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include <math.h>
#include "vl53l5cx_class.h"

uint8_t VL53L5CX::vl53l5cx_motion_indicator_init(
  VL53L5CX_Motion_Configuration *p_motion_config,
  uint8_t       resolution)
{
  uint8_t status = VL53L5CX_STATUS_OK;

  (void)memset(p_motion_config, 0, sizeof(VL53L5CX_Motion_Configuration));

  p_motion_config->ref_bin_offset = 13633;
  p_motion_config->detection_threshold = 2883584;
  p_motion_config->extra_noise_sigma = 0;
  p_motion_config->null_den_clip_value = 0;
  p_motion_config->mem_update_mode = 6;
  p_motion_config->mem_update_choice = 2;
  p_motion_config->sum_span = 4;
  p_motion_config->feature_length = 9;
  p_motion_config->nb_of_aggregates = 16;
  p_motion_config->nb_of_temporal_accumulations = 16;
  p_motion_config->min_nb_for_global_detection = 1;
  p_motion_config->global_indicator_format_1 = 8;
  p_motion_config->global_indicator_format_2 = 0;
  p_motion_config->spare_1 = 0;
  p_motion_config->spare_2 = 0;
  p_motion_config->spare_3 = 0;

  status |= vl53l5cx_motion_indicator_set_resolution(
              p_motion_config, resolution);

  return status;
}

uint8_t VL53L5CX::vl53l5cx_motion_indicator_set_distance_motion(
  VL53L5CX_Motion_Configuration *p_motion_config,
  uint16_t      distance_min_mm,
  uint16_t      distance_max_mm)
{
  uint8_t status = VL53L5CX_STATUS_OK;
  float tmp;

  if (((distance_max_mm - distance_min_mm) > (uint16_t)1500)
      || (distance_min_mm < (uint16_t)400)
      || (distance_max_mm > (uint16_t)4000)) {
    status |= VL53L5CX_STATUS_INVALID_PARAM;
  } else {
    tmp = (float)((((float)distance_min_mm / (float)37.5348)
                   - (float)4.0) * (float)2048.5);
    p_motion_config->ref_bin_offset = (int32_t)tmp;

    tmp = (float)((((((float)distance_max_mm -
                      (float)distance_min_mm) / (float)10.0) + (float)30.02784)
                   / ((float)15.01392)) + (float)0.5);
    p_motion_config->feature_length = (uint8_t)tmp;

    status |= vl53l5cx_dci_write_data(
                (uint8_t *)(p_motion_config),
                VL53L5CX_DCI_MOTION_DETECTOR_CFG,
                (uint16_t)sizeof(*p_motion_config));
  }

  return status;
}

uint8_t VL53L5CX::vl53l5cx_motion_indicator_set_resolution(
  VL53L5CX_Motion_Configuration *p_motion_config,
  uint8_t       resolution)
{
  uint8_t i, status = VL53L5CX_STATUS_OK;

  switch (resolution) {
    case VL53L5CX_RESOLUTION_4X4:
      for (i = 0; i < (uint8_t)VL53L5CX_RESOLUTION_4X4; i++) {
        p_motion_config->map_id[i] = (int8_t)i;
      }
      (void)memset(p_motion_config->map_id + 16, -1, 48);
      break;

    case VL53L5CX_RESOLUTION_8X8:
      for (i = 0; i < (uint8_t)VL53L5CX_RESOLUTION_8X8; i++) {
        p_motion_config->map_id[i] = (int8_t)((((int8_t)
                                                i % 8) / 2) + (4 * ((int8_t)i / 16)));
      }
      break;

    default:
      status |= VL53L5CX_STATUS_ERROR;
      break;
  }

  if (status != (uint8_t)0) {
    status |= vl53l5cx_dci_write_data(
                (uint8_t *)(p_motion_config),
                VL53L5CX_DCI_MOTION_DETECTOR_CFG,
                (uint16_t)sizeof(*p_motion_config));
  }

  return status;
}

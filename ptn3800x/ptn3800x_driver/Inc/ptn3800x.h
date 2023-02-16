/*
 * ptn3800x.h
 *
 *  Created on: Feb 3, 2023
 *      Author: Mateus Pantoja
 */

#ifndef PTN3800X_DRIVER_INC_PTN3800X_H_
#define PTN3800X_DRIVER_INC_PTN3800X_H_

#include "stm32g0xx_hal.h"
#include "stdbool.h"

/*
 * Endereços
 */
#define ADDR_PTN_DFP (0x30 << 1)


/*
 * Set and Unset to datas
 */
#define set_bit(data, bit) ((data)|= 1 << (bit))
#define clear_bit(data, bit)((data)&= ~(1 << (bit)))
#define is_set(data, bit) (data & (1 << bit))


/*
 *
 * Registradores do PTN3800x
 *
 */

#define	CHIP_ID  				0x00
#define	CHIP_REV 				0x01
#define	CHIP_RESERVED 			0x02
#define	FLAT_GAIN_CTRL 			0x03
#define MODE_CTRL 				0x04
#define DEV_CTRL 				0x05
#define DP_LINK_CTRL 			0x06
#define DP_LCTL21_EQ_LANE0 		0x07
#define DP_LCTL3_SWING_LANE0 	0x08
#define DP_LCTL21_EQ_LANE1 		0x09
#define DP_LCTL3_SWING_LANE1 	0x0a
#define DP_LCTL21_EQ_LANE2 		0x0b
#define DP_LCTL3_SWING_LANE2 	0x0c
#define DP_LCTL21_EQ_LANE3 		0x0d
#define DP_LCTL3_SWING_LANE3 	0x0e
#define USB_LOS_DETECTOR 		0x0f
#define USB3_RCTL21_EQ_DOWNS 	0x10
#define USB3_RCTL3_SWING_UPS 		0x11
#define USB3_LCTL21_EQ_UPS 		0x12
#define USB3_LCTL3_SWING_DOWNS	0x13
#define TBT_LINK_CTRL 			0x14
#define TBT_RCTL21_EQ_DOWNS		0x15
#define TBT_RCTL3_SWING_UPS		 	0x16
#define TBT_LCTL21_EQ_UPS	 	0x17
#define TBT_LCTL3_SWING_DOWNS	 	0x18


typedef enum {
	MD_SAFE_STATE = 0,
	MD_SINGLE_USB3,
	MD_USB3_DP2LANE,
	MD_DP4LANE,
	MD_THUNDERBOLT,
	MD_USB4,
} operation_mode;

typedef enum {
	NORMAL = 0,
	REVERSE,
} conector_orient;


typedef struct {
	bool auto_oriention_en;
	bool orientation_done_bit;
	bool plug_orientation_ctrl;
	bool aux_snooping_polarity_bit;
} ptn3800x_orientation_t;

typedef struct {
	uint8_t addr;
	I2C_HandleTypeDef* i2c;
	uint8_t mode;
	ptn3800x_orientation_t orient_handler;
} ptn3800x_HandlerTypedef;

typedef enum {
	TIME_25us = 0,
	TIME_50us,
	TIME_100us,
	TIME_250us,
	TIME_500us,
	TIME_1ms,
	TIME_2ms,
	TIME_5ms,

}time_discon_prog_tbt_usb4_t;

typedef struct{
	bool LT_bypass_control;//[5]
	uint8_t programeble_values_disconect_tbt_usb4; //[4:2]
	bool aux_sb_snooping_mux; //[1]
} dev_ctrl_t;

typedef enum {
	DP_RATE_1DOT62Gbps = 0,
	DP_RATE_2DOT7Gbps,
	DP_RATE_5DOT4Gbps,
	DP_RATE_8DOT1Gbps,
} dp_link_rate_t;

typedef enum {
	DP_0_LANE = 0,
	DP_1_LANE,
	DP_2_LANES,
	DP_4_LANES,
} dp_count_t;

typedef struct{
	bool dp_power_save_en;
	uint8_t dp_lane_count;
	uint8_t dp_link_rate;
} dp_ctrl_and_status_t;

typedef union {
	uint8_t data;
	struct{
		uint8_t eq_data:4,
				swing_data:2;
	} Bits;
}tune_data_t;

typedef union {
	uint8_t data;
	struct {
		uint8_t laneD_tx:1,//LSB
				laneC_tx:1,
				laneA_tx:1,
				laneB_tx:1,
				laneD_rx:1,
				laneB_rx:1; //MSB
	}Bits;
} flat_gain_t;


typedef union {
	uint8_t data;
	struct {
		uint8_t us_LOS_threshold:2,
				select_U2U3_complience:2,
				ds_LOS_threshold:2,
				LOS_det:1,
				LFPS_det:1;
	}Bits;
} los_detector_treshold_t;

/*
 *

 * Functions Prototype
 */

bool ptn3800x_init(ptn3800x_HandlerTypedef *ptn3800x, I2C_HandleTypeDef *i2cHandler);
bool ptn3800x_read_data(ptn3800x_HandlerTypedef *ptn3800x, uint8_t reg, uint8_t *data);
bool ptn3800x_write_data(ptn3800x_HandlerTypedef *ptn3800x, uint8_t reg, uint8_t *data);
bool ptn3800x_operation_mode(ptn3800x_HandlerTypedef *ptn3800x, uint8_t operation);
bool ptn3800x_orientation_config(ptn3800x_HandlerTypedef *ptn3800x, bool orientation);
bool ptn3800x_AUX_pin_polarity(ptn3800x_HandlerTypedef *ptn3800x, bool orientation);
bool ptn3800x_dev_ctrl_get(ptn3800x_HandlerTypedef *ptn3800x, dev_ctrl_t *dev_ctrl_values);
bool ptn3800x_dev_ctrl_set(ptn3800x_HandlerTypedef *ptn3800x, dev_ctrl_t *dev_ctrl_values);
bool ptn3800x_dp_ctrl_get(ptn3800x_HandlerTypedef *ptn3800x, dp_ctrl_and_status_t *dp_ctrl_and_status);
bool ptn3800x_dp_ctrl_set(ptn3800x_HandlerTypedef *ptn3800x, dp_ctrl_and_status_t *dp_ctrl_and_status);
bool ptn3800x_usb_redriver_tune(ptn3800x_HandlerTypedef *ptn3800x, uint8_t mode, tune_data_t *tune_struct);
bool ptn3800x_flat_gain_ctrl(ptn3800x_HandlerTypedef *ptn3800x, flat_gain_t *flat_gain_struct);
bool ptn3800x_los_detector_threshold(ptn3800x_HandlerTypedef *ptn3800x, los_detector_treshold_t *los_threshold_struct);





#endif /* PTN3800X_DRIVER_INC_PTN3800X_H_ */

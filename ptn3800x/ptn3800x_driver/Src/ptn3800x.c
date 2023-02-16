/*
 * ptn3800x.c
 *
 *  Created on: Feb 3, 2023
 *      Author: Mateus Pantoja
 */

#include "ptn3800x.h"
#define I2C_TIMEOUT 100


bool ptn3800x_read_data(ptn3800x_HandlerTypedef *ptn3800x, uint8_t reg, uint8_t *data)
{
	bool ret = false;
	uint8_t buffer = 0x00;

	if(HAL_I2C_Mem_Read(ptn3800x->i2c, ptn3800x->addr, reg, 1, &buffer, 1, I2C_TIMEOUT) != HAL_OK)
		ret = false;
	else
		ret = true;

	*data = buffer;
	return ret;
}

bool ptn3800x_write_data(ptn3800x_HandlerTypedef *ptn3800x, uint8_t reg, uint8_t *data)
{
	bool ret = false;
	uint8_t buffer = 0x00;

	buffer = *data;

	if(HAL_I2C_Mem_Write(ptn3800x->i2c, ptn3800x->addr, reg, 1, &buffer, 1, I2C_TIMEOUT) != HAL_OK)
		ret = false;

	else
		ret = true;

	return ret;
}

bool ptn3800x_orientation_config(ptn3800x_HandlerTypedef *ptn3800x, bool orientation)
{
	bool ret = false;
	uint8_t data = 0x00;

	//Lendo Mode CTRL para verificar se AUTO_ORIENT_EN está Ativo
	if (ptn3800x_read_data(ptn3800x, MODE_CTRL, &data))
	{
		ptn3800x->orient_handler.auto_oriention_en = is_set(data, 7);
		ptn3800x->orient_handler.orientation_done_bit = is_set(data, 6);
		ptn3800x->orient_handler.plug_orientation_ctrl = is_set(data, 4);

		//Verificando se o 7º bit é 1
		if(ptn3800x->orient_handler.auto_oriention_en)
			ret = false; // O pino do Auto_ORIENT está ativo

		else
		{
			if(orientation == NORMAL)
			{
				ptn3800x->orient_handler.plug_orientation_ctrl = NORMAL;
				data |= clear_bit(data, 4);
				ret = ptn3800x_write_data(ptn3800x, MODE_CTRL, &data);
			}
			else if (orientation == REVERSE)
			{
				ptn3800x->orient_handler.plug_orientation_ctrl = REVERSE;
				data |= set_bit(data, 4);
				ret = ptn3800x_write_data(ptn3800x, MODE_CTRL, &data);
			}

		}
	}

	return ret;
}

bool ptn3800x_operation_mode(ptn3800x_HandlerTypedef *ptn3800x, uint8_t operation)
{
	bool ret = false;
	uint8_t data = 0x00;

	ptn3800x_read_data(ptn3800x, MODE_CTRL, &data);
	data &= 0xf8; // garante que os bits [2:0] estao setados e aguardando configuração dos bits restantes
	/*
	 * 	Op Mode 		[2:0] 	-> Safe State, USB3, USB3+DP2LANE, DP4LANE, TBT, USB4
	 */
	switch(operation)
	{
		case MD_SAFE_STATE:
			data &= 0xf8;
			break;

		case MD_SINGLE_USB3:
			data |= set_bit(data, 0); // 0000 0001 = 0x01
			break;

		case MD_USB3_DP2LANE:
			data |= set_bit(data, 1); // 0000 0010 = 0x02
			break;

		case MD_DP4LANE:
			data |= set_bit(data, 1) | set_bit(data, 0); //0000 0010 = 0x03
			break;

		case MD_THUNDERBOLT:
			data |= set_bit(data, 2); //0000 0100 = 0x04
			break;

		case MD_USB4:
			data |= set_bit(data, 2) | set_bit(data, 0); //0000 0101 0x05
			break;

		default:
			//seleção de modo inexistente.
			ret = false;
	}

	ret = ptn3800x_write_data(ptn3800x, MODE_CTRL, &data);
	return ret;
}


bool ptn3800x_init(ptn3800x_HandlerTypedef *ptn3800x, I2C_HandleTypeDef *i2cHandler){

	bool ret = true;

	ptn3800x->addr = ADDR_PTN_DFP;
	ptn3800x->i2c = i2cHandler;
	ptn3800x->mode = MD_SAFE_STATE;

	if (HAL_I2C_IsDeviceReady(ptn3800x->i2c, ADDR_PTN_DFP, 2, I2C_TIMEOUT) != HAL_OK)
		ret = false;

	else
		ret = true;

	return ret;

}

bool ptn3800x_AUX_pin_polarity(ptn3800x_HandlerTypedef *ptn3800x, bool orientation){
	uint8_t ret = false;
	uint8_t data = 0x00;

	if(ptn3800x_read_data(ptn3800x, MODE_CTRL, &data)){
		if(!orientation)
			data &= 0xf7; //garante que o bit [3] seja limpo

		else
			data |= set_bit(data,3) ; //garante que o bit [3] seja limpo

		ptn3800x->orient_handler.aux_snooping_polarity_bit = orientation;
		ret = ptn3800x_write_data(ptn3800x, MODE_CTRL, &data);
	}
	else
		ret = false;


	return ret;
}


bool ptn3800x_dev_ctrl_reset(ptn3800x_HandlerTypedef *ptn3800x){

	uint8_t data = 0x01;
	return ptn3800x_write_data(ptn3800x, DEV_CTRL, &data);
}

bool ptn3800x_dev_ctrl_get(ptn3800x_HandlerTypedef *ptn3800x, dev_ctrl_t *dev_ctrl_values){
	bool ret = false;
	uint8_t data = 0x00;

	if(ptn3800x_read_data(ptn3800x, DEV_CTRL, &data))
	{
		dev_ctrl_values->LT_bypass_control = is_set(data, 5);
		dev_ctrl_values->programeble_values_disconect_tbt_usb4 = (data & 0x1c) >> 2;
		dev_ctrl_values->aux_sb_snooping_mux = is_set(data,1);
		ret = true;
	}

	else
		ret = false;

	return ret;

}

bool ptn3800x_dev_ctrl_set(ptn3800x_HandlerTypedef *ptn3800x, dev_ctrl_t *dev_ctrl_values){
	bool ret = false;
	uint8_t data = 0x00;

	data = (dev_ctrl_values->LT_bypass_control<<5) | (dev_ctrl_values->programeble_values_disconect_tbt_usb4<<2) | (dev_ctrl_values->aux_sb_snooping_mux<<1);

	ret = ptn3800x_write_data(ptn3800x, DEV_CTRL, &data);

	return ret;
}


bool ptn3800x_dp_ctrl_get(ptn3800x_HandlerTypedef *ptn3800x, dp_ctrl_and_status_t* dp_ctrl_and_status){
	bool ret = false;
	uint8_t data = 0x00;


	if(ptn3800x_read_data(ptn3800x, DP_LINK_CTRL, &data))
	{
		dp_ctrl_and_status->dp_power_save_en = is_set(data, 4);
		dp_ctrl_and_status->dp_lane_count = (data & 0x0c) >> 2;
		dp_ctrl_and_status->dp_link_rate = (data & 0x03);
		ret = true;
	}

	else
		ret = false;


	return ret;

}

bool ptn3800x_dp_ctrl_set(ptn3800x_HandlerTypedef *ptn3800x, dp_ctrl_and_status_t* dp_ctrl_and_status){
	bool ret = false;
	uint8_t data = 0x00;

	data = (dp_ctrl_and_status->dp_power_save_en << 4) | (dp_ctrl_and_status->dp_lane_count << 2) | (dp_ctrl_and_status->dp_link_rate);

	ret = ptn3800x_write_data(ptn3800x, DP_LINK_CTRL, &data);

	return ret;
}


bool ptn3800x_usb_redriver_tune(ptn3800x_HandlerTypedef *ptn3800x, uint8_t mode, tune_data_t *tune_values){
	bool ret = false;
	uint8_t data = 0x00;

	data = tune_values->data;


	switch(mode)
	{

		case MD_SINGLE_USB3:

			ptn3800x_write_data(ptn3800x, USB3_LCTL21_EQ_UPS, &data);
			ptn3800x_write_data(ptn3800x, USB3_RCTL21_EQ_DOWNS, &data);

			data = data >> 4;
			ptn3800x_write_data(ptn3800x, USB3_RCTL3_SWING_UPS, &data);
			ptn3800x_write_data(ptn3800x, USB3_LCTL3_SWING_DOWNS, &data);
			ret = true;
			break;

		case MD_DP4LANE:

			ptn3800x_write_data(ptn3800x, DP_LCTL21_EQ_LANE0, &data);
			ptn3800x_write_data(ptn3800x, DP_LCTL21_EQ_LANE1, &data);
			ptn3800x_write_data(ptn3800x, DP_LCTL21_EQ_LANE2, &data);
			ptn3800x_write_data(ptn3800x, DP_LCTL21_EQ_LANE3, &data);

			data = data >> 4;
			ptn3800x_write_data(ptn3800x, DP_LCTL3_SWING_LANE0, &data);
			ptn3800x_write_data(ptn3800x, DP_LCTL3_SWING_LANE1, &data);
			ptn3800x_write_data(ptn3800x, DP_LCTL3_SWING_LANE2, &data);
			ptn3800x_write_data(ptn3800x, DP_LCTL3_SWING_LANE3, &data);
			ret = true;
			break;

		case MD_THUNDERBOLT | MD_USB4:
			ptn3800x_write_data(ptn3800x, TBT_LCTL21_EQ_UPS, &data);
			ptn3800x_write_data(ptn3800x, TBT_RCTL21_EQ_DOWNS, &data);

			data = data >> 4;
			ptn3800x_write_data(ptn3800x, TBT_RCTL3_SWING_UPS, &data);
			ptn3800x_write_data(ptn3800x, TBT_LCTL3_SWING_DOWNS, &data);
			ret = true;
			break;

		default:
			//seleção de modo inexistente.
			ret = false;
	}

	return ret;
}

bool ptn3800x_flat_gain_ctrl(ptn3800x_HandlerTypedef *ptn3800x, flat_gain_t *flat_gain_struct){
	bool ret = false;

	uint8_t data = 0x00;

	data = flat_gain_struct->data;

	ret = ptn3800x_write_data(ptn3800x, FLAT_GAIN_CTRL, &data);

	return ret;
}

bool ptn3800x_los_detector_threshold(ptn3800x_HandlerTypedef *ptn3800x, los_detector_treshold_t *los_threshold_struct){
	bool ret = false;

	uint8_t data = 0x00;

	if(ptn3800x_operation_mode(ptn3800x, MD_SAFE_STATE))
	{
		data = los_threshold_struct->data;
		ret = ptn3800x_write_data(ptn3800x, USB_LOS_DETECTOR, &data);
	}

	else
		ret = false;

	return ret;

}

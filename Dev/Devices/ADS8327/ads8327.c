/*
 * ads8327.c
 *
 *  Created on: Mar 2, 2025
 *      Author: DELL
 */
#include "ads8327.h"
#include "main.h"



static uint16_t CFR_user_default = 0x0EFD;		//0000 1110 1111 1101

static void ADS8327_Transmit16(ADS8327_Device_t *dev, uint16_t data)
{
	while (!LL_SPI_IsActiveFlag_TXE(dev->spi));

	LL_SPI_TransmitData8(dev->spi, (uint8_t)(data>>8));
	while (!LL_SPI_IsActiveFlag_TXE(dev->spi));  // Đợi TXE
	while (LL_SPI_IsActiveFlag_BSY(dev->spi));   // Đợi
	(void)LL_SPI_ReceiveData8(dev->spi);

	LL_SPI_TransmitData8(dev->spi, (uint8_t)data);
	while (!LL_SPI_IsActiveFlag_TXE(dev->spi));  // Đợi TXE
	while (LL_SPI_IsActiveFlag_BSY(dev->spi));   // Đợi BSY
	(void)LL_SPI_ReceiveData8(dev->spi);

}

static uint16_t ADS8327_Receive16(ADS8327_Device_t *dev)
{
	uint16_t data1 = 0;
	uint16_t data2 = 0;

	while (!LL_SPI_IsActiveFlag_TXE(dev->spi));
	LL_SPI_TransmitData8(dev->spi, 0xAA);
	while (LL_SPI_IsActiveFlag_BSY(dev->spi));

	while (!LL_SPI_IsActiveFlag_RXNE(dev->spi));
	data1 = LL_SPI_ReceiveData8(dev->spi);
	while (LL_SPI_IsActiveFlag_BSY(dev->spi));

	while (!LL_SPI_IsActiveFlag_TXE(dev->spi));
	LL_SPI_TransmitData8(dev->spi, 0xAA);
	while (LL_SPI_IsActiveFlag_BSY(dev->spi));

	while (!LL_SPI_IsActiveFlag_RXNE(dev->spi));
	data2 = LL_SPI_ReceiveData8(dev->spi);
	while (LL_SPI_IsActiveFlag_BSY(dev->spi));

	return (data1 << 8)|data2;
}

void ADS8327_Wake_Up(ADS8327_Device_t *dev)
{
	dev->CMD = ADS8327_CMD_WAKE_UP;
    LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin);

    ADS8327_Transmit16(dev, dev->CMD);

    LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);
}

void ADS8327_Write_CFR(ADS8327_Device_t *dev, uint16_t CFR)
{
	LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin);

	dev->CMD = ADS8327_CMD_WRITE_CONFIG;
	dev->CFR_value = CFR;
	uint16_t temp = (dev->CMD & 0xF000) | (dev->CFR_value & 0x0FFF);

	ADS8327_Transmit16(dev, temp);

	LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);
}


void ADS8327_Default_CFR(ADS8327_Device_t *dev, CFR_default_t CFR_default)
{
	if (CFR_default == USER_DEFAULT)
	{
		ADS8327_Write_CFR(dev, CFR_user_default);
	}

	else if (CFR_default == FACTORY_DEFAULT)
	{
		LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin);
		dev->CMD = ADS8327_CMD_DEFAULT_MODE;
		dev->CFR_value = ADS8327_FACTORY_CFR_DEFAULT;
		ADS8327_Transmit16(dev, dev->CMD);
		LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);
	}
}


uint16_t ADS8327_Read_Data_Polling(ADS8327_Device_t *dev, int timeout)
{
	LL_GPIO_ResetOutputPin(dev->convst_port, dev->convst_pin);
	int time = timeout;
	while(LL_GPIO_IsInputPinSet(dev->EOC_port, dev->EOC_pin) && (time--));
	time = timeout;
	while(!LL_GPIO_IsInputPinSet(dev->EOC_port, dev->EOC_pin) && (time--));

	LL_GPIO_SetOutputPin(dev->convst_port, dev->convst_pin);

	LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin);

	dev->ADC_val = ADS8327_Receive16(dev);

	while (LL_SPI_IsActiveFlag_BSY(dev->spi));
	LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);

	return dev->ADC_val;
}

/*
void ISR_TIMTrigger_IRQHandler(ADS8327_Device_t *dev)
{
	LL_GPIO_SetOutputPin(dev->convst_port, dev->convst_pin);
	LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin);
	LL_SPI_EnableIT_TXE(dev->spi);
	LL_SPI_TransmitData8(dev->spi, 0xAA); 	// Transmit dummy
	dev->tran_ind = 1;

	// Lay data ra****************************************



}

void ISR_SPI_IRQHandler(ADS8327_Device_t *dev)
{
	// Kiểm tra cờ ngắt TXE
	if (LL_SPI_IsActiveFlag_TXE(dev->spi) && LL_SPI_IsEnabledIT_TXE(dev->spi))
	{
		if(dev->tran_ind < 2)
		{
			dev->buf[dev->tran_ind-1] = LL_SPI_ReceiveData8(dev->spi);
			LL_SPI_TransmitData8(dev->spi, 0xAA); 	// Transmit dummy
			dev->tran_ind++;
		}
		else
		{
			//Trigger convst
			LL_GPIO_ResetOutputPin(dev->convst_port, dev->convst_pin);
			dev->buf[dev->tran_ind-1] = LL_SPI_ReceiveData8(dev->spi);
			LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);
			dev->ADC_val = (dev->buf[0] << 8) | (dev->buf[1]);
			LL_SPI_DisableIT_TXE(dev->spi);
			dev->tran_ind = 0;
		}
	}
}
*/

void ISR_TIMTrigger_IRQHandler(ADS8327_Device_t *dev)
{
	WRITE_REG(PHOTO_ADC_CS_GPIO_Port->BSRR, PHOTO_ADC_CS_Pin);
	WRITE_REG(PHOTO_ADC_CONV_GPIO_Port->BSRR, PHOTO_ADC_CONV_Pin << 16);
	WRITE_REG(PHOTO_ADC_CONV_GPIO_Port->BSRR, PHOTO_ADC_CONV_Pin);
	SPI2->DR = 0xAAAA;
	WRITE_REG(PHOTO_ADC_CS_GPIO_Port->BSRR, PHOTO_ADC_CS_Pin << 16);
	WRITE_REG(TIM1->SR, ~(TIM_SR_UIF));

}

void ISR_SPI_DMA_IRQHandler(ADS8327_Device_t *dev)
{
	//WRITE_REG(PD_CS_GPIO_Port->BSRR, PD_CS_Pin);
	WRITE_REG(DMA1->LIFCR , DMA_LIFCR_CTCIF3);
	CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
	CLEAR_BIT(TIM1->DIER, TIM_DIER_UIE);
	//LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)adc_ptr++);
	//LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
}

void SPI_Tranceiver_IT(ADS8327_Device_t *dev)
{
	LL_SPI_TransmitData8(dev->spi, 0xAA); 	// Transmit dummy
	LL_SPI_EnableIT_TXE(dev->spi);
	dev->tran_ind = 1;
}

void SPI_SetPrescaler(SPI_TypeDef *spi, uint32_t Prescaler)
{
    // Kiểm tra giá trị prescaler hợp lệ
    if ((Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV2) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV4) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV8) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV16) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV32) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV64) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV128) &&
        (Prescaler != LL_SPI_BAUDRATEPRESCALER_DIV256))
    {
        // Xử lý lỗi nếu prescaler không hợp lệ
        return;
    }
    // Vô hiệu hóa SPIx
    LL_SPI_Disable(spi);
    // Cấu hình lại prescaler
    LL_SPI_SetBaudRatePrescaler(spi, Prescaler);
    // Kích hoạt lại SPIx
    LL_SPI_Enable(spi);
}

// Hàm thay đổi data length của SPIx
void SPI_SetDataLength(SPI_TypeDef *spi, uint32_t DataWidth)
{
    // Kiểm tra giá trị DataWidth hợp lệ
    if ((DataWidth != LL_SPI_DATAWIDTH_8BIT)  &&
        (DataWidth != LL_SPI_DATAWIDTH_16BIT))
    {
        // Xử lý lỗi nếu DataWidth không hợp lệ
        return;
    }
    // Vô hiệu hóa SPIx để thay đổi cấu hình
    LL_SPI_Disable(spi);
    // Cập nhật Data Width
    LL_SPI_SetDataWidth(spi, DataWidth);
    // Kích hoạt lại SPIx
    LL_SPI_Enable(spi);
}


void ADS8327_Device_Init(	ADS8327_Device_t *dev,
							SPI_TypeDef *spi,
							GPIO_TypeDef *cs_port,
							uint32_t cs_pin,
							GPIO_TypeDef *convst_port,
							uint32_t convst_pin,
							GPIO_TypeDef *EOC_port,
							uint32_t EOC_pin)
{

	dev->spi = spi;
	dev->cs_port = cs_port;
	dev->cs_pin = cs_pin;
	dev->convst_port = convst_port;
	dev->convst_pin = convst_pin;
	dev->EOC_port = EOC_port;
	dev->EOC_pin = EOC_pin;
	dev->ADC_val = 0xFFFF;
	dev->tran_ind = 0;

	while (!LL_SPI_IsEnabled(dev->spi))
	{
		LL_SPI_Enable(dev->spi);
		__NOP();
	}

	ADS8327_Default_CFR(dev, USER_DEFAULT);
	ADS8327_Wake_Up(dev);
}








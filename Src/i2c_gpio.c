#include "stm32f3xx_hal.h"
#include "i2c_gpio.h"

void f103_72m_delay_I2C(void) //STM32F103,72MHz, delay 2.5us 
{
	volatile uint32_t count = 18;
	while(count--); 
}

uint8_t I2C_GPIO_Init(void) //I2C GPIO Configuration
{
	GPIO_InitTypeDef I2C_GPIO_InitStruct;
	
	//GPIO Ports Clock Enable
	if( (I2C_SCK_OUT_PORT == GPIOA) | (I2C_SDA_OUT_PORT == GPIOA) | (I2C_SDA_IN_PORT == GPIOA) )
		__HAL_RCC_GPIOA_CLK_ENABLE();
	
	if( (I2C_SCK_OUT_PORT == GPIOB) | (I2C_SDA_OUT_PORT == GPIOB) | (I2C_SDA_IN_PORT == GPIOB) )
		__HAL_RCC_GPIOB_CLK_ENABLE();
	
	if( (I2C_SCK_OUT_PORT == GPIOC) | (I2C_SDA_OUT_PORT == GPIOC) | (I2C_SDA_IN_PORT == GPIOC) )
		__HAL_RCC_GPIOC_CLK_ENABLE();
	
	if( (I2C_SCK_OUT_PORT == GPIOD) | (I2C_SDA_OUT_PORT == GPIOD) | (I2C_SDA_IN_PORT == GPIOD) )
		__HAL_RCC_GPIOD_CLK_ENABLE();
	
	if( (I2C_SCK_OUT_PORT == GPIOE) | (I2C_SDA_OUT_PORT == GPIOE) | (I2C_SDA_IN_PORT == GPIOE) )
		__HAL_RCC_GPIOE_CLK_ENABLE();
	
//	if( (I2C_SCK_OUT_PORT == GPIOF) | (I2C_SDA_OUT_PORT == GPIOF) | (I2C_SDA_IN_PORT == GPIOF) )
//		__HAL_RCC_GPIOF_CLK_ENABLE();
//	
//	if( (I2C_SCK_OUT_PORT == GPIOG) | (I2C_SDA_OUT_PORT == GPIOG) | (I2C_SDA_IN_PORT == GPIOG) )
//		__HAL_RCC_GPIOG_CLK_ENABLE();
	
	// GPIO pin Output Level : High
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	
	// Configure GPIO pins : SCK_OUT
	I2C_GPIO_InitStruct.Pin = I2C_SCK_OUT_PIN;
	I2C_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	I2C_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C_SCK_OUT_PORT, &I2C_GPIO_InitStruct);
	
	// Configure GPIO pins : SDA_OUT
	I2C_GPIO_InitStruct.Pin = I2C_SDA_OUT_PIN;
	I2C_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	I2C_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C_SDA_OUT_PORT, &I2C_GPIO_InitStruct);
	
	//SDA 핀이 풀업 되었는지 확인과정
	//SDA 핀 입력을 내부 풀 다운 한 후 
	//하이 임피던스 입력 상태로 변경 후
	//핀 레벨을 읽어 본다
	// Configure GPIO pin : SDA_IN
	I2C_GPIO_InitStruct.Pin = I2C_SDA_IN_PIN;
	I2C_GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	I2C_GPIO_InitStruct.Pull = GPIO_PULLDOWN; 
	HAL_GPIO_Init(I2C_SDA_IN_PORT, &I2C_GPIO_InitStruct);
	
	HAL_Delay(1);
	
	I2C_GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(I2C_SDA_IN_PORT, &I2C_GPIO_InitStruct);
	
	//Checking GPIO Pull-up status
	if ( HAL_GPIO_ReadPin(I2C_SDA_IN_PORT, I2C_SDA_IN_PIN) == 1 )
	{
		return 1;
	}
	else
	{
		HAL_GPIO_DeInit(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN);
		HAL_GPIO_DeInit(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN);
		HAL_GPIO_DeInit(I2C_SDA_IN_PORT, I2C_SDA_IN_PIN);
		return 0;
	}
}

void I2C_Start(void) //I2C Start condition
{
	// SDA  -.  
	//     ~ |  
	//       '--
	// SCK ---. 
	//        | 
	//        '-
	
	// 1.All High (Dummy stop)
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 2.SDA Low
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
	
	// 3.SCK Low
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
}

void I2C_ReStart(void) //I2C Repeated Start condition
{
	// SDA --.
	//       |
	//       '--
	// SCK  .-. 
	//      | |  
	//     -' '-
	
	//HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	
	// 1.SDA High
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 2.SCK High
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 3.SDA Low
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
	
	// 4.SCK Low
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
}

void I2C_Stop(void) //I2C Stop condition
{
	// SDA   .-
	//       | 
	//     --' 
	// SCK  .--
	//      |  
	//     -'  
	
	// 1. All Low
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
	
	// 2. SCK High
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 3. SDA High
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
}

void I2C_DATA_Set(uint8_t data) //I2C Data to slave
{
	// SDA .----.   .----. 
	//     : MSB: ~ : LSB:
	//     '----'   '----' 
	// SCK  .--.     .--.  
	//      |  |  ~  |  |  
	//     -'  '-   -'  '- 
	
	uint8_t i;
	
	//HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	
	for(i=0; i<8; i++)
	{
		// SDA Data 1bit
		if((data << i) & 0x80)
			HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_RESET);
		f103_72m_delay_I2C();
		
		// SCK High
		HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
		f103_72m_delay_I2C();
		
		// SCK Delay
		f103_72m_delay_I2C();
		
		// SCK Low
		HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
		f103_72m_delay_I2C();
	}
}

uint8_t I2C_DATA_Get(void) //I2C Data from slave
{
	// SDA .-- --.   .-- --.
	//     |  ^    ~    ^
	//     ' MSB       LSB
	// SCK  .- -.     .- -. 
	//      |   |  ~  |   |
	//     -'   '-   -'   '-
	
	uint8_t i;
	uint8_t data = 0;
	
	//HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	
	// 1.SDA Hi-Z
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	
	
	for(i=0; i<8; i++)
	{
		data <<= 1;
		
		// SCK Delay
		f103_72m_delay_I2C();
		
		// SCK High
		HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
		f103_72m_delay_I2C();
		
		// SDA Data 1bit Read
		if( HAL_GPIO_ReadPin(I2C_SDA_IN_PORT, I2C_SDA_IN_PIN) )
			data |= 0x01;
			
		f103_72m_delay_I2C();
		
		// SCK Low
		HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
		f103_72m_delay_I2C();
	}
	
	return data;
}

uint8_t I2C_ACK_Get(void) //I2C ACK from slave
{
	// SDA .-- --.
	//     |  ^  
	//     '    
	// SCK  .- -. 
	//      |   | 
	//     -'   '-
	
	uint8_t ACK_Level;
	
	//HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	
	// 1.SDA Hi-Z
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 2.SCK High
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 3.Slave ACK check
	ACK_Level = HAL_GPIO_ReadPin(I2C_SDA_IN_PORT, I2C_SDA_IN_PIN);
	f103_72m_delay_I2C();
	
	// 4.SCK Low
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
	
	if(ACK_Level == 1)
		return 0;
	else
		return 1;
}

void I2C_NACK_Set(void) //I2C NACK to slave
{
	// SDA .----.
	//     |    
	//     '    
	// SCK  .--. 
	//      |  | 
	//     -'  '-
	
	//HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	
	// 1.SDA High
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 2.SCK High
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 3.SCK Delay
	f103_72m_delay_I2C();
	
	// 4.SCK Low
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
}

void I2C_ACK_Set(void) //I2C ACK to slave
{
	// SDA .
	//     |    
	//     '----'    
	// SCK  .--. 
	//      |  | 
	//     -'  '-
	
	//HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	
	// 1.SDA LOW
	HAL_GPIO_WritePin(I2C_SDA_OUT_PORT, I2C_SDA_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
	
	// 2.SCK High
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_SET);
	f103_72m_delay_I2C();
	
	// 3.SCK Delay
	f103_72m_delay_I2C();
	
	// 4.SCK Low
	HAL_GPIO_WritePin(I2C_SCK_OUT_PORT, I2C_SCK_OUT_PIN, GPIO_PIN_RESET);
	f103_72m_delay_I2C();
}

uint8_t I2C_EEP_Write(uint8_t DevAddress, uint16_t WordAddress, uint8_t WordAddSize, uint8_t *pData, int16_t DataSize)
{
	int16_t i;
	
	//분할 컴파일 환경에서는
	//매개변수로 받은 배열의 크기 확인이 불가능,
	//(이유: sizeof 함수의 실행시점)
	//uint16_t PtrSize = (sizeof(pData) / sizeof(*pData));
	//if ( PtrSize < DataSize ) //Check Array Size
	//	DataSize = PtrSize;
	
	// 1.Start
	I2C_Start();
	
	// 2.Device Addres + Write
	I2C_DATA_Set(DevAddress & 0xFE);
	
	// 3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	// 4.Word Address
	//   First Word Address
	if(WordAddSize > 1) 
	{
		I2C_DATA_Set(WordAddress >> 8); //First Word Address
		
		//ACK
		if( I2C_ACK_Get() == 0 )
		{
			I2C_Stop();
			return 0;
		}
	}
	//   Second Word Address
	I2C_DATA_Set(WordAddress & 0xFF);
	
	// 5.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	for(i=0; i<DataSize; i++)
	{
		// 6.DATA Write
		I2C_DATA_Set( *(pData + i) );
		
		// 7.ACK
		if( I2C_ACK_Get() == 0 )
		{
			I2C_Stop();
			return 0;
		}
	}
	
	// 8.Stop
	I2C_Stop();
	
	// 9.Wait more 5ms
	HAL_Delay(6);
	
	return 1;
}

uint8_t I2C_EEP_Read(uint8_t DevAddress, uint16_t WordAddress, uint8_t WordAddSize, uint8_t *pData, int16_t DataSize)
{
	int16_t i;
	
	// 1. Dummy Write
	// 1-1.Start
	I2C_Start();
	
	// 1-2.Device Addres + Write
	I2C_DATA_Set(DevAddress & 0xFE);
	
	// 1-3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	// 1-4.Word Address
	//   First Word Address
	if(WordAddSize > 1) 
	{
		I2C_DATA_Set(WordAddress >> 8); //First Word Address
		
		//ACK
		if( I2C_ACK_Get() == 0 )
		{
			I2C_Stop();
			return 0;
		}
	}
	//   Second Word Address
	I2C_DATA_Set(WordAddress & 0xFF);
	
	// 1-5.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	// 2.DATA Read
	// 2-1 Repeated Start
	I2C_ReStart();
	
	// 2-2.Device Addres + Read
	I2C_DATA_Set(DevAddress | 0x01);
	
	// 2-3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	for(i=0; i<DataSize; i++)
	{
		// 2-4.DATA Read
		*(pData + i) = I2C_DATA_Get();
		
		// 2-5.ACK
		if(i < DataSize-1)
			I2C_ACK_Set();
		else
			I2C_NACK_Set();
	}
	
	// 3.Stop
	I2C_Stop();
	
	return 1;
}

uint8_t I2C_EEP_Addr_Set(uint8_t DevAddress, uint16_t WordAddress, uint8_t WordAddSize)
{
	// 1.Start
	I2C_Start();

	
	// 2.Device Addres + Write
	I2C_DATA_Set(DevAddress & 0xFE);
	
	// 3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	// 4.Word Address
	//   First Word Address
	if(WordAddSize > 1) 
	{
		I2C_DATA_Set(WordAddress >> 8); //First Word Address
		
		//ACK
		if( I2C_ACK_Get() == 0 )
		{
			I2C_Stop();
			return 0;
		}
	}
	//   Second Word Address
	I2C_DATA_Set(WordAddress & 0xFF);
	
	// 5.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	// 6.Stop
	I2C_Stop();
	
	return 1;
}

uint8_t I2C_ADDW_Only(uint8_t DevAddress)
{
	// 1.Start
	I2C_Start();
	
	// 2.Device Addres + Write
	I2C_DATA_Set(DevAddress & 0xFE);
	
	// 3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	// 4.Stop
	I2C_Stop();
	
	return 1;
}

uint8_t I2C_Write(uint8_t DevAddress, uint8_t *pData, int16_t DataSize)
{
	int16_t i;
	
	// 1.Start
	I2C_Start();
	
	// 2.Device Addres + Write
	I2C_DATA_Set(DevAddress & 0xFE);
	
	// 3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	for(i=0; i<DataSize; i++)
	{
		// 4.DATA Write
		I2C_DATA_Set( *(pData + i) );
		
		// 5.ACK
		if( !I2C_ACK_Get() )
		{
			I2C_Stop();
			return 0;
		}
	}
	
	// 6.Stop
	I2C_Stop();
	
	return 1;
}

uint8_t I2C_Read(uint8_t DevAddress, uint8_t *pData, int16_t DataSize)
{
	int16_t i;
	
	// 1.Start
	I2C_Start();
	
	// 2.Device Addres + Read
	I2C_DATA_Set(DevAddress | 0x01);
	
	// 3.ACK
	if( I2C_ACK_Get() == 0 ) 
	{
		I2C_Stop();
		return 0;
	}
	
	for(i=0; i<DataSize; i++)
	{
		// 3.DATA Read
		*(pData + i) = I2C_DATA_Get();
		
		// 4.ACK
		if(i < DataSize-1)
			I2C_ACK_Set();
		else
			I2C_NACK_Set();
	}
	
	// 5.Stop
	I2C_Stop();
	
	return 1;
}

#define I2C_SCK_OUT_PORT GPIOC
#define I2C_SCK_OUT_PIN  GPIO_PIN_0

#define I2C_SDA_OUT_PORT GPIOC
#define I2C_SDA_OUT_PIN  GPIO_PIN_1

#define I2C_SDA_IN_PORT  GPIOC
#define I2C_SDA_IN_PIN   GPIO_PIN_2

void f103_72m_delay_I2C(void); //STM32F103,72MHz, delay 2.5us 
uint8_t I2C_GPIO_Init(void);      //I2C GPIO Configuration

void I2C_Start(void);          //I2C Start condition
void I2C_ReStart(void);        //I2C Repeated Start condition
void I2C_Stop(void);           //I2C Stop condition

void I2C_DATA_Set(uint8_t data); //I2C Data to slave
uint8_t I2C_DATA_Get(void);      //I2C Data from slave

uint8_t I2C_ACK_Get(void); //I2C ACK from slave
void I2C_NACK_Set(void);   //I2C NACK to slave
void I2C_ACK_Set(void);    //I2C ACK to slave

//I2C EEPROM Write
uint8_t I2C_EEP_Write(uint8_t DevAddress, uint16_t WordAddress, uint8_t WordAddSize, uint8_t *pData, int16_t DataSize);
//I2C EEPROM Read
uint8_t I2C_EEP_Read(uint8_t DevAddress, uint16_t WordAddress, uint8_t WordAddSize, uint8_t *pData, int16_t DataSize);
//I2C EEPROM WordAddres Set
uint8_t I2C_EEP_Addr_Set(uint8_t DevAddress, uint16_t WordAddress, uint8_t WordAddSize);

//I2C Addres+Write
uint8_t I2C_ADDW_Only(uint8_t DevAddress);
//I2C Write
uint8_t I2C_Write(uint8_t DevAddress, uint8_t *pData, int16_t DataSize);
//I2C Read
uint8_t I2C_Read(uint8_t DevAddress, uint8_t *pData, int16_t DataSize);

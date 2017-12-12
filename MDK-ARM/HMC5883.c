#include "stm32f3xx_hal_rcc.h"

void HMC5883_Init() {
	
	uint8_t temp = (0x03 << 5) | (0x10 << 4) 	//Register A 설정 샘플값 8개, 데이터 출력 속도 15Hz, 노말모드
}
###hellloworld!
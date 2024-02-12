#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_exti.h"
#include "stdio.h"
#include "stm32l1xx_ll_dac.h"

#include "math.h"
#include "dwt_delay.h"

void SystemClock_Config(void);

/**------------------------** Display function **------------------------**/
void display(char seg[4]);
void SEG_Config(void);
void LED_Config(void);
char disp_str[4];
uint8_t i,k;


/**------------------------** Sensor function **------------------------**/
void SensorGPIO_Config(void);
void SensorTIM_IC_Config(void);
float period;
uint16_t IC1 = 0;
uint16_t IC2 = 0;
uint16_t Diff = 0;
uint16_t state = 0;
uint32_t TIM2CLK;
uint32_t PSC;
uint32_t IC1PSC;
uint32_t level, dist;


/**------------------------** Notification function **------------------------**/
#define TIMx_PSC 2
#define C				(uint16_t)1047
#define D				(uint16_t)1175
#define E				(uint16_t)1318
#define F				(uint16_t)1397
#define G				(uint16_t)1568
#define M				(uint16_t)1000000

#define ARR_CAL(N) ( 32000000 / (TIMx_PSC * N))

void Notic_TIM_BASE_Config(uint16_t);
void Notic_TIM_OC_GPIO_Config(void);
void Notic_TIM_OC_Config(uint16_t);
void TIM_Duration(uint16_t);

uint16_t note[] = {M, E, M, E, M, E, M, E, M, E, M, E, M, E, G, M, C, M, D, M, E, M, 
									 F, M, F, M, F, M, E, M, E, M, E, M, D, M, D, M, D, M, E, M, D, M, G, M, 
									 E, M, E, M, E, M, E, M, E, M, E, M, E, M, G, M, C, M, D, M, E, M, 
									 F, M, F, M, F, M, E, M, E, M, E, M, G, M, G, M, F, M, D, M, C};
	
int inDex= 0;
int size = sizeof(note) / sizeof(note[0]);


/**------------------------** Coins classify function **------------------------**/
void Coins_Config(void);
void one_check(void);
void two_check(void);
void five_check(void);
void ten_check(void);
//void Coins_check(void);
uint16_t one_b_state = 0, one_b = 0, two_b_state = 0, two_b = 0, five_b_state = 0, five_b = 0, ten_b_state = 0, ten_b = 0;
uint16_t balance_b = 0, test = 0;


/**------------------------** Pump function **------------------------**/
void water_TIMBase(void);			// Timer for measurement water level per 1 sec
void pump_Config(void);				// button, function to change 'pump_state' using EXTI0
uint16_t pump_state = 0, machine_status = 0, water_cnt; 			// pump 0 is off, 1 is on


int main()
{
	SystemClock_Config();
	SensorGPIO_Config();
	SensorTIM_IC_Config();
	SEG_Config();
	LED_Config();
	Coins_Config();
	water_TIMBase();
	pump_Config();
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
	
	while(1)
	{
		
			/*-------------///--------///---------- Music ----------///--------///-------------*/

			Notic_TIM_OC_Config(ARR_CAL(note[inDex]));
			TIM_Duration(20);

			while( inDex < size )
			{
					if(LL_TIM_IsActiveFlag_UPDATE(TIM3) == SET)
					{
						if(note[inDex+1] == M)
						{
							TIM_Duration(20);
						}
						else
						{
							TIM_Duration(80);
						}
						
						++inDex;
						LL_TIM_ClearFlag_UPDATE(TIM3);
						Notic_TIM_OC_Config(ARR_CAL(note[inDex]));
						LL_TIM_SetCounter(TIM3, 0);
				
					}
		
					
					/*-------------///--------///---------- Ultrasonic Sensor ----------///--------///-------------*/
					
					if( balance_b == 0 )
					{
						pump_state = 0;				// reset pump to 0
						machine_status = 0;		// deboucing relay, forced to disable PA3
						LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);		// disable PA3 (relay)
						
						LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
						LL_mDelay(1);
						LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
						
						sprintf(disp_str, "%04d", level);
						display(disp_str);
						
						if(state==2)
						{
							PSC = LL_TIM_GetPrescaler(TIM2) + 1;
							TIM2CLK = SystemCoreClock / PSC;
							IC1PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(TIM2, LL_TIM_CHANNEL_CH2));
							period = (Diff*(PSC) * 1.0) / (TIM2CLK*IC1PSC*1.0);
							dist = (period*170) * 100;
							level = ((26 -  dist) * 100 ) / 26;	
							state = 0;
						}
					}
					
			
					// Check water level
					if( level >= 30  )  
					{
						machine_status = 1;		// enable PA3, could change pump_state
						LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); 	// LED Green ON 'machine available'
						LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
						
						// check coins
						one_check();
						two_check(); 
						five_check();
						ten_check();

						
						// money > 0
						if( balance_b > 0 )
						{			
								if( pump_state == 1)
								{
									// 1 liter per 35
									if( LL_TIM_IsActiveFlag_UPDATE(TIM9) == SET )
									{
											LL_TIM_ClearFlag_UPDATE(TIM9);
											if(water_cnt <= 110)
												water_cnt++;
											if(water_cnt == 111) 
											{
												water_cnt = 1;
												balance_b--;
											}
									}	
								}
								// display total money
								sprintf(disp_str, "%4d", balance_b);
								display(disp_str);
						}				  
					}
					
				// water level less than 50
				if( level < 30 )
				{
						pump_state = 0;				// reset pump to 0
						machine_status = 0;		// deboucing relay, forced to disable PA3
						LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);	
									
						// machine unavailable
						LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);	
						LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
				}	
		
		}
			
		inDex = 0;
	}	
}

/************************************ Display Function ************************************/
void display(char seg[4])
{
	for(i=0, k=1 ; i<4 ; ++i, k*=2)
	{
		GPIOC->ODR &= ~0x000F;
		GPIOB->ODR &= ~0xFC04;
			
		if(seg[i] == '0') GPIOB->ODR |= 0x7C04;
		else if (seg[i] == '1') GPIOB->ODR |= 0x0C00;
		else if (seg[i] == '2') GPIOB->ODR |= 0xB404;
		else if (seg[i] == '3') GPIOB->ODR |= 0x9C04;
		else if (seg[i] == '4') GPIOB->ODR |= 0xCC00;
		else if (seg[i] == '5') GPIOB->ODR |= 0xD804;
		else if (seg[i] == '6') GPIOB->ODR |= 0xFB04;
		else if (seg[i] == '7') GPIOB->ODR |= 0x0C04;
		else if (seg[i] == '8') GPIOB->ODR |= 0xFC04;
		else if (seg[i] == '9') GPIOB->ODR |= 0xDC04;
		else GPIOB->ODR &= ~0xFC04;
		GPIOC->ODR |= k;
		LL_mDelay(1);
	}
}

void SEG_Config(void)
{
	LL_GPIO_InitTypeDef seg_init;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
	seg_init.Mode = LL_GPIO_MODE_OUTPUT;
	seg_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	seg_init.Pull = LL_GPIO_PULL_NO;
	seg_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	seg_init.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_10 |
									LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 |
									LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB, &seg_init);
	
	seg_init.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 |
									LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC, &seg_init);
}

void LED_Config(void)
{
	LL_GPIO_InitTypeDef led_config;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	led_config.Mode = LL_GPIO_MODE_OUTPUT;
	led_config.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	led_config.Pull = LL_GPIO_PULL_NO;
	led_config.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	led_config.Pin = LL_GPIO_PIN_8;		// RED
	LL_GPIO_Init(GPIOA, &led_config);
	
	led_config.Pin = LL_GPIO_PIN_9;		// GREEN
	LL_GPIO_Init(GPIOA, &led_config);
	
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
}


/************************************ Sensor Function ************************************/
void SensorGPIO_Config(void)
{
	LL_GPIO_InitTypeDef timic_init;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	
	timic_init.Pin = LL_GPIO_PIN_1;
	timic_init.Mode = LL_GPIO_MODE_ALTERNATE;
	timic_init.Alternate = LL_GPIO_AF_1;
	timic_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	timic_init.Pull = LL_GPIO_PULL_NO;
	timic_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOA, &timic_init);
	
	timic_init.Mode = LL_GPIO_MODE_OUTPUT;
	timic_init.Pin= LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOA, &timic_init);
}

void SensorTIM_IC_Config(void)
{
	LL_TIM_IC_InitTypeDef timic;
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	
	timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
	timic.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
	timic.ICPolarity = LL_TIM_IC_POLARITY_BOTHEDGE;
	timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
	LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH2, &timic);
	
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	
	LL_TIM_EnableIT_CC2(TIM2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM2);
}

void TIM2_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC2(TIM2) == SET)
	{
		LL_TIM_ClearFlag_CC2(TIM2);
		if(state==0)
		{
			IC1 = LL_TIM_IC_GetCaptureCH2(TIM2);
			state = 1;
		}
		else if(state==1)
		{
			IC2 = LL_TIM_IC_GetCaptureCH2(TIM2);
			if(IC2 > IC1)
				Diff = IC2 - IC1;
			else if(IC2 < IC1)
				Diff = ((LL_TIM_GetAutoReload(TIM2) - IC1) + IC2) +1;
			state = 2;
		}
	}
}


/************************************ Notification Function ************************************/
void TIM_Duration(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_dura;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	//Time-base configure
	timbase_dura.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_dura.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_dura.Autoreload = ARR - 1;
	timbase_dura.Prescaler =  32000 - 1;
	LL_TIM_Init(TIM3, &timbase_dura);
	
	LL_TIM_EnableCounter(TIM3); 
	LL_TIM_ClearFlag_UPDATE(TIM3); //Force clear update flag
}

void Notic_TIM_BASE_Config(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC - 1;
	LL_TIM_Init(TIM4, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM4); 
}


void Notic_TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef speaker_config;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	speaker_config.Mode = LL_GPIO_MODE_ALTERNATE;
	speaker_config.Alternate = LL_GPIO_AF_2;
	speaker_config.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	speaker_config.Pin = LL_GPIO_PIN_6;
	speaker_config.Pull = LL_GPIO_PULL_NO;
	speaker_config.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &speaker_config);
}

void Notic_TIM_OC_Config(uint16_t note)
{
	LL_TIM_OC_InitTypeDef tim_oc_iconf;
	
	Notic_TIM_OC_GPIO_Config();
	Notic_TIM_BASE_Config(note);
	
	tim_oc_iconf.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_iconf.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_iconf.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_iconf.CompareValue = LL_TIM_GetAutoReload(TIM4) / 2;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_iconf);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}

void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
	}
}


/************************************ Coins Classify Function ************************************/
void Coins_Config(void)
{
	LL_GPIO_InitTypeDef gpio_config;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	
	gpio_config.Mode = LL_GPIO_MODE_INPUT;
	gpio_config.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_config.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_config.Pull = LL_GPIO_PULL_NO;
	
	gpio_config.Pin = LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOB, &gpio_config);
	
	gpio_config.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOD, &gpio_config);
	
	gpio_config.Pin = LL_GPIO_PIN_11;
	LL_GPIO_Init(GPIOA, &gpio_config);
	
	gpio_config.Pin = LL_GPIO_PIN_12;
	LL_GPIO_Init(GPIOA, &gpio_config);
}

void one_check(void)
{
	one_b = one_b_state;
	one_b_state = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_2);
	if( one_b == 1 && one_b_state == 0)
	{
		++balance_b;
	}
}

void two_check(void)
{
	two_b = two_b_state;
	two_b_state = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_3);
	if( two_b == 1 && two_b_state == 0)
	{
		balance_b += 2;
	}
}

void five_check(void)
{
	five_b = five_b_state;
	five_b_state = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11);
	if( five_b == 1 && five_b_state == 0)
	{		
		balance_b += 5;
	}
}

void ten_check(void)
{
	ten_b = ten_b_state;
	ten_b_state = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_12);
	if( ten_b == 1 && ten_b_state == 0)
	{
		balance_b += 10;
	}
}

/************************************ Pump Function ************************************/
void water_TIMBase(void)
{
	LL_TIM_InitTypeDef tim_water;
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
	
	tim_water.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	tim_water.CounterMode = LL_TIM_COUNTERMODE_UP;
	tim_water.Autoreload = 4000 - 1;
	tim_water.Prescaler = 3000 - 1;
	LL_TIM_Init(TIM9, &tim_water);
}

void pump_Config(void)
{
    RCC -> AHBENR |= (1<<0);
    GPIOA -> MODER |= (1<<6);
    GPIOA -> MODER &= ~(3<<0);

    RCC->APB2ENR |= (1<<0);
    SYSCFG->EXTICR[0] = 0x0;
    EXTI->IMR |= (1<<0);
    EXTI->FTSR |= (1<<0);

    NVIC_EnableIRQ((IRQn_Type) 6);
    NVIC_SetPriority((IRQn_Type) 6, 0);
}

void EXTI0_IRQHandler(void) 
{
    if((EXTI->PR & (1<<0)) == 1 )
    {
			if( machine_status == 1 )
			{
				pump_state = !pump_state;
				if(pump_state == 1) // pump work
				{
					LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
					LL_TIM_EnableCounter(TIM9);
				}
				else
				{
					LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
					LL_TIM_DisableCounter(TIM9);
				}
			}
			
			if( machine_status == 0 )
			 LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
		}
		EXTI->PR |= (1<<0);

}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}

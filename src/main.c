/*
 * This is freertos release 9.0.0 configured for a stm32f103vct6 board,
 * named Hy-MiniSTM32V. It should run on any stm32f103 with a few changes.
 * 
 * There are two tasks running. One is blinking two LEDs, the other 
 * is sending stuff via USART1.
 * 
 * Author: Nils Stec, stecdose@gmail.com
 * Date 2016-07-01
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "stm32f10x.h"
#include "bool.h"
#include "usart.h"
#include "eeprom.h"

#include "stm32_dsp.h"

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"
#include "freertos/include/queue.h"
#include "freertos/include/timers.h"
#include "freertos/include/semphr.h"

static SemaphoreHandle_t buttonSemaphore;

const portTickType xDelay = 10 / portTICK_RATE_MS;

static uint32_t tim4Tick = 0;

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Tim4_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* NVIC_PriorityGroup */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  
  TIM_DeInit(TIM4);

  TIM_TimeBaseStructure.TIM_Period = 1000;//装载值
  //prescaler is 72, that is 72000000/72/1000 = 1000Hz;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;//分频系数
  //set clock division 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
  //count up
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //clear the TIM4 overflow interrupt flag
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  //TIM4 overflow interrupt enable
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //enable TIM4
  TIM_Cmd(TIM4, DISABLE);
}

void Tim4_Enable(void)
{
  TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
    tim4Tick++;

    if(tim4Tick % 10 == 0) { /* 10ms */
    }

#if (defined USART1_IBUS) && (defined USART1_ENABLE)
    usart1_idle_tick++;
#endif
#if (defined USART2_IBUS) && (defined USART2_ENABLE)
    usart2_idle_tick++;
#endif
#if (defined USART3_IBUS) && (defined USART3_ENABLE)
    usart3_idle_tick++;
#endif
    //
    // 清除 TIM4
    TIM_ClearITPendingBit(TIM4, /*TIM_IT_Update*/ TIM_FLAG_Update);
  }
}

/*
*
*/

/******************************************************************
 2 函數名稱:CreateSinWave()
 3 函數功能:模擬采樣數據，采樣數據中包含3種頻率正弦波(350Hz，8400Hz，18725Hz)
 4 參數說明:
 5 備    注:在buf32數組中，每個數據的高16位存儲采樣數據的實部，
 6           低16位存儲采樣數據的虛部(總是為0)
 7 作　　者:博客園 依舊淡然（http://www.cnblogs.com/menlsh/）
 8 *******************************************************************/

#include <math.h>
#define PI2 6.28318530717959

void CreateSinWave(uint32_t npt, uint32_t sampleRate, uint32_t *buf32, uint16_t *buf16)
{
  float fx;
  size_t i;
  for(i=0;i<npt;i++) {
    float fx = 4000 * sin(PI2 * i * 350.0 / sampleRate) +
    4000 * sin(PI2 * i * 5000.0 / sampleRate) +
    3000 * sin(PI2 * i * 18725.0 / sampleRate);
    if(buf32)
      buf32[i] = ((signed short)fx) << 16;
    if(buf16)
      buf16[i] = (signed short)fx;
    //Usart2_Printf("%d %d\r\n", i, (int16_t)fx);
  }
}

void Main_Task(void *p)
{  
  #define SR 44800
  #define N 256
  #define M 16

  uint16_t sineWave16[N+M-1];

  CreateSinWave(N+M-1, SR, 0, sineWave16);

  uint32_t sineWave32[N];
  uint32_t spectrum[N];

/*
*
*/

//FIR滤波系数由matlab 生成，由于系数要求是unsigned short型的，所以系数都乘以了65536
/*
* https://octave-online.net/
*
* Sample rate 44800, Low pass cut off frequency 8000 Hz
*
* fs=44800
* fc=8000 
* w=2*fc/fs
* b=fir1(15,w)
* b=b*65536
*/  

  int16_t h[M] = {190, 320, -112, -1742, -2441, 2159, 12561, 21832, 21832, 12561, 2159, -2441, -1742, -112, 320, 190};

  COEFS fir_coefs;
  fir_coefs.h = h;
  fir_coefs.nh = M;

  fir_16by16_stm32(sineWave32, sineWave16, &fir_coefs, N); /*performs the FIR filtering*/

/*
* https://blog.csdn.net/iloveyoumj/article/details/53308142
*
* http://newgoodlooking.pixnet.net/blog/post/112149183
*
*/

  size_t c;
  for(c=0;c<N;c++)
    sineWave32[c] &= 0xffff0000; /* 數據的高16位存儲采樣數據的實部,低16位存儲采樣數據的虛部(總是為0) */

  uint32_t P = SR / N; /* 解析頻寬 */

  cr4_fft_256_stm32(spectrum, sineWave32, N);

  int16_t lX, lY;
  float X, Y, Mag;
  size_t i;
  Usart2_Printf("Index\tFreq\tMagnitude\r\n");
  for(i=0;i<N/2;i++) {
    /* 計算各次諧波幅值,先將spectrum分解成實部(X)和虛部(Y)，然后計算幅值(sqrt(X*X+Y*Y) */
    lX  = (spectrum[i] << 16) >> 16;
    lY  = (spectrum[i] >> 16);
    X = N * ((float)lX) / 32768;
    Y = N * ((float)lY) / 32768;
    Mag = sqrtf(X * X + Y * Y) / N;
    if(i == 0)
      Usart2_Printf("%d\t%d\t%d\r\n", i, P*i, (unsigned long)(Mag * 32768));
    else
      Usart2_Printf("%d\t%d\t%d\r\n", i, P*i, (unsigned long)(Mag * 65536));
  }

  static portTickType latestTick;
  while(1) {
    latestTick = xTaskGetTickCount();
    vTaskDelayUntil(&latestTick, 10 / portTICK_RATE_MS); /* 100 Hz */
  }
}

/*
*
*/

static TimerHandle_t hLedTimer;

static void LedTimerCb(TimerHandle_t th)
{
  static uint16_t tick = 0;

  if(tick++ % 2)
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
  else
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

/*
*
*/

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

/*
  #define configMAX_SYSCALL_INTERRUPT_PRIORITY 191 （0xBF也即优先级11），
  故在中断优先级为0～10的中断，均不会被内核延迟，并且可嵌套但不能调用API函数。在11～15之间的中断可以调用以​FromISR结尾的API函数。
*/

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); /* 4 bits preemption. It's requirement of FreeRTOS */
#if 0
  /* Enable Prefetch Buffer */
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  /* Flash 2 wait state */
  FLASH_SetLatency(FLASH_Latency_2);
  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  EE_Init();
#endif
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // PB12 : LED
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, GPIO_Pin_12);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  // PA0 : LED / PA7 : Buzzer
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_0);
  GPIO_ResetBits(GPIOA, GPIO_Pin_7);

  // PA3 / PA4 : LED Matrix
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

  // PB5 / PB6 : LED Matrix
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);

#ifdef USART2_ENABLE
  Usart2_Init(115200);
  Usart2_Puts("\r\nF3F BASE v0.1\r\n");
#endif
  Tim4_Init();
  Tim4_Enable();

  xTaskCreate(Main_Task, (const char*)"Main_Task", 1024, NULL, 4, NULL); /* heighest priority 4 */
  
  hLedTimer = xTimerCreate(
      "ledTimer", /* debug name of task */
      pdMS_TO_TICKS(500), /* period */
      pdTRUE, /* auto-reload */
      NULL, /* no timer ID */
      LedTimerCb
      );

  xTimerStart(hLedTimer, 0);

	// Start RTOS scheduler

	vTaskStartScheduler();

	for(;;) { // Never here ...
	}
		
	return 0;
}


void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
// 	printf("ERROR: vApplicationStackOverflowHook(): Task \"%s\" overflowed its stack\n", pcTaskName);
// 	fflush(stdout);
// 	assert(false);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // for using printf()
#include <string.h> // for using memset(), strlen(), strcmp, strtok()
#include "hdc1080\hdc1080.h"  // for using hdc1080 through I2C
#include "clcd\clcd.h"  // for using clcd library
#include "ringbuffer\rb.h"  // for using ring buffer
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Demo2 protocol command define
#define STX       0x02
#define ETX       0x03

#define VAR         'V'
#define CDS         'P'
#define TC1047     'T'
#define ENCODER     'E'
#define SERVO       'S'
#define HDC1080     'I'
#define DAC         'D'
#define LED         'L'
#define RGBLED      'M'
#define TACTSWITCH  'R'
#define CLCD_L1     '1'
#define CLCD_L2     '2'

#define CMD_READ  0x52
#define CMD_WRITE 0x57

// receive packet wait time
#define TIMEOUT  1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RingFifo_t gUart1Fifo;
volatile int gEnCoderCnt;
volatile uint16_t gADCxConvertedValue[3];
volatile uint8_t gRGBPulse[3];
int gTimerFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Porting to UART2 for serial debugging */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  if (ch == '\n')
    HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xFFFF);
  HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}

/**
 * @brief   DAC7512 Write through SPI
 * @param   data   unsigned short data for send through SPI
 * @retval  0 is success to send the data, 1 is fail
 */
uint8_t
DAC7512_write (unsigned short data)
{
  uint8_t buff[2], ret;

  HAL_GPIO_WritePin (SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  buff[0] = (uint8_t) (data >> 8);
  buff[1] = (uint8_t) data;

  ret = HAL_SPI_Transmit (&hspi2, buff, 2, 1000);

  if (ret)
    return ret;

  HAL_GPIO_WritePin (SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  return 0;
}

/**
 * @brief   Block until Ring buffer received packet or timeout occurs
 * @param   timeout   timeout[ms]
 * @retval  0 is Ring buffer received packet, 1 is timeout occurs
 */
int
WaitPacket (int timeout)
{
  if (!RB_isempty (&gUart1Fifo))
    return 0;
  else
    {
      int cnt = 0;
      while (cnt != timeout)
        {
          if (!RB_isempty (&gUart1Fifo))
            break;
          cnt++;
          HAL_Delay (1);
        }

      if (cnt != timeout)
        return 0;
      else
        return 1;
    }
}

/**
 * @brief   Send AT Command to HM-10
 * @param   command   AT Command.
 * @param   buf       Buffer to store HM-10's reply message.
 * @param   bufsize   Buffer size.
 * @param   timeout   UART Transmit and Receive timeout.
 * @retval  0 is success, else is HAL_Status error in UART.
 */
int
SendATCommand (const char *command, char *buf, int bufsize, int timeout)
{
  int ret;

  memset ((void*) buf, 0, bufsize);

  ret = HAL_UART_Transmit (&huart1, (uint8_t*) command, strlen (command), timeout);
  if (ret != HAL_OK)
    return ret;

  ret = HAL_UART_Receive (&huart1, (uint8_t*) buf, bufsize, timeout);
  if (ret != HAL_OK)
    return ret;

  return 0;
}

/**
 * @brief   Initialize HM-10
 * @param   None
 * @retval  None
 */
void
InitHM10 (void)
{
  char rcvBuf[32] =
    { 0 };
  char *ptr;
  int cnt = 0;

  printf ("Initialize HM-10\n");

  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE); // Disable UART1 Interrupt

  printf ("HM-10 Connecting. .");
  fflush(stdout);
  while (cnt++ < 10)
    {
      // Check whether AT commands can be used. If it can use, receive message "OK"
      SendATCommand ("AT", rcvBuf, sizeof(rcvBuf), 500);

      if (!strcmp (rcvBuf, "OK"))
        {
          printf ("\nHM-10-> %s\n", rcvBuf);
          printf ("HM-10 can use AT Command!\n\n");
          break;
        }
      printf (" .");
      fflush(stdout);

      HAL_Delay (1000);
    }

  if (cnt >= 10)
    {
      printf ("HM-10 connecting fail!\n");
      return;
    }

  SendATCommand ("AT+NAME?", rcvBuf, sizeof(rcvBuf), 100);

  // if Name is not NUCLEOEVB, Restore all setup value to factory setup and rename to NUCLEOEVB
  if (strcmp (rcvBuf, "OK+NAME:NUCLEOEVB"))
    {
      printf ("Restore all setup and rename to \"NUCLEOEVB\".\n");

      /*  default setting
       *  Name: HMSoft
       *  Baud: 9600, N, 8, 1
       *  Pin code: 000000
       *  Peripheral(Slave) Role
       *  Transmit mode(Can configure parameters until module is connected)
       */
      SendATCommand ("AT+RENEW", rcvBuf, sizeof(rcvBuf), 1000);
      printf ("AT Renew: %s\n", rcvBuf);
      HAL_Delay(1000);

      // set Name NUCLEOEVB
      SendATCommand ("AT+NAMENUCLEOEVB", rcvBuf, sizeof(rcvBuf), 100);

      // HM-10's SW reset to apply the changed settings
      SendATCommand ("AT+RESET", rcvBuf, sizeof(rcvBuf), 500);
      printf ("AT Reset: %s\n", rcvBuf);
      HAL_Delay(1000);
    }

  printf ("Show HM-10 settings\n");

  // get addr
  SendATCommand ("AT+ADDR?", rcvBuf, sizeof(rcvBuf), 100);
  ptr = strtok (rcvBuf, ":");
  ptr = strtok (NULL, ":");
  printf (" Addr: %s\n", ptr);

  // get name
  SendATCommand ("AT+NAME?", rcvBuf, sizeof(rcvBuf), 100);
  ptr = strtok (rcvBuf, ":");
  ptr = strtok (NULL, ":");
  printf (" Name: %s\n", ptr);

  // get pass(pin code)
  SendATCommand ("AT+PASS?", rcvBuf, sizeof(rcvBuf), 100);
  ptr = strtok (rcvBuf, ":");
  ptr = strtok (NULL, ":");
  printf (" PW(pin): %s\n", ptr);

  // get type(type0: Not need PIN Code)
  SendATCommand ("AT+TYPE?", rcvBuf, sizeof(rcvBuf), 100);
  ptr = strtok (rcvBuf, ":");
  ptr = strtok (NULL, ":");
  printf (" Type: %s\n", ptr);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  // Enable UART1 Interrupt
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t bat_stat;
  uint16_t I2C_id;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf("Application Demo2\n\n");

  /* Start ADC Calibration */
  if (HAL_ADCEx_Calibration_Start (&hadc1) != HAL_OK)
    {
      Error_Handler ();
    }
  printf ("ADC Calibration Done!\n");

  /* Start ADC in DMA mode,
   *
   * ADC Rank1 - VAR,
   *     Rank2 - CDS,
   *     Rank3 - TC1047
   */
  if (HAL_ADC_Start_DMA (&hadc1, (uint32_t*) &gADCxConvertedValue, 3) != HAL_OK)
    {
      Error_Handler ();
    }
  printf ("Start ADC in DMA Mode\n");

  /* Init HDC1080 */
  // HDC1080 id read to I2C
  HAL_Delay(50);
  hdc1080_read_reg (&hi2c1, HDC1080_ID_DEV, &I2C_id);
  hdc1080_init (&hi2c1, HDC1080_T_RES_14, HDC1080_RH_RES_14, 1, &bat_stat);
  printf ("I2C Initialization Done!");
  printf ("HDC1080's ID = %X\n", I2C_id);

  /* TIM4 PWM start for Servo */
  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_2);

  /* TIM3 PWM start for RGB LED */
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_3);
  printf ("Start PWM for Servo & RGB LED\n");

  /* Init LCD */
  CLCD_Init (16, 2);
  printf ("CLCD Initialization Done!\n");

  /* Init RingBuffer */
  if (RB_init (&gUart1Fifo, 64))
    {
      Error_Handler ();
    }

  /* Init Bluetooth(HM-10) */
  InitHM10 ();

  /* Start TIM2 Interrupt (50msec) */
  HAL_TIM_Base_Start_IT (&htim2);

  printf ("\nMain program start!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int send_idx = 0, read_idx = 0;
  int i, rb_flag = 0;
  uint8_t ch;
  uint8_t hdc_humi;
  int led_value, switch_value;
  float hdc_temp;
  char rcvbuf[21] = { 0 };
  char clcd[17] = { 0 };

  while (1)
  {
      /***********************************************************************************************/
      /************************************* Data Acquisition ****************************************/
      /***********************************************************************************************/
      hdc1080_measure (&hi2c1, &hdc_temp, &hdc_humi);
      led_value = (HAL_GPIO_ReadPin (LED0_GPIO_Port, LED0_Pin) << 24) | (HAL_GPIO_ReadPin (LED1_GPIO_Port, LED1_Pin) << 16)
          | (HAL_GPIO_ReadPin (LED2_GPIO_Port, LED2_Pin) << 8) | HAL_GPIO_ReadPin (LED3_GPIO_Port, LED3_Pin);

      switch_value = ((!HAL_GPIO_ReadPin (SW0_GPIO_Port, SW0_Pin)) << 24) | ((!HAL_GPIO_ReadPin (SW1_GPIO_Port, SW1_Pin)) << 16)
          | ((!HAL_GPIO_ReadPin (SW2_GPIO_Port, SW2_Pin)) << 8) | (!HAL_GPIO_ReadPin (SW3_GPIO_Port, SW3_Pin));

      /***********************************************************************************************/
      /*********************************** Data Send Routine *****************************************/
      /***********************************************************************************************/
#if 1
      // Check timer flag
      if (gTimerFlag == 1)
        {
          gTimerFlag = 0;

          uint8_t sendBuf[10] = { 0 };
          int index = 0;

          // send packet
          switch (send_idx)
            {
            case 0: // VAR data packet
              send_idx++;
              uint16_t var_val = gADCxConvertedValue[0];

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = VAR;
              sendBuf[index++] = 2;
              sendBuf[index++] = var_val;
              sendBuf[index++] = var_val >> 8;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - VAR: %d\n", send_idx, var_val);
              break;

            case 1: // CDS data packet
              send_idx++;
              uint16_t cds_val = gADCxConvertedValue[1];

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = CDS;
              sendBuf[index++] = 2;
              sendBuf[index++] = cds_val;
              sendBuf[index++] = cds_val >> 8;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - CDS: %d\n", send_idx, cds_val);
              break;

            case 2: // TC1047 data packet
              send_idx++;
              float tc_temp = (float) (((gADCxConvertedValue[2] * 3.3 / 4096 * 1000) - 500) / 10);
              uint32_t temp = *(uint32_t*) &tc_temp;

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = TC1047;
              sendBuf[index++] = 4;
              sendBuf[index++] = temp;
              sendBuf[index++] = temp >> 8;
              sendBuf[index++] = temp >> 16;
              sendBuf[index++] = temp >> 24;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - TC1047: %.2f\n", send_idx, *(float *)&temp);
              break;

            case 3: // Encoder data packet
              send_idx++;
              int encoder_temp = gEnCoderCnt;

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = ENCODER;
              sendBuf[index++] = 4;
              sendBuf[index++] = encoder_temp;
              sendBuf[index++] = encoder_temp >> 8;
              sendBuf[index++] = encoder_temp >> 16;
              sendBuf[index++] = encoder_temp >> 24;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - Encoder: %4d\n", send_idx, encoder_temp);
              break;

            case 4: // HDC1080 data packet
              send_idx++;
              uint32_t _hdc_temp = *(uint32_t*) &hdc_temp;
              uint8_t _hdc_humi = hdc_humi;

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = HDC1080;
              sendBuf[index++] = 5;
              sendBuf[index++] = _hdc_temp;
              sendBuf[index++] = _hdc_temp >> 8;
              sendBuf[index++] = _hdc_temp >> 16;
              sendBuf[index++] = _hdc_temp >> 24;
              sendBuf[index++] = _hdc_humi;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - HDC1080: %.2f, %d\n", send_idx, *(float *)&hdc_temp, hdc_humi);
              break;

            case 5: // LED data packet
              send_idx++;
              int led_temp = led_value;

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = LED;
              sendBuf[index++] = 4;
              sendBuf[index++] = led_temp;
              sendBuf[index++] = led_temp >> 8;
              sendBuf[index++] = led_temp >> 16;
              sendBuf[index++] = led_temp >> 24;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - LED: %08x\n", send_idx, led_temp);
              break;

            case 6: // RGB LED data packet
              send_idx++;

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = RGBLED;
              sendBuf[index++] = 4;
              sendBuf[index++] = gRGBPulse[0];
              sendBuf[index++] = gRGBPulse[1];
              sendBuf[index++] = gRGBPulse[2];
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - RGB: %3d, %3d, %3d\n", send_idx, gRGBPulse[0], gRGBPulse[1], gRGBPulse[2]);
              break;

            case 7: // Tact Switch data packet
              send_idx++;
              int sw_temp = switch_value;

              sendBuf[index++] = STX;
              sendBuf[index++] = CMD_READ;
              sendBuf[index++] = TACTSWITCH;
              sendBuf[index++] = 4;
              sendBuf[index++] = sw_temp;
              sendBuf[index++] = sw_temp >> 8;
              sendBuf[index++] = sw_temp >> 16;
              sendBuf[index++] = sw_temp >> 24;
              sendBuf[index++] = ETX;

              HAL_UART_Transmit (&huart1, sendBuf, index, 50);
              //printf ("Sequence%d - Switch: %08x\n\n", send_idx, sw_temp);
              send_idx = 0;
              break;

            default:
              Error_Handler ();
              break;
            }
        }
#endif

      /***********************************************************************************************/
      /*********************************** Data Receive Routine **************************************/
      /***********************************************************************************************/
      // Check ring buffer
      if (!RB_isempty (&gUart1Fifo))
        {
          ch = RB_read (&gUart1Fifo);
#if 1
          // Check STX
          if (ch == STX)
            {
              memset ((void*) rcvbuf, 0, sizeof(rcvbuf));
              read_idx = 0;
              HAL_Delay (30);

              rcvbuf[read_idx++] = ch;

              // Read CMD, DEVICE, DATASIZE
              for (i = 0; i < 3; i++)
                {
                  if (!WaitPacket (TIMEOUT))
                    rcvbuf[read_idx++] = RB_read (&gUart1Fifo);
                  else
                    {
                      rb_flag = 1;
                      break;
                    }
                }

              if (rb_flag)
                {
                  printf ("Packet receive fail!\n");
                  for (i = 0; i < read_idx; i++)
                    printf ("%02x ", rcvbuf[i]);
                  printf ("\n\n");
                  read_idx = 0;
                  rb_flag = 0;
                  continue;
                }

              // Read DATA, ETX
              for (i = 0; i < rcvbuf[3] + 1; i++)
                {
                  if (!WaitPacket (TIMEOUT))
                    rcvbuf[read_idx++] = RB_read (&gUart1Fifo);
                  else
                    {
                      rb_flag = 1;
                      break;
                    }
                }

              if (rb_flag)
                {
                  printf ("Packet receive fail: ");
                  for (i = 0; i < read_idx; i++)
                    printf ("%02x ", rcvbuf[i]);
                  printf ("\n\n");
                  read_idx = 0;
                  rb_flag = 0;
                  continue;
                }

              printf ("Packet receive complete: ");
              for (i = 0; i < read_idx; i++)
                printf ("%02x ", rcvbuf[i]);
              printf ("\n\n");

              // DATA processing
              switch (rcvbuf[2])
                {
                case ENCODER:
                  {
                    int encoder_tmp = 0;

                    encoder_tmp |= rcvbuf[4];
                    encoder_tmp |= rcvbuf[5] << 8;
                    encoder_tmp |= rcvbuf[6] << 16;
                    encoder_tmp |= rcvbuf[7] << 24;

                    gEnCoderCnt = encoder_tmp;

                    break;
                  }

                case SERVO:
                  {
                    short servo_tmp = 0;
                    int servo_pulse;

                    servo_tmp |= rcvbuf[4];
                    servo_tmp |= rcvbuf[5] << 8;

                    if (servo_tmp > 90)
                      servo_tmp = 90;
                    else if (servo_tmp < -90)
                      servo_tmp = -90;

                    servo_pulse = ((int) ((servo_tmp + 90) * 1000 / 180)) + 1000;
                    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, servo_pulse);

                    break;
                  }

                case DAC:
                  {
                    uint16_t dac_tmp = 0;

                    dac_tmp |= rcvbuf[4];
                    dac_tmp |= rcvbuf[5] << 8;

                    DAC7512_write (dac_tmp);

                    break;
                  }

                case LED:
                  {
                    HAL_GPIO_WritePin (LED3_GPIO_Port, LED3_Pin, rcvbuf[4]); // LED3
                    HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, rcvbuf[5]); // LED2
                    HAL_GPIO_WritePin (LED1_GPIO_Port, LED1_Pin, rcvbuf[6]); // LED1
                    HAL_GPIO_WritePin (LED0_GPIO_Port, LED0_Pin, rcvbuf[7]); // LED0

                    break;
                  }

                case RGBLED:
                  {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, rcvbuf[4]); // Blue
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, rcvbuf[5]); // Green
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rcvbuf[6]); // Red

                    break;
                  }

                case CLCD_L1:
                  {
                    int j = 0;
                    for (int i = 19; i >= 4; i--)
                      clcd[j++] = (rcvbuf[i] == 0) ? 0x20 : rcvbuf[i];

                    CLCD_Puts (0, 0, clcd);

                    break;
                  }

                case CLCD_L2:
                  {
                    int j = 0;
                    for (int i = 19; i >= 4; i--)
                      clcd[j++] = (rcvbuf[i] == 0) ? 0x20 : rcvbuf[i];

                    CLCD_Puts (0, 1, clcd);

                    break;
                  }

                default:
                  printf ("Device Type is wrong!\n");
                  break;
                } // DATA processing
            } // Check STX
#endif
        } // Check ring buffer

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 249;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CLCD_D0_Pin|CLCD_D1_Pin|CLCD_D2_Pin|CLCD_D3_Pin
                          |CLCD_EN_Pin|CLCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED0_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLCD_D0_Pin CLCD_D1_Pin CLCD_D2_Pin CLCD_D3_Pin
                           CLCD_EN_Pin CLCD_RS_Pin */
  GPIO_InitStruct.Pin = CLCD_D0_Pin|CLCD_D1_Pin|CLCD_D2_Pin|CLCD_D3_Pin
                          |CLCD_EN_Pin|CLCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED0_Pin LED1_Pin LED2_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED0_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW0_Pin SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_SW_Pin */
  GPIO_InitStruct.Pin = EN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_B_SIG_Pin */
  GPIO_InitStruct.Pin = EN_B_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_B_SIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_A_SIG_Pin */
  GPIO_InitStruct.Pin = EN_A_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_A_SIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void
HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
#if 1
  if (htim->Instance == TIM2)  // 20Hz(50msec) Timer
    {
      gTimerFlag = 1;
    }
#endif
}

void
HAL_UART_RxCpltCallback (UART_HandleTypeDef *UartHandle)
{
  uint8_t rx;

  if (UartHandle->Instance == USART1)
    {
      rx = (uint8_t) (UartHandle->Instance->DR & (uint8_t) 0x00FF);
      RB_write (&gUart1Fifo, rx);
    }

}

void
HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
    {
    case GPIO_PIN_1:  // SW0 for LED0, if pushed LED ON
      if (HAL_GPIO_ReadPin (SW0_GPIO_Port, SW0_Pin))
        HAL_GPIO_WritePin (LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
      else
        HAL_GPIO_WritePin (LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      break;

    case GPIO_PIN_2:  // SW1 for LED1
      if (HAL_GPIO_ReadPin (SW1_GPIO_Port, SW1_Pin))
        HAL_GPIO_WritePin (LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      else
        HAL_GPIO_WritePin (LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      break;

    case GPIO_PIN_4:  // SW2 for LED2
      if (HAL_GPIO_ReadPin (SW2_GPIO_Port, SW2_Pin))
        HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      else
        HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      break;

    case GPIO_PIN_5:  // SW3 for LED3
      if (HAL_GPIO_ReadPin (SW3_GPIO_Port, SW3_Pin))
        HAL_GPIO_WritePin (LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
      else
        HAL_GPIO_WritePin (LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      break;

    case GPIO_PIN_8: // Encoder SW
      gEnCoderCnt = 0;
      break;

    case GPIO_PIN_12: // Encoder A_SIG
      if (!HAL_GPIO_ReadPin (EN_B_SIG_GPIO_Port, EN_B_SIG_Pin))
        gEnCoderCnt++;
      break;

    case GPIO_PIN_15: // Encoder B_SIG
      if (!HAL_GPIO_ReadPin (EN_A_SIG_GPIO_Port, EN_A_SIG_Pin))
        gEnCoderCnt--;
      break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

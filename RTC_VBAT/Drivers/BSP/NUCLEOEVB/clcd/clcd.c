/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "clcd.h"

/* Private HD44780 structure */
typedef struct
{
  uint8_t DisplayControl;
  uint8_t DisplayFunction;
  uint8_t DisplayMode;
  uint8_t Rows;
  uint8_t Cols;
  uint8_t currentX;
  uint8_t currentY;
} HD44780_Options_t;

__STATIC_INLINE uint32_t
DWT_DELAY_Init (void)
{
#if !defined(STM32F0xx)
  uint32_t c;

  /* Enable TRC */
  CoreDebug->DEMCR &= ~0x01000000;
  CoreDebug->DEMCR |= 0x01000000;

  /* Enable counter */
  DWT->CTRL &= ~0x00000001;
  DWT->CTRL |= 0x00000001;

  /* Reset counter */
  DWT->CYCCNT = 0;

  /* Check if DWT has started */
  c = DWT->CYCCNT;

  /* 2 dummys */
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Return difference, if result is zero, DWT has not started */
  return (DWT->CYCCNT - c);
#else
  /* Return OK */
  return 1;
#endif
}

/**
 * @brief  Delays for amount of micro seconds
 * @param  micros: Number of microseconds for delay
 * @retval None
 */
__STATIC_INLINE void
Delay (__IO uint32_t micros)
{
#if !defined(STM32F0xx)
  uint32_t start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  micros *= (HAL_RCC_GetHCLKFreq () / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - start) < micros)
    ;
#else
  /* Go to clock cycles */
  micros *= (SystemCoreClock / 1000000) / 5;

  /* Wait till done */
  while (micros--)
    ;
#endif
}

/* Private functions */
static void CLCD_Cmd (uint8_t cmd);
static void CLCD_Cmd4bit (uint8_t cmd);
static void CLCD_Data (uint8_t data);
static void CLCD_CursorSet (uint8_t col, uint8_t row);

/* Private variable */
static HD44780_Options_t HD44780_Opts;

/* Pin definitions */
#define HD44780_RS_LOW              HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_RESET)
#define HD44780_RS_HIGH             HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_SET)
#define HD44780_E_LOW               HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_RESET)
#define HD44780_E_HIGH              HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_SET)

#define HD44780_E_BLINK             HD44780_E_HIGH; HD44780_Delay(20); HD44780_E_LOW; HD44780_Delay(20)
#define HD44780_Delay(x)            Delay(x)

/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

void
CLCD_Init (uint8_t cols, uint8_t rows)
{

  DWT_DELAY_Init ();

  /* At least 40ms */
  HD44780_Delay (45000);

  /* Set LCD width and height */
  HD44780_Opts.Rows = rows;
  HD44780_Opts.Cols = cols;

  /* Set cursor pointer to beginning for LCD */
  HD44780_Opts.currentX = 0;
  HD44780_Opts.currentY = 0;

  HD44780_Opts.DisplayFunction =
    HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_1LINE;
  if (rows > 1)
    {
      HD44780_Opts.DisplayFunction |= HD44780_2LINE;
    }

  /* Try to set 4bit mode */
  CLCD_Cmd4bit (0x03);
  HD44780_Delay (4500);

  /* Second try */
  CLCD_Cmd4bit (0x03);
  HD44780_Delay (4500);

  /* Third goo! */
  CLCD_Cmd4bit (0x03);
  HD44780_Delay (4500);

  /* Set 4-bit interface */
  CLCD_Cmd4bit (0x02);
  HD44780_Delay (100);

  /* Set # lines, font size, etc. */
  CLCD_Cmd (HD44780_FUNCTIONSET | HD44780_Opts.DisplayFunction);

  /* Turn the display on with no cursor or blinking default */
  HD44780_Opts.DisplayControl = HD44780_DISPLAYON;
  CLCD_DisplayOn ();

  /* Clear lcd */
  CLCD_Clear ();

  /* Default font directions */
  HD44780_Opts.DisplayMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
  CLCD_Cmd (HD44780_ENTRYMODESET | HD44780_Opts.DisplayMode);

  /* Delay */
  HD44780_Delay (4500);
}

void
CLCD_Clear (void)
{
  CLCD_Cmd (HD44780_CLEARDISPLAY);
  HD44780_Delay (3000);
}

void
CLCD_Puts (uint8_t x, uint8_t y, char *str)
{
  CLCD_CursorSet (x, y);
  while (*str)
    {
      if (HD44780_Opts.currentX >= HD44780_Opts.Cols)
        {
          HD44780_Opts.currentX = 0;
          HD44780_Opts.currentY++;
          CLCD_CursorSet (HD44780_Opts.currentX, HD44780_Opts.currentY);
        }
      if (*str == '\n')
        {
          HD44780_Opts.currentY++;
          CLCD_CursorSet (HD44780_Opts.currentX, HD44780_Opts.currentY);
        }
      else if (*str == '\r')
        {
          CLCD_CursorSet (0, HD44780_Opts.currentY);
        }
      else
        {
          CLCD_Data (*str);
          HD44780_Opts.currentX++;
        }
      str++;
    }
}

void
CLCD_DisplayOn (void)
{
  HD44780_Opts.DisplayControl |= HD44780_DISPLAYON;
  CLCD_Cmd (HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void
CLCD_DisplayOff (void)
{
  HD44780_Opts.DisplayControl &= ~HD44780_DISPLAYON;
  CLCD_Cmd (HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void
CLCD_BlinkOn (void)
{
  HD44780_Opts.DisplayControl |= HD44780_BLINKON;
  CLCD_Cmd (HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void
CLCD_BlinkOff (void)
{
  HD44780_Opts.DisplayControl &= ~HD44780_BLINKON;
  CLCD_Cmd (HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void
CLCD_CursorOn (void)
{
  HD44780_Opts.DisplayControl |= HD44780_CURSORON;
  CLCD_Cmd (HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void
CLCD_CursorOff (void)
{
  HD44780_Opts.DisplayControl &= ~HD44780_CURSORON;
  CLCD_Cmd (HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void
CLCD_ScrollLeft (void)
{
  CLCD_Cmd (HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void
CLCD_ScrollRight (void)
{
  CLCD_Cmd (HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void
CLCD_CreateChar (uint8_t location, uint8_t * data)
{
  uint8_t i;
  /* We have 8 locations available for custom characters */
  location &= 0x07;
  CLCD_Cmd (HD44780_SETCGRAMADDR | (location << 3));

  for (i = 0; i < 8; i++)
    {
      CLCD_Data (data[i]);
    }
}

void
CLCD_PutCustom (uint8_t x, uint8_t y, uint8_t location)
{
  CLCD_CursorSet (x, y);
  CLCD_Data (location);
}

/* Private functions */
static void
CLCD_Cmd (uint8_t cmd)
{
  /* Command mode */
  HD44780_RS_LOW;

  /* High nibble */
  CLCD_Cmd4bit (cmd >> 4);
  /* Low nibble */
  CLCD_Cmd4bit (cmd & 0x0F);
}

static void
CLCD_Data (uint8_t data)
{
  /* Data mode */
  HD44780_RS_HIGH;

  /* High nibble */
  CLCD_Cmd4bit (data >> 4);
  /* Low nibble */
  CLCD_Cmd4bit (data & 0x0F);
}

static void
CLCD_Cmd4bit (uint8_t cmd)
{
  /* Set output port */
  HAL_GPIO_WritePin (HD44780_D7_PORT, HD44780_D7_PIN,
                     (GPIO_PinState) (cmd & 0x08));
  HAL_GPIO_WritePin (HD44780_D6_PORT, HD44780_D6_PIN,
                     (GPIO_PinState) (cmd & 0x04));
  HAL_GPIO_WritePin (HD44780_D5_PORT, HD44780_D5_PIN,
                     (GPIO_PinState) (cmd & 0x02));
  HAL_GPIO_WritePin (HD44780_D4_PORT, HD44780_D4_PIN,
                     (GPIO_PinState) (cmd & 0x01));
  HD44780_E_BLINK;
}

static void
CLCD_CursorSet (uint8_t col, uint8_t row)
{
  uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

  /* Go to beginning */
  if (row >= HD44780_Opts.Rows)
    {
      row = 0;
    }

  /* Set current column and row */
  HD44780_Opts.currentX = col;
  HD44780_Opts.currentY = row;

  /* Set location address */
  CLCD_Cmd (HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}

#include "oled.h"

#include "oledfont.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define OLED_I2C_ADDR      (0x3CU << 1)
#define OLED_PAGE_COUNT    8U
#define OLED_DATA_CHUNK    16U
#define OLED_COLUMN_OFFSET 2U  // SH1106 typically uses offset 2

static I2C_HandleTypeDef *s_oled_i2c = &hi2c1;
static uint8_t s_oled_gram[OLED_PAGE_COUNT][X_WIDTH];
static uint8_t s_dirty_valid;
static uint8_t s_dirty_x_min;
static uint8_t s_dirty_x_max;
static uint8_t s_dirty_page_min;
static uint8_t s_dirty_page_max;

static void OLED_DirtyReset(void)
{
  s_dirty_valid = 0U;
  s_dirty_x_min = 0U;
  s_dirty_x_max = 0U;
  s_dirty_page_min = 0U;
  s_dirty_page_max = 0U;
}

static void OLED_MarkDirty(uint8_t x, uint8_t page)
{
  if (s_dirty_valid == 0U)
  {
    s_dirty_valid = 1U;
    s_dirty_x_min = x;
    s_dirty_x_max = x;
    s_dirty_page_min = page;
    s_dirty_page_max = page;
    return;
  }

  if (x < s_dirty_x_min)
  {
    s_dirty_x_min = x;
  }
  else if (x > s_dirty_x_max)
  {
    s_dirty_x_max = x;
  }

  if (page < s_dirty_page_min)
  {
    s_dirty_page_min = page;
  }
  else if (page > s_dirty_page_max)
  {
    s_dirty_page_max = page;
  }
}

static void OLED_WriteCommand(uint8_t cmd)
{
  uint8_t packet[2] = {0x00U, cmd};
  (void)HAL_I2C_Master_Transmit(s_oled_i2c, OLED_I2C_ADDR, packet, sizeof(packet), HAL_MAX_DELAY);
}

static void OLED_WriteDataBlock(const uint8_t *data, uint16_t size)
{
  uint8_t packet[OLED_DATA_CHUNK + 1U];
  uint16_t offset = 0U;

  while (offset < size)
  {
    uint16_t chunk = (uint16_t)(size - offset);
    if (chunk > OLED_DATA_CHUNK)
    {
      chunk = OLED_DATA_CHUNK;
    }

    packet[0] = 0x40U;
    memcpy(&packet[1], &data[offset], chunk);
    (void)HAL_I2C_Master_Transmit(s_oled_i2c, OLED_I2C_ADDR, packet, (uint16_t)(chunk + 1U), HAL_MAX_DELAY);
    offset = (uint16_t)(offset + chunk);
  }
}

static void OLED_UpdateArea(uint8_t x, uint8_t page, const uint8_t *buf, uint8_t len)
{
  if (page >= OLED_PAGE_COUNT)
  {
    return;
  }

  OLED_Set_Pos(x, page);
  OLED_WriteDataBlock(buf, len);
}

void IIC_Start(void)
{
}

void IIC_Stop(void)
{
}

void Write_IIC_Byte(unsigned char IIC_Byte)
{
  (void)IIC_Byte;
}

void Write_IIC_Command(unsigned char IIC_Command)
{
  OLED_WriteCommand(IIC_Command);
}

void Write_IIC_Data(unsigned char IIC_Data)
{
  OLED_WriteDataBlock(&IIC_Data, 1U);
}

void OLED_WR_Byte(unsigned dat, unsigned cmd)
{
  uint8_t value = (uint8_t)dat;
  if (cmd == OLED_DATA)
  {
    OLED_WriteDataBlock(&value, 1U);
  }
  else
  {
    OLED_WriteCommand(value);
  }
}

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
  uint8_t col = (uint8_t)(x + OLED_COLUMN_OFFSET);

  OLED_WriteCommand((uint8_t)(0xB0U + y));
  OLED_WriteCommand((uint8_t)(0x00U + (col & 0x0FU)));
  OLED_WriteCommand((uint8_t)(0x10U + ((col >> 4) & 0x0FU)));
}

void OLED_Display_On(void)
{
  OLED_WriteCommand(0x8DU);
  OLED_WriteCommand(0x14U);
  OLED_WriteCommand(0xAFU);
}

void OLED_Display_Off(void)
{
  OLED_WriteCommand(0x8DU);
  OLED_WriteCommand(0x10U);
  OLED_WriteCommand(0xAEU);
}

void fill_picture(unsigned char fill_Data)
{
  for (uint8_t page = 0U; page < OLED_PAGE_COUNT; ++page)
  {
    memset(s_oled_gram[page], fill_Data, X_WIDTH);
    OLED_UpdateArea(0U, page, s_oled_gram[page], X_WIDTH);
  }

  OLED_DirtyReset();
}

void OLED_Clear(void)
{
  fill_picture(0x00U);
}

void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
  uint8_t page;
  uint8_t bit_mask;

  if ((x >= X_WIDTH) || (y >= Y_WIDTH))
  {
    return;
  }

  page = (uint8_t)(y / 8U);
  bit_mask = (uint8_t)(1U << (y % 8U));

  if (t != 0U)
  {
    s_oled_gram[page][x] |= bit_mask;
  }
  else
  {
    s_oled_gram[page][x] &= (uint8_t)(~bit_mask);
  }

  OLED_MarkDirty(x, page);
}

void OLED_RefreshDirty(void)
{
  uint8_t page;
  uint8_t len;

  if (s_dirty_valid == 0U)
  {
    return;
  }

  len = (uint8_t)(s_dirty_x_max - s_dirty_x_min + 1U);
  for (page = s_dirty_page_min; page <= s_dirty_page_max; ++page)
  {
    OLED_UpdateArea(s_dirty_x_min, page, &s_oled_gram[page][s_dirty_x_min], len);
  }

  OLED_DirtyReset();
}

void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot)
{
  uint8_t x;
  uint8_t y;

  if ((x1 >= X_WIDTH) || (x2 >= X_WIDTH) || (y1 >= Y_WIDTH) || (y2 >= Y_WIDTH) || (x1 > x2) || (y1 > y2))
  {
    return;
  }

  for (y = y1; y <= y2; ++y)
  {
    for (x = x1; x <= x2; ++x)
    {
      OLED_DrawPoint(x, y, dot);
    }
  }

  OLED_RefreshDirty();
}

void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 Char_Size)
{
  uint8_t c;
  uint8_t i;

  if (x > (Max_Column - 1U))
  {
    return;
  }

  if (chr < ' ')
  {
    chr = ' ';
  }
  c = (uint8_t)(chr - ' ');

  if (Char_Size == 16U)
  {
    if (y >= (OLED_PAGE_COUNT - 1U))
    {
      return;
    }

    for (i = 0U; i < 8U; ++i)
    {
      if ((x + i) < X_WIDTH)
      {
        s_oled_gram[y][x + i] = F8X16[(c * 16U) + i];
        s_oled_gram[y + 1U][x + i] = F8X16[(c * 16U) + i + 8U];
      }
    }

    OLED_UpdateArea(x, y, &s_oled_gram[y][x], 8U);
    OLED_UpdateArea(x, (uint8_t)(y + 1U), &s_oled_gram[y + 1U][x], 8U);
  }
  else
  {
    if (y >= OLED_PAGE_COUNT)
    {
      return;
    }

    for (i = 0U; i < 6U; ++i)
    {
      if ((x + i) < X_WIDTH)
      {
        s_oled_gram[y][x + i] = F6x8[c][i];
      }
    }

    OLED_UpdateArea(x, y, &s_oled_gram[y][x], 6U);
  }
}

static u32 oled_pow(u8 m, u8 n)
{
  u32 result = 1U;
  while (n--)
  {
    result *= m;
  }
  return result;
}

void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2)
{
  u8 t;
  u8 temp;
  u8 enshow = 0U;

  for (t = 0U; t < len; t++)
  {
    temp = (u8)((num / oled_pow(10U, (uint8_t)(len - t - 1U))) % 10U);
    if ((enshow == 0U) && (t < (len - 1U)))
    {
      if (temp == 0U)
      {
        OLED_ShowChar((u8)(x + (size2 / 2U) * t), y, ' ', size2);
        continue;
      }
      enshow = 1U;
    }

    OLED_ShowChar((u8)(x + (size2 / 2U) * t), y, (u8)(temp + '0'), size2);
  }
}

void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 Char_Size)
{
  uint8_t j = 0U;

  if (chr == NULL)
  {
    return;
  }

  while (chr[j] != '\0')
  {
    OLED_ShowChar(x, y, chr[j], Char_Size);
    x = (uint8_t)(x + 8U);
    if (x > 120U)
    {
      x = 0U;
      y = (uint8_t)(y + 2U);
    }
    ++j;
  }
}

void OLED_ShowCHinese(u8 x, u8 y, u8 no)
{
  uint8_t t;
  uint8_t index = (uint8_t)(2U * no);

  if (y >= (OLED_PAGE_COUNT - 1U))
  {
    return;
  }

  for (t = 0U; t < 16U; ++t)
  {
    if ((x + t) < X_WIDTH)
    {
      s_oled_gram[y][x + t] = (uint8_t)Hzk[index][t];
      s_oled_gram[y + 1U][x + t] = (uint8_t)Hzk[index + 1U][t];
    }
  }

  OLED_UpdateArea(x, y, &s_oled_gram[y][x], 16U);
  OLED_UpdateArea(x, (uint8_t)(y + 1U), &s_oled_gram[y + 1U][x], 16U);
}

void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
  unsigned int j = 0U;
  unsigned char x;
  unsigned char y;

  if ((BMP == NULL) || (x1 > X_WIDTH) || (y1 > OLED_PAGE_COUNT) || (x0 >= x1) || (y0 >= y1))
  {
    return;
  }

  for (y = y0; y < y1; y++)
  {
    for (x = x0; x < x1; x++)
    {
      s_oled_gram[y][x] = BMP[j++];
    }
    OLED_UpdateArea(x0, y, &s_oled_gram[y][x0], (uint8_t)(x1 - x0));
  }
}

void Delay_50ms(unsigned int Del_50ms)
{
  while (Del_50ms--)
  {
    HAL_Delay(50U);
  }
}

void Delay_1ms(unsigned int Del_1ms)
{
  HAL_Delay(Del_1ms);
}

void Picture(void)
{
}

void OLED_Init(void)
{
  static const uint8_t init_cmds[] = {
    0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
    0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
    0x40, 0x8D, 0x14, 0xAF
  };
  uint8_t i;

  memset(s_oled_gram, 0, sizeof(s_oled_gram));
  HAL_Delay(50U);

  for (i = 0U; i < (uint8_t)sizeof(init_cmds); ++i)
  {
    OLED_WriteCommand(init_cmds[i]);
  }

  OLED_DirtyReset();
  OLED_Clear();
}

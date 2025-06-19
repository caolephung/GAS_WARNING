#include "liquidcrystal_i2c.h"

static uint8_t dpFunction;
static uint8_t dpControl;
static uint8_t dpMode;
static uint8_t dpRows;
static uint8_t dpBacklight = LCD_BACKLIGHT;

static void DelayUS(uint32_t us);
static void Write4Bits(uint8_t value);
static void ExpanderWrite(uint8_t data);
static void PulseEnable(uint8_t data);
static void Send(uint8_t value, uint8_t mode);
static void SendCommand(uint8_t cmd);
static void SendChar(uint8_t ch);

void HD44780_Init(uint8_t rows) {
    dpRows = rows;
    dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    if (dpRows > 1) dpFunction |= LCD_2LINE;

    for (int i = 0; i < 3; i++) { // kiem tra lenh 8 bit
        Write4Bits(0x03 << 4);
        DelayUS(4500);
    }
    Write4Bits(0x02 << 4); // chuyen sang lenh 4 bit
    DelayUS(100);

    SendCommand(LCD_FUNCTIONSET | dpFunction);
    dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    HD44780_Display();
    HD44780_Clear();

    dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    SendCommand(LCD_ENTRYMODESET | dpMode);
    HD44780_Home();
}

void HD44780_Clear() {	// xoa noi dung tren lcd
    SendCommand(LCD_CLEARDISPLAY);
    DelayUS(2000);
}

void HD44780_Home() {	//	dua con tro ve hang 0, cot 0
    SendCommand(LCD_RETURNHOME);
    DelayUS(2000);
}

void HD44780_Display() {	// bat man hinh
    dpControl |= LCD_DISPLAYON;
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoDisplay() {	// tat man hinh
    dpControl &= ~LCD_DISPLAYON;
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_SetCursor(uint8_t col, uint8_t row) {	//set dong de dat con tro
    const uint8_t row_offsets[] = {0x00, 0x40}; // dia chi dau dong
    if (row >= dpRows) row = dpRows - 1;
    SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void HD44780_PrintStr(const char *str) { // gui chuoi str
    while (*str) SendChar(*str++);
}

void HD44780_ScrollDisplayLeft() {
    SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | 0x00);
}

static void SendCommand(uint8_t cmd) { // gui lenh
    Send(cmd, 0);
}

static void SendChar(uint8_t ch) {  // gui ki tu
    Send(ch, RS);
}

static void Send(uint8_t value, uint8_t mode) { // gui 1 byte(8bit) den lcd thong qua pcf8547
    Write4Bits((value & 0xF0) | mode);
    Write4Bits(((value << 4) & 0xF0) | mode);
}

static void Write4Bits(uint8_t value) { // gui 4 bit du lieu den lcd(1/2 lan)
    ExpanderWrite(value);
    PulseEnable(value);
}

static void ExpanderWrite(uint8_t data) { // gui i2c den module mo rong
    data |= dpBacklight;	//bat den nen

    I2C1->CR1 |= I2C_CR1_START; //bat dau i2c
    while (!(I2C1->SR1 & I2C_SR1_SB));	// doi den khi start thanh cong

    I2C1->DR = DEVICE_ADDR;	// dia chi cua slave
    while (!(I2C1->SR1 & I2C_SR1_ADDR));	// doi ack tra loi
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    I2C1->CR1 |= I2C_CR1_STOP;	// ket thuc giao tiep
}

static void PulseEnable(uint8_t data) { //tao xug enable bat dau truyen vao lcd
    ExpanderWrite(data | ENABLE); // bat enable cho lcd de nhan du lieu
    DelayUS(1);
    ExpanderWrite(data & ~ENABLE); // tat enable, chot du lieu
    DelayUS(50);
}

static void DelayUS(uint32_t us) {  // delay us
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

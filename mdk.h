// Copyright (c) 2022 Cesanta
// All rights reserved

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BIT(x) ((uint32_t) 1U << (x))
#define REG(x) ((volatile uint32_t *) (x))




#define ESP32_SENS_SAR_DAC_CTRL1  0x3ff48898
#define ESP32_SENS_SAR_DAC_CTRL2  0x3ff4889C
#define ESP32_RTCIO_PAD_DAC1 0x3ff48484
#define ESP32_RTCIO_PAD_DAC2 0x3ff48488
#define ESP32_RTCIO_ADC_PAD 0x3ff48480
#define ESP32_SENS_SAR_READ_CTRL2 0x3ff48890
#define ESP32_SENS_SAR_MEAS_START2 0x3ff48894

#define ESP32_SENS_SAR_READ_CTRL1 0x3ff48800
#define ESP32_SENS_SAR_MEAS_START1 0x3ff48854

#define DR_REG_SENS_BASE                        0x3ff48800
#define SENS_SAR_READ_CTRL_REG          (DR_REG_SENS_BASE + 0x0000)
#define SENS_SAR_READ_STATUS1_REG          (DR_REG_SENS_BASE + 0x0004)
#define SENS_SAR_MEAS_WAIT1_REG          (DR_REG_SENS_BASE + 0x0008)
#define SENS_SAR_MEAS_WAIT2_REG          (DR_REG_SENS_BASE + 0x000c)
#define SENS_SAR_MEAS_CTRL_REG          (DR_REG_SENS_BASE + 0x0010)
#define SENS_SAR_READ_STATUS2_REG          (DR_REG_SENS_BASE + 0x0014)
#define SENS_SAR_START_FORCE_REG          (DR_REG_SENS_BASE + 0x002c)
#define SENS_SAR_ATTEN1_REG          (DR_REG_SENS_BASE + 0x0034)
#define SENS_SAR_SLAVE_ADDR1_REG  (DR_REG_SENS_BASE + 0x003c)
#define SENS_SAR_ATTEN2_REG          (DR_REG_SENS_BASE + 0x0038)
#define SENS_SAR_SLAVE_ADDR2_REG  (DR_REG_SENS_BASE + 0x0040)
#define SENS_SAR_MEAS_START1_REG          (DR_REG_SENS_BASE + 0x0054)
#define SENS_SAR_TOUCH_CTRL1_REG          (DR_REG_SENS_BASE + 0x0058)
#define SENS_SAR_TOUCH_CTRL2_REG          (DR_REG_SENS_BASE + 0x0084)
#define SENS_SAR_TOUCH_ENABLE_REG          (DR_REG_SENS_BASE + 0x008c)
#define SENS_SAR_READ_CTRL2_REG          (DR_REG_SENS_BASE + 0x0090)
#define SENS_SAR_MEAS_START2_REG          (DR_REG_SENS_BASE + 0x0094)
#define SENS_SAR_MEAS_CTRL2_REG          (DR_REG_SENS_BASE + 0x0a0)

#define DR_REG_SYSCON_BASE 0x3ff66000 //0x60002600
#define APB_CTRL_SYSCLK_CONF_REG (DR_REG_SYSCON_BASE + 0x0)
#define APB_SARADC_CTRL_REG (DR_REG_SYSCON_BASE + 0x10)
#define APB_SARADC_CTRL2_REG (DR_REG_SYSCON_BASE + 0x14)
#define APB_SARADC_FSM_REG (DR_REG_SYSCON_BASE + 0x18)
#define APB_SARADC_SAR1_PATT_TAB1_REG (DR_REG_SYSCON_BASE + 0x1C)
#define APB_SARADC_SAR2_PATT_TAB1_REG (DR_REG_SYSCON_BASE + 0x2C)

#define RNG_REG 0x3FF75144

#define I2S_FIFO_WR_REG 0x3FF4F000
#define I2S_FIFO_RD_REG 0x3FF4F004 
#define I2S_CONF_REG 0x3FF4F008 
#define I2S_INT_RAW_REG 0x3FF4F00C
#define I2S_INT_ENA_REG 0x3FF4F014
#define I2S_CONF1_REG 0x3FF4F0A0
#define I2S_PD_CONF_REG 0x3FF4F0A4
#define I2S_CONF2_REG 0x3FF4F0A8
#define I2S_CLKM_CONF_REG 0x3FF4F0AC
#define I2S_SAMPLE_RATE_CONF_REG 0x3FF4F0B0
#define I2S_PDM_CONF_REG 0x3FF4F0B4
#define I2S_STATE_REG 0x3FF4F0BC
#define I2S_TIMING_REG 0x3FF4F01C 
#define I2S_FIFO_CONF_REG 0x3FF4F020 
#define I2S_CONF_SINGLE_DATA_REG 0x3FF4F028
#define I2S_CONF_CHAN_REG 0x3FF4F02c
#define I2S_LC_HUNG_CONF_REG 0x3FF4F074
#define I2S_LC_CONF_REG 0x3FF4F060
#define I2S_RXEOF_NUM_REG 0x3FF4F024
#define I2S_IN_LINK_REG 0x3FF4F034
#define I2S_INLINK_DSCR_REG 0x3FF4F048
//CALL REGS AS REGS AND NOT IF NOT

#define DPORT_PERIP_CLK_EN_REG 0x3FF000C0
#define DPORT_PERIP_RST_EN_REG 0x3FF000C4
#define DPORT_WIFI_CLK_EN_REG 0x3FF000CC
#define DPORT_PRO_GPIO_INTERRUPT_MAP_REG 0x3FF0015C
#define GPIO_PIN_REG 0x3FF44088
#define IO_MUX_GPIO2_REG 0x3FF49040
#define IO_MUX_GPIO12ISH_REG 0x3FF49030
#define IO_MUX_GPIO16_REG 0x3FF4904C
#define IO_MUX_GPIO34_REG 0x3FF49014
#define IO_MUX_GPIO36_REG 0x3FF49004

#define GPIO_STATUS_REG 0x3FF44044
#define GPIO_STATUS_W1TC_REG 0x3FF4404C
extern  void ets_isr_unmask(uint32_t mask);
extern  void xtos_set_interrupt_handler(int irq_number, void(*function)(void));






#define ESP32_DPORT 0x3ff00000
#define ESP32_AES 0x3ff01000
#define ESP32_RSA 0x3ff02000
#define ESP32_SHA 0x3ff03000
#define ESP32_FLASH_MMU_TABLE_PRO 0x3ff10000
#define ESP32_FLASH_MMU_TABLE_APP 0x3ff12000
#define ESP32_DPORT_END 0x3ff13FFC
#define ESP32_UART0 0x3ff40000
#define ESP32_SPI1 0x3ff42000
#define ESP32_SPI0 0x3ff43000
#define ESP32_GPIO 0x3ff44000
#define ESP32_GPIO_SD 0x3ff44f00
#define ESP32_FE2 0x3ff45000
#define ESP32_FE 0x3ff46000
#define ESP32_FRC_TIMER 0x3ff47000
#define ESP32_RTCCNTL 0x3ff48000
#define ESP32_RTCIO 0x3ff48400
#define ESP32_SENS 0x3ff48800
#define ESP32_RTC_I2C 0x3ff48C00
#define ESP32_IO_MUX 0x3ff49000
#define ESP32_HINF 0x3ff4B000
#define ESP32_UHCI1 0x3ff4C000
#define ESP32_I2S 0x3ff4F000
#define ESP32_UART1 0x3ff50000
#define ESP32_BT 0x3ff51000
#define ESP32_I2C_EXT 0x3ff53000
#define ESP32_UHCI0 0x3ff54000
#define ESP32_SLCHOST 0x3ff55000
#define ESP32_RMT 0x3ff56000
#define ESP32_PCNT 0x3ff57000
#define ESP32_SLC 0x3ff58000
#define ESP32_LEDC 0x3ff59000
#define ESP32_EFUSE 0x3ff5A000
#define ESP32_SPI_ENCRYPT 0x3ff5B000
#define ESP32_NRX 0x3ff5CC00
#define ESP32_BB 0x3ff5D000
#define ESP32_PWM 0x3ff5E000
#define ESP32_TIMERGROUP0 0x3ff5F000
#define ESP32_TIMERGROUP1 0x3ff60000
#define ESP32_RTCMEM0 0x3ff61000
#define ESP32_RTCMEM1 0x3ff62000
#define ESP32_RTCMEM2 0x3ff63000
#define ESP32_SPI2 0x3ff64000
#define ESP32_SPI3 0x3ff65000
#define ESP32_SYSCON 0x3ff66000
#define ESP32_APB_CTRL 0x3ff66000  // Old name for SYSCON, to be removed
#define ESP32_I2C1_EXT 0x3ff67000
#define ESP32_SDMMC 0x3ff68000
#define ESP32_EMAC 0x3ff69000
#define ESP32_CAN 0x3ff6B000
#define ESP32_PWM1 0x3ff6C000
#define ESP32_I2S1 0x3ff6D000
#define ESP32_UART2 0x3ff6E000
#define ESP32_PWM2 0x3ff6F000
#define ESP32_PWM3 0x3ff70000
#define PERIPHS_SPI_ENCRYPTADDR ESP32_SPI_ENCRYPT





// Perform `count` "NOP" operations
static inline void spin(volatile unsigned long count) {
  while (count--) asm volatile("nop");
}

static inline uint64_t systick(void) {
  REG(ESP32_TIMERGROUP0)[3] = 1;
  spin(1);
  return ((uint64_t) REG(ESP32_TIMERGROUP0)[2] << 32) |
         REG(ESP32_TIMERGROUP0)[1];
}

static inline uint64_t uptime_us(void) {
  return systick() >> 5;
}

static inline void delay_us(unsigned long us) {
  uint64_t until = uptime_us() + us;    // Calculate timeout timestamp
  while (uptime_us() < until) spin(1);  // Wait until until
}

static inline void delay_ms(unsigned long ms) {
  delay_us(ms * 1000);
}

static inline void wdt_feed(void) {
  REG(ESP32_RTCCNTL)[40] |= BIT(31);
}

static inline void wdt_disable(void) {
  REG(ESP32_RTCCNTL)[41] = 0x50d83aa1;  // Disable write protection
  wdt_feed();
  REG(ESP32_RTCCNTL)[35] = 0;      // Disable RTC WDT
  REG(ESP32_TIMERGROUP0)[18] = 0;  // Disable task WDT
  REG(ESP32_TIMERGROUP1)[18] = 0;  // Disable task WDT
}


//RTC_CNTL_CLK_CONF_REG 70

static inline void cpu_freq_240(void) {
  // TRM 3.2.3. We must set SEL_0 = 1, SEL_1 = 2
  // *SEL_0: The vaule of register RTC_CNTL_SOC_CLK_SEL
  // *SEL_1: The vaule of register CPU_CPUPERIOD_SEL
  REG(ESP32_RTCCNTL)[28] |= 1UL << 27;  // Register 31.24  SEL0 -> 1
//DPORT_CPU_PER_CONF_REG 3c
  REG(ESP32_DPORT)[15] |= 2UL << 0;     // Register 5.9    SEL1 -> 2
  REG(ESP32_UART0)[5] = 0x4001e0;       // UART_CLKDIV_REG
}

static inline void soc_init(void) {
  wdt_disable();
  REG(ESP32_TIMERGROUP0)[0] |= BIT(31);  // Enable TIMG0
  cpu_freq_240();
}

// API GPIO
#define GPIO_FUNC_OUT_SEL_CFG_REG REG(0X3ff44530)  // Pins 0-39
#define GPIO_FUNC_IN_SEL_CFG_REG REG(0X3ff44130)   // Pins 0-39
#define GPIO_OUT_REG REG(0X3ff44004)               // Pins 0-31
#define GPIO_IN_REG REG(0x3FF4403C)                // Pins 0-31
#define GPIO_ENABLE_REG REG(0X3ff44020)            // Pins 0-31
#define GPIO_OUT1_REG REG(0X3ff44010)              // Pins 32-39
#define GPIO_IN1_REG REG(0X3ff44040)               // Pins 32-39
#define GPIO_ENABLE1_REG REG(0X3ff4402c)           // Pins 32-39

static inline void gpio_output_enable(int pin, bool enable) {
  volatile uint32_t *r = GPIO_ENABLE_REG;
  if (pin > 31) pin -= 31, r = GPIO_ENABLE1_REG;
  r[0] &= ~BIT(pin);
  r[0] |= (enable ? 1U : 0U) << pin;
}

static inline void gpio_output(int pin) {
  GPIO_FUNC_OUT_SEL_CFG_REG[pin] = 256;  // Simple output, TRM 4.3.3
  gpio_output_enable(pin, 1);
}

static inline void gpio_write(int pin, bool value) {
  volatile uint32_t *r = GPIO_OUT_REG;
  if (pin > 31) pin -= 31, r = GPIO_OUT1_REG;
  r[0] &= ~BIT(pin);                 // Clear first
  r[0] |= (value ? 1U : 0U) << pin;  // Then set
}

static inline void gpio_toggle(int pin) {
  volatile uint32_t *r = GPIO_OUT_REG;
  if (pin > 31) pin -= 31, r = GPIO_OUT1_REG;
  r[0] ^= BIT(pin);
}

static inline void gpio_input(int pin) {
  // Index lookup table for IO_MUX_GPIOx_REG, TRM 4.12
  unsigned char map[40] = {17, 34, 16, 33, 18, 27, 24, 25, 26, 21,  // 0-9
                           22, 23, 13, 14, 12, 15, 19, 20, 28, 29,  // 10-19
                           30, 31, 32, 35, 36, 9,  10, 11, 0,  0,   // 20-29
                           0,  0,  7,  8,  5,  6,  1,  2,  3,  4};  // 30-39
  volatile uint32_t *mux = REG(0X3ff49000);
  if (pin < 0 || pin > (int) sizeof(map) || map[pin] == 0) return;
  gpio_output_enable(pin, 0);  // Disable output
  mux[map[pin]] |= BIT(9);     // Enable input
}

static inline bool gpio_read(int pin) {
  volatile uint32_t *r = GPIO_IN_REG;
  if (pin > 31) pin -= 31, r = GPIO_IN1_REG;
  return r[0] & BIT(pin) ? 1 : 0;
}



#include <mdk.h>

//static int led_pin = LED1;  // To override: make EXTRA_CFLAGS=-DLED1=5
//static int led_state = 0;

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
#define SENS_SAR_ATTEN2_REG          (DR_REG_SENS_BASE + 0x0038)
#define SENS_SAR_MEAS_START1_REG          (DR_REG_SENS_BASE + 0x0054)
#define SENS_SAR_TOUCH_CTRL1_REG          (DR_REG_SENS_BASE + 0x0058)
#define SENS_SAR_TOUCH_CTRL2_REG          (DR_REG_SENS_BASE + 0x0084)
#define SENS_SAR_TOUCH_ENABLE_REG          (DR_REG_SENS_BASE + 0x008c)
#define SENS_SAR_READ_CTRL2_REG          (DR_REG_SENS_BASE + 0x0090)
#define SENS_SAR_MEAS_START2_REG          (DR_REG_SENS_BASE + 0x0094)
#define SENS_SAR_MEAS_CTRL2_REG          (DR_REG_SENS_BASE + 0x0a0)

#define DPORT_PRO_GPIO_INTERRUPT_MAP_REG 0x3FF0015C
#define GPIO_PIN_REG 0x3FF44088
#define IO_MUX_GPIO16_REG 0x3FF4904C
#define GPIO_STATUS_REG 0x3FF44044
#define GPIO_STATUS_W1TC_REG 0x3FF4404C
extern  void ets_isr_unmask(uint32_t mask);
extern  void xtos_set_interrupt_handler(int irq_number, void(*function)(void));
#define delaysiz (1<<18)
static uint8_t delaybuff[delaysiz];
static int delayptr;

void gpioHandler() {
  uint32_t r = REG(GPIO_STATUS_REG)[0];

  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;
  volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START1);
  
  delaybuff[delayptr]=(uint8_t)(rr[0]>>4);
  delayptr++;
  if (delayptr>=delaysiz) delayptr=0;
  if (r & BIT(17)){printf("holyinterruptus17 %08x\n",(int)r);}
//printf("holyinterruptus 17\n");
  if (r & BIT(16)){printf("holyinterruptus16 %08x\n",(int)r);}
// printf("holyinterruptus 16\n");
  //printf("holyinterruptus %08x\n",(int)r[0]);
  //REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;
}


int main(void) {

 REG(GPIO_ENABLE_REG)[0]=0;
  ets_isr_unmask(1u << 0);
  REG(DPORT_PRO_GPIO_INTERRUPT_MAP_REG)[0]=0;
  REG(GPIO_PIN_REG)[16]=BIT(15)|BIT(8)|BIT(8); //7risingedge 8falling) 15prointerrupt 13appinterrup
  REG(GPIO_PIN_REG)[17]=BIT(15)|BIT(8)|BIT(8); //7risingedge 15prointerrupt 13appinterrup
  REG(IO_MUX_GPIO16_REG)[0]=BIT(9)|BIT(8); //input enable16
  REG(IO_MUX_GPIO16_REG)[1]=BIT(9)|BIT(8); //input enable17
  xtos_set_interrupt_handler(0,gpioHandler);
//  gpio_output(led_pin);



  REG(ESP32_SENS_SAR_DAC_CTRL1)[0] = 0x0; 
  REG(ESP32_SENS_SAR_DAC_CTRL2)[0] = 0x0; 
//gpio_output_enable(4,true);

  //REG(SENS_SAR_ATTEN1_REG)[0] = 0x0;
  //REG(SENS_SAR_ATTEN2_REG)[0] = 0x0;4 IO_MUX and GPIO Matrix (GPIO, IO_MUX)

  REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(28); //inverse data is normal
  REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(29); //inv

  REG(SENS_SAR_MEAS_START1_REG)[0] |= BIT(31); //pad force
  REG(SENS_SAR_MEAS_START1_REG)[0] |= BIT(18); //start force
  REG(SENS_SAR_MEAS_START2_REG)[0] |= BIT(31);
  REG(SENS_SAR_MEAS_START2_REG)[0] |= BIT(18);

  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)3<<18); //clear force xpd 
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= (2<<16);//powerdiwb forx xpdamp0
//wait2 0x00220001

  //REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFFF<<0); //clear fsm
  //REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFF<<4); //clear fsm truly
//0707 0000


 // REG(SENS_SAR_MEAS_CTRL_REG)[0] = 0x073F0380;
  //NNsar2xpdwait Nsarrstbfsm Nxpdsarfsm 
  //Nampshortgnd Nampshortref Namprstfb Nxpdsarampfsm
//try different fsm
//https://github.com/krzychb/esp32-lna/blob/master/components/lna/lna.c




  REG(SENS_SAR_MEAS_WAIT1_REG)[0] = 0x00010001;
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)0xFFFF);
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= 0x1;

  REG(SENS_SAR_TOUCH_ENABLE_REG)[0] = 0; //touch pads off

  REG(ESP32_RTCIO_ADC_PAD)[0] = BIT(28)|BIT(22)|BIT(21)|BIT(18)|
      BIT(29)|BIT(27)|BIT(26)|BIT(23);
//28 is adc2 mux rtc, 22 fun sel 21, 18 ie
//for sar2 that is
 // REG(ESP32_RTCIO_ADC_PAD)[0] = BIT(28)|BIT(18)|
  //    BIT(29)|BIT(27)|BIT(26)|BIT(23);

  int i = 0;

  for (;;) {
    i++;

    volatile uint32_t *r = REG(ESP32_SENS_SAR_MEAS_START1);
    volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);

    printf("holyadc%08x %08x %08x\n",(int)r[0],(int)rr[0],delayptr);
    REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
    REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
//17 start, 18 start force, 
//31 force pad bitmap, 19 first bit enable
//gpio_toggle(0);

    REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
//10 is xpd force, 17 is mux sel, 18 is xpd, 
    REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
    //printf("LshshshED: %d\n", led_state);  // Print current state to console
    //gpio_write(led_pin, led_state);  // Blink an LED
    //led_state = !led_state;          // Toggle state
    delay_ms(1);                   // Delay a bit
  }

  return 0;
}

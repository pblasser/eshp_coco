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

#define DR_REG_SYSCON_BASE 0x3ff66000 //0x60002600
#define APB_CTRL_SYSCLK_CONF_REG (DR_REG_SYSCON_BASE + 0x0)
#define APB_SARADC_CTRL_REG (DR_REG_SYSCON_BASE + 0x10)
#define APB_SARADC_CTRL2_REG (DR_REG_SYSCON_BASE + 0x14)
#define APB_SARADC_FSM_REG (DR_REG_SYSCON_BASE + 0x18)
#define APB_SARADC_SAR1_PATT_TAB1_REG (DR_REG_SYSCON_BASE + 0x1C)
#define APB_SARADC_SAR2_PATT_TAB1_REG (DR_REG_SYSCON_BASE + 0x2C)



#define I2S_FIFO_WR_REG 0x3FF4F000
#define I2S_FIFO_RD_REG 0x3FF4F004 
#define I2S_CONF_REG 0x3FF4F008 
#define I2S_INT_RAW_REG 0x3FF4F00C
#define I2S_INT_ENA_REG 0x3FF4F014
#define I2S_CONF1_REG 0x3FF4F0A0
#define I2S_PD_CONF_REG 0x3FF4F0A4
#define I2S_CONF2_REG 0x3FF4F0A8
#define I2S_CLKM_CONF_REG 0x3FF4F0AC
#define I2S_STATE_REG 0x3FF4F0BC
#define I2S_TIMING_REG 0x3FF4F01C 
#define I2S_FIFO_CONF_REG 0x3FF4F020 
#define I2S_CONF_SINGLE_DATA_REG 0x3FF4F028
#define I2S_LC_HUNG_CONF_REG 0x3FF4F074


#define DPORT_PERIP_CLK_EN_REG 0x3FF000C0
#define DPORT_WIFI_CLK_EN_REG 0x3FF000CC
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
  int i = 0;
#define dgc 0

void initDIG() {
  printf("dportwifi%08x\n",(int)REG(DPORT_WIFI_CLK_EN_REG)[0]);
  REG(DPORT_WIFI_CLK_EN_REG)[0]=0;
printf("dportwifi%08x\n",(int)REG(DPORT_WIFI_CLK_EN_REG)[0]);

  printf("dportperip%08x\n",(int)REG(DPORT_PERIP_CLK_EN_REG)[0]);
  REG(DPORT_PERIP_CLK_EN_REG)[0]|=BIT(4);
printf("dportperip%08x\n",(int)REG(DPORT_PERIP_CLK_EN_REG)[0]);


  REG(I2S_CONF_REG)[0]=0x3F; //txrxreset fifor startrxtx
  REG(I2S_CONF_REG)[0]&= ~((uint32_t)0x3F);//reset rx, fifo
  REG(I2S_CONF_REG)[0]=0x30;

  REG(I2S_CONF2_REG)[0]=BIT(5);//LCD mode extadx6
  printf("i2sconf2%08x\n",(int)REG(I2S_CONF2_REG)[0]);
  printf("i2pdconf%08x\n",(int)REG(I2S_PD_CONF_REG)[0]);


  printf("sarmeasurewait2%08x\n",(int)REG(SENS_SAR_MEAS_WAIT2_REG)[0]);
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= BIT(19)|BIT(18); //force xpd 
  printf("sarmeasurewait2%08x\n",(int)REG(SENS_SAR_MEAS_WAIT2_REG)[0]);
  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= (2<<16);//powerdiwb forx xpdamp0




  REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(31)|BIT(19+6); //pin 34
  REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(31)|BIT(19); //pin g4
  REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(27)|BIT(28); //dig force inv
  printf("sarreadctrl%08x\n",(int)REG(SENS_SAR_READ_CTRL_REG)[0]);
  REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(28)|BIT(29);//|130; //dig force inv
  printf("sarreadctrl2%08x\n",(int)REG(SENS_SAR_READ_CTRL2_REG)[0]);
  printf("sarforce%08x\n",(int)REG(SENS_SAR_START_FORCE_REG)[0]);
  REG(SENS_SAR_START_FORCE_REG)[0] |= BIT(10);
  printf("sarforce%08x\n",(int)REG(SENS_SAR_START_FORCE_REG)[0]);
 
  printf("apbctrl%08x\n",(int)REG(APB_CTRL_SYSCLK_CONF_REG)[0]);
  REG(APB_CTRL_SYSCLK_CONF_REG)[0] |= BIT(11);
  printf("apbctrl%08x\n",(int)REG(APB_CTRL_SYSCLK_CONF_REG)[0]);

  printf("sarfsm%08x\n",(int)REG(APB_SARADC_FSM_REG)[0]);
  REG(APB_SARADC_FSM_REG)[0]=0x0216FF08;
  //samplecycle2 startwait16 stanbyFF rstb8 
  printf("sarfsm%08x\n",(int)REG(APB_SARADC_FSM_REG)[0]);


  REG(APB_SARADC_CTRL_REG)[0]=BIT(26)|BIT(25)|BIT(23)|BIT(24)|BIT(9)|BIT(6)|BIT(3)|BIT(2)|BIT(1)|BIT(0); 
  printf("saradc%08x\n",(int)REG(APB_SARADC_CTRL_REG)[0]);
  //REG(APB_SARADC_CTRL2_REG)[0]|=BIT(10)|BIT(9);
  printf("saradc2%08x\n",(int)REG(APB_SARADC_CTRL2_REG)[0]);


#define RTC_CNTL_ANA_CONF_REG 0x3FF48030 
//  printf("rtcana%08x\n",(int)REG(RTC_CNTL_ANA_CONF_REG)[0]);
//  REG(RTC_CNTL_ANA_CONF_REG)[0]|=BIT(24);
//REG(RTC_CNTL_ANA_CONF_REG)[0]&= ~BIT(23);
//printf("rtcana%08x\n",(int)REG(RTC_CNTL_ANA_CONF_REG)[0]);





  printf("i2sclkmconf%08x\n",(int)REG(I2S_CLKM_CONF_REG)[0]);
  //REG(I2S_CLKM_CONF_REG)[0]|=BIT(21)|128;
  printf("i2sclkmconf%08x\n",(int)REG(I2S_CLKM_CONF_REG)[0]);

  printf("i2sfifoconf%08x\n",(int)REG(I2S_FIFO_CONF_REG)[0]);
  REG(I2S_FIFO_CONF_REG)[0]=BIT(5)|BIT(11)|BIT(19)|BIT(20);
  //rxdata5 txdata11 forcemodtxrx1920
  printf("i2sfifoconf%08x\n",(int)REG(I2S_FIFO_CONF_REG)[0]);

  //REG(I2S_CONF_REG)[0]=BIT(5)
//REG(I2S_CONF_REG)[0]=BIT(5)|BIT(1);
REG(I2S_CONF_REG)[0]=0x30;
  printf("i2sconf%08x\n",(int)REG(I2S_CONF_REG)[0]);


  REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(27)|BIT(28)|130; //dig force inv
  printf("sarreadctrl%08x\n",(int)REG(SENS_SAR_READ_CTRL_REG)[0]);
  REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(28)|BIT(29)|130; //dig force inv
  //REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(28); //dig force inv
  //REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(29); //dig force inv
  printf("sarreadctrl2%08x\n",(int)REG(SENS_SAR_READ_CTRL2_REG)[0]);
  REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(31)|BIT(19+6); //pin 34
  REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(31)|BIT(19); //pin g4



  REG(APB_SARADC_CTRL_REG)[0]=BIT(26)|BIT(25)|BIT(23)|BIT(24)|BIT(9)|BIT(6)|BIT(3)|BIT(2); 
  printf("saradc%08x\n",(int)REG(APB_SARADC_CTRL_REG)[0]);
  REG(APB_SARADC_CTRL2_REG)[0]=BIT(10)|BIT(9);
  printf("saradc2%08x\n",(int)REG(APB_SARADC_CTRL2_REG)[0]);

  //26datatoi2s 25sarsel 9clkdiv4 6clkgated 3double 2sar2mux
  REG(APB_SARADC_SAR1_PATT_TAB1_REG)[0] = 0x6F6F6F6F; 
printf("pat%08x\n",(int)REG(APB_SARADC_SAR1_PATT_TAB1_REG)[0]);
  REG(APB_SARADC_SAR2_PATT_TAB1_REG)[0] = 0x0F0F0F0F;

  //REG(SENS_SAR_ATTEN1_REG)[0] = 0x0;
  //REG(SENS_SAR_ATTEN2_REG)[0] = 0x0;
  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)3<<18); //clear force xpd 
  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= (2<<16);//powerdiwb forx xpdamp0
  //REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFFF<<0); //clear fsm
  //REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFF<<4); //clear fsm truly
  //REG(SENS_SAR_MEAS_CTRL_REG)[0] = 0x073F0380;
  //NNsar2xpdwait Nsarrstbfsm Nxpdsarfsm 
  //Nampshortgnd Nampshortref Namprstfb Nxpdsarampfsm
  //REG(SENS_SAR_MEAS_WAIT1_REG)[0] = 0x00010001;
  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)0xFFFF);
  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= 0x1;
  REG(SENS_SAR_TOUCH_ENABLE_REG)[0] = 0; //touch pads off
  //REG(ESP32_RTCIO_ADC_PAD)[0] = BIT(22)|BIT(21)|BIT(18)|BIT(27)|BIT(26)|BIT(23);// |BIT(28)|BIT(29);
//28 is adc2 mux rtc, 22 fun sel 21, 18 ie
//for sar2 that is
  REG(I2S_CONF_REG)[0]=BIT(5)|BIT(1);
  REG(I2S_CONF_REG)[0]&= ~(BIT(5)|BIT(1));//reset rx, fifo
}


void digHandler() {
  uint32_t r = REG(GPIO_STATUS_REG)[0];
  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;
  r = BIT(16);
  volatile uint32_t *rrr = REG(I2S_FIFO_RD_REG);
  volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);
  delaybuff[delayptr]=(uint8_t)(rr[0]>>4);
  delayptr++;
  if (delayptr>=delaysiz) delayptr=0;
  if (r & BIT(17)){printf("holyinterruptus17 %08x\n",(int)r);}
//printf("holyinterruptus 17\n");
  if (r & BIT(16)){
  //printf("holyinterruptus16 %08x\n",(int)r);
   printf("fifo%08x sarmeas%08x i2sc2%08x i2sc%08x saradcctrl%08x raw%08x\n",(int)rrr[0],(int)rr[0],(int)REG(I2S_CONF2_REG)[0],(int)REG(I2S_CONF_REG)[0],(int)REG(APB_SARADC_CTRL_REG)[0],(int)REG(I2S_INT_RAW_REG)[0]);
   i++;
    REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
    REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
  }
   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
    REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
//REG(I2S_CONF2_REG)[0]=BIT(5); //lcd5 extadc6
    REG(I2S_LC_HUNG_CONF_REG)[0]=0x10;
  REG(APB_SARADC_CTRL_REG)[0]|=BIT(1)|BIT(0)|BIT(23)|BIT(24);//2324clearpointer
  REG(APB_SARADC_CTRL_REG)[0]&= ~(BIT(1)|BIT(0));
  REG(APB_SARADC_CTRL_REG)[0]|=BIT(1)|BIT(0)|BIT(2);

    
  REG(I2S_CONF_REG)[0]=BIT(5);//1 is rx reset, 5 is rxstart
    
    
}






void rtcHandler() {
  uint32_t r = REG(GPIO_STATUS_REG)[0];
  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF; 
  r=BIT(16);
  volatile uint32_t *rrr = REG(ESP32_SENS_SAR_MEAS_START1);
  volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);
  delaybuff[delayptr]=(uint8_t)(rr[0]>>4);
  delayptr++;
  if (delayptr>=delaysiz) delayptr=0;
  if (r & BIT(17)){printf("holyinterruptus17 %08x\n",(int)r);}
//printf("holyinterruptus 17\n");
  if (r & BIT(16)){
  //printf("holyinterruptus16 %08x\n",(int)r);
      printf("holyadc%08x %08x %08x\n",(int)rrr[0],(int)rr[0],delayptr);
    i++;
    REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
    REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
  }
  
   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
    REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
   
}


void initRTC() {
  REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(28); //inverse data is normal
  REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(29); //inv
  REG(SENS_SAR_MEAS_START1_REG)[0] |= BIT(31); //pad force
  REG(SENS_SAR_MEAS_START1_REG)[0] |= BIT(18); //start force
  REG(SENS_SAR_MEAS_START2_REG)[0] |= BIT(31);
  REG(SENS_SAR_MEAS_START2_REG)[0] |= BIT(18);









  //REG(SENS_SAR_ATTEN1_REG)[0] = 0x0;
  //REG(SENS_SAR_ATTEN2_REG)[0] = 0x0;4 IO_MUX and GPIO Matrix (GPIO, IO_MUX)
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

}




int main(void) {

  

  REG(ESP32_SENS_SAR_DAC_CTRL1)[0] = 0x0; 
  REG(ESP32_SENS_SAR_DAC_CTRL2)[0] = 0x0; 
 #if (dgc)
  initDIG();
  xtos_set_interrupt_handler(0,digHandler);
 #else
  initRTC();
  xtos_set_interrupt_handler(0,rtcHandler);
 #endif
 REG(GPIO_ENABLE_REG)[0]=0;
  ets_isr_unmask(1u << 0);
  REG(DPORT_PRO_GPIO_INTERRUPT_MAP_REG)[0]=0;
  REG(GPIO_PIN_REG)[16]=BIT(15)|BIT(8)|BIT(8); //7risingedge 8falling) 15prointerrupt 13appinterrup
  REG(GPIO_PIN_REG)[17]=BIT(15)|BIT(8)|BIT(8); //7risingedge 15prointerrupt 13appinterrup
  REG(IO_MUX_GPIO16_REG)[0]=BIT(9)|BIT(8); //input enable16
  REG(IO_MUX_GPIO16_REG)[1]=BIT(9)|BIT(8); //input enable17
  for (;;) {
    digHandler();
    delay_us(10);                   // Delay a bit
  }

  return 0;
} 

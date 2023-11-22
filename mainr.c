#include <mdk.h>


#define delaysiz (1<<18)
static uint8_t delaybuff[delaysiz];
static int delayptr;

void rtcHandler() {
  uint32_t r = REG(GPIO_STATUS_REG)[0];
  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF; 
  //r=BIT(2);

  if (r & BIT(2)){  
    volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);
    uint32_t rdrr = REG(ESP32_SENS_SAR_MEAS_START1)[0];
    uint32_t rdr = rr[0];
    if(rdr&BIT(16)) {               
     REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
     REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
     delayptr++;
     if (delayptr>=delaysiz) delayptr=0;
     GPIO_OUT_REG[0]=(uint32_t)(delayptr<<12);
     delaybuff[delayptr]=(uint8_t)(rr[0]>>4);
    }  
    if((delayptr%400)==0) printf("h%08x %08x %08x\n",(int)rdr,(int)rdrr,delayptr);
                         
    REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((delayptr&0xFF)<<19);
    REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((delayptr&0xFF)<<19);
  }
  
   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);

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
   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
    REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
}

int main(void) {
  REG(ESP32_SENS_SAR_DAC_CTRL1)[0] = 0x0; 
  REG(ESP32_SENS_SAR_DAC_CTRL2)[0] = 0x0; 
  initRTC();

  //function 2 on the 12 block
  REG(IO_MUX_GPIO12ISH_REG)[0]=BIT(13);
  REG(IO_MUX_GPIO12ISH_REG)[1]=BIT(13);
  REG(IO_MUX_GPIO12ISH_REG)[2]=BIT(13);
  REG(IO_MUX_GPIO12ISH_REG)[3]=BIT(13);
  //straight out
  GPIO_FUNC_OUT_SEL_CFG_REG[12]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[13]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[14]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[15]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[16]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[17]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[18]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[19]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[21]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[22]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[23]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[27]=256;
  REG(GPIO_ENABLE_REG)[0]=0|BIT(12)|BIT(13)
  |BIT(14)|BIT(15)|BIT(16)|BIT(17)|BIT(18)
  |BIT(19)|BIT(21)|BIT(22)|BIT(23)|BIT(27);
  
  xtos_set_interrupt_handler(0,rtcHandler);
  ets_isr_unmask(1u << 0); 
  REG(DPORT_PRO_GPIO_INTERRUPT_MAP_REG)[0]=0;
  REG(GPIO_PIN_REG)[2]=BIT(15)|BIT(8)|BIT(8);
  //7risingedge 8falling) 15prointerrupt 13appinterrup
  //REG(GPIO_PIN_REG)[16]=BIT(15)|BIT(8)|BIT(8); 
  //REG(GPIO_PIN_REG)[17]=BIT(15)|BIT(8)|BIT(8); /
  REG(IO_MUX_GPIO2_REG)[0]=BIT(9)|BIT(8); //input enable
  //REG(IO_MUX_GPIO16_REG)[0]=BIT(9)|BIT(8); //input enable16
  //REG(IO_MUX_GPIO16_REG)[1]=BIT(9)|BIT(8); //input enable17
  for (;;) { }
  return 0;
}  

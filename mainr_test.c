#include <mdk.h>


//#define delaysiz (1<<18)
#define delaysiz (1<<17)
static uint8_t delaybuff[delaysiz];
uint8_t *delptr=delaybuff; 
static uint32_t delayptr=0;
uint8_t del=0;
uint8_t dell;


void rtcHandler() {
  //uint32_t r = REG(GPIO_STATUS_REG)[0];
  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF; 

  //r=BIT(2);
  //uint8_t delia=0;

    volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);
    uint32_t rdrr = REG(ESP32_SENS_SAR_MEAS_START1)[0];
    uint32_t rdr = rr[0];
    if(rdr&BIT(16)) {       
    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
     REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
    } //moved from whole phrase to just adc trigger
     if (GPIO_IN1_REG[0]&0x80) delayptr++;
     else delayptr--;
     delayptr=delayptr&0x1FFFF;
     GPIO_OUT_REG[0]=(uint32_t)(delayptr<<12);
     
     
     dell = delptr[delayptr];
     REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((dell&0xFF)<<19);
     //REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((delayptr&0xFF)<<19);
     REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((REG(RNG_REG)[0]&0xFF)<<19);
     //if (~GPIO_IN1_REG[0]&0x8) 
     delptr[delayptr]=(uint8_t)(rdr>>4);
     //dell = delaybuff[delayptr];
    //}  
    //del++;
    //if((delayptr%400)==0) printf("h%08x %08x %08x %08x\n",(int)rdr,(int)dell,(int)del,(int)GPIO_IN1_REG[0]);
                     
                     
    printf("h%08x %08x %08x %08x\n",(int)rdr,(int)rdrr,(int)del,(int)GPIO_IN1_REG[0]);
  
    //REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((delayptr&0xFF)<<19);

   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);

}

void prr(const char * c, uint32_t r)  {
  printf("%s %08x\n",c,(int)REG(r)[0]);
}
void printR(){
  prr("APB_SARADC_CTRL_REG",APB_SARADC_CTRL_REG);
  prr("APB_SARADC_CTRL2_REG",APB_SARADC_CTRL2_REG);
  
  prr("SENS_SAR_READ_CTRL_REG",SENS_SAR_READ_CTRL_REG);
  prr("SENS_SAR_READ_CTRL2_REG",SENS_SAR_READ_CTRL2_REG);
  prr("APB_SARADC_CTRL2_REG",APB_SARADC_CTRL2_REG);
  prr("APB_SARADC_CTRL2_REG",APB_SARADC_CTRL2_REG);
  
  prr("APB_SARADC_FSM_REG",APB_SARADC_FSM_REG);
  prr("APB_SARADC_SAR1_PATT_TAB1_REG",APB_SARADC_SAR1_PATT_TAB1_REG);
  prr("I2S_CONF_REG",I2S_CONF_REG);
  prr("I2S_INT_RAW_REG",I2S_INT_RAW_REG);
  prr("I2S_CONF1_REG",I2S_CONF1_REG);
  prr("I2S_CONF2_REG",I2S_CONF2_REG);
  prr("I2S_CLKM_CONF_REG",I2S_CLKM_CONF_REG);
  prr("I2S_SAMPLE_RATE_CONF_REG",I2S_SAMPLE_RATE_CONF_REG);
  prr("I2S_CONF_SINGLE_DATA_REG",I2S_CONF_SINGLE_DATA_REG);
  prr("I2S_CONF_CHAN_REG",I2S_CONF_CHAN_REG);
  prr("I2S_IN_LINK_REG",I2S_IN_LINK_REG);
  prr("I2S_INLINK_DSCR_REG",I2S_INLINK_DSCR_REG);

}
  



void initRTC() {
//REG(DPORT_WIFI_CLK_EN_REG)[0]=0;
  //delptr=malloc(1<<16);
 // for (int i=0; i<delaysiz; i++) delaybuff[i]=0;
  REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(28); //inverse data is normal
  REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(29); //inv
  REG(SENS_SAR_MEAS_START1_REG)[0] |= BIT(31); //pad force
  REG(SENS_SAR_MEAS_START1_REG)[0] |= BIT(18); //start force
  REG(SENS_SAR_MEAS_START2_REG)[0] |= BIT(31);
  REG(SENS_SAR_MEAS_START2_REG)[0] |= BIT(18);

  REG(SENS_SAR_ATTEN1_REG)[0] = 0x2<<12;
  REG(SENS_SAR_ATTEN2_REG)[0] = 0x2;//4 IO_MUX and GPIO Matrix (GPIO, IO_MUX)
  //REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)3<<18); //clear force xpd 
  
  printf("SARMEASCTRL%08x\n",(int)REG(SENS_SAR_MEAS_CTRL_REG)[0]);
  REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFFF<<0); //clear fsm
  //REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFF<<4); //clear fsm truly
//0707 0000
// REG(SENS_SAR_MEAS_CTRL_REG)[0] = 0x073F0380;
  printf("SARMEASCTRL%08x\n",(int)REG(SENS_SAR_MEAS_CTRL_REG)[0]);
  //NNsar2xpdwait             Nsarrstbfsm Nxpdsarfsm 
  //Nampshortgnd Nampshortref Namprstfb Nxpdsarampfsm
//try different fsm
//https://github.com/krzychb/esp32-lna/blob/master/components/lna/lna.c
  REG(SENS_SAR_MEAS_WAIT1_REG)[0] = 0x00010001;
  
  
  printf("SARMEASWAIT2%08x\n",(int)REG(SENS_SAR_MEAS_WAIT2_REG)[0]);
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= (2<<16);//powerdiwb forx xpdamp0
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)0xFFFF);
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= 0x2;//|BIT(17)|BIT(19);
   REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= BIT(18)|BIT(19); //force power on to xpd sar
   printf("SARMEASWAIT2%08x\n",(int)REG(SENS_SAR_MEAS_WAIT2_REG)[0]);

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
  printR();
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
  
  //36 and 39
  REG(GPIO_ENABLE_REG)[3]=0;
  REG(IO_MUX_GPIO36_REG)[0]=BIT(9)|BIT(8); //input enable
  REG(IO_MUX_GPIO36_REG)[3]=BIT(9)|BIT(8); //input enable
  REG(IO_MUX_GPIO34_REG)[1]=BIT(9)|BIT(8); //input enable
  
  
  xtos_set_interrupt_handler(0,rtcHandler);
  //ets_isr_unmask(1u << 0); 
  //REG(DPORT_PRO_GPIO_INTERRUPT_MAP_REG)[0]=0;
  //REG(GPIO_PIN_REG)[2]=BIT(15)|BIT(8)|BIT(8);
  //7risingedge 8falling) 15prointerrupt 13appinterrup
  //REG(IO_MUX_GPIO2_REG)[0]=BIT(9)|BIT(8); //input enable
  for (;;) {
  
  rtcHandler();
  spin(1000000);
  
  
   }
  return 0;
}  

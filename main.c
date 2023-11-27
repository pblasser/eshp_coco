#include <mdk.h>



#define delaysiz (1<<17)
static uint8_t delaybuff[delaysiz];
static uint16_t dmabuff[300];
static uint32_t dmall[3];
static int delayptr;
  int i = 0;


void digHandler() {
  uint32_t r = REG(GPIO_STATUS_REG)[0];
  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;
  if (r & BIT(2)){  
    
   //volatile uint32_t *rrr = REG(I2S_FIFO_RD_REG);
   volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);
   delaybuff[delayptr]=(uint8_t)(rr[0]>>4);
   delayptr++;
   delayptr=delayptr&0x1FFFF;
   dmabuff[0]=0;
   printf("fifo%08x sarmeas%08x i2sc2%08x i2sc%08x dma%08x\n",(int)REG(I2S_FIFO_RD_REG)[0],(int)rr[0],(int)REG(I2S_CONF2_REG)[0],(int)REG(I2S_CONF_REG)[0],(int)dmabuff[i%300]);//(int)REG(I2S_INT_RAW_REG)[0]);
   i++;
     REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((REG(RNG_REG)[0]&0xFF)<<19);
    REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);
  }
 //printf("currinlink%08x\n",(int)REG(I2S_INLINK_DSCR_REG)[0]);
//   REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6); //pin 34
//    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19); //pin g4
//    REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(18)|BIT(31)|BIT(19+6)|BIT(17);
//    REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(18)|BIT(31)|BIT(19)|BIT(17);
//REG(I2S_CONF2_REG)[0]=BIT(5); //lcd5 extadc6
//    REG(I2S_LC_HUNG_CONF_REG)[0]=0x10;
  //REG(APB_SARADC_CTRL_REG)[0]|=BIT(1)|BIT(0)|BIT(23)|BIT(24);//2324clearpointer
  //REG(APB_SARADC_CTRL_REG)[0]&= ~(BIT(1)|BIT(0));
  //REG(APB_SARADC_CTRL_REG)[0]|=BIT(1)|BIT(0)|BIT(2);

    
 // REG(I2S_CONF_REG)[0]=BIT(5);//1 is rx reset, 5 is rxstart
    
    
}


void initDIG() {
  //printf("dportwifi%08x\n",(int)REG(DPORT_WIFI_CLK_EN_REG)[0]);
  //REG(DPORT_WIFI_CLK_EN_REG)[0]=0;
  //printf("dportwifi%08x\n",(int)REG(DPORT_WIFI_CLK_EN_REG)[0]);

  printf("SARMEASCTRL%08x\n",(int)REG(SENS_SAR_MEAS_CTRL_REG)[0]);
  REG(SENS_SAR_MEAS_CTRL_REG)[0] &= ~((uint32_t)0xFFFF<<0); //clear fsm
  printf("SARMEASCTRL%08x\n",(int)REG(SENS_SAR_MEAS_CTRL_REG)[0]);
  printf("SARMEASWAIT2%08x\n",(int)REG(SENS_SAR_MEAS_WAIT2_REG)[0]);
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] &= ~((uint32_t)0xFFFF);
  REG(SENS_SAR_MEAS_WAIT2_REG)[0] |= BIT(17)|BIT(18)|BIT(19)|BIT(1); //force power on to xpd sar
   printf("SARMEASWAIT2%08x\n",(int)REG(SENS_SAR_MEAS_WAIT2_REG)[0]);

  REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(31)|BIT(19+6)|BIT(18); //pin 34 startforce
  REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(31)|BIT(19)|BIT(18); //pin g4
  REG(SENS_SAR_READ_CTRL_REG)[0] |= BIT(27)|BIT(28); //dig force inv
  printf("sarreadctrl%08x\n",(int)REG(SENS_SAR_READ_CTRL_REG)[0]);
  REG(SENS_SAR_READ_CTRL2_REG)[0] |= BIT(28)|BIT(29);//|130; //dig force inv
  printf("sarreadctrl2%08x\n",(int)REG(SENS_SAR_READ_CTRL2_REG)[0]);
  
  //printf("sarforce%08x\n",(int)REG(SENS_SAR_START_FORCE_REG)[0]);
  //REG(SENS_SAR_START_FORCE_REG)[0] |= BIT(10);
  //printf("sarforce%08x\n",(int)REG(SENS_SAR_START_FORCE_REG)[0]);
  //printf("apbctrl%08x\n",(int)REG(APB_CTRL_SYSCLK_CONF_REG)[0]);
  //REG(APB_CTRL_SYSCLK_CONF_REG)[0] |= BIT(11);
  //printf("apbctrl%08x\n",(int)REG(APB_CTRL_SYSCLK_CONF_REG)[0]);

  printf("sarfsm%08x\n",(int)REG(APB_SARADC_FSM_REG)[0]);
  REG(APB_SARADC_FSM_REG)[0]=0x0216FF08;
  //samplecycle2 startwait16 stanbyFF rstb8 
  printf("sarfsm%08x\n",(int)REG(APB_SARADC_FSM_REG)[0]);
  #define CTRLJING BIT(26)|BIT(8)|BIT(6)|BIT(3)|BIT(2)
  #define CTRLJONG BIT(24)|BIT(23)
    //26datatoi2s 25sarsel 9clkdiv4 6clkgated 3double 2sar2mux
  REG(APB_SARADC_CTRL_REG)[0]=CTRLJONG|CTRLJING;
  REG(APB_SARADC_CTRL_REG)[0]=CTRLJING;
  printf("saradc%08x\n",(int)REG(APB_SARADC_CTRL_REG)[0]);
  REG(APB_SARADC_CTRL2_REG)[0]|=BIT(10)|BIT(9);
  printf("saradc2%08x\n",(int)REG(APB_SARADC_CTRL2_REG)[0]);
  REG(APB_SARADC_SAR1_PATT_TAB1_REG)[0] = 0x6D6D6D6D; 
printf("pat%08x\n",(int)REG(APB_SARADC_SAR1_PATT_TAB1_REG)[0]);
  REG(APB_SARADC_SAR2_PATT_TAB1_REG)[0] = 0x0D0D0D0D;







  //printf("dportperip%08x\n",(int)REG(DPORT_PERIP_CLK_EN_REG)[0]);
  REG(DPORT_PERIP_CLK_EN_REG)[0]|=BIT(4);
  //printf("dportperip%08x\n",(int)REG(DPORT_PERIP_CLK_EN_REG)[0]);
  REG(DPORT_PERIP_RST_EN_REG)[0]|=BIT(4);
  REG(DPORT_PERIP_RST_EN_REG)[0]=0;


  REG(I2S_CONF_REG)[0]=0xF|BIT(17)|BIT(9); //txrxreset fifor 
  REG(I2S_CONF_REG)[0]=BIT(17)|BIT(9); 

  //17msbright 9rightfirst
  //REG(I2S_CONF_REG)[0]&= ~((uint32_t)0x3F);//reset rx, fifo
  //REG(I2S_CONF_REG)[0]|=BIT(5);770x30;
  //bit 5 rx start
  REG(I2S_CONF1_REG)[0]|=BIT(7);//PCM bypass
  
  REG(I2S_LC_CONF_REG)[0]=0xF|BIT(10);//resets
  REG(I2S_LC_CONF_REG)[0]=BIT(10);//bursts
  
  REG(I2S_CONF2_REG)[0]=BIT(5);//LCD enable
  printf("i2sconf2%08x\n",(int)REG(I2S_CONF2_REG)[0]);
  
  REG(I2S_FIFO_CONF_REG)[0]=BIT(12)|BIT(11)|BIT(5)|BIT(16)|BIT(20);
  //20forcemod 16rxmod 11datanum32 12dma descriptor enable 5rxdatanum32
  //REG(I2S_FIFO_CONF_REG)[0]=BIT(5)|BIT(11)|BIT(19)|BIT(20);
  //rxdata5 txdata11 forcemodtxrx1920
  printf("i2sfifoconf%08x\n",(int)REG(I2S_FIFO_CONF_REG)[0]);
  
  REG(I2S_CONF_CHAN_REG)[0]=BIT(3); //3singlechanrx
  REG(I2S_PDM_CONF_REG)[0]=0;;
  REG(I2S_CLKM_CONF_REG)[0]|=BIT(20)|4;//clockenable
  printf("i2sclkmconf%08x\n",(int)REG(I2S_CLKM_CONF_REG)[0]);
    printf("i2scrconf%08x\n",(int)REG(I2S_SAMPLE_RATE_CONF_REG)[0]);
  REG(I2S_SAMPLE_RATE_CONF_REG)[0]=BIT(22)|BIT(16)|(50<<6)|6;
  //16bit length channels 12+4 18+4
  printf("i2scrconf%08x\n",(int)REG(I2S_SAMPLE_RATE_CONF_REG)[0]);


#define bufflough 7
  dmall[0]=0xC0|BIT(bufflough+12)|BIT(bufflough);
  //owner dma, eof, 128, 128
  uint16_t*buff=&dmabuff[0];
  dmall[1]=(uint32_t)buff;
  dmall[2]=0;
  uint8_t*duff=&delaybuff[0];
  uint32_t*muff=&dmall[0];
  printf("iBUFF%08x dBUFF%08x dBmall%08x\n",(int)buff, (int)duff, (int)muff);



  REG(APB_SARADC_CTRL_REG)[0]=CTRLJONG|CTRLJING;
  
  
  REG(I2S_RXEOF_NUM_REG)[0]=BIT(bufflough-2);
  REG(I2S_IN_LINK_REG)[0]=(0xFFFFF&(int)muff);
  
  REG(APB_SARADC_CTRL_REG)[0]=CTRLJING;
   //pattern pointer cleared
    REG(I2S_IN_LINK_REG)[0]|=BIT(29);
    printf("inlink%08x\n",(int)REG(I2S_IN_LINK_REG)[0]);
  //REG(I2S_CONF_REG)[0]=BIT(5)
 //REG(I2S_CONF_REG)[0]=BIT(5)|BIT(1);
  REG(I2S_CONF_REG)[0]=BIT(17)|BIT(9)|BIT(5); //start rx
  printf("i2sconf%08x\n",(int)REG(I2S_CONF_REG)[0]);
  printf("currinlink%08x\n",(int)REG(I2S_INLINK_DSCR_REG)[0]);


  REG(ESP32_SENS_SAR_MEAS_START1)[0]=BIT(31)|BIT(19+6); //pin 34
  REG(ESP32_SENS_SAR_MEAS_START2)[0]=BIT(31)|BIT(19); //pin g4


  REG(SENS_SAR_TOUCH_ENABLE_REG)[0] = 0; //touch pads off
  //REG(ESP32_RTCIO_ADC_PAD)[0] = BIT(22)|BIT(21)|BIT(18)|BIT(27)|BIT(26)|BIT(23);// |BIT(28)|BIT(29);
//28 is adc2 mux rtc, 22 fun sel 21, 18 ie
//for sar2 that is

}





int main(void) {
  REG(ESP32_SENS_SAR_DAC_CTRL1)[0] = 0x0; 
  REG(ESP32_SENS_SAR_DAC_CTRL2)[0] = 0x0; 

  initDIG();
  
  xtos_set_interrupt_handler(0,digHandler);
  ets_isr_unmask(1u << 0); 
  REG(DPORT_PRO_GPIO_INTERRUPT_MAP_REG)[0]=0;
  REG(GPIO_PIN_REG)[2]=BIT(15)|BIT(8)|BIT(8);
  //7risingedge 8falling) 15prointerrupt 13appinterrup
  REG(IO_MUX_GPIO2_REG)[0]=BIT(9)|BIT(8); //input enable



  for (;;) {}
  return 0;
}  

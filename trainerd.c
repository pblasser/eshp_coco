#include <mdk.h>


//#define delaysiz (1<<18)
#define delaysiz (1<<17)
static uint8_t delaybuff[delaysiz];
static uint16_t dmabuff[300];
static uint32_t dmall[3];
static int delayptr;
  int i = 0;


void digHandler() {
  //uint32_t r = REG(GPIO_STATUS_REG)[0];
  REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;

    
   //volatile uint32_t *rrr = REG(I2S_FIFO_RD_REG);
   volatile uint32_t *rr = REG(ESP32_SENS_SAR_MEAS_START2);
   delaybuff[delayptr]=(uint8_t)(rr[0]>>4);
   delayptr++;
   delayptr=delayptr&0x1FFFF;
   //dmabuff[0]=0;
   printf("fifo%08x sarmeas%08x i2sc2%08x i2sc%08x dma%08x\n",(int)REG(I2S_FIFO_RD_REG)[0],(int)rr[0],(int)REG(I2S_CONF2_REG)[0],(int)REG(I2S_CONF_REG)[0],(int)dmabuff[i%300]);//(int)REG(I2S_INT_RAW_REG)[0]);
   i++;
     REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((REG(RNG_REG)[0]&0xFF)<<19);
    REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((i&0xFF)<<19);

 //pintf("currinlink%08x\n",(int)REG(I2S_INLINK_DSCR_REG)[0]);
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


void prr(const char * c, uint32_t r)  {
  printf("%s %08x\n",c,(int)REG(r)[0]);
}
void printR(){
  prr("APB_SARADC_CTRL_REG",APB_SARADC_CTRL_REG);
  prr("APB_SARADC_CTRL2_REG",APB_SARADC_CTRL2_REG);
  
  prr("SENS_SAR_READ_CTRL_REG",SENS_SAR_READ_CTRL_REG);
  prr("SENS_SAR_READ_CTRL2_REG",SENS_SAR_READ_CTRL2_REG);
  
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
    prr("I2S_INT_ST_REG",I2S_INT_ST_REG);
    prr("I2S_LC_STATE0_REG",I2S_LC_STATE0_REG);
        prr("I2S_LC_STATE1_REG",I2S_LC_STATE1_REG);
   prr("I2S_STATE_REG",I2S_STATE_REG); 
     prr("I2S_RXEOF_NUM_REG",I2S_RXEOF_NUM_REG);   
      prr("I2S_CONF_SINGLE_DATA_REG",I2S_CONF_SINGLE_DATA_REG); 
            prr("I2S_IN_EOF_DES_ADDR_REG",I2S_IN_EOF_DES_ADDR_REG); 
                  prr("I2S_INLINK_DSCR_BF0_REG",I2S_INLINK_DSCR_BF0_REG); 
                        prr("I2S_INLINK_DSCR_BF1_REG",I2S_INLINK_DSCR_BF1_REG);   
                            prr("I2S_PD_CONF_REG",I2S_PD_CONF_REG); 
                            for (int j=0; j<200;j++) printf("%08x ",(int)dmabuff[j]);
                              printf("%08x\n",0);
          prr("ESP32_SENS_SAR_MEAS_START1",ESP32_SENS_SAR_MEAS_START1);         
          prr("I2S_CONF_SINGLE_DATA_REG",I2S_CONF_SINGLE_DATA_REG);  
                    prr("APB_SARADC_FSM_REG",APB_SARADC_FSM_REG);  
          
                      
                          

}
  
#define CHANG(reg, val) \
 preval = (int)REG(reg)[0]; \
 REG(reg)[0] = (val); \
 printf("%s %08x--->%08x\n",#reg,preval,(int)REG(reg)[0]); 
 
#define CHANGOR(reg, val) \
 preval = (int)REG(reg)[0]; \
 REG(reg)[0] |= (val); \
 printf("%s %08x--->%08x\n",#reg,preval,(int)REG(reg)[0]); 
 
#define CHANGNO(reg, val) \
 preval = (int)REG(reg)[0]; \
 REG(reg)[0] &= ~(uint32_t)(val); \
 printf("%s %08x--->%08x\n",#reg,preval,(int)REG(reg)[0]); 
 
#define CHANGNOR(reg, val) \
 preval = (int)REG(reg)[0]; \
 REG(reg)[0] |= (val); \
 REG(reg)[0] &= ~(uint32_t)(val); \
 printf("%s %08x-%08x->%08x\n",#reg,preval,(int)(preval|(int)(val)),(int)REG(reg)[0]); 
 
int preval;

void initDIG() {
  CHANG(SENS_SAR_ATTEN1_REG,0x2<<12)
  CHANG(SENS_SAR_ATTEN2_REG,0x2)
  //CHANG(DPORT_WIFI_CLK_EN_REG,0)







  //I2S CLOCK
  CHANGOR(DPORT_PERIP_CLK_EN_REG,BIT(4))
  CHANGNOR(DPORT_PERIP_RST_EN_REG,BIT(4))

  //ADC POWER ALWAYS ON
  CHANGNO(SENS_SAR_MEAS_CTRL_REG,(uint32_t)0xFFFF)
  CHANGOR(SENS_SAR_MEAS_WAIT2_REG,BIT(17)|BIT(18)|BIT(19))
  CHANGNO(SENS_SAR_MEAS_WAIT2_REG,(uint32_t)0xFFFF)

  CHANG(I2S_INT_ENA_REG,0) //disable interrupt
  CHANGNO(I2S_INT_CLR_REG,0)
  CHANG(I2S_CONF_REG,0)
  CHANGNOR(I2S_CONF_REG,BIT(1))//rx reset
  CHANGOR(I2S_CONF_REG,BIT(17)|BIT(9)) //msb right first
  
  CHANGNOR(I2S_CONF_REG,BIT(3)) //rx fifo reset
  CHANGOR(I2S_CONF1_REG,BIT(7))//PCM bypass
  
  //enable DMA
  CHANG(I2S_LC_CONF_REG,0)
  CHANGNOR(I2S_LC_CONF_REG,BIT(2))//ahb fifo rst
  CHANGNOR(I2S_LC_CONF_REG,BIT(3))//ahb reset
  CHANGNOR(I2S_LC_CONF_REG,BIT(0))//in rst
  CHANGOR(I2S_LC_CONF_REG,BIT(10))//burst inlink

  CHANG(I2S_CONF2_REG,0);//LCD enable
  CHANGOR(I2S_CONF2_REG,BIT(5));//LCD enable
  
  CHANG(I2S_FIFO_CONF_REG,BIT(12)|BIT(5)|BIT(20))
  //bit 16 is single channel|BIT(17)) is 32bit
  //20forcemod 16rxmod 12dmaconnect 5rxdatanum32
  CHANG(I2S_CONF_CHAN_REG,BIT(3)) //3singlechanrx
  CHANG(I2S_PDM_CONF_REG,0)
  CHANGOR(I2S_CLKM_CONF_REG,BIT(21))//clockenable
  CHANGNO(I2S_SAMPLE_RATE_CONF_REG,(63<<6))
  CHANGOR(I2S_SAMPLE_RATE_CONF_REG,(50<<6))
  prr("I2S_INT_RAW_REG",I2S_INT_RAW_REG);
  
  //adc set i2s data len patterns should be zero
  
  //adc set data pattern
  CHANG(APB_SARADC_SAR1_PATT_TAB1_REG,0x6E6E6E6E)
  CHANG(APB_SARADC_SAR2_PATT_TAB1_REG,0x0E0E0E0E)
  
  //adc set controller DIG
  CHANGOR(SENS_SAR_READ_CTRL_REG,BIT(27)|BIT(28))
  CHANGOR(SENS_SAR_READ_CTRL2_REG,BIT(28)|BIT(29))
  //CHANG(ESP32_SENS_SAR_MEAS_START1,BIT(31)|BIT(19+6)|BIT(18)) 
  //CHANG(ESP32_SENS_SAR_MEAS_START2,BIT(31)|BIT(19)|BIT(18))
   //seems to not need bitmap
 
 
   #define CTRLJING BIT(26)|BIT(8)|BIT(6)|BIT(2)|BIT(3)
   #define CTRLPATT 0 //BIT(15)|BIT(19)
  #define CTRLJONG BIT(24)|BIT(23)
    //26datatoi2s 25sarsel 9clkdiv4 6clkgated 3double 2sar2mux
    //8 clock div 2   
      CHANG(APB_SARADC_CTRL_REG,CTRLJING|CTRLPATT)


  //REG(APB_SARADC_FSM_REG)[0]=0x0216FF08;
  //CHANG(APB_SARADC_FSM_REG,0xFF056408) //from code example
 //timekeep-1 startwait5 standbywait100 rstbwait8

  CHANGNOR(APB_SARADC_CTRL_REG,CTRLJONG)
  CHANGOR(I2S_CLKM_CONF_REG,BIT(21)|4)//clockenable
  CHANGOR(APB_SARADC_CTRL2_REG,BIT(10)|BIT(9)|BIT(0));
  CHANG(APB_SARADC_CTRL2_REG,BIT(10)|BIT(9)|BIT(1)|BIT(0));//trying to limit to 1

#define bufflough 7
  dmall[0]=0xC0|BIT(bufflough+12)|BIT(bufflough);
  //owner dma, eof, 128, 128
  uint16_t*buff=&dmabuff[0];
  dmall[1]=(uint32_t)buff;
  dmall[2]=0;
  uint8_t*duff=&delaybuff[0];
  uint32_t*muff=&dmall[0];
  printf("iBUFF%08x dBUFF%08x dBmall%08x\n",(int)buff, (int)duff, (int)muff);



  CHANG(APB_SARADC_CTRL_REG,CTRLJONG|CTRLJING|CTRLPATT)
  
  
  CHANG(I2S_RXEOF_NUM_REG,BIT(bufflough-2))
    CHANG(I2S_RXEOF_NUM_REG,1)
  CHANGOR(I2S_IN_LINK_REG,(0xFFFFF&(int)muff))
      //REG(I2S_IN_LINK_REG)[0]|=BIT(30);
  CHANGNOR(I2S_CONF_REG,BIT(1))//rx reset  
  CHANGNOR(I2S_CONF_REG,BIT(3)) //rx fifo reset
    CHANG(APB_SARADC_CTRL_REG,CTRLJING|CTRLPATT)
   //pattern pointer cleared
    CHANGOR(I2S_IN_LINK_REG,BIT(29))
    
    REG(I2S_INT_CLR_REG)[0]=0xFFFF;
  //REG(I2S_CONF_REG)[0]=BIT(5)
 //REG(I2S_CONF_REG)[0]=BIT(5)|BIT(1);
  REG(I2S_CONF_REG)[0]=BIT(17)|BIT(9)|BIT(5); //start rx
}

 
int main(void) {
for (uint16_t j=0; j<200;j++) dmabuff[j]=j+1;
  

  printR();
  REG(ESP32_SENS_SAR_DAC_CTRL1)[0] = 0x0; 
  REG(ESP32_SENS_SAR_DAC_CTRL2)[0] = 0x0; 
  initDIG();


   

      
      
  xtos_set_interrupt_handler(0,digHandler);
  //ets_isr_unmask(1u << 0); 
  //REG(DPORT_PRO_GPIO_INTERRUPT_MAP_REG)[0]=0;
  //REG(GPIO_PIN_REG)[2]=BIT(15)|BIT(8)|BIT(8);
  //7risingedge 8falling) 15prointerrupt 13appinterrup
  //REG(IO_MUX_GPIO2_REG)[0]=BIT(9)|BIT(8); //input enable
  if (true) {
     printR();
   spin(100000);
   digHandler();
      printR();
      REG(I2S_IN_LINK_REG)[0]|=BIT(29);
      spin(100000);
   digHandler();
      printR();
  //rtcHandler();
  spin(1000000);
  
  
   }
  return 0;
}  

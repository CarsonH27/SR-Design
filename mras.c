/* External 100 kHz excitation. ADC1 triggered by TIM6 at 800 kHz.
 * Configure ADC DMA NORMAL, halfword, memory increment; UART2 115200.
 * No DAC. Amplitude only; no external phase reference required. */
#include "main.h"
#include "mras.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
#define N 256U
#define BLOCKS 16U
static uint16_t samples[N];
static volatile uint8_t done;
static float refs[6];
static uint8_t valid[6];
static const char *names[]={"Air","Aluminum","Copper","Steel","Stainless steel","Brass"};
static const int16_t c[8]={32767,23170,0,-23170,-32767,-23170,0,23170};
static const int16_t s[8]={0,23170,32767,23170,0,-23170,-32767,-23170};
static void send(const char *p){HAL_UART_Transmit(&huart2,(uint8_t*)p,strlen(p),100);}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h){if(h->Instance==ADC1)done=1;}
static int measure(float *amp){
 float total=0;
 for(unsigned b=0;b<BLOCKS;b++){
  done=0; __HAL_TIM_SET_COUNTER(&htim6,0);
  if(HAL_ADC_Start_DMA(&hadc1,(uint32_t*)samples,N)!=HAL_OK)return 0;
  if(HAL_TIM_Base_Start(&htim6)!=HAL_OK){HAL_ADC_Stop_DMA(&hadc1);return 0;}
  uint32_t t=HAL_GetTick();
  while(!done && (HAL_GetTick()-t)<20U){}
  HAL_TIM_Base_Stop(&htim6); HAL_ADC_Stop_DMA(&hadc1);
  if(!done)return 0;
  int64_t i=0,q=0;
  for(unsigned k=0;k<N;k++){
   if(samples[k]<=5 || samples[k]>=4090)return 0;
   i+=(int32_t)samples[k]*c[k&7]; q-=(int32_t)samples[k]*s[k&7];
  }
  total+=2.0f*hypotf((float)i,(float)q)/(N*32767.0f);
 }
 *amp=total/BLOCKS; return 1;
}
void MRAS_Init(void){
 if(HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED)!=HAL_OK){send("ADC calibration failed\r\n");return;}
 send("MRAS: 0=air, 1=Al, 2=Cu, 3=steel, 4=stainless, 5=brass, r=read\r\n");
}
void MRAS_Task(void){
 uint8_t cmd; char msg[128]; float a;
 if(HAL_UART_Receive(&huart2,&cmd,1,100)!=HAL_OK)return;
 if(!((cmd>='0' && cmd<='5') || cmd=='r'))return;
 if(!measure(&a)){send("ADC timeout or clipping\r\n");return;}
 if(cmd!='r'){
  unsigned j=cmd-'0'; refs[j]=a; valid[j]=1;
  snprintf(msg,sizeof msg,"Calibrated %s: %ld counts peak\r\n",names[j],(long)(a+0.5f)); send(msg);return;
 }
 for(unsigned j=0;j<6;j++)if(!valid[j]){send("Calibrate all 6 references first\r\n");return;}
 unsigned best=0; float d=1e9f, second=1e9f;
 for(unsigned j=0;j<6;j++){
  float e=fabsf(a-refs[j]);
  if(e<d){second=d;d=e;best=j;}else if(e<second)second=e;
 }
 /* Provisional thresholds in ADC counts; replace using repeatability data. */
 const char *label=(d>20.0f || second-d<10.0f)?"Unknown / ambiguous":names[best];
 snprintf(msg,sizeof msg,"%s | peak=%ld counts | distance=%ld\r\n",label,(long)(a+0.5f),(long)(d+0.5f));send(msg);
}

NUCLEO-L432KC external excitation, 100 kHz sine.
CubeIDE: configure ADC1 PA0 IN5, 12-bit right aligned, single rank,
external TIM6 TRGO rising edge, continuous conversion disabled,
DMA continuous requests enabled, DMA NORMAL peripheral/memory halfword,
memory increment enabled, DMA IRQ enabled. ADC clock/sample time must allow
800 kS/s. TIM6 update TRGO, PSC=0 ARR=99 for 80 MHz timer clock.
USART2 115200 8N1, board virtual COM. No DAC.
Copy mras.c into Core/Src, mras.h into Core/Inc. Include mras.h in main.c.
After all MX initialization call MRAS_Init(); inside while(1) call MRAS_Task();
Do not duplicate HAL_ADC_ConvCpltCallback; merge if already defined.
Build and Run as STM32 C/C++ application. 
Open serial terminal, 
send 0 with air,
1 aluminum, 2 copper, 3 steel, 4 stainless, 5 brass, r to classify once.
Keep geometry and generator amplitude fixed. Bias analog input around 1.65 V
and keep within 0..VDDA, with adequate analog bandwidth and anti-alias filter.
Ground reference must be safe and common. Never connect coil directly.
Amplitude-only, no phase reference. Calibration in RAM; thresholds provisional.
Not compiled with target HAL or tested on hardware.

// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"

// Pin bitbands
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))

#define MAX_CHARS 80
#define MAX_FIELDS 6

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4

// PortD masks
#define TX_MASK 8
#define A0_MASK 4
#define FSS_MASK 2
#define CLK_MASK 1


#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
#define pi 3.14159265358979323846

char str[MAX_CHARS+1];
char str1[MAX_CHARS+1];
char str2[MAX_CHARS+1];
char array[MAX_CHARS+1];
char Cmd[10];

uint8_t pos[MAX_FIELDS];
uint8_t argCount=0;
uint8_t count=0;
uint16_t argNo=0;
uint16_t tdata=0;
uint16_t LUT[3][4096];
uint8_t channel1=0;
uint8_t channel2=1;
uint32_t del_phi1=0;
uint32_t del_phi2=0;
uint32_t phi1=0;
uint32_t phi2=0;
uint32_t cycles=0;
uint32_t nofcycles=0;
uint32_t counter=0;
uint8_t Trigger=0;
uint8_t channel=1;
float freq=0.0;
float amp=0;
float offset=0;
float freq1=0.0;
float freq2=0.0;
float gain=0.0;
float Vout=0.0;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A  peripherals
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0 | SYSCTL_RCGCADC_R1;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE;;

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                     // enable TX, RX, and module
    // Configure AIN0 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE3

    // Configure AIN1 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE2)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE2

       // Configure ADC
       ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
       ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
       ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
       ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
       ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
       ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

       // Configure ADC
              ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
              ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
              ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
              ADC1_SSMUX3_R = 1;                               // set first sample to AIN1
              ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
              ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

              // Enable clocks
                  SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;
                  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;

                  // Configure A0
                  GPIO_PORTD_DIR_R |= A0_MASK;                       // make bit 2 an output
                  GPIO_PORTD_DR2R_R |= A0_MASK;                      // set drive strength to 2mA
                  GPIO_PORTD_DEN_R |= A0_MASK;                         // enable bit 1 for digital

                  // Configure SSI1 pins for SPI configuration
                  GPIO_PORTD_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK; // make SSI1 TX, FSS, and CLK outputs
                  GPIO_PORTD_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK; // set drive strength to 2mA
                  GPIO_PORTD_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK; // select alternative functions
                  GPIO_PORTD_PCTL_R = GPIO_PCTL_PD3_SSI1TX | GPIO_PCTL_PD1_SSI1FSS | GPIO_PCTL_PD0_SSI1CLK; // map alt fns to SSI1
                  GPIO_PORTD_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK; // enable digital operation
                  GPIO_PORTD_PUR_R |= CLK_MASK;                      // SCLK must be enabled when SPO=1 (see 15.4)

                  // Configure the SSI1 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
                  SSI1_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI1 to allow re-configuration
                  SSI1_CR1_R = 0;                                    // select master mode
                  SSI1_CC_R = 0;                                     // select system clock as the clock source
                  SSI1_CPSR_R = (10);                                  // set bit rate to 1 MHz (if SR=0 in CR0)
                  SSI1_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit

                  SSI1_CR1_R |= SSI_CR1_SSE;                         // turn on SSI1


                      // Configure Timer 1 as the time base
                  // Enable clocks
                        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
                      TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
                      TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
                      TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
                      TIMER1_TAILR_R = 400;                       // set load value to 40e6 for 1 Hz interrupt rate
                      TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
                      NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
                      TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void timer1Isr()
{
    if(Trigger==1)
    {
    if(counter<cycles)
    {
        A0=0;
                   // set A0 for data
           A0 =1;
    counter++;
     phi1+=del_phi1;
     phi2+=del_phi2;
     //SSI1_DR_R= 45056+ 2048;
     SSI1_DR_R = LUT[channel1][phi1>>20];                  // write data
     SSI1_DR_R = LUT[channel2][phi2>>20];

    if(counter==cycles)
    {
        A0=0;
           A0 =1;
        Trigger=0; counter=0;
        phi1=0; phi2=0;
        del_phi1=0; del_phi2=0;
        SSI1_DR_R= 45056+ 2048;
        SSI1_DR_R= 12288+ 2048;

    }
    }
    else if(cycles==0)
    {
        A0=0;
           A0 =1;
        phi1+=del_phi1;
        phi2+=del_phi2;
        //SSI1_DR_R= 45056+ 2048;
   SSI1_DR_R = LUT[channel1][phi1>>20];                  // write data
   while(SSI1_SR_R & SSI_SR_BSY);
   SSI1_DR_R = LUT[channel2][phi2>>20];
    }
    }
    else if (Trigger==0)
    {

        A0=0;        // set A0 for data
                   A0 =1;
        SSI1_DR_R= 45056+ 2048;
        SSI1_DR_R= 12288+ 2048;
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getString(char str[],uint8_t maxChars)
{


    char c;
Loop1:
    c= getcUart0();
    while(1)
    {

        if(c==8 | c==127)
        {
            if(count>0)
            {

                count-- ;
            }
            else
            {
                goto Loop1;
            }
        }

    else
    {
            if(c==10 | c==13 )
            {
                str[count]=0;
                break;
            }
            else
            {
                if(c>=32)
                {
                    str[count++]=c;
                }
                else
                {
                    goto Loop1;
                }
            }
    }


        if(count == maxChars)
        {
            str[count]=0;
            break;

        }
        else
        {
            goto Loop1;

        }
      }

    }

void parseString(char str[], uint8_t pos[], uint8_t argCount,char str1[], char str2[],char* array)
{


        strcat(str2,str);

        //char *ptr = strtok(str, delim);
        uint8_t i;
        uint8_t j;
        putcUart0(count);
        uint8_t temp_count=0;
        for(i=0;i<count;i++)
               {
            if(str2[i]==32 | str2[i]==44)
                   {
                       str2[i]=0x00;
                   }
            if(str2[i]!=0x00)
                   {


                       str1[temp_count++] = str2[i];
                   }
               }


        putsUart0("\r\n The string after removing delimitters is:\r\n");
        putsUart0(str1);
       /* while (ptr != NULL)
        {
            putsUart0(ptr);


            strcat(str1,ptr);
            ptr = strtok(NULL, delim);

        }*/


        for(i=0;i<strlen(str);i++)
                {
                    for(j=0;j<strlen(str1);j++)
                    {
                    if(str1[j]==str[i] & (str[i-1]==32 | str[i-1]==44 | str[i-1]==0x00) & (str1[j] != str1[j+1]))
                    {
                        pos[argCount]=i;

                        argCount++;

                        break;
                    }
                    }
                }

        putsUart0("\r\n The no. of arguments are:\r\n");

        putcUart0(0x30+argCount);
        putcUart0(0x0a);
           putcUart0(0x0d);

        uint8_t l=0;
        putsUart0("\r\n The individual arguments are:\r\n");
        putcUart0(0x0a);
        putcUart0(0x0d);

    for(i=pos[l];i<count;i++)
    {
        if(str[i]!=0x00)
                         {

                             array[l++] = str1[i];
                         }
    }
        while(str1[l]!=NULL)
        {


            array[++l]=str[l];
        }
    putsUart0(array);
    return;
}

char* getArgString(uint16_t argNo)
{

    argCount=count;
    if(argNo<argCount)
    {
        char *array1[81];
        array1[argNo]=strtok(str2," ,");

        while(array1[argNo]!=NULL)
        {
            array1[argNo]=strtok(NULL," ,");

            return &str2[pos[argNo]];
        }

        }
}

uint32_t getArgInt(uint16_t argNo)
{
    int value=atoi(getArgString(argNo));
    return value;
}

float getArgFloat(uint16_t argNo)
{

    float value=atof(getArgString(argNo));
    return value;

}
// Request and read one sample from SS3
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

// Request and read one sample from SS3
int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

int sgn(float value)
{
    return value<0?-1:value>0?1:0;
}
void run()
{
    putsUart0("please enter run cmd");
    getString(str, MAX_CHARS);

    cycles=(uint32_t)(nofcycles*(1/freq)*100000);
    Trigger = 1;
    return;
}
void lookUpTable()
{

    if(channel==1)
    {
        if(channel1==0 & channel2==1){channel1=2;}
        else if(channel1==0 & channel2==2)
        {
            channel1=1;
        }
        else if(channel1==1 & channel2==0)
        {
            channel1=2;
        }
        else if(channel1==1 & channel2==2)
               {
                   channel1=0;
               }
        else if(channel1==2 & channel2==0)
               {
                   channel1=1;
               }
        else if(channel1==2 & channel2==1)
               {
                   channel1=0;
               }
       uint16_t i;
       if(strcmp(Cmd, "dc")==0)
              {
              for(i=0;i<4096;i++)
              {

                  LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp*sin(2*pi*i/4096))+4096*(-0+ 0.5));

              }
              }
       if(strcmp(Cmd, "differ")==0)
                     {
           offset=-Vout;
                     for(i=0;i<4096;i++)
                     {

                         LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset)  +4096*(0.5));

                     }
                     }
       else if(strcmp(Cmd, "sine")==0)
       {
       for(i=0;i<4096;i++)
       {

           LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp*sin(2*pi*i/4096))+4096*(-0+ 0.5));

       }
       }
       else if(strcmp(Cmd,"gain")==0)
             {
             for(i=0;i<4096;i++)
             {

                 LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+4*sin(2*pi*i/4096))+4096*(-0+ 0.5));

             }
             }
       else if(strcmp(Cmd, "square")==0)
          {


          for(i=0;i<4096;i++)
          {
              LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp*sgn(sin(2*pi*i/4096)))+4096*(-0+ 0.5));
          }
          }
       else if(strcmp(Cmd,"sqduty")==0)
         {
           char str12[10];
           putsUart0("\r\n Please enter the duty cycle \r\n");
           getString(str12,MAX_CHARS);
           putsUart0(str12);
           uint16_t dutycycle=atoi(str12);
           uint16_t value=(dutycycle*4096)/100;
           amp=4; offset = 0;
           for(i=0;i<4096;i++)
                    {
                       if(i<value)
                    {
                        LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp)+4096*(-0+ 0.5));
                    }
                       else {
                           LUT[channel1][i]=12288+2048;

                       }
                    }
         }

       else if(strcmp(Cmd, "sawtooth")==0)
          {


          for(i=0;i<4096;i++)
          {
              LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp*i/4096)+4096*(-0+ 0.5));
              //LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+(amp/2-(amp/pi)*pow(-1,i)*(sin(2*pi*i/4096))))+4096*(-0+ 0.5));
          }
          }
       else if(strcmp(Cmd, "triangle")==0)
                 {


                 for(i=0;i<2048;i++)
                 {
                     LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp*i/2048)+4096*(-0.01+ 0.5));
                 }
                 for(i=2048;i<4096;i++)
                               {
                                   LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+amp -amp*(i-2048)/2048)+4096*(-0.01+ 0.5));
                               }

                 }
       del_phi1 = ((1<<20)/100000)*4096*freq;

     }

    else if(channel==2)
        {

    if(channel2==0 & channel1==1)
    {
        channel2=2;
    }
    else if(channel2==0 & channel1==2)
    {
        channel2=1;
    }
    else if(channel2==1 & channel1==0)
    {
        channel2=2;
    }
    else if(channel2==1 & channel1==2)
           {
               channel2=0;
           }
    else if(channel2==2 & channel1==0)
           {
               channel2=1;
           }
    else if(channel2==2 & channel1==1)
           {
               channel2=0;
           }
   uint16_t i;
   float cal_val=4096*(-0+0.5);
   if(strcmp(Cmd, "dc")==0)
      {
      for(i=0;i<4096;i++)
      {

          LUT[channel2][i]=45056+ (uint16_t)(409.6*(-0.95)*(offset+amp*sin(2*pi*i/4096))+ cal_val);


      }
      }
   else if(strcmp(Cmd, "sine")==0)
   {
   for(i=0;i<4096;i++)
   {

       LUT[channel2][i]=45056+ (uint16_t)(409.6*(-0.95)*(offset+amp*sin(2*pi*i/4096))+ cal_val);


   }
   }
   if(strcmp(Cmd, "differ")==0)
                 {
       offset=-Vout;
                 for(i=0;i<4096;i++)
                 {

                     LUT[channel2][i]=45056+(uint16_t)(409.6*(-0.95)*(offset)  +4096*(0.5));

                 }
                 }
   else if(strcmp(Cmd, "square")==0)
       {



       for(i=0;i<4096;i++)
       {
           LUT[channel2][i]=45056+(uint16_t)(409.6*(-0.95)*(offset+amp*sgn(sin(2*pi*i/4096)))+4096*(-0+ 0.5));
       }
       }
   else if(strcmp(Cmd, "sawtooth")==0)
            {


            for(i=0;i<4096;i++)
            {
                LUT[channel2][i]=45056+(uint16_t)(409.6*(-0.95)*(offset+amp*i/4096)+4096*(-0+ 0.5));
                //LUT[channel1][i]=12288+(uint16_t)(409.6*(-0.95)*(offset+(amp/2-(amp/pi)*pow(-1,i)*(sin(2*pi*i/4096))))+4096*(-0+ 0.5));
            }
            }
   else if(strcmp(Cmd, "triangle")==0)
             {


             for(i=0;i<2048;i++)
             {
                 LUT[channel2][i]=45056+(uint16_t)(409.6*(-0.95)*(offset+amp*i/2048)+4096*(-0.01+ 0.5));
             }
             for(i=2048;i<4096;i++)
                           {
                               LUT[channel2][i]=45056+(uint16_t)(409.6*(-0.95)*(offset+amp -amp*(i-2048)/2048)+4096*(-0.01+ 0.5));
                           }

             }
   else if(strcmp(Cmd,"gain")==0)
               {
               for(i=0;i<4096;i++)
               {

                   LUT[channel2][i]=45056+(uint16_t)(409.6*(-0.95)*(offset+4*sin(2*pi*i/4096))+4096*(-0+ 0.5));

               }
               }
   del_phi2 = ((1<<20)/100000)*4096*freq;

         }
    if(strcmp(Cmd,"gain")==0)
             {
                 Trigger=1;
             }
    return;

}


void isCommand()
{

   if(strcmp(Cmd,"sine")==0 & count>=4)
   {
       putsUart0("Mode: sine");
       if(strcmp("1",array[1])==0 | strcmp("2",array[1])==0 )
       {
           lookUpTable();
       }
   }

   else if(strcmp(Cmd,"voltage")==0)
     {
         putsUart0("Mode: Voltage Command");
         uint16_t raw1=0;  uint16_t x1[16];
         char str5[100];
          uint8_t index1 = 0;
          uint16_t sum1 = 0; // total fits in 16b since 12b adc output x 16 samples

          float v1;
          uint8_t i;
          for (i = 0; i < 16; i++)
          {x1[i] = 0; }

          uint16_t integer; uint16_t fraction;
          uint16_t counter_voltage=0;
          while(true)
              {

                  if(channel==1){raw1 = readAdc0Ss3();}
                  else if(channel==2){raw1 = readAdc1Ss3();}
                  counter_voltage++;
                      // FIR sliding average filter with circular addressing
                      sum1 -= x1[index1];
                      sum1 += raw1;
                      x1[index1] = raw1;
                      index1 = (index1 + 1) % 16;
                      v1= (3.3*(sum1/16))/4096;
                      if (counter_voltage==100)
                      {
                      sprintf(str5, "\r\nFiltered Raw ADC0: %4u\r\n", sum1 / 16);
                      putsUart0(str5);
                      char str6[100];
                      integer = (uint16_t)(v1);
                      fraction = (uint16_t)((v1-integer)*1000);
                      sprintf(str6,"\r\nAbsolute Voltage  is: %4u.%4u\r\n",integer,fraction);
                      putsUart0(str6);
                      putsUart0("\r\n");
                      //waitMicrosecond(1000000);
                      break;
                      }
              }
     }

   else if(strcmp(Cmd,"dc")==0 )
   {
       putsUart0("Mode: dc");
     lookUpTable();
   }
   else if(strcmp(Cmd,"square")==0 )
     {

         putsUart0("Mode: square");
         lookUpTable();
     }
   else if(strcmp(Cmd,"differ")==0 )
        {
       uint16_t raw; float rawFloat;
            putsUart0("Mode: Differential");
            raw = readAdc0Ss3();
            rawFloat = (float)(raw);
            Vout= 3.3 * rawFloat / 4096;

            lookUpTable();
        }

   else if(strcmp(Cmd,"sawtooth")==0 )
     {

         putsUart0("Mode: sawtooth");
         lookUpTable();
     }
   else if(strcmp(Cmd, "triangle")==0)
   {

       putsUart0("Mode: triangle");
       lookUpTable();

   }
   else if(strcmp(Cmd, "ncycle")==0)
   {
       putsUart0("Please enter the no. of cycles");

       getString(str, MAX_CHARS);
       nofcycles=atoi(str);


   }
   else if(strcmp(Cmd,"gain")==0 )
    {
        putsUart0("Mode: Gain");
        putsUart0("\r\n please enter  frequency 1: \r\n");
            char str8[10];
            getString(str8, MAX_CHARS);
            putsUart0(str8);
            freq1= atoi(str8);
            putsUart0("\r\n please enter  frequency 2: \r\n");
            char str9[10];
            getString(str9, MAX_CHARS);
            putsUart0(str9);
            freq2=atoi(str9);
            uint16_t integer; uint16_t fraction;
            uint16_t i;
            char str10[100];
            char str11[100];

            for(i=freq1;i<=freq2;)
            {
                freq=i;
                sprintf(str11,"\r\n Frequency : %u\r\n",i);
                putsUart0(str11);
                lookUpTable();
                float raw1=(float)(readAdc0Ss3()+1);
                float raw2=(float)(readAdc1Ss3()+1);
                gain=(raw1/raw2);
                integer = (uint16_t)(gain);
                fraction = (gain-integer)*1000;
                sprintf(str10,"\r\n Gain : %4u.%.3u\r\n",integer,fraction);
                putsUart0(str10);
                i=2*i;

    }
            if(2*i>freq2){i=freq2;}
                freq = freq2;
                sprintf(str11,"\r\n Frequency : %4lu\r\n",i);
                                putsUart0(str11);
                                lookUpTable();
                                float raw1=(float)(readAdc0Ss3()+1);
                                float raw2=(float)(readAdc1Ss3()+1);
                                gain=(raw1/raw2);
                                integer = (uint16_t)(gain);
                                fraction = (gain-integer)*1000;
                                sprintf(str10,"\r\n Gain : %4u.%.3u\r\n",integer,fraction);
                                putsUart0(str10);
                                waitMicrosecond(1000000);

            Trigger=0;
    }
   else if(strcmp(Cmd,"sqduty")==0)
   {
       putsUart0("Square with duty cycle");
       lookUpTable();
   }
   else if (strcmp(Cmd,"stop")==0)
   {
       Trigger=0;
   }

   else if(strcmp(Cmd,"reset")==0 )
   {
       NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
   }
   else
   {
       putsUart0("InValid Command");
   }

return;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)

{
    // Initialize hardware


    initHw();

    while(1)
    {

        //Trigger=0; //cycles=0;
   putsUart0("\r\n Please enter the string \r\n");

   getString(str, MAX_CHARS);

   putsUart0(str);
   putcUart0(0x0a);
   putcUart0(0x0d);
   parseString(str, pos, argCount,str1, str2, array);
  strcpy(Cmd,getArgString(0));
  freq=getArgFloat(4);
  amp=getArgFloat(3);
  channel=getArgInt(1);
  offset=getArgFloat(2);
  Trigger=0;
  isCommand();
  if( strcmp(Cmd,"ncycle")!=0 & strcmp(Cmd,"stop")!=0 )
  {
      if(strcmp(Cmd, "gain")!=0){run(); }
  }

   memset(str2,0,sizeof(str2));
   memset(str1,0,sizeof(str1));
   memset(str,0,sizeof(str));
   argCount=0;
    }
}

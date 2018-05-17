#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c_master_noint.h"
#include<stdio.h>
#include"st7735.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1// use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define ADDR 0b1101011

void initExp();
void i2cwrite(unsigned char reg, unsigned char val);
unsigned char i2cread();
void i2c_read_multiple(unsigned char add, unsigned char reg, unsigned char * data, int length);

void drawChar(unsigned short x, unsigned short y, unsigned char mess, unsigned short color1, unsigned short color2);
void drawString(unsigned short x, unsigned short y, unsigned char* message, unsigned short color1, unsigned short color2);
void drawProgressBar(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2);
void drawProgressBarLeft(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2);
void drawProgressBarUp(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2);
void drawProgressBarDown(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2);


int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    
    TRISAbits.TRISA4 = 0;

    __builtin_enable_interrupts();
    int sysclkfreq = 48000000;
    LCD_init();
   
    
    
    initExp();
    LCD_clearScreen(CYAN);
    if(i2cread()==0x69) {
    LCD_clearScreen(MAGENTA);
    
    while(1) {
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT()<=(sysclkfreq/(2*2*20)))
            {
                LATAbits.LATA4 = 1;
            }
            while(_CP0_GET_COUNT()>(sysclkfreq/(2*2*20)) && _CP0_GET_COUNT()<=(2*sysclkfreq/(2*2*20)))
            {
                LATAbits.LATA4 = 0;
            }
            short max = 32767;
            char message[26];
            unsigned char input[14];
            i2c_read_multiple(ADDR,0x20,input,14);
            signed short data1 = ((input[1]<<8)|input[0]);
            //sprintf(message,"%d",data1);
            //drawString(28,30,message,CYAN,MAGENTA);
            signed short data2 = ((input[3]<<8)|input[2]);
            //sprintf(message,"%d",data2);
            //drawString(28,40,message,CYAN,MAGENTA);
            signed short data3 = ((input[5]<<8)|input[4]);
            //sprintf(message,"%d",data3);
            //drawString(28,50,message,CYAN,MAGENTA);
            signed short data4 = ((input[7]<<8)|input[6]);
            //sprintf(message,"%d",data4);
            //drawString(28,60,message,CYAN,MAGENTA);
            signed short data5 = ((input[9]<<8)|input[8]);
            float data5value = 200.0*data5/32767.0;
            signed short output5 = data5value;
            //float data5percent = 100.0*data5/32767.0;
            //sprintf(message,"%f",data5percent);
            //drawString(28,70,message,CYAN,MAGENTA);
            signed short data6 = ((input[11]<<8)|input[10]);
            float data6value = 200.0*data6/32767.0;
            signed short output6 = data6value;
            //sprintf(message,"%f",data6percent);
            //drawString(28,80,message,CYAN,MAGENTA);
            signed short data7 = ((input[13]<<8)|input[12]);
            float data7value = 200.0*data7/32767.0;
            signed short output7 = data7value;
            //sprintf(message,"%f",data7percent);
            //drawString(28,90,message,CYAN,MAGENTA);
            if(output5>0)
            {
                unsigned short rightbar = output5;
                drawProgressBar(64,80,10,rightbar,WHITE,100,BLACK);
                drawProgressBarLeft(64,80,10,0,WHITE,100,BLACK);
            }
            else
            {
                unsigned short leftbar = -1*output5;
                drawProgressBarLeft(64,80,10,leftbar,WHITE,100,BLACK);
                drawProgressBar(64,80,10,0,WHITE,100,BLACK);
            }
            if(output7>0)
            {
                unsigned short upbar = output7;
                drawProgressBarUp(64,80,10,upbar,WHITE,100,BLACK);
                drawProgressBarDown(64,80,10,0,WHITE,100,BLACK);
            }
            else
            {
                unsigned short downbar = -1*output7;
                drawProgressBarDown(64,80,10,downbar,WHITE,100,BLACK);
                drawProgressBarUp(64,80,10,0,WHITE,100,BLACK);
            }
            
            
    }
}
    else{
       LCD_clearScreen(GREEN);
    }
}

void initExp(){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    i2c_master_setup();
    
    i2cwrite(0x10,0b10000010); // CTRL1_XL: 1.66kHz, 2g sensitivity, 100Hz filter
    i2cwrite(0x11,0b10001000); // CTRL2_G: 1.66 kHz, 1000 dps
    i2cwrite(0x12,0b00000100); // CTRL3_C: IF_INC Enabled
}



void i2cwrite(unsigned char reg, unsigned char val){
    i2c_master_start(); // make the start bit

i2c_master_send(ADDR<<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing

i2c_master_send(reg); // the register to write to

i2c_master_send(val); // the value to put in the register

i2c_master_stop(); // make the stop bit
}

unsigned char i2cread(){
    i2c_master_start(); // make the start bit

i2c_master_send(ADDR<<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing

i2c_master_send(0x0F); // reads from whoami IMU register

i2c_master_restart(); // make the restart bit

i2c_master_send(ADDR<<1|1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading

unsigned char r = i2c_master_recv(); // save the value returned

i2c_master_ack(1); // make the ack so the slave knows we got it

i2c_master_stop(); // make the stop bit

return r;
}

void i2c_read_multiple(unsigned char add, unsigned char reg, unsigned char * data, int length){
i2c_master_start(); // make the start bit

i2c_master_send(ADDR<<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing

i2c_master_send(reg); // reads from whoami IMU register

i2c_master_restart(); // make the restart bit

i2c_master_send(ADDR<<1|1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading

int k = 0;
for (k = 0; k<length; k++)
{
    data[k] = i2c_master_recv(); // save the value returned

    if(k==length-1)
    {
        i2c_master_ack(1); // Final ack
    }
    else
    {
        i2c_master_ack(0); //interim ack
    }
}
i2c_master_stop(); // make the stop bit
}

int main2() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    
    TRISAbits.TRISA4 = 0;

    __builtin_enable_interrupts();
    int sysclkfreq = 48000000;

    LCD_init();
   
    LCD_clearScreen(MAGENTA);
    int q = 0;
    float fps = 0;
    while(1)
    {
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<(sysclkfreq/(2*10)))
        {//wait
        }
        
            q++;
            if(q>100)
            {
                q=q-100;
            }
            char message[26];
            sprintf(message,"Hello World %d!   ",q);
            drawString(28,32,message,CYAN,MAGENTA);
            drawProgressBar(14,80,10,q,WHITE,100,BLACK);
            fps = (sysclkfreq/2.0)/_CP0_GET_COUNT();
            sprintf(message,"FPS: %f",fps);
            drawString(60,120,message,CYAN,MAGENTA);
        
    }
    
}

void drawString(unsigned short x, unsigned short y, unsigned char* message, unsigned short color1, unsigned short color2)
{

    int i = 0;
    while(message[i])
    {
        drawChar((x+5*i),y,message[i],color1,color2);
        i++;
    }
}

void drawChar(unsigned short x, unsigned short y, unsigned char mess, unsigned short color1, unsigned short color2)
{
    char row = mess - 0x20;
    int col = 0;
    for (col = 0;col<5;col++)
    {
        char pixels = ASCII[row][col];
        int j = 0;
        for (j=0;j<8;j++)
        {
            if((pixels>>j)&1==1)
            {
                LCD_drawPixel(x+col,y+j,color1);
            }
            else
            {
                LCD_drawPixel(x+col,y+j,color2);
            }
        }
    }
}

void drawProgressBar(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2)
{
    int k = 0;
    int l = 0;
    for (k=0;k<length2;k++)
    {
        for (l=0;l<height;l++)
        {
            if(k<length1)
            {
                LCD_drawPixel(x+k,y+l,color1);
            }
            else
            {
                LCD_drawPixel(x+k,y+l,color2);
            }
        }
    }
}

void drawProgressBarLeft(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2)
{
    int k = 0;
    int l = 0;
    for (k=0;k<length2;k++)
    {
        for (l=0;l<height;l++)
        {
            if(k<length1)
            {
                LCD_drawPixel(x-k,y+l,color1);
            }
            else
            {
                LCD_drawPixel(x-k,y+l,color2);
            }
        }
    }
}

void drawProgressBarUp(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2)
{
    int k = 0;
    int l = 0;
    for (k=0;k<length2;k++)
    {
        for (l=0;l<height;l++)
        {
            if(k<length1)
            {
                LCD_drawPixel(x+l,y-k,color1);
            }
            else
            {
                LCD_drawPixel(x+l,y-k,color2);
            }
        }
    }
}

void drawProgressBarDown(unsigned short x, unsigned short y, unsigned short height,unsigned short length1,unsigned short color1,unsigned short length2,unsigned short color2)
{
    int k = 0;
    int l = 0;
    for (k=0;k<length2;k++)
    {
        for (l=0;l<height;l++)
        {
            if(k<length1)
            {
                LCD_drawPixel(x+l,y+k,color1);
            }
            else
            {
                LCD_drawPixel(x+l,y+k,color2);
            }
        }
    }
}
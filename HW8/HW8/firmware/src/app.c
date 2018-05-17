/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

#define ADDR 0b1101011
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

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

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

    TRISBbits.TRISB4 = 1;
    __builtin_enable_interrupts();
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            
            int sysclkfreq = 48000000;
    LCD_init();
            ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    i2c_master_setup();
    
    i2cwrite(0x10,0b10000010); // CTRL1_XL: 1.66kHz, 2g sensitivity, 100Hz filter
    i2cwrite(0x11,0b10001000); // CTRL2_G: 1.66 kHz, 1000 dps
    i2cwrite(0x12,0b00000100); // CTRL3_C: IF_INC Enabled
    
    if(i2cread()==0x69) {
    LCD_clearScreen(0xF81F);
    
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
                drawProgressBar(64,80,10,rightbar,0xFFFF,100,0x0000);
                drawProgressBarLeft(64,80,10,0,0xFFFF,100,0x0000);
            }
            else
            {
                unsigned short leftbar = -1*output5;
                drawProgressBarLeft(64,80,10,leftbar,0xFFFF,100,0x0000);
                drawProgressBar(64,80,10,0,0xFFFF,100,0x0000);
            }
            if(output7>0)
            {
                unsigned short upbar = output7;
                drawProgressBarUp(64,80,10,upbar,0xFFFF,100,0x0000);
                drawProgressBarDown(64,80,10,0,0xFFFF,100,0x0000);
            }
            else
            {
                unsigned short downbar = -1*output7;
                drawProgressBarDown(64,80,10,downbar,0xFFFF,100,0x0000);
                drawProgressBarUp(64,80,10,0,0xFFFF,100,0x0000);
            }
            
            
    }
}
    else{
       LCD_clearScreen(0x07E0);
    }
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

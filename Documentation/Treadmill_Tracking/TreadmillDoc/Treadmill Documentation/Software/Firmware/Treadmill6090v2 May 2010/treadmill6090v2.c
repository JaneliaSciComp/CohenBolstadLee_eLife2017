/*4-Axis Optical Motion Tracking System
Interface to a pair of the AVAGO ADNS-6090 Optical Mouse Camera Chips for Motion Tracking
       
Implemented in CodeVisionAVR IDE, v2.04.6 standard
treadmill6090v2.hex included for direct upload to atmel chip (so you don't have to purchase CodeVision unless you want to change the code)
Use Atmel's AVR Studio to upload the hex file to the ATMega644p
   
Version 1.1
May 5, 2010

(c) Gus K Lott III, PhD
                                                                                         
Neurobiological Instrumentation Engineer
HHMI - Janelia Farm Research Campus
19700 Helix Dr., Ashburn, VA 20147
lottg@janelia.hhmi.org

*/                    

//Defines for the ports on which the cameras connect
#include <mega324.h>
#define gMISO_0 PORTB.4
#define gdMISO_0 DDRB.4
#define gMISO_1 PORTB.3
#define gdMISO_1 DDRB.3
#define gSCLK PORTB.6
#define gdSCLK DDRB.6
#define gMOSI PORTB.5
#define gdMOSI DDRB.5
#define gRESET PORTB.1
#define gdRESET DDRB.1
#define gNCS PORTB.2
#define gdNCS DDRB.2

//Ports for Real-Time Trigger Signals
#define gCLK PORTB.0
#define gdCLK DDRB.0
#define gTRG PORTD.4
#define gdTRG DDRD.4
#define gaTRG PORTC.0
#define gdaTRG DDRC.0

#define RBUFFL 900


void initialize(void);
void puts_int(unsigned char newT);   
void pause50us(void);     
void pause10us(void);
void ADNS_write(unsigned char data);
void ADNS_read(void);
void EEPROMDump(void); 
void firmUpload(void);   
void writeEEPROM(void);
void commandExec(void);
void grabFrames(void);
void grabMotion(void);
void resetADNS(void);
void dumpRegisters(void);               
void setAnalogOut(void);

//ADNS and gTemp must be register       
unsigned char register tick, ADNS, gTemp; 
unsigned char register gTemp2, ADNS0, ADNS1, sstate=0;
unsigned char x[2],y[2],motion[2],squal[2],shut_low[2],shut_high[2],maxpix[2];
unsigned char r_char, serialFlag=0, t_char, t_buffer[RBUFFL], r_buffer[RBUFFL], vTarget=0, vidTime=0, vidFlag=0;
unsigned char Vf, Vs, Om, SampleCount=0, reportState=1;
unsigned int time_ms, serialTimeout=0, t_index=0, t_length=0, r_index=0;               
unsigned char binTime=0, binTarget=2, bin0x=128, bin0y=128, bin1x=128, bin1y=128, binFlag=0, binTemp=0, binScale=8;
unsigned char bin0xc=0, bin0yc=0, bin1xc=0, bin1yc=0; 








//Timer Interrupt dedicated to sample rate control
interrupt [TIM1_COMPA] void sample(void){
        gCLK=1;     //Raise sample clock (gCLK Port)           
        gTRG=1;     //Keep Trigger high during experiment
        #asm("sei")
        grabMotion();
}

//Time base control for periods greater than one milisecond (such as serial timeout, video frame period, and analog output binning
interrupt [TIM2_COMPA] void timebase(void)
{           
           tick++;              
           gaTRG=0;         
           if (tick==5)  //5 ticks per ms, update ms clocks
           {                    
                tick=0;
                time_ms++;
                vidTime++;
                binTime++;                         
                 
                if (serialTimeout>1) serialTimeout--;
                if (serialTimeout==1){
                        serialTimeout=0;
                        sstate=0;
                }                                                                   
                if (vidTime==vTarget&vTarget!=0){
                        vidTime=0;
                        vidFlag=1;
                }   
                if (binTime==binTarget&binTarget!=0){
                        binTime=0;
                        //latch in bin values
                        bin0xc=bin0x;
                        bin0x=128;
                        bin0yc=bin0y;
                        bin0y=128;
                        bin1xc=bin1x;
                        bin1x=128;
                        bin1yc=bin1y;
                        bin1y=128;
                        setAnalogOut();  
                        gaTRG=1;
                }
           }            
}

//Communications Interrupts - USART receive complete (data from user on PC)
interrupt [USART0_RXC] void serial_receive0(void) // Non-Blocking ISR Driven Read to an input command Buffer
{                        
        r_char=UDR0;
        
        switch(sstate){
                case 0:       
                        sstate=r_char;  
                        r_index=0;
                        break;        
                case 1: //High speed mode (7kHz) - No Shutter or SQual values
                        r_buffer[r_index]=r_char;
                        serialFlag=1;   
                        break;
                case 128: //reading in new EEPROM contents from user
                        r_buffer[r_index++]=r_char;
                        if (r_index==1986){    
                                #asm("sei")  
                                writeEEPROM();     
                                firmUpload();
                        }
                case 246: //Data return state.  0 = rotation coordinates, 1 = raw camera x/y      
                        reportState=r_char;
                        sstate=0;
                        break;       
                case 247: //Bin Time for Analog Output    
                        r_buffer[r_index++]=r_char;
                        if(r_index==2) serialFlag=1;
                        break;
                case 248:       //Write Arbitrary byte to ADNS
                        r_buffer[r_index++]=r_char;
                        if (r_index==2) serialFlag=1;          
                        break;
                case 249:       //Read Arbitrary byte from ADNS
                        r_buffer[r_index]=r_char;
                        serialFlag=1; 
                        break;         
                case 250: //Turn off video
                        r_buffer[r_index]=r_char;
                        serialFlag=1; 
                        break;
                case 251: //Read out a frame of pixels from the camera and send it to the user
                        r_buffer[r_index]=r_char;
                        serialFlag=1; 
                        break;
                case 252: //Dump Internal Registers
                        r_buffer[r_index]=r_char;
                        serialFlag=1;
                        break;             
                case 253: //Set Sample Rate of Motion Acquisition (2 byte sample period)
                        r_buffer[r_index++]=r_char;
                        if (r_index==2) serialFlag=1;
                        break;
                case 254: //Stop Data Acquisition
                        r_buffer[r_index]=r_char;
                        serialFlag=1; 
                        break;
                case 255: //Start Data Acquisition
                        r_buffer[r_index]=r_char;
                        serialFlag=1;
                        break;
        }
        //Turn on board LED
        serialTimeout=500;  //500ms Timeout on serial port commands so the system doesn't hang    

}                   
                                                
// ISR Driven Non-Blocking UART Write from FIFO Buffer of length RBUFFL  (Data register empty interrupt)
interrupt [USART0_DRE] void uart0_send(void)
{       
        
        if (t_index==t_length){  //End of Buffer
                UCSR0B=UCSR0B&0b11011111;  //Turn Off Send Loop if @ end of buffer        
        }
        else{ 
                //Keep streaming out buffer
                t_index++;         
                if (t_index==RBUFFL) t_index=0;
                UDR0=t_buffer[t_index];                        
        }
}                                          

//This function is called on chip reset and power up
void main(void){
        unsigned int i;
        
        initialize(); 
        
        //Reset Camera Chips        
        gNCS=0; //Open Communication Interface
        ADNS_write(0x0a|128);
        ADNS_write(0b00011100);   
        pause50us();
        i=0;
        ADNS_write(0x2C|128);
        ADNS_write(i);   
        pause50us();
        ADNS_write(0x2D|128);
        ADNS_write(~i);      
        pause50us();         
        ADNS_write(0x09|128);
        ADNS_write(0b00000111);      
        gNCS=1;  //Close Communication Interface         
        
            
        //The "Operating System."
        //This while loop spins while interrupts are handled 
        while(1){
                if (serialFlag==1) commandExec();
                if (vidFlag==1) grabFrames();  
        }
}                   

void setAnalogOut(void){    
        
        //Make sure chip select is asserted
        PORTD.5=0; //CS (active low)
        //Raise latch to block out changes
        PORTC.3=1; //LDAC (active low)        
                     
        //Address DAC A (Vx0)
        PORTD.7=0; //A0
        PORTC.2=0; //A1
        //Drop Write input for transition to latch
        PORTD.6=0; //WR (active low)
        //update Vx0             
        PORTA = bin0xc;
        //latch value into register
        PORTD.6=1; //Raise WR
        
        //Address DAC B (Vy0)
        PORTD.7=1; //A0
        PORTC.2=0; //A1
        //Drop Write input for transition to latch
        PORTD.6=0; //WR (active low)
        //update Vy0
        PORTA=bin0yc;
        //latch value into register
        PORTD.6=1; //Raise WR
        
        //Address DAC C (Vy1)
        PORTD.7=0; //A0
        PORTC.2=1; //A1
        //Drop Write input for transition to latch
        PORTD.6=0; //WR (active low)
        //update Vy0   
        PORTA=bin1yc;
        //latch value into register
        PORTD.6=1; //Raise WR
        
        //Address DAC D (Vx1)
        PORTD.7=1; //A0
        PORTC.2=1; //A1
        //Drop Write input for transition to latch
        PORTD.6=0; //WR (active low)
        //update Vy0
        PORTA=bin1xc;
        //latch value into register
        PORTD.6=1; //Raise WR
        
        //Drop latch to Update all DAC registers
        PORTC.3=0; //LDAC (active low)
        //deassert chip select
        PORTD.5=1; //CS (active low)
        
}               
             
void grabMotion(void){
        
        gNCS=0;    
        
        ADNS_write(0x50);
        TCNT0=0;
        while(TCNT0<24){};  //~75us allowing for events to be handled   
        ADNS_read();    
        motion[0]=ADNS0;
        motion[1]=ADNS1;
        ADNS_read();
        x[0]=ADNS0;
        x[1]=ADNS1;
        ADNS_read();
        y[0]=ADNS0;
        y[1]=ADNS1;
        ADNS_read();  //Can stop here to increase sample speed
        squal[0]=ADNS0;
        squal[1]=ADNS1;
        ADNS_read();
        shut_high[0]=ADNS0;
        shut_high[1]=ADNS1;
        ADNS_read();
        shut_low[0]=ADNS0;     
        shut_low[1]=ADNS1;      
        gNCS=1;
        
        //Calculate Fly Based Coordinates (or not) 
        switch(reportState){
        case 0:
                Vf=(unsigned char)((signed char)y[0]+(signed char)y[1])+128;
                Vs=(unsigned char)((signed char)y[0]-(signed char)y[1])+128;
                Om=(unsigned char)((signed char)x[0]+(signed char)x[1])+128;
                x[0]=Vf;
                y[0]=Vs;
                x[1]=Om;
                y[1]=128;                 
                break;
        case 1:    
                x[0]+=128;
                y[0]+=128;
                x[1]+=128;
                y[1]+=128;    
                break;       
        }
        
             
        //Velocity Bins
        if (x[0]>128) bin0x=bin0x+((x[0]-128)<<binScale);  
        if (x[0]<128) bin0x=bin0x-((128-x[0])<<binScale);  
        if (y[0]>128) bin0y=bin0y+((y[0]-128)<<binScale);
        if (y[0]<128) bin0y=bin0y-((128-y[0])<<binScale); 
        if (x[1]>128) bin1x=bin1x+((x[1]-128)<<binScale);
        if (x[1]<128) bin1x=bin1x-((128-x[1])<<binScale); 
        if (y[1]>128) bin1y=bin1y+((y[1]-128)<<binScale);
        if (y[1]<128) bin1y=bin1y-((128-y[1])<<binScale);
                      
        //Clean Packet Structure so that only byte 0 has a value of 0 in any case                        
        if (SampleCount==0) SampleCount=1;
        shut_high[0]+=1;
        shut_high[1]+=1;
        if (shut_low[0]==0) shut_low[0]=1;
        if (shut_low[1]==0) shut_low[1]=1;
        squal[0]+=1;
        squal[1]+=1;
        
        //Output Data to Serial Port Buffer
        t_length=0;                       
        t_buffer[t_length++]=0; 
        t_buffer[t_length++]=SampleCount++;
        t_buffer[t_length++]=x[0];
        t_buffer[t_length++]=y[0];
        t_buffer[t_length++]=x[1];
        t_buffer[t_length++]=y[1];
        
        t_buffer[t_length++]=squal[0];
        t_buffer[t_length++]=squal[1];
        t_buffer[t_length++]=shut_high[0];
        t_buffer[t_length++]=shut_low[0];
        t_buffer[t_length++]=shut_high[1];
        t_buffer[t_length++]=shut_low[1];
        
        //Initialize interrupt driven serial port output (takes about 75us in parallel with this loop to complete)
        t_index=0;
        t_length=11;
        UDR0 =t_buffer[t_index];               
        UCSR0B=UCSR0B|0b00100000;                            
        
        gCLK=0;  //Lower sample clock (gCLK port)  
        
}        

void commandExec(void){                         
        TCCR1B=0b00001000;  //Turn off any motion tracking
        vTarget=0;      //Turn off any video acquisition
        vidFlag=0;
        gTRG=0;
        
        switch (sstate){
                case 247:       //Set AO bin speed/scale
                        binTarget=r_buffer[1];
                        binScale=r_buffer[2];
                        break;      
                case 248:       //Write Arbitrary byte to ADNS
                        gNCS=0;
                        ADNS_write(r_buffer[1]|0x80);
                        pause50us(); 
                        ADNS_write(r_buffer[2]);
                        gNCS=1;
                        break;
                case 249:       //Read Arbitrary byte from ADNS
                        gNCS=0;
                        ADNS_write(r_buffer[1]);
                        pause50us();
                        ADNS_read();
                        gNCS=1;  
                        puts_int(ADNS0);
                        break;
                case 250:
                        vTarget=0;        //End Video, Reset Chip
                        resetADNS();                
                        break;
                case 251://Handle Video from the camera
                        vTarget=50;
                        break;
                case 252:  //Return status information from cameras 
                        dumpRegisters();
                        break; 
                case 253:       //Set Sample Period
                        OCR1AH=r_buffer[1];
                        OCR1AL=r_buffer[2];
                        break;   
                case 254: //Stop Motion Acquisition & video
                        vTarget = 0;
                        TCCR1B=0b00001000;    
                        while(t_index!=t_length){}      
                        #asm("rjmp 0")
                        break;
                case 255: //Start Motion Acquisition
                        gNCS=0;
                        /*ADNS_write(0x0b|128);  //Configure for fixed frame rate
                        ADNS_write(0b10001000);
                        pause50us();
                        ADNS_write(0x19|128);
                        ADNS_write(0x05);
                        pause50us();
                        ADNS_write(0x1a|128);
                        ADNS_write(0x0D);    */                       
                                                
                        pause50us();           
                        ADNS_write(0x12|128);  //Motion Clear Register
                        ADNS_write(0xaa);      
                        pause50us();           
                        
                        gNCS=1;
                        TCNT1H=0;
                        TCNT1L=0;
                        TCCR1B=0b00001010;
                        break;
                
        }     
        //Turn off board LED
        sstate=0;
        serialFlag=0;
}                                       


void grabFrames(void){
        unsigned int i;   
        PORTB.0=1;
        TCCR1B=0b00001000;  //Stop any Motion Acquisition
        
        gNCS=0;     
        ADNS_write(0x0a|128);
        ADNS_write(0b00011100);
        TCNT0=0;
        while(TCNT0<16){};  //~50us allowing for events to be handled 
        ADNS_write(0x13|128);
        ADNS_write(0x83);
        gNCS=1;       

        time_ms=0;
        while(time_ms<3){} //Wait 10us + 3 frame periods, 3ms should cover that.  Can be less

        gNCS=0;
        ADNS_write(0x40);
        TCNT0=0;
        while(TCNT0<16){};  //~50us allowing for events to be handled
                 
        for (i=0; i<RBUFFL; i++){
                ADNS_read();
                puts_int(ADNS0);
                puts_int(ADNS1);
                TCNT0=0;
                while(TCNT0<4){};  //~10us allowing for events to be handled
        }               

        //Send SQUAL        
        //ADNS_write(0x05);
        //pause50us();
        //ADNS_read();
        //puts_int(ADNS0);
        //puts_int(ADNS1);
             
        gNCS=1;
                          
        vidFlag=0;
        PORTB.0=0;
}                     

//Write the contents of r_buffer to the EEPROM for uploading the local firmware if Avago upgrades
void writeEEPROM(void){
        unsigned int i;
         
        for (i=0; i<1986; i++){ 
                PINB.0=1;
                while(EECR.1==1){};
                while(SPMCSR&0x01==1){};
                EEARH=i>>8;
                EEARL=i;                
                EEDR=r_buffer[i];                       
                
                EECR=0b00000100;   
                EECR=0b00000110;
        }
}

//Upload EEPROM Contents into ADNS Chip                       
void firmUpload(void){
        unsigned int i, address;
        
        gNCS=0;
        //Write 0x1d to register 0x14 (SROM_enable register)
        ADNS_write(0x14|128);
        ADNS_write(0x1D);
        gNCS=1;
        //Wait at least 1 frame period
        time_ms=0;
        while(time_ms<5){};                                 
        gNCS=0;
        //Write 0x18 to register 0x14 (SROM_enable register)
        ADNS_write(0x14|128);
        ADNS_write(0x18);
        gNCS=1;
        pause50us();
        gNCS=0;
        
        //Begin burst mode writing
        ADNS_write(0x60|128);                                                      
        
        //EEPROM Address register to 0     

        while(EECR.1==1) {PINB.0=1;}
        address=0;
        EEARH=address>>8;
        EEARL=address;
        
        for (i=0; i<1986; i++){
                pause10us();
                
                while(EECR.1==1){};
                
                //Execute Read Command w/ increment
                EECR=0b00000001;
                address++;
                EEARH=address>>8;
                EEARL=address;
                                
                ADNS=EEDR;       
                ADNS_write(ADNS);
                pause10us();
        }
                                               
        gNCS=1;   
        pause50us();
        pause50us();
        pause50us();
}

//Reset Chip, takes 200ms                    
void resetADNS(void){
        //Toggle Reset
        gRESET=1;
        pause50us();
        gRESET=0;    
        
        //wait for inputs & motion data to be valid    
        time_ms=0;          
        while(time_ms<200){};  
        
}

//DUMP Status Values from Each Camera to the User (currently only camera 0)
void dumpRegisters(void){
        unsigned char regLoc[25]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x09,0x0a,0x0b,0x0e,0x0f,0x10,0x11,0x16,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x2c,0x2d,0x3d};
        unsigned char i;
        
        gNCS=0;    
        
        for (i=0; i<25; i++){
                ADNS_write(regLoc[i]); 
                TCNT0=0;
                while(TCNT0<16){};  //~50us allowing for events to be handled
                ADNS_read();
                puts_int(ADNS0);
                TCNT0=0;
                while(TCNT0<16){};  //~50us allowing for events to be handled         
        }
        

        gNCS=1;                        
}

//Dump EEPROM for firmware verification
void EEPROMDump(void){  
        unsigned int i;
        
        for (i=0; i<1986; i++){
                while(EECR.1==1){};
                //Execute Read Command w/ increment
                EEARH=i>>8;
                EEARL=i;
                EECR=0b00000001;
                puts_int(EEDR);
                pause50us();
        }
}
                     
//Pause for communication protocol
void pause50us(void){
        unsigned char i=0;
        while(i<250){
                i++;
        }
}      
//Pause for communication protocol
void pause10us(void){
        unsigned char i=0;
        while(i<40){
                i++;
        }
}
     
//Initiate an interrupt driven UART output of a buffer's contents 
//or add a character to the FIFO buffer for transmit
void puts_int(unsigned char newT)
{                    
        UCSR0B=UCSR0B&0b11011111;  //Turn off DRE interrupt so that an interrupt doesn't happend during this fast buffer access and cause index/length mismatch
        t_length++;
        if (t_length==RBUFFL) t_length=0;
        t_buffer[t_length]=newT;
                                     
        if (((UCSR0A&0b00100000)==0b00100000))
        {       //if not outputting, start subsystem
                t_index++;
                if (t_index==RBUFFL) t_index=0;
                UDR0=t_buffer[t_index];
                UCSR0B=UCSR0B|0b00100000; 
        }                                                                                                                                                
        UCSR0B=UCSR0B|0b00100000;  //Re-enable DRE interrupt
}


      

void initialize(void){
        
                           
        //Timer1 Setup for Sample Period, 16-bits for more timer resolution
        TCCR1A=0b00000000;
        TCCR1B=0b00001000;
        TCCR1C=0b00000000;
        TCNT1H=0;
        TCNT1L=0;
        OCR1AH=2;   //4kHz
        OCR1AL=112;
        //OCR1AH=1;   //5kHz
        //OCR1AL=243;

        TIMSK1=0b00000010;
        
        //Timer2 Setup for 1ms timebase (this counts 0.2 ms exactly), 5 ticks = 1ms
        TCCR2A=0b00000010;
        TCCR2B=0b00000011;
        TCNT2=0b00000000;
        OCR2A=125;
        TIMSK2=0b00000010;  
        
        //Timer0 setup for delay timing
        TCCR0A=0b00000000;
        TIMSK0=0b00000000;
        TCCR0B=0b00000011;
        TCNT0=0;   
                      
        //Setup UART0 for send/receive w/ user
        UBRR0H=0x00;
        UBRR0L=1;  //1=1.25M, 4=0.5M, 21 = 115.2k, 42 = 57.6k
        UCSR0A=0b00000010;
        UCSR0B=0b10011000;
        UCSR0C=0b00000110;
                          
        //Setup UART1 for comms with Other devices
        //UBRR1H=0x00;
        //UBRR1L=42;  //1=1.25M, 4=0.5M, 21 = 115.2k, 42 = 57.6k
        //UCSR1A=0b00000010;
        //UCSR1B=0b10011000;
        //UCSR1C=0b00000110;

        //Activate interrupts
        #asm("sei") 
                                                    
        //Raise Global NCS to reset ADNS comm interface
        gdNCS=1;         
        gNCS=1;                                     
        //Setup Read Ports for input
        gdMISO_0=0;  //Camera 0 (MISO)
        gdMISO_1=0;  //Camera 1 (MISO)
        //Setup Commoun Output ports for Writing to Cameras
        gdSCLK=1;
        gSCLK=0;  //SCLK
        gdMOSI=1;
        gMOSI=0;  //MOSI
        //Lower Active High Reset Pin
        gdRESET=1;
        gRESET=0;     
        
        gdCLK=1;
        gCLK=0;
        gdTRG=1;
        gTRG=0;       
        gdaTRG=1;
        gaTRG=0;
        
        //DAC Communications
        DDRA=0xff;
        DDRD=0xff;
        DDRC=0xff;
        PORTD.5=1; //CS
        
        //Reset ADNS6090 Camera Chips              
        time_ms=0;
        while (time_ms==0){}
        gRESET=1; //Raise Reset pin
        time_ms=0;
        while(time_ms<200){}
        gRESET=0; //Drop Reset Pin
        time_ms=0;
        while(time_ms<180){}  //Motion Data is Valid 180ms after reset drops      
        
}                             


//Tight Assembly Code loop for bit banging protocol @1MHz to WRITE a byte to the bus
//byte to write is stored in ADNS (r4) but is passed as an input argument to the function
void ADNS_write(unsigned char data){          
        ADNS=data;
        #asm       
                mov _gTemp,_ADNS
                clr r22
        gWriteLoop:                   
                cbi $05,6       ;PortB = $05, Drop clock
                nop
                nop    
                nop                               
                nop
                nop
                nop
                    
                sbrs _gTemp,7       ;Conditional statement to clock out bit 7
                cbi $05,5        
                sbrc _gTemp,7
                sbi $05,5
                
                sbi $05,6       ;Raise Clock
                nop
                nop
                nop
                nop      
                nop
                nop
                lsl _gTemp          ;Logial Shift register left to place next bit in 7th position
                inc r22
                cpi r22,8
                breq gWriteOut                
                rjmp gWriteLoop
        gWriteOut:

        #endasm       
        
}

//Tight Assembly code 1MHz bit banging protocol to READ a pair of parallel bytes into the ADNS0 and ADNS1  registers
void ADNS_read(void){
        #asm   
                ;r22 is free for arithmatic  (according to codevision documentation)
                clr _ADNS0          ;clear ADNS data register
                clr _ADNS1
                ldi r22,0x01                             
                mov _gTemp2,r22      ;gTemp2 is mask
                clr r22         ;Increment through loop
                                  
        gReadLoop:
                lsl _ADNS0          ;Shift ADNS0 to left by 1
                lsl _ADNS1          ;Shift ADNS1 to left by 1  
                cbi $05,6       ;drop clock to clock out bit from ADNS chip
                nop
                inc r22
                nop         
                nop
                nop
                clr _gTemp 
                sbi $05,6       ;raise clock
                nop
                nop
                nop
                in _gTemp,$03       ;read in the pin values on port b                             
                                                               
                sbrc _gTemp,4
                or _ADNS0,_gTemp2        ;if bit 4 is set, MISO0 is high, set LSB of ADNS0
                sbrc _gTemp,3                                      
                or _ADNS1,_gTemp2        ;if bit 3 is set, MISO1 is high, set LSB of ADNS1
                
                        
                cpi r22,8
                breq gReadOut
                rjmp gReadLoop            
        gReadOut: 
        #endasm                    
                                                                                              
}


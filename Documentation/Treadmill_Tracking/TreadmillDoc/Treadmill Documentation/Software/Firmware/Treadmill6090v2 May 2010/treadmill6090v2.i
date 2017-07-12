
#pragma used+
sfrb PINA=0;
sfrb DDRA=1;
sfrb PORTA=2;
sfrb PINB=3;
sfrb DDRB=4;
sfrb PORTB=5;
sfrb PINC=6;
sfrb DDRC=7;
sfrb PORTC=8;
sfrb PIND=9;
sfrb DDRD=0xa;
sfrb PORTD=0xb;
sfrb TIFR0=0x15;
sfrb TIFR1=0x16;
sfrb TIFR2=0x17;
sfrb PCIFR=0x1b;
sfrb EIFR=0x1c;
sfrb EIMSK=0x1d;
sfrb GPIOR0=0x1e;
sfrb EECR=0x1f;
sfrb EEDR=0x20;
sfrb EEARL=0x21;
sfrb EEARH=0x22;
sfrw EEAR=0X21;   
sfrb GTCCR=0x23;
sfrb TCCR0A=0x24;
sfrb TCCR0B=0x25;
sfrb TCNT0=0x26;
sfrb OCR0A=0x27;
sfrb OCR0B=0x28;
sfrb GPIOR1=0x2a;
sfrb GPIOR2=0x2b;
sfrb SPCR=0x2c;
sfrb SPSR=0x2d;
sfrb SPDR=0x2e;
sfrb ACSR=0x30;
sfrb OCDR=0x31;
sfrb SMCR=0x33;
sfrb MCUSR=0x34;
sfrb MCUCR=0x35;
sfrb SPMCSR=0x37;
sfrb SPL=0x3d;
sfrb SPH=0x3e;
sfrb SREG=0x3f;
#pragma used-

#asm
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x01
	.EQU __sm_mask=0x0E
	.EQU __sm_powerdown=0x04
	.EQU __sm_powersave=0x06
	.EQU __sm_standby=0x0C
	.EQU __sm_ext_standby=0x0E
	.EQU __sm_adc_noise_red=0x02
	.SET power_ctrl_reg=smcr
	#endif
#endasm

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

unsigned char register tick, ADNS, gTemp; 
unsigned char register gTemp2, ADNS0, ADNS1, sstate=0;
unsigned char x[2],y[2],motion[2],squal[2],shut_low[2],shut_high[2],maxpix[2];
unsigned char r_char, serialFlag=0, t_char, t_buffer[900], r_buffer[900], vTarget=0, vidTime=0, vidFlag=0;
unsigned char Vf, Vs, Om, SampleCount=0, reportState=1;
unsigned int time_ms, serialTimeout=0, t_index=0, t_length=0, r_index=0;               
unsigned char binTime=0, binTarget=2, bin0x=128, bin0y=128, bin1x=128, bin1y=128, binFlag=0, binTemp=0, binScale=8;
unsigned char bin0xc=0, bin0yc=0, bin1xc=0, bin1yc=0; 

interrupt [14] void sample(void){
PORTB.0=1;     
PORTD.4=1;     
#asm("sei")
grabMotion();
}

interrupt [10] void timebase(void)
{           
tick++;              
PORTC.0=0;         
if (tick==5)  
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

bin0xc=bin0x;
bin0x=128;
bin0yc=bin0y;
bin0y=128;
bin1xc=bin1x;
bin1x=128;
bin1yc=bin1y;
bin1y=128;
setAnalogOut();  
PORTC.0=1;
}
}            
}

interrupt [21] void serial_receive0(void) 
{                        
r_char=(*(unsigned char *) 0xc6);

switch(sstate){
case 0:       
sstate=r_char;  
r_index=0;
break;        
case 1: 
r_buffer[r_index]=r_char;
serialFlag=1;   
break;
case 128: 
r_buffer[r_index++]=r_char;
if (r_index==1986){    
#asm("sei")  
writeEEPROM();     
firmUpload();
}
case 246: 
reportState=r_char;
sstate=0;
break;       
case 247: 
r_buffer[r_index++]=r_char;
if(r_index==2) serialFlag=1;
break;
case 248:       
r_buffer[r_index++]=r_char;
if (r_index==2) serialFlag=1;          
break;
case 249:       
r_buffer[r_index]=r_char;
serialFlag=1; 
break;         
case 250: 
r_buffer[r_index]=r_char;
serialFlag=1; 
break;
case 251: 
r_buffer[r_index]=r_char;
serialFlag=1; 
break;
case 252: 
r_buffer[r_index]=r_char;
serialFlag=1;
break;             
case 253: 
r_buffer[r_index++]=r_char;
if (r_index==2) serialFlag=1;
break;
case 254: 
r_buffer[r_index]=r_char;
serialFlag=1; 
break;
case 255: 
r_buffer[r_index]=r_char;
serialFlag=1;
break;
}

serialTimeout=500;  

}                   

interrupt [22] void uart0_send(void)
{       

if (t_index==t_length){  
(*(unsigned char *) 0xc1)=(*(unsigned char *) 0xc1)&0b11011111;  
}
else{ 

t_index++;         
if (t_index==900) t_index=0;
(*(unsigned char *) 0xc6)=t_buffer[t_index];                        
}
}                                          

void main(void){
unsigned int i;

initialize(); 

PORTB.2=0; 
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
PORTB.2=1;  

while(1){
if (serialFlag==1) commandExec();
if (vidFlag==1) grabFrames();  
}
}                   

void setAnalogOut(void){    

PORTD.5=0; 

PORTC.3=1; 

PORTD.7=0; 
PORTC.2=0; 

PORTD.6=0; 

PORTA = bin0xc;

PORTD.6=1; 

PORTD.7=1; 
PORTC.2=0; 

PORTD.6=0; 

PORTA=bin0yc;

PORTD.6=1; 

PORTD.7=0; 
PORTC.2=1; 

PORTD.6=0; 

PORTA=bin1yc;

PORTD.6=1; 

PORTD.7=1; 
PORTC.2=1; 

PORTD.6=0; 

PORTA=bin1xc;

PORTD.6=1; 

PORTC.3=0; 

PORTD.5=1; 

}               

void grabMotion(void){

PORTB.2=0;    

ADNS_write(0x50);
TCNT0=0;
while(TCNT0<24){};  
ADNS_read();    
motion[0]=ADNS0;
motion[1]=ADNS1;
ADNS_read();
x[0]=ADNS0;
x[1]=ADNS1;
ADNS_read();
y[0]=ADNS0;
y[1]=ADNS1;
ADNS_read();  
squal[0]=ADNS0;
squal[1]=ADNS1;
ADNS_read();
shut_high[0]=ADNS0;
shut_high[1]=ADNS1;
ADNS_read();
shut_low[0]=ADNS0;     
shut_low[1]=ADNS1;      
PORTB.2=1;

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

if (x[0]>128) bin0x=bin0x+((x[0]-128)<<binScale);  
if (x[0]<128) bin0x=bin0x-((128-x[0])<<binScale);  
if (y[0]>128) bin0y=bin0y+((y[0]-128)<<binScale);
if (y[0]<128) bin0y=bin0y-((128-y[0])<<binScale); 
if (x[1]>128) bin1x=bin1x+((x[1]-128)<<binScale);
if (x[1]<128) bin1x=bin1x-((128-x[1])<<binScale); 
if (y[1]>128) bin1y=bin1y+((y[1]-128)<<binScale);
if (y[1]<128) bin1y=bin1y-((128-y[1])<<binScale);

if (SampleCount==0) SampleCount=1;
shut_high[0]+=1;
shut_high[1]+=1;
if (shut_low[0]==0) shut_low[0]=1;
if (shut_low[1]==0) shut_low[1]=1;
squal[0]+=1;
squal[1]+=1;

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

t_index=0;
t_length=11;
(*(unsigned char *) 0xc6) =t_buffer[t_index];               
(*(unsigned char *) 0xc1)=(*(unsigned char *) 0xc1)|0b00100000;                            

PORTB.0=0;  

}        

void commandExec(void){                         
(*(unsigned char *) 0x81)=0b00001000;  
vTarget=0;      
vidFlag=0;
PORTD.4=0;

switch (sstate){
case 247:       
binTarget=r_buffer[1];
binScale=r_buffer[2];
break;      
case 248:       
PORTB.2=0;
ADNS_write(r_buffer[1]|0x80);
pause50us(); 
ADNS_write(r_buffer[2]);
PORTB.2=1;
break;
case 249:       
PORTB.2=0;
ADNS_write(r_buffer[1]);
pause50us();
ADNS_read();
PORTB.2=1;  
puts_int(ADNS0);
break;
case 250:
vTarget=0;        
resetADNS();                
break;
case 251:
vTarget=50;
break;
case 252:  
dumpRegisters();
break; 
case 253:       
(*(unsigned char *) 0x89)=r_buffer[1];
(*(unsigned char *) 0x88)=r_buffer[2];
break;   
case 254: 
vTarget = 0;
(*(unsigned char *) 0x81)=0b00001000;    
while(t_index!=t_length){}      
#asm("rjmp 0")
break;
case 255: 
PORTB.2=0;

pause50us();           
ADNS_write(0x12|128);  
ADNS_write(0xaa);      
pause50us();           

PORTB.2=1;
(*(unsigned char *) 0x85)=0;
(*(unsigned char *) 0x84)=0;
(*(unsigned char *) 0x81)=0b00001010;
break;

}     

sstate=0;
serialFlag=0;
}                                       

void grabFrames(void){
unsigned int i;   
PORTB.0=1;
(*(unsigned char *) 0x81)=0b00001000;  

PORTB.2=0;     
ADNS_write(0x0a|128);
ADNS_write(0b00011100);
TCNT0=0;
while(TCNT0<16){};  
ADNS_write(0x13|128);
ADNS_write(0x83);
PORTB.2=1;       

time_ms=0;
while(time_ms<3){} 

PORTB.2=0;
ADNS_write(0x40);
TCNT0=0;
while(TCNT0<16){};  

for (i=0; i<900; i++){
ADNS_read();
puts_int(ADNS0);
puts_int(ADNS1);
TCNT0=0;
while(TCNT0<4){};  
}               

PORTB.2=1;

vidFlag=0;
PORTB.0=0;
}                     

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

void firmUpload(void){
unsigned int i, address;

PORTB.2=0;

ADNS_write(0x14|128);
ADNS_write(0x1D);
PORTB.2=1;

time_ms=0;
while(time_ms<5){};                                 
PORTB.2=0;

ADNS_write(0x14|128);
ADNS_write(0x18);
PORTB.2=1;
pause50us();
PORTB.2=0;

ADNS_write(0x60|128);                                                      

while(EECR.1==1) {PINB.0=1;}
address=0;
EEARH=address>>8;
EEARL=address;

for (i=0; i<1986; i++){
pause10us();

while(EECR.1==1){};

EECR=0b00000001;
address++;
EEARH=address>>8;
EEARL=address;

ADNS=EEDR;       
ADNS_write(ADNS);
pause10us();
}

PORTB.2=1;   
pause50us();
pause50us();
pause50us();
}

void resetADNS(void){

PORTB.1=1;
pause50us();
PORTB.1=0;    

time_ms=0;          
while(time_ms<200){};  

}

void dumpRegisters(void){
unsigned char regLoc[25]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x09,0x0a,0x0b,0x0e,0x0f,0x10,0x11,0x16,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x2c,0x2d,0x3d};
unsigned char i;

PORTB.2=0;    

for (i=0; i<25; i++){
ADNS_write(regLoc[i]); 
TCNT0=0;
while(TCNT0<16){};  
ADNS_read();
puts_int(ADNS0);
TCNT0=0;
while(TCNT0<16){};  
}

PORTB.2=1;                        
}

void EEPROMDump(void){  
unsigned int i;

for (i=0; i<1986; i++){
while(EECR.1==1){};

EEARH=i>>8;
EEARL=i;
EECR=0b00000001;
puts_int(EEDR);
pause50us();
}
}

void pause50us(void){
unsigned char i=0;
while(i<250){
i++;
}
}      

void pause10us(void){
unsigned char i=0;
while(i<40){
i++;
}
}

void puts_int(unsigned char newT)
{                    
(*(unsigned char *) 0xc1)=(*(unsigned char *) 0xc1)&0b11011111;  
t_length++;
if (t_length==900) t_length=0;
t_buffer[t_length]=newT;

if ((((*(unsigned char *) 0xc0)&0b00100000)==0b00100000))
{       
t_index++;
if (t_index==900) t_index=0;
(*(unsigned char *) 0xc6)=t_buffer[t_index];
(*(unsigned char *) 0xc1)=(*(unsigned char *) 0xc1)|0b00100000; 
}                                                                                                                                                
(*(unsigned char *) 0xc1)=(*(unsigned char *) 0xc1)|0b00100000;  
}

void initialize(void){

(*(unsigned char *) 0x80)=0b00000000;
(*(unsigned char *) 0x81)=0b00001000;
(*(unsigned char *) 0x82)=0b00000000;
(*(unsigned char *) 0x85)=0;
(*(unsigned char *) 0x84)=0;
(*(unsigned char *) 0x89)=2;   
(*(unsigned char *) 0x88)=112;

(*(unsigned char *) 0x6f)=0b00000010;

(*(unsigned char *) 0xb0)=0b00000010;
(*(unsigned char *) 0xb1)=0b00000011;
(*(unsigned char *) 0xb2)=0b00000000;
(*(unsigned char *) 0xb3)=125;
(*(unsigned char *) 0x70)=0b00000010;  

TCCR0A=0b00000000;
(*(unsigned char *) 0x6e)=0b00000000;
TCCR0B=0b00000011;
TCNT0=0;   

(*(unsigned char *) 0xc5)=0x00;
(*(unsigned char *) 0xc4)=1;  
(*(unsigned char *) 0xc0)=0b00000010;
(*(unsigned char *) 0xc1)=0b10011000;
(*(unsigned char *) 0xc2)=0b00000110;

#asm("sei") 

DDRB.2=1;         
PORTB.2=1;                                     

DDRB.4=0;  
DDRB.3=0;  

DDRB.6=1;
PORTB.6=0;  
DDRB.5=1;
PORTB.5=0;  

DDRB.1=1;
PORTB.1=0;     

DDRB.0=1;
PORTB.0=0;
DDRD.4=1;
PORTD.4=0;       
DDRC.0=1;
PORTC.0=0;

DDRA=0xff;
DDRD=0xff;
DDRC=0xff;
PORTD.5=1; 

time_ms=0;
while (time_ms==0){}
PORTB.1=1; 
time_ms=0;
while(time_ms<200){}
PORTB.1=0; 
time_ms=0;
while(time_ms<180){}  

}                             

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


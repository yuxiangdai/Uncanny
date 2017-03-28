
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "configBits.h"
#include "constants.h"
#include "lcd.h"
#include "I2C.h"
#include "macros.h"
#include "functions.h"
#include "main.h"
//
//
// servo macros and duty cycles
#define SERVO_UPSCALE 1.25
#define SERVO_UPTIME 0.25
#define SERVO_DOWNTIME 20
#define AA 0
#define C 0.33
#define NINEV 0.67
#define DEAD 0.99
#define SERVO_UP 1
#define SERVO_DOWN 0


void FINALoperation(void);
void LightSensorRA5_Pop(void);
void LightSensorRA4_Soup(void);
//void stepper_ISR(void);
//void stepper_delay(void);

void solenoid_push(void);

void stepper_rotate(void);

void stepper_delay(void);

void DC_motor_RA(int pin);

void servo_rotate_RC0(int degree);
void servo_rotate_RC1(int degree);

void update_servo_position(float);

void set_external_interrupt0(float time);
void set_external_interrupt1(float time);
void set_external_interrupt3(float time);
void servo_ISR(void);

void set_time(void);
int dec_to_hex(int num);
void date_time(void);
void read_time(void);
void can_count(void);
void can_time(void);
void standby(void);
void operation(void);
void operationend(void);
void emergencystop(void);
void servo_rotate0(int degree);
void servo_rotate1(int degree);
void servo_rotate2(int degree);
void keyPressed(void);

void readADC(char channel);
void ADC_RA3(void);


int run_number;
void display_log(void);
void read_sensor(void);

const char keys[] = "123A456B789C*0#D";
const char currentTime[7] = {   0x50, //Seconds 
                            0x56, //Minutes
                            0x07, //Hour, 24 hour mode
                            0x03, //Day of the week, Monday = 1
                            0x01, //Day/Date
                            0x03, //Month
                            0x17};//Year, last two digits

enum state {
        STANDBY,
        EMERGENCYSTOP,
        OPERATION,
        OPERATIONEND,
        CANCOUNT,
        CANTIME,
        LOGS,
        TEST
    };
    
enum state curr_state;

volatile unsigned char servo_current_state;
volatile float servo_up_period;
volatile float servo_down_period;

void DCforwardPWM(void);

unsigned char time[7];
unsigned char start_time[2];
unsigned char end_time[2];
int stime;
int etime;
int operation_time;

int can_display_position = -1; //Data for can display screen
int log_position = -1;
int SOUP_LBL_count[4] = {0,0,0, 0};
int SOUP_NOLBL_count[4] = {0,0,0, 0};
int POPCAN_TAB_count[4] = {0,0,0, 0};
int POPCAN_NOTAB_count[4] = {0,0,0, 0};
int TOTAL_CAN_count[4] = {0,0,0,0};

int degree = 0;

int high = 0;
int DC_motor_direction = 0;

unsigned char curr_time[3];

int LoggedTimes[4] = {0,0,0, 0};
int TempTimes[4] = {0,0,0, 0};

int operation_disp = 0; //Data for operation running animation

unsigned char keypress = NULL;

//For queue       0 = SOUP_LBL
//                1 = SOUP_NOLBL
//                2 = POPCAN_TAB
//                3 = POPCAN_NOTAB



int canqueue[11];  // initialize array for 12 cans
int canqueue_tail;
int canqueue_head;
int data, testdata;
int moved = 1;

void initialize(void){
     // Set internal oscillator to run at 8 MHZ
    OSCCON = 0xF2; 
    // Enable PLL for the internal oscillator, Processor now runs at 32MHZ
    OSCTUNEbits.PLLEN = 1; 
    
    
    
    
    // TRIS register (Data Direction Register) 0 = output, 1 = input
    TRISA = 0b00111100; // RA0, RA1 as outputs
                        // RA0-1 - DC Motor H Bridge
                            // RA0 - Backwards
                            // RA1 - Forwards
    //RA0-3             // RA2 - ADCON (not programmed yet)
                        // RA3 - ADCON
                        // RA4 - Light Sensor
                        // RA5 - Light Sensor
                        // RA6 - Solenoid
    
                        
    
    //TRISB = 0b11110011;
    
    TRISB = 0b11111111; // RB1, RB4-7 as inputs
                        // RB2 - Sensor
                        // RB4-7 - Keypad data pins for the encoder PIC
                        // RB3 - CCP2 pin if CCP2MX is enabled in the configuration register - Servo motor 1
                        // RB1 - Keypad data available pin (active high)
    
    TRISC = 0x00; // RC0 - First Servo
                  // Timer1 oscillator output
                  // RC1 - Servo Motor 2
                  // RC2 - CCP1 pin: Maybe Servo motor 2
                  // RC3, RC4 - I2C pins for the RTC
                 
                  // LATC0, LATC1, LATC2 are SERVO
                   
    TRISD = 0x00; // All output mode for LCD, RD0 and RD1 unused
    
                  
    TRISE = 0x00; // RE0 - SOlenoid


    ADCON0 = 0x00; // Disable ADC
    ADCON1 = 0xFF; // Set PORTB digital
    CVRCON = 0x00; // Disable CCP reference voltage output
    ADFM = 1; // Right justify A/D result

    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    ADCON0 = 0x00;          //Disable ADC
    ADCON1 = 0xFF;          //Set PORTB to be digital instead of analog default  
    
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0x0B;  //AN0 to AN3 used as analog input
    CVRCON = 0x00; // Disable CCP reference voltage output
    CMCONbits.CIS = 0;
    ADFM = 1;
    
    //ei();                   //Global Interrupt Mask
    GIE = 1;
    INT1IE = 1;             //Enable KP interrupts
    INT0IE = 0;             //Disable external interrupts
    INT2IE = 0;
    
    nRBPU = 0;
    
    initLCD();
    I2C_Master_Init(10000); //Initialize I2C Master with 100KHz clock
    
    
    //Set Timer Properties
    TMR0 = 0;
    T08BIT = 0;
    T0CS = 0;
    PSA = 0;
    T0PS2 = 1;
    T0PS1 = 1;
    T0PS0 = 1; 
    
    // configure timer0
    
    //prescaler is 256
    T0PS0=1;
    T0PS1=1;
    T0PS2=1;
    
    PSA=0;      //Timer Clock Source is from Prescaler
    T0CS=0;     //Prescaler gets clock from FCPU (5MHz)
    T0SE=0;     // increment on every rising edge
    T08BIT=0;   //16 BIT MODE
    PEIE=1;     //Enable Peripheral Interrupt
    GIE=1;      //Enable INTs globally
    TMR0ON=0;   //disable timer0
    TMR0IE=0;   //disable timer0 interrupts
    TMR0IF=0;   //clear interrupt flag
    
    
    
    // <editor-fold defaultstate="collapsed" desc="Timer config">
    // configure timer0
    
    //prescaler is 256
    T0PS0=1;
    T0PS1=1;
    T0PS2=1;
    
    PSA=0;      //Timer Clock Source is from Prescaler
    T0CS=0;     //Prescaler gets clock from FCPU (5MHz)
    T0SE=0;     // increment on every rising edge
    T08BIT=0;   //16 BIT MODE
    PEIE=1;     //Enable Peripheral Interrupt
    GIE=1;      //Enable INTs globally
    TMR0ON=0;   //disable timer0
    TMR0IE=0;   //disable timer0 interrupts
    TMR0IF=0;   //clear interrupt flag
    
    
    // configure timer1
    T1CON = 0b10000000; // 16-bit operation
    // bit 6 is unimplemented
    // prescale - 1:8
    T1CKPS1 = 1;
    T1CKPS0 = 1;

    T1OSCEN = 0; // timer1 oscillator shut off
    T1SYNC = 1; // (bit is ignored when TMR1CS=0) do not synchronize with external clock input
    TMR1CS = 0; // timer mode
    TMR1ON=0; // disable timer1 for now
    TMR1IE = 0; // disable interrupts
    TMR1IF=0; //clear interrupt flag
    
    
    // configure timer2
   // bit7 is unimplemented
   // postscale 1:16
   // prescale 16
   // timer2 off
   T2CON = 0b01111010;
   
   PR2 = 0b10011100; //period for 20ms
   TMR2IF = 0; //clear interrupt flag
   TMR2IE = 0; // disable timer2 interrupts
    
   
    // configure timer3
    T3CON = 0b10000000; // 16-bit operation
    // prescale - 1:8
    T3CKPS1 = 1;
    T3CKPS0 = 1;
  
  
    // timer1 is clock source for compare/capture of CCP modules (not used)
    T3CCP2 = 0;
    T3CCP1 = 0;
    
    T3SYNC = 1; // (bit is ignored when TMR3CS=0) do not synchronize with external clock input
    TMR3CS = 0; // timer mode
    TMR3ON=0; // disable timer3 for now
    TMR3IE = 0; // disable interrupts
    TMR3IF = 0; //clear interrupt flag
      // </editor-fold>



}


void readADC(char channel){
    // Select A2D channel to read
    ADCON0 = ((channel <<2));
    ADON = 1;
    ADCON0bits.GO = 1;
   while(ADCON0bits.GO_NOT_DONE){__delay_ms(5);}
    
    
}

void main(void) {
//    set_time();
    initialize();
    update_servo_position(C);
    set_external_interrupt1(servo_down_period);
    
    TMR1IE = 0;
//    set_external_interrupt0(50);
    curr_state = STANDBY;
    
    while(1){
        switch(curr_state){
            case STANDBY:
                standby();
                break;
            case EMERGENCYSTOP:
                emergencystop();
                break;
            case OPERATION:
                operation();
                break;
            case OPERATIONEND:
                operationend();
                break;
            case CANCOUNT:
                can_count();
                break;
            case CANTIME:
                can_time();
                break;
            case LOGS:
                display_log();
                break;
        }
        __delay_ms(200);
    }
    
    return;
}



void FINALoperation(void){
    LATAbits.LATA6 = 1;   //RA6 - Push in the solenoid
    servo_rotate_RC0(90);
    servo_rotate_RC1(90);      
    
        if(PORTAbits.RA4 ){   // if 
            
            //RA4 - Sensor on Soup Can sides
            LATAbits.LATA1 = 0;  // Stop motor
            readADC(2);     // readADC(2) - RA2 for Soup Can side
            
            if (ADRESH*256 > 299){
                servo_rotate_RC1(180);  // RC1 for Soup Can side
                
            } else {
                servo_rotate_RC1(0);
            }
            servo_rotate_RC1(90);
            
        } else {
            LATAbits.LATA1 = 1;           
            __delay_ms(500);    
            LATAbits.LATA1 = 0;
            __delay_ms(1000);
        }
        
}

void LightSensorRA4_Soup(void){
    LATAbits.LATA6 = 1;   //RA6 - Push in the solenoid
    servo_rotate_RC0(90);
    servo_rotate_RC1(90); 
    run_number = 0;
    SOUP_NOLBL_count[run_number] = 0;
    POPCAN_TAB_count[run_number] = 0;
    POPCAN_NOTAB_count[run_number] = 0;
    TOTAL_CAN_count[run_number] = 0;    
    
        if(PORTAbits.RA4 ){   // if 
            
            //RA4 - Sensor on Soup Can sides
            LATAbits.LATA1 = 0;  // Stop motor
            readADC(2);     // readADC(2) - RA2 for Soup Can side
            
            if (ADRESH*256 > 299){
                servo_rotate_RC1(180);  // RC1 for Soup Can side
                SOUP_LBL_count[run_number] += 1;
            } else {
                servo_rotate_RC1(0);
                SOUP_NOLBL_count[run_number] += 1;
            }
            servo_rotate_RC1(90);
            
        } else {
            LATAbits.LATA1 = 1;           
            __delay_ms(500);    
            LATAbits.LATA1 = 0;
            __delay_ms(1000);
        }
        
        
                
                //TURN DC MOTOR STUFF
                
//        if(PORTAbits.RA4 ){//|| PORTAbits.RA4
//            LATAbits.LATA1 = 0;  
//            
//        } else {      
//            
//            LATAbits.LATA1 = 1;
//            __delay_ms(500);    
//            LATAbits.LATA1 = 0;
//            __delay_ms(1000);
            
           
            
//            servo_rotate_RC1(0);
//            servo_rotate_RC1(90);      
                    
              
        //}
    //}
}


void LightSensorRA5_Pop(void){
    while(1){
        if(PORTAbits.RA5 ){//|| PORTAbits.RA4
            LATAbits.LATA1 = 0;
        } else {      
            LATAbits.LATA1 = 1;
            __delay_ms(500);    
            LATAbits.LATA1 = 0;
            __delay_ms(1000);
            
//            readADC(2);
            if(PORTAbits.RA5){
                servo_rotate_RC0(0);
                servo_rotate_RC0(90);
            }       
             
//            else if(PORTAbits.RA4){
//                if (ADRESH*256 > 299){
//                    servo_rotate_RC1(180);
//                } else {
//                    servo_rotate_RC1(0);
//                }
//                servo_rotate_RC1(90);
//            }
        }
    }
}




void ADC_RA3(void){
    while(1) {
       readADC(2);
       __lcd_home();
       printf("%x+%x", ADRESH*256,ADRESL);
       printf(" AND ");
       
       if (ADRESH*256 > 299){
           servo_rotate_RC1(180);
       } else {
           servo_rotate_RC1(0);
       }
       
       servo_rotate_RC1(90);
       
//       
//       
//       analog_reading= ADRESL + (ADRESH *256);
//       printf(analog_reading); 
       
       
       readADC(3);
       printf("%x %x", ADRESH*256,ADRESL);
       
       
       printf("    ");
       __delay_1s();
    }
}

// <editor-fold defaultstate="collapsed" desc="Servo config">


void servo_rotate0(int degree){
    
//    65535-(int)((float)time*2000/256)
    
    unsigned int i;
    unsigned int j;
//     duty = ((degree+90)*5/90)+10;
    unsigned long duty = 1.27;
    unsigned long duty2 = 2.35;

    
    //1.255 is average

    INT1IF = 0;
    while(!INT1IF){
        for (i=0; i<50; i++) {

            LATCbits.LATC0 = 1;

            __delay_ms(1.5);
            LATCbits.LATC0 = 0;
            __delay_ms(18.5);

        }
        for (i=0; i<50; i++) {

            LATCbits.LATC0 = 1;
            __delay_ms(1);
            LATCbits.LATC0 = 0;
            __delay_ms(19);

        }
        
        for (i=0; i<50; i++) {

            LATCbits.LATC0 = 1;
            __delay_ms(2);
            LATCbits.LATC0 = 0;
            __delay_ms(18);

        }
    }
    
    
    
}

void servo_rotate1(int degree){
    unsigned int i;
    unsigned int j;
    int duty = 1500;

    for (i=0; i<80; i++) {
        LATCbits.LATC1 = 1;
        
        __delay_ms(1.5);
        LATCbits.LATC1 = 0;
        __delay_ms(18.5);
    }
    
     for (i=0; i<80; i++) {
        LATCbits.LATC1 = 1;
        __delay_ms(1);
        LATCbits.LATC1 = 0;
        __delay_ms(19);
    }
    return;
}

void servo_rotate_RC0(int degree){   
//    65535-(int)((float)time*2000/256)
    switch(degree) {
        case 0:
            // ROTATE RIGHT
            for (int i=0; i<70; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(0.7);
                LATCbits.LATC0 = 0;
                __delay_ms(19.3);
            }
            break;
        case 90:
            for (int i=0; i<70; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(1.2);
                LATCbits.LATC0 = 0;
                __delay_ms(18.8);
            }
            break;
        case 180:
            //ROTATE LEFT
            for (int i=0; i<70; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(1.6);
                LATCbits.LATC0 = 0;
                __delay_ms(18.4);
            }
            break;
        
    }   
    return;
}

void servo_rotate_RC1(int degree){ 
    switch(degree) {
        case 0:
            // ROTATE RIGHT
            for (int i=0; i<70; i++) {
                LATCbits.LATC1 = 1;
                __delay_ms(0.5);
                LATCbits.LATC1 = 0;
                __delay_ms(19.5);
            }
            break;
        case 90:
            for (int i=0; i<70; i++) {
                LATCbits.LATC1 = 1;
                __delay_ms(1);
                LATCbits.LATC1 = 0;
                __delay_ms(19);
            }
            break;
        case 180:
            //ROTATE LEFT
            for (int i=0; i<70; i++) {
                LATCbits.LATC1 = 1;
                __delay_ms(1.5);
                LATCbits.LATC1 = 0;
                __delay_ms(18.5);
            }
            break;
        
    }   
    return;
}
 // </editor-fold>

void solenoid_push(void){
    int i;
    
    while(1){
        LATAbits.LATA5 = 1;
        for (i=0; i<1000; i++) {   
                 __delay_ms(1);
            }
           

        LATAbits.LATA5 = 0;
     
        for (i=0; i<1000; i++) {
            __delay_ms(1);
        }
    }
}

void delay_10ms(unsigned char n) { 
         while (n-- != 0) { 
             __delay_ms(10); 
         } 
     }

void interrupt isr(void){
    if (INT1IF){  
//        if(currstate == UI_state) { // "If we're supposed to be in the UI..."
//            input = keys[(PORTB & 0xF0) >> 4];
//            
//            if(input == '*'){
//                machine_state = Testing_state;
//            }
//            else{
//                updateMenu();
//            }
//        }
        switch(PORTB){
            case 239:   //KP_#
                can_display_position = -1;
                log_position += 1;
                curr_state = LOGS;
                break;
            case 15:    //KP_1
                
//                
//                LATAbits.LATA2 = 1; //Start motor
//                INT0IE = 1;     //Enable external interrupts
//                INT2IE = 1;
//                TMR0IE = 1;     //Start timer with interrupts
//                TMR0ON = 1;
//                TMR0 = 0;
//                read_time();
//                start_time[1] = time[1];
//                start_time[0] = time[0];
//                
//                canqueue_head = canqueue_tail = 0; //Initiate queue
//                
//                __lcd_clear();
//                can_display_position = -1;
//                servo_rotate0(90);
                curr_state = OPERATION;
                break;
            case 31:    //KP_2
                can_display_position += 1;
                curr_state = CANCOUNT;
                while(PORTB == 31){}
                break;
            case 47:    //KP_3
                operation_time = etime - stime;
//                
//                
//                LoggedTimes[0] = operation_time;
//                LoggedTimes[1] = LoggedTimes[0];
//                LoggedTimes[2] = LoggedTimes[1];
//                LoggedTimes[3] = LoggedTimes[2];
                
                can_display_position = -1;
                curr_state = CANTIME;
                break;
            case 63:    //KP_A
                can_display_position = -1;
                curr_state = STANDBY;
                break;
            case 79:    //KP_4
                curr_state = OPERATIONEND;
                break;
            case 207:   //KP_*
                LATAbits.LATA2 = 0; //Stop motor
                di();               //Disable all interrupts
                TMR1ON = 0;
                INT1IE = 0;
                __lcd_clear();
                curr_state = EMERGENCYSTOP;
                break;
            case 127:   //KP_B
                read_time();
                start_time[1] = time[1];
                start_time[0] = time[0];
                __lcd_clear();

                can_display_position = -1;
                
                LightSensorRA4_Soup();
                break;
            case 191:   //KP_C              
                read_time();
                start_time[1] = time[1];
                start_time[0] = time[0];
                __lcd_clear();

                can_display_position = -1;
                
                
                curr_state = OPERATION;
                
//                ADC_RA3();
                
//                ADC_RA3();
//                read_ADC();
//                servo_rotate0(2);
                break;
        }
        INT1IF = 0;
    }
//   if (TMR0IE && TMR0IF) {
//        servo_ISR();
//////        stepper_ISR();
//        return; 
//    }  
//    
//    if (TMR1IF) {
////        TMR1IF = 0; // Clear interrupt flag
////        TMR1ON = 0;
//        
////    }
//
//    if (TMR1IE && TMR1IF) {
//        servo_ISR();
//        return;
//    }
    else if (INT0IF){   //Interrupt for first sensor at RB0
        if(PORTAbits.RA0){
            ADC_RA3();
            
            __delay_ms(150);
        }
        INT0IF = 0;
    }
    else if (INT2IF){   //Interrupt for second sensor at RB2
        if(PORTAbits.RA1){
            
            
            testdata = 1; //TESTING SERVO
            data = testdata;
            switch (data){
                case 0:
                    servo_rotate0(0);
                    servo_rotate2(0);
//                    SOUP_LBL_count += 1;
                    break;
                case 1:
                    servo_rotate0(0);
                    servo_rotate2(0);
//                    SOUP_NOLBL_count += 1;
                    break;
                case 2:
                    servo_rotate0(0);
                    servo_rotate2(120);
//                    POPCAN_TAB_count += 1;
                    break;
                case 3:
                    servo_rotate0(0);
                    servo_rotate2(120);
//                    POPCAN_NOTAB_count += 1;
                    break;
                // add additional cases if needed
            }
        }
        INT2IF = 0;
    }
//    else if (TMR0IF){ 
//          __lcd_home();
//          printf("TMR0IF");
//          __delay_1s();// interrupt of other sensor
////        LATAbits.LATA2 = 0;
////        TMR0ON = 0;
////        read_time();
////        end_time[1] = time[1];
////        end_time[0] = time[0];
////        stime = 60*dec_to_hex(start_time[1])+dec_to_hex(start_time[0]);
////        etime = 60*dec_to_hex(end_time[1])+dec_to_hex(end_time[0]);
////        __lcd_clear();
////        curr_state = OPERATIONEND;
////        can_display_position = -1;
//          TMR0IF = 0;
//    }
    else{
        while(1){
            __lcd_home();
            printf("Invalid Action");
            __delay_1s();
        }
    }
    return;
}

void getRTC(void){
    // Sets members in time array by reading from RTC module
       I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b11010000); //7 bit RTC address + Write
        I2C_Master_Write(0x00); //Set memory pointer to seconds
        I2C_Master_Stop(); //Stop condition

        //Read Current Time
        I2C_Master_Start();
        I2C_Master_Write(0b11010001); //7 bit RTC address + Read
        for(unsigned char j=0;j<0x06;j++){
            time[j] = I2C_Master_Read(1);
        }
        time[6] = I2C_Master_Read(0);       //Final Read without ack
        I2C_Master_Stop();
    //Reset RTC memory pointer
}


void DCforwardPWM(void){
    LATAbits.LATA0 = 0;
    DC_motor_direction = 0;
    while(1){
        LATAbits.LATA1 = 1;
        __delay_ms(1000);
        LATAbits.LATA1 = 0;
    }
}

//void DC_motor_RA(int pin){
//    if()
//            PORTAbits.RA0 = 1;
//            LATAbits.LATA0 = 1;
//            PORTAbits.RA1 = 0;
//            LATAbits.LATA1 = 0;
//    
//            PORTAbits.RA1 = 1;
//            LATAbits.LATA1 = 1;
//            PORTAbits.RA0 = 0;
//            LATAbits.LATA0 = 0;
//        break;
//    };
//}

void standby(void){
    getRTC();
    __lcd_home();
    
    printf("%02x/%02x %02x:%02x:%02x", time[5],time[4],time[2],time[1],time[0]);    //Print date in YY/MM/DD
    __lcd_newline();
    
    printf("1:Start #:Logs  ");    

    //DEBUGGING LINES
//    
//    __lcd_newline(); 
//    printf("DEBUGPORTB: %d ", PORTB);
//    return;
    
}

void set_time(void){
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    for(char i=0; i<7; i++){
        I2C_Master_Write(currentTime[i]);
    }    
    I2C_Master_Stop(); //Stop condition
}

int dec_to_hex(int num) {                   //Convert decimal unsigned char to hexadecimal int
    int i = 0, quotient = num, temp, hexnum = 0;
 
    while (quotient != 0) {
        temp = quotient % 16;
        
        hexnum += temp*pow(10,i);
        
        quotient = quotient / 16;
        i += 1;
    }
    return hexnum;
}

void read_time(void){
    //Reset RTC memory pointer 
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    I2C_Master_Stop(); //Stop condition

    //Read Current Time
    I2C_Master_Start();
    I2C_Master_Write(0b11010001); //7 bit RTC address + Read
    for(unsigned char i=0;i<0x06;i++){
        time[i] = I2C_Master_Read(1);   //Read with ack
    }
    time[6] = I2C_Master_Read(0);       //Final Read without ack
    I2C_Master_Stop();
    return;
}

void can_count(void){
    switch(can_display_position % 3){
        case 0:
            __lcd_home();
            printf("Operation Summary");
            __lcd_newline();
            printf("TOTAL CAN: %d    ", TOTAL_CAN_count);
            break;
        case 1:
            __lcd_home();
            printf("SOUP LBL: %d     ", SOUP_LBL_count);
            __lcd_newline();
            printf("SOUP NOLBL: %d   ", SOUP_NOLBL_count);
            break;
        case 2:
            __lcd_home();
            printf("POPCAN TAB: %d   ", POPCAN_TAB_count);
            __lcd_newline();
            printf("POPCAN NOTAB: %d ", POPCAN_NOTAB_count);
            break;

        default:
            while(1){
                __lcd_home();
                printf("ERROR: %d", can_display_position);
            }
            break;
    }
    return;
}

void display_log(void){
    switch(log_position % 3){
        case 0:

            //TODOS: REPLACE NUMBERS WITH %d
            
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count);
            keyPressed();   
            __lcd_home();
            printf("Run #1 Time:  %d ", LoggedTimes[0]);
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count);
            keyPressed();
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count);
            keyPressed();  
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count);
            keyPressed();  
            
            log_position = 1;
            break;
            
        case 1:
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count);
            keyPressed();   
            __lcd_home();
            printf("Run #2 Time:  %d ", LoggedTimes[1]);
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count);
            keyPressed();
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count);
            keyPressed();  
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count);
            keyPressed();  
            
            log_position = 2;
            break;
        case 2:
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count);
            keyPressed();   
            __lcd_home();
            printf("Run #3 Time:  %d ", LoggedTimes[2]);
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count);
            keyPressed();
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count);
            keyPressed();  
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count);
            keyPressed();  
            log_position = 0;
            break;// Cycle back to start
            
        default:
            while(1){
                __lcd_home();
                printf("ERROR: %d", log_position);
            }
            break;
    }
    return;
}

void can_time(void){
    __lcd_home();
    printf("Total Operation  ");
    __lcd_newline();
    printf("Time: %d s       ", operation_time);
    return;
}

void operation(void){
//    __lcd_home();
//    printf("Running..              ");  
//    __lcd_newline();
//    printf("PRESS 4 TO STOP  ");  
    printf("%02x %02x       ", start_time[1], start_time[0]);
    __delay_ms(500);
    int i;
    INT1IF = 0;
    
//    TMR1IF = 1; //clear_interrupt(timer1);
//    TMR1IE = 1;
//    servo_ISR();
    
    while(1){
        __delay_ms(1000);  
        read_time();
        curr_time[1] = time[1] - start_time[1];
        curr_time[0] = time[0] - start_time[0];
        
        getRTC();
        __lcd_home();
        printf("%02x %02x    %02x   %02x  ", curr_time[1], curr_time[0] , time[1], time[0]);
        //printf("%02x/%02x %02x:%02x:%02x", time[5],time[4],time[2],time[1],time[0]);    //Print date in YY/MM/DD
        __lcd_newline();

        if(curr_time[0] >= 10){
            curr_state = OPERATIONEND;
        }
    }
//          switch(operation_disp){
//                case 0:
//                    __lcd_home();
//                    printf("Running.              ");
//                    operation_disp = 1;
//                    break;
//                case 1:
//                    __lcd_home();
//                    printf("Running..              ");
//                    operation_disp = 2;
//                    break;
//                case 2:
//                    __lcd_home();
//                    printf("Running...              ");
//                    operation_disp = 3;
//                    break;
//                case 3:
//                    __lcd_home();
//                    printf("Running...              ");
//                    operation_disp = 4;
//                    break;
//                case 4:
//                    __lcd_home();
//                    printf("Running...              ");
//                    operation_disp = 0;
//                    break;
//            }
//
//            __lcd_newline();
//            read_sensor();
//            printf("PRESS 4 TO STOP  ");
//    
    
    return;
}

void operationend(void){
    __lcd_newline();
    __lcd_newline();
    printf("Operation Done!  ");
    __delay_ms(1000);
    
    LATCbits.LATC0 = 0; //Stop motor
    INT0IE = 0;         //Disable external interrupts
    INT2IE = 0;
    TMR0IE = 0;         //Disable timer
    TMR0ON = 0;
                
    read_time();
    end_time[1] = time[1];
                end_time[0] = time[0];
                stime = 60*dec_to_hex(start_time[1])+dec_to_hex(start_time[0]);
                etime = 60*dec_to_hex(end_time[1])+dec_to_hex(end_time[0]);
                __lcd_clear();
                can_display_position = -1;
                curr_state = STANDBY;
                
    operation_time = etime - stime; 
    TempTimes[3] = LoggedTimes[3];
    TempTimes[2] = LoggedTimes[2];
    TempTimes[1] = LoggedTimes[1];
    TempTimes[0] = LoggedTimes[0];
    
    LoggedTimes[3] = TempTimes[2];
    LoggedTimes[2] = TempTimes[1];
    LoggedTimes[1] = TempTimes[0];
    LoggedTimes[0] = operation_time;
    
    
   
    return;
}

void emergencystop(void){
    di();
    LATA = 0x00; // output low
    LATB = 0x00; // output low
    LATC = 0x00; // output low
    LATD = 0x00; // output low
    LATE = 0x00; // output low

    __lcd_home();
    printf("EMERGENCY,STOPPED");
    while(1){}
    return;
}


void servo_rotate2(int degree){
    unsigned int i;
    unsigned int j;
    int duty = ((degree+90)*5/90)+10;
    for (i=0; i<50; i++) {
        LATCbits.LATC2 = 1;
        for(j=0; j<duty; j++) __delay_us(100);
        LATCbits.LATC2 = 0;
        for(j=0; j<(200 - duty); j++) __delay_us(100);
    }
    return;
}

void read_sensor(void){
    return;
}

void read_ADC(void){
           readADC(3);
           __lcd_home();

           if(high >= 4) {
               printf("voltage");
               __delay_1s();
               __delay_1s();
               __delay_1s();
               __delay_1s();
               high = 0;
           }

           if(ADRESH >= 3){
               high = high + 1;
               printf("hig");
           } else {
               high = 0;
               printf("%x", ADRESH);
           }

           printf("   ");

           readADC(2);

           if (ADRESH >= 3) {
               high += 1;   
               printf("hig");
           } else {
               high = 0;
               printf("%x", ADRESH);
           }        

           printf("    ");
           __delay_1s();
           return;
        }

//
//////TEST CODE
//
//
void set_external_interrupt0(float time) {
//    extern volatile char FSM_enabled;
	// set interrupt for external timer0 for time in ms to control FSM
    unsigned int set_time = 65535-(int)((float)time*2000/256); // with oscillation at 8MHz, prescaler 256
    TMR0H = set_time >> 8;
    TMR0L = set_time & 0b11111111;
	TMR0ON=1; // turn timer0 on
    TMR0IF = 0; // clear interrupt flag
    TMR0IE=1; // enable timer0 interrupts
    
	// FSM stalls until countdown is completed
//	FSM_enabled = FALSE;
	return;
}

void set_external_interrupt1(float time) {
	// set interrupt for external timer1 for time in ms to control servo
	unsigned int set_time = 65535-(int)((float)time*2000/8); // with oscillation at 8MHz, prescaler 8
    TMR1H = set_time >> 8;
    TMR1L = set_time & 0b11111111;
	TMR1ON=1; // turn timer0 on
    TMR1IF = 0; // clear interrupt flag
    TMR1IE=1; // enable timer0 interrupts
	return;
}
//

void set_external_interrupt2(float time) {
	// set interrupt for external timer2 for time in ms to control wheel stepper
	unsigned int set_time = (int)((float)time*2000/256); // with oscillation 8MHz, prescaler 16, postscaler 16
    PR2 = set_time;
    TMR2IF = 0; // clear interrupt flag
    TMR2IE = 1;
    TMR2ON = 1;
	return;
}
//
//void set_external_interrupt3(float time) {
//	// set interrupt for external timer2 for time in ms to control belt stepper
//	unsigned int set_time = 65535-(int)((float)time*2000/8); // with oscillation at 8MHz, prescaler 8
//    TMR3H = set_time >> 8;
//    TMR3L = set_time & 0b11111111;
//	TMR3ON=1; // turn timer0 on
//    TMR3IF = 0; // clear interrupt flag
//    TMR3IE=1; // enable timer0 interrupts
//	return;
//}
//

#define SERVO_LAT LATCbits.LC0

void servo_ISR(void) {

	TMR1IF = 0; //clear_interrupt(timer1);
    TMR1IE = 0; // disable timer1 interrupts
    TMR1ON = 0; // turn timer1 off
    

	if (servo_current_state == SERVO_UP) {
		set_external_interrupt1(servo_down_period);
		servo_current_state = SERVO_DOWN;
		SERVO_LAT = 0;
	}
	else if (servo_current_state == SERVO_DOWN) {
		set_external_interrupt1(servo_up_period);
		servo_current_state = SERVO_UP;
		SERVO_LAT = 1;
	}

 
	return;
}

//
//#define SERVO_UPSCALE 1.25
//#define SERVO_UPTIME 0.25
//#define SERVO_DOWNTIME 20
//#define AA 0
//#define C 0.33
//#define NINEV 0.67
//#define DEAD 0.99
//#define SERVO_UP 1
//#define SERVO_DOWN 0

void update_servo_position(float bat_type) {
	// updates up and down period, which is used to drive the servo_IRQ
    extern volatile float servo_up_period;
    extern volatile float servo_down_period;

	servo_up_period = SERVO_UPTIME + SERVO_UPSCALE * bat_type;
	servo_down_period = SERVO_DOWNTIME;
	return;
}

int stepper_state;
#define STEPPER_LAT1 LATAbits.LA0
#define STEPPER_LAT2 LATAbits.LA1
#define STEPPER_LAT3 LATAbits.LA2
#define STEPPER_LAT4 LATAbits.LA3

void stepper_rotate(void){
    unsigned int i;
    unsigned int j;
    unsigned long duty = 500;

    for (i=0; i<180; i++) {
        STEPPER_LAT1 = 1;
        STEPPER_LAT2 = 0;
        STEPPER_LAT3 = 0;
        STEPPER_LAT4 = 0;
        
        __delay_ms(10);
        
        STEPPER_LAT1 = 0;
        STEPPER_LAT2 = 1;
        STEPPER_LAT3 = 0;
        STEPPER_LAT4 = 0;
        
        __delay_ms(10);
        
        STEPPER_LAT1 = 0;
        STEPPER_LAT2 = 0;
        STEPPER_LAT3 = 1;
        STEPPER_LAT4 = 0;
        __delay_ms(10);
        
        STEPPER_LAT1 = 0;
        STEPPER_LAT2 = 0;
        STEPPER_LAT3 = 0;
        STEPPER_LAT4 = 1;
        __delay_ms(10);
        
    }
    
    return;
}

//
//
//void stepper_ISR(void){ 
//    
//    TMR0IF = 0; //clear_interrupt(timer1);
//    TMR0IE = 0; // disable timer1 interrupts
//    TMR0ON = 0;
//           
//    stepper_state = 1;
//    switch(stepper_state){
//            case 1:
//                STEPPER_LAT1 = 1;
//                STEPPER_LAT2 = 0;
//                STEPPER_LAT3 = 0;
//                STEPPER_LAT4 = 0;
//                break;
//            case 2:
//                STEPPER_LAT1 = 0;
//                STEPPER_LAT2 = 1;
//                STEPPER_LAT3 = 0;
//                STEPPER_LAT4 = 0;
//                break;
//            case 3:
//                STEPPER_LAT1 = 0;
//                STEPPER_LAT2 = 0;
//                STEPPER_LAT3 = 1;
//                STEPPER_LAT4 = 0;
//                break;
//            case 4:
//                STEPPER_LAT1 = 0;
//                STEPPER_LAT2 = 0;
//                STEPPER_LAT3 = 0;
//                STEPPER_LAT4 = 1;
//                break;
//    }
//    stepper_state += 1;
//    stepper_state = stepper_state % 4;
//    set_external_interrupt0(50);
////    stepper_delay();
//}


//#define STEPPER_PERIOD 20;
//
//
//void stepper_ISR() {
//    extern volatile unsigned char belt_pin_num;
//    extern volatile unsigned char belt_angle_count;
//    extern volatile unsigned char rotary_pin_num;
//    extern volatile unsigned char rotary_angle_count;
//	TMR0IF = 0; //clear_interrupt(timer3);
//    TMR0IE = 0; // disable timer3 interrupts
//    TMR0ON = 0; // turn timer3 off
//    
//    if (belt_angle_count > 0){
//        switch(belt_pin_num) {
//           case 0:
//               STEPPER_LAT1=1;
//               STEPPER_LAT2=0;
//               STEPPER_LAT3=0;
//               STEPPER_LAT4=0;
//               break;
//           case 1:
//               STEPPER_LAT1=0;
//               STEPPER_LAT2=1;
//               STEPPER_LAT3=0;
//               STEPPER_LAT4=0;
//               break;
//           case 2:
//               STEPPER_LAT1=0;
//               STEPPER_LAT2=0;
//               STEPPER_LAT3=1;
//               STEPPER_LAT4=0;
//               break;
//           case 3:
//               STEPPER_LAT1=0;
//               STEPPER_LAT2=0;
//               STEPPER_LAT3=0;
//               STEPPER_LAT4=1;
//               break;
////            default:
////                whoops(10);
//       }
//        
//        
////        belt_pin_num = belt_pin_num == 3 ? 0 : belt_pin_num+1;
////        belt_angle_count = belt_angle_count == BELT_STEPPER_COUNT+1 ? 0 : belt_angle_count+1;
//    }
//    
//    set_external_interrupt0(STEPPER_PERIOD);
//	return;
//}





//    if (rotary_angle_count > 0){
//        switch(belt_pin_num) {
//           case 0:
//               ROTARY_LAT0=1;
//               ROTARY_LAT1=0;
//               ROTARY_LAT2=0;
//               ROTARY_LAT3=0;
//               break;
//           case 1:
//               ROTARY_LAT0=0;
//               ROTARY_LAT1=1;
//               ROTARY_LAT2=0;
//               ROTARY_LAT3=0;
//               break;
//           case 2:
//               ROTARY_LAT0=0;
//               ROTARY_LAT1=0;
//               ROTARY_LAT2=1;
//               ROTARY_LAT3=0;
//               break;
//           case 3:
//               ROTARY_LAT0=0;
//               ROTARY_LAT1=0;
//               ROTARY_LAT2=0;
//               ROTARY_LAT3=1;
//               break;
//            default:
//                whoops(11);
//       }
//        rotary_pin_num = rotary_pin_num == 3 ? 0 : rotary_pin_num+1;
//        rotary_angle_count = rotary_angle_count == ROTARY_STEPPER_COUNT+1 ? 0 : rotary_angle_count+1;
//    }
    

void keyPressed(void){
    while(PORTBbits.RB1 == 0){
        // RB1 is the interrupt pin, so if there is no key pressed, RB1 will be 0
            // the PIC will wait and do nothing until a key press is signaled
        }
        keypress = (PORTB & 0xF0)>>4; // Read the 4 bit character code
        while(PORTBbits.RB1 == 1){
            // Wait until the key has been released
        }
    Nop();  //breakpoint b/c compiler optimizations
    return;
}


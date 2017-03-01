
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

void display_log(void);
void read_sensor(void);

const char keys[] = "123A456B789C*0#D";
const char currentTime[7] = {   0x50, //Seconds 
                            0x35, //Minutes
                            0x21, //Hour, 24 hour mode
                            0x08, //Day of the week, Monday = 1
                            0x05, //Day/Date
                            0x02, //Month
                            0x17};//Year, last two digits

enum state {
        STANDBY,
        EMERGENCYSTOP,
        OPERATION,
        OPERATIONEND,
        DATETIME,
        CANCOUNT,
        CANTIME,
        LOGS
    };
    
enum state curr_state;

unsigned char time[7];
unsigned char start_time[2];
unsigned char end_time[2];
int stime;
int etime;
int operation_time;

int can_display_position = -1; //Data for can display screen
int log_position = -1;
int SOUP_LBL_count = 0;
int SOUP_NOLBL_count = 0;
int POPCAN_TAB_count = 0;
int POPCAN_NOTAB_count = 0;
int TOTAL_CAN_count = 0;

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

void initialize(void){
     // Set internal oscillator to run at 8 MHZ
    OSCCON = 0xF2; 
    // Enable PLL for the internal oscillator, Processor now runs at 32MHZ
    OSCTUNEbits.PLLEN = 1; 
    
    // TRIS register (Data Direction Register) 0 = output, 1 = input
    TRISA = 0b00001111; // RA0, RA1, RA2, RA3 as inputs
                        // RA0, RA1 - analog input for sensor
                        // RA4-7 - Stepper Motor
    
    //TRISB = 0b11110011;
    
    TRISB = 0b11111111; // RB1, RB4-7 as inputs
                        //RB2 LASER
                         // RB4-7 - Keypad data pins for the encoder PIC
                         // RB3 - CCP2 pin if CCP2MX is enabled in the configuration register - Servo motor 1
                         // RB1 - Keypad data available pin (active high)
    
    TRISC = 0x00; // RC0 - Timer1 oscillator output or Timer1/Timer3 clock input
                  // RC1 - Timer1 oscillator output
                  // RC2 - CCP1 pin: Servo motor 2
                  // RC3, RC4 - I2C pins for the RTC
    
    
                  // LATC0, LATC1,LATC2, SERVO
    
                  
    TRISD = 0x00; // All output mode for LCD, RD0 and RD1 unused
    TRISE = 0x00; // 


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
}



void main(void) {
    initialize();
//    // <editor-fold defaultstate="collapsed" desc=" STARTUP SEQUENCE ">
//    
//    // Set internal oscillator to run at 8 MHZ
//    OSCCON = 0xF2; 
//    // Enable PLL for the internal oscillator, Processor now runs at 32MHZ
//    OSCTUNEbits.PLLEN = 1; 
//    
//    // TRIS register (Data Direction Register) 0 = output, 1 = input
//    TRISA = 0b00001111; // RA0, RA1, RA2, RA3 as inputs
//                        // RA0, RA1 - analog input for sensor
//                        // RA4-7 - Stepper Motor
//    
//    //TRISB = 0b11110011;
//    
//    TRISB = 0b11111111; // RB1, RB4-7 as inputs
//                        //RB2 LASER
//                         // RB4-7 - Keypad data pins for the encoder PIC
//                         // RB3 - CCP2 pin if CCP2MX is enabled in the configuration register - Servo motor 1
//                         // RB1 - Keypad data available pin (active high)
//    
//    TRISC = 0x00; // RC0 - Timer1 oscillator output or Timer1/Timer3 clock input
//                  // RC1 - Timer1 oscillator output
//                  // RC2 - CCP1 pin: Servo motor 2
//                  // RC3, RC4 - I2C pins for the RTC
//    
//    
//                  // LATC0, LATC1,LATC2, SERVO
//    
//                  
//    TRISD = 0x00; // All output mode for LCD, RD0 and RD1 unused
//    TRISE = 0x00; // 
//
//
//    ADCON0 = 0x00; // Disable ADC
//    ADCON1 = 0xFF; // Set PORTB digital
//    CVRCON = 0x00; // Disable CCP reference voltage output
//    ADFM = 1; // Right justify A/D result
//
//    LATA = 0x00;
//    LATB = 0x00; 
//    LATC = 0x00;
//    LATD = 0x00;
//    LATE = 0x00;
//    
//    ADCON0 = 0x00;          //Disable ADC
//    ADCON1 = 0xFF;          //Set PORTB to be digital instead of analog default  
//    
//    //ei();                   //Global Interrupt Mask
//    GIE = 1;
//    INT1IE = 1;             //Enable KP interrupts
//    INT0IE = 0;             //Disable external interrupts
//    INT2IE = 0;
//    
//    nRBPU = 0;
//    
//    initLCD();
//    I2C_Master_Init(10000); //Initialize I2C Master with 100KHz clock
//    
//    
//    //Set Timer Properties
//    TMR0 = 0;
//    T08BIT = 0;
//    T0CS = 0;
//    PSA = 0;
//    T0PS2 = 1;
//    T0PS1 = 1;
//    T0PS0 = 1;  
//
//    //</editor-fold>
    
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

void interrupt isr(void){
    if (INT1IF) {
        switch(PORTB){
            case 239:   //KP_#
                can_display_position = -1;
                log_position += 1;
                printf("reached logs");
                curr_state = LOGS;
                break;
            case 15:    //KP_1
                LATAbits.LATA2 = 1; //Start centrifuge motor
                INT0IE = 1;     //Enable external interrupts
                INT2IE = 1;
                TMR0IE = 1;     //Start timer with interrupts
                TMR0ON = 1;
                TMR0 = 0;
                printf("reached logs");
                read_time();
                start_time[1] = time[1];
                start_time[0] = time[0];
                
                canqueue_head = canqueue_tail = 0; //Initiate queue
                
                __lcd_clear();
                can_display_position = -1;
                curr_state = OPERATION;
                break;
            case 31:    //KP_2
                can_display_position += 1;
                curr_state = CANCOUNT;
                while(PORTB == 31){}
                break;
            case 47:    //KP_3
                operation_time = etime - stime;
                can_display_position = -1;
                curr_state = CANTIME;
                break;
            case 63:    //KP_A
                can_display_position = -1;
                curr_state = STANDBY;
                break;
            case 79:    //KP_4
                LATAbits.LATA2 = 0; //Stop centrifuge motor
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
                curr_state = OPERATIONEND;
                break;
            case 207:   //KP_*
                LATAbits.LATA2 = 0; //Stop centrifuge motor
                di();               //Disable all interrupts
                TMR0ON = 0;
                __lcd_clear();
                curr_state = EMERGENCYSTOP;
                break;
            case 127:   //KP_B
                servo_rotate0(1);
                break;
            case 191:   //KP_C
                servo_rotate0(2);
                break;
        }
        INT1IF = 0;
    }
    else if (INT0IF){   //Interrupt for first laser sensor at RB0
        if(PORTAbits.RA3){
            read_sensor();
            
            __delay_ms(150);
        }
        INT0IF = 0;
    }
    else if (INT2IF){   //Interrupt for second laser sensor at RB2
        if(PORTAbits.RA4){
            
            
            testdata = 1; //TESTING SERVO
            data = testdata;
            switch (data){
                case 0:
                    servo_rotate0(0);
                    servo_rotate2(0);
                    SOUP_LBL_count += 1;
                    break;
                case 1:
                    servo_rotate0(0);
                    servo_rotate2(0);
                    SOUP_NOLBL_count += 1;
                    break;
                case 2:
                    servo_rotate0(0);
                    servo_rotate2(120);
                    POPCAN_TAB_count += 1;
                    break;
                case 3:
                    servo_rotate0(0);
                    servo_rotate2(120);
                    POPCAN_NOTAB_count += 1;
                    break;
                // add additional cases if needed
            }
        }
        INT2IF = 0;
    }
    else if (TMR0IF){ // interrupt of other sensor
        
        
        
        
        LATAbits.LATA2 = 0;
        TMR0ON = 0;
        read_time();
        end_time[1] = time[1];
        end_time[0] = time[0];
        stime = 60*dec_to_hex(start_time[1])+dec_to_hex(start_time[0]);
        etime = 60*dec_to_hex(end_time[1])+dec_to_hex(end_time[0]);
        __lcd_clear();
        curr_state = OPERATIONEND;
        can_display_position = -1;
        TMR0IF = 0;
    }
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

void standby(void){
    getRTC();
    __lcd_home();
//    printf("%02x/%02x %02x:%02x:%02x", time[5],time[4],time[2],time[1],time[0]);    //Print date in YY/MM/DD
//    __lcd_newline();
    printf("A:Start #:Logs  ");    

    //DEBUGGING LINES
    
    __lcd_newline(); 
    printf("DEBUGPORTB: %d ", PORTB);
    return;
    
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

void keycheck(void){
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

void can_count(void){
    switch(can_display_position % 3){
        case 0:
            __lcd_home();
            printf("Can Count       ");
            __lcd_newline();
            printf("Total: %d       ", TOTAL_CAN_count);
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
            keycheck();   
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count);
            keycheck();
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count);
            keycheck();  
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count);
            keycheck();  
            
            log_position += 1;
            
            
        case 1:
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count);
            keycheck();   
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count);
            keycheck();
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count);
            keycheck();  
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count);
            keycheck();  
            
            log_position += 1;
        case 2:
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count);
            keycheck();   
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count);
            keycheck();
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count);
            keycheck();  
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count);
            keycheck();  
            
            log_position += 1;
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
    switch(operation_disp){
        case 0:
            __lcd_home();
            printf("Running.              ");
            operation_disp = 1;
            break;
        case 1:
            __lcd_home();
            printf("Running..              ");
            operation_disp = 2;
            break;
        case 2:
            __lcd_home();
            printf("Running...              ");
            operation_disp = 0;
            break;
        case 3:
            __lcd_home();
            printf("Running...              ");
            operation_disp = 0;
            break;
        case 4:
            __lcd_home();
            printf("Running...              ");
            operation_disp = 0;
            break;
    }
    
    __lcd_newline();
    read_sensor();
    printf("PRESS 4 TO STOP  ");
    return;
}

void operationend(void){
    __lcd_home();
    printf("Operation Done!  ");
    return;
}

void emergencystop(void){
    di();
    PORTAbits.RA2 = 0;
    __lcd_clear();
    __lcd_home();
    printf("EMERGENCY,STOPPED");
    while(1){}
    return;
}

void servo_rotate0(int degree){
    unsigned int i;
    unsigned int j;
    int duty = degree;
    for (i=0; i<50; i++) {
        LATCbits.LATC0 = 1;
        for(j=0; j<duty; j++) __delay_ms(1);
        LATCbits.LATC0 = 0;
        for(j=0; j<(20 - duty); j++) __delay_ms(1);
    }
    return;
}

void servo_rotate1(int degree){
    unsigned int i;
    unsigned int j;
    int duty = ((degree+90)*5/90)+10;
    for (i=0; i<50; i++) {
        LATCbits.LATC1 = 1;
        for(j=0; j<duty; j++) __delay_us(100);
        LATCbits.LATC1 = 0;
        for(j=0; j<(200 - duty); j++) __delay_us(100);
    }
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
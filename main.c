
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

int sensor_triggered;

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

int solenoid = 0;
int conductive;
int run_number;
void display_log(void);
void read_sensor(void);

int Loading_Rotate_No_Sensor; 

unsigned char solenoid_pin;

const char keys[] = "123A456B789C*0#D";
const char currentTime[7] = {0x50, //Seconds 
    0x56, //Minutes
    0x07, //Hour, 24 hour mode
    0x03, //Day of the week, Monday = 1
    0x01, //Day/Date
    0x03, //Month
    0x17}; //Year, last two digits

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

unsigned char current_time[2];

int stime;
int etime;
int operation_time;

int can_display_position = -1; //Data for can display screen
int log_position = -1;
int SOUP_LBL_count[4] = {0, 0, 0, 0};
int SOUP_NOLBL_count[4] = {0, 0, 0, 0};
int POPCAN_TAB_count[4] = {0, 0, 0, 0};
int POPCAN_NOTAB_count[4] = {0, 0, 0, 0};
int TOTAL_CAN_count[4] = {0, 0, 0, 0};

int degree = 0;

int ROTATION_COUNT;

int high = 0;
int DC_motor_direction = 0;

int curr_time;
float time_diff;
int begin_time;

int current_seconds;
int minutes_difference;
int start_minutes;
int start_seconds;

int Total_Operation_Time;

int LoggedTimes[4] = {0, 0, 0, 0};
int TempTimes[4] = {0, 0, 0, 0};

int operation_disp = 0; //Data for operation running animation

unsigned char keypress = NULL;

//For queue       0 = SOUP_LBL
//                1 = SOUP_NOLBL
//                2 = POPCAN_TAB
//                3 = POPCAN_NOTAB



int canqueue[11]; // initialize array for 12 cans
int canqueue_tail;
int canqueue_head;
int data, testdata;
int moved = 1;

void initialize(void) {
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


    TRISE = 0x00; // RE0 - Solenoid
    // RE1 - Solenoid


    ADCON0 = 0x00; // Disable ADC
    ADCON1 = 0xFF; // Set PORTB digital
    CVRCON = 0x00; // Disable CCP reference voltage output
    ADFM = 1; // Right justify A/D result

    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;

    ADCON0 = 0x00; //Disable ADC
    ADCON1 = 0xFF; //Set PORTB to be digital instead of analog default  

    ADCON0 = 0x00; //Disable ADC
    ADCON1 = 0x0B; //AN0 to AN3 used as analog input
    CVRCON = 0x00; // Disable CCP reference voltage output
    CMCONbits.CIS = 0;
    ADFM = 1;

    //ei();                   //Global Interrupt Mask
    GIE = 1;
    INT1IE = 1; //Enable KP interrupts
    INT0IE = 0; //Disable external interrupts
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
    T0PS0 = 1;
    T0PS1 = 1;
    T0PS2 = 1;

    PSA = 0; //Timer Clock Source is from Prescaler
    T0CS = 0; //Prescaler gets clock from FCPU (5MHz)
    T0SE = 0; // increment on every rising edge
    T08BIT = 0; //16 BIT MODE
    PEIE = 1; //Enable Peripheral Interrupt
    GIE = 1; //Enable INTs globally
    TMR0ON = 0; //disable timer0
    TMR0IE = 0; //disable timer0 interrupts
    TMR0IF = 0; //clear interrupt flag



    // <editor-fold defaultstate="collapsed" desc="Timer config">
    // configure timer0

    //prescaler is 256
    T0PS0 = 1;
    T0PS1 = 1;
    T0PS2 = 1;

    PSA = 0; //Timer Clock Source is from Prescaler
    T0CS = 0; //Prescaler gets clock from FCPU (5MHz)
    T0SE = 0; // increment on every rising edge
    T08BIT = 0; //16 BIT MODE
    PEIE = 1; //Enable Peripheral Interrupt
    GIE = 1; //Enable INTs globally
    TMR0ON = 0; //disable timer0
    TMR0IE = 0; //disable timer0 interrupts
    TMR0IF = 0; //clear interrupt flag


    // configure timer1
    T1CON = 0b10000000; // 16-bit operation
    // bit 6 is unimplemented
    // prescale - 1:8
    T1CKPS1 = 1;
    T1CKPS0 = 1;

    T1OSCEN = 0; // timer1 oscillator shut off
    T1SYNC = 1; // (bit is ignored when TMR1CS=0) do not synchronize with external clock input
    TMR1CS = 0; // timer mode
    TMR1ON = 0; // disable timer1 for now
    TMR1IE = 0; // disable interrupts
    TMR1IF = 0; //clear interrupt flag


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
    TMR3ON = 0; // disable timer3 for now
    TMR3IE = 0; // disable interrupts
    TMR3IF = 0; //clear interrupt flag
    // </editor-fold>



}

void readADC(char channel) {
    // Select A2D channel to read
    ADCON0 = ((channel << 2));
    ADON = 1;
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO_NOT_DONE) {
        __delay_ms(5);
    }


}

void main(void) {
    //    set_time();
    initialize();
    //    update_servo_position(C);
    set_external_interrupt1(servo_down_period);

    TMR1IE = 0;
    // TAKE THIS OUT ONCE SERVO TESTS IS ODNE
    //    set_external_interrupt0(50);
    curr_state = STANDBY;

    while (1) {
        switch (curr_state) {
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

void FINALoperation(void) {
    LATEbits.LATE0 = 1; //RA6 - Push in the solenoid
    servo_rotate_RC0(90); // Set Servos to Initial Position
    servo_rotate_RC1(90);

    run_number = run_number + 1 % 4;
    conductive = 0;
    SOUP_LBL_count[run_number] = 0;
    SOUP_NOLBL_count[run_number] = 0;
    POPCAN_TAB_count[run_number] = 0;
    POPCAN_NOTAB_count[run_number] = 0;
    TOTAL_CAN_count[run_number] = 0;

    Total_Operation_Time = 0;
    Loading_Rotate_No_Sensor = 0; 

    ROTATION_COUNT = 0; // For incrementing rotation routine, if weight slows

    //====================
    // ROTATING CONE LOOP
    //==================== 
    for (int i = 0; i < 3; i++) { //Do forward/backwards i times
        LATCbits.LATC5 = 1;
        LATCbits.LATC6 = 0;

        for (int i = 0; i < 10; i++) {
            LATCbits.LATC7 = 1;
            __delay_ms(10);
            LATCbits.LATC7 = 0;
            __delay_ms(10);
        }

        Total_Operation_Time += 20 * 10;

        for (int i = 0; i < 17; i++) {
            LATCbits.LATC7 = 1;
            __delay_ms(4);
            LATCbits.LATC7 = 0;
            __delay_ms(16);
        }

        Total_Operation_Time += 20 * 17;

        //__delay_ms(250);

        LATCbits.LATC5 = 0;
        LATCbits.LATC6 = 1;

        for (int i = 0; i < 22; i++) {
            LATCbits.LATC7 = 1;
            __delay_ms(6);
            LATCbits.LATC7 = 0;
            __delay_ms(14);
        }

        Total_Operation_Time += 20 * 22;

    }

    //===============
    // WINDMILL STUFF
    //===============

    while (TOTAL_CAN_count[run_number] < 12 && Total_Operation_Time < 180 * 1000) {
        if (PORTAbits.RA4) { //RA4 - Sensor on Soup Can sides
            LATAbits.LATA1 = 0; // Stop Windmill
            TOTAL_CAN_count[run_number] += 1;

            __delay_ms(500);
            Total_Operation_Time += 500;

            for (int i = 0; i < 5; i++) {
                readADC(2);
                __lcd_newline();
                printf("ADRESH %x ", ADRESH * 256);
                __lcd_newline();

                if (ADRESH * 256 > 100) {
                    LATEbits.LATE0 = 0; // some interference
                    LATEbits.LATE1 = 0;
                    conductive = 1;
                    while (PORTAbits.RA4) {
                        // RC1 for Soup Can side
                        servo_rotate_RC1(180);
                        Total_Operation_Time += 20 * 70;
                    }
                    SOUP_LBL_count[run_number] += 1;
                }
                __delay_ms(500);
                Total_Operation_Time += 500;
            }
            // readADC(2) - RA2 for Soup Can side    
            if (!conductive) {
                LATEbits.LATE0 = 0;
                LATEbits.LATE1 = 0;
                while (PORTAbits.RA4) {
                    servo_rotate_RC1(0);
                    Total_Operation_Time += 20 * 70;
                }
                SOUP_NOLBL_count[run_number] += 1;
            }

            servo_rotate_RC1(90);
            Total_Operation_Time += 20 * 70;
            conductive = 0;

            LATEbits.LATE0 = 1;
            LATEbits.LATE1 = 1;
            
            Loading_Rotate_No_Sensor = 0; 
        } else if (PORTAbits.RA5) {
            TOTAL_CAN_count[run_number] += 1;
            //RA5 - Sensor on Pop Can sides
            LATAbits.LATA1 = 0; // Stop motor
            //            __delay_ms(500);
            //            Total_Operation_Time += 500;

            servo_rotate_RC0(85);
            Total_Operation_Time += 20 * 70;

            LATEbits.LATE0 = 0;

            for (int i = 0; i < 5; i++) {
                readADC(3); // readADC(3) - RA3 - Pop Can side
                //                __lcd_newline();
                //                printf("POP: ");
                //                __lcd_newline();
                //                printf("%x %x       ", ADRESH * 256, PORTAbits.RA3);
                __delay_ms(250);
                Total_Operation_Time += 400;

                if (ADRESH * 256 > 300 && !conductive) {
                    LATEbits.LATE0 = 1;
                    __delay_ms(500);
                    Total_Operation_Time += 500;

                    conductive = 1;
                    while (PORTAbits.RA5) { // RC0 for Pop Can side
                        servo_rotate_RC0(180);
                        Total_Operation_Time += 20 * 70;
                        servo_rotate_RC0(90);
                        Total_Operation_Time += 20 * 70;
                    }
                    POPCAN_TAB_count[run_number] += 1;

                } else if (conductive) {
                    break;
                }
            }

            if (!conductive) {
                LATEbits.LATE0 = 1;
                LATEbits.LATE1 = 1;

                while (PORTAbits.RA5) {
                    servo_rotate_RC0(0);
                    Total_Operation_Time += 20 * 70;
                    servo_rotate_RC0(90);
                    Total_Operation_Time += 20 * 70;
                }
                POPCAN_NOTAB_count[run_number] += 1;

            }

            conductive = 0;
            LATEbits.LATE0 = 1;
            LATEbits.LATE1 = 1;
            
            Loading_Rotate_No_Sensor = 0; 
        } else {
            
            if(Loading_Rotate_No_Sensor > 15){break;}
            
            Loading_Rotate_No_Sensor += 1; 

            //====================
            // ROTATING CONE STUFF
            //====================  

            for (int i = 0; i < 3; i++) { //Do forward/backwards i times

                LATCbits.LATC5 = 1;
                LATCbits.LATC6 = 0;

                if (ROTATION_COUNT > 10) { // Slower speed: 6/14 - 4/16 - 4/16
                    for (int i = 0; i < 10; i++) {
                        LATCbits.LATC7 = 1;
                        __delay_ms(6);
                        LATCbits.LATC7 = 0;
                        __delay_ms(14);
                    }

                    Total_Operation_Time += 20 * 10;

                    for (int i = 0; i < 17; i++) {
                        LATCbits.LATC7 = 1;
                        __delay_ms(4);
                        LATCbits.LATC7 = 0;
                        __delay_ms(16);
                    }

                    Total_Operation_Time += 20 * 17;

                    //__delay_ms(250);

                    LATCbits.LATC5 = 0;
                    LATCbits.LATC6 = 1;

                    for (int i = 0; i < 22; i++) {
                        LATCbits.LATC7 = 1;
                        __delay_ms(4);
                        LATCbits.LATC7 = 0;
                        __delay_ms(16);
                    }

                    Total_Operation_Time += 20 * 22;

                } else {
                    for (int i = 0; i < 10; i++) {
                        LATCbits.LATC7 = 1;
                        __delay_ms(10);
                        LATCbits.LATC7 = 0;
                        __delay_ms(10);
                    }

                    Total_Operation_Time += 20 * 10;

                    for (int i = 0; i < 17; i++) {
                        LATCbits.LATC7 = 1;
                        __delay_ms(4);
                        LATCbits.LATC7 = 0;
                        __delay_ms(16);
                    }

                    Total_Operation_Time += 20 * 17;
                    //__delay_ms(250);

                    LATCbits.LATC5 = 0;
                    LATCbits.LATC6 = 1;

                    for (int i = 0; i < 22; i++) {
                        LATCbits.LATC7 = 1;
                        __delay_ms(6);
                        LATCbits.LATC7 = 0;
                        __delay_ms(14);
                    }

                    Total_Operation_Time += 20 * 22;

                }

                ROTATION_COUNT += 1;
                

            }

            //===============
            // WINDMILL STUFF
            //===============

            LATAbits.LATA1 = 1; //Start forwards
            __delay_ms(500);
            LATAbits.LATA1 = 0; // Stop backwards
            __delay_ms(1500); /// Wait for cans to drop into places
            LATAbits.LATA0 = 1; // Go backwards 
            __delay_ms(300);
            LATAbits.LATA0 = 0; // Stop backwards
            __delay_ms(1500); /// Wait for cans to drop into places

            Total_Operation_Time += 3800;
        }
    }

    curr_state = OPERATIONEND;

}


// <editor-fold defaultstate="collapsed" desc="Soup & Pop Tests">

void LightSensorRA4_Soup(void) {

    servo_rotate_RC1(90);
    LATEbits.LATE0 = 1; //RA6 - Push in the solenoid

    run_number = 0;
    conductive = 0;
    SOUP_LBL_count[run_number] = 0;
    SOUP_NOLBL_count[run_number] = 0;
    POPCAN_TAB_count[run_number] = 0;
    POPCAN_NOTAB_count[run_number] = 0;
    TOTAL_CAN_count[run_number] = 0;

    while (TOTAL_CAN_count[run_number] < 12) {
        __lcd_home();
        printf("%x %x %x %x %x", SOUP_LBL_count[run_number], SOUP_NOLBL_count[run_number], POPCAN_TAB_count[run_number], POPCAN_NOTAB_count[run_number], TOTAL_CAN_count[run_number]);
        LATEbits.LATE0 = 1;
        LATEbits.LATE1 = 1;

        if (PORTAbits.RA4) { // if    
            //RA4 - Sensor on Soup Can sides
            LATAbits.LATA1 = 0; // Stop motor
            TOTAL_CAN_count[run_number] += 1;

            __delay_ms(1000);

            for (int i = 0; i < 5; i++) {
                readADC(2);
                __lcd_newline();
                printf("ADRESH %x ", ADRESH * 256);
                __lcd_newline();

                if (ADRESH * 256 > 100) {
                    LATEbits.LATE0 = 0; // some interference
                    conductive = 1;
                    while (PORTAbits.RA4) {
                        servo_rotate_RC1(180); // RC1 for Soup Can side                    
                        servo_rotate_RC1(90);
                    }

                    SOUP_LBL_count[run_number] += 1;
                }
                __delay_ms(300);
            }
            // readADC(2) - RA2 for Soup Can side    
            if (!conductive) {
                LATEbits.LATE0 = 0;

                while (PORTAbits.RA4) {
                    servo_rotate_RC1(0);
                    servo_rotate_RC1(90);
                }
                SOUP_NOLBL_count[run_number] += 1;
            }
            conductive = 0;

            LATEbits.LATE0 = 1;
            LATEbits.LATE1 = 1;
        } else {
            LATAbits.LATA1 = 1;
            __delay_ms(650);
            LATAbits.LATA1 = 0;
            __delay_ms(1500); /// WAIT SOME TIME
            LATAbits.LATA0 = 1;
            __delay_ms(300);
            LATAbits.LATA0 = 0;
            __delay_ms(1500);

            //            if(!PORTAbits.RA4 && !PORTAbits.RA5){  // If the sensor still doesn't detect anything
            //                 // GO BACKWARDS
            //                   ///WAIT SOME MORE TIME
            //            }
        }
    }
    curr_state = OPERATIONEND; //at the end of while loop, go to operationend
}

void LightSensorRA5_Pop(void) {
    LATEbits.LATE0 = 1; //RA6 - Push in the solenoid

    servo_rotate_RC0(90);

    run_number = 0;
    conductive = 0;
    SOUP_LBL_count[run_number] = 0;
    SOUP_NOLBL_count[run_number] = 0;
    POPCAN_TAB_count[run_number] = 0;
    POPCAN_NOTAB_count[run_number] = 0;
    TOTAL_CAN_count[run_number] = 0;

    while (TOTAL_CAN_count[run_number] < 12) {
        __lcd_home();
        printf("%x %x %x %x %x", SOUP_LBL_count[run_number], SOUP_NOLBL_count[run_number], POPCAN_TAB_count[run_number], POPCAN_NOTAB_count[run_number], TOTAL_CAN_count[run_number]);
        LATEbits.LATE0 = 1;
        LATEbits.LATE1 = 1;

        if (PORTAbits.RA5) {
            TOTAL_CAN_count[run_number] += 1;
            //RA5 - Sensor on Pop Can sides
            LATAbits.LATA1 = 0; // Stop motor

            __delay_ms(500);

            LATEbits.LATE0 = 0;
            LATEbits.LATE1 = 0;


            for (int i = 0; i < 5; i++) {
                readADC(3); // readADC(3) - RA3 - Pop Can side
                __lcd_newline();
                printf("POP: ");
                __lcd_newline();
                printf("%x %x       ", ADRESH * 256, PORTAbits.RA3);
                __delay_ms(400);
                if (ADRESH * 256 > 300 && !conductive) {
                    LATEbits.LATE0 = 1;
                    LATEbits.LATE1 = 1;

                    __delay_ms(1000);

                    conductive = 1;
                    while (PORTAbits.RA5) {
                        servo_rotate_RC0(180);
                        servo_rotate_RC0(90);
                    }
                    // RC0 for Pop Can side

                    POPCAN_TAB_count[run_number] += 1;
                }

            }

            if (!conductive) {
                LATEbits.LATE0 = 1;
                LATEbits.LATE1 = 1;

                while (PORTAbits.RA5) {
                    servo_rotate_RC0(0);
                    servo_rotate_RC0(90);
                }
                POPCAN_NOTAB_count[run_number] += 1;

            }

            conductive = 0;
            LATEbits.LATE0 = 1;
            LATEbits.LATE1 = 1;

        }
        else {
            LATAbits.LATA1 = 1;
            __delay_ms(650);
            LATAbits.LATA1 = 0;
            __delay_ms(1500); /// WAIT SOME TIME
            LATAbits.LATA0 = 1;
            __delay_ms(300);
            LATAbits.LATA0 = 0;
            __delay_ms(1500);

            //            if(!PORTAbits.RA4 && !PORTAbits.RA5){  // If the sensor still doesn't detect anything
            //                 // GO BACKWARDS
            //                   ///WAIT SOME MORE TIME
            //            }
        }
    }
    curr_state = OPERATIONEND;
}

// </editor-fold>

void ADC_RA3(void) {
    while (1) {
        readADC(2);
        __lcd_home();
        printf("%x+%x", ADRESH * 256, ADRESL);
        printf(" AND ");

        if (ADRESH * 256 > 299) {
            servo_rotate_RC1(180);
        } else {
            servo_rotate_RC1(0);
        }

        servo_rotate_RC1(90);

        readADC(3);
        printf("%x %x", ADRESH * 256, ADRESL);

        printf("    ");
        __delay_1s();
    }
}

// <editor-fold defaultstate="collapsed" desc="Servo Codes">

void servo_rotate0(int degree) {
    //    65535-(int)((float)time*2000/256)  
    unsigned int i;
    unsigned int j;
    //     duty = ((degree+90)*5/90)+10;
    unsigned long duty = 1.27;
    unsigned long duty2 = 2.35;

    INT1IF = 0;
    while (!INT1IF) {
        for (i = 0; i < 50; i++) {
            LATCbits.LATC0 = 1;
            __delay_ms(1.5);
            LATCbits.LATC0 = 0;
            __delay_ms(18.5);
        }
        for (i = 0; i < 50; i++) {
            LATCbits.LATC0 = 1;
            __delay_ms(1);
            LATCbits.LATC0 = 0;
            __delay_ms(19);
        }
        for (i = 0; i < 50; i++) {
            LATCbits.LATC0 = 1;
            __delay_ms(2);
            LATCbits.LATC0 = 0;
            __delay_ms(18);
        }
    }
}

void servo_rotate1(int degree) {
    unsigned int i;
    unsigned int j;
    int duty = 1500;

    for (i = 0; i < 80; i++) {
        LATCbits.LATC1 = 1;

        __delay_ms(1.5);
        LATCbits.LATC1 = 0;
        __delay_ms(18.5);
    }

    for (i = 0; i < 80; i++) {
        LATCbits.LATC1 = 1;
        __delay_ms(1);
        LATCbits.LATC1 = 0;
        __delay_ms(19);
    }
    return;
}

void servo_rotate_RC0(int degree) {
    //    65535-(int)((float)time*2000/256)
    switch (degree) {
        case 0:
            // ROTATE RIGHT
            for (int i = 0; i < 50; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(0.4);
                LATCbits.LATC0 = 0;
                __delay_ms(19.6);
            }
            break;
        case 85:
            for (int i = 0; i < 50; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(1.1);
                LATCbits.LATC0 = 0;
                __delay_ms(18.9);
            }
            break;
        case 90:
            for (int i = 0; i < 70; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(1.2);
                LATCbits.LATC0 = 0;
                __delay_ms(18.8);
            }
            break;
        case 180:
            //ROTATE LEFT
            for (int i = 0; i < 50; i++) {
                LATCbits.LATC0 = 1;
                __delay_ms(1.7);
                LATCbits.LATC0 = 0;
                __delay_ms(18.3);
            }
            break;

    }
    return;
}

void servo_rotate_RC1(int degree) {
    switch (degree) {
        case 0:
            // ROTATE RIGHT
            for (int i = 0; i < 70; i++) {
                LATCbits.LATC1 = 1;
                __delay_ms(0.3);
                LATCbits.LATC1 = 0;
                __delay_ms(19.7);
            }
            break;
        case 90:
            for (int i = 0; i < 70; i++) {
                LATCbits.LATC1 = 1;
                __delay_ms(1.0);
                LATCbits.LATC1 = 0;
                __delay_ms(19.0);
            }
            break;
        case 180:
            //ROTATE LEFT
            for (int i = 0; i < 70; i++) {
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

void delay_10ms(unsigned char n) {
    while (n-- != 0) {
        __delay_ms(10);
    }
}

void interrupt isr(void) {
    if (INT1IF) {
        switch (PORTB) {
            case 239: //KP_#
                can_display_position = -1;
                log_position += 1;
                curr_state = LOGS;
                break;
            case 15: //KP_1     
                read_time();
                start_time[1] = time[1];
                start_time[0] = time[0];
                __lcd_clear();

                can_display_position = -1;

                FINALoperation();

                //                servo_rotate_RC1(0);
                //                servo_rotate_RC1(180);
                //                servo_rotate_RC1(90);
                //                LATEbits.LATE0 = 1;
                //                servo_rotate_RC0(0);
                //                servo_rotate_RC0(180);
                //                servo_rotate_RC0(90);

                //                curr_state = OPERATION;
                break;
            case 31: //KP_2
                can_display_position += 1;
                curr_state = CANCOUNT;
                while (PORTB == 31) {
                }
                break;
            case 47: //KP_3
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
            case 63: //KP_A
                can_display_position = -1;
                curr_state = STANDBY;
                break;
            case 79: //KP_4          
                can_display_position = -1;
                curr_state = OPERATIONEND;
                break;
            case 207: //KP_*
                LATAbits.LATA2 = 0; //Stop motor
                di(); //Disable all interrupts
                TMR1ON = 0;
                INT1IE = 0;
                __lcd_clear();
                //                servo_rotate_RC0(0);
                //                servo_rotate_RC0(180);
                //                servo_rotate_RC0(90);

                servo_rotate_RC1(0);
                servo_rotate_RC1(180);
                servo_rotate_RC1(90);

                //curr_state = EMERGENCYSTOP;
                break;
            case 127: //KP_B
                read_time();
                start_time[1] = time[1];
                start_time[0] = time[0];
                __lcd_clear();

                can_display_position = -1;

                LightSensorRA4_Soup();
                break;
            case 191: //KP_C              
                read_time();
                start_time[1] = time[1];
                start_time[0] = time[0];
                __lcd_clear();

                can_display_position = -1;

                LightSensorRA5_Pop();
                break;
            case 255: //KP_D
                curr_state = OPERATION;
                break;
            case 143: // KP_7  CONE SPEED TEST
                LATCbits.LATC5 = 1;
                LATCbits.LATC6 = 0;

                //                for (int i=0; i<150; i++) {
                //                   LATCbits.LATC7 = 1; 
                //                    __delay_ms(5);
                //                    LATCbits.LATC7 = 0; 
                //                   __delay_ms(15);
                //                }

                ROTATION_COUNT = 0;
                Total_Operation_Time = 0;

                //====================
                // ROTATING CONE STUFF
                //====================  
                while (1) {
                    for (int i = 0; i < 3; i++) { //Do forward/backwards i times

                        __lcd_newline();
                        printf("%d            ", Total_Operation_Time);

                        LATCbits.LATC5 = 1;
                        LATCbits.LATC6 = 0;

                        if (ROTATION_COUNT < 50) {
                            for (int i = 0; i < 12; i++) {
                                LATCbits.LATC7 = 1;
                                __delay_ms(10);
                                LATCbits.LATC7 = 0;
                                __delay_ms(10);
                            }

                            Total_Operation_Time += 20 * 11 * 1.25;

                            //                            __lcd_newline();
                            //                            printf("%d            ", Total_Operation_Time);

                            for (int i = 0; i < 17; i++) {
                                LATCbits.LATC7 = 1;
                                __delay_ms(6);
                                LATCbits.LATC7 = 0;
                                __delay_ms(14);
                            }

                            Total_Operation_Time += 20 * 17 * 1.25;

                            //__delay_ms(250);

                            LATCbits.LATC5 = 0;
                            LATCbits.LATC6 = 1;

                            for (int i = 0; i < 25; i++) {
                                LATCbits.LATC7 = 1;
                                __delay_ms(6);
                                LATCbits.LATC7 = 0;
                                __delay_ms(14);
                            }

                            Total_Operation_Time += 20 * 22 * 1.25;


                        } else { // Slower speed: 6/14 - 4/16 - 4/16
                            for (int i = 0; i < 11; i++) {
                                LATCbits.LATC7 = 1;
                                __delay_ms(6);
                                LATCbits.LATC7 = 0;
                                __delay_ms(14);
                            }

                            Total_Operation_Time += 20 * 11 * 1.25;

                            for (int i = 0; i < 17; i++) {
                                LATCbits.LATC7 = 1;
                                __delay_ms(4);
                                LATCbits.LATC7 = 0;
                                __delay_ms(16);
                            }

                            Total_Operation_Time += 20 * 17 * 1.25;
                            //__delay_ms(250);

                            LATCbits.LATC5 = 0;
                            LATCbits.LATC6 = 1;

                            for (int i = 0; i < 22; i++) {
                                LATCbits.LATC7 = 1;
                                __delay_ms(4);
                                LATCbits.LATC7 = 0;
                                __delay_ms(16);
                            }

                            Total_Operation_Time += 20 * 22 * 1.25;

                        }
                        ROTATION_COUNT += 1;
                    }

                    __delay_ms(2000);


                            //}
                }
                curr_state = STANDBY;
                break;
        }
        INT1IF = 0;
    }        //   if (TMR0IE && TMR0IF) {
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
        //    else if (INT0IF){   //Interrupt for first sensor at RB0
        //        if(PORTAbits.RA0){
        //            ADC_RA3();
        //            
        //            __delay_ms(150);
        //        }
        //        INT0IF = 0;
        //    }
        //    else if (INT2IF){   //Interrupt for second sensor at RB2
        //        if(PORTAbits.RA1){
        //            
        //            
        //            testdata = 1; //TESTING SERVO
        //            data = testdata;
        //            switch (data){
        //                case 0:
        //                    servo_rotate0(0);
        //                    servo_rotate2(0);
        ////                    SOUP_LBL_count += 1;
        //                    break;
        //                case 1:
        //                    servo_rotate0(0);
        //                    servo_rotate2(0);
        ////                    SOUP_NOLBL_count += 1;
        //                    break;
        //                case 2:
        //                    servo_rotate0(0);
        //                    servo_rotate2(120);
        ////                    POPCAN_TAB_count += 1;
        //                    break;
        //                case 3:
        //                    servo_rotate0(0);
        //                    servo_rotate2(120);
        ////                    POPCAN_NOTAB_count += 1;
        //                    break;
        //                // add additional cases if needed
        //            }
        //        }
        //        INT2IF = 0;
        //    }
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
    else {
        while (1) {
            __lcd_home();
            printf("Invalid Action");
            __delay_1s();
        }
    }
    return;
}

void getRTC(void) {
    // Sets members in time array by reading from RTC module
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    I2C_Master_Stop(); //Stop condition

    //Read Current Time
    I2C_Master_Start();
    I2C_Master_Write(0b11010001); //7 bit RTC address + Read
    for (unsigned char j = 0; j < 0x06; j++) {
        time[j] = I2C_Master_Read(1);
    }
    time[6] = I2C_Master_Read(0); //Final Read without ack
    I2C_Master_Stop();
    //Reset RTC memory pointer
}

void standby(void) {
    getRTC();
    __lcd_home();

    printf("%02x/%02x %02x:%02x:%02x", time[5], time[4], time[2], time[1], time[0]); //Print date in YY/MM/DD
    //    __lcd_newline();
    //    
    //    printf("1:Start #:Logs  ");    

    //DEBUGGING LINES 
    __lcd_newline();
    printf("DEBUGPORTB: %d ", PORTB);
    return;

}

void set_time(void) {
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    for (char i = 0; i < 7; i++) {
        I2C_Master_Write(currentTime[i]);
    }
    I2C_Master_Stop(); //Stop condition
}

int dec_to_hex(int num) { //Convert decimal unsigned char to hexadecimal int
    int i = 0, quotient = num, temp, hexnum = 0;

    while (quotient != 0) {
        temp = quotient % 16;

        hexnum += temp * pow(10, i);

        quotient = quotient / 16;
        i += 1;
    }
    return hexnum;
}

void read_time(void) {
    //Reset RTC memory pointer 
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    I2C_Master_Stop(); //Stop condition

    //Read Current Time
    I2C_Master_Start();
    I2C_Master_Write(0b11010001); //7 bit RTC address + Read
    for (unsigned char i = 0; i < 0x06; i++) {
        time[i] = I2C_Master_Read(1); //Read with ack
    }
    time[6] = I2C_Master_Read(0); //Final Read without ack
    I2C_Master_Stop();
    return;
}

void can_count(void) {
    switch (can_display_position % 3) {
        case 0:
            __lcd_home();
            printf("Operation Summary");
            __lcd_newline();
            printf("TOTAL CAN: %d    ", TOTAL_CAN_count[1]);
            break;
        case 1:
            __lcd_home();
            printf("SOUP LBL: %d     ", SOUP_LBL_count[1]);
            __lcd_newline();
            printf("SOUP NOLBL: %d   ", SOUP_NOLBL_count[1]);
            break;
        case 2:
            __lcd_home();
            printf("POPCAN TAB: %d   ", POPCAN_TAB_count[1]);
            __lcd_newline();
            printf("POPCAN NOTAB: %d ", POPCAN_NOTAB_count[1]);
            break;

        default:
            while (1) {
                __lcd_home();
                printf("ERROR: %d", can_display_position);
            }
            break;
    }
    return;
}

void display_log(void) {
    log_position = log_position % 5;
    switch (log_position) {
        case 0:

            //TODOS: REPLACE NUMBERS WITH %d

            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("SOUP LBL:  %d     ", SOUP_LBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #1 Time:  %d ", LoggedTimes[0]);
            __lcd_newline();
            printf("SOUP NOLBL:  %d   ", SOUP_NOLBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("POPCAN TAB:  %d   ", POPCAN_TAB_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #1           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  %d ", POPCAN_NOTAB_count[log_position]);
            keyPressed();

            log_position = 1;
            break;

        case 1:
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("SOUP LBL:  %d     ", SOUP_LBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #2 Time:  %d ", LoggedTimes[1]);
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #2           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count[log_position]);
            keyPressed();

            log_position = 2;
            break;
        case 2:
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #3 Time:  %d ", LoggedTimes[2]);
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count[log_position]);
            keyPressed();
            log_position = 0;
            break; // Cycle back to start
        case 3:
            __lcd_home();
            printf("Run #4           ");
            __lcd_newline();
            printf("SOUP LBL:  4     ", SOUP_LBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #3 Time:  %d ", LoggedTimes[2]);
            __lcd_newline();
            printf("SOUP NOLBL:  4   ", SOUP_NOLBL_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #4           ");
            __lcd_newline();
            printf("POPCAN TAB:  4   ", POPCAN_TAB_count[log_position]);
            keyPressed();
            __lcd_home();
            printf("Run #3           ");
            __lcd_newline();
            printf("POPCAN NOTAB:  4 ", POPCAN_NOTAB_count[log_position]);
            keyPressed();
            log_position = 0;
            break; // Cycle back to start

        default:
            while (1) {
                __lcd_home();
                printf("ERROR: %d", log_position);
            }
            break;
    }
    return;
}

void can_time(void) {
    __lcd_home();
    printf("Total Operation  ");
    __lcd_newline();
    printf("Time: %d s       ", operation_time);
    return;
}

void operation(void) {
    //    __lcd_home();
    //    printf("Running..              ");  
    //    __lcd_newline();
    //    printf("PRESS 4 TO STOP  ");  

    read_time();
    start_time[1] = time[1];
    start_time[0] = time[0];
    __lcd_clear();


    printf("start: %02x %02x       ", start_time[1], start_time[0]);
    __delay_ms(500);
    int i;
    INT1IF = 0;
    start_minutes = start_time[1];
    start_seconds = start_time[0];
    time_diff = 0;

    begin_time = 60 * dec_to_hex(start_time[1]) + dec_to_hex(start_time[0]);

    __lcd_newline();
    printf("begin: %x       ", begin_time);

    while (1) {
        __delay_ms(1000);
        read_time();

        current_time[1] = time[1];
        current_time[0] = time[0];

        curr_time = 60 * dec_to_hex(current_time[1]) + dec_to_hex(current_time[0]);

        time_diff = (time[0] - 1);

        getRTC();
        __lcd_home();
        printf("ct: %x | %x %x   ", curr_time, time[1], time[0]);
        __lcd_newline();
        printf("bt: %x | %x      ", begin_time, time_diff);
        //printf("%02x/%02x %02x:%02x:%02x", time[5],time[4],time[2],time[1],time[0]);    //Print date in YY/MM/DD
        __lcd_newline();

        if (time_diff == 10) {
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

void operationend(void) {
    __lcd_newline();
    __lcd_newline();
    printf("Operation Done!  ");
    __delay_ms(1000);

    LATCbits.LATC0 = 0; //Stop motor
    INT0IE = 0; //Disable external interrupts
    INT2IE = 0;
    TMR0IE = 0; //Disable timer
    TMR0ON = 0;

    read_time();
    end_time[1] = time[1];
    end_time[0] = time[0];
    stime = 60 * dec_to_hex(start_time[1]) + dec_to_hex(start_time[0]);
    etime = 60 * dec_to_hex(end_time[1]) + dec_to_hex(end_time[0]);
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

void emergencystop(void) {
    di();
    LATA = 0x00; // output low
    LATB = 0x00; // output low
    LATC = 0x00; // output low
    LATD = 0x00; // output low
    LATE = 0x00; // output low

    __lcd_home();
    printf("EMERGENCY,STOPPED");
    while (1) {
    }
    return;
}

void servo_rotate2(int degree) {
    unsigned int i;
    unsigned int j;
    int duty = ((degree + 90)*5 / 90) + 10;
    for (i = 0; i < 50; i++) {
        LATCbits.LATC2 = 1;
        for (j = 0; j < duty; j++) __delay_us(100);
        LATCbits.LATC2 = 0;
        for (j = 0; j < (200 - duty); j++) __delay_us(100);
    }
    return;
}

void read_sensor(void) {
    return;
}

void read_ADC(void) {
    readADC(3);
    __lcd_home();

    if (high >= 4) {
        printf("voltage");
        __delay_1s();
        __delay_1s();
        __delay_1s();
        __delay_1s();
        high = 0;
    }

    if (ADRESH >= 3) {
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
    unsigned int set_time = 65535 - (int) ((float) time * 2000 / 256); // with oscillation at 8MHz, prescaler 256
    TMR0H = set_time >> 8;
    TMR0L = set_time & 0b11111111;
    TMR0ON = 1; // turn timer0 on
    TMR0IF = 0; // clear interrupt flag
    TMR0IE = 1; // enable timer0 interrupts

    // FSM stalls until countdown is completed
    //	FSM_enabled = FALSE;
    return;
}

void set_external_interrupt1(float time) {
    // set interrupt for external timer1 for time in ms to control servo
    unsigned int set_time = 65535 - (int) ((float) time * 2000 / 8); // with oscillation at 8MHz, prescaler 8
    TMR1H = set_time >> 8;
    TMR1L = set_time & 0b11111111;
    TMR1ON = 1; // turn timer0 on
    TMR1IF = 0; // clear interrupt flag
    TMR1IE = 1; // enable timer0 interrupts
    return;
}
//

void set_external_interrupt2(float time) {
    // set interrupt for external timer2 for time in ms to control wheel stepper
    unsigned int set_time = (int) ((float) time * 2000 / 256); // with oscillation 8MHz, prescaler 16, postscaler 16
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

void keyPressed(void) {
    while (PORTBbits.RB1 == 0) {
        // RB1 is the interrupt pin, so if there is no key pressed, RB1 will be 0
        // the PIC will wait and do nothing until a key press is signaled
    }
    keypress = (PORTB & 0xF0) >> 4; // Read the 4 bit character code
    while (PORTBbits.RB1 == 1) {
        // Wait until the key has been released
    }
    Nop(); //breakpoint b/c compiler optimizations
    return;
}


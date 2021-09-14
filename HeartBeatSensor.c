#include <xc.h>
#include <htc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program M

#define _XTAL_FREQ 6000000      // PIC frequency (Hz)

/* Define LCD I/O port */
#define LCD_RS RB3
#define LCD_RW RB2
#define LCD_EN RB1
#define LCD_DATA PORTD
#define LCD_DIR_RS TRISB3
#define LCD_DIR_RW TRISB2
#define LCD_DIR_EN TRISB1
#define LCD_DIR_DATA TRISD
#define LCD_STROBE() ((LCD_EN = 1), NOP(), (LCD_EN=0))

void calculate_heart_beat();
void __interrupt() ISR();
void ADC_Init();
uint16_t ADC_Read();
void lcd_write(unsigned char);
void lcd_clear();
void lcd_puts();
void lcd_goto(unsigned char);
void lcd_init();

float BPM = 0;                            // So nhip tim trong 1 phut
volatile uint8_t overflow_counter = 0;    // Bien dem so lan overflow cua Timer1

void calculate_heart_beat() { 
    uint16_t thresh = 600;                  // Nguong nhip tim du doan
    volatile bool Pulse = false;            // "true" khi dang co nhip tim
    float average_IBI = 0;                  // Thoi gian trung binh giua 2 nhip tim
    float runningTotal = 0;                 // Tong thoi gian lay mau
    volatile uint16_t adc_value = 0;        // Ket qua tu ADC
    volatile uint16_t miss_counter = 0;     // Dem thoi gian lap lai vong for ma k tim thay nhip tim
    
    for (int i = 0; i < 6; )    // Lay mau 5 lan (5 khoang thoi gian giua 2 nhip tim)
    {
        adc_value = ADC_Read();
        if (adc_value > thresh && Pulse == false) {     // Xac dinh suon len cua nhip tim
            Pulse = true;
            if (i == 0) {
                TMR1ON = 1;     // TMR1 Bat dau dem de lay mau
            }
            else if (i == 5) {      // Ket thuc lay mau
                TMR1ON = 0;
                runningTotal = 0.04369 * overflow_counter;      // Tinh toan tong thoi gian lay mau
                overflow_counter = 0;
                TMR1 = 0;
            }
            i++;
        }
        else if (adc_value < thresh && Pulse == true) {     // Xac dinh suon xuong cua nhip tim
            Pulse = false;
            miss_counter = 0;       // Reset neu da thay 1 nhip
        }
        else {      // K tim thay nhip tim
            miss_counter++;
            if (miss_counter > 40000) {     // Qua 5s ma k thay nhip tim 
                if (thresh > 500) {     // Giam nguong de de phong nhip tim yeu
                    thresh -= 200;
                }
                else {                  // Nguong da thap roi ma van k thay nhip tim thi thoi
                    break;
                }
                miss_counter = 0;   // Reset cho lan sau
            }
        }
    } 
    
    average_IBI = runningTotal / 5;     // Chia trung binh 5 mau       
    BPM = 60 / average_IBI;
}

void main(void) {
    // Config Interrupt cho Timer1
    TMR1IE = 1;                 // Timer1 Interrupt enable bit
    TMR1IF = 0;                 // Clear the Interrupt flag bit for timer1        
    PEIE = 1;                   // Peripherals Interrupts enable bit 
    GIE = 1;                    // Global Interrupt Enable bit
    
    // Config Timer1
    // Clear the Timer1 register to start counting from 0
    TMR1ON = 0;
    TMR1 = 0;                   
    // Clear the Timer1 clock select bit to choose local clock source
    TMR1CS = 0;                 
    // Prescaler ratio 1:1
    T1CKPS0 = 0;
    T1CKPS1 = 0;
   
    lcd_init(); // Khoi tao LCD
    ADC_Init(); // Khoi tao ADC
    
    lcd_puts("Press to Start");
    while(1){
        if (RB0 == 0) {     // Nhan nut de bat dau do
            lcd_clear();
            lcd_puts("Hold...");
            calculate_heart_beat();      
        
            /* Hien thi BPM */
            lcd_clear();
            char result[10];
            sprintf(result, "%d", (int)BPM);
            lcd_puts("BPM: ");
            lcd_puts(result);
            lcd_goto(0x40);
            lcd_puts("TRY AGAIN...");
        }
    }
}

/* Interrupt for timer */
void __interrupt() ISR(void){
    if(TMR1IF == 1){               // Check the flag bit 
        overflow_counter++;        // Tang bien dem 
        TMR1IF = 0;                // Clear interrupt bit for timer1
    }               
} 

void ADC_Init(){
    //------[There are 2 registers to configure ADCON0 and ADCON1]---------
    // ADCON0 = 0x41
    // Select clock option Fosc/8
    ADCS0 = 1;
    ADCS1 = 0;
    // Turn ADC on
    ADON = 1;
    
    //ADCON1 = 0x80
    // Result mode: Right justified
    ADFM = 1;
    // Select clock option Fosc/8
    ADCS2 = 0;
    // Configure all 8 channels are analog 
    PCFG0 = 0;
    PCFG1 = 0;
    PCFG2 = 0;
    PCFG3 = 0;
}

uint16_t ADC_Read(){
    // Write ADC__channel into register ADCON0
    CHS0 = 0;
    CHS1 = 0;
    CHS2 = 0;
    
    // Wait the Acquisition time 
    __delay_us(25);
    
    // Start A/D conversion
    GO_DONE = 1;
    
    // (Polling) Wait for the conversion to complete
    while(GO_DONE);
    
    // Read the ADC result ("right justified" mode)
    uint16_t result = ((ADRESH << 8) + ADRESL);
    return result;
}

/* Control LCD */
void lcd_write(unsigned char c)
{
    __delay_us(70);
    LCD_DATA = c;
    LCD_STROBE();
}
/* Clear and home the LCD */
void lcd_clear(void)
{
    LCD_RS = 0;
    lcd_write(0x1);
    __delay_ms(5);
}
/* Write a string of chars to the LCD */
void lcd_puts(const char * s)
{
    LCD_RS = 1; // write characters
    while(*s)
    lcd_write(*s++);
}
/* Go to the specified position */
void lcd_goto(unsigned char pos)
{
    LCD_RS = 0;
    lcd_write(0x80+pos);
}
/* Initialise LCD - put into 8 bit mode */
void lcd_init()
{
    // Set pins output
    LCD_DIR_RS = 0;
    LCD_DIR_RW = 0;
    LCD_DIR_EN = 0;
    LCD_DIR_DATA = 0x00;
    // Clear output
    LCD_RS = 0;
    LCD_EN = 0;
    LCD_RW = 0;
    LCD_DATA = 0x00;
    
    // wait 15mSec after power applied,
    __delay_ms(400);
    lcd_write(0x3C); // Function set;
    __delay_ms(10);
    lcd_write(0x0F); // Turn on display
    __delay_ms(10);
    lcd_write(0x01); // Clear display
    __delay_ms(10);
    lcd_write(0x02); // Go to home
    __delay_ms(10);
    lcd_write(0x06); // Entry mode set
    __delay_ms(10);
    lcd_goto(0); // Ve dau dong
}
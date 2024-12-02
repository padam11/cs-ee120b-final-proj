#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "spiAVR.h"
#include "irAVR.h"
#include <avr/pgmspace.h>


#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_REST 0

const uint16_t song[] PROGMEM = { //later add the length for each note
    NOTE_C4,
    NOTE_D4, 
    NOTE_E4, 
    NOTE_F4, 
    NOTE_G4,
    NOTE_A4,
    NOTE_B4,
    NOTE_C5,
    NOTE_REST
};

#define SONG_LENGTH (sizeof(song) / sizeof(song[0])) // Number of elements

void playNote(uint16_t frequency) {
    if (frequency == NOTE_REST) {
        // Turn off buzzer for a rest
        TCCR1B &= ~(1 << CS10) & ~(1 << CS11) & ~(1 << CS12);
        PORTB &= ~(1 << PB1); 
        return;
    }

    // Calculate OCR0A for the desired frequency
    uint16_t ocrValue = (F_CPU / (2 * 8 * frequency)) - 1; // Prescaler = 8
    OCR1A = ocrValue;
    

    // Enable PWM on PORTD6
    PORTD = SetBit(PORTD, 6, 1);
    TCCR1B = (TCCR1B & 0xF8) | 0x02; // Set prescaler to 8
}

void playSong() {
    for (uint16_t i = 0; i < SONG_LENGTH; i ++) {
        //progmem
        uint16_t note = pgm_read_word(&song[i]);
        //uint16_t duration = pgm_read_word(&song[i + 1]);

        //play note
        playNote(note);
        //serial_println(note);
        //delay duration
        _delay_ms(500);

        //turn off buzzer
        playNote(NOTE_REST);
        _delay_ms(50); //pause between note
    }
}


//TODO: declare variables for cross-task communication

// TODO: Change this depending on which exercise you are doing.
// Exercise 1: 3 tasks
// Exercise 2: 5 tasks
// Exercise 3: 7 tasks
#define NUM_TASKS 7


//Task struct for concurrent synchSMs implmentations
typedef struct _task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;


//TODO: Define Periods for each task
// e.g. const unsined long TASK1_PERIOD = <PERIOD>
const unsigned long GCD_PERIOD = /* TODO: Calulate GCD of tasks */ 1;

task tasks[NUM_TASKS]; // declared task array with NUM_TASKS amount of tasks

//TODO: Define, for each task:
// (1) enums and
// (2) tick functions

void TimerISR() {
    
    //TODO: sample inputs here

	for ( unsigned int i = 0; i < NUM_TASKS; i++ ) {                   // Iterate through each task in the task array
		if ( tasks[i].elapsedTime == tasks[i].period ) {           // Check if the task is ready to tick
			tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
			tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
		}
		tasks[i].elapsedTime += GCD_PERIOD;                        // Increment the elapsed time by GCD_PERIOD
	}
}

void HardwareReset() {
    PORTB = SetBit(PORTB, PB0, 0);
    _delay_ms(200);
    PORTB = SetBit(PORTB, PB0, 1); 
    _delay_ms(200);
}

void Send_Command(uint8_t command) {
    PORTB = SetBit(PORTB, PB1, 0); // A0 = 0 (command mode)
    PORTB = SetBit(PORTB, PB2, 0); // CS = 0 (enable lcd)
    SPI_SEND(command);     //SPI
    PORTB = SetBit(PORTB, PB2, 1); // CS = 1 (diable lcd)
}

void Send_Data(uint8_t data) {
    PORTB = SetBit(PORTB, PB1, 1); // A0 = 1 (data mode)
    PORTB = SetBit(PORTB, PB2, 0); // CS = 0 (enable lcd)
    SPI_SEND(data);        // SPI
    PORTB = SetBit(PORTB, PB2, 1); // CS = 1 (disable lcd)
}

void ST7735_Init() {
    HardwareReset();         // Perform hardware reset

    Send_Command(0x01);      // SWRESET (Software Reset)
    _delay_ms(150);

    Send_Command(0x11);      // SLPOUT (Sleep Out)
    _delay_ms(200);

    Send_Command(0x3A);      // COLMOD (Color Mode)
    Send_Data(0x06);         
    _delay_ms(10);

    Send_Command(0x29);      // DISPON (Display On)
    _delay_ms(200);
}

void SetColumn(uint16_t start, uint16_t end) {
    Send_Command(0x2A); // CASET
    Send_Data(0x00); 
    Send_Data(start);        
    Send_Data(0x00);   
    Send_Data(end);          
}

void SetRow(uint16_t start, uint16_t end) {
    Send_Command(0x2B); // RASET
    Send_Data(0x00); 
    Send_Data(start);        
    Send_Data(0x00);   
    Send_Data(end);          
}

void WriteToMemory(uint16_t color, uint16_t count) {
    Send_Command(0x2C); // RAMWR
    for (uint16_t i = 0; i < count; i++) {
        Send_Data((color >> 8) & 0xFF); 
        Send_Data(color & 0xFF);        
    }
}

void DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    SetColumn(x, x + width - 1); // Set column range
    SetRow(y, y + height - 1);   //set row range
    WriteToMemory(color, width * height); // Fill with color
}

const uint16_t sprite[5][5] = {
    {0x07E0, 0x07E0, 0x07E0, 0x07E0, 0x07E0}, //greenborder
    {0x07E0, 0xF800, 0xF800, 0xF800, 0x07E0}, //green edges and red center
    {0x07E0, 0xF800, 0xF800, 0xF800, 0x07E0},
    {0x07E0, 0xF800, 0xF800, 0xF800, 0x07E0},
    {0x07E0, 0x07E0, 0x07E0, 0x07E0, 0x07E0}  //greenborder
};

void DrawSprite(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t sprite[5][5], uint16_t color) {
    DrawRectangle(x, y, x + width - 1, y + height - 1, color); 
    Send_Command(0x2C); 

    // Loop through the sprite data
    for (uint16_t row = 0; row < height; row++) {
        for (uint16_t col = 0; col < width; col++) {
            uint16_t color = sprite[row][col];
            Send_Data((color >> 8) & 0xFF); //high
            Send_Data(color & 0xFF);       //low
        }
    }
}

int main(void) {
    //TODO: initialize all your inputs and ouputs

    ADC_init();   // initializes ADC
    SPI_INIT(); //initializes SPI for LCD communication
    
    DDRB = 0xFF; PORTB = 0x00;
    DDRD = 0xFF; PORTD = 0x00;

    TCCR1A = (1 << COM1A0); 
    TCCR1B = (1 << WGM12);  
    DDRB |= (1 << PB1);    
    //WGM11, WGM12, WGM13 set timer to fast pwm mode

    ICR1 = 39999; //20ms pwm period
    //DDRD |= (1 << PD6); // Set D6 as output
    //TODO: Initialize tasks here
    // e.g. tasks[0].period = TASK1_PERIOD
    // tasks[0].state = ...
    // tasks[0].elapsedTime = ...
    // tasks[0].TickFct = &task1_tick_function;
    ST7735_Init();
    

    DrawRectangle(10, 20, 50, 50, 0xF800); // RGB565 for red color
    DrawSprite(10, 10, 5, 5, sprite, 0xF800);
    playSong();
    TimerSet(GCD_PERIOD);
    TimerOn();

    while (1) {}

    return 0;
}

#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "spiAVR.h"
#include "irAVR.h"
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <serialATmega.h>


#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_REST 0

#define MAX_DOTS 10

const uint16_t song[] PROGMEM = {
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
#define NUM_TASKS 4


//Task struct for concurrent synchSMs implmentations
typedef struct _task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t dx;
    uint8_t dy;
    uint8_t radius;
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    uint8_t bounce_left;
    uint8_t bounce_right;
    uint8_t bounce_top;
    uint8_t bounce_bottom;
    
} Dot;

typedef struct {
    uint8_t start_col;
    uint8_t end_col;
    uint8_t start_row;
    uint8_t end_row;
} Wall;

#define MAX_WALLS 10
Wall walls[MAX_WALLS];
uint8_t num_walls = 0;


//TODO: Define Periods for each task
// e.g. const unsined long TASK1_PERIOD = <PERIOD>
const unsigned long GCD_PERIOD = /* TODO: Calulate GCD of tasks */ 1; //make 0 for testing
const unsigned long JOYSTICK_PERIOD = 3;
const unsigned long SCREEN_PERIOD = 1;
const unsigned long GREENDOT_PERIOD =  5;
const unsigned long BUTTON_PERIOD = 100;

task tasks[NUM_TASKS]; // declared task array with NUM_TASKS amount of tasks
Dot dots[MAX_DOTS]; //array of dots
uint8_t numDots = 0; //number of active dots

//TODO: Define, for each task:
// (1) enums and
// (2) tick functions

enum Joystick_States {JOYSTICK_IDLE, JOYSTICK_READ };
enum Screen_States {SCREEN_UPDATE};
enum DotTask_States { DOTS_MOVE };
enum ButtonHandler_States { OFF, PRESS, ON, PRESS2};
enum Game_States {OFF1, HOME_SCREEN, GAME1, GAME2};

unsigned char systemOn = 0;
unsigned char gameOn = 0;
unsigned char gameWon = 0;
unsigned int numFails = 0;
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

void black_screen()
{
    Send_Command(0x2A);
    Send_Data(0);
    Send_Data(1);
    Send_Data(0);
    Send_Data(130);

    Send_Command(0x2B);
    Send_Data(0);
    Send_Data(1);
    Send_Data(0);
    Send_Data(130);

    Send_Command(0x2C);
    for (unsigned short i = 0; i < 16900; ++i)
    {
        Send_Data(0);
        Send_Data(0);
        Send_Data(0);
    }
}

void fill_screen(uint8_t start_col, uint8_t end_col, uint8_t start_row, uint8_t end_row, uint8_t red, uint8_t green, uint8_t blue)
{
    Send_Command(0x2A);
    Send_Data(0);
    Send_Data(start_col);
    Send_Data(0);
    Send_Data(end_col);

    Send_Command(0x2B);
    Send_Data(0);
    Send_Data(start_row);
    Send_Data(0);
    Send_Data(end_row);

    Send_Command(0x2C);
    for (uint16_t row = start_row; row <= end_row; ++row) {
        for (uint16_t col = start_col; col <= end_col; ++col) {
            Send_Data(red);   // Red value
            Send_Data(green); // Green value
            Send_Data(blue);  // Blue value
        }
    }
}
//128x128
void create_sprite(uint8_t cx, uint8_t cy, uint8_t r, uint8_t blue, uint8_t green, uint8_t red)
{
    
    // Define the bounding box for the circle
    uint8_t x_start = (cx > r) ? (cx - r) : 0;
    uint8_t x_end = (cx + r < 128) ? (cx + r) : 127;
    uint8_t y_start = (cy > r) ? (cy - r) : 0;
    uint8_t y_end = (cy + r < 128) ? (cy + r) : 127;

    // Set the column address (x range)
    Send_Command(0x2A); 
    Send_Data(0);      
    Send_Data(x_start); 
    Send_Data(0);      
    Send_Data(x_end);   

    // Set the row address (y range)
    Send_Command(0x2B); 
    Send_Data(0);       
    Send_Data(y_start); 
    Send_Data(0);       
    Send_Data(y_end);   

    // Begin writing pixel data
    Send_Command(0x2C); // Memory write

    for (uint8_t y = y_start; y <= y_end; ++y) //square/rectangular dimensions
    {
        for (uint8_t x = x_start; x <= x_end; ++x)
        {
            // Check if the pixel is within the circle
            uint16_t dx = x - cx;
            uint16_t dy = y - cy;
            uint16_t distance_squared = dx * dx + dy * dy;

            if (distance_squared <= r * r)
            {
                // Inside the circle: Set a color
                Send_Data(blue); //blue
                Send_Data(green); //green
                Send_Data(red); // red
            }
            else
            {
                //outside the circle, but within rectangular/square dimensions
                Send_Data(0);
                Send_Data(0);
                Send_Data(0);
            }
        }
    }
}

volatile uint8_t cx = 6; //moveable character
volatile uint8_t cy = 100;
volatile uint8_t radius = 5;

volatile uint8_t green_x = 64; //evil sprite
volatile uint8_t green_y = 64;
volatile uint8_t green_dx = -1;
volatile uint8_t green_radius = 5;

int Screen_Tick(int state)
{
    switch (state) {
        case SCREEN_UPDATE:
            
            create_sprite(cx, cy, radius, 0, 0, 255); //moveable sprite
            //setting create sprite with cx and cy will only create a moveable sprite
            //create_sprite(green_x - green_dx, green_y, green_radius, 0, 0, 0); 
            //create_sprite(green_x, green_y, green_radius, 0, 255, 0); //evil sprite
            //no_trace(cx, cy, radius);
            //state = SCREEN_UPDATE;
            for (uint8_t i = 0; i < numDots; i++) //create each evil dot sprite
            {
                create_sprite(dots[i].x - dots[i].dx, dots[i].y - dots[i].dy, 
                            dots[i].radius, 0, 0, 0);

                create_sprite(dots[i].x, dots[i].y, dots[i].radius, dots[i].red,
                dots[i].green, dots[i].blue); 
            }
            break;
        
        default:
            state = SCREEN_UPDATE;
            break;
    }
    return state;
}

uint8_t check_collision(uint8_t cx, uint8_t cy, uint8_t radius) {
    for (uint8_t i = 0; i < num_walls; i++) {
        if (cx + radius >= walls[i].start_col && cx - radius <= walls[i].end_col &&
            cy + radius >= walls[i].start_row && cy - radius <= walls[i].end_row) {
            return 1; // Collision detected
        }
    }
    return 0; // No collision
}

int Joystick_Tick(int state)
{
    static uint16_t adc_x, adc_y;
    uint8_t new_cx = cx;
    uint8_t new_cy = cy;

    switch (state) {
        case JOYSTICK_IDLE:
            if (gameOn)
            {
                state = JOYSTICK_READ;
            }
            break;
        case JOYSTICK_READ:
            adc_y = ADC_read(0);
            adc_x = ADC_read(1);

            if (adc_x > 550 && cx < 127)
            {
                new_cx++; //move right
            }
            if (adc_x < 470 && cx > 0)
            {
                new_cx--; //move left
            }
            if (adc_y > 550 && cy > 0)
            {
                new_cy--; //move up
            }
            if (adc_y < 470 && cy < 127)
            {
                new_cy++; //move down
            }
            if (!check_collision(new_cx, new_cy, radius))
            {
                cx = new_cx;
                cy = new_cy; //applies new coordinates to original variable if no wall
            }
            state = JOYSTICK_IDLE;
            break;
        default:
            state = JOYSTICK_IDLE;
            break;
    }
    return state;
}


void init_dot(uint8_t index, uint8_t x, uint8_t y, int8_t dx, int8_t dy, 
              uint8_t radius, uint8_t red, uint8_t green, uint8_t blue,
              uint8_t bounce_left, uint8_t bounce_right, uint8_t bounce_top,
              uint8_t bounce_bottom)
    {
        if (index >= MAX_DOTS) return; // Prevent out-of-bounds access

        dots[index].x = x;
        dots[index].y = y;
        dots[index].dx = dx;
        dots[index].dy = dy;
        dots[index].radius = radius;
        dots[index].red = red;
        dots[index].green = green;
        dots[index].blue = blue;

        dots[index].bounce_left = bounce_left;
        dots[index].bounce_right = bounce_right;
        dots[index].bounce_top = bounce_top;
        dots[index].bounce_bottom = bounce_bottom;
        
    }

void add_wall(uint8_t start_col, uint8_t end_col, uint8_t start_row, uint8_t end_row)
{
    if (num_walls >= MAX_WALLS) return;
    walls[num_walls].start_col = start_col;
    walls[num_walls].end_col = end_col;
    walls[num_walls].start_row = start_row;
    walls[num_walls].end_row = end_row;
    num_walls++;
}



int Dots_Tick(int state)
{
    switch(state) {
        case DOTS_MOVE:
            if (gameOn) {
                for (uint8_t i = 0; i < numDots; i++)
                {
                    dots[i].x += dots[i].dx;
                    dots[i].y += dots[i].dy; //update positions
                    
                    //check horizontal wall collisions
                   
                    if (dots[i].x < dots[i].bounce_left + dots[i].radius) {
                        
                        dots[i].dx = abs(dots[i].dx); // Bounce right. change to abs var instead of 1 because we can also use it to modify the speed too, important when theres a ton of dots.
                    }
                    if (dots[i].x > dots[i].bounce_right - dots[i].radius) {
                        dots[i].dx = -abs(dots[i].dx); // Bounce left
                    }

                    // Check vertical boundaries
                    if (dots[i].y <= dots[i].bounce_top + dots[i].radius) {
                        dots[i].dy = abs(dots[i].dy); // Bounce down
                    }
                    if (dots[i].y >= dots[i].bounce_bottom - dots[i].radius) {
                        dots[i].dy = -abs(dots[i].dy); // Bounce up
                    }
                }
            }
            break;
        default:
            state = DOTS_MOVE;
            break;
            
    }
    return state;
}


int ButtonHandler_Tick(int state)
{
    switch(state)
    {
        case OFF:
            if (GetBit(PINC, PC4) == 1)
            {
                state = PRESS;
                
            }
            break;
        case PRESS:
            if (GetBit(PINC, PC4) == 0)
            {
                state = ON;
                systemOn = 1;
            }
            break;
        case ON:
            if (GetBit(PINC, PC4) == 1)
            {
                state = PRESS2;
            }
            break;
        case PRESS2:
            if (GetBit(PINC, PC4) == 0)
            {
                state = OFF;
                systemOn = 0;
            }
            break;
        default:
            state = OFF;
            break;
    }
    return state;
}

int Game_Tick(int state)
{
    switch(state)
    {
        case OFF1:
            state = HOME_SCREEN;
            ST7735_Init();
            
            
            break;
        case HOME_SCREEN:
            if (systemOn)
            {
                state = GAME1;
                gameOn = 1;
                cx = 6;
                cy = 100;
                init_dot(0, 64, 64, -1, 0, 5, 0, 255, 0, 31, 100, 0, 115);
                init_dot(1, 82, 82, 1, 0, 5, 0, 255, 0, 31, 100, 0, 115); 
                fill_screen(16, 31, 0, 115, 0, 0, 250); //red
                fill_screen(50, 130, 115, 130, 0, 0, 250); //red
                fill_screen(100, 115, 15, 116, 0, 0, 250); //red
                fill_screen(16, 49, 116, 130, 0, 0, 0); //black
                fill_screen(0, 15, 0, 130, 0, 0, 0); //black
                fill_screen(31, 100, 0, 115, 0, 0, 0); //black
                fill_screen(101, 115, 0, 14, 0, 0, 0); //black
                fill_screen(116, 130, 0, 115, 0, 0, 0); //black
                add_wall(16, 31, 0, 115);
                add_wall(50, 130, 115, 130);
                add_wall(100, 114, 15, 116); //red are the only walls.
                numDots = 2;
                
            }
            
            //add title
            break;
        case GAME1: //SCREEN_UPDATE
            
            create_sprite(cx, cy, radius, 0, 0, 255); //moveable sprite
            
            //setting create sprite with cx and cy will only create a moveable sprite
            //create_sprite(green_x - green_dx, green_y, green_radius, 0, 0, 0); 
            //create_sprite(green_x, green_y, green_radius, 0, 255, 0); //evil sprite
            for (uint8_t i = 0; i < numDots; i++) //create each evil dot sprite
            {
                create_sprite(dots[i].x - dots[i].dx, dots[i].y - dots[i].dy, 
                            dots[i].radius, 0, 0, 0);

                create_sprite(dots[i].x, dots[i].y, dots[i].radius, dots[i].red,
                dots[i].green, dots[i].blue); 

                uint16_t dx = cx - dots[i].x;
                uint16_t dy = cy - dots[i].y;
                uint16_t distance_squared = dx * dx + dy * dy;

                if (distance_squared <= ((dots[i].radius+5) * (dots[i].radius+5)))
                {
                    gameOn = 0;
                    state = HOME_SCREEN;
                    numFails++;
                }

                /*if (GetBit(PINC, PC2) == 1)
                {
                    gameOn = 0;
                    state = HOME_SCREEN;
                }*/
                
            }
            if (cx >= 114 && cy <= 14)
                {
                    state = GAME2;
                    //gameOn = 1;
                    num_walls = 0;
                    cx = 64;
                    cy = 125;
                    init_dot(0, 64, 77, -2, 0, 5, 0, 255, 0, 0, 130, 0, 115);
                    init_dot(1, 64, 92, 2, 0, 5, 0, 255, 0, 0, 130, 0, 115);
                    init_dot(2, 64, 62, 2, 0, 5, 0, 255, 0, 0, 130, 0, 115); //vert doesnt matter if dy is 0 anyway
                    init_dot(3, 64, 47, -2, 0, 5, 0, 255, 0, 0, 130, 0, 115); //more dots = more speed
                    fill_screen(0, 49, 100, 115, 0, 0, 250); //red
                    fill_screen(80, 130, 100, 115, 0, 0, 250); //red
                    fill_screen(0, 49, 15, 30, 0, 0, 250); //red
                    fill_screen(80, 130, 15, 30, 0, 0, 250); //red
                    fill_screen(0, 130, 0, 15, 0, 0, 0); //black
                    fill_screen(49, 80, 15, 30, 0, 0, 0); //black
                    fill_screen(0, 130, 30, 100, 0, 0, 0); //black
                    fill_screen(49, 80, 100, 115, 0, 0, 0); //black
                    fill_screen(0, 130, 115, 130, 0, 0, 0);
                    add_wall(0, 49, 100, 115);
                    add_wall(80, 130, 100, 115);
                    add_wall(0, 49, 15, 30); //red are the only walls.
                    add_wall(80, 130, 15, 30);
                    
                    numDots = 4;

                }
            if (!systemOn)
            {
                state = OFF1;
                ST7735_Init();
                
            }
            break;
            //game related work, sprites, and check if game has been won
        case GAME2:
            //HardwareReset();
            create_sprite(cx, cy, radius, 0, 0, 255); //moveable sprite
            
            for (uint8_t i = 0; i < numDots; i++) //create each evil dot sprite
            {
                create_sprite(dots[i].x - dots[i].dx, dots[i].y - dots[i].dy, 
                            dots[i].radius, 0, 0, 0);

                create_sprite(dots[i].x, dots[i].y, dots[i].radius, dots[i].red,
                dots[i].green, dots[i].blue); 

                uint16_t dx = cx - dots[i].x;
                uint16_t dy = cy - dots[i].y;
                uint16_t distance_squared = dx * dx + dy * dy;

                if (distance_squared <= ((dots[i].radius+5) * (dots[i].radius+5)))
                {
                    gameOn = 0;
                    state = HOME_SCREEN;
                    numFails++;
                }

                /*if (GetBit(PINC, PC2) == 1)
                {
                    gameOn = 0;
                    state = HOME_SCREEN;
                }*/
                
            }
            if (cx >= 114 && cy <= 14)
                {
                    state = GAME2;
                    num_walls = 0;
                    fill_screen(0, 130, 0, 130, 0, 0, 0);
                    init_dot(0, 64, 64, -1, 0, 5, 0, 255, 0, 31, 100, 0, 115);
                    init_dot(1, 82, 82, 1, 0, 5, 0, 255, 0, 31, 100, 0, 115);
                    numDots = 2;

                }
            if (!systemOn)
            {
                state = OFF1;
                ST7735_Init();
                
            }
            
            break;
        default:
            state = OFF1;
            break;
        
    }
    return state;
}

int main(void) {
    //TODO: initialize all your inputs and ouputs

    ADC_init();   // initializes ADC
    SPI_INIT(); //initializes SPI for LCD communication
    
    DDRB = 0xFF; PORTB = 0x00;
    DDRD = 0xFF; PORTD = 0x00;
    serial_init(9600);
    

    //TCCR1A = (1 << COM1A0); 
    //TCCR1B = (1 << WGM12);  
    //DDRB |= (1 << PB1);    
    //WGM11, WGM12, WGM13 set timer to fast pwm mode
    
    ICR1 = 39999; //20ms pwm period
    //DDRD |= (1 << PD6); // Set D6 as output
    //TODO: Initialize tasks here
    // e.g. tasks[0].period = TASK1_PERIOD
    // tasks[0].state = ...
    // tasks[0].elapsedTime = ...
    // tasks[0].TickFct = &task1_tick_function;
    //ST7735_Init();
    

    //DrawRectangle(10, 20, 50, 50, 0xF800); // RGB565 for red color
    //DrawSprite(10, 10, 5, 5, sprite, 0xF800);
    //black_screen();
    //create_sprite(64, 64, 30);
    //playSong();

    
    tasks[0].state = JOYSTICK_IDLE;
    tasks[0].period = JOYSTICK_PERIOD;
    tasks[0].elapsedTime = 0;
    tasks[0].TickFct = &Joystick_Tick;

    /*tasks[1].state = SCREEN_UPDATE;
    tasks[1].period = SCREEN_PERIOD;
    tasks[1].elapsedTime = 0;
    tasks[1].TickFct = &Screen_Tick;*/

    tasks[1].state = OFF1;
    tasks[1].period = SCREEN_PERIOD;
    tasks[1].elapsedTime = 0;
    tasks[1].TickFct = &Game_Tick;


    tasks[2].state = DOTS_MOVE;
    tasks[2].period = GREENDOT_PERIOD;
    tasks[2].elapsedTime = 0;
    tasks[2].TickFct = &Dots_Tick;

    tasks[3].state = OFF;
    tasks[3].period = BUTTON_PERIOD;
    tasks[3].elapsedTime = 0;
    tasks[3].TickFct = &ButtonHandler_Tick;

    TimerSet(GCD_PERIOD);
    TimerOn();

    while (1) {}

    return 0;
}

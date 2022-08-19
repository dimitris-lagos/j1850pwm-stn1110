/* Button debounce library for the pins 2(INT0) and 3(INT1) of the Arduino.
 * Should be used in conjunction with 1kOhm in series and 100nF cap to ground.
 * It is intented this library to expand to all the PCINT pins of the ATMega328p.
 * For the time being works only with those 2 pins.
 * Difference from the rest of the libraries i saw is that, this library doesn't
 * need to have a function inside loop() to check every once in a while if a button is pressed.
 * Uses callback functions which are called when a pin state change interrupt is fired. 
 */

#ifndef Debouncy_h
#define Debouncy_h
#include "Arduino.h" 

#ifndef DEBOUNCE_TIME
#define DEBOUNCE_TIME 25
#endif

#ifndef PRESS_DELAY
#define PRESS_DELAY 100
#endif

#define INT0PIN 2
#define INT1PIN 3
/* Callback function type definition */
typedef void (*voidFuncPtr)(unsigned int);

/* Library class definition */
class Debouncy
{
private:
    static void risingInterruptINT0();
    static void fallingInterruptINT0();
    static void fallingInterruptINT1();
    static void risingInterruptINT1();
public:
    Debouncy();
    ~Debouncy();

    void setupDebouncedButton(int arduinoButtonPin, void (*callbackFunction)(unsigned int));
};

extern Debouncy Debouncy_Buttons;
#endif
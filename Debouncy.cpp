#include "Debouncy.h"

static void nothing(unsigned int) {
}
/* Qualified volatile because the values of intFunc change inside ISRs.
 * Volatile informs the compiler to store the variable inside the stack
 * of the SRAM. Declared static so it can be statically accessed from the
 * class functions.
 */
static volatile voidFuncPtr intFunc[] = {nothing, nothing};
unsigned long timeDiff[2] = {0,0};

Debouncy::Debouncy()
{
}

Debouncy::~Debouncy()
{
}
/*
 * void (*callbackFunction)(unsigned int) : void function with unsigned int parameter
 */
void Debouncy::setupDebouncedButton(int arduinoButtonPin, void (*callbackFunction)(unsigned int)){
    if(arduinoButtonPin == INT0PIN ){
        pinMode(INT0PIN, INPUT_PULLUP); 
        attachInterrupt(digitalPinToInterrupt(INT0PIN), fallingInterruptINT0, FALLING); 
        intFunc[0] = callbackFunction;

    }else if( arduinoButtonPin == INT1PIN ){
        pinMode(INT1PIN, INPUT_PULLUP); 
        attachInterrupt(digitalPinToInterrupt(INT1PIN), fallingInterruptINT1, FALLING); 
        intFunc[1] = callbackFunction;

    }else{
        //todo: implement non INT0/INT1 buttons
    }
}

void Debouncy::risingInterruptINT0(){
    if(timeDiff[0] > 0){
        unsigned int timePassed = millis() - timeDiff[0];
        if(timePassed  > DEBOUNCE_TIME){
            intFunc[0](timePassed);
        }
    }
    timeDiff[0] = 0;
    attachInterrupt(digitalPinToInterrupt(INT0PIN), fallingInterruptINT0, FALLING);
}

void Debouncy::risingInterruptINT1(){
    if(timeDiff[1] > 0){
        unsigned int timePassed = millis() - timeDiff[1];
        if(timePassed  > DEBOUNCE_TIME){
            intFunc[1](timePassed);
        }
    }
    timeDiff[1] = 0;
    attachInterrupt(digitalPinToInterrupt(INT1PIN), fallingInterruptINT1, FALLING);
    
}

void Debouncy::fallingInterruptINT0(){
    if(millis() - timeDiff[0] >  (unsigned long) PRESS_DELAY){
        timeDiff[0] = millis();
    }
    attachInterrupt(digitalPinToInterrupt(INT0PIN), risingInterruptINT0, RISING);
}

void Debouncy::fallingInterruptINT1(){
    if(millis() - timeDiff[1] >  (unsigned long) PRESS_DELAY){
        timeDiff[1] = millis();
    }
    attachInterrupt(digitalPinToInterrupt(INT1PIN), risingInterruptINT1, RISING);
}


 /* Η διασύνδεση:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 19
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 14
 * LCD D6 pin to digital pin 15
 * LCD D7 pin to digital pin 16
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 * */

#define LCD_NUMBER_OF_ROWS  2 //2 γραμμές οθόνης
#define LCD_NUMBER_OF_COLUMNS  16 //16 στοιχείων
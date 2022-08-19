#include <SoftwareSerial.h> //Χρήση της βιβλιοθήκης που δίνει την δυνατότητα για σειριακή σύνδεση με υλοποίση επιπέδου λογισμικού
#include <avr/wdt.h>        //Χρήση της βιβλιοθήκης υπεύθυνης για την λειτουργία του watchdog timer και συναρτήσεων καθυστέρησης
#include <avr/pgmspace.h>   //Χρήση της EEPROM του ATmega328 για την αποθήκευση των String, ώστε να εξοικονομηθεί χώρος runtime στην sram
#include <LiquidCrystal.h>  // Βιβλιοθήκη που διαχειρίζεται την διασύνδεση με την οθόνη LCD
#include "global.h"
#include "OBD_Interface.h"
#include "LCD_Interface.h"
#include "Debouncy.h"

#define RED_LED 19
#define VENTILATOR_PIN 9
#define BUTTON_1_PIN 2
#define BUTTON_2_PIN 3

/* Η διασύνδεση:
Button Digital Pin 16
LED    Digital Pin 19
Mosfet Digital Pin 9
*/
/* Τμήμα δήλωσης global μεταβλητών */

Debouncy Debouncy_Buttons;
int counter = 0;
float previous_cycle_sterm = 0.00;
float total_cons = 0.00000;// Total consuumption of the vehicle for the current session
float temp_cons = 0.00000;
float real_time_cons = 0.00000;
int menu = 0;
boolean dtc = false;

int threshold = 95;
int temp_thresh = 95;
int analogPin = A3; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V

/* Αρχικοποίηση βιβλιοθήκης που διαχειρίζεται την επικοινωνία με την οθόνη,
 με τους ακροδέκτες του Arduino που συνδέονται στην οθόνη  */
LiquidCrystal lcd(12, 13, 5, 14, 15, 16); // LiquidCrystal(rs, enable, d4, d5, d6, d7)

/* Set timer1 interrupt at 1Hz */
void setup_1hz_timer_interrupt(){
  TCCR1A = 0;                                                  // set entire TCCR1A register to 0
  TCCR1B = 0;                                                  // same for TCCR1B
  TCNT1 = 0;                                                   // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}


/* Setup the i/o pin that drives the mosfet that turns on and off the fridge ventilator */
void setup_ventilator_mosfet_driver_pin(){
  pinMode(VENTILATOR_PIN, OUTPUT);   //Ορισμός ακροδέκτη 9 ως έξοδο ventilator mosfet
  digitalWrite(VENTILATOR_PIN, LOW); // ventilator off
}

/* Η συνάρτηση που αρχικοποιεί τις βασικές ρυθμίσεις για την διασύνδεση
 των περιφερειακών και των λειτουργιών του προγράμματος */
void setup()
{
  Debouncy_Buttons.setupDebouncedButton(BUTTON_1_PIN, button1);
  Debouncy_Buttons.setupDebouncedButton(BUTTON_2_PIN, button2);
  setup_ventilator_mosfet_driver_pin();
  init_STN_Serial_Interface();
  // set up the LCD's number of columns and rows:
  lcd.begin(LCD_NUMBER_OF_COLUMNS, LCD_NUMBER_OF_ROWS); //Ορισμός γραμμών και στηλών της οθόνης(2x16)
  pinMode(RED_LED, OUTPUT);                                  //Ορισμός ακροδέκτη 19 ως έξοδο(LED 1)
  setup_1hz_timer_interrupt();

  lcd.clear();
  digitalWrite(RED_LED, HIGH); // LED on
  delay(1000);            // Καθυστέρηση 1sec
  lcd.setCursor(0, 0);    /* μετακίνηση του κέρσορα στην πρώτη γραμμή και πρώτη στήλη της οθόνης */
  lcd.write(WELLCOME_TEXT); /* Γράψε στην οθόνη το κείμενο εισαγωγής*/
  digitalWrite(RED_LED, LOW);  // LED off
  delay(1000);
  wait_for_STN_ready();
  setup_STN_communication_properties();
  menu = 0;
  send_init_page_to_lcd();


  sei();       //ενεργοποίηση της εξυπηρέτησης των διακοπών
}

void send_init_page_to_lcd(){
  lcd.setCursor(0, 1);
  float fuel_lvl = read_fuel_lvl();
  lcd.print(fuel_lvl, 1);
  lcd.print(" %F  ");
  lcd.print(calculate_fuel_in_liters(fuel_lvl), 1);
  lcd.print(" L");
  delay(5000); //περίμενε 5"
}

ISR(TIMER1_COMPA_vect)
{
  if (counter == 0)
  {
    counter = 1;
  }
  total_cons += (temp_cons / (float)counter);
  temp_cons = 0;
  counter = 0;
}



void loop()
{
  read_temp_threshold();
  int coolant_temp = read_coolant_temperature();//Διάβασε την θερμοκρασία του οχήματος
  float fuel_consumption = calculate_fuel_consumption();
  float milage = calculate_milage(fuel_consumption);
  if(dtc){
    dtc = false;
    read_dtc();
  }
  switch (menu)
  {
  case 0: //Επίλεξε σελίδα πληροφοριών 1
    char battery_voltage[6];
    read_battery_voltage(battery_voltage);
    send_page1_to_lcd(coolant_temp, battery_voltage, fuel_consumption, milage);    //Στείλε τα δεδομένα στην οθόνη
    break;
  case 1: //Επίλεξε σελίδα πληροφοριών 2
    send_page2_to_lcd(read_fuel_lvl(), read_fstat());        //Στείλε τα δεδομένα στην οθόνη
    break;
  case 2://Επίλεξε σελίδα πληροφοριών 3
    FordSpecificTemperatures fordTemps;
    read_Ford_specific_temperatures(&fordTemps); //Διάβασε την θερμοκρασία των κυλίνδρων
    send_page3_to_lcd(coolant_temp, fordTemps.uknown_temperature, fordTemps.cylinder_temperature); //Στείλε τα δεδομένα στην οθόνη
    break;
  case 3: //Επίλεξε σελίδα πληροφοριών 3
    send_page4_to_lcd(read_maf(), read_oxvolts(), previous_cycle_sterm, read_fterm());        //Στείλε τα δεδομένα στην οθόνη
    break;
  // case 4:
  //  send_page5_to_lcd(medium_milage);
  //  break;
  }
  delay(10);
}

 /* Συνάρτηση που διαχειρίζεται το πάτημα του κουμπιού 1 */
void button1(unsigned int holdTime){
    ++menu;
    if (menu == 4){
      menu = 0;
    }
    delay(20);
}

/* Συνάρτηση που διαχειρίζεται το πάτημα του κουμπιού 2 */
void button2(unsigned int holdTime) {
    delay(50);
    dtc = true;
} 


void read_temp_threshold()
{
  int val = analogRead(analogPin);
  if (val >= 0 && val < 85)
    threshold = 90;
  else if (val >= 86 && val < 170)
    threshold = 101;
  else if (val >= 171 && val < 255)
    threshold = 102;
  else if (val >= 256 && val < 340)
    threshold = 103;
  else if (val >= 341 && val < 425)
    threshold = 104;
  else if (val >= 426 && val < 510)
    threshold = 105;
  else if (val >= 511 && val < 595)
    threshold = 106;
  else if (val >= 596 && val < 680)
    threshold = 107;
  else if (val >= 681 && val < 765)
    threshold = 108;
  else if (val >= 766 && val < 850)
    threshold = 109;
  else if (val >= 851 && val < 935)
    threshold = 110;
  else if (val >= 936)
    threshold = 120;
  if (threshold != temp_thresh)
  {
    temp_thresh = threshold;
    inform_temperature_threshold_changed();
  }
}

void inform_temperature_threshold_changed()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(threshold, DEC);
  lcd.print("C Threshold");
  delay(200);
}


void check_coolant_temperature_threshold(int temperature)
{
  if (temperature > threshold && temperature < 250)
  {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(VENTILATOR_PIN, HIGH);
  } /* Αν η θερμοκρασία του ψυκτικού είναι μεγαλύτερη του threshold, άναψε το LED 2 και άνοιξε το ventilator*/
  else
  {
    digitalWrite(RED_LED, LOW);
    digitalWrite(VENTILATOR_PIN, LOW);
  }
}





float calculate_fuel_consumption()
{
  float maf = read_maf();
  int spd = read_vehicle_speed();
  cli();
  float consumption = 0.00;
  real_time_cons = ((maf / 10437.00000) * (1 + (previous_cycle_sterm / 100)) * DEVIATION_FACTOR);// After testing my car consumed 17% more fuel.
  if (spd > 0)
  {
    consumption = (spd / 3600.00) / real_time_cons; //(spd/3600)*((14,7*710)/maf);
    //milage = 100.00 / cons;
  }
  temp_cons += real_time_cons;
  ++counter;
  sei();
  previous_cycle_sterm = read_sterm(); //Διαβάζουμε έδω το short term fuel trim για να εφαρμοστεί στον επόμενο κύκλο
  return consumption;
}



// void send_page5_to_lcd(float milage){
//   lcd.clear();
//     if (milage >= 10)
//   {
//     lcd.setCursor(6, 0);
//   }
//   else
//   {
//     lcd.setCursor(7, 0);
//   }
//   lcd.print(milage, 1);
//   lcd.print(" L/100");
// }

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της πρώτης σελίδας στην οθόνη */
/*Αναλόγως πόσα ψηφία έχει η τιμή, βάζει τον κέρσορα στη σωστή θέση της
οθόνης ώστε οι πληροφορίες να είναι ευανάγνωστες. Επίσης εκτυπώνει τις μονάδες
των τιμών στα σωστά σημεία*/
void send_page1_to_lcd(int coolant_temperature, char* bat, float cons, float milage)
{
  int i = 0;
  lcd.clear();
  if (coolant_temperature <= -10)
  {
  }
  else if (coolant_temperature < 0 and coolant_temperature > -10)
  {
    lcd.setCursor(1, 0);
    i = 1;
  }
  else if (coolant_temperature < 10)
  {
    lcd.setCursor(2, 0);
    i = 2;
  }
  else if (coolant_temperature < 100)
  {
    lcd.setCursor(1, 0);
    i = 1;
  }
  lcd.print(coolant_temperature, DEC);
  lcd.print("C  ");
  if (milage >= 10)
  {
    lcd.setCursor(6, 0);
  }
  else
  {
    lcd.setCursor(7, 0);
  }
  lcd.print(milage, 1);
  lcd.print(" L/100");
  lcd.setCursor(0, 1);
  lcd.write(bat);
  lcd.print("  ");
  if (cons < 10)
  {
    lcd.print(" ");
  }
  lcd.print(cons, 1);
  lcd.print(" Km/L");
}

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της δεύτερης σελίδας στην οθόνη */
void send_page2_to_lcd(float fuel_lvl, int fstat)
{
  int i = 0;
  lcd.clear();
  if (total_cons < 10)
  {
    lcd.setCursor(2, 0);
  }
  else if (total_cons < 100)
  {
    lcd.setCursor(1, 0);
  }
  lcd.print(total_cons, 3);
  lcd.print("L Cons ");
  lcd.setCursor(0, 1);
  if (fuel_lvl < 10)
  {
    lcd.setCursor(1, 1);
  }
  lcd.print(fuel_lvl, 1);
  lcd.print(" % ");
  lcd.print(calculate_fuel_in_liters(fuel_lvl), 1);
  lcd.print(" L");
  lcd.setCursor(14, 1);
  lcd.print(fstat);
}

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της τρίτης σελίδας στην οθόνη */
void send_page3_to_lcd(int coolant_temperature, int unknown_temperature, int cylinder_temperature)
{
  lcd.clear();
  lcd.print("C1 ");
  lcd.print(coolant_temperature, DEC);
  lcd.print("C ");
  lcd.print("C2 ");
  lcd.print(unknown_temperature, DEC);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Cyl ");
  lcd.print(cylinder_temperature, DEC);
  lcd.print("C");
}
void send_page4_to_lcd(float mass_air_flow, float oxygen_volts, float sterm, float fterm)
{
  lcd.clear();
  lcd.print(mass_air_flow, 2);
  lcd.print("G/s ");
  lcd.print(oxygen_volts, 3);
  lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print(sterm, 2);
  lcd.print("% ");
  lcd.print(fterm, 2);
  lcd.print("%");
}


float calculate_fuel_in_liters(float fuel_tank_percentage){
  return fuel_tank_percentage * 0.55;
}

/* Calculate the milage in fuel liters per 100km, given the kilometers per liter consumption */
float calculate_milage(float fuel_consumption){
  if(fuel_consumption>0.00){
    return 100.00 / fuel_consumption;
  }
  return 0.00;

}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την κατάσταση σφαλμάτων του οχήματος  */
void read_dtc()
{
  int i = 1;
  char tmp[2];
  unsigned int active_dtcs = 0;
  /* Διαβάζει 4 byte: Α, Β, C και D
  Το πρώτο byte (A) περιέχει δύο στοιχεία.
  Το Bit A7 (MSB του byte A, το πρώτο byte) δείχνει εάν το MIL (λαμπάκι βλάβης του κινητήρα)
  ανάβει ή όχι. Τα bits A6 έως A0 αντιπροσωπεύουν τον αριθμό διαγνωστικών κωδικών βλάβης που
  επισημαίνονται επί του παρόντος στο ECU.
  Το δεύτερο, το τρίτο και το τέταρτο bytes (B, C και D) παρέχουν πληροφορίες σχετικά με τη
  διαθεσιμότητα και την πληρότητα ορισμένων δοκιμών επί του οχήματος*/
  lcd.clear();
  lcd.setCursor(0, 0);
  stn_com("0101");
  if (stn_buffer[2] & 0b10000000)
  {                                     // if we got MIL ON and active DTCs
    active_dtcs = stn_buffer[2] - 0x80; // byte-128 to find how many active dtcs exist cause of mil bit
    lcd.print("MIL is ON!");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
  }
  else
    active_dtcs = stn_buffer[2] & 0b01111111;
  if (active_dtcs)
  {
    lcd.print(active_dtcs);
    lcd.print(" DTCs FOUND!");
    delay(3000);   //Πάγωσε την οθόνη για 3" ώστε ο χρήστης να προλάβει να διαβάσει
    stn_com("03"); // Mode 03 – Δίαβασε DTCs (Diagnostic Trouble Codes), δεν χρειάζεται PID
    lcd.clear();
    lcd.setCursor(0, 0);
    for (int j = 1; j <= active_dtcs; j++)
    { // byte#0 is only the mode reply-not needed
      if (stn_buffer[i] <= 0x3F)
      {
        lcd.print("P");
        sprintf(tmp, "%02x", stn_buffer[i]);
        lcd.print(tmp);
        sprintf(tmp, "%02x", stn_buffer[i + 1]);
        lcd.print(tmp);
        lcd.print(" ");
      }
      else if (stn_buffer[i] <= 0x7F)
      {
        lcd.print("C");
        lcd.print((stn_buffer[i] >> 4) - 4);
        lcd.print(stn_buffer[i] && 0b00001111);
        sprintf(tmp, "%02x", stn_buffer[i + 1]);
        lcd.print(tmp);
        lcd.print(" ");
      }
      else if (stn_buffer[i] <= 0xBF)
      {
        lcd.print("B");
        lcd.print((stn_buffer[i] >> 4) - 8);
        lcd.print(stn_buffer[i] && 0b00001111);
        sprintf(tmp, "%02x", stn_buffer[i + 1]);
        lcd.print(tmp);
        lcd.print(" ");
      }
      else
      {
        lcd.print("U");
        lcd.print((stn_buffer[i] >> 4) - 12);
        lcd.print(stn_buffer[i] && 0b00001111);
        sprintf(tmp, "%02x", stn_buffer[i + 1]);
        lcd.print(tmp);
        lcd.print(" ");
      }
      i = i + 2;
      if (j == 3)
      {
        lcd.setCursor(0, 1);
      }
    }

    delay(8000); //Πάγωσε την οθόνη για 8" ώστε ο χρήστης να προλάβει να διαβάσει
  }
  else
  {
    lcd.print("NO DTCs FOUND");
    delay(2000);
  }
}



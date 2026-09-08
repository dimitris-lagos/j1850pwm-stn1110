#include <avr/pgmspace.h>   //Αποθήκευση των σταθερών κειμένων στη flash του ATmega328 (F(), PSTR), ώστε να εξοικονομηθεί χώρος runtime στην sram
#include <LiquidCrystal.h>  // Βιβλιοθήκη που διαχειρίζεται την διασύνδεση με την οθόνη LCD
#include "global.h"
#include "OBD_Interface.h"
#include "LCD_Interface.h"
#include "Debouncy.h"

#define RED_LED 19
#define VENTILATOR_PIN 9
#define BUTTON_1_PIN 2
#define BUTTON_2_PIN 3
#define TEMP_THRESHOLD_POT_PIN A3   // potentiometer wiper (middle terminal) connected to analog pin 3
                                    // outside leads to ground and +5V

/* Η διασύνδεση:
Button Digital Pin 16
LED    Digital Pin 19
Mosfet Digital Pin 9
*/

/* Πρωτότυπα (ο Arduino builder τα παράγει μόνος του, αλλά όχι πάντα σωστά για
 * static συναρτήσεις ή ορίσματα struct. Ρητά είναι πιο ασφαλές.) */
void button1(unsigned int holdTime);
void button2(unsigned int holdTime);
void sample_fuel();
static float fuel_correction_factor();
float instant_km_per_liter();
float calculate_milage(float km_per_liter);
float calculate_fuel_in_liters(float fuel_tank_percentage);
void read_page_specific(uint8_t page);
void read_temp_threshold();
void inform_temperature_threshold_changed();
void check_coolant_temperature_threshold(int16_t temperature);
void send_init_page_to_lcd();
void send_page1_to_lcd(int16_t coolant_temperature, const char *bat, float cons, float milage);
void send_page2_to_lcd(float fuel_lvl, uint8_t fstat);
void send_page3_to_lcd(int16_t coolant_temperature, int16_t unknown_temperature, int16_t cylinder_temperature);
void send_page4_to_lcd(float mass_air_flow, float oxygen_volts, float sterm, float fterm);
void read_dtc();

/* ============================================================================
 *  Τμήμα δήλωσης global μεταβλητών
 * ============================================================================
 * Όλες με το μικρότερο τύπο που χωράει την τιμή (uint8_t αντί int όπου γίνεται):
 * η SRAM του ATmega328 είναι 2 KB και τα 128 bytes του Serial RX/TX buffer, το
 * αντικείμενο της LCD και η στοίβα τα μοιράζονται. */

Debouncy Debouncy_Buttons;

/* Αρχικοποίηση βιβλιοθήκης που διαχειρίζεται την επικοινωνία με την οθόνη,
 με τους ακροδέκτες του Arduino που συνδέονται στην οθόνη  */
LiquidCrystal lcd(12, 13, 5, 14, 15, 16); // LiquidCrystal(rs, enable, d4, d5, d6, d7)

/* --- Κατάσταση UI. volatile: γράφονται μέσα στα ISR των κουμπιών (Debouncy) και
 *     διαβάζονται στο loop(). Το volatile υποχρεώνει τον compiler να τις ξαναδιαβάζει
 *     από τη μνήμη κάθε φορά, αντί να κρατά αντίγραφο σε register. --- */
volatile uint8_t menu = 0;
volatile bool dtc_requested = false;
uint8_t last_drawn_page = 0xFF;      // 0xFF = υποχρεωτική ανανέωση οθόνης

/* --- Κατανάλωση καυσίμου --- */
/* Total consumption of the vehicle for the current session, σε µL.
 * Ακέραιος 32 bit (έως 4294 L): κρατά την ίδια ακρίβεια όσο κι αν μεγαλώσει το
 * σύνολο. Ένας float στα 30 L έχει βήμα ~2 µL, οπότε κάθε πρόσθεση των ~100 µL
 * ενός δείγματος ρελαντί θα στρογγυλευόταν κατά 1-2 %. */
uint32_t total_fuel_ul = 0;
static float fuel_ul_carry = 0.0f;       // κλάσμα µL που δεν χώρεσε στον ακέραιο, μεταφέρεται στο επόμενο δείγμα
static float last_rate_ulps = 0.0f;      // ρυθμός καυσίμου του τελευταίου έγκυρου δείγματος [µL/s]
static unsigned long last_sample_ms = 0; // millis() του τελευταίου έγκυρου δείγματος
static bool have_last_sample = false;

/* --- Τελευταίες έγκυρες τιμές από την ECU (cache) ---
 * Κάθε PID διαβάζεται εκ περιτροπής (βλ. loop). Οι σελίδες εμφανίζουν ό,τι υπάρχει
 * εδώ. Σε άκυρη απάντηση οι read_* δεν αγγίζουν την τιμή, άρα μένει η προηγούμενη. */
uint16_t maf_raw = 0;                    // PID 10, μονάδες 0.01 g/s
uint8_t  speed_kmh = 0;                  // PID 0D
uint8_t  fuel_status = 0;                // PID 03
uint8_t  stft_raw = 128;                 // PID 06, 128 = 0 %
uint8_t  ltft_raw = 128;                 // PID 07, 128 = 0 %
int16_t  coolant_temp = 0;               // PID 05, °C
uint8_t  o2_raw = 0;                     // PID 14, V = A/200
uint16_t fuel_lvl_raw = 0;               // Ford 22 16C1, % = raw × 0.003
char     battery_voltage[6] = "";        // ATRV, ASCII π.χ. "12.4V"
FordSpecificTemperatures fordTemps = {0, 0};

/* --- Όριο θερμοκρασίας για τον ανεμιστήρα --- */
uint8_t threshold = 95;
uint8_t temp_thresh = 95;


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

  lcd.clear();
  digitalWrite(RED_LED, HIGH); // LED on
  delay(1000);            // Καθυστέρηση 1sec
  lcd.setCursor(0, 0);    /* μετακίνηση του κέρσορα στην πρώτη γραμμή και πρώτη στήλη της οθόνης */
  lcd.print(F(WELLCOME_TEXT)); /* Γράψε στην οθόνη το κείμενο εισαγωγής (F(): από τη flash) */
  digitalWrite(RED_LED, LOW);  // LED off
  delay(1000);
  wait_for_STN_ready();
  setup_STN_communication_properties();
  send_init_page_to_lcd();
  /* Τα interrupts είναι ήδη ενεργά από το init() του Arduino (τα χρειάζονται
   * delay() και Serial), οπότε τα κουμπιά δουλεύουν από εδώ και πέρα. */
}

void send_init_page_to_lcd(){
  lcd.setCursor(0, 1);
  if (read_fuel_lvl(&fuel_lvl_raw))
  {
    float pct = fuel_level_percent(fuel_lvl_raw);
    lcd.print(pct, 1);
    lcd.print(F(" %F  "));
    lcd.print(calculate_fuel_in_liters(pct), 1);
    lcd.print(F(" L"));
  }
  else
  {
    lcd.print(F("FUEL LVL N/A"));
  }
  delay(5000); //περίμενε 5"
}


/* ============================================================================
 *  Κύριος βρόχος
 * ============================================================================
 * Το πρόγραμμα περιέχει 4 σελίδες προβολής πληροφοριών στην οθόνη.
 * Το κουμπί 1 αλλάζει μεταξύ των σελίδων 1 ως 4.
 * Το κουμπί 2 ενεργοποιεί την ανάγνωση των κωδικών σφαλμάτων.
 *
 * Σχεδιασμός: ο χρόνος του loop() καθορίζεται σχεδόν αποκλειστικά από τις αιτήσεις
 * στο J1850 bus (~50-150 ms η καθεμία). Οι κύκλοι της CPU δεν είναι το πρόβλημα,
 * οι αιτήσεις είναι. Άρα:
 *  1. MAF σε ΚΑΘΕ κύκλο (μόνο αυτό χρειάζεται η ολοκλήρωση της κατανάλωσης).
 *  2. Ταχύτητα μόνο όταν τη δείχνει η οθόνη (σελίδα 1).
 *  3. ΕΝΑ δευτερεύον PID ανά κύκλο, εκ περιτροπής.
 * Έτσι κάθε κύκλος έχει 2-3 αιτήσεις και σταθερή διάρκεια (~200 ms) ανεξάρτητα
 * από τη σελίδα, και ο MAF δειγματοληπτείται αρκετά πυκνά για να πιάνει τις
 * μεταβολές του (σε ξεκίνημα από στάση πάει από 3 σε 40 g/s μέσα σε 200 ms).
 */
void loop()
{
  static uint8_t slot = 0;               // ποιο δευτερεύον PID έχει σειρά
  static unsigned long last_lcd_ms = 0;
  uint8_t page = menu;                   // μία ανάγνωση του volatile ανά κύκλο. Αν αλλάξει από το κουμπί στη μέση, το πιάνουμε στον επόμενο

  read_temp_threshold();                 // analogRead, ~0.1 ms

  sample_fuel();                         // Διάβασε MAF και ολοκλήρωσε την κατανάλωση

  if (page == 0) read_vehicle_speed(&speed_kmh);   // μόνο η σελίδα 1 χρησιμοποιεί ταχύτητα (στιγμιαία km/L)

  switch (slot)
  {
  case 0:
    read_fstat(&fuel_status);            // Διάβασε κατάσταση κυκλώματος κινητήρα (closed/open loop), το χρειάζεται το μοντέλο καυσίμου
    break;
  case 1:
    if (read_coolant_temperature(&coolant_temp))   //Διάβασε την θερμοκρασία του οχήματος
      check_coolant_temperature_threshold(coolant_temp);   // και έλεγξε το όριο για ανεμιστήρα/LED, μόνο με έγκυρη τιμή
    break;
  case 2:
    /* Trims: για τον υπολογισμό καυσίμου (APPLY_FUEL_TRIMS) και για τη σελίδα 4.
     * Με APPLY_FUEL_TRIMS = 0 ο compiler κρατά μόνο τον έλεγχο της σελίδας. */
    if (APPLY_FUEL_TRIMS || page == 3)
    {
      read_sterm(&stft_raw);
      read_fterm(&ltft_raw);
    }
    break;
  default:
    read_page_specific(page);            // ό,τι χρειάζεται μόνο η τρέχουσα σελίδα
    break;
  }
  if (++slot > 3) slot = 0;

  if (dtc_requested)
  {
    dtc_requested = false;
    read_dtc();                          // μπλοκάρει 2-11 s. Το κενό στην ολοκλήρωση το γεφυρώνει το Δt (βλ. sample_fuel)
    last_drawn_page = 0xFF;              // η οθόνη έχει άλλο περιεχόμενο, ξαναζωγράφισε
  }

  /* Οθόνη: κάθε LCD_REFRESH_MS ή αμέσως σε αλλαγή σελίδας. Το lcd.clear() κοστίζει
   * ~2 ms νεκρής αναμονής + ~3 ms για 32 χαρακτήρες. Σε κάθε κύκλο των ~200 ms θα
   * ήταν χαμένος χρόνος και τρεμόπαιγμα χωρίς νέα πληροφορία. */
  unsigned long now = millis();
  if (page != last_drawn_page || now - last_lcd_ms >= LCD_REFRESH_MS)
  {
    last_drawn_page = page;
    last_lcd_ms = now;
    switch (page)
    {
    case 0: //Επίλεξε σελίδα πληροφοριών 1
    {
      float cons = instant_km_per_liter();
      send_page1_to_lcd(coolant_temp, battery_voltage, cons, calculate_milage(cons));    //Στείλε τα δεδομένα στην οθόνη
      break;
    }
    case 1: //Επίλεξε σελίδα πληροφοριών 2
      send_page2_to_lcd(fuel_level_percent(fuel_lvl_raw), fuel_status);        //Στείλε τα δεδομένα στην οθόνη
      break;
    case 2: //Επίλεξε σελίδα πληροφοριών 3
      send_page3_to_lcd(coolant_temp, fordTemps.uknown_temperature, fordTemps.cylinder_temperature); //Στείλε τα δεδομένα στην οθόνη
      break;
    case 3: //Επίλεξε σελίδα πληροφοριών 4
      send_page4_to_lcd(maf_g_per_s(maf_raw), o2_volts(o2_raw), trim_percent(stft_raw), trim_percent(ltft_raw));   //Στείλε τα δεδομένα στην οθόνη
      break;
    }
  }
}

/* Η ανάγνωση που χρειάζεται μόνο η τρέχουσα σελίδα. Μία ανά 4 κύκλους. */
void read_page_specific(uint8_t page)
{
  switch (page)
  {
  case 0: read_battery_voltage(battery_voltage); break;          //Διάβασε την τάση της μπαταρίας (ATRV, χωρίς κίνηση στο bus)
  case 1: read_fuel_lvl(&fuel_lvl_raw); break;                   //Διάβασε τη στάθμη καυσίμου (2 AT + 1 Mode 22)
  case 2: read_Ford_specific_temperatures(&fordTemps); break;    //Διάβασε την θερμοκρασία των κυλίνδρων (2 AT + 2 Mode 22)
  case 3: read_oxvolts(&o2_raw); break;                          //Διάβασε την τάση του αισθητήρα οξυγόνου (PID 14)
  }
}


/* ============================================================================
 *  Κατανάλωση καυσίμου
 * ============================================================================
 * Καλείται σε ΚΑΘΕ κύκλο του loop(). Διαβάζει τον MAF, υπολογίζει τον ρυθμό
 * καυσίμου σε µL/s και ολοκληρώνει με το ΠΡΑΓΜΑΤΙΚΟ Δt από το προηγούμενο δείγμα:
 *
 *      ΔV = (r_prev + r_now) / 2 × Δt          (μέθοδος τραπεζίου)
 *
 * Γιατί ολοκλήρωση με Δt και όχι με σταθερό χρονικό βήμα (timer):
 *  Το καύσιμο κάθε χρονικού διαστήματος μετριέται ακριβώς μία φορά, όσο κι αν
 *  διαρκέσει ο κύκλος του loop(): σελίδα με πολλές αιτήσεις, read_dtc() που
 *  μπλοκάρει 3-11 s, αλλαγή threshold. Αν ένα δείγμα απορριφθεί (άκυρη απάντηση)
 *  απλά δεν ενημερώνουμε τίποτα, και το επόμενο έγκυρο δείγμα ολοκληρώνει πάνω
 *  σε ολόκληρο το κενό με τον τελευταίο γνωστό ρυθμό. Η ακρίβεια του millis()
 *  (resonator ±0.5 %) είναι υπεραρκετή για τον σκοπό.
 *
 * Επαλήθευση: γέμισμα ως το κλικ, μηδένισε (power cycle), οδήγησε, ξαναγέμισε ως το
 * κλικ και σύγκρινε με τη σελίδα 2. Μία διαδρομή εθνική, μία πόλη. Αν η εθνική
 * βγαίνει σωστά και λείπει στην πόλη, ρύθμισε τους OPEN_LOOP_*_FACTOR. Αν λείπουν
 * και οι δύο το ίδιο ποσοστό, κοίτα το LTFT (σελίδα 4) και το APPLY_FUEL_TRIMS.
 */
void sample_fuel()
{
  uint16_t raw;
  if (!read_maf_raw(&raw)) return;       // άκυρη απάντηση: το δείγμα αγνοείται
  unsigned long now = millis();
  maf_raw = raw;                         // για τη σελίδα 4

  /* µL/s σε λ = 1, ένας πολλαπλασιασμός (η σταθερά υπολογίζεται στη μεταγλώττιση) */
  float rate = raw * UL_PER_SEC_PER_MAF_COUNT;
  rate *= fuel_correction_factor();

  if (have_last_sample)
  {
    unsigned long dt_ms = now - last_sample_ms;   // unsigned αφαίρεση: σωστή και στο wrap του millis() (49 ημέρες)
    if (dt_ms > 30000UL) dt_ms = 30000UL;         // >30 s σημαίνει νεκρό bus. Όριο για να μην υπερχειλίσει το cast παρακάτω
    /* (r1 + r2)/2 × dt_ms/1000 = (r1 + r2) × dt_ms × 0.0005 */
    float inc = (rate + last_rate_ulps) * (float)dt_ms * 0.0005f + fuel_ul_carry;
    uint32_t whole = (uint32_t)inc;
    fuel_ul_carry = inc - (float)whole;           // το υπόλοιπο < 1 µL δεν χάνεται
    total_fuel_ul += whole;
  }
  have_last_sample = true;
  last_sample_ms = now;
  last_rate_ulps = rate;
}

/* Συντελεστής πάνω στο MAF/AFR ανάλογα με την κατάσταση της ECU (PID 03).
 * Βλ. global.h για την αιτιολόγηση των συντελεστών και των trims. */
static float fuel_correction_factor()
{
  float f = 1.0f;
  if (fuel_status & FSTAT_OPEN_LOOP_LOAD)      f = OPEN_LOOP_LOAD_FACTOR;
  else if (fuel_status & FSTAT_OPEN_LOOP_COLD) f = OPEN_LOOP_COLD_FACTOR;

#if APPLY_FUEL_TRIMS
  /* (A - 128)/128 είναι ακριβώς το κλάσμα του trim, χωρίς πολλαπλασιασμό με 1.28.
   * Το LTFT η ECU το εφαρμόζει πάντα. Το STFT μόνο σε closed loop (σε open loop
   * παγώνει στο 0), οπότε το προσθέτουμε μόνο τότε. */
  int16_t trim = (int16_t)ltft_raw - 128;
  if (fuel_status & (FSTAT_CLOSED_LOOP | FSTAT_CLOSED_LOOP_FAULT))
    trim += (int16_t)stft_raw - 128;
  f *= 1.0f + trim * (1.0f / 128.0f);
#endif
  return f;
}

/* Στιγμιαία κατανάλωση σε km/L από την τελευταία ταχύτητα και τον τελευταίο ρυθμό.
 *   km/L = (km/h ÷ 3600) ÷ (µL/s ÷ 1e6) = km/h × (1e6/3600) / (µL/s)
 * Σε στάση ή χωρίς ρυθμό επιστρέφει 0 (η οθόνη δείχνει 0.0). */
float instant_km_per_liter()
{
  if (speed_kmh == 0 || last_rate_ulps <= 0.0f) return 0.0f;
  return (float)speed_kmh * (1000000.0f / 3600.0f) / last_rate_ulps;
}

/* Calculate the milage in fuel liters per 100km, given the kilometers per liter consumption */
float calculate_milage(float km_per_liter){
  if (km_per_liter > 0.0f){
    return 100.0f / km_per_liter;
  }
  return 0.0f;
}

float calculate_fuel_in_liters(float fuel_tank_percentage){
  return fuel_tank_percentage * (FUEL_TANK_LITERS / 100.0f);   // σταθερά στη μεταγλώττιση (= 0.55)
}


/* ============================================================================
 *  Κουμπιά
 * ============================================================================
 * Καλούνται ΜΕΣΑ από ISR (Debouncy → attachInterrupt), με τα interrupts κλειστά.
 * Κάνουν μόνο μία ανάθεση σε volatile μεταβλητή. Τίποτα που περιμένει (delay,
 * Serial, LCD) δεν ανήκει σε ISR: το millis() δεν προχωρά όσο τρέχει. Το debounce
 * το κάνει η Debouncy. */

 /* Συνάρτηση που διαχειρίζεται το πάτημα του κουμπιού 1: επόμενη σελίδα */
void button1(unsigned int holdTime){
  (void)holdTime;                        // η υπογραφή την απαιτεί (Debouncy), δεν τη χρησιμοποιούμε
  uint8_t m = menu + 1;
  if (m >= 4) m = 0;
  menu = m;
}

/* Συνάρτηση που διαχειρίζεται το πάτημα του κουμπιού 2: ανάγνωση DTC στον επόμενο κύκλο */
void button2(unsigned int holdTime) {
  (void)holdTime;
  dtc_requested = true;
}


/* ============================================================================
 *  Όριο θερμοκρασίας / ανεμιστήρας
 * ============================================================================ */
void read_temp_threshold()
{
  int val = analogRead(TEMP_THRESHOLD_POT_PIN);
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
  lcd.print(F("C Threshold"));
  delay(200);
  last_drawn_page = 0xFF;   // να ξαναζωγραφιστεί η σελίδα στον επόμενο κύκλο
}

void check_coolant_temperature_threshold(int16_t temperature)
{
  if (temperature > (int16_t)threshold && temperature < 250)
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


/* ============================================================================
 *  Σελίδες οθόνης
 * ============================================================================
 * Όλα τα σταθερά κείμενα με F(): μένουν στη flash και διαβάζονται από εκεί κατά
 * την εκτύπωση, αντί να αντιγράφονται στην SRAM κατά την εκκίνηση. */

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της πρώτης σελίδας στην οθόνη */
/*Αναλόγως πόσα ψηφία έχει η τιμή, βάζει τον κέρσορα στη σωστή θέση της
οθόνης ώστε οι πληροφορίες να είναι ευανάγνωστες. Επίσης εκτυπώνει τις μονάδες
των τιμών στα σωστά σημεία*/
void send_page1_to_lcd(int16_t coolant_temperature, const char *bat, float cons, float milage)
{
  lcd.clear();
  if (coolant_temperature <= -10)
  {
  }
  else if (coolant_temperature < 0)
  {
    lcd.setCursor(1, 0);
  }
  else if (coolant_temperature < 10)
  {
    lcd.setCursor(2, 0);
  }
  else if (coolant_temperature < 100)
  {
    lcd.setCursor(1, 0);
  }
  lcd.print(coolant_temperature, DEC);
  lcd.print(F("C  "));
  if (milage >= 10)
  {
    lcd.setCursor(6, 0);
  }
  else
  {
    lcd.setCursor(7, 0);
  }
  lcd.print(milage, 1);
  lcd.print(F(" L/100"));
  lcd.setCursor(0, 1);
  lcd.print(bat);
  lcd.print(F("  "));
  if (cons < 10)
  {
    lcd.print(' ');
  }
  lcd.print(cons, 1);
  lcd.print(F(" Km/L"));
}

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της δεύτερης σελίδας στην οθόνη */
void send_page2_to_lcd(float fuel_lvl, uint8_t fstat)
{
  float total_l = (float)total_fuel_ul * 1e-6f;   // µL → L, ένας πολλαπλασιασμός
  lcd.clear();
  if (total_l < 10)
  {
    lcd.setCursor(2, 0);
  }
  else if (total_l < 100)
  {
    lcd.setCursor(1, 0);
  }
  lcd.print(total_l, 3);
  lcd.print(F("L Cons "));
  lcd.setCursor(0, 1);
  if (fuel_lvl < 10)
  {
    lcd.setCursor(1, 1);
  }
  lcd.print(fuel_lvl, 1);
  lcd.print(F(" % "));
  lcd.print(calculate_fuel_in_liters(fuel_lvl), 1);
  lcd.print(F(" L"));
  lcd.setCursor(14, 1);
  lcd.print(fstat);
}

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της τρίτης σελίδας στην οθόνη */
void send_page3_to_lcd(int16_t coolant_temperature, int16_t unknown_temperature, int16_t cylinder_temperature)
{
  lcd.clear();
  lcd.print(F("C1 "));
  lcd.print(coolant_temperature, DEC);
  lcd.print(F("C "));
  lcd.print(F("C2 "));
  lcd.print(unknown_temperature, DEC);
  lcd.print('C');
  lcd.setCursor(0, 1);
  lcd.print(F("Cyl "));
  lcd.print(cylinder_temperature, DEC);
  lcd.print('C');
}

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της τέταρτης σελίδας στην οθόνη: MAF, τάση O2, STFT, LTFT */
void send_page4_to_lcd(float mass_air_flow, float oxygen_volts, float sterm, float fterm)
{
  lcd.clear();
  lcd.print(mass_air_flow, 2);
  lcd.print(F("G/s "));
  lcd.print(oxygen_volts, 3);
  lcd.print('V');
  lcd.setCursor(0, 1);
  lcd.print(sterm, 2);
  lcd.print(F("% "));
  lcd.print(fterm, 2);
  lcd.print('%');
}


/* ============================================================================
 *  Κωδικοί σφαλμάτων (DTC)
 * ============================================================================ */

/* Ένα hex ψηφίο στην οθόνη, χωρίς sprintf (το sprintf κοστίζει ~1.5 KB flash) */
static void lcd_print_hex_nibble(uint8_t v)
{
  lcd.print((char)(v < 10 ? '0' + v : 'A' + v - 10));
}

/* Αποκωδικοποίηση ενός DTC κατά SAE J2012 από τα 2 bytes του, A και B:
 *   A bits 7-6: γράμμα (00 P, 01 C, 10 B, 11 U)
 *   A bits 5-4: 1ο ψηφίο (0-3)
 *   A bits 3-0: 2ο ψηφίο (hex)
 *   B         : 3ο και 4ο ψηφίο (hex)
 * Μία φόρμουλα για όλα τα γράμματα. Ο πίνακας "PCBU" μένει στη flash (PSTR). */
static void print_dtc(uint8_t a, uint8_t b)
{
  lcd.print((char)pgm_read_byte(PSTR("PCBU") + (a >> 6)));
  lcd_print_hex_nibble((a >> 4) & 0x03);
  lcd_print_hex_nibble(a & 0x0F);
  lcd_print_hex_nibble(b >> 4);
  lcd_print_hex_nibble(b & 0x0F);
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την κατάσταση σφαλμάτων του οχήματος  */
void read_dtc()
{
  /* Διαβάζει 4 byte: Α, Β, C και D
  Το πρώτο byte (A) περιέχει δύο στοιχεία.
  Το Bit A7 (MSB του byte A, το πρώτο byte) δείχνει εάν το MIL (λαμπάκι βλάβης του κινητήρα)
  ανάβει ή όχι. Τα bits A6 έως A0 αντιπροσωπεύουν τον αριθμό διαγνωστικών κωδικών βλάβης που
  επισημαίνονται επί του παρόντος στο ECU.
  Το δεύτερο, το τρίτο και το τέταρτο bytes (B, C και D) παρέχουν πληροφορίες σχετικά με τη
  διαθεσιμότητα και την πληρότητα ορισμένων δοκιμών επί του οχήματος*/
  uint8_t active_dtcs;
  lcd.clear();
  lcd.setCursor(0, 0);
  if (!get_dtc_number(&active_dtcs))
  {
    lcd.print(F("DTC READ ERROR"));
    delay(2000);
    return;
  }
  if (active_dtcs == 0)
  {
    lcd.print(F("NO DTCs FOUND"));
    delay(2000);
    return;
  }

  lcd.print(active_dtcs);
  lcd.print(F(" DTCs FOUND!"));
  delay(3000);   //Πάγωσε την οθόνη για 3" ώστε ο χρήστης να προλάβει να διαβάσει

  /* Mode 03. Κάθε frame της απάντησης έχει 7 bytes: το byte#0 είναι μόνο η απάντηση
   * του mode (0x43), δεν χρειάζεται, και ακολουθούν 3 ζεύγη (A, B), ένα ανά DTC.
   * Με πάνω από 3 DTCs έρχονται περισσότερα frames, το ένα πίσω από το άλλο στον
   * buffer, το καθένα με το δικό του 0x43 μπροστά. Κενή θέση = ζεύγος 0x0000.
   * Εμφάνιση: 2 κωδικοί ανά γραμμή (5 χαρακτήρες ο καθένας, χωράνε άνετα στις
   * 16 στήλες), 4 ανά οθόνη, 8" η κάθε οθόνη. */
  uint8_t len = read_active_dtc();
  uint8_t shown = 0;
  lcd.clear();
  for (uint8_t f = 0; f + 6 < len; f += 7)              // f = θέση του 0x43 κάθε frame
  {
    for (uint8_t k = 1; k <= 5; k += 2)
    {
      uint8_t a = stn_buffer[f + k];
      uint8_t b = stn_buffer[f + k + 1];
      if (a == 0 && b == 0) continue;                   // κενή θέση
      if (shown != 0 && (shown & 3) == 0)               // γέμισε η οθόνη (4 κωδικοί)
      {
        delay(8000);
        lcd.clear();
      }
      lcd.setCursor((shown & 1) ? 8 : 0, (shown >> 1) & 1);
      print_dtc(a, b);
      shown++;
    }
  }
  if (shown == 0) lcd.print(F("MODE 03 N/A"));
  delay(8000); //Πάγωσε την οθόνη για 8" ώστε ο χρήστης να προλάβει να διαβάσει
}

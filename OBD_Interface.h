#ifndef __obd_interface_h_included__
#define __obd_interface_h_included__
#include <Arduino.h>
#include "global.h"

/* Μέγεθος buffer απάντησης σε bytes. uint8_t γιατί κάθε τιμή είναι 0-255 (μισή
 * SRAM σε σχέση με int). Τα 24 bytes χωράνε 3 frames Mode 03 (7 bytes το καθένα
 * = 9 DTCs), που είναι η μεγαλύτερη απάντηση που ζητάμε. */
#define STN_BUFFER_SIZE 24

/* Μέγιστη αναμονή για το prompt '>' του STN. Το ίδιο το STN κλείνει κάθε αίτηση
 * με '>' μετά το δικό του timeout (~200 ms), οπότε το 1 s πιάνει μόνο
 * πραγματικές βλάβες (αποσύνδεση, κολλημένο STN) χωρίς να κρεμάει ο Uno. */
#define STN_RX_TIMEOUT_MS 1000

typedef struct FordSpecificTemperatures{
  int16_t cylinder_temperature;
  int16_t uknown_temperature;
}FordSpecificTemperatures;

/* Global μεταβλητή με τα bytes της τελευταίας απάντησης του STN
 * (μετά την αποκωδικοποίηση από ASCII-hex σε bytes) */
extern uint8_t stn_buffer[STN_BUFFER_SIZE];

/* --- Αρχικοποίηση / χαμηλό επίπεδο --- */
void init_STN_Serial_Interface();
void wait_for_STN_ready();
void setup_STN_communication_properties();
/* Στέλνει εντολή από τη flash (F("...")) και επιστρέφει πλήθος bytes απάντησης.
 * Για AT εντολές και Mode 03. Για PIDs χρησιμοποίησε τις read_* παρακάτω. */
uint8_t stn_com(const __FlashStringHelper *cmd);

/* --- Αναγνώσεις PID ---
 * Όλες επιστρέφουν true μόνο αν η απάντηση επαληθεύτηκε (σωστό mode, σωστό PID,
 * αρκετά bytes, καθαρό hex). Σε false το *out ΔΕΝ αλλάζει, ώστε ο καλών να
 * κρατήσει την τελευταία καλή τιμή. Επιστρέφουν raw τιμές: οι μετατροπές σε
 * φυσικές μονάδες γίνονται μόνο όπου χρειάζονται (inline παρακάτω). */
bool read_coolant_temperature(int16_t *celsius);
bool read_vehicle_speed(uint8_t *kmh);
bool read_maf_raw(uint16_t *raw);          // PID 0x10, μονάδες 0.01 g/s
bool read_fstat(uint8_t *status);          // PID 0x03, bits FSTAT_*
bool read_sterm(uint8_t *raw);             // PID 0x06, raw A (128 = 0 %)
bool read_fterm(uint8_t *raw);             // PID 0x07, raw A (128 = 0 %)
bool read_oxvolts(uint8_t *raw);           // PID 0x14, raw A (V = A/200)
bool read_fuel_lvl(uint16_t *raw);         // Ford Mode 22 PID 16C1, % = raw × 0.003
bool read_battery_voltage(char *bat);      // bat: τουλάχιστον 6 chars, π.χ. "12.4V"
bool read_Ford_specific_temperatures(FordSpecificTemperatures *fordTemps);
bool get_dtc_number(uint8_t *count);       // PID 0x01, bits 6-0 του A
uint8_t read_active_dtc();                 // Mode 03, επιστρέφει bytes στον stn_buffer

/* --- Μετατροπές raw → φυσικές μονάδες (μόνο για εμφάνιση) --- */
static inline float maf_g_per_s(uint16_t raw)       { return raw * 0.01f; }
static inline float trim_percent(uint8_t raw)       { return ((int16_t)raw - 128) * (100.0f / 128.0f); } // (A/1.28)-100
static inline float o2_volts(uint8_t raw)           { return raw * 0.005f; }   // A/200
static inline float fuel_level_percent(uint16_t raw){ return raw * 0.003f; }

#endif

#include "OBD_Interface.h"

uint8_t stn_buffer[STN_BUFFER_SIZE];

void init_STN_Serial_Interface(){
    Serial.begin(115200); //Εκίνηση της σειριακής επικοινωνίας με το STN1110 Baud 115200
}

/* Αδειάζει ό,τι έχει μείνει στον RX buffer από προηγούμενη (π.χ. ημιτελή λόγω
 * timeout) απάντηση, ώστε κάθε αίτηση να διαβάζει μόνο τη δική της απάντηση. */
static void stn_flush()
{
  while (Serial.read() >= 0) {}
}

/* Το STN1110 δεν στέλνει 16δικο, στέλνει ASCII, άρα κάθε ΗΕΧ ΒΥΤΕ αποστέλνεται από
 * το STN1110 με 2 χαρακτήρες. Η συνάρτηση επιστρέφει την τιμή (0-15) ενός τέτοιου
 * χαρακτήρα, ή -1 αν δεν είναι hex ψηφίο. Η αποκωδικοποίηση γίνεται χαρακτήρα προς
 * χαρακτήρα, χωρίς String και χωρίς δέσμευση heap: με 2 KB SRAM κάθε δέσμευση
 * κατακερματίζει τη μνήμη. */
static int8_t hex_val(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/* Η αντίστροφη μετατροπή: τιμή 0-15 → ASCII hex ψηφίο, για την αποστολή εντολών */
static char hex_digit(uint8_t v)
{
  return v < 10 ? '0' + v : 'A' + v - 10;
}

static void send_hex_byte(uint8_t b)
{
  Serial.write(hex_digit(b >> 4));
  Serial.write(hex_digit(b & 0x0F));
}

/* Διαβάζει την απάντηση του STN1110 μέχρι τον χαρακτήρα τερματισμού '>' (0x3E) και
 * αποθηκεύει τα δεδομένα σε έναν buffer. Κάθε 2 ASCII χαρακτήρες είναι ένα 16δικο
 * BYTE. Τα κενά και τα carriage return προσπερνιούνται.
 *
 * Επιστρέφει το πλήθος bytes. Στο *clean γράφει true μόνο αν η απάντηση ήταν
 * καθαρά hex: οποιοσδήποτε άλλος εκτυπώσιμος χαρακτήρας σημαίνει μήνυμα του STN
 * ("NO DATA", "STOPPED", "?", "BUS ERROR") και όχι δεδομένα, οπότε η απάντηση
 * σημαδεύεται άκυρη και δεν μεταφράζεται ποτέ σε αριθμό. Το ίδιο για περιττό
 * αριθμό ψηφίων, υπέρβαση του buffer ή timeout.
 *
 * Ο buffer μηδενίζεται πριν από κάθε απάντηση, ώστε κάθε θέση του να αντιστοιχεί
 * αποκλειστικά στην τρέχουσα απάντηση. */
static uint8_t stn_receive(bool *clean)
{
  uint8_t n = 0, nibbles = 0, byte_val = 0;
  bool ok = true;
  unsigned long t0 = millis();

  memset(stn_buffer, 0, STN_BUFFER_SIZE);
  for (;;)
  {
    if (millis() - t0 > STN_RX_TIMEOUT_MS) { ok = false; break; }
    int c = Serial.read();
    if (c < 0) continue;                  // τίποτα ακόμα στον RX buffer
    if (c == '>') break;                  // '>'=0x3E, χαρακτήρας τερματισμού
    int8_t v = hex_val((char)c);
    if (v < 0)
    {
      if (c > ' ') ok = false;            // εκτυπώσιμος μη-hex χαρακτήρας = μήνυμα του STN, όχι δεδομένα
      continue;                           // strip from spaces/carriage return etc etc
    }
    byte_val = (byte_val << 4) | v;
    if (++nibbles == 2)                   // Κάθε 2 ASCII χαρακτήρες είναι ένα 16δικο BYTE
    {
      nibbles = 0;
      if (n < STN_BUFFER_SIZE) stn_buffer[n++] = byte_val;   // Μετατροπή από ASCII σε byte
      else ok = false;                    // ποτέ έξω από τον buffer
      byte_val = 0;
    }
  }
  if (nibbles) ok = false;                // έμεινε μισό byte
  *clean = ok;
  return n;
}

/* Συνάρτηση που παίρνει σαν όρισμα την εντολή προς αποστολή (από τη flash, με F(),
 * ώστε το κείμενο να μην πιάνει SRAM) και αποθηκεύει τα δεδομένα που λαμβάνονται
 * ως απάντηση από το STN1110 στον stn_buffer. Επιστρέφει το πλήθος bytes.
 * Για AT εντολές (η απάντηση "OK" δεν είναι hex, δεν μας ενδιαφέρει) και Mode 03. */
uint8_t stn_com(const __FlashStringHelper *cmd)
{
  bool clean;
  stn_flush();
  Serial.print(cmd);
  Serial.print('\r');                     // send end of transmission char.
  return stn_receive(&clean);
}

/* Αίτηση Mode 01. Στέλνει "01" + PID + "1". Το τελευταίο, μονό, ψηφίο λέει στο
 * STN πόσες απαντήσεις να περιμένει (μία: στο Focus απαντά μόνο το PCM). Έτσι το
 * STN επιστρέφει το '>' αμέσως μετά το πρώτο frame, αντί να περιμένει το timeout
 * του (ATST / adaptive timing) μήπως απαντήσει και άλλο module. Τα 2 hex ψηφία
 * του PID φτιάχνονται on the fly, χωρίς sprintf και χωρίς buffer.
 *
 * Επαληθεύει: [0] == 0x41 (0x40 + mode), [1] == PID, μήκος >= 2 + data_len. */
static bool obd_request_01(uint8_t pid, uint8_t data_len)
{
  bool clean;
  stn_flush();
  Serial.print(F("01"));
  send_hex_byte(pid);
  Serial.print(F("1\r"));
  uint8_t n = stn_receive(&clean);
  return clean && n >= (uint8_t)(2 + data_len)
         && stn_buffer[0] == 0x41 && stn_buffer[1] == pid;
}

/* Αίτηση Mode 22 (Ford Manufacturer Extended PID, 16 bit). Ίδια λογική με την 01.
 * Επαληθεύει: [0] == 0x62, [1..2] == PID, μήκος >= 3 + data_len. */
static bool obd_request_22(uint16_t pid, uint8_t data_len)
{
  bool clean;
  stn_flush();
  Serial.print(F("22"));
  send_hex_byte(pid >> 8);
  send_hex_byte(pid & 0xFF);
  Serial.print(F("1\r"));
  uint8_t n = stn_receive(&clean);
  return clean && n >= (uint8_t)(3 + data_len) && stn_buffer[0] == 0x62
         && stn_buffer[1] == (uint8_t)(pid >> 8) && stn_buffer[2] == (uint8_t)pid;
}

/* Αναμονή μέχρι το STN1110 να είναι διαθέσιμο. Στέλνουμε "ati" και διαβάζουμε τους
 * χαρακτήρες που λαμβάνονται από την σειριακή σύνδεση μέχρι τον χαρακτήρα
 * τερματισμού '>', με timeout 500 ms, και επαναλαμβάνουμε μέχρι να απαντήσει.
 * Η ενεργή ερώτηση δουλεύει και όταν κάνει reset μόνο ο Uno (κουμπί reset) και
 * όχι το STN, που σε αυτή την περίπτωση δεν στέλνει banner εκκίνησης. */
void wait_for_STN_ready()
{
  for (;;)
  {
    stn_flush();
    Serial.print(F("ati\r"));
    unsigned long t0 = millis();
    while (millis() - t0 < 500)
    {
      if (Serial.read() == '>') return;
    }
  }
}

void setup_STN_communication_properties()
{
  stn_com(F("ate0"));   // echo off: η απάντηση να αρχίζει από το 0x41 και όχι από την επανάληψη της εντολής μας.
                        // Ρητά, ώστε να μη βασιζόμαστε στις αποθηκευμένες programmable parameters του STN
  stn_com(F("atsp1"));  // set protocol jpwm
  stn_com(F("ats0"));   // spaces off on replies from stn1110
  /* Σημείωση: το "atat2" (επιθετικό adaptive timing) θα κόντευε κι άλλο την αναμονή,
   * αλλά ρισκάρει να χάσει το 2ο/3ο frame του Mode 03. Το ψηφίο πλήθους απαντήσεων
   * στις obd_request_* κερδίζει το ίδιο για τα PIDs χωρίς αυτό το ρίσκο. */
}

/* ---------------------------------------------------------------------------
 *  Αναγνώσεις PID. Επιστρέφουν raw τιμές, false αν η απάντηση δεν επαληθεύτηκε.
 * --------------------------------------------------------------------------- */

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας θερμοκρασίας του ψυκτικού υγρού του οχήματος */
bool read_coolant_temperature(int16_t *celsius)
{
  if (!obd_request_01(0x05, 1)) return false;   // AT command: Mode:01 PID: 05
  *celsius = (int16_t)stn_buffer[2] - 40;       //Α-40
  return true;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας ταχύτητας του οχήματος */
bool read_vehicle_speed(uint8_t *kmh)
{
  if (!obd_request_01(0x0D, 1)) return false;   // AT command: Mode:01 PID: 0D
  *kmh = stn_buffer[2];                         //Α, km/h
  return true;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας μάζας εισαγωγής αέρα στην μηχανή του οχήματος */
/* (256*A+B)/100 g/s. Επιστρέφει το raw (256*A+B), δηλαδή μονάδες 0.01 g/s: ο
 * υπολογισμός κατανάλωσης δουλεύει με το raw, η μετατροπή σε g/s γίνεται μόνο για την οθόνη. */
bool read_maf_raw(uint16_t *raw)
{
  if (!obd_request_01(0x10, 2)) return false;   // AT command: Mode:01 PID: 10
  *raw = ((uint16_t)stn_buffer[2] << 8) | stn_buffer[3];   // 256*A+B με shift: χωρίς πολλαπλασιασμό, χωρίς υπερχείλιση int
  return true;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την κατάσταση κυκλώματος της μηχανής του οχήματος  */
/* Byte A, bits FSTAT_* στο global.h: closed loop / open loop και ο λόγος */
bool read_fstat(uint8_t *status)
{
  if (!obd_request_01(0x03, 1)) return false;   // AT command: Mode:01 PID: 03
  *status = stn_buffer[2];
  return true;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του Short Term Fuel Trim της μηχανής  */
/* Επιστρέφει το raw A. Ποσοστό = (A/1.28)-100, βλ. trim_percent() */
bool read_sterm(uint8_t *raw)
{
  if (!obd_request_01(0x06, 1)) return false;   // AT command: Mode:01 PID: 06
  *raw = stn_buffer[2];
  return true;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του Long Term Fuel Trim της μηχανής */
/* Επιστρέφει το raw A. Ποσοστό = (A/1.28)-100, βλ. trim_percent() */
bool read_fterm(uint8_t *raw)
{
  if (!obd_request_01(0x07, 1)) return false;   // AT command: Mode:01 PID: 07
  *raw = stn_buffer[2];
  return true;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του αισθητήρα οξυγόνου του οχήματος  */
/* Επιστρέφει το raw A. Volt = A/200, βλ. o2_volts().
 * Το B της ίδιας απάντησης είναι το STFT του ίδιου bank: (B/1.28)-100 */
bool read_oxvolts(uint8_t *raw)
{
  if (!obd_request_01(0x14, 2)) return false;   // AT command: Mode:01 PID: 14
  *raw = stn_buffer[2];
  return true;
}

/* Συνάρτηση που διαβάζει το ποσοστό πλήρωσης % του καυσίμου.
 * Επιστρέφει το raw (256*A+B). Ποσοστό = raw * 0.003, βλ. fuel_level_percent() */
bool read_fuel_lvl(uint16_t *raw)
{
  stn_com(F("atsh e410f1"));                    //Αλλαγή του header σε Ford Manufacturer Extended PID mode(22=physical addressing mode)
  bool ok = obd_request_22(0x16C1, 2);          //Διάβασε το ποσοστό πλήρωσης % του καυσίμου(MODE=22 και PID=16C1)
  if (ok) *raw = ((uint16_t)stn_buffer[3] << 8) | stn_buffer[4];   //Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
  stn_com(F("atsh 616af1"));                    //Επανέφερε τον header σε Standard PID Mode
  return ok;
}

/* Συνάρτηση που διαβάζει τις Ford-specific θερμοκρασίες (κύλινδρος και ψυκτικό, αισθητήρας 2) */
bool read_Ford_specific_temperatures(FordSpecificTemperatures *fordTemps)
{
  stn_com(F("atsh e410f1"));                    //Αλλαγή του header σε Ford Manufacturer Extended PID mode(22=physical addressing mode)

  bool ok = obd_request_22(0x1624, 2);          // Δίαβασε την θερμοκρασία των κυλίνδρων(MODE=22 και PID=1624)
  if (ok)
  {
    /* int32: με hi έως 127 το hi×512 ξεπερνά τα 32767 του int του AVR */
    int32_t hi = stn_buffer[3];
    int32_t lo = stn_buffer[4];
    int32_t t;
    if (hi == 0)        t = lo * 2;
    else if (hi <= 127) t = hi * 512 + lo * 2;
    else                t = (hi - 254) * 512 + lo * 2;
    fordTemps->cylinder_temperature = (int16_t)((t - 32) * 0.56f);   // °F → °C
  }

  if (obd_request_22(0x1139, 1)) fordTemps->uknown_temperature = stn_buffer[3];   //Διάβασε την θερμοκρασία του ψυκτικού(αισθητήρας 2)(MODE=22 και PID=1139)
  else ok = false;

  stn_com(F("atsh 616af1"));                    //Επανέφερε τον header σε Standard PID Mode
  return ok;
}

/* Συνάρτηση που διαβάζει την τιμή της τάσης της μπαταρίας του οχήματος  */
/* H τιμή της τάσης της μπαταρίας είναι σε ASCII ("12.4V"), οπότε απλά αντιγράφουμε
 * τους χαρακτήρες (έως 5) χωρίς να περάσουν από την hex αποκωδικοποίηση.
 * Γράφουμε σε τοπικό buffer και αντιγράφουμε μόνο σε επιτυχία, ώστε ένα timeout
 * να μη χαλάσει την τιμή που δείχνει η οθόνη. */
bool read_battery_voltage(char *bat)
{
  char tmp[6];
  uint8_t n = 0;
  stn_flush();
  Serial.print(F("atrv\r"));                    // AT command: ATRV
  unsigned long t0 = millis();
  for (;;)
  {
    if (millis() - t0 > STN_RX_TIMEOUT_MS) return false;
    int c = Serial.read();
    if (c < 0) continue;
    if (c == '>') break;                        //'>'=0x3E
    if (c > ' ' && n < 5) tmp[n++] = (char)c;   // strip from spaces/carriage return etc etc
  }
  if (n == 0) return false;
  tmp[n] = '\0';
  memcpy(bat, tmp, n + 1);
  return true;
}

/* Συνάρτηση που διαβάζει το πλήθος των ενεργών κωδικών σφάλματος.
 * Διαβάζει 4 byte: Α, Β, C και D.
 * Το πρώτο byte (A) περιέχει δύο στοιχεία.
 * Το Bit A7 (MSB του byte A, το πρώτο byte) δείχνει εάν το MIL (λαμπάκι βλάβης του κινητήρα)
 * ανάβει ή όχι. Τα bits A6 έως A0 αντιπροσωπεύουν τον αριθμό διαγνωστικών κωδικών βλάβης που
 * επισημαίνονται επί του παρόντος στο ECU.
 * Το δεύτερο, το τρίτο και το τέταρτο bytes (B, C και D) παρέχουν πληροφορίες σχετικά με τη
 * διαθεσιμότητα και την πληρότητα ορισμένων δοκιμών επί του οχήματος. */
bool get_dtc_number(uint8_t *count)
{
  if (!obd_request_01(0x01, 4)) return false;   // AT command: Mode:01 PID: 01
  *count = stn_buffer[2] & 0x7F;                // κρατάμε τα bits A6-A0 (πλήθος), αφαιρώντας το bit A7 (MIL)
  return true;
}

/* Mode 03 – Δίαβασε DTCs (Diagnostic Trouble Codes), δεν χρειάζεται PID.
 * ΧΩΡΙΣ ψηφίο πλήθους απαντήσεων: με πάνω από 3 DTCs η ECU στέλνει πολλά frames και
 * το STN πρέπει να περιμένει το timeout του για να τα μαζέψει όλα.
 * Επιστρέφει πόσα bytes μπήκαν στον stn_buffer. */
uint8_t read_active_dtc()
{
  return stn_com(F("03"));
}

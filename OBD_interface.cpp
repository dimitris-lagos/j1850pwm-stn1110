#include "OBD_Interface.h"

char inByte;
int stn_buffer[16];
void init_STN_Serial_Interface(){
    Serial.begin(115200); //Εκίνηση της σειριακής επικοινωνίας με το STN1110 Baud 115200
}
/* Συνάρτηση που παίρνει σαν όρισμα ένα δείκτη σε πίνακα χαρακτήρων
 που περιέχει την εντολή(PID) προς αποστολή και αποθηκεύη τα δεδομένα
 που λαμβάνονται ως απάντηση απο το STN1110 σε έναν buffer */
void stn_com(char *str)
{ // str είναι η εντολή που στέλνουμε και bytes ο αριθμός των bytes μαζί με το '>'
  int s = 0;
  int i = 0;
  char value[3];
  Serial.print(str);
  Serial.print("\r"); // send end of transmission char.
  while (Serial.available() < 1){}
  inByte = 'A';
  while (inByte != '>'){ //'>'=0x3E
    if (Serial.available() > 0)
    {
      inByte = Serial.read();
      if ((inByte < 91 and inByte > 64) or (inByte >= 45 and inByte < 58))
      { // strip from spaces/carriage return etc etc
        value[i++] = inByte;
        if (i == 2)
        { //Κάθε 2 ASCII χαρακτήρες είναι ένα 16δικο BYTE
          value[i] = '\0';
          i = 0;
          stn_buffer[s++] = hexToDec(value); //Μετατροπή από ASCII σε INT
        }
      }
    }
  }
}

void wait_for_STN_ready(){
while (Serial.available() < 1){} //Αναμονή μέχρι η σειριακή σύνδεση να είναι διαθέσιμη
  char inByte = 'A';
  while (inByte != '>')
  { //Διάβασε τους χαρακτήρες που λαμβάνονται από την σειριακή σύνδεση
    //χαρακτήρας τερματισμού ">"
    inByte = Serial.read();
  }
}

void setup_STN_communication_properties(){
  stn_com("atsp1"); // set protocol jpwm
  stn_com("ats0");  // spaces off on replies from stn1110
  delay(50);
}

/* Συνάρτηση η οποία παίρνει σαν όρισμα ASCII χαρακτήρες(κάθε χαρακτήρας 1 byte) και επιστρέφει τον αντίστοιχο 10δικό.
Το STN1110 δεν στέλνει 16δικο, στέλνει ASCII, άρα κάθε ΗΕΧ ΒΥΤΕ αποστέλνεται από το STN1110 με 2 χαρακτήρες*/
unsigned int hexToDec(String hexString)
{

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++)
  {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57)
      nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70)
      nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102)
      nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας θερμοκρασίας του ψυκτικού υγρού του οχήματος */
int read_coolant_temperature()
{
  stn_com("0105");           // AT command: Mode:01 PID: 05
  return (stn_buffer[2] - 40); //Α-40
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας ταχύτητας του οχήματος */
int read_vehicle_speed()
{
  stn_com("010D");     // AT command: Mode:01 PID: 0D
  return stn_buffer[2]; //Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
  //Α
}


/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του Short Term Fuel Trim της μηχανής  */
float read_sterm()
{
  stn_com("0106"); // AT command: Mode:01 PID: 06
  //Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
  return ((float)stn_buffer[2] / 1.28) - 100.00; //(A/1.28)-100
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του Long Term Fuel Trim της μηχανής */
float read_fterm()
{
  stn_com("0107");                                // AT command: Mode:01 PID: 06
  return ((float)stn_buffer[2] / 1.28) - 100.00; //(A/1.28)-100
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του αισθητήρα οξυγόνου του οχήματος  */
float read_oxvolts()
{
  stn_com("0114");                                // AT command: Mode:01 PID: 14
  //sterm2 = ((float)stn_buffer[3] / 1.28) - 100.0; // (B/1.28)-100
  return ((float)stn_buffer[2] / 200.000);      // A/200
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την κατάσταση κυκλώματος της μηχανής του οχήματος  */
int read_fstat()
{
  stn_com("0103"); // AT command: Mode:01 PID: 03
  return stn_buffer[2];
}

float read_fuel_lvl()
{
  stn_com("atsh e410f1");                                            //Αλλαγή του header σε Ford Manufacturer Extended PID mode(22=physical addressing mode)
  stn_com("2216C1");                                                 //Διάβασε το ποσοστό πλήρωσης % του καυσίμου(MODE=22 και PID=16C1)
  float fuel_lvl = (float)((256 * stn_buffer[3]) + stn_buffer[4]) * 0.003; //Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
  //fuel = fuel_lvl * 0.55;
  stn_com("atsh 616af1"); //Επανέφερε τον header σε Standard PID Mode
  return fuel_lvl;
}

/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας μάζας εισαγωγής αέρα στην μηχανή του οχήματος */
/* Επίσης διενεργεί υπολογισμό κατανάλωσης καυσίμου σε Χιλιόμετρα ανά Λίτρο */
float read_maf()
{
  stn_com("0110"); // AT command: Mode:01 PID: 10
  unsigned int maf1 = (256 * stn_buffer[2]) + stn_buffer[3];
  /* 256*(A+B)/100 */
  /* Τμήμα κώδικα που υπολογίζει την κατανάλωση καυσίμου */
  float maf2 = ((float)maf1) / 100.00;
  return maf2;
}


/* Συνάρτηση που διαβάζει την τιμή της τάσης της μπαταρίας του οχήματος  */
char* read_battery_voltage(char *bat)
{
  // H τιμή της τάσης της μπαταρίας είναι σε ASCII, οπότε απλά αντιγράφουμε τους χαρακτήρες
  int s = 0;
  Serial.print("atrv\r"); // AT command: ATRV
  while (Serial.available() < 1){}
  char inByte = 'A';
  while (inByte != '>')
  { //'>'=0x3E
    if (Serial.available() > 0)
    {
      inByte = Serial.read();
      if ((inByte < 91 and inByte > 64) or (inByte >= 45 and inByte < 58))
      { // strip from spaces/carriage return etc etc
        stn_buffer[s++] = inByte;
      }
    }
  }
  bat[0] = stn_buffer[0];
  bat[1] = stn_buffer[1];
  bat[2] = stn_buffer[2];
  bat[3] = stn_buffer[3];
  bat[4] = stn_buffer[4];
  bat[5] = '\0';

  return bat;
}

void read_Ford_specific_temperatures(FordSpecificTemperatures *fordTemps)
{
  stn_com("atsh e410f1"); //Αλλαγή του header σε Ford Manufacturer Extended PID mode(22=physical addressing mode)
  stn_com("221624");      // Δίαβασε την θερμοκρασία των κυλίνδρων(MODE=22 και PID=1624)

  int temp2 = stn_buffer[3];
  int temp3 = stn_buffer[4];
  if (temp2 == 0)
  {
    (temp2 = temp3 * 2);
  }
  else if (temp2 <= 127)
  {
    temp2 = (temp2 * 512) + (temp3 * 2);
  }
  else
  {
    temp2 = (temp2 - 254) * 512 + temp3 * 2;
  }
  temp2 = (float)(temp2 - 32) * (float)(0.56);

  stn_com("221139"); //Διάβασε την θερμοκρασία του ψυκτικού(αισθητήρας 2)(MODE=22 και PID=1139)
  temp3 = stn_buffer[3];

  stn_com("atsh 616af1"); //Επανέφερε τον header σε Standard PID Mode

  fordTemps->cylinder_temperature = temp2;
  fordTemps->uknown_temperature = temp3;
}

bool check_mil(){
  stn_com("0101");
  if (stn_buffer[2] & 0b10000000){
    return true;
  }
  return false;
}

int get_dtc_number(){
  stn_com("0101");
  if (stn_buffer[2] & 0b10000000){ // if we got MIL ON and active DTCs
    return stn_buffer[2] - 0x80; // byte-128 to find how many active dtcs exist cause of mil bit
  }
  else{
    return stn_buffer[2] & 0b01111111;
  }
}

void read_active_dtc(){
  stn_com("03"); // Mode 03 – Δίαβασε DTCs (Diagnostic Trouble Codes), δεν χρειάζεται PID
}
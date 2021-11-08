#include <SoftwareSerial.h>//Χρήση της βιβλιοθήκης που δίνει την δυνατότητα για σειριακή σύνδεση με υλοποίση επιπέδου λογισμικού
#include <avr/wdt.h>//Χρήση της βιβλιοθήκης υπεύθυνης για την λειτουργία του watchdog timer και συναρτήσεων καθυστέρησης
#include <SPI.h>//Χρήση βιβλιοθήκης για την διασύνδεση της κάρτας SD μέσω του πρωτοκόλλου επικοινωνίας Serial Peripheral Interface (SPI)
#include <avr/pgmspace.h>//Χρήση της EEPROM του ATmega328 για την αποθήκευση των String, ώστε να εξοικονομηθεί χώρος runtime στην sram
#include <LiquidCrystal.h> // Βιβλιοθήκη που διαχειρίζεται την διασύνδεση με την οθόνη LCD
#include <avr/pgmspace.h> //Χρήση της EEPROM του ATmega328 για την αποθήκευση των String, ώστε να εξοικονομηθεί χώρος runtime στην sram
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
 
 Button Digital Pin 16
 LED    Digital Pin 19
 Mosfet Digital Pin 9
*/
/* Τμήμα δήλωσης global μεταβλητών */
const int numRows = 2; //2 γραμμές οθόνης
const int numCols = 16; //16 στοιχείων
char inByte;
int temp=0; //μεταβλητή αποθήκευσης θερμοκρασίας ψυκτικού υγρού
int temp2=0; //μεταβλητή αποθήκευσης θερμοκρασίας κινητήρα
int temp3=0; //μεταβλητή αποθήκευσης θερμοκρασίας Κυλίνδρων
int airtemp=0; //μεταβλητή αποθήκευσης θερμοκρασίας αέρα
int spd=0;//μεταβλητή αποθήκευσης ταχύτητας οχήματος
float fuel=0.00;//μεταβλητή αποθήκευσης τιμής ποσότητας καυσίμου
float fuel_lvl=0.00000;
float f2=0.00;//μεταβλητή αποθήκευσης
int k=0;
int s=0;
int i=0;
int j=0;
int counter=0;
unsigned int active_dtcs=0;
/*μεταβλητές αποθήκευσης αποκωδηκοποιημένων δεδομένων κατάστασης του οχήματος*/
int fstat;
unsigned int maf1=0;
float maf2=0.00000;
float sterm=0.00;
float fterm=0.00;
float oxvolt=0.000;
float sterm2=0.0;
float cons=0.00000;
float total_cons=0.00000;
float temp_cons=0.00000;
float real_time_cons=0.00000;
float milage=0.00;
int stn_buffer[30];
char bat[6];
int menu=0;
char value[3];
int val=0;
int threshold=95;
int temp_thresh=95;
int analogPin = A3; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V

/* Αρχικοποίηση βιβλιοθήκης που διαχειρίζεται την επικοινωνία με την οθόνη,
 με τους ακροδέκτες του Arduino που συνδέονται στην οθόνη  */
LiquidCrystal lcd(12, 13, 5, 14, 15, 16);//LiquidCrystal(rs, enable, d4, d5, d6, d7) 
int button1State = 0;
int button2State = 0;

/* Η συνάρτηση που αρχικοποιεί τις βασικές ρυθμίσεις για την διασύνδεση
 των περιφερειακών και των λειτουργιών του προγράμματος */
void setup() {
  pinMode(9, OUTPUT);//Ορισμός ακροδέκτη 9 ως έξοδο ventilator mosfet
  digitalWrite(9, LOW);//ventilator off
  Serial.begin(115200);//Εκίνηση της σειριακής επικοινωνίας με το STN1110 Baud 115200
  // set up the LCD's number of columns and rows:
  lcd.begin(numCols, numRows);//Ορισμός γραμμών και στηλών της οθόνης(2x16)
  pinMode(19, OUTPUT);//Ορισμός ακροδέκτη 19 ως έξοδο(LED 1)
  

  pinMode(2,INPUT_PULLUP);//Ορισμός ακροδέκτη 2 ως είσοδο, ενεργοποιώντας την αντίσταση τερματισμού(Κουμπί 1)
  pinMode(3,INPUT_PULLUP);//Ορισμός ακροδέκτη 3 ως είσοδο, ενεργοποιώντας την αντίσταση τερματισμού(Κουμπί 2)
  attachInterrupt(digitalPinToInterrupt(2), button1, FALLING);/*Ορισμός διακοπής στην αρνητική ακμή του σήματος στον ακροδέκτη 2*/
  attachInterrupt(digitalPinToInterrupt(3), button2, FALLING);/*Ορισμός διακοπής στην αρνητική ακμή του σήματος στον ακροδέκτη 3*/
  //set timer1 interrupt at 1Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  
  
  lcd.clear();
  digitalWrite(19, HIGH);//LED on
  delay(1000);// Καθυστέρηση 1/2"
  lcd.setCursor(0, 0);/* μετακίνηση του κέρσορα στην πρώτη γραμμή και πρώτη στήλη της οθόνης */
  lcd.write("Jiml OBD2");/* Γράψε στην οθόνη το κείμενο εισαγωγής*/
  digitalWrite(19, LOW);//LED off
  delay(1000);
  while(Serial.available() < 1){}//Αναμονή μέχρι η σειριακή σύνδεση να είναι διαθέσιμη
  inByte='A';
  while(inByte != '>'){ //Διάβασε τους χαρακτήρες που λαμβάνονται από την σειριακή σύνδεση
  //χαρακτήρας τερματισμού ">"
      inByte = Serial.read();} 
  lcd.setCursor(0, 1);
  stn_com("atsp1\r"); //set protocol jpwm 
  stn_com("ats0\r"); //spaces off on replies from stn1110
  delay(50);
  read_fuel_lvl();
  lcd.print(fuel_lvl,1);
  lcd.print(" %F  ");
  lcd.print(fuel,1);
  lcd.print(" L");
  //read_temp();
  //for(int i=0;i<s;i++){lcd.print(stn_buffer[i]);lcd.print(" ");}
  menu=0;
  delay(5000);//περίμενε 5"
  sei();//ενεργοποίηση της εξυπηρέτησης των διακοπών
}
ISR(TIMER1_COMPA_vect){
  if(counter==0){counter=1;}
total_cons+=(temp_cons/(float)counter);
temp_cons=0;
counter=0;
}
void loop() {

/* Το πρόγραμμα περιέχει 4 σελίδες προβολής πληροφοριών στην οθόνη.
 Το κουμπί 1 αλλάζει μεταξύ των σελίδων 1 ως 3.
 Το κουμπί 2 ενεργοποιεί την σελίδα 4 που περιέχει τους κωδικούς σφαλμάτων */
    if (button1State == 1) {delay(100);++menu;button1State =0;if(menu==4){menu=0;}sei();}/* Αν πατηθεί το κουμπί 1, πήγαινε στην επόμενη σελίδα  */
    if (button2State == 1) {delay(100);button2State =0;read_dtc();}/* Αν πατηθεί το κουμπί 1, πήγαινε στην σελίδα κωδικών σφαλμάτων */
    read_temp_threshold();
    switch (menu){
    case 0://Επίλεξε σελίδα πληροφοριών 1
         read_temp();//Διάβασε την θερμοκρασία του οχήματος
         read_cons();
         read_bat();//Διάβασε την τάση της μπαταρίας
         send2lcd();//Στείλε τα δεδομένα στην οθόνη
      break;
    case 1://Επίλεξε σελίδα πληροφοριών 2
     read_cons();
     read_fuel_lvl();
     read_fstat();//Διάβασε κατάσταση κυκλώματος κινητήρα
     read_temp();//Διάβασε την θερμοκρασία του οχήματος
         send2lcd2();//Στείλε τα δεδομένα στην οθόνη
      break;
    case 2://Επίλεξε σελίδα πληροφοριών 3
         read_temp();//Διάβασε την θερμοκρασία του οχήματος
         read_cyl_temp();//Διάβασε την θερμοκρασία των κυλίνδρων
     read_cons();
         send2lcd3();//Στείλε τα δεδομένα στην οθόνη
      break;
   case 3://Επίλεξε σελίδα πληροφοριών 3
     read_cons();
     read_fterm();
     read_oxvolts();
     read_temp();//Διάβασε την θερμοκρασία του οχήματος
         send2lcd4();//Στείλε τα δεδομένα στην οθόνη
      break;
        }
    delay(50); 
}
        


void button1(){cli();button1State =1;} /* Συνάρτηση που διαχειρίζεται το πάτημα του κουμπιού 1 */
void button2(){button2State =1;}/* Συνάρτηση που διαχειρίζεται το πάτημα του κουμπιού 2 */


/* Συνάρτηση που παίρνει σαν όρισμα ένα δείκτη σε πίνακα χαρακτήρων
 που περιέχει την εντολή(PID) προς αποστολή και αποθηκεύη τα δεδομένα
 που λαμβάνονται ως απάντηση απο το STN1110 σε έναν buffer */
void stn_com(char* str){ //str είναι η εντολή που στέλνουμε και bytes ο αριθμός των bytes μαζί με το '>'
    s=0;
  int i=0;
  Serial.print(str);
    while(Serial.available() < 1){}
  inByte = 'A';
  while(inByte != '>'){ //'>'=0x3E
    if(Serial.available() > 0){
      inByte=Serial.read();
      if((inByte<91 and inByte>64) or (inByte>=45 and inByte<58)) {// strip from spaces/carriage return etc etc
        value[i++]=inByte;
        if(i==2){  //Κάθε 2 ASCII χαρακτήρες είνανι ένα 16δικο BYTE
          value[i]='\0';
          i=0;
          stn_buffer[s++] = hexToDec(value); //Μετατροπή από ASCII σε INT
        }
      
      }
    }
  }
}

void read_temp_threshold(){
  val = analogRead(analogPin);
  if(val>=0 && val<85) threshold=50;
  else if(val>=86 && val<170) threshold=92;
  else if(val>=171 && val<255) threshold=93;
  else if(val>=256 && val<340) threshold=94;
  else if(val>=341 && val<425) threshold=95;
  else if(val>=426 && val<510) threshold=96;
  else if(val>=511 && val<595) threshold=97;
  else if(val>=596 && val<680) threshold=98;
  else if(val>=681 && val<765) threshold=99;
  else if(val>=766 && val<850) threshold=100;
  else if(val>=851 && val<935) threshold=101;
  else if(val>=936) threshold=102;
  if(threshold!=temp_thresh){temp_thresh=threshold;thresh_change();}    
}


void thresh_change(){
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(threshold,DEC);
  lcd.print("C Threshold");
  delay(200);
  
}
/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας θερμοκρασίας του ψυκτικού υγρού του οχήματος */
void read_temp(){
       stn_com("0105\r");//AT command: Mode:01 PID: 05
       temp=stn_buffer[2]-40;//Α-40
       if(temp>threshold && temp<250){digitalWrite(19, HIGH);digitalWrite(9, HIGH);}/* Αν η θερμοκρασία του ψυκτικού είναι μεγαλύτερη του threshold, άναψε το LED 2 και άνοιξε το ventilator*/
       else{digitalWrite(19, LOW);digitalWrite(9, LOW);}
 }


/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας ταχύτητας του οχήματος */
  void read_spd(){
    stn_com("010D\r");//AT command: Mode:01 PID: 0D
    spd=stn_buffer[2];//Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
    //Α
  }
  
/* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή της τρέχουσας μάζας εισαγωγής αέρα στην μηχανή του οχήματος */
/* Επίσης διενεργεί υπολογισμό κατανάλωσης καυσίμου σε Χιλιόμετρα ανά Λίτρο */
  void read_maf(){
    stn_com("0110\r");//AT command: Mode:01 PID: 10
    maf1=(256*stn_buffer[2])+stn_buffer[3];
  /* 256*(A+B)/100 */
    /* Τμήμα κώδικα που υπολογίζει την κατανάλωση καυσίμου */
    maf2=((float)maf1)/100.00;
  }
  
  void read_cons(){
  read_maf();
  read_spd();
  real_time_cons=(maf2/10437.00000)*(1+(sterm/100));
  if(spd>0){
    cons=(spd/3600.00)/real_time_cons;//(spd/3600)*((14,7*710)/maf2);
    milage=100.00/cons;
    }else{cons=0.00;milage=0.00;} 
  read_sterm(); //Διαβάζουμε έδω το short term fuel trim για να εφαρμοστεί στον επόμενο κύκλο
  cli();
  temp_cons +=real_time_cons;
  ++counter;
  sei();
  }
  
  /* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του Short Term Fuel Trim της μηχανής  */
  void read_sterm(){
    stn_com("0106\r");//AT command: Mode:01 PID: 06
  //Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
    sterm=((float)stn_buffer[2]/1.28)-100.00;//(A/1.28)-100
  }
  
  /* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του Long Term Fuel Trim της μηχανής */
  void read_fterm(){
    stn_com("0107\r");//AT command: Mode:01 PID: 06
    fterm=((float)stn_buffer[2]/1.28)-100.00;//(A/1.28)-100

  }

  /* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την τιμή του αισθητήρα οξυγόνου του οχήματος  */
    void read_oxvolts(){
    stn_com("0114\r");//AT command: Mode:01 PID: 14
    oxvolt=((float)stn_buffer[2]/200.000);// A/200
    sterm2=((float)stn_buffer[3]/1.28)-100.0;// (B/1.28)-100

  }
  
  /* Συνάρτηση που διαβάζει την τιμή της τάσης της μπαταρίας του οχήματος  */
  void read_bat(){
  //H τιμή της τάσης της μπαταρίας είναι σε ASCII, οπότε απλά αντιγράφουμε τους χαρακτήρες
  s=0;
  Serial.print("atrv\r");//AT command: ATRV
    while(Serial.available() < 1){}
  inByte = 'A';
  while(inByte != '>'){ //'>'=0x3E
    if(Serial.available() > 0){
      inByte=Serial.read();
      if((inByte<91 and inByte>64) or (inByte>=45 and inByte<58)) {// strip from spaces/carriage return etc etc
        stn_buffer[s++]=inByte;
      }
    }
  }
      bat[0]=stn_buffer[0];
      bat[1]=stn_buffer[1];
      bat[2]=stn_buffer[2];
      bat[3]=stn_buffer[3];
      bat[4]=stn_buffer[4];
      bat[5]='\0';
  }

    /* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την κατάσταση κυκλώματος της μηχανής του οχήματος  */
  void read_fstat(){
    stn_com("0103\r");//AT command: Mode:01 PID: 03
  fstat=stn_buffer[2];
  }



  void read_cyl_temp(){
        stn_com("atsh e410f1\r"); //Αλλαγή του header σε Ford Manufacturer Extended PID mode(22=physical addressing mode)
        stn_com("221624\r");// Δίαβασε την θερμοκρασία των κυλίνδρων(MODE=22 και PID=1624)
    
        temp2=stn_buffer[3];
        temp3=stn_buffer[4];
        if(temp2==0){(temp2=temp3*2);}
        else if(temp2<=127){temp2=(temp2*512)+(temp3*2);}
        else{temp2=((temp2-254)*512)+temp3*2;}
        temp2=(float)(temp2-32)*(float)(0.56); //Fahrenheit to Celsius

        stn_com("221139\r"); //Διάβασε την θερμοκρασία του ψυκτικού(αισθητήρας 2)(MODE=22 και PID=1139)
        temp3=stn_buffer[3];
    
        stn_com("atsh 616af1\r"); //Επανέφερε τον header σε Standard PID Mode

  }
  void read_fuel_lvl(){
    stn_com("atsh e410f1\r"); //Αλλαγή του header σε Ford Manufacturer Extended PID mode(22=physical addressing mode)
    stn_com("2216C1\r"); //Διάβασε το ποσοστό πλήρωσης % του καυσίμου(MODE=22 και PID=16C1)
    fuel_lvl=(float)((256*stn_buffer[3])+stn_buffer[4])*0.003;//Μετατροπή της τιμής από δεκαεξαδικό σε δεκαδικό
    fuel=fuel_lvl*0.55;
    stn_com("atsh 616af1\r"); //Επανέφερε τον header σε Standard PID Mode  
  }

/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της πρώτης σελίδας στην οθόνη */
/*Αναλόγως πόσα ψηφία έχει η τιμή, βάζει τον κέρσορα στη σωστή θέση της 
οθόνης ώστε οι πληροφορίες να είναι ευανάγνωστες. Επίσης εκτυπώνει τις μονάδες
των τιμών στα σωστά σημεία*/
  void send2lcd(){
    i=0;
    lcd.clear();
    if(temp<=-10){}
    else if(temp<0 and temp>-10){lcd.setCursor(1, 0);i=1;}
    else if(temp<10){lcd.setCursor(2, 0);i=2;}
    else if(temp<100){lcd.setCursor(1, 0);i=1;}
    lcd.print(temp,DEC);
    lcd.print("C  ");
    if(milage<10){lcd.setCursor(7, 0);}
    else{lcd.setCursor(6, 0);}
    lcd.print(milage,1);
    lcd.print(" L/100");
    lcd.setCursor(0, 1);
    lcd.write(bat);
    lcd.print("  ");
  if(cons<10){lcd.print(" ");}
    lcd.print(cons,1);
    lcd.print(" Km/L");
    
  }
/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της δεύτερης σελίδας στην οθόνη */
  void send2lcd2(){
    i=0;
    lcd.clear();
    if(total_cons<10){lcd.setCursor(2, 0);}
    else if(total_cons<100){lcd.setCursor(1, 0);}
    lcd.print(total_cons,3);
    lcd.print("L Cons ");
    lcd.setCursor(0, 1);
    if(fuel_lvl<10){lcd.setCursor(1, 1);}
    lcd.print(fuel_lvl,1);
    lcd.print(" % ");
    lcd.print(fuel,1);
    lcd.print(" L");
    lcd.setCursor(14, 1);
    lcd.print(fstat);
  }
/* Συνάρτηση η οποία εμφανίζει τα δεδομένα της τρίτης σελίδας στην οθόνη */
    void send2lcd3(){
    lcd.clear();
    lcd.print("C1 ");
    lcd.print(temp,DEC);
    lcd.print("C ");
    lcd.print("C2 ");
    lcd.print(temp3,DEC);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Cyl ");
    lcd.print(temp2,DEC);
    lcd.print("C");

  }
   void send2lcd4(){
    lcd.clear();
    lcd.print(maf2,2);
    lcd.print("G/s ");  
    lcd.print(oxvolt,3);
    lcd.print("V");
    lcd.setCursor(0, 1);
    lcd.print(sterm,2);
    lcd.print("% ");
    lcd.print(fterm,2);
    lcd.print("%");

    
  } 
  
  /* Συνάρτηση που διαβάζει και αποκωδηκοποιεί την κατάσταση σφαλμάτων του οχήματος  */
  void read_dtc(){
    i=1;
  char tmp[2];
  /* Διαβάζει 4 byte: Α, Β, C και D
  Το πρώτο byte (A) περιέχει δύο στοιχεία. 
  Το Bit A7 (MSB του byte A, το πρώτο byte) δείχνει εάν το MIL (λαμπάκι βλάβης του κινητήρα) 
  ανάβει ή όχι. Τα bits A6 έως A0 αντιπροσωπεύουν τον αριθμό διαγνωστικών κωδικών βλάβης που 
  επισημαίνονται επί του παρόντος στο ECU.
  Το δεύτερο, το τρίτο και το τέταρτο bytes (B, C και D) παρέχουν πληροφορίες σχετικά με τη
  διαθεσιμότητα και την πληρότητα ορισμένων δοκιμών επί του οχήματος*/
    lcd.clear();
  lcd.setCursor(0, 0);
  stn_com("0101\r");
  if(stn_buffer[2] & 0b10000000){ //if we got MIL ON and active DTCs
    active_dtcs= stn_buffer[2]-0x80; // byte-128 to find how many active dtcs exist cause of mil bit
    lcd.print("MIL is ON!");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);}
  else active_dtcs=stn_buffer[2]& 0b01111111;
  if(active_dtcs){
    lcd.print(active_dtcs);
    lcd.print(" DTCs FOUND!");
    delay(3000);//Πάγωσε την οθόνη για 3" ώστε ο χρήστης να προλάβει να διαβάσει
    stn_com("03\r"); //Mode 03 – Δίαβασε DTCs (Diagnostic Trouble Codes), δεν χρειάζεται PID
    lcd.clear();
    lcd.setCursor(0, 0);
    for(int j=1; j<=active_dtcs;j++){// byte#0 is only the mode reply-not needed
      if(stn_buffer[i] <= 0x3F){
        lcd.print("P");
        sprintf(tmp,"%02x", stn_buffer[i]);
        lcd.print(tmp);
        sprintf(tmp,"%02x", stn_buffer[i+1]);
        lcd.print(tmp);
        lcd.print(" ");
      } 
      else if(stn_buffer[i] <= 0x7F){
        lcd.print("C");
        lcd.print((stn_buffer[i]>>4)-4);
        lcd.print(stn_buffer[i]&&0b00001111);
        sprintf(tmp,"%02x", stn_buffer[i+1]);
        lcd.print(tmp);
        lcd.print(" ");       
      }
      else if(stn_buffer[i] <= 0xBF){
        lcd.print("B");
        lcd.print((stn_buffer[i]>>4)-8);
        lcd.print(stn_buffer[i]&&0b00001111);
        sprintf(tmp,"%02x", stn_buffer[i+1]);
        lcd.print(tmp);
        lcd.print(" ");
      }
      else {
        lcd.print("U");
        lcd.print((stn_buffer[i]>>4)-12);
        lcd.print(stn_buffer[i]&&0b00001111);
        sprintf(tmp,"%02x", stn_buffer[i+1]);
        lcd.print(tmp);
        lcd.print(" ");
      }
      i=i+2;
      if(j==3){lcd.setCursor(0, 1);}
    }
    
  delay(10000);//Πάγωσε την οθόνη για 3" ώστε ο χρήστης να προλάβει να διαβάσει
  
  }
  else{lcd.print("NO DTCs FOUND");delay(2000);}
  }
  
   /* Συνάρτηση η οποία παίρνει σαν όρισμα ASCII χαρακτήρες(κάθε χαρακτήρας 1 byte) και επιστρέφει τον αντίστοιχο 10δικό.
  Το STN1110 δεν στέλνει 16δικο, στέλνει ASCII, άρα κάθε ΗΕΧ ΒΥΤΕ αποστέλνεται από το STN1110 με 2 χαρακτήρες*/
unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

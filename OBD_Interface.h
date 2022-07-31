#include <SPI.h>            //Χρήση βιβλιοθήκης για την διασύνδεση της κάρτας SD μέσω του πρωτοκόλλου επικοινωνίας Serial Peripheral Interface (SPI)

/* End of Transmission character for the STN */
#define EOT_CHAR '>' 



typedef struct FordSpecificTemperatures{
  int cylinder_temperature;
  int uknown_temperature;

}FordSpecificTemperatures; 

unsigned int hexToDec(String hexString);
void stn_com(char *str);
void wait_for_STN_ready();
void setup_STN_communication_properties();
int read_coolant_temperature();
int read_vehicle_speed();
float read_sterm();
float read_fterm();
float read_oxvolts();
int read_fstat();
float read_fuel_lvl();
float read_maf();
char* read_battery_voltage(char *bat);
void init_STN_Serial_Interface();
void read_Ford_specific_temperatures(FordSpecificTemperatures *fordTemps);
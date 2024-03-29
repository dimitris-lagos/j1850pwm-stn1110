#ifndef __obd_interface_h_included__
#define __obd_interface_h_included__
#include <SPI.h>            //Χρήση βιβλιοθήκης για την διασύνδεση της κάρτας SD μέσω του πρωτοκόλλου επικοινωνίας Serial Peripheral Interface (SPI)
#include "global.h"
/* End of Transmission character for the STN */
#define EOT_CHAR '>' 

typedef struct FordSpecificTemperatures{
  int cylinder_temperature;
  int uknown_temperature;

}FordSpecificTemperatures; 

//global variable holding the last data stn sent
extern int stn_buffer[16];

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
bool check_mil();
int get_dtc_number();
void read_active_dtc();
#endif
/*
 * ESPDataLogger.h
 *
 * Created on: May 26, 2020
 * Author: controllerstech
 */

#ifndef INC_ESP8266_CODE_H_
#define INC_ESP8266_CODE_H_

void ESP_Init (char *SSID, char *PASSWD);
void ESP_Send_Data (char *APIkey, int Field_num, float value);
void ESP_Send_Multi (char *APIkey, int numberoffileds, float value[]);

#endif /* INC_ESP8266_CODE_H_ */

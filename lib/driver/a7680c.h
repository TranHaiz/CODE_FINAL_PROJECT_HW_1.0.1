/**
 * @file       a7680c.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    major.minor.patch
 * @date       2026-02-08
 * @author     Hai Tran
 *
 * @brief     Module SIM A7680C driver
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _A7680C_H_
#define _A7680C_H_
/* Includes ----------------------------------------------------------- */
#include "common_type.h"

/* Public defines ----------------------------------------------------- */

// ===================
// Basic AT commands and responses
// ===================
#define SIM_CMD_AT                     "AT"    // Test module connection
#define SIM_CMD_ATE0                   "ATE0"  // Disable command echo
#define SIM_CMD_ATE1                   "ATE1"  // Enable command echo

#define SIM_RESP_OK                    "OK"
#define SIM_RESP_ERROR                 "ERROR"

// ===================
// Status check commands
// ===================
#define SIM_CMD_CHECK_SIM              "AT+CPIN?"     // Check SIM card status
#define SIM_CMD_CHECK_NETWORK          "AT+CREG?"     // Check network registration
#define SIM_CMD_CHECK_GPRS             "AT+CGATT?"    // Check GPRS/LTE attachment
#define SIM_CMD_CHECK_PDP              "AT+CGDCONT?"  // Check PDP context

#define SIM_RESP_CPIN_READY            "+CPIN: READY"
#define SIM_RESP_CREG_REGISTERED       "+CREG: 0,1"
#define SIM_RESP_CREG_ROAM             "+CREG: 0,5"
#define SIM_RESP_CGATT_ATTACHED        "+CGATT: 1"
#define SIM_RESP_CGDCONT_OK            "+CGDCONT:"

// ===================
// Network configuration commands
// ===================
#define SIM_CMD_SET_APN                "AT+CGDCONT=1,\"IP\",\"v-internet\""  // Set APN for Viettel
#define SIM_CMD_APN_SET(apn)           "AT+CGDCONT=1,\"IP\",\"" apn "\""     // Set custom APN
#define SIM_CMD_SSL_VERSION            "AT+CSSLCFG=\"sslversion\",0,4"       // Set SSL version to TLS 1.2
#define SIM_CMD_SSL_AUTHMODE           "AT+CSSLCFG=\"authmode\",0,0"         // Disable SSL certificate check
#define SIM_CMD_SSL_IGNORE_TIME        "AT+CSSLCFG=\"ignorelocaltime\",0,1"  // Ignore local time check

// ===================
// HTTP service commands
// ===================
#define SIM_CMD_HTTP_INIT              "AT+HTTPINIT"                                   // Initialize HTTP service
#define SIM_CMD_HTTP_TERM              "AT+HTTPTERM"                                   // Terminate HTTP service
#define SIM_CMD_HTTP_SET_URL(url)      "AT+HTTPPARA=\"URL\",\"" url "\""               // Set HTTP URL
#define SIM_CMD_HTTP_SET_CONTENT       "AT+HTTPPARA=\"CONTENT\",\"application/json\""  // Set Content-Type to JSON

// ===================
// HTTP data transfer commands
// ===================
#define SIM_CMD_HTTPDATA(len, timeout) "AT+HTTPDATA=" #len "," #timeout  // Prepare to send HTTP data
#define SIM_CMD_HTTPACTION_GET         "AT+HTTPACTION=0"                 // HTTP GET request
#define SIM_CMD_HTTPACTION_POST        "AT+HTTPACTION=1"                 // HTTP POST request
#define SIM_CMD_HTTPACTION_PUT         "AT+HTTPACTION=4"                 // HTTP PUT request
#define SIM_CMD_HTTPACTION_PATCH       "AT+HTTPACTION=5"                 // HTTP PATCH request
#define SIM_CMD_HTTPREAD(offset, len)  "AT+HTTPREAD=" #offset "," #len   // Read HTTP response

// ===================
// Example Firebase URLs
// ===================
#define SIM_FIREBASE_URL_DEVICE        "https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201.json"
#define SIM_FIREBASE_URL_BATTERY \
  "https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201/Battery%20level.json"
#define SIM_FIREBASE_URL_POSITION "https://tracking-project-cf57e-default-rtdb.firebaseio.com/Device%201/Position.json"
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif /*End file _A7680C_H_*/

/* End of file -------------------------------------------------------- */

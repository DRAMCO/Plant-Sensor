/*
 * app_gnss.h
 *
 *  Created on: Mar 11, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_GNSS_H_
#define APP_GNSS_H_

#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif
#include "main.h"

#define GNSS_ADDRESS             0x74

/**
 * @brief Constant that indicates the maximum lenght of NMEA message field.
 */
#define MAX_MSG_LEN 48/* 32 */ /* was 19 */

/**
 * @brief Constant that indicates the maximum number of satellites.
 */
#define MAX_SAT_NUM 12

/**
 * @brief Constant that indicates the maximum lenght of a string.
 */
#define MAX_STR_LEN 32

/**
 * @brief Constant that indicates the maximum number of sat per GSV message.
 */
#define GSV_MSG_SATS 4

/**
 * @brief Enumeration structure that contains the two states of a parsing process
 */
typedef enum {
  PARSE_FAIL = 1, /**< Fail status */
  PARSE_SUCC = 0  /**< Success status */
} ParseStatus_t;

/**
 * @brief Data structure that contains the UTC information
 */
typedef struct {
  int32_t utc;  /**< UTC Info */
  int16_t hh;   /**< Hours */
  int16_t mm;   /**< Minutes */
  int16_t ss;   /**< Seconds */
} UTC_Info_t;

/**
 * @brief Data structure that contains the coordinates information
 */
typedef struct {
  double  lat; /**< Latitude */
  double  lon; /**< Longitude */
  double  alt; /**< Altitude */
  uint8_t ns;  /**< Nord / Sud latitude type */
  uint8_t ew;  /**< East / West longitude type */
  uint8_t mis; /**< Altitude unit misure */
} Coords_t;

/**
 * @brief Data structure that contains the Gps geoids information
 */
typedef struct {
  int16_t height; /**< Geoid height */
  uint8_t mis;    /**< Geoid height misure unit */
} Geoid_Info_t;

/**
 * @brief Data structure that contains all the Recommended Minimum Specific GPS/Transit data.
 */
typedef struct {
  UTC_Info_t utc;         /**< UTC Time */
  uint8_t    status;      /**< 'A' = valid, 'V' = Warning */
  Coords_t   xyz;         /**< Coords data member */
  float      speed;       /**< Speed over ground in knots */
  float      trackgood;   /**< Course made good */
  int32_t    date;        /**< Date of Fix */
  float      mag_var;     /**< Magnetic Variation */
  uint8_t    mag_var_dir; /**< Magnetic Variation Direction */
  uint32_t   checksum;    /**< Checksum of the message bytes */
} GPRMC_Info_t;

/**
 * @brief Data structure that contains all of the information about the GPS position
 */
typedef struct {
  UTC_Info_t   utc;       /**< UTC Time */
  Coords_t     xyz;	      /**< Coords data member */
  double       acc;       /**< GPS Accuracy */
  int16_t      sats;	  /**< Number of satellities acquired */
  uint8_t      valid;     /**< GPS Signal fix quality */
  Geoid_Info_t geoid;	  /**< Geoids data info member */
  int16_t      update;    /**< Update time from the last acquired GPS Info */
  uint32_t     checksum;  /**< Checksum of the message bytes */
} GPGGA_Info_t;

/**
 * @brief Data structure that contains all of the information about the GSA satellites
 */
typedef struct {
  uint8_t constellation[MAX_STR_LEN]; /**< Constellation enabled: GPGSA (GPS), GLGSA (GLONASS), GAGSA (GALILEO), BDGSA (BEIDOU), GNGSA (more than one) */
  uint8_t operating_mode;             /**< Operating Mode: 'M' = Manual, 'A' = Auto (2D/3D) */
  int16_t current_mode;               /**< Current Mode: 1. no fix available, 2. 2D, 3. 3D */
  int32_t sat_prn[MAX_SAT_NUM];       /**< Satellites list used in position fix (max N 12) */
  double pdop;	                      /**< Position Dilution of Precision, max: 99.0 */
  double hdop;                        /**< Horizontal Dilution of Precision, max: 99.0 */
  double vdop;                        /**< Vertical Dilution of Precision, max: 99.0 */
  uint32_t checksum;                  /**< Checksum of the message bytes */
} GSA_Info_t;

/**
 * @brief Data structure that contains the GSV information
 */
typedef struct {
  int16_t prn;   /**< PRN */
  int16_t elev;  /**< Elevation of satellite in degree, 0 ... 90 */
  int16_t azim;  /**< Azimuth of satellite in degree, ref. "North, " 0 ... 359 */
  int16_t cn0;   /**< Carrier to noise ratio for satellite in dB, 0 ... 99 */
} GSV_SAT_Info_t;

/**
 * @brief Data structure that contains all of the information about the GSV satellites
 */
typedef struct {
  uint8_t constellation[MAX_STR_LEN];    /**< Constellation enabled: GPGSV (GPS), GLGSV (GLONASS), GAGSV (GALILEO), BDGSV (BEIDOU), QZGSV (QZSS), GNGSV (more than one) */
  int16_t amount;                        /**< Total amount of GSV messages, max. 3 */
  int16_t number;                        /**< Continued GSV number of this message */
  int16_t tot_sats;                      /**< Total Number of Satellites in view, max. 12 */
  int16_t current_sats;
  GSV_SAT_Info_t gsv_sat_i[MAX_SAT_NUM]; /**< Satellite info  */
  uint32_t checksum;	                 /**< Checksum of the message bytes */
} GSV_Info_t;


void GNSSThreadInit();
void StartGNSSThread();
void scan_utc(uint8_t *pUTCStr, UTC_Info_t *pUTC);
uint32_t nmea_checksum(const uint8_t buf[]);
uint32_t char2int(uint8_t c);

/**
 * @brief  Function that makes the parsing of the $GPRMC NMEA string with Recommended Minimum Specific GPS/Transit data.
 * @param  pGPRMCInfo    Pointer to a GPRMC_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_t NMEA_ParseGPRMC(GPRMC_Info_t *pGPRMCInfo, char NMEA[]);
void GNSS_DATA_GetGPRMCInfo(GPRMC_Info_t *pGPRMCInfo);

/**
 * @brief  Function that makes the parsing of the $GPGGA NMEA string with all Global Positioning System Fixed data.
 * @param  pGPGGAInfo     Pointer to GPGGA_Info_t struct
 * @param  NMEA	          NMEA string read by the Gps expansion
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_t NMEA_ParseGPGGA(GPGGA_Info_t *pGPGGAInfo, char NMEA[]);
void GNSS_DATA_GetGPGGAInfo(GPGGA_Info_t *pGPGGAInfo);

/**
 * @brief  Function that makes the parsing of the $GSA NMEA string.
 * @param  pGSAInfo      Pointer to a GSA_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_t NMEA_ParseGSA(GSA_Info_t *pGSAInfo, char NMEA[]);
void GNSS_DATA_GetGSAInfo(GSA_Info_t *pGSAInfo);


void GNSS_DATA_GetGSVInfo(GSV_Info_t *pGSVInfo);


#endif /* APP_GNSS_H_ */

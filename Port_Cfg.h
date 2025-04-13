#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Total number of pins */
#define PORT_NUMBER_OF_PORT_PINS  (43U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT  (STD_ON)

/* Pre-processor switch to enable / disable the use of the function Port_SetPinDirection() */
#define PORT_SET_PIN_DIRECTION_API  (STD_ON)

/* Pre-processor switch to enable / disable the use of the function Port_SetPinMode() */
#define PORT_SET_PIN_MODE_API  (STD_ON)

/* Pre-processor switch to enable / disable the API to read out the modules version information */
#define PORT_VERSION_INFO_API  (STD_ON)

/* Port IDs */
#define PORT_A  (0U)
#define PORT_B  (1U)
#define PORT_C  (2U)
#define PORT_D  (3U)
#define PORT_E  (4U)
#define PORT_F  (5U)

/* Pin IDs */
#define PIN_0  (0U)
#define PIN_1  (1U)
#define PIN_2  (2U)
#define PIN_3  (3U)
#define PIN_4  (4U)
#define PIN_5  (5U)
#define PIN_6  (6U)
#define PIN_7  (7U)

#endif /* PORT_CFG_H_ */

 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"
#include "Std_Types.h"
#include "Port_Regs.h"
#include "Port_Cfg.h"


/* Id for the company in the AUTOSAR */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (120U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)


/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port_Init  */
#define Port_Init_SID  (uint8)0x00

/* Service ID for Port_SetPinDirection */
#define Port_SetPinDirection_SID  (uint8)0x01

/* Service ID for Port_RefreshPortDirection */
#define Port_RefreshPortDirection_SID  (uint8)0x02

/* Service ID for Port_GetVersionInfo */
#define Port_GetVersionInfo_SID  (uint8)0x03

/* Service ID for Port_SetPinMode */
#define Port_SetPinMode_SID  (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN  (uint8)0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE  (uint8)0x0B

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG  (uint8)0x0C

/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE  (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE   (uint8)0x0E

/* API service called without module initialization */
#define PORT_E_UNINIT (uint8)0x0F

/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER (uint8)0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/*  shall be used for the symbolic name of a Port Pin. */
typedef uint8 Port_PinType;

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

typedef enum
{
    PORT_PIN_MODE_ADC,  //Port Pin used by ADC
    PORT_PIN_MODE_DIO,  //Port Pin configured for DIO. It shall be used under control of the DIO driver.
    PORT_PIN_MODE_SPI = 2,  //Port Pin used by SPI
    PORT_PIN_MODE_PWM = 4,  //Port Pin used by PWM 4,5(A,F)
    PORT_PIN_MODE_DIO_GPT = 7,  //Port Pin configured for DIO. It shall be used under control of the general purpose timer driver.
    PORT_PIN_MODE_DIO_WDG = 7,  //Port Pin configured for DIO. It shall be used under control of the watchdog driver.
    PORT_PIN_MODE_ICU = 7,  //Port Pin used by ICU
    PORT_PIN_MODE_CAN = 8,  //Port Pin used for CAN
    PORT_PIN_MODE_FLEXRAY = 9,  //Port Pin used for FlexRay, not supported
    PORT_PIN_MODE_LIN = 9,  //Port Pin used for LIN, not supported
    PORT_PIN_MODE_MEM = 9  //Port Pin used for external memory under control of a memory driver. // x
}Port_PinModeType;

typedef enum
{
    PORT_PIN_LEVEL_HIGH, PORT_PIN_LEVEL_LOW
}Port_PinLevelValue ;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *  3. the direction of pin --> INPUT or OUTPUT
 *  4. the internal resistor --> Disable, Pull up or Pull down
 *  5. pin initial value (PORT_PIN_LEVEL_HIGH/PORT_PIN_LEVEL_LOW)
 *  6. Pin mode
 *  7. Pin direction changeable during runtime (STD_ON/STD_OFF)
 *  8. Pin mode changeable during runtime (STD_ON/STD_OFF)
 */
typedef struct 
{
    uint8 port_num; 
    uint8 pin_num; 
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    uint8 initial_value;
    Port_PinModeType pin_mode;
    uint8 pin_changeable_direction;
    uint8 pin_changeable_mode;
}Port_ConfigPin;

/* Data Structure required for initializing the Port Driver */
typedef struct Port_ConfigType
{
    Port_ConfigPin Pins[PORT_NUMBER_OF_PORT_PINS];
} Port_ConfigType;

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: non-reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr);

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
*                  Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction during runtime
************************************************************************************/
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction);

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes the direction of all configured ports to the configured direction
************************************************************************************/
void Port_RefreshPortDirection(void);

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);\

/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
*                  Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode);

/************************************************************************************
* Service Name: Port_SetupGpioPin
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_SetupGpioPin(const Port_ConfigPin *ConfigPtr );

#endif /* PORT_H */

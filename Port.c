 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
#include "Det.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigType * Port_PortConfig = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

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
/*  Det errors: PORT_E_PARAM_CONFIG */
void Port_Init(const Port_ConfigType* ConfigPtr)
{
    uint8 i = 0;
    uint32* portBaseAddress;
    volatile unsigned long delay;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_Init_SID,
                        PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /*
         * Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures
         */
        Port_Status       = PORT_INITIALIZED;
        Port_PortConfig = ConfigPtr; /* address of the first Pin structure */
    }

    /* Set PORT */
    for(i=0 ; i<PORT_NUMBER_OF_PORT_PINS; i++)
    {
        /* get port */
        switch(ConfigPtr->Pins[i].port_num)
        {
        case 0:
            portBaseAddress = (uint32*)GPIO_PORTA_BASE_ADDRESS;
            break;
        case 1:
            portBaseAddress = (uint32*)GPIO_PORTB_BASE_ADDRESS;
            break;
        case 2:
            portBaseAddress = (uint32*)GPIO_PORTC_BASE_ADDRESS;
            break;
        case 3:
            portBaseAddress = (uint32*)GPIO_PORTD_BASE_ADDRESS;
            break;
        case 4:
            portBaseAddress = (uint32*)GPIO_PORTE_BASE_ADDRESS;
            break;
        case 5:
            portBaseAddress = (uint32*)GPIO_PORTF_BASE_ADDRESS;
            break;
        }

        /* Activate the clock for the port using the RCGCGPIO register */
        SYSCTL_RCGCGPIO_REG |= (1 << ConfigPtr->Pins[i].port_num);

        /* Wait until the clock for the port is ready */
        delay = SYSCTL_RCGCGPIO_REG;

        if(((ConfigPtr->Pins[i].port_num==3) && (ConfigPtr->Pins[i].pin_num==7)) || ((ConfigPtr->Pins[i].port_num==5)&& (ConfigPtr->Pins[i].pin_num==0)))
        {
            /* Unlock the GPIOCR register */
            *(volatile uint32*)((uint8*)portBaseAddress + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;

            /* Set the corresponding bit in the CR register */
            SET_BIT(*(volatile uint32*)((uint8*)portBaseAddress + PORT_COMMIT_REG_OFFSET),ConfigPtr->Pins[i].pin_num);
        }
        else if((ConfigPtr->Pins[i].port_num==2) && (ConfigPtr->Pins[i].pin_num<=3))
        {
            continue; /* JTAG connections, better not to use normally */
        }

        /* set mode */
        switch(ConfigPtr->Pins[i].pin_mode)
        {
        case PORT_PIN_MODE_ADC:
            /* Enable analog functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),ConfigPtr->Pins[i].pin_num);
            /* Disable digital functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            /* Clear the pin corresponding 4 PMCx bits in the GPIOPCTL */
            *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &= ~(0xF << (ConfigPtr->Pins[i].pin_num * 4));
            /* Disable alternative functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            break;

        case PORT_PIN_MODE_DIO:
            /* Disable analog functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),ConfigPtr->Pins[i].pin_num);
            /* Enable digital functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            /* Clear the pin corresponding 4 PMCx bits in the GPIOPCTL */
            *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &= ~(0xF << (ConfigPtr->Pins[i].pin_num * 4));
            /* Disable alternative functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            break;

        case PORT_PIN_MODE_PWM:
            /* Disable analog functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),ConfigPtr->Pins[i].pin_num);
            /* Enable digital functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            /* Set the corresponding PMCx bits in the GPIOPCTL */
            if((ConfigPtr->Pins[i].port_num==0) || (ConfigPtr->Pins[i].port_num==5))
            {
                *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) =
                        (*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &
                                ~(0xF << (ConfigPtr->Pins[i].pin_num * 4))) |
                                (0x5 << (ConfigPtr->Pins[i].pin_num * 4));
            }
            else
            {
                *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) =
                        (*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &
                                ~(0xF << (ConfigPtr->Pins[i].pin_num * 4))) |
                                (ConfigPtr->Pins[i].pin_mode << (ConfigPtr->Pins[i].pin_num * 4));
            }
            /* Enable alternative functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            break;

        case 2: /* PORT_PIN_MODE_SPI */
        case 7: /* PORT_PIN_MODE_DIO_GPT, PORT_PIN_MODE_DIO_WDG , PORT_PIN_MODE_ICU */
        case 8: /* PORT_PIN_MODE_CAN */
            /* Disable analog functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),ConfigPtr->Pins[i].pin_num);
            /* Enable digital functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            /* Set the corresponding PMCx bits in the GPIOPCTL */
            *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) =
                    (*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &
                            ~(0xF << (ConfigPtr->Pins[i].pin_num * 4))) |
                            (ConfigPtr->Pins[i].pin_mode << (ConfigPtr->Pins[i].pin_num * 4));
            /* Enable alternative functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            break;

        case 9: /* Unsupported modes */
            break;
        }

        /* set Direction */
        if(ConfigPtr->Pins[i].direction == PORT_PIN_IN)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIR_REG_OFFSET),ConfigPtr->Pins[i].pin_num);

            if(ConfigPtr->Pins[i].resistor == PULL_UP)
            {
                /* Enable internal pull up resistor */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_PULL_UP_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            }
            else if(ConfigPtr->Pins[i].resistor == PULL_DOWN)
            {
                /* Enable internal pull down resistor */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            }
            else /* OFF */
            {
                /* Disable internal pull up resistor */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_PULL_UP_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
                /* Disable internal pull down resistor */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            }
        }
        else /* PORT_PIN_OUT */
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIR_REG_OFFSET),ConfigPtr->Pins[i].pin_num);

            /* set initial value */
            if(ConfigPtr->Pins[i].initial_value == PORT_PIN_LEVEL_HIGH)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DATA_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DATA_REG_OFFSET), ConfigPtr->Pins[i].pin_num);
            }
        }
    }
}
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
/*  Det errors: PORT_E_UNINIT, PORT_E_PARAM_PIN, PORT_E_DIRECTION_UNCHANGEABLE */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction)
{
    boolean error = FALSE;
    uint32* portBaseAddress;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin is within the valid range */
    if (PORT_NUMBER_OF_PORT_PINS <= Pin)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    if(Port_PortConfig->Pins[Pin].pin_changeable_direction == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID, PORT_E_DIRECTION_UNCHANGEABLE);
    }
#endif
    /* In-case there are no errors */
    if(FALSE == error)
    {
        /* get port */
        switch(Port_PortConfig->Pins[Pin].port_num)
        {
        case 0:
            portBaseAddress = (uint32*)GPIO_PORTA_BASE_ADDRESS;
            break;
        case 1:
            portBaseAddress = (uint32*)GPIO_PORTB_BASE_ADDRESS;
            break;
        case 2:
            portBaseAddress = (uint32*)GPIO_PORTC_BASE_ADDRESS;
            break;
        case 3:
            portBaseAddress = (uint32*)GPIO_PORTD_BASE_ADDRESS;
            break;
        case 4:
            portBaseAddress = (uint32*)GPIO_PORTE_BASE_ADDRESS;
            break;
        case 5:
            portBaseAddress = (uint32*)GPIO_PORTF_BASE_ADDRESS;
            break;
        }

        if((Port_PortConfig->Pins[Pin].port_num==2) && (Port_PortConfig->Pins[Pin].pin_num<=3))
        {
            return; /* JTAG connections, better not to use normally */
        }

        /* set Direction */
        if(Direction == PORT_PIN_IN)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIR_REG_OFFSET),Port_PortConfig->Pins[Pin].pin_num);
        }
        else /* PORT_PIN_OUT */
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIR_REG_OFFSET),Port_PortConfig->Pins[Pin].pin_num);
        }
    }
}
#endif
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
/*  Det errors: PORT_E_UNINIT */
void Port_RefreshPortDirection(void)
{
    boolean error = FALSE;
    uint8 i;
    uint32* portBaseAddress;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif

    if(FALSE == error)
    {
        for(i=0 ; i<PORT_NUMBER_OF_PORT_PINS; i++)
        {
            /*  exclude pins that are configured as 'pin direction changeable during runtime' */
            if(Port_PortConfig->Pins[i].pin_changeable_direction == STD_ON)
            {
                continue;
            }

            /* get port */
            switch(Port_PortConfig->Pins[i].port_num)
            {
            case 0:
                portBaseAddress = (uint32*)GPIO_PORTA_BASE_ADDRESS;
                break;
            case 1:
                portBaseAddress = (uint32*)GPIO_PORTB_BASE_ADDRESS;
                break;
            case 2:
                portBaseAddress = (uint32*)GPIO_PORTC_BASE_ADDRESS;
                break;
            case 3:
                portBaseAddress = (uint32*)GPIO_PORTD_BASE_ADDRESS;
                break;
            case 4:
                portBaseAddress = (uint32*)GPIO_PORTE_BASE_ADDRESS;
                break;
            case 5:
                portBaseAddress = (uint32*)GPIO_PORTF_BASE_ADDRESS;
                break;
            }

            if((Port_PortConfig->Pins[i].port_num==2) && (Port_PortConfig->Pins[i].pin_num<=3))
            {
                return; /* JTAG connections, better not to use normally */
            }

            /* set Direction */
            if(Port_PortConfig->Pins[i].direction == PORT_PIN_IN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIR_REG_OFFSET),Port_PortConfig->Pins[i].pin_num);
            }
            else /* PORT_PIN_OUT */
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIR_REG_OFFSET),Port_PortConfig->Pins[i].pin_num);
            }
        }
    }
}
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
/*  Det errors: PORT_E_UNINIT, PORT_E_PARAM_POINTER */
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
    boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_GetVersionInfo_SID, PORT_E_UNINIT);
        error = TRUE;
    }

    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == versioninfo)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_GetVersionInfo_SID,
                        PORT_E_PARAM_POINTER);
    }
#endif
    if(FALSE == error)
    {
        versioninfo->moduleID = PORT_MODULE_ID;
        versioninfo->sw_major_version = PORT_SW_MAJOR_VERSION;
        versioninfo->sw_minor_version = PORT_SW_MINOR_VERSION;
        versioninfo->sw_patch_version = PORT_SW_PATCH_VERSION;
        versioninfo->vendorID = PORT_VENDOR_ID;
    }
}
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
/*  Det errors: PORT_E_UNINIT, PORT_E_PARAM_PIN, PORT_E_PARAM_INVALID_MODE, PORT_E_MODE_UNCHANGEABLE */
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
    boolean error = FALSE;
    uint32* portBaseAddress;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinMode_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    /* Check if the used pin is within the valid range */
    if (PORT_NUMBER_OF_PORT_PINS <= Pin)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinMode_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    if(Port_PortConfig->Pins[Pin].pin_changeable_mode == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID, PORT_E_MODE_UNCHANGEABLE);
    }
    if((Mode > PORT_PIN_MODE_MEM) || (Mode < PORT_PIN_MODE_ADC) ||
       (Mode == 5) || (Mode == 6) || (Mode == 3))
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID, PORT_E_PARAM_INVALID_MODE);
    }
#endif
    /* In-case there are no errors */
    if(FALSE == error)
    {
        /* get port */
        switch(Port_PortConfig->Pins[Pin].port_num)
        {
        case 0:
            portBaseAddress = (uint32*)GPIO_PORTA_BASE_ADDRESS;
            break;
        case 1:
            portBaseAddress = (uint32*)GPIO_PORTB_BASE_ADDRESS;
            break;
        case 2:
            portBaseAddress = (uint32*)GPIO_PORTC_BASE_ADDRESS;
            break;
        case 3:
            portBaseAddress = (uint32*)GPIO_PORTD_BASE_ADDRESS;
            break;
        case 4:
            portBaseAddress = (uint32*)GPIO_PORTE_BASE_ADDRESS;
            break;
        case 5:
            portBaseAddress = (uint32*)GPIO_PORTF_BASE_ADDRESS;
            break;
        }
        /* set mode */
        switch(Mode)
        {
        case PORT_PIN_MODE_ADC:
            /* Enable analog functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),Port_PortConfig->Pins[Pin].pin_num);
            /* Disable digital functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            /* Clear the pin corresponding 4 PMCx bits in the GPIOPCTL */
            *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &= ~(0xF << (Port_PortConfig->Pins[Pin].pin_num * 4));
            /* Disable alternative functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            break;

        case PORT_PIN_MODE_DIO:
            /* Disable analog functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),Port_PortConfig->Pins[Pin].pin_num);
            /* Enable digital functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            /* Clear the pin corresponding 4 PMCx bits in the GPIOPCTL */
            *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &= ~(0xF << (Port_PortConfig->Pins[Pin].pin_num * 4));
            /* Disable alternative functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            break;

        case PORT_PIN_MODE_PWM:
            /* Disable analog functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),Port_PortConfig->Pins[Pin].pin_num);
            /* Enable digital functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            /* Set the corresponding PMCx bits in the GPIOPCTL */
            if((Port_PortConfig->Pins[Pin].port_num==0) || (Port_PortConfig->Pins[Pin].port_num==5))
            {
                *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) =
                        (*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &
                                ~(0xF << (Port_PortConfig->Pins[Pin].pin_num * 4))) |
                                (0x5 << (Port_PortConfig->Pins[Pin].pin_num * 4));
            }
            else
            {
                *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) =
                        (*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &
                                ~(0xF << (Port_PortConfig->Pins[Pin].pin_num * 4))) |
                                (Port_PortConfig->Pins[Pin].pin_mode << (Port_PortConfig->Pins[Pin].pin_num * 4));
            }
            /* Enable alternative functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            break;

        case 2: /* PORT_PIN_MODE_SPI */
        case 7: /* PORT_PIN_MODE_DIO_GPT, PORT_PIN_MODE_DIO_WDG , PORT_PIN_MODE_ICU */
        case 8: /* PORT_PIN_MODE_CAN */
            /* Disable analog functionality */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET),Port_PortConfig->Pins[Pin].pin_num);
            /* Enable digital functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            /* Set the corresponding PMCx bits in the GPIOPCTL */
            *(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) =
                    (*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_CTL_REG_OFFSET) &
                            ~(0xF << (Port_PortConfig->Pins[Pin].pin_num * 4))) |
                            (Port_PortConfig->Pins[Pin].pin_mode << (Port_PortConfig->Pins[Pin].pin_num * 4));
            /* Enable alternative functionality */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)portBaseAddress + PORT_ALT_FUNC_REG_OFFSET), Port_PortConfig->Pins[Pin].pin_num);
            break;

        case 9: /* Unsupported modes */
            break;
        }
    }
}
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
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_SetupGpioPin(const Port_ConfigPin * ConfigPtr )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

    switch(ConfigPtr->port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		         break;
     	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		         break;
	    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		         break;
	    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		         break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		         break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		         break;
    }
    
    if( ((ConfigPtr->port_num == 3) && (ConfigPtr->pin_num == 7)) || ((ConfigPtr->port_num == 5) && (ConfigPtr->pin_num == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , ConfigPtr->pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (ConfigPtr->port_num == 2) && (ConfigPtr->pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
    if(ConfigPtr->direction == PORT_PIN_OUT)
    {
	    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(ConfigPtr->initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , ConfigPtr->pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , ConfigPtr->pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(ConfigPtr->direction == PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ConfigPtr->pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(ConfigPtr->resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , ConfigPtr->pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(ConfigPtr->resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , ConfigPtr->pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }

    /* Setup the pin mode as GPIO */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->pin_num * 4));     /* Clear the PMCx bits for this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

}

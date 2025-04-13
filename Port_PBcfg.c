#include "tm4c123gh6pm_registers.h"
#include "Port.h"


/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif

const Port_ConfigType Port_Configuration = { PORT_A, PIN_0, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_1, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_2, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_3, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_4, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_5, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_6, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_A, PIN_7, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_0, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_1, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_2, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_3, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_4, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_5, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_6, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_B, PIN_7, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_0, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_1, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_2, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_3, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_4, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_5, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_6, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_C, PIN_7, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_0, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_1, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_2, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_3, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_4, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_5, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_6, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_D, PIN_7, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_E, PIN_0, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_E, PIN_1, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_E, PIN_2, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_E, PIN_3, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_E, PIN_4, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_E, PIN_5, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_F, PIN_0, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                            /*led*/          PORT_F, PIN_1, PORT_PIN_OUT, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_F, PIN_2, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                             PORT_F, PIN_3, PORT_PIN_IN, OFF, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                           /*button*/        PORT_F, PIN_4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_HIGH, PORT_PIN_MODE_DIO, STD_ON, STD_ON,
                                           };

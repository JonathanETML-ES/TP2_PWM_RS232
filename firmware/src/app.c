/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <stdbool.h>
#include "bsp.h"
#include "app.h"
#include "Mc32DriverAdc.h"
#include "Mc32DriverLcd.h"
#include "Mc32CalCrc16.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"

#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "gestPWM.h"



// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

APP_DATA_TO_SEND appDataSend;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Initialize ( void )    //Initialisation de nos valeurs
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.mesValeurs.AngleSetting = 0; 
    appData.mesValeurs.SpeedSetting = 0; 
    appData.mesValeurs.absAngle = 0; 
    appData.mesValeurs.absSpeed = 0; 
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void APP_UpdateState (APP_STATES NewState)  //Création fonction pour update la machine d'état
{
    appData.state = NewState;
}

void ExtinctionDesLeds(void)                //Création fonction pour éteindre toutes les LEDs
{
    BSP_LEDOff(BSP_LED_0);
    BSP_LEDOff(BSP_LED_1);
    BSP_LEDOff(BSP_LED_2);
    BSP_LEDOff(BSP_LED_3);
    BSP_LEDOff(BSP_LED_4);
    BSP_LEDOff(BSP_LED_5);
    BSP_LEDOff(BSP_LED_6);
    BSP_LEDOff(BSP_LED_7);
}
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
S_ADCResults ADCResults;

void APP_Tasks ( void )
{
    //S_pwmSettings PWMData; //Si jvx utiliser PWMData dans dautre fichier
    uint8_t CommStatus = 0;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;         //initialisation de mes valeurs par défaut
            
            lcd_init();                         //Initialisation de mon LCD
            lcd_gotoxy(1,1);                    //Allez à la ligne 1 de mon affichage
            printf_lcd("TP1 PWM 2022-2023");    //
            lcd_gotoxy(1,2);                    //Allez à la ligne 2 de mon affichage
            printf_lcd("Neziri");         //
            lcd_gotoxy(1,3);                    //Allez à la ligne 3 de mon affichage
            printf_lcd("Shifteh");              //
            lcd_bl_on();
            
            BSP_InitADC10();                    //Initialisation de l'AD
            
            ExtinctionDesLeds();                //Extinction des LEDs
            
            GPWM_Initialize(&appData.mesValeurs);    //Initialisation des start Timers, OCs
            
            InitFifoComm();
            
            if (appInitialized)
            {
                appData.state = APP_STATE_WAIT;
            }
            break;
        }
        
        /* Application's wait state. */
        case APP_STATE_WAIT:
        {
            break;
        }
        
        /* Application's Service tasks state. */
        case APP_STATE_SERVICE_TASKS:
        {
            //Réceptuib param. remote
            CommStatus = GetMessage(&appData.mesValeurs);
            
            //Lecture pot.
            if(CommStatus == 0) //Local?
                GPWM_GetSettings(&appData.mesValeurs);                  // Obtention vitesse et angle
            else 
                GPWM_GetSettings(&appDataSend.mesValeurs);              // Obtention vitesse et angle
            
            //Affichage
            GPWM_DispSettings(&appData.mesValeurs, CommStatus);         // Affichage
            
            GPWM_ExecPWM(&appData.mesValeurs);                          // Execution PWM et gestion moteur
            
            //Envoi valeurs
            if(CommStatus == 0) //Local?
                SendMessage(&appData.mesValeurs);
            else 
                SendMessage(&appDataSend.mesValeurs);
            
            appData.state = APP_STATE_WAIT;
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

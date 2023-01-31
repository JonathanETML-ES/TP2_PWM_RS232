/*--------------------------------------------------------*/
// GestPWM.c
/*--------------------------------------------------------*/
//	Description :	Gestion des PWM 
//			        pour TP1 2022-2023
//
//	Auteur 		: 	C. HUBER
//  Mod         :   JSH, GSU
//	Version		:	V1.1
//	Compilateur	:	XC32 V1.42 + Harmony 1.08
//
/*--------------------------------------------------------*/

#include <stdint.h>
#include "bsp.h"
#include "GestPWM.h"
#include "Mc32DriverAdc.h"
#include "Mc32DriverLCD.h"
#include "system_config/default/framework/driver/tmr/drv_tmr_static.h"
#include "system_config/default/framework/driver/oc/drv_oc_static.h"
#include "peripheral/oc/plib_oc.h"


#define MOYENNE 10
#define CONVE_SPEED_SIGNED 99
#define CONVE_ANGLE_SIGNED 90
#define PERIOD_TIMER_2 1999
#define PERIOD_TIMER_3 8749

void GPWM_Initialize(S_pwmSettings *pData)
{
   // Init les data 
     //déjà fait lors du début du programme
    
   // Init état du pont en H
    BSP_EnableHbrige(); //Activation 
    
   // lance les timers et OC
    DRV_TMR0_Start(); 
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_OC0_Start();
    DRV_OC1_Start();
}

// Obtention vitesse et angle
void GPWM_GetSettings(S_pwmSettings *pData)	
{
    // Lecture du convertisseur AD
    static int TableauMoyenneP0[MOYENNE], TableauMoyenneP1[MOYENNE];
    static int i = 0;
    static int moyenneP0 = 0, moyenneP1 = 0;
    static int val_bruteP0 = 0, val_bruteP1 = 0, val_0_198 = 0;
    
    val_bruteP0 = BSP_ReadAllADC().Chan0;   //Prendre valeur du pot0
    val_bruteP1 = BSP_ReadAllADC().Chan1;   //Prendre valeur du pot1
    
    TableauMoyenneP0[i] = val_bruteP0;      //Mise dans un tableau afin de calculé la moyenne du pot0
    TableauMoyenneP1[i] = val_bruteP1;      //Mise dans un tableau afin de calculé la moyenne du pot1
    
    if(i >= 9){                             //Lorsque nous avons récolté 10 valeurs de pot,
        
        while(i >= 0){
         
         moyenneP0 += TableauMoyenneP0[i];  //Calcul intermédiaire afin d'avoir la moyenne du pot0
         moyenneP1 += TableauMoyenneP1[i];  //Calcul intermédiaire afin d'avoir la moyenne du pot1
         i--;
        }
        moyenneP0 /= 11;                    //Obtention de la moyenne de pot0
        moyenneP1 /= 11;                    //Obtention de la moyenne de pot1                  
    }
    i++;                                    //Incrémentation i
    // conversion
    val_0_198 = moyenneP0 / 5.15;           //changer pour pas avoir nombre a virgule donc en plusieur division par exemple
    pData->absAngle = moyenneP1 / 5.67;     // ''
    
    pData->SpeedSetting = (val_0_198 - CONVE_SPEED_SIGNED); // Mise de la valeur de la vitesse dans notre pData
    pData->AngleSetting = pData->absAngle - CONVE_ANGLE_SIGNED; // Mise de la valeur de l'angle dans notre pData
    
    pData->absSpeed = abs(pData->SpeedSetting); // Mise de la valeur de la vitesse absolue dans notre pData
}

// Affichage des information en exploitant la structure
void GPWM_DispSettings(S_pwmSettings *pData, int Remote)
{
        lcd_ClearLine(1);                   //Suppresion des lignes 1, 2, 3 et 4
        lcd_ClearLine(2);                   //
        lcd_ClearLine(3);                   //
        lcd_ClearLine(4);                   //
        
        lcd_gotoxy(1,1);                    //Allez à la ligne 1 de mon affichage
        
        if(Remote == 1)
            printf_lcd("Remote settings");      //Afficher la valeur de la vitesse dans notre pData
        else
            printf_lcd("Local settings");      //Afficher la valeur de la vitesse dans notre pData
        
        
        lcd_gotoxy(1,2);                    //Allez à la ligne 2 de mon affichage
        printf_lcd("SpeedSetting %3d", pData->SpeedSetting);      //Afficher la valeur de la vitesse dans notre pData
        lcd_gotoxy(1,3);                    //Allez à la ligne 3 de mon affichage
        printf_lcd("absSpeed %7d", pData->absSpeed);        //Afficher la valeur de l'angle dans notre pData
        lcd_gotoxy(1,4);                    //Allez à la ligne 4 de mon affichage
        printf_lcd("Angle %10d", pData->AngleSetting);      //Afficher la valeur de la vitesse absolue dans notre pData
}

// Execution PWM et gestion moteur à partir des info dans structure
void GPWM_ExecPWM(S_pwmSettings *pData)
{
    float PulseOC3start = ((PERIOD_TIMER_3/70)*6);      //Création de l'offset de 0.6 mS
    int PulseOC2 = 0, PulseOC3 = 0;
    
    if(pData->SpeedSetting < 0)                         //Lorsque nous avons une vitesse inférieur à 0,
    {
        AIN1_HBRIDGE_W = 0;   //AIN1 Low                //Activation de mon moteur dans le sens anti-horaire
        AIN2_HBRIDGE_W = 1;   //AIN2 High               //
        STBY_HBRIDGE_W = 1;   //STBY High               //
    }
    else if(pData->SpeedSetting > 0)                   //Lorsque nous avons une vitesse supérieur à 0,
    {
        AIN1_HBRIDGE_W = 1;   //AIN1 High               //Activation de mon moteur dans le sens horaire
        AIN2_HBRIDGE_W = 0;   //AIN2 Low                //
        STBY_HBRIDGE_W = 1;   //STBY High               //
    }
    else                                                //Lorsque nous avons une vitesse nul donc à 0,
    {
        AIN1_HBRIDGE_W = 0;   //AIN1 Low                //Mise du moteur en standby
        AIN2_HBRIDGE_W = 0;   //AIN2 Low                //
        STBY_HBRIDGE_W = 0;   //STBY Low                //
    }
    
    
    PulseOC2 = pData->absSpeed * (PERIOD_TIMER_2/CONVE_SPEED_SIGNED);   //Création de la valeur de ma pulse pour l'OC2
    PulseOC3 = PulseOC3start + (pData->absAngle * (((PERIOD_TIMER_3/70)*24 - PulseOC3start) / 180));    //Création de la valeur de ma pulse pour l'OC3
    
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, PulseOC2);      //Mise de la valeur dans l'OC2
    PLIB_OC_PulseWidth16BitSet(OC_ID_3, PulseOC3);      //Mise de la valeur dans l'OC3
}

// Execution PWM software
void GPWM_ExecPWMSoft(S_pwmSettings *pData)    //Ici, nous venons créer un PWM purement software       
{
    static uint8_t compteur;
    
    compteur +=1;
    
    if(compteur <= pData->absSpeed)   BSP_LEDOff(BSP_LED_2);    //Lorsque mon compteur est plus petit ou égal à ma vitesse absolue souhaitée, il va venir mettre un état High sur la LED2
    else    BSP_LEDOn(BSP_LED_2);                               //Sinon, il va venir mettre un état Low sur la LED2 
    
    if(compteur >= 100)   compteur = 0;                         //Une fois que nous avons compté jusqu'a 100, il reset sont compteur.
}



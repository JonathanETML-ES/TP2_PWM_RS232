// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'�mission et de r�ception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoy� r�ponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"
#include "gestPWM.h"
#include "Mc32CalCrc16.h"

#include "Mc32DriverLcd.h"


typedef union {
        uint16_t val;
        struct {uint8_t lsb;
                uint8_t msb;} shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)



#define StartByte 0xAA
#define EndByte 0x02

// Structure d�crivant le message
typedef struct {
    uint8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    uint8_t MsbCrc;
    uint8_t LsbCrc;
} StruMess;


// Struct pour �mission des messages
StruMess TxMess;
// Struct pour r�ception des messages
StruMess RxMess;

// Declaration des FIFO pour r�ception et �mission
#define FIFO_RX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de r�ception
S_fifo descrFifoRX;


int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'�mission
S_fifo descrFifoTX;


// Initialisation de la communication s�rielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de r�ception
    InitFifo ( &descrFifoRX, FIFO_RX_SIZE, fifoRX, 0 );
    // Initialisation du fifo d'�mission
    InitFifo ( &descrFifoTX, FIFO_TX_SIZE, fifoTX, 0 );
    
    // Init RTS 
    RS232_RTS = 1;   // interdit �mission par l'autre
   
} // InitComm

 
// Valeur de retour 0  = pas de message re�u donc local (data non modifi�)
// Valeur de retour 1  = message re�u donc en remote (data mis � jour)
int GetMessage(S_pwmSettings *pData)
{
    uint8_t TabValeur[5] = {0,0,0,0,0};
    uint8_t Val_enregistre = 0;
    
    int commStatus = 0, NbCharToRead = 0;
    int8_t RxC = 0;
    int GetSituation = 0, MessIdx = 0; 
    int8_t EndMess = 0;
    
    
    // Valeur initi� pou rle calcul du CrC 
    uint16_t ValCrc16 = 0xFFFF, RxCrcMsb = 0, RxCrcLsb = 0, RxCrc16Calcul = 1;
    
    // Traitement de r�ception � introduire ICI
    // Contr�le du nombre de charact�re � lire
    NbCharToRead = GetReadSize(&descrFifoRX);
    
    EndMess = 0;
    //if(NbCharToRead >= MESS_SIZE)
    while((NbCharToRead >= 1) && (EndMess == 0))
    {
        GetCharFromFifo(&descrFifoRX, &RxC);    // Lecture des charact�re dans le FIFO
        NbCharToRead --;                        // D�crementation du nombre de caract�re � chaque passage dans le code (de 5 � 0)
        Val_enregistre = RxC;                   // Enrengistre la valeur re�u sur le Rx
        
        switch(GetSituation)                    // Switch case pour la gestion de la lecture de la trame
        {
            case 0 :                            
                if(Val_enregistre == StartByte)                                 // Contr�le du byte de start
                {
                    MessIdx = 0;                                                
                    TabValeur[MessIdx] = Val_enregistre;                        // Enrengistrement de la tableau du byte
                    ValCrc16 = updateCRC16(ValCrc16, TabValeur[MessIdx]);       // Mise � jour du CRC
                    MessIdx++;                                                  
                    GetSituation = 1;                                           // Prochain case qui encha�ne sur le suite de la trame
                }
                break;
                
            case 1:
                TabValeur[MessIdx] = Val_enregistre;                            // M�me proc�d� que le start byte
                
                if(MessIdx < 3)                                                 // Mise � jour du CRC sur les trois premier byte de la trame
                    ValCrc16 = updateCRC16(ValCrc16, TabValeur[MessIdx]);
                
                if(MessIdx == 3)                                                // Contr�le du CRC  MSB
                {
                    RxMess.MsbCrc = (ValCrc16 & 0xFF00) >> 8;
                    RxCrcMsb = TabValeur[3] ;
                }
                
                else if(MessIdx == 4)                                           // Contr�le du CRC  LSB
                {
                    RxMess.LsbCrc = (ValCrc16 & 0x00FF);
                    RxCrcLsb = TabValeur[4] ;
                    
                    RxCrc16Calcul = ((RxCrcMsb << 8) & 0xFF00) | (RxCrcLsb & 0x00FF);           
                }
                
                MessIdx++;    
                
                if(RxCrc16Calcul == ValCrc16)                                   
                {   
                    RxMess.Start = TabValeur[0];
                    RxMess.Speed = TabValeur[1];
                    RxMess.Angle = TabValeur[2];
                    RxMess.MsbCrc = TabValeur[3];
                    RxMess.LsbCrc = TabValeur[4];
                    
                    pData->SpeedSetting = RxMess.Speed;
                    pData->AngleSetting = RxMess.Angle;
                    
                    TabValeur[MessIdx] = 0;
                    MessIdx = 0;
                    EndMess = 1;
                    commStatus = 1;
                    GetSituation = 0;
                    
                    
                    //TEST
                    /*
                    lcd_ClearLine(2);                   //
                    lcd_ClearLine(3);                   //
                    lcd_gotoxy(1,2);                    //Allez � la ligne 1 de mon affichage
                                      //Allez � la ligne 2 de mon affichage
                    printf_lcd("%d, %d, %d", TabValeur[0], pData->SpeedSetting, pData->AngleSetting);      //Afficher la valeur de la vitesse dans notre pData
                    lcd_gotoxy(1,3);                    //Allez � la ligne 3 de mon affichage
                    printf_lcd("%d, %d, %d", TabValeur[3], TabValeur[4], commStatus);        //Afficher la valeur de l'angle dans notre pData
                    lcd_gotoxy(1,4);                    //Allez � la ligne 4 de mon affichage
                    */
                }
                break;
        }
    }
    
    // Gestion controle de flux de la r�ception
    if(GetWriteSpace ( &descrFifoRX) >= (2*MESS_SIZE)) {
        // autorise �mission par l'autre
        RS232_RTS = 0;
    }
    return commStatus;
} // GetMessage

// Fonction d'envoi des messages, appel cyclique
void SendMessage(S_pwmSettings *pData)
{
    int8_t freeSize;
    uint16_t ValCrc16 = 0xFFFF;
    
    // Traitement �mission � introduire ICI
    //Test si place pour �crire message dans le FIFO
    freeSize = GetWriteSpace(&descrFifoTX);
    if(freeSize >= MESS_SIZE)
    {
        //Compose le message 
        TxMess.Start = 0xAA;                                                       
        TxMess.Speed = pData->SpeedSetting;
        TxMess.Angle = pData->AngleSetting;
        
        ValCrc16 = updateCRC16(ValCrc16, TxMess.Start);                         // Mise � jour du CRC pour l'envoi des bytes de data
        ValCrc16 = updateCRC16(ValCrc16, TxMess.Speed);
        ValCrc16 = updateCRC16(ValCrc16, TxMess.Angle);
        
        //Attribution du CRC
        TxMess.MsbCrc = (ValCrc16 & 0xFF00) >> 8;                               
        TxMess.LsbCrc = (ValCrc16 & 0x00FF);
        
        //D�pose le message dans le fifo
        PutCharInFifo(&descrFifoTX, TxMess.Start);
        PutCharInFifo(&descrFifoTX, TxMess.Speed);
        PutCharInFifo(&descrFifoTX, TxMess.Angle);
        PutCharInFifo(&descrFifoTX, TxMess.MsbCrc);
        PutCharInFifo(&descrFifoTX, TxMess.LsbCrc); 
    }
    
    
    
    // Formatage message et remplissage fifo �mission
    // ...
    
    
    // Gestion du controle de flux
    // si on a un caract�re � envoyer et que CTS = 0
    freeSize = GetReadSize(&descrFifoTX);
    if ((RS232_CTS == 0) && (freeSize > 0))
    {
        // Autorise int �mission    
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
    }
}


// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la r�ponse g�n�r�e dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    USART_ERROR UsartStatus;    


    // Marque d�but interruption avec Led3
    LED3_W = 1;
    int8_t Val_send = 0;
    int8_t c;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) ) {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur � la r�ception.
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) ) {

        // Oui Test si erreur parit� ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0) {

            // Traitement RX � faire ICI
            // Lecture des caract�res depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
      
            while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {
                c = PLIB_USART_ReceiverByteReceive(USART_ID_1);
                
                PutCharInFifo(&descrFifoRX, c);
                BSP_LEDToggle(BSP_LED_3);
            }
      
            LED4_W = !LED4_R; // Toggle Led4
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } else {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN) {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }

        
        // Traitement controle de flux reception � faire ICI
        // Gerer sortie RS232_RTS en fonction de place dispo dans fifo reception
        // ...

        
    } // end if RX

    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) ) {

        // Traitement TX � faire ICI
        // Envoi des caract�res depuis le fifo SW -> buffer HW
     //PLIB_USART_TransmitterEnable(USART_ID_1);
        
        // Avant d'�mettre, on v�rifie 3 conditions :
        //  Si CTS = 0 autorisation d'�mettre (entr�e RS232_CTS)
        if(RS232_CTS == 0)
        {
            //  S'il y a un carat�res � �mettre dans le fifo
            if(GetReadSize(&descrFifoTX) != 0)
            {
            //  S'il y a de la place dans le buffer d'�mission (PLIB_USART_TransmitterBufferIsFull)
            //   (envoi avec PLIB_USART_TransmitterByteSend())
                if(PLIB_USART_TransmitterBufferIsFull(USART_ID_1) == 0)
                {
                    int i;
                    for(i = 0; i<5; i++)
                    {
                    // ...
                    GetCharFromFifo(&descrFifoTX, &Val_send);
                    
                    //PLIB_USART_TransmitterByteSend(USART_ID_1, descrFifoTX);
                    PLIB_USART_TransmitterByteSend(USART_ID_1, Val_send);
                    }
                }
            }
        }
	   
        //PLIB_USART_TransmitterDisable(INT_ID_0);
        
        LED5_W = !LED5_R; // Toggle Led5
		
        // disable TX interrupt (pour �viter une interrupt. inutile si plus rien � transmettre)
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
        // Clear the TX interrupt Flag (Seulement apres TX) 
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    // Marque fin interruption avec Led3
    LED3_W = 0;
 }





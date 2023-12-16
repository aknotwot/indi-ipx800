/*******************************************************************************
This file is part of the IPX800 V4 INDI Driver.
A driver for the IPX800 (GCE - http : //www.aagware.eu/)

Copyright (C) 2022 Arnaud Dupont (aknotwot@protonmail.com)

IPX800 V4 INDI Driver is free software : you can redistribute it
and / or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

IPX800 V4  INDI Driver is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with IPX800 V4  INDI Driver.  If not, see
< http : //www.gnu.org/licenses/>.

*******************************************************************************/

#include "indi_ipx800_v4.h"

#include "indicom.h"
#include "config.h"

#include "indistandardproperty.h"
#include "indipropertyswitch.h"
#include "connectionplugins/connectiontcp.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <string.h>
#include <algorithm>
#include <chrono>
#include <thread>

//Network related includes:
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>


// We declare an auto pointer to ipx800_v4.
std::unique_ptr<Ipx800_v4> ipx800v4(new Ipx800_v4());

#define ROLLOFF_DURATION 20 // 20 seconds until roof is fully opened or closed
#define DEFAULT_POLLING_TIMER 2000


/*************************************************************************/
/** Constructor                                                         **/
/*************************************************************************/

Ipx800_v4::Ipx800_v4()
{
    LOG_INFO("Setting Capabilities...");
    //Forcer les particularités du DOME
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK);
    //Abort for emergency only. S/OFF power
	//Connection Ethernet obligatoire
    setDomeConnection(CONNECTION_TCP);
	setDomeState(DOME_UNKNOWN);
    // init variables
	Roof_Status = UNKNOWN_STATUS;
	Mount_Status = NONE_PARKED;
	
	//IPX800_command = ClearR;
	//IPXRelaysCommands = UNUSED_RELAY;
	//IPXDigitalRead = UNUSED_DIGIT;
	//initialiser les variables dans le .H 
	
	//LOG_INFO("Delete Property...");
	//INDI::Dome::deleteProperty(DomeMotionSP.name);
    LOG_INFO("Capabilities Set...");
	setVersion(IPX800_V4_VERSION_MAJOR,IPX800_V4_VERSION_MINOR);
}


/*************************************************************************/
/** DeConstructor                                                         **/
/*************************************************************************/
/*
Ipx800_v4::~Ipx800_v4()
{
	INDI::Dome::~Dome();
}*/

/************************************************************************************
*
************************************************************************************/
bool Ipx800_v4::initProperties()
{
    LOG_INFO("Starting device...");
    
	INDI::Dome::initProperties();
	//buildSkeleton("indi_ipx800v4_sk.xml");
    SetParkDataType(PARK_NONE);
    addDebugControl();
	addConfigurationControl();
	//setDebug(true);
	
    //creation liste deroulante Relais
    IUFillSwitch(&RelaisInfoS[0], "Unused", "",ISS_ON);
    IUFillSwitch(&RelaisInfoS[1], "Roof Engine Power", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[2], "Telescope Ventilation", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[3], "Heating Resistor 1", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[4], "Heating Resistor 2", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[5], "Roof Control Command", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[6], "Mount Power Supply", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[7], "Camera Power Supply ", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[8], "Other Power Supply 1", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[9], "Other Power Supply 2", "", ISS_OFF);
    IUFillSwitch(&RelaisInfoS[10], "Other Power Supply 3", "", ISS_OFF);
    
	//set default value of each relay 
	for(int i=0;i<11;i++)
    {
        Relais1InfoS[i] = RelaisInfoS[i];
        Relais2InfoS[i] = RelaisInfoS[i];
        Relais3InfoS[i] = RelaisInfoS[i];
        Relais4InfoS[i] = RelaisInfoS[i];
        Relais5InfoS[i] = RelaisInfoS[i];
        Relais6InfoS[i] = RelaisInfoS[i];
        Relais7InfoS[i] = RelaisInfoS[i];
        Relais8InfoS[i] = RelaisInfoS[i];
    }

    //creation du selecteur de configuration des relais
    IUFillSwitchVector(&RelaisInfoSP[0], Relais1InfoS, 11, getDeviceName(), "RELAY_1_CONFIGURATION", "Relay 1", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[1], Relais2InfoS, 11, getDeviceName(), "RELAY_2_CONFIGURATION", "Relay 2", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[2], Relais3InfoS, 11, getDeviceName(), "RELAY_3_CONFIGURATION", "Relay 3", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[3], Relais4InfoS, 11, getDeviceName(), "RELAIS_4_CONFIGURATION", "Relay 4", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[4], Relais5InfoS,11, getDeviceName(), "RELAIS_5_CONFIGURATION", "Relay 5", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[5], Relais6InfoS, 11, getDeviceName(), "RELAIS_6_CONFIGURATION", "Relay 6", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[6], Relais7InfoS,11, getDeviceName(), "RELAIS_7_CONFIGURATION", "Relay 7", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaisInfoSP[7], Relais8InfoS, 11, getDeviceName(), "RELAIS_8_CONFIGURATION", "Relay 8", RELAYS_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    //creation liste deroulante entrees discretes
    IUFillSwitch(&DigitalInputS[0], "Unused", "",ISS_ON);
    IUFillSwitch(&DigitalInputS[1], "DEC Axis Parked", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[2], "RA Axis Parked", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[3], "Roof Opened", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[4], "Roof Closed", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[5], "Roof Engine Supplied", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[6], "Raspberry Power Supplied", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[7], "Main PC Supplied", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[8], "Other Digital 1", "", ISS_OFF);
    IUFillSwitch(&DigitalInputS[9], "Other Digital 2", "", ISS_OFF);
	
	//set default value of each digital input 
    for(int i=0;i<10;i++)
    {
        Digital1InputS[i] = DigitalInputS[i];
        Digital2InputS[i] = DigitalInputS[i];
        Digital3InputS[i] = DigitalInputS[i];
        Digital4InputS[i] = DigitalInputS[i];
        Digital5InputS[i] = DigitalInputS[i];
        Digital6InputS[i] = DigitalInputS[i];
        Digital7InputS[i] = DigitalInputS[i];
        Digital8InputS[i] = DigitalInputS[i];
    }
	
	//creation du selecteur de configuration des entrées discretes
    IUFillSwitchVector(&DigitalInputSP[0], Digital1InputS, 10, getDeviceName(), "DIGITAL_1_CONFIGURATION", "Digital 1", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[1], Digital2InputS, 10, getDeviceName(), "DIGITAL_2_CONFIGURATION", "Digital 2", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[2], Digital3InputS, 10, getDeviceName(), "DIGITAL_3_CONFIGURATION", "Digital 3", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[3], Digital4InputS, 10, getDeviceName(), "DIGITAL_4_CONFIGURATION", "Digital 4", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[4], Digital5InputS, 10, getDeviceName(), "DIGITAL_5_CONFIGURATION", "Digital 5", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[5], Digital6InputS, 10, getDeviceName(), "DIGITAL_6_CONFIGURATION", "Digital 6", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[6], Digital7InputS, 10, getDeviceName(), "DIGITAL_7_CONFIGURATION", "Digital 7", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitalInputSP[7], Digital8InputS, 10, getDeviceName(), "DIGITAL_8_CONFIGURATION", "Digital 8", DIGITAL_INPUT_CONFIGURATION_TAB,
                     IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    //Si l'acces à l'IPX est protégé
    IUFillText(&LoginPwdT[0], "LOGIN_VAL", "Login", "");
    IUFillText(&LoginPwdT[1], "PASSWD_VAL", "Password", "");
    IUFillTextVector(&LoginPwdTP, LoginPwdT, 2, getDeviceName(), "ACCESS_IPX", "IPX Access", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    //enregistrement des onglets de configurations
    for(int i=0;i<8;i++)
    {
        defineProperty(&RelaisInfoSP[i]);
        defineProperty(&DigitalInputSP[i]);
    }

    //champ de gestion du mot de passe
		defineProperty(&LoginPwdTP);

    ///////////////////////////////////////////////
    //Page de presentation de l'état des relais
	///////////////////////////////////////////////
    //char name[3] = 'yyy', nameM[3] = 'xxx';
	const char *name = "";
	const char *nameM = "";
	
    for(int i=0;i<2;i++)
    {
        if (i ==0) {
            name = "On";
            nameM = "ON"; }
        else if (i ==1) {
            name = "Off";
            nameM = "OFF";
        }
		else {
			LOG_ERROR ("Initialization Error...");
			return false;
		}
        IUFillSwitch(&Relay1StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay2StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay3StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay4StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay5StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay6StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay7StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Relay8StateS[i], name, nameM, ISS_OFF);
    }
    IUFillSwitchVector(&RelaysStatesSP[0], Relay1StateS, 2, getDeviceName(), "RELAY_1_STATE", "Relay 1", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[1], Relay2StateS, 2, getDeviceName(), "RELAY_2_STATE", "Relay 2", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[2], Relay3StateS, 2, getDeviceName(), "RELAY_3_STATE", "Relay 3", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[3], Relay4StateS, 2, getDeviceName(), "RELAY_4_STATE", "Relay 4", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[4], Relay5StateS, 2, getDeviceName(), "RELAY_5_STATE", "Relay 5", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[5], Relay6StateS, 2, getDeviceName(), "RELAY_6_STATE", "Relay 6", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[6], Relay7StateS, 2, getDeviceName(), "RELAY_7_STATE", "Relay 7", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&RelaysStatesSP[7], Relay8StateS, 2, getDeviceName(), "RELAY_8_STATE", "Relay 8", RAW_DATA_TAB,
                     IP_RW,ISR_1OFMANY, 60, IPS_IDLE);
					 
    //////////////////////////////////////////////////////////
    //page de presentation de l'état des entrées discretes
	//////////////////////////////////////////////////////////
	
    for(int i=0;i<2;i++)
    {
        if (i ==0) {
            name = "On";
            nameM = "ON"; }
        else if (i ==1) {
            name = "Off";
            nameM = "OFF";
        }
		else {
			LOG_ERROR ("Initialization Error...");
			return false;
		}
        IUFillSwitch(&Digit1StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Digit2StateS[i], name, nameM,ISS_OFF);
        IUFillSwitch(&Digit3StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Digit4StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Digit5StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Digit6StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Digit7StateS[i], name, nameM, ISS_OFF);
        IUFillSwitch(&Digit8StateS[i], name, nameM,ISS_OFF);
    }
    IUFillSwitchVector(&DigitsStatesSP[0], Digit1StateS, 2, getDeviceName(), "DIGIT_1_STATE", "Digital 1", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[1], Digit2StateS, 2, getDeviceName(), "DIGIT_2_STATE", "Digital 2", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[2], Digit3StateS, 2, getDeviceName(), "DIGIT_3_STATE", "Digital 3", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[3], Digit4StateS, 2, getDeviceName(), "DIGIT_4_STATE", "Digital 4", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[4], Digit5StateS, 2, getDeviceName(), "DIGIT_5_STATE", "Digital 5", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[5], Digit6StateS, 2, getDeviceName(), "DIGIT_6_STATE", "Digital 6", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[6], Digit7StateS, 2, getDeviceName(), "DIGIT_7_STATE", "Digital 7", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
    IUFillSwitchVector(&DigitsStatesSP[7], Digit8StateS, 2, getDeviceName(), "DIGIT_8_STATE", "Digital 8", RAW_DATA_TAB,
                     IP_RO,ISR_1OFMANY, 60, IPS_IDLE);
	//commentaire dans le code du dome - weather gere dans le driver du watchdog
	//IDSnoopDevice("Wheather Watcher", "WEATHER_STATUS");
	//IDSnoopDevice("Telescope", "TELESCOPE_PARK");
	// gros doute sur le nom de la device "TELESCOPE"
	setDefaultPollingPeriod(2000);
	
	//suppression open/close
	// DON'T WORK
	//INDI::Dome::deleteProperty(DomeMotionSP.name);
	//tcpConnection = new Connection::TCP(this);
    //tcpConnection->registerHandshake([&]() { return Handshake(); });
    //tcpConnection->establishConnection(Connection::Serial::B_57600);
    //serialConnection->setDefaultPort("/dev/ttyACM0");
    //registerConnection(tcpConnection);
	//
	return true;
}

bool Ipx800_v4::Handshake()
{
    if (isSimulation())
    {
        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());
        return true;
    }

    // TODO: Any initial communciation needed with our device; we have an active
    // connection with a valid file descriptor called PortFD. This file descriptor
    // can be used with the tty_* functions in indicom.h

    return true;
}


/*
void Ipx800_v4::ISGetProperties (const char *dev) {
Ipx800_v4
	DefaultDevice::ISGetProperties(dev);
	LOG_DEBUG("ISGetProperties loading....");
	loadConfig(&DigitalInputSP);
	loadConfig(&RelaisInfoSP);
}
*/


bool Ipx800_v4::ISSnoopDevice(XMLEle *root)
{
	/*LOG_INFO("SNOOP...");
	const char *propName = findXMLAttValu(root, "device");
	LOGF_DEBUG("Snooping device : %s",propName );
   */
    /*if (isConnected())
    {
        bool rc = false;

        //rc = updateIPXData();
        //ssetObsStatus();
    }*/
    return INDI::Dome::ISSnoopDevice(root);
}

bool Ipx800_v4::SetupParms()
{
   LOG_DEBUG("Setting Params...");
   // If we have parking data
   if (InitPark())
    {
        if (isParked())
        {
            fullOpenLimitSwitch   = ISS_OFF;
            fullClosedLimitSwitch = ISS_ON;
        }
        else
        {
            fullOpenLimitSwitch   = ISS_ON;
            fullClosedLimitSwitch = ISS_OFF;
        }
    }
    // If we don't have parking data
    else
    {
        fullOpenLimitSwitch   = ISS_OFF;
        fullClosedLimitSwitch = ISS_OFF;
    }

    return true;
}

bool Ipx800_v4::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    ISwitch *myRelaisInfoS;
    ISwitchVectorProperty myRelaisInfoSP;

    ISwitch *myDigitalInputS;
    ISwitchVectorProperty myDigitalInputSP;

   // IPXRelaysCommands currentCommands=UNUSED_RELAY;
    int currentRIndex, currentDIndex =0;
    bool infoSet = false;
	
	
	// Make sure the call is for our device, and Fonctions Tab are initialized
   if(!strcmp(dev,getDeviceName()))
   {
		for(int i=0;i<8;i++)
		{
			myRelaisInfoSP = ipx800v4->getMyRelayVector(i);
			myDigitalInputSP = ipx800v4->getMyDigitsVector(i);
			
			////////////////////////////////////////////////////
			// Relay Configuration
			////////////////////////////////////////////////////
			if (!strcmp(name, myRelaisInfoSP.name))
				{
				
				LOGF_DEBUG("Relay function selected - SP : %s", myRelaisInfoSP.name);
				//currentRS = IUFindSwitch(&myRelaisInfoSP, myRelaisInfoSP.name);
				IUUpdateSwitch(&myRelaisInfoSP,states,names,n);
				
				myRelaisInfoS = myRelaisInfoSP.sp;
				myRelaisInfoSP.s = IPS_OK;
				IDSetSwitch(&myRelaisInfoSP,nullptr);
				
				currentRIndex = IUFindOnSwitchIndex(&myRelaisInfoSP);
				
				if (currentRIndex != -1) {
					Relay_Fonction_Tab [currentRIndex] = i;
					/*if (is_Initialized == true) { */
				LOGF_DEBUG("Relay fonction index : %d", currentRIndex);
					//std::strcpy(RelaysStatesSP[i].label,myRelaisInfoS[currentRIndex].label);
					//LOGF_DEBUG("Relay Label updated : %s", RelaysStatesSP[i].label);
					
					//defineProperty(&RelaysStatesSP[i]);
					//IUUpdateSwitch(&RelaysStatesSP[i],states,names,n);
					//IDSetSwitch(&RelaysStatesSP[i],nullptr);
					
					// store the relay number function 
					//Relay_Fonction_Tab [currentRIndex] = i;
					//LOGF_DEBUG("Relay Label Index: %s", i); 
					defineProperty(&RelaysStatesSP[i]);}
				else 
					LOG_DEBUG("No On Switches found"); 
				
				infoSet = true;
			}
				
			////////////////////////////////////////////////////
			// Digits Configuration
			////////////////////////////////////////////////////
			if (!strcmp(name, myDigitalInputSP.name))
				{ 
				LOGF_DEBUG("Digital init : %s", myDigitalInputSP.name);
				IUUpdateSwitch(&myDigitalInputSP,states,names,n);
				
				myDigitalInputS = myDigitalInputSP.sp;
				// myRelaisInfoS[].s  = ISS_ON;
				myDigitalInputSP.s = IPS_OK;
				IDSetSwitch(&myDigitalInputSP,nullptr);
				//sauvegarde de la configuration
				currentDIndex = IUFindOnSwitchIndex(&myDigitalInputSP);
				if (currentDIndex != -1) {
					Digital_Fonction_Tab [currentDIndex] = i;
				LOGF_DEBUG("Digital Inp. fonction index : %d", currentDIndex);
				//std::strcpy(DigitsStatesSP[i].label,myDigitalInputS[currentDIndex].label);
				//LOGF_DEBUG("Digital Inp Label updated : %s", DigitsStatesSP[i].label);
					defineProperty(&DigitsStatesSP[i]);
				// store the function 
				 }
				else 
					LOG_DEBUG("No On Switches found"); 
				
				infoSet = true; 
			}
		}
		
		LOG_DEBUG("ISNewSwitch - First Init + UpDate");
		firstFonctionTabInit();
		updateIPXData();
		//first_Start = true;
	
		if (infoSet) 
			setObsStatus();
		
   }
   return INDI::Dome::ISNewSwitch(dev, name, states, names, n);	

}


bool Ipx800_v4::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    ////////////////////////////////////////////////////
    // IPX Access
    ////////////////////////////////////////////////////
    IText* myLoginT = ipx800v4->getMyLogin();
    ITextVectorProperty myLoginVector = ipx800v4->getMyLoginVector();
    if (!strcmp(name, myLoginVector.name))
    {
        IUUpdateText(&myLoginVector, texts, names, n);
        myLoginVector.s = IPS_OK;
        myPasswd = myLoginT[1].text;
        myLogin = myLoginT[0].text;
        IDSetText(&myLoginVector, nullptr);
     }

    ////////////////////////////////////////////////////
    // IP Address
    ////////////////////////////////////////////////////
 /*   IText* myHostAddress = ipx800v4->getText(&Connection::TCP::AddressT[0].text)
    ITextVectorProperty myLoginVector = ipx800v4->getMyLoginVector();


    if (!strcmp(name, myLoginVector.name))
    {
        IUUpdateText(&myLoginVector, texts, names, n);
        myLoginVector.s = IPS_OK;
        myPasswd = myLoginT[1].text;
        myLogin = myLoginT[0].text;
        IDSetText(&myLoginVector, nullptr);
     }*/
	return INDI::Dome::ISNewText(dev, name, texts, names, n);
    //return RollOff::ISNewText(dev, name, texts, names, n);
  }

///////////////////////////////////////////
// When IPX800 is connected two more tabs appear : States of Relays
//  States of Digitals inputs
///////////////////////////////////////////
bool Ipx800_v4::Connect()
{
	INDI::Dome::Connect();
    //RollOff::Connect();
	//bool rc2 = firstFonctionTabInit();
    //bool rc = updateIPXData();
	
    return true;
}

bool Ipx800_v4::Disconnect()
{
    INDI::Dome::deleteProperty(DomeMotionSP.name);
    //this->updateProperties();
    return true;
}

const char *Ipx800_v4::getDefaultName()
{
    return (const char *)"Ipx800 V4";
}

/////////////////////////////////////////
// Used after connection 
// ** complete ** 
// ajouter le snoop device
/////////////////////////////////////////
bool Ipx800_v4::updateProperties()
{
	INDI::Dome::updateProperties();
	LOG_DEBUG("updateProperties - Starting");
    if (isConnected())
    { // Connect both states tabs 
		updateIPXData();
		firstFonctionTabInit();
		setObsStatus();
        for(int i=0;i<8;i++)
        {
            defineProperty(&RelaysStatesSP[i]);
			//MàJ DomeState
        }
        for(int i=0;i<8;i++)
        {

             defineProperty(&DigitsStatesSP[i]);
        }
       SetupParms();
	  
	   
	   ///**  Recuperation aux parametres meteo **/
	   //INDI::Dome::IDSnoopDevice("Weather Watcher", "WEATHER_STATUS");
    }
    else { // Disconnect both "States TAB"
        for(int i=0;i<8;i++)
        {
            deleteProperty(RelaysStatesSP[i].name);

        }
        for(int i=0;i<8;i++)
        {

             deleteProperty(DigitsStatesSP[i].name);
        }
    }
	// snoop present a l'init, ou le mettre --pas necesaire
    return true;
}

//////////////////////////////////////////
//
// ** complete **
//////////////////////////////////////////
void Ipx800_v4::TimerHit()
{
    if (!isConnected()) {
        return; //  No need to reset timer if we are not connected anymore
	}
	LOG_DEBUG("TimerHit - Starting");
    if (DomeMotionSP.s == IPS_BUSY)
    {
        // Abort called
        if (MotionRequest < 0)
        {
            LOG_INFO("Roof motion is stopped.");
            setDomeState(DOME_IDLE);
            SetTimer(DEFAULT_POLLING_TIMER);
            return;
        }

        // Roll off is opening
        if (DomeMotionS[DOME_CW].s == ISS_ON)
        {
            if (getFullOpenedLimitSwitch())
            {
                LOG_INFO("Roof is open.");
                SetParked(false);
                return;
            }
        }
        // Roll Off is closing
        else if (DomeMotionS[DOME_CCW].s == ISS_ON)
        {
            if (getFullClosedLimitSwitch())
            {
                LOG_INFO("Roof is closed.");
                SetParked(true);
                return;
            }
        }

      SetTimer(DEFAULT_POLLING_TIMER);
    }
    updateIPXData();
	firstFonctionTabInit();
	setObsStatus();

    SetTimer(DEFAULT_POLLING_TIMER);
}
//////////////////////////////////////
/* Save conf */
bool Ipx800_v4::saveConfigItems(FILE *fp)
{
	INDI::Dome::saveConfigItems(fp);
    //RollOff::saveConfigItems(fp);
    IUSaveConfigText(fp, &LoginPwdTP);
	/** sauvegarde de la configuration des relais et entrées discretes **/ 
    for(int i=0;i<8;i++)
    {

        IUSaveConfigSwitch(fp, &RelaisInfoSP[i]);
        IUSaveConfigSwitch(fp, &DigitalInputSP[i]);
		//IUSaveConfigSwitch(fp, &RelaysStatesSP[i]);
		//IUSaveConfigSwitch(fp, &DigitsStatesSP[i]);
    }
    return true;
}
//////////////////////////////////////
/* Move Roof */
IPState Ipx800_v4::Move(DomeDirection dir, DomeMotionCommand operation)
{
	LOG_DEBUG("MOOOOOOVVVVVE");
    //LOGF_INFO("direction %s, motion %s",dir, operation);
	bool rc = false;
	LOGF_DEBUG("OPERATION : %d", operation);
	if (operation == MOTION_START)
    {
        // no way to choose open or close move, just choose to move
	/*	if (roof_Status == ROOF_CLOSED && getWeatherState() == IPS_ALERT)
        {
            LOG_WARN("Weather conditions are in the danger zone. Cannot open roof.");
            return IPS_ALERT;
        }//INDI::RollOff::isLocked()
        else if (mount_Status != BOTH_PARKED  && RollOff::isLocked())
        {
            DEBUG(INDI::Logger::DBG_WARNING,
                  "Cannot close dome when mount is locking. Telescope not parked, see parking policy, in options tab");
            return IPS_ALERT;
        }
        else if(mount_Status == BOTH_PARKED) {
            LOG_WARN("Roof is moving");
            int relayNumber = Relay_Fonction_Tab [ROOF_CONTROL_COMMAND];
           // isEngineOn = digitalState[Digital_Fonction_Tab[ROOF]];
            LOGF_DEBUG("Switching On Relay Number %d",relayNumber+1);
            rc = writeCommand(SetR, relayNumber+1);
        }
*/		
		if (mount_Status != BOTH_PARKED || engine_Powered == false) {
			LOG_WARN("Roof move cancelled. Mount or Roof's engine not ready");
			return IPS_ALERT;
		}
		else if (mount_Status == BOTH_PARKED && engine_Powered == true) {
			LOG_WARN("Roof is moving");
            int relayNumber = Relay_Fonction_Tab [ROOF_CONTROL_COMMAND];
           // isEngineOn = digitalState[Digital_Fonction_Tab[ROOF]];
            LOGF_DEBUG("Switching On Relay Number %d",relayNumber+1);
            rc = writeCommand(SetR, relayNumber+1);
			//readAnswer();
		}
		/**
		 if (dir == DOME_CW && fullOpenLimitSwitch == ISS_ON)
		 {
			LOG_WARN("Roof is already fully opened.");
			return IPS_ALERT;
		}
		else if (dir == DOME_CW && getWeatherState() == IPS_ALERT)
		{
			LOG_WARN("Weather conditions are in the danger zone. Cannot open roof.");
			return IPS_ALERT;
		}
		else if (dir == DOME_CCW && fullClosedLimitSwitch == ISS_ON)
		{
			LOG_WARN("Roof is already fully closed.");
			return IPS_ALERT;
		}
		else if (dir == DOME_CCW && INDI::Dome::isLocked())
		{
			DEBUG(INDI::Logger::DBG_WARNING,
				  "Cannot close dome when mount is locking. See: Telescope parking policy, in options tab");
			return IPS_ALERT;
		}
		else if(mount_Status == BOTH_PARKED) {
			int relayNumber = Relay_Fonction_Tab [ROOF_CONTROL_COMMAND];
           // isEngineOn = digitalState[Digital_Fonction_Tab[ROOF]];
            LOGF_DEBUG("Switching On Relay Number %d",relayNumber+1);
            rc = writeCommand(SetR, relayNumber+1);
		}*/
		else {
			LOGF_DEBUG("Move asked dir : %d", dir); 
		}
        fullOpenLimitSwitch   = ISS_OFF;
        fullClosedLimitSwitch = ISS_OFF;
        MotionRequest         = ROLLOFF_DURATION;
        gettimeofday(&MotionStart, nullptr);
        SetTimer(1000); // delai 20 sec
        return IPS_BUSY;
	}

    return (INDI::Dome::Abort() ? IPS_OK : IPS_ALERT);
}
//////////////////////////////////////
/* Park Roof */
IPState Ipx800_v4::Park()
{
	IPState rc = INDI::Dome::Move(DOME_CCW, MOTION_START);
    //IPState rc = RollOff::Move(DOME_CCW, MOTION_START);
    LOG_INFO("PARKKKKKK");
    if (rc == IPS_BUSY)
    {
        LOG_INFO("Roll off is parking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}
////////////////////////////////////
// Roof be open 
//
////////////////////////////////////

IPState Ipx800_v4::UnPark()
{
	IPState rc = INDI::Dome::Move(DOME_CW, MOTION_START);
    //IPState rc = RollOff::Move(DOME_CW, MOTION_START);
     LOG_INFO("UNNNNNPARKKKKKK");
    if (rc == IPS_BUSY)
    {
        LOG_INFO("Roll off is unparking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}

//////////////////////////////////////
/* Emergency stop */
bool Ipx800_v4::Abort()
{
    MotionRequest = -1;
    bool isEngineOn = false;
    bool rc = false;
    int relayNumber = Relay_Fonction_Tab [ROOF_ENGINE_POWER_SUPPLY];
    isEngineOn = digitalState[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]];
    
	if (isEngineOn == false) {
		LOG_WARN("Roof engine power supply already off."); 
		return rc;
	}
	else { // TO TEST 
		LOG_WARN("Emergency Stop");
		LOGF_DEBUG("Switching off Relay Number %d",relayNumber+1);
		rc = writeCommand(SetR, relayNumber+1);
		//readAnswer();
		// If both limit switches are off, then we're neither parked nor unparked.
		if ((fullOpenLimitSwitch == ISS_OFF && fullClosedLimitSwitch == ISS_OFF) or (mount_Status == UNKNOWN_STATUS))
		{
			LOG_DEBUG("Abort -  Idle state");
			IUResetSwitch(&ParkSP);
			ParkSP.s = IPS_IDLE;
			IDSetSwitch(&ParkSP, nullptr);
		}
		
		if (rc == true)
		{
			LOG_INFO("Roof Emergency Stop - Roof power supply switched OFF");
			//rc = INDI::RollOff::Abort();
			//rc = INDI::Dome::Abort();
			rc = updateIPXData(); //update digitals inputs and relays states
			firstFonctionTabInit();
			setObsStatus();
		}
		return rc; 
	}
}

//////////////////////////////////////
/* readCommand */
bool Ipx800_v4::readCommand(IPX800_command rCommand) {

    bool rc = false;
    std::string ipx_url = "";
    //int bytesWritten = 0, totalBytes = 0;
    switch (rCommand) {
    case GetR :
        ipx_url = "Get=R";
        LOG_DEBUG ("readCommand - Sending Get R...");
        break;
    case GetD :
        ipx_url = "Get=D";
        LOG_DEBUG ("readCommand - Sending Get D...");
        break;
    default :
    {
        LOGF_ERROR("readCommand - Unknown Command %s", rCommand);
        return false;
    }
	
    }
    LOGF_DEBUG ("readCommand - Sending %s",ipx_url.c_str());
    rc = writeTCP(ipx_url);

    return rc;
};

//////////////////////////////////////
/* writeCommand */
bool Ipx800_v4::writeCommand(IPX800_command wCommand, int toSet) {

    std::string ipx_url;
    std::string number;
    bool rc = false;
    //int bytesWritten = 0, totalBytes = 0;
    if (toSet <10)
        number = "0"+ std::to_string(toSet);
    else {
        number = std::to_string(toSet);
    }
    switch (wCommand) {
    case SetR :
        ipx_url = "SetR=" + number ;
        LOGF_DEBUG ("Sending Set R %s",number.c_str());
        break;
    case ClearR :
        ipx_url = "ClearR="+number;
        LOGF_DEBUG ("Sending Clear R %s",number.c_str());
        break;
    default :
    {
        LOGF_ERROR("Unknown Command %s", wCommand);
        return false;
    }

    }
    rc = writeTCP(ipx_url);
	readAnswer();
    return rc;
};

// Lit sur le port TCP la reponse a une requete GetR ou GetD sous forme de buffer
void Ipx800_v4::readAnswer(){
    int received = 0;
    int bytes, total = 0;
    int portFD = tcpConnection->getPortFD();
    char tmp[58] = "";
    total = 58;
	int i = 0;
    do {
        bytes = read(portFD,tmp+received,total-received);

        if (bytes < 0) {
            LOGF_ERROR("readAnswer - ERROR reading response from socket %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
			i++;
			if (i>2)
				break;
			}
        else if (bytes == 0) {
            LOG_INFO("readAnswer : end of stream");
            break; }
        received+=bytes;
    } while (received < total);

    LOGF_DEBUG("readAnswer - Longeur reponse : %i", received);
	
    strncpy(tmpAnswer,tmp,8);
	
    LOGF_DEBUG ("readAnswer - Reponse reçue : %s", tmpAnswer);

  };

//////////////////////////////////////
/* recordData */
void Ipx800_v4::recordData(IPX800_command recCommand) {
    int tmpDNumber, i = -1;
	int tmpDR = UNUSED_DIGIT;
	switch (recCommand) {
    case GetD :
			for (i=0;i<8;i++){
				DigitsStatesSP[i].s = IPS_OK;
				if (tmpAnswer[i] == '0') {
					LOGF_DEBUG("recordData - Digital Input N° %d is %s",i+1,"OFF");
					DigitsStatesSP[i].sp[0].s = ISS_OFF;
					DigitsStatesSP[i].sp[1].s = ISS_ON;
					digitalState[i] = false;}
				else if(tmpAnswer[i] == '1'){
					LOGF_DEBUG("recordData - Digital Input N° %d is %s",i+1,"ON");
					DigitsStatesSP[i].sp[0].s  = ISS_ON;
					DigitsStatesSP[i].sp[1].s = ISS_OFF;
					digitalState[i] = true;
				}
				tmpAnswer[i] = ' ';
			defineProperty(&DigitsStatesSP[i]);
			}
			
			for (i=0;i<10;i++) {
				tmpDR = Digital_Fonction_Tab[i];
				if (tmpDR >= 0) {
					 
					if (tmpDR == ROOF_ENGINE_POWERED ) {
						if (DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[0].s == ISS_OFF) {
							LOG_DEBUG("recordData - inverting ROOF_ENGINE_POWERED TO ON");
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[0].s  = ISS_ON;
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[1].s = ISS_OFF;
							digitalState[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]] = true; }
						else {
							LOG_DEBUG("recordData - inverting ROOF_ENGINE_POWERED TO OFF");
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[0].s = ISS_OFF;
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[1].s = ISS_ON;
							digitalState[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]] = false;}
						defineProperty(&DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]]);
					}
					else if(tmpDR == RASPBERRY_SUPPLIED) {
						if (DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[0].s == ISS_OFF) {
							LOG_DEBUG("recordData - inverting RASPBERRY_SUPPLIED TO ON");
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[0].s  = ISS_ON;
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[1].s = ISS_OFF;
							digitalState[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]] = true; }
						else {
							LOG_DEBUG("recordData - inverting RASPBERRY_SUPPLIED TO OFF");
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[0].s = ISS_OFF;
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[1].s = ISS_ON;
							digitalState[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]] = false;}
						defineProperty(&DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]]);
					}
					else if (tmpDR == MAIN_PC_SUPPLIED) { 
						if (DigitsStatesSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].sp[0].s == ISS_OFF) {
							LOG_DEBUG("recordData - inverting MAIN_PC_SUPPLIED TO ON");
							DigitsStatesSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].sp[0].s  = ISS_ON;
							DigitsStatesSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].sp[1].s = ISS_OFF;
							digitalState[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]] = true; }
						else {
							LOG_DEBUG("recordData - inverting MAIN_PC_SUPPLIED TO OFF");
							DigitsStatesSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].sp[0].s = ISS_OFF;
							DigitsStatesSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].sp[1].s = ISS_ON;
							digitalState[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]] = false;}
						defineProperty(&DigitsStatesSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]]);
					}
				}
				else {
					LOGF_WARN("Wrong Digital ROOF_ENGINE_POWERED initialisation. Now it's = %d",tmpDR);
				}
			}
			
			 
		break;
    case GetR :
        for (int i=0;i<8;i++){
            RelaysStatesSP[i].s = IPS_OK;
            if (tmpAnswer[i] == '0') {
                LOGF_DEBUG("recordData - Relay N° %d is %s",i+1,"OFF");
                RelaysStatesSP[i].sp[0].s = ISS_OFF;
                RelaysStatesSP[i].sp[1].s = ISS_ON;
                relayState[i]= false;

            }
            else {
                LOGF_DEBUG("recordData - Relay N° %d is %s",i+1,"ON");
                RelaysStatesSP[i].sp[0].s  = ISS_ON;
                RelaysStatesSP[i].sp[1].s  = ISS_OFF;
                relayState[i]=true;
            }
            tmpAnswer[i] = ' ';
			defineProperty(&RelaysStatesSP[i]);
        }
        break;
    default :
        LOGF_ERROR("recordData - Unknown Command %s", recCommand);
        break;
    }
	
    LOG_DEBUG("recordData - Switches States Recorded");

};

//////////////////////////////////////
/* writeTCP */
bool Ipx800_v4::writeTCP(std::string toSend) {

    int bytesWritten = 0, totalBytes = 0;
    totalBytes = toSend.length();
    int portFD = tcpConnection->getPortFD();

    LOGF_DEBUG("writeTCP - Command to send %s", toSend.c_str());
    LOGF_DEBUG ("writeTCP - Numéro de socket %i", portFD);

    if (!isSimulation()) {
        while (bytesWritten < totalBytes)
        {
            int bytesSent = write(portFD, toSend.c_str(), totalBytes - bytesWritten);
            if (bytesSent >= 0)
                bytesWritten += bytesSent;
            else
            {
                LOGF_ERROR("writeTCP - Error writing to IPX800 v4. %s", strerror(errno));
                return false;
            }
        }
    }

    LOGF_DEBUG ("writeTCP - octets à envoyer : %s", toSend.c_str());
    LOGF_DEBUG ("writeTCP - Nombre d'octets envoyé : %d", bytesWritten);
    return true;
}

float Ipx800_v4::CalcTimeLeft(timeval start)
{
    double timesince;
    double timeleft;
    struct timeval now { 0, 0 };
    gettimeofday(&now, nullptr);

    timesince =
        (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) - (double)(start.tv_sec * 1000.0 + start.tv_usec / 1000);
    timesince = timesince / 1000;
    timeleft  = MotionRequest - timesince;
    return timeleft;
}
//////////////////////////////////////
/* getFullOpenedLimitSwitch */
bool Ipx800_v4::getFullOpenedLimitSwitch()
{
    double timeleft = CalcTimeLeft(MotionStart);
	//setObsStatus();
	
    if (timeleft <= 0 && roof_Status == ROOF_IS_OPENED)
    {
        fullOpenLimitSwitch = ISS_ON;
        return true;
    }
    else
        return false;
}
//////////////////////////////////////
/* getFullClosedLimitSwitch */

bool Ipx800_v4::getFullClosedLimitSwitch()
{
    double timeleft = CalcTimeLeft(MotionStart);
	//setObsStatus();
	
    if (timeleft <= 0 && roof_Status == ROOF_IS_CLOSED)
    {	

        fullClosedLimitSwitch = ISS_ON;
        return true;
    }
    else
        return false;
}


//////////////////////////////////////
/* updateIPXData */

bool Ipx800_v4::updateIPXData()
{
    bool res = false;
	LOG_INFO("Updating IPX Data...");
    res = readCommand(GetR);
    if (res==false)
        LOG_ERROR("updateIPXData - Send Command GetR failed");
    else {
        LOG_INFO("updateIPXData - Send Command GetR successfull");
        readAnswer();
        if (!checkAnswer()) {
            LOG_ERROR("updateIPXData - Wrong Command GetR send");
            res = false;
        }
        else {
            recordData(GetR);
        }
    }
    if (res == false)
            return res;
    res = readCommand(GetD);
    if (res==false) {
        LOG_ERROR("updateIPXData - Send Command GetD failed");
    }
    else {
        LOG_INFO("updateIPXData - Send Command GetD successfull");
        readAnswer();
        if (!checkAnswer())
        {
            LOG_ERROR("updateIPXData - Wrong Command GetD send");
            res = false;
        }
        else {
            recordData(GetD); }
    }
    return res;
}

//////////////////////////////////////
/* setObsStatus */
void Ipx800_v4::setObsStatus()
{
    if (isConnected()) {
		LOG_INFO("Updating observatory status ...");
		//Mount STATUS definition
		// DecAxis RaAxis are digit input number to use
		int DecAxis = Digital_Fonction_Tab [DEC_AXIS_PARKED];
		int RaAxis =  Digital_Fonction_Tab [RA_AXIS_PARKED];
		
		if (digitalState[DecAxis] && digitalState[RaAxis])
			mount_Status = BOTH_PARKED;
		else if (digitalState[DecAxis] && !digitalState[RaAxis]) {
			mount_Status = DEC_PARKED; }
		else if (digitalState[RaAxis] && !digitalState[DecAxis])
			mount_Status = RA_PARKED;
		else {
			mount_Status = NONE_PARKED;
		}
		LOGF_DEBUG("setObsStatus - Dec Axis input  %d", DecAxis);
		LOGF_DEBUG("setObsStatus - Ra Axis input  %d", RaAxis);
		LOGF_DEBUG("setObsStatus - Dec Axis status  %d", digitalState[DecAxis]);
		LOGF_DEBUG("setObsStatus - Ra Axis status  %d", digitalState[RaAxis]);
		LOGF_DEBUG("setObsStatus - Mount Status %d", mount_Status);

		//Roof Status definition
		int openedRoof = Digital_Fonction_Tab [ROOF_OPENED];
		int closedRoof =  Digital_Fonction_Tab [ROOF_CLOSED];
		LOGF_DEBUG("setObsStatus - Roof openedRoof %d", digitalState[openedRoof]);
		LOGF_DEBUG("setObsStatus - Roof closedRoof %d", digitalState[closedRoof]);
		if (digitalState[openedRoof] && !digitalState[closedRoof]) {
			roof_Status = ROOF_IS_OPENED;
			setDomeState(DOME_UNPARKED);
			LOG_INFO("Roof is Open.");
		}
		else if (!digitalState[openedRoof] && digitalState[closedRoof]) {
			roof_Status = ROOF_IS_CLOSED; 
			setDomeState(DOME_PARKED);
			LOG_INFO("Roof is Closed.");}
		else {
			LOG_ERROR("Roof status unknown !");
			roof_Status = UNKNOWN_STATUS;
		}
		int tPower = Digital_Fonction_Tab [ROOF_ENGINE_POWERED];
		// Roof Engine status
		LOGF_DEBUG("setObsStatus - Roof engine input : %d", tPower+1); 
		//tPower = tPower +1;
		engine_Powered = digitalState[tPower];
		
		LOGF_DEBUG("setObsStatus - Roof Engine is (0 : Off, 1 : On) : %d", engine_Powered);
		
		LOGF_DEBUG("setObsStatus - Roof Status %d", roof_Status);
	}
}

//////////////////////////////////////
/* randomInit */
bool Ipx800_v4::firstFonctionTabInit()
{
	int currentDIndex = -1;
	int currentRIndex = -1;
	int cptD, cptR =0;
	for(int i=0;i<8;i++)
	{
		currentRIndex = IUFindOnSwitchIndex(&RelaisInfoSP[i]);
		if (currentRIndex != -1) {
			Relay_Fonction_Tab [currentRIndex] = i;
			LOGF_DEBUG("firstFonctionTabInit - Relay %d is supporting function %d ",i+1, currentRIndex);
			currentRIndex = -1; 
			//if (currentRIndex >0)
			//	cptR = cptR +1;
			}
		else
			LOGF_DEBUG("firstFonctionTabInit - Function unknown for Relay %d", i+1);
		
		currentDIndex = IUFindOnSwitchIndex(&DigitalInputSP[i]);
		if (currentDIndex != -1) {
			Digital_Fonction_Tab [currentDIndex] = i;
			LOGF_DEBUG("firstFonctionTabInit - Digital Input %d is supporting function %d ",i+1, currentDIndex);
			currentDIndex = -1;
			//if (currentDIndex >0)
			//	cptD = cptD +1;
			}
		else
			LOGF_DEBUG("firstFonctionTabInit - Function unknown for Digital Input %d", i+1);
	}
	//if (cptR != 0 && cptD != 0)
		//is_Initialized = true;
	
	return true;
}	

//////////////////////////////////////
/* checkAnswer */
bool Ipx800_v4::checkAnswer()
{
    for (int i=0;i<8;i++)
    {
        if ((tmpAnswer[i] == '0') || (tmpAnswer[i] == '1'))
        {
			return true;
        }
        else {
            LOGF_ERROR("Wrong data in IPX answer : %s", tmpAnswer[i]);
            return false;
        }
    }

    return true;
}
///////////////////////////////////////////////////////////
// A developper via connexion aag_cloudwatcher
///////////////////////////////////////////////////////////
IPState Ipx800_v4::getWeatherState()
{
	IPState weatherStatus = IPS_BUSY;
	weatherStatus = IPS_OK; 
	return weatherStatus;
} 
 

IText* Ipx800_v4::getMyLogin()
{
    return LoginPwdT;
}

ITextVectorProperty Ipx800_v4::getMyLoginVector()
{
     return LoginPwdTP;
}

ISwitchVectorProperty Ipx800_v4::getMyRelayVector(int i)
{
     return RelaisInfoSP[i];
}

ISwitchVectorProperty Ipx800_v4::getMyDigitsVector(int i)
{
     return DigitalInputSP[i];
}


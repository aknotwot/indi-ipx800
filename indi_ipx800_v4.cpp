/*******************************************************************************
This file is part of the IPX800 V4 INDI Driver.
A driver for the IPX800 (GCE Electronics - https://www.gce-electronics.com)

Copyright (C) 2024 Arnaud Dupont (aknotwot@protonmail.com)

IPX800 V4 INDI Driver is free software : you can redistribute it
and / or modify it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

IPX800 V4  INDI Driver is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the Lesser GNU General Public License
along with IPX800 V4  INDI Driver.  If not, see
< http : //www.gnu.org/licenses/>.

This driver is adapted from RollOff ino drivers developped by Jasem Mutlaq.
The main purpose of this driver is to connect to IPX to driver, communicate, and manage 
opening and closing of roof. 
It is able to read IPX800 digital datas to check status and position of the roof.
User can select, partially, for this first release, how IPX 800 is configured 
*******************************************************************************/

#include "indi_ipx800_v4.h"

#include "indicom.h"
#include "config.h"

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
#include <functional>
#include <regex>

//Network related includes:
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>

#define ROLLOFF_DURATION 30 // 30 seconds until roof is fully opened or closed
#define DEFAULT_POLLING_TIMER 2000

// Read only
#define ROOF_OPENED_SWITCH 0
#define ROOF_CLOSED_SWITCH 1

// Write only
#define ROOF_OPEN_RELAY     "OPEN"
#define ROOF_CLOSE_RELAY    "CLOSE"
#define ROOF_ABORT_RELAY    "ABORT"

// Rollfino
#define INACTIVE_STATUS  5 

// We declare an auto pointer to ipx800_v4.
std::unique_ptr<Ipx800_v4> ipx800v4(new Ipx800_v4());

void ISPoll(void *p);

/*************************************************************************/
/** Constructor                                                         **/
/*************************************************************************/

Ipx800_v4::Ipx800_v4() : INDI::InputInterface(this), INDI::OutputInterface(this)
{
    
	Roof_Status = UNKNOWN_STATUS;
	Mount_Status = NONE_PARKED;

	setVersion(IPX800_V4_VERSION_MAJOR,IPX800_V4_VERSION_MINOR);
}

/************************************************************************************
*
************************************************************************************/
bool Ipx800_v4::initProperties()
{
    LOG_INFO("Starting device...");
    
	INDI::DefaultDevice::initProperties();
	INDI::InputInterface::initProperties("Inputs&Outputs", DIGITAL_INTPUTS, 0, "Digital");
    INDI::OutputInterface::initProperties("Inputs&Outputs", RELAYS_OUTPUTS, "Relay");
		
   // SetParkDataType(PARK_NONE);
    //addDebugControl(); 
    addAuxControls();         // This is for standard controls not the local auxiliary switch
	addConfigurationControl();
	
    //Rolling list of possible functions managed by relays
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

    //TO Manage in a next release
    //IUFillText(&LoginPwdT[0], "LOGIN_VAL", "Login", "");
    //IUFillText(&LoginPwdT[1], "PASSWD_VAL", "Password", "");
    //IUFillTextVector(&LoginPwdTP, LoginPwdT, 2, getDeviceName(), "ACCESS_IPX", "IPX Access", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
	//defineProperty(&LoginPwdTP);
	// 
	
    //enregistrement des onglets de configurations
    for(int i=0;i<8;i++)
    {
        defineProperty(&RelaisInfoSP[i]);
        defineProperty(&DigitalInputSP[i]);
    }

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
	
	setDefaultPollingPeriod(2000);
	
	tcpConnection = new Connection::TCP(this);
	tcpConnection->setConnectionType(Connection::TCP::TYPE_TCP);
	tcpConnection->setDefaultHost("192.168.1.1");
	tcpConnection->setDefaultPort(666);
	
	LOG_DEBUG("Updating Connection - Handshake");
	tcpConnection->registerHandshake([&]()
	{
		LOG_DEBUG("Updating Connection - Call Handshake");
		return Handshake();
	}
	
	);
	registerConnection(tcpConnection);	
		
	return true;
}

bool Ipx800_v4::Handshake()
{
	bool status = false;
	bool res = false;
    if (isSimulation())
    {
        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());
        return true;
    }
	else {
		res = readCommand(GetR);
		readAnswer();
		if (res==false) {
			LOG_ERROR("Handshake with IPX800 failed");
			return false;
		}
		else {
			LOG_INFO("Handshake with IPX800 successfull");
			return true;
		}
		readAnswer();
		if (!checkAnswer())
		{
				LOG_ERROR("Handshake with IPX800 failed - Wrong answer");
				res = false;
		}
		else {
				recordData(GetR); 
		}
	}		
    return status;
}

void Ipx800_v4::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);

}

void ISSnoopDevice(XMLEle *root)
{
    ipx800v4->ISSnoopDevice(root);
}

bool Ipx800_v4::ISSnoopDevice(XMLEle *root)
{
    return INDI::DefaultDevice::ISSnoopDevice(root);
}

/********************************************************************************************
** Establish conditions on a connect.
*********************************************************************************************/
bool Ipx800_v4::setupParams()
{	
    LOG_DEBUG("Setting Params...");
    updateObsStatus(); 


    return true;
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    ipx800v4->ISNewSwitch(dev, name, states, names, n);
}

bool Ipx800_v4::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    ISwitch *myRelaisInfoS;
    ISwitchVectorProperty myRelaisInfoSP;

    ISwitch *myDigitalInputS;
    ISwitchVectorProperty myDigitalInputSP;

    int currentRIndex, currentDIndex =0;
    bool infoSet = false;
	
	
	// Make sure the call is for our device, and Fonctions Tab are initialized
   if(dev != nullptr && !strcmp(dev,getDeviceName()))
   {
	   if (INDI::OutputInterface::processSwitch(dev, name, states, names, n))
            return true;
		
		
		for(int i=0;i<8;i++)
		{
			myRelaisInfoSP = ipx800v4->getMyRelayVector(i);
			myDigitalInputSP = ipx800v4->getMyDigitsVector(i);
			
			////////////////////////////////////////////////////
			// Relay Configuration
			////////////////////////////////////////////////////
			
			
			if (!strcmp(name, myRelaisInfoSP.name))
				{
				if (INDI::OutputInterface::processSwitch(dev, name, states, names, n))
					
				
				LOGF_DEBUG("Relay function selected - SP : %s", myRelaisInfoSP.name);
				IUUpdateSwitch(&myRelaisInfoSP,states,names,n);
				
				myRelaisInfoS = myRelaisInfoSP.sp;
				myRelaisInfoSP.s = IPS_OK;
				IDSetSwitch(&myRelaisInfoSP,nullptr);
				
				currentRIndex = IUFindOnSwitchIndex(&myRelaisInfoSP);
				
				if (currentRIndex != -1) {
					Relay_Fonction_Tab [currentRIndex] = i;
					
					LOGF_DEBUG("Relay fonction index : %d", currentRIndex);

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
					defineProperty(&DigitsStatesSP[i]);
			
				 }
				else 
					LOG_DEBUG("No On Switches found"); 
				
				infoSet = true; 
			}
		}
		
		LOG_DEBUG("ISNewSwitch - First Init + UpDate");
		updateIPXData();
			
		if (infoSet) 
			updateObsStatus();
		
   }
   return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);	

}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    ipx800v4->ISNewText(dev, name, texts, names, n);
}

bool Ipx800_v4::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    ////////////////////////////////////////////////////
    // IPX Access - If password protected
	// To manage in a next release
    ////////////////////////////////////////////////////
    /*
	IText* myLoginT = ipx800v4->getMyLogin();
    ITextVectorProperty myLoginVector = ipx800v4->getMyLoginVector();
    if (dev && !strcmp(name, myLoginVector.name))
    {	
        IUUpdateText(&myLoginVector, texts, names, n);
        myLoginVector.s = IPS_OK;
        myPasswd = myLoginT[1].text;
        myLogin = myLoginT[0].text;
        IDSetText(&myLoginVector, nullptr);
		LOG_INFO("Password updated");
     }
	 */
	 
	 
	 if (INDI::InputInterface::processText(dev, name, texts, names, n))
            return true;
     if (INDI::OutputInterface::processText(dev, name, texts, names, n))
            return true;
	
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n); 
  }

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    ipx800v4->ISNewNumber(dev, name, values, names, n);
}

bool Ipx800_v4::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(RoofTimeoutNP.name, name))
        {
            IUUpdateNumber(&RoofTimeoutNP, values, names, n);
            RoofTimeoutNP.s = IPS_OK;
            IDSetNumber(&RoofTimeoutNP, nullptr);
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

///////////////////////////////////////////
// When IPX800 is connected two more tabs appear : Relays Status
//  Digital inputs Status
///////////////////////////////////////////
bool Ipx800_v4::Connect()
{
	bool status = INDI::DefaultDevice::Connect();
	LOG_DEBUG("Connecting to device...");

    return status;
    
}

bool Ipx800_v4::Disconnect()
{
    bool status = INDI::DefaultDevice::Disconnect();
    return status;
}

const char *Ipx800_v4::getDefaultName()
{
    return (const char *)"Ipx800 V4";
}

/////////////////////////////////////////
// Used after connection / Disconnection
/////////////////////////////////////////
bool Ipx800_v4::updateProperties()
{
	INDI::DefaultDevice::updateProperties();
	
	LOG_DEBUG("updateProperties - Starting");
	///////////////////////////
    if (isConnected())
    { // Connect both states tabs 
		updateIPXData();
		//firstFonctionTabInit();
		//updateObsStatus();
		INDI::InputInterface::updateProperties();
		INDI::OutputInterface::updateProperties();
        for(int i=0;i<8;i++)
        {
            defineProperty(&RelaysStatesSP[i]);
			//MàJ DomeState
        }
        for(int i=0;i<8;i++)
        {
             defineProperty(&DigitsStatesSP[i]);
        }
	
        setupParams(); 

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
    return true;
}

//////////////////////////////////////////
// ** complete **
//////////////////////////////////////////
void Ipx800_v4::TimerHit()
{
  
    if (!isConnected())  {
        return; //  No need to reset timer if we are not connected anymore
	}
	
	updateIPXData();
    
    SetTimer(getPollingPeriod());
}
//////////////////////////////////////
/* Save conf */
bool Ipx800_v4::saveConfigItems(FILE *fp)
{

	INDI::DefaultDevice::saveConfigItems(fp);
    //IUSaveConfigText(fp, &LoginPwdTP);
	
	/** sauvegarde de la configuration des relais et entrées discretes **/ 
    ////////////////////////////
	for(int i=0;i<8;i++)
    {
        IUSaveConfigSwitch(fp, &RelaisInfoSP[i]);
        IUSaveConfigSwitch(fp, &DigitalInputSP[i]);
    }
	INDI::InputInterface::saveConfigItems(fp);
    INDI::OutputInterface::saveConfigItems(fp);
    return true;////////
	
}
//////////////////////////////////////
/* Move Roof */
/*
 * Direction: DOME_CW Clockwise = Open; DOME-CCW Counter clockwise = Close
 * Operation: MOTION_START, | MOTION_STOP
 */
 /*
IPState Ipx800_v4::Move(DomeDirection dir, DomeMotionCommand operation)
{
	LOG_DEBUG("MOOOOOOVVVVVE");
    //LOGF_INFO("direction %s, motion %s",dir, operation);
	bool rc = false;
    updateObsStatus();

	LOGF_DEBUG("OPERATION : %d", operation);
	if (operation == MOTION_START)
    {
		if (roofOpening)
        {
            LOG_WARN("Roof is in process of opening, wait for completion or abort current operation");
            return IPS_OK;
        }
        if (roofClosing)
        {
            LOG_WARN("Roof is in process of closing, wait for completion or abort current operation");
            return IPS_OK;
        }

        // Open Roof
        // DOME_CW --> OPEN. If we are asked to "open" while we are fully opened as the
        // limit switch indicates, then we simply return false.
        if (dir == DOME_CW)
        {
            if (fullyOpenedLimitSwitch == ISS_ON)
            {
                LOG_WARN("DOME_CW directive received but roof is already fully opened");
                SetParked(false);
                return IPS_ALERT;
            }
			if (mount_Status != BOTH_PARKED || engine_Powered == false) {
				LOG_WARN("Roof move cancelled. Mount not parked or Roof's engine not powered on");
				return IPS_ALERT;
			}
            // Initiate action
			if (mount_Status == BOTH_PARKED && engine_Powered == true) {
				LOG_WARN("Roof is moving");
				roofOpening = true;
                roofClosing = false;
                LOG_INFO("Roof is opening...");
				int relayNumber = Relay_Fonction_Tab [ROOF_CONTROL_COMMAND];
				// isEngineOn = digitalState[Digital_Fonction_Tab[ROOF]];
				LOGF_DEBUG("Switching On Relay Number %d",relayNumber+1);
				rc = writeCommand(SetR, relayNumber+1);
				//readAnswer();
			}
            else
            {
                LOG_WARN("Failed to operate controller to open roof");
                return IPS_ALERT;
            }
        }

        // Close Roof
        else if (dir == DOME_CCW)
        {
            if (fullyClosedLimitSwitch == ISS_ON)
            {
                SetParked(true);
                LOG_WARN("DOME_CCW directive received but roof is already fully closed");
                return IPS_ALERT;
            }
			if (mount_Status != BOTH_PARKED || engine_Powered == false) {
				LOG_WARN("Roof move cancelled. Mount not parked or Roof's engine not powered on");
				return IPS_ALERT;
			}
            // Initiate action
			if (mount_Status == BOTH_PARKED && engine_Powered == true) {
				LOG_WARN("Roof is moving");
				roofOpening = false;
                roofClosing = true;
                LOG_INFO("Roof is closing...");
				int relayNumber = Relay_Fonction_Tab [ROOF_CONTROL_COMMAND];
				// isEngineOn = digitalState[Digital_Fonction_Tab[ROOF]];
				LOGF_DEBUG("Switching On Relay Number %d",relayNumber+1);
				rc = writeCommand(SetR, relayNumber+1);
				//readAnswer();
			}
            else
            {
                LOG_WARN("Failed to operate controller to close roof");
                return IPS_ALERT;
            }
        }
		roofTimedOut = EXPIRED_CLEAR;
        MotionRequest = (int)RoofTimeoutN[0].value;
        LOGF_DEBUG("Roof motion timeout setting: %d", (int)MotionRequest);
        gettimeofday(&MotionStart, nullptr);
        SetTimer(1000);
        return IPS_BUSY;
    }
    return    IPS_ALERT;
}

*/

//////////////////////////////////////
/* Park Roof */
//TODO renvoit State vs ipstate??
/*
IPState Ipx800_v4::Park()
{
	IPState rc = INDI::DefaultDevice::Move(DOME_CCW, MOTION_START);
    
    LOG_INFO("PARKKKKKK");
    if (rc == IPS_BUSY)
    {
        LOG_INFO("Roll off is parking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}
*/
////////////////////////////////////
// Roof be open 
//
////////////////////////////////////
/*
IPState Ipx800_v4::UnPark()
{
	IPState rc = INDI::DefaultDevice::Move(DOME_CW, MOTION_START);
    LOG_INFO("UNNNNNPARKKKKKK");
    if (rc == IPS_BUSY)
    {
        LOG_INFO("Roll off is unparking...");
        return IPS_BUSY;
    }
    else
        return IPS_ALERT;
}
*/
//////////////////////////////////////
/* Emergency stop */
/*
bool Ipx800_v4::Abort()
{
	bool openState;
    bool closeState;

    updateObsStatus();
    openState = (fullyOpenedLimitSwitch == ISS_ON);
    closeState = (fullyClosedLimitSwitch == ISS_ON);

    MotionRequest = -1;
    bool isEngineOn = false;
    bool rc = false;
    int relayNumber = Relay_Fonction_Tab [ROOF_ENGINE_POWER_SUPPLY];
    isEngineOn = digitalState[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]];
    
	if (isEngineOn == false) {
		LOG_WARN("Roof engine power supply already off."); 
		return true;
	}
	else if (closeState && DomeMotionSP.getState() != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be closed and stationary, no action taken on abort request");
        return true;
    }
    else if (openState && DomeMotionSP.getState() != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be open and stationary, no action taken on abort request");
        return true;
    }
	else if (DomeMotionSP.getState() != IPS_BUSY)
    {
        LOG_WARN("Roof appears to be partially open and stationary, no action taken on abort request");
    }
    else if (DomeMotionSP.getState() == IPS_BUSY)
    {
        if (DomeMotionSP[DOME_CW].getState() == ISS_ON)
        {
            LOG_WARN("Abort roof action requested while the roof was opening. Direction correction may be needed on the next move request.");
        }
        else if (DomeMotionSP[DOME_CCW].getState() == ISS_ON)
        {
            LOG_WARN("Abort roof action requested while the roof was closing. Direction correction may be needed on the next move request.");
        }
        roofClosing = false;
        roofOpening = false;
        MotionRequest = -1;
		LOG_WARN("Emergency Stop");
		LOGF_DEBUG("Switching off Relay Number %d",relayNumber+1);
		rc = writeCommand(SetR, relayNumber+1);
		if (rc == true)
		{
			LOG_INFO("Roof Emergency Stop - Roof power supply switched OFF");
			rc = updateIPXData(); //update digitals inputs and relays states
			//firstFonctionTabInit();
			updateObsStatus();
		}
    }
	
	if ((fullyOpenedLimitSwitch == ISS_OFF && fullyClosedLimitSwitch == ISS_OFF) or (mount_Status == UNKNOWN_STATUS))
		{
			LOG_DEBUG("Abort -  Idle state");

		    ParkSP.reset(); // IUResetSwitch(&ParkSP);
			ParkSP.setState(IPS_IDLE);
			ParkSP.apply();
			//IDSetSwitch(&ParkSP, nullptr);
		}
		
	if (rc == true)
		{
			LOG_INFO("Roof Emergency Stop - Roof power supply switched OFF");
			//rc = INDI::RollOff::Abort();
			//rc = INDI::Dome::Abort();
			rc = updateIPXData(); //update digitals inputs and relays states
			firstFonctionTabInit();
			updateObsStatus();
		}
	
	return rc; 

}
*/
//////////////////////////////////////
/* readCommand */
bool Ipx800_v4::readCommand(IPX800_command rCommand) {

    bool rc = false;
    std::string ipx_url = "";
    //int bytesWritten = 0, totalBytes = 0;
    switch (rCommand) {
		case GetR :
			ipx_url = "Get=R";
			break;
		case GetD :
			ipx_url = "Get=D";
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
	
    return rc;
};
//////////////////////////////////////
/* readAnswer */
// TCP Answer reading 
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
    int i = -1;
	int tmpDR = UNUSED_DIGIT;
	switch (recCommand) {
    case GetD :
			for (i=0;i<8;i++){				
				DigitsStatesSP[i].s = IPS_OK;
				DigitalInputsSP[i].reset();
				if (tmpAnswer[i] == '0') {
					LOGF_DEBUG("recordData - Digital Input N° %d is %s",i+1,"OFF");
					DigitalInputsSP[i][0].setState(ISS_ON);
					DigitsStatesSP[i].sp[0].s = ISS_OFF;
					DigitsStatesSP[i].sp[1].s = ISS_ON;
					digitalState[i] = false;}
				else if(tmpAnswer[i] == '1'){
					LOGF_DEBUG("recordData - Digital Input N° %d is %s",i+1,"ON");
					DigitalInputsSP[i][1].setState(ISS_ON);
					DigitsStatesSP[i].sp[0].s = ISS_ON ;
					DigitsStatesSP[i].sp[1].s = ISS_OFF;
					digitalState[i] = true;
				}
				tmpAnswer[i] = ' ';
			    DigitalInputsSP[i].setState(IPS_OK);
                DigitalInputsSP[i].apply();
			    defineProperty(&DigitsStatesSP[i]);
				DigitsStatesSP[i].apply();
			}
			
			for (i=0;i<10;i++) {
				tmpDR = Digital_Fonction_Tab[i];
				if (tmpDR >= 0) {
					 
					if (tmpDR == ROOF_ENGINE_POWERED ) {
						if (DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[0].s == ISS_OFF) {
							LOG_DEBUG("recordData - inverting ROOF_ENGINE_POWERED TO ON");
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[0].s  = ISS_ON;
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[1].s = ISS_OFF;
							digitalState[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]] = true;
							DigitalInputsSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]][0].setState(ISS_ON);}
						else {
							LOG_DEBUG("recordData - inverting ROOF_ENGINE_POWERED TO OFF");
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[0].s = ISS_OFF;
							DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].sp[1].s = ISS_ON;
							digitalState[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]] = false;
							DigitalInputsSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]][1].setState(ISS_ON);}
						
						
						DigitalInputsSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].setState(IPS_OK);
						DigitalInputsSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]].apply();		
						defineProperty(&DigitsStatesSP[Digital_Fonction_Tab[ROOF_ENGINE_POWERED]]);
					}
					else if(tmpDR == RASPBERRY_SUPPLIED) {
						if (DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[0].s == ISS_OFF) {
							LOG_DEBUG("recordData - inverting RASPBERRY_SUPPLIED TO ON");
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[0].s  = ISS_ON;
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[1].s = ISS_OFF;
							digitalState[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]] = true;
							DigitalInputsSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]][0].setState(ISS_ON);}
						else {
							LOG_DEBUG("recordData - inverting RASPBERRY_SUPPLIED TO OFF");
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[0].s = ISS_OFF;
							DigitsStatesSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].sp[1].s = ISS_ON;
							digitalState[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]] = false;
							DigitalInputsSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]][1].setState(ISS_ON);}
							
						DigitalInputsSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].setState(IPS_OK);
						DigitalInputsSP[Digital_Fonction_Tab[RASPBERRY_SUPPLIED]].apply();	
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
						
						DigitalInputsSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].setState(IPS_OK);
						DigitalInputsSP[Digital_Fonction_Tab[MAIN_PC_SUPPLIED]].apply();	
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
			DigitalOutputsSP[i].reset();
            if (tmpAnswer[i] == '0') {
                LOGF_DEBUG("recordData - Relay N° %d is %s",i+1,"OFF");
                RelaysStatesSP[i].sp[0].s = ISS_OFF;
                RelaysStatesSP[i].sp[1].s = ISS_ON;
				DigitalOutputsSP[i][0].setState(ISS_ON);
                relayState[i]= false;
            }
            else {
                LOGF_DEBUG("recordData - Relay N° %d is %s",i+1,"ON");
                RelaysStatesSP[i].sp[0].s  = ISS_ON;
                RelaysStatesSP[i].sp[1].s  = ISS_OFF;
				DigitalOutputsSP[i][1].setState(ISS_ON);
                relayState[i]=true;
            }
            tmpAnswer[i] = ' ';
			DigitalOutputsSP[i].setState(IPS_OK);
            DigitalOutputsSP[i].apply();
			defineProperty(&RelaysStatesSP[i]);
			RelaysStatesSP[i].apply();
        }
        break;
    default :
        LOGF_ERROR("recordData - Unknown Command %s", recCommand);
        break;
    }
	
    LOG_DEBUG("recordData - Switches States Recorded");

};

//////////////////////////////////////
/* writeTCP Write Command on TCP socket */
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
                LOGF_ERROR("writeTCP - Error request to IPX800 v4. %s", strerror(errno));
                return false;
            }
        }
    }

    LOGF_DEBUG ("writeTCP - bytes to send : %s", toSend.c_str());
    LOGF_DEBUG ("writeTCP - Number of bytes sent : %d", bytesWritten);
    return true;
}

//////////////////////////////////////
/* getFullOpenedLimitSwitch */
/*
bool Ipx800_v4::getFullOpenedLimitSwitch(bool* switchState)
{
    if (isSimulation())
    {
        if (simRoofOpen)
        {
            fullyOpenedLimitSwitch = ISS_ON;
            *switchState = true;
        }
        else
        {
            fullyOpenedLimitSwitch = ISS_OFF;
            *switchState = false;
        }
        return true;
    }

    if (readRoofSwitch(ROOF_OPENED_SWITCH, switchState))
    {
        if (*switchState)
            fullyOpenedLimitSwitch = ISS_ON;
        else
            fullyOpenedLimitSwitch = ISS_OFF;
        return true;
    }
    else
    {
        LOG_WARN("Unable to obtain from the controller whether or not the roof is opened");
        return false;
    }
}
*/
//////////////////////////////////////
/* getFullClosedLimitSwitch */
/*
bool Ipx800_v4::getFullClosedLimitSwitch(bool* switchState)
{
    if (isSimulation())
    {
        if (simRoofClosed)
        {
            fullyClosedLimitSwitch = ISS_ON;
            *switchState = true;
        }
        else
        {
            fullyClosedLimitSwitch = ISS_OFF;
            *switchState = false;
        }
        return true;
    }

    if (readRoofSwitch(ROOF_CLOSED_SWITCH, switchState))
    {
        if (*switchState)
            fullyClosedLimitSwitch = ISS_ON;
        else
            fullyClosedLimitSwitch = ISS_OFF;
        return true;
    }
    else
    {
        LOG_WARN("Unable to obtain from the controller whether or not the roof is closed");
        return false;
    }
}

*/
//////////////////////////////////////
// readRoofSwitch
/*
 * If unable to determine switch state due to errors, return false.
 * If no errors return true. Return in result true if switch and false if switch off.
 */
bool Ipx800_v4::readRoofSwitch(const int roofSwitchId, bool *result)
{   
	/*
    bool status;
	// TODO modifier le type de roofswitchid (entier c'est mieux)
	// ecrire comparason roofswitchid vs roof_stauts
	if ( roofSwitchId == roof_Status)
		return true;
	else 
		return false; 
    //Roof Status definition
	int openedRoof = Digital_Fonction_Tab [ROOF_OPENED];
	int closedRoof =  Digital_Fonction_Tab [ROOF_CLOSED];
	LOGF_DEBUG("updateObsStatus - Roof openedRoof %d", digitalState[openedRoof]);
	LOGF_DEBUG("updateObsStatus - Roof closedRoof %d", digitalState[closedRoof]);
	if (digitalState[openedRoof] && !digitalState[closedRoof]) {
		roof_Status = ROOF_IS_OPENED;
	}
	else if (!digitalState[openedRoof] && digitalState[closedRoof]) {
		roof_Status = ROOF_IS_CLOSED; 
	}
	*/
	return result;
	
}

//////////////////////////////////////
/* updateIPXData */
/* prepare commands to update Inputs Status */
//////////////////////////////////////
bool Ipx800_v4::updateIPXData()
{
    bool res = false;
	LOG_INFO("Updating IPX Data...");
	
    res = ipx800v4->UpdateDigitalOutputs();
    if (res==false) {
        LOG_ERROR("updateIPXData - Send Command GetR failed");
		return false;
	}
   
    res = ipx800v4->UpdateDigitalInputs();
    if (res==false) {
        LOG_ERROR("updateIPXData - Send Command GetD failed");
		return false;
    }
  
    return true;
}

//////////////////////////////////////
/* updateObsStatus */
void Ipx800_v4::updateObsStatus()
{
	bool openedState = false;
    bool closedState = false;

  //  getFullOpenedLimitSwitch(&openedState);
   // getFullClosedLimitSwitch(&closedState);
	
	if (!openedState && !closedState && !roofOpening && !roofClosing) {
        DEBUG(INDI::Logger::DBG_WARNING, "Roof stationary, neither opened or closed, adjust to match PARK button");
    }
	if (openedState && closedState) {
        DEBUG(INDI::Logger::DBG_WARNING, "Roof showing it is both opened and closed according to the controller");
	}
	
	RoofStatusL[ROOF_STATUS_OPENED].s = IPS_IDLE;
    RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_IDLE;
    RoofStatusL[ROOF_STATUS_MOVING].s = IPS_IDLE;
    RoofStatusLP.s = IPS_IDLE;
	
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
		LOGF_DEBUG("updateObsStatus - Dec Axis input  %d", DecAxis);
		LOGF_DEBUG("updateObsStatus - Ra Axis input  %d", RaAxis);
		LOGF_DEBUG("updateObsStatus - Dec Axis status  %d", digitalState[DecAxis]);
		LOGF_DEBUG("updateObsStatus - Ra Axis status  %d", digitalState[RaAxis]);
		LOGF_DEBUG("updateObsStatus - Mount Status %d", mount_Status);

		//Roof Status definition
		int openedRoof = Digital_Fonction_Tab [ROOF_OPENED];
		int closedRoof =  Digital_Fonction_Tab [ROOF_CLOSED];
		LOGF_DEBUG("updateObsStatus - Roof openedRoof %d", digitalState[openedRoof]);
		LOGF_DEBUG("updateObsStatus - Roof closedRoof %d", digitalState[closedRoof]);
		if (digitalState[openedRoof] && !digitalState[closedRoof]) {
			roof_Status = ROOF_IS_OPENED;
			RoofStatusL[ROOF_STATUS_OPENED].s = IPS_OK;
			RoofStatusLP.s = IPS_OK;
			roofOpening = false;
			//setDomeState(DOME_UNPARKED);
			LOG_INFO("Roof is Open.");
		}
		else if (!digitalState[openedRoof] && digitalState[closedRoof]) {
			roof_Status = ROOF_IS_CLOSED; 
			//setDomeState(DOME_PARKED);
			roofClosing = false;
            RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_OK;
            RoofStatusLP.s = IPS_OK;
			LOG_INFO("Roof is Closed.");
		}
		else if (roofOpening || roofClosing) {
			
            if (roofOpening)
            {
                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_BUSY;
                RoofStatusL[ROOF_STATUS_MOVING].s = IPS_BUSY;
            }
            else if (roofClosing)
            {
                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_BUSY;
                RoofStatusL[ROOF_STATUS_MOVING].s = IPS_BUSY;
            }
            RoofStatusLP.s = IPS_BUSY;
        }
		else {
			LOG_ERROR("Roof status unknown !");
			roof_Status = UNKNOWN_STATUS;
			if (roofTimedOut == EXPIRED_OPEN)
                RoofStatusL[ROOF_STATUS_OPENED].s = IPS_ALERT;
            else if (roofTimedOut == EXPIRED_CLOSE)
                RoofStatusL[ROOF_STATUS_CLOSED].s = IPS_ALERT;
            RoofStatusLP.s = IPS_ALERT;
		}
		int tPower = Digital_Fonction_Tab [ROOF_ENGINE_POWERED];
		// Roof Engine status
		LOGF_DEBUG("updateObsStatus - Roof engine input : %d", tPower+1); 
		//tPower = tPower +1;
		engine_Powered = digitalState[tPower];
		
		LOGF_DEBUG("updateObsStatus - Roof Engine is (0 : Off, 1 : On) : %d", engine_Powered);
		
		LOGF_DEBUG("updateObsStatus - Roof Status %d", roof_Status);
	}
	IDSetLight(&RoofStatusLP, nullptr);
}

//////////////////////////////////////
/* randomInit */
bool Ipx800_v4::firstFonctionTabInit()
{
	int currentDIndex = -1;
	int currentRIndex = -1;
	//int cptD, cptR =0;
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
/*
Password Management for a next release
IText* Ipx800_v4::getMyLogin()
{
    return LoginPwdT;
}

ITextVectorProperty Ipx800_v4::getMyLoginVector()
{
     return LoginPwdTP;
}
*/

ISwitchVectorProperty Ipx800_v4::getMyRelayVector(int i)
{
     return RelaisInfoSP[i];
}

ISwitchVectorProperty Ipx800_v4::getMyDigitsVector(int i)
{
     return DigitalInputSP[i];
}


//////////////////////////////////////
/* UpdateDigitalInputs */
bool Ipx800_v4::UpdateDigitalInputs() 
{	
	// update of all digital inputs
	bool res = readCommand(GetD);
	readAnswer();
	if (res==false) {
		LOG_ERROR("UpdateDigitalInputs - Send Command GetD failed");
		return res;
		}
	else {
		LOG_INFO("UpdateDigitalInputs - Send Command GetD successfull");
		
		if (!checkAnswer())
		{
			LOG_ERROR("UpdateDigitalInputs - Wrong Command GetD send");
			res = false;
		}
		else {
			recordData(GetD); 
		}
	}
	return true; 
}

bool Ipx800_v4::UpdateAnalogInputs()
{
	return true;
}

//////////////////////////////////////
/* UpdateDigitalOutputs */
// Update Relays Status
bool Ipx800_v4::UpdateDigitalOutputs()
{
			// update of all digital inputs
		bool res = readCommand(GetR);
		readAnswer();
		if (res==false) {
			LOG_ERROR("UpdateDigitalOutputs - Send Command GetR failed");
			
			return res;
			}
		else {
			LOG_INFO("UpdateDigitalOutputs - Send Command GetR successfull");
			
			if (!checkAnswer())
			{
				LOG_ERROR("UpdateDigitalOutputs - Wrong Command GetR send");
				res = false;
			}
			else {
				recordData(GetR); 
			}
		}
		return true;
}

//////////////////////////////////////
/* CommandOutput */
//  
bool Ipx800_v4::CommandOutput(uint32_t index, OutputState command) 
{
	int relayNumber = index;
	bool rc = writeCommand(SetR, relayNumber);
	readAnswer();
	// update digits & relays state
	return rc;
}

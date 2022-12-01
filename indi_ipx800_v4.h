/*******************************************************************************
This file is part of the IPX800 V4 INDI Driver.
A driver for the IPX800 (AAGware - http : //www.aagware.eu/)

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
#pragma once

#include "indidome.h"
#include "roll_off.h"
//class Ipx800_v4 : public INDI::RollOff
class Ipx800_v4 : public RollOff
{
  public:
  
	Ipx800_v4();
    virtual ~Ipx800_v4();
  
    bool Connect();
    bool Disconnect();
  
	const char *getDefaultName();
	
	bool initProperties();
    bool updateProperties();
  
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    //virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
	virtual bool saveConfigItems(FILE *fp) override;
    virtual bool ISSnoopDevice(XMLEle *root);
   
    IText* getMyLogin();
    ITextVectorProperty getMyLoginVector();
    ISwitchVectorProperty getMyRelayVector(int i);
    ISwitchVectorProperty getMyDigitsVector(int i);

  protected:

    enum IPX800_command {
       GetR   = 1 << 0,
       GetD  = 1 << 1,
       SetR = 1 << 2,
       ClearR = 1 << 3
   } ;
   
    void TimerHit();

    virtual IPState Move(DomeDirection dir, DomeMotionCommand operation);
    virtual IPState Park();
    virtual IPState UnPark();
    virtual bool Abort();

    virtual bool getFullOpenedLimitSwitch();
    virtual bool getFullClosedLimitSwitch();
    
	///////////////////////////////////////////
	// IPX800 Communication
	///////////////////////////////////////////
	bool updateIPXData();
    void setObsStatus();
    bool readCommand(IPX800_command);
    bool writeCommand(IPX800_command, int toSet);
    bool checkAnswer();
    void readAnswer();
    void recordData(IPX800_command command);
    bool writeTCP(std::string toSend);
	IPState getWeatherState();

  private:
	// List of possible commands 
    enum {
        UNUSED_RELAY,
        ROOF_ENGINE_POWER_SUPPLY,
        TUBE_VENTILATION,
        HEATING_RESISTOR_1,
        HEATING_RESISTOR_2,
        ROOF_CONTROL_COMMAND,
        MOUNT_POWER_SUPPLY,
        CAM_POWER_SUPPLY,
        OTHER_POWER_SUPPLY_1,
        OTHER_POWER_SUPPLY_2,
        OTHER_POWER_SUPPLY_3
        }IPXRelaysCommands;

	// List of possibles digital inputs
    enum  {
        UNUSED_DIGIT,
        DEC_AXIS_PARKED,
        RA_AXIS_PARKED,
        ROOF_OPENED,
        ROOF_CLOSED,
        ROOF_ENGINE_POWER_SUPPLIED,
        RASPBERRY_SUPPLIED,
        MAIN_PC_SUPPLIED,
        OTHER_DIGITAL_1,
        OTHER_DIGITAL_2 }IPXDigitalRead;
	
    char tmpAnswer[8]= {0};
    bool SetupParms();
    float CalcTimeLeft(timeval);

    ISState fullOpenLimitSwitch { ISS_ON };
    ISState fullClosedLimitSwitch { ISS_OFF };
    double MotionRequest { 0 };
    struct timeval MotionStart { 0, 0 };

    ISwitch RelaisInfoS[11] {};
    ISwitch Relais1InfoS[11], Relais2InfoS[11],Relais3InfoS[11],Relais4InfoS[11],Relais5InfoS[11],Relais6InfoS[11],Relais7InfoS[11],Relais8InfoS[11] {};
    ISwitchVectorProperty RelaisInfoSP[8] {};

    ISwitch DigitalInputS[10] {};
    ISwitch Digital1InputS[10], Digital2InputS[10], Digital3InputS[10], Digital4InputS[10], Digital5InputS[10], Digital6InputS[10], Digital7InputS[10], Digital8InputS[10];
    ISwitchVectorProperty DigitalInputSP[8] {};

    ISwitch Relay1StateS[2],Relay2StateS[2],Relay3StateS[2],Relay4StateS[2],Relay5StateS[2],Relay6StateS[2],Relay7StateS[2],Relay8StateS[2] ;
    ISwitchVectorProperty RelaysStatesSP[8] ;
    ISwitch Digit1StateS[2],Digit2StateS[2],Digit3StateS[2],Digit4StateS[2],Digit5StateS[2],Digit6StateS[2],Digit7StateS[2],Digit8StateS[2] ;
    ISwitchVectorProperty DigitsStatesSP[8] ;

    //Gestion Acces IPX
    IText LoginPwdT[2];
    ITextVectorProperty LoginPwdTP;

    enum {
        ROOF_IS_OPENED ,
        ROOF_IS_CLOSED ,
        UNKNOWN_STATUS
    }
    Roof_Status;

    enum {
        RA_PARKED    ,
        DEC_PARKED   ,
        BOTH_PARKED  ,
        NONE_PARKED
    }
    Mount_Status;

	const char *ROLLOFF_TAB        = "Roll Off";
	const char *RELAYS_CONFIGURATION_TAB        = "Relays";
	const char *DIGITAL_INPUT_CONFIGURATION_TAB        = "Digitals Inputs";
	const char *RAW_DATA_TAB = "States";

    // Identification des relais, indexés par fonction. le tableau renvoie le numéro du relais. 
	// Les fonctions sont ordonnées comme suit
    //int Relay_Fonction_Tab [11] = {0;0};
	int Relay_Fonction_Tab [11] = {0};
    /* 0: UNUSED_RELAY,
    ROOF_ENGINE_POWER_SUPPLY,
    TUBE_VENTILATION,
    HEATING_RESISTOR_1,
    HEATING_RESISTOR_2,
    ROOF_CONTROL_COMMAND,
    MOUNT_POWER_SUPPLY,
    CAM_POWER_SUPPLY,
    OTHER_POWER_SUPPLY_1,
    OTHER_POWER_SUPPLY_2,
    10 : OTHER_POWER_SUPPLY_3 */

    bool relayState[8];
    bool digitalState[8];

    // Identification des entrees numeriques, indexées par fonction. le tableau renvoie le numéro du relais. 
	// Les fonctions sont ordonnées comme suit
    int Digital_Fonction_Tab [10];
    /*
       0:  UNUSED_DIGIT,
        DEC_AXIS_PARKED,
        RA_AXIS_PARKED,
        ROOF_OPENED,
        ROOF_CLOSED,
        ROOF_ENGINE_POWER_SUPPLIED,
        RASPBERRY_SUPPLIED,
        MAIN_PC_SUPPLIED,
        OTHER_DIGITAL_1,
        9 : OTHER_DIGITAL_2
    */
    int mount_Status = RA_PARKED | DEC_PARKED | BOTH_PARKED | NONE_PARKED;
    int roof_Status  = ROOF_IS_OPENED | ROOF_IS_CLOSED | UNKNOWN_STATUS;

    std::string myPasswd = "";
    std::string myLogin = "";

};

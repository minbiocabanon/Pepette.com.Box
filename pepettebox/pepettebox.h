//--------------------------------------------------
//! \file		pepettebox.h
//! \brief		header file for structures and enumerate
//! \date		2015-Mar
//! \author		minbiocabanon
//--------------------------------------------------

// Analog input
#define NB_SAMPLE_ANALOG		16
#define VCC_5V					4.97
#define VOLT_DIVIDER_INPUT		10.54123	// Voltage divider ratio for mesuring input voltage. 
#define MIN_DC_IN				9			// Minimum input voltage
#define MAX_DC_IN				36			// Max input voltage


// Lipo
// battery level trigger for alarm , in % , WARNING, LIPO level are only 100,66 and 33%
// Do not use value < 33% because linkitone will not give you another value until 0% ...
#define LIPO_LEVEL_TRIG	33	// in % , values available 33 - 66 (0 and exlucded logically)

// flood sensor
#define FLOODSENSOR  			8			// Analog input where flood sensor is connected
#define FLOODSENSOR_ACTIVE		0			// 0 or 1 ,Set level when flood sensor is active (water detected)
#define MIN_FLOOR				0			// Minimum value for flood sensor trigger
#define MAX_FLOOR				1000		// Maximum value for flood sensor trigger



//----------------------------------------------------------------------
//!\brief	Structure where user parameters are stored in EEPROM (tuned by SMS)
//!\brief	DO NOT MODIFY THIS !
//----------------------------------------------------------------------
struct EEPROM_param {
	char smssecret[5];
	bool flag_data_written;				// when true, this structure contains data. Should be false only at the very first start
	bool flag_alarm_onoff;				// 1 = alarm geofencing on ; 0 = no geofencing alarm
	bool flag_periodic_status_onoff;	// 1 = send SMS periodic status ; 0 = no periodic status
	bool flag_alarm_low_bat;			// 1 = send SMS alarm when low voltage detected at input voltage (can be an external batt.) ; 0 = do not check input voltage
	bool flag_alarm_flood;				// 1 = send alarm when water detected ; 0 = don't care about water
	char myphonenumber[13];				// Default phone number where to send messages
	unsigned int radius;	
	double base_lat;
	char base_lat_dir;
	double base_lon;
	char base_lon_dir;
	unsigned int lipo_level_trig;		// battery level, when trigged, should send an alarm
	float trig_input_level;				// trigger alarm for low level input 
	float flood_sensor_trig;				// trigger alarm for low level input 
}MyParam;

//----------------------------------------------------------------------
//!\brief	Other structures used in programm
//----------------------------------------------------------------------
struct GPSPos {
	float latitude;
	char latitude_dir;
	float longitude;
	char longitude_dir;
	int hour;
	int minute;
	int second;
	int	num;
	int fix;
	}MyGPSPos;


struct Battery {
	unsigned int LiPo_level;
	unsigned int charging_status;
	}MyBattery;
	
struct AnalogInput {
	double raw;
	double analog_voltage;
	double input_voltage;
	}MyExternalSupply;	

struct DigitalSensor {
	unsigned int value;
	double raw_divider; 
	double analog_voltage_divider;
	double raw; 
	double analog_voltage;
	}MyFloodSensor;	
	
struct SMS {
	char message[256];
	char incomingnumber[13];
	int menupos;
	int menulevel;
	}MySMS;
	
struct FlagReg {
	bool taskGetGPS;	// flag to indicate when process to get GPS possition
	bool taskGetLiPo;	// flag to indicate that we have to get battery level and charging status
	bool taskGetAnalog;	// flag to indicate that we have to read analog input of external supply
	bool taskTestGeof;	// flag to indicate when process geofencing
	bool taskCheckSMS;	// flag to indicate when check SMS
	bool taskCheckFlood;// flag to indicate when check Flood sensor
	bool taskStatusSMS; // flat to indicate when it's time to send a periodic status SMS
	bool SMSReceived;	// flag to indicate that an SMS has been received
	bool fix3D;			// flag to indicate if fix is 3D (at least) or not
	bool PosOutiseArea;	// flag to indicate if fix is 3D (at least) or not
	bool taskCheckInputVoltage;	// flag to indicate when do an input voltage check
	bool taskCheckFW;	// flag to indicate when it's time to check if is FW hour check !
	bool ForceFWUpdate; // flag to indicate that a manual force update is asked by SMS		
	}MyFlag;

enum FixQuality {
	Invalid,	// 0
	GPS,		// 1
	DGPS,		// 2
	PPS,		// 3
	RTK,		// 4 Real Time Kinematic
	FloatRTK,	// 5
	DR,			// 6 Dead Reckoning
	Manual,		// 7
	Simulation,	// 8
	Error		// 9
	}GPSfix;

enum SMSMENU{
	SM_NOPE,		//0
	SM_LOGIN,		//1
	SM_MENU_MAIN,	//2
	SM_CHG_NUM,		//3
	SM_CHG_COORD,	//4
	SM_CHG_RADIUS,	//5
	SM_CHG_SECRET,	//6
	SM_RESTORE_DFLT, //7
	SM_CHG_LOWPOW_TRIG, 	//8
	SM_CHG_FLOODSENSORTRIG,	//9
	};

enum CMDSMS{
	CMD_EXIT,			//0
	CMD_STATUS,			//1
	CMD_ALM_ON,			//2
	CMD_ALM_OFF,		//3
	CMD_PARAMS,			//4
	CMD_CHG_NUM,		//5
	CMD_CHG_COORD,		//6
	CMD_CHG_RADIUS,		//7
	CMD_CHG_SECRET,		//8
	CMD_PERIODIC_STATUS_ON,	//9
	CMD_PERIODIC_STATUS_OFF,//10
	CMD_LOWPOWER_ON,	//11
	CMD_LOWPOWER_OFF,	//12
	CMD_CHG_LOWPOW_TRIG,//13
	CMD_CHG_FLOOD_TRIG,	//14
	CMD_UPDATE_FW,		//15
	CMD_RESTORE_DFLT	//16
	};	
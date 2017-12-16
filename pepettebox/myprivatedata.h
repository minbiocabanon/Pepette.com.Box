//--------------------------------------------------
//! \file		myprivatedata.h
//! \brief		header file for private data as GPS coordinate, phone number and wifi credentials
//! \date		2017-Dec
//! \author		minbiocabanon
//--------------------------------------------------

//----------------------------------------------------------------------
//!\brief	These are DEFAULTS parameters !
//----------------------------------------------------------------------

// Alarm allowed ?
#define FLAG_ALARM_ONOFF			1	// 1 = send SMS geofencing alarm 	; 0 = alarm not allowed (will not send SMS)
#define FLAG_PERIODIC_STATUS_ONOFF	1	// 1 = send periodic status  		; 0 = periodic status not allowed (will not send SMS)
#define FLAG_ALARM_LOW_BAT			1	// 1 = send alarm when low voltage, set TRIG_INPUT_LEVEL to define treshol 	; 0 = no check
#define FLAG_ALARM_FLOOD			1	// 1 = send alarm if flood sensor detects water ;  0 = don't care about flooding
#define AUTOTESTSMSFAIL				0	// 0 = by default autotest SMS has not failed
#define TRIG_INPUT_LEVEL			11.6	// in volt, when input voltage is lower than this value, an SMS alarm will be sent
											// 11.6V is a good level trig for 12V lead acid battery. Set lower voltage at your own risk !
											// 23.2V is a good level trig for 24V lead acid battery. Set lower voltage at your own risk !

//Params for geofencing
#define RADIUS					150			// radius in meter for geofencing centered in BASE_LAT,BASE_LON. When GPS pos is outside this radius -> Alarm !

// Params for flood sensor (analog)
#define FLOODSENSOR_TRIG		600.0	// Trigger value on raw data (ADC) on direct ready from flood sensor
										// Dry sensor : 0-10
										// Sensor touched with finger : 100-300
										// wet sensor : ~700-800


// Lat/Lon station position (for geofencing)
#define BASE_LAT	43.56457		
#define BASE_LAT_DIR	'N'	
#define BASE_LON	7.0751
#define BASE_LON_DIR	'E'

// Phone number to call or for SMS
#define MYPHONENUMBER	"+33000000000"		// Default phone number where to send messages
#define SIMPHONENUMBER	"+33123456789"		// phone number of SIM card inserted in the linkitone

// SMS Menu
#define SMSSECRET	"1234"					// Default secret code to activate menu

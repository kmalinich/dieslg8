#include <mcp_can.h>
#include <SPI.h>

#include <SD.h>

#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>



// Config: unit limits for gauges
const int boost_psi_max = 40;
const int coolant_c_max = 150;

// Config: Various value targets
const int coolant_c_target = 75;

// Config: disable/enable SD card logging
bool logging_enable = 0;

// Config: disable/enable automatic code clearing
bool code_clear_all_enable      = 0;
bool code_clear_specific_enable = 0;

// Config: disable/enable turn signal LED flash when coolant temp reaches target
bool temp_flash_enable = 0;

// Config: disable/enable gauge sweep
bool gauge_sweep_enable = 1;

// Config: disable/enable gauge hijacking
bool hijack_fuel_boost_enable   = 0;
bool hijack_fuel_coolant_enable = 0;
bool hijack_oil_boost_enable    = 0;
bool hijack_oil_coolant_enable  = 1;

// Config: step limits for gauges
const int steps_max_large = 4667; // Large gauges (speedo, tach)
const int steps_max_small = 1800; // Small gauges (fuel %, oil)


// 1 psi = 68.9475729318 hPa
// 1 hPA = 0.0145037738 psi
const float hpa2psi = 68.9475729318;
const float psi2hpa = 0.0145037738;

// Ignition bitmask values to match against
const int mask_ignition_acc = 0xC1; // buf[0]
const int mask_ignition_run = 0xC5; // buf[0]
const int mask_ignition_sta = 0xD5; // buf[0]


// Status variables
unsigned long loop_count_01 = 0;
unsigned long loop_count_02 = 0;

// 0 = Ambient        + Boost actual
// 1 = Coolant temp   + Boost target
// 2 = Pedal position + Engine RPM
unsigned int data_expected = 0;

// Integer status values
unsigned int ambient_hpa;
unsigned int engine_rpm;
unsigned int boost_hpa_actual;
unsigned int boost_hpa_target;

// Float status values
float boost_psi_actual;
float boost_psi_target;
float coolant_temp_c;
float throttle_percent;

// Boolean status values
bool temp_flashed = 0;

bool hijack_fuel_active = 0;
bool hijack_oil_active  = 0;

bool ignition_off = 1;
bool ignition_acc = 0;
bool ignition_run = 0;
bool ignition_sta = 0;

bool sweep_done = 0;



// SPI pin config
const int SPI_CS_CAN = 9;
const int SPI_CS_SD  = 4;

// Declare CAN handle
MCP_CAN CAN(SPI_CS_CAN);

// Declare File handle for logging
File log_file;

// Declare GPS handle
NMEAGPS gps;
gps_fix currentFix;





// Send CAN message
void can_send(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
	digitalWrite(LED_BUILTIN, LOW);

	// Serial.print("[dieslg8][CAN ][SEND] 0x");
	// Serial.print(address, HEX);
	// Serial.print(" => ");
	// Serial.print(a, HEX); Serial.print(" ");
	// Serial.print(b, HEX); Serial.print(" ");
	// Serial.print(c, HEX); Serial.print(" ");
	// Serial.print(d, HEX); Serial.print(" ");
	// Serial.print(e, HEX); Serial.print(" ");
	// Serial.print(f, HEX); Serial.print(" ");
	// Serial.print(g, HEX); Serial.print(" ");
	// Serial.print(h, HEX); Serial.print(" ");
	// Serial.println();

	unsigned char DataToSend[8] = {a, b, c, d, e, f, g, h};

	delay(25);
	CAN.sendMsgBuf(address, 0, 8, DataToSend);

	LED_GPS_status();
}

// Clears these codes:
// 3FF1 : Mass air flow sensor
// 40D4 : EGR actuator position control
// 4596 : Smooth running controller cylinder 3
// 485C : EGR cooler bypass valve control
// 4A1E : Glow plug activation, cylinder 6
// 4A24 : Glow plug activation, cylinder 5
// 4A2E : Glow plug activation, cylinder 5
// 4B39 : EGR actuator control
// 4B73 : EGR engine exhaust heating control
// 4CAE : EGR position sensor plausibility
void code_clear() {
	// Return if disabled
	if (code_clear_specific_enable == 0) return;

	Serial.println("[dieslg8][CAN ][FUNC][DTC ][CLR ] Specific");

	can_send(0x6F1, 0x12, 0x04, 0x18, 0x02, 0xFF, 0xFF, 0x00, 0x00); delay(75);

	turn_set(0x01);
	can_send(0x612, 0xF1, 0x10, 0x1A, 0x58, 0x08, 0x4C, 0xAE, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x23, 0x3F, 0xF1, 0x21, 0x4B, 0x39, 0x21); delay(75);
	can_send(0x612, 0xF1, 0x24, 0x4A, 0x24, 0xE1, 0xFF, 0xFF, 0xFF); delay(75);

	turn_set(0x02);
	can_send(0x612, 0xF1, 0x21, 0x48, 0x5C, 0xE1, 0x4B, 0x73, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x03, 0x14, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x7F, 0x14, 0x78, 0xFF, 0xFF, 0xFF); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x54, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF); delay(75);

	turn_reset();

	delay(75);

	code_clear_all();
}

// Clear all DDE codes
void code_clear_all() {
	// Return if disabled
	if (code_clear_all_enable == 0) return;

	Serial.println("[dieslg8][CAN ][FUNC][DTC ][CLR ] All");

	turn_set(0x01);

	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x02); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x01); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x02); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x01); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);

	turn_reset();
}

void temp_flash() {
	// Return if disabled
	if (temp_flash_enable == 0) return;

	// Return if already flashed
	if (temp_flashed == 1) return;

	Serial.println("[dieslg8][CAN ][FUNC][DDE ][COOL] Turn LED flash");

	turn_set(0x01); delay(111);
	turn_set(0x03); delay(111);
	turn_set(0x02); delay(111);

	turn_set(0x01); delay(111);
	turn_set(0x03); delay(111);
	turn_set(0x02); delay(111);

	turn_set(0x01); delay(111);
	turn_set(0x03); delay(111);
	turn_set(0x02); delay(111);

	turn_reset();

	temp_flashed = 1;
}

// Illuminate turn signal LED(s) in the cluster
// led_mask:
// 0x01 : Left
// 0x02 : Right
// 0x03 : Both
void turn_set(unsigned int led_mask) {
	Serial.println("[dieslg8][CAN ][FUNC][KOMB][TURN] Set");

	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, led_mask, 0x04, 0x00);
}

// Reset turn signal LED(s) in the cluster
void turn_reset() {
	Serial.println("[dieslg8][CAN ][FUNC][KOMB][TURN] Reset");

	can_send(0x6F1, 0x60, 0x03, 0x30, 0x2B, 0x00, 0x00, 0x00, 0x00);
}


// Illuminate active cruise LED(s) in the cluster
// led_mask_1:
// 0x01 :
// 0x02 :
// 0x03 :
void acc_set() {
	Serial.println("[dieslg8][CAN ][FUNC][KOMB][ACC ] Set");

	can_send(0x6F1, 0x60, 0x03, 0x70, 0x27, 0x6A, 0x00, 0x00, 0x00);
}

// Reset active cruise LED(s) in the cluster
void acc_reset() {
	Serial.println("[dieslg8][CAN ][FUNC][KOMB][ACC ] Reset");

	can_send(0x6F1, 0x60, 0x03, 0x70, 0x2B, 0x6E, 0x00, 0x00, 0x00);
}


void gauge_sweep() {
	// Return if disabled
	if (gauge_sweep_enable == 0) return;

	Serial.println("[dieslg8][CAN ][FUNC][KOMB][SWP ] Performing");

	// Speedo/tach to steps_max_large
	hijack_gauge(0x20, steps_max_large);
	hijack_gauge(0x21, steps_max_large);

	// Fuel/oil to steps_max_small
	hijack_fuel(steps_max_small);
	hijack_oil(steps_max_small);

	delay(1000);

	// Reset gauges
	reset_gauge(0x20);
	reset_gauge(0x21);
	reset_gauge(0x22);
	reset_gauge(0x23);
}


void hijack_fuel_boost() {
	// Return if disabled
	if (hijack_fuel_boost_enable == 0) return;

	// Return if less than 10 hPa
	if (boost_hpa_actual < 10 || throttle_percent < 49) {
		reset_fuel();
		return;
	}

	unsigned int steps = boost_hpa_actual * (steps_max_small / (boost_psi_max * hpa2psi));
	hijack_fuel(steps);
}

void hijack_fuel_coolant() {
	// Return if disabled
	if (hijack_fuel_coolant_enable == 0) return;

	// Return if less than 10 hPa
	if (coolant_temp_c < 0 || coolant_temp_c > coolant_c_max) {
		return;
	}

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	hijack_fuel(steps);
}


void hijack_oil_boost() {
	// Return if disabled
	if (hijack_oil_boost_enable == 0) return;

	// Return if less than 10 hPa
	if (boost_hpa_target < 10) {
		reset_oil();
		return;
	}

	unsigned int steps = boost_hpa_target * (steps_max_small / (boost_psi_max * hpa2psi));
	hijack_oil(steps);
}

void hijack_oil_coolant() {
	// Return if disabled
	if (hijack_oil_coolant_enable == 0) return;

	// Return if less than 10 hPa
	if (coolant_temp_c < 0 || coolant_temp_c > coolant_c_max) {
		return;
	}

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	hijack_oil(steps);
}


void hijack_fuel(unsigned int steps) {
	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		reset_fuel();
		return;
	}

	hijack_fuel_active = 1;
	hijack_gauge(0x22, steps);
}

void hijack_oil(unsigned int steps) {
	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		reset_oil();
		return;
	}

	hijack_oil_active = 1;
	hijack_gauge(0x23, steps);
}


void reset_fuel() {
	// Return if hijack is inactive
	if (hijack_fuel_active == 0) return;

	reset_gauge(0x22);
	hijack_fuel_active = 0;
}

void reset_oil() {
	// Return if hijack is inactive
	if (hijack_oil_active == 0) return;

	reset_gauge(0x23);
	hijack_oil_active = 0;
}


// Hijack a gauge
void hijack_gauge(int gauge_id, int steps) {
	uint8_t msb = (steps / 256); // MSB
	uint8_t lsb = (steps % 256); // LSB

	can_send(0x6F1, 0x60, 0x05, 0x30, gauge_id, 0x06, msb, lsb, 0xFF);
}


// Un-hijack a gauge
void reset_gauge(unsigned int gauge_id) {
	// Gauge IDs
	// 0x20 = Speedometer
	// 0x21 = Tachometer
	// 0x22 = Fuel
	// 0x23 = Oil

	can_send(0x6F1, 0x60, 0x03, 0x30, gauge_id, 0x00, 0xFF, 0xFF, 0xFF);
}


// Get data
// 0x01, 0xF4 : [SPLAD]  Boost pressure, target     hPA  x*0.091554
// 0x07, 0x6D : [IPLAD]  Boost pressure, actual     hPa  x*0.091554
// 0x05, 0x47 : [ITKUM]  Coolant temp               C    (x/100)-100
// 0x0C, 0x1C : [IPUMG]  Ambient pressure           hPa  x*0.030518
// 0x01, 0x62 : [IFPWG]  Pedal position (filtered)  %    x*0.012207
// 0x18, 0x81 : [INMOT]  Engine RPM, actual         %    x*0.5
//
// 0x05, 0x79 : [SMIBA]  Limiter, internal torque   Nm   (x/10)
// 0x05, 0x7B : [IMBEG]  Setpoint, internal torque  Nm   (x*0.114443)-2500
// 0x07, 0xD1 : [IMOAK]  Torque, actual             Nm   (x*0.114443)-2500
//
// 0x0F, 0xD2 : [ITUMG]  Ambient temp                       C  (x/10)-273.14
// 0x07, 0x6F : [ITLAL]  Intake air temp, post intercooler  C  (x/100)-100
// 0x0A, 0xF1 : [ITMOT]  Engine temp                        C  (x/10)-273.14
// 0x0A, 0x8C : [ITOEL]  Oil temp                           C  (x/100)-100
void status_messwertblock_lesen() {
	// 0 = Ambient pressure + Boost pressure actual
	// 1 = Coolant temp     + Boost pressure target
	// 2 = Pedal position   + Engine RPM
	if (data_expected == 0) {
		data_expected = 1;
		// SPLAD + ITKUM
		can_send(0x6F1, 0x12, 0x06, 0x2C, 0x10, 0x01, 0xF4, 0x05, 0x47);
	}
	else if (data_expected == 1) {
		data_expected = 2;
		// IFPWG + INMOT
		can_send(0x6F1, 0x12, 0x06, 0x2C, 0x10, 0x01, 0x62, 0x18, 0x81);
	}
	else if (data_expected == 2) {
		data_expected = 0;
		// IPUMG + IPLAD
		can_send(0x6F1, 0x12, 0x06, 0x2C, 0x10, 0x0C, 0x1C, 0x07, 0x6D);
	}
}


void sdcard_log() {
	// Return if disabled
	if (logging_enable == 0) return;

	log_file = SD.open("logfile.csv", FILE_WRITE);

	log_file.print(ignition_off);     log_file.print(",");
	log_file.print(ignition_acc);     log_file.print(",");
	log_file.print(ignition_run);     log_file.print(",");
	log_file.print(ignition_sta);     log_file.print(",");
	log_file.print(engine_rpm);       log_file.print(",");
	log_file.print(throttle_percent); log_file.print(",");
	log_file.print(coolant_temp_c);   log_file.print(",");
	log_file.print(boost_psi_target); log_file.print(",");

	log_file.println(boost_psi_actual);

	log_file.close();

	// Serial.println("[dieslg8][SD  ][FUNC][WRIT][LINE] Done");
}


void LED_GPS_status() {
	if (currentFix.valid.altitude && currentFix.valid.location && currentFix.valid.speed) {
		digitalWrite(LED_BUILTIN, HIGH);
	}
}

void setup() {
	// Initialize digital pin LED_BUILTIN as an output
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize serial output for logging
	Serial.begin(115200);

	// Wait for serial port to come up before proceeding
	// while (!Serial) ;

	// Init CAN, baudrate 500k
	while (CAN.begin(CAN_500KBPS) != CAN_OK) {
		Serial.println("[dieslg8][INIT][CAN ] Failure");
		delay(1000);
	}

	Serial.println("[dieslg8][INIT][CAN ] OK");

	// Initialize SD card logging
	if (logging_enable == 1) {
		if (!SD.begin(SPI_CS_SD)) {
			Serial.println("[dieslg8][INIT][SD  ] Failure");
			delay(1000);
		}

		Serial.println("[dieslg8][INIT][SD  ] OK");
	}

	gpsPort.begin(9600);

	Serial.println("[dieslg8][INIT][MAIN] Ready");
}

void loop() {
	// Increment loop counters
	loop_count_01++;
	loop_count_02++;

	// Request status data every 1900 loops
	if (loop_count_01 == 1900) {
		loop_count_01 = 0;
		if (ignition_run == 1) status_messwertblock_lesen();
	}

	if (loop_count_02 == 10000000) {
		loop_count_02 = 0;
	}

	while (gps.available(gpsPort)) {
		currentFix = gps.read();

		Serial.print("[dieslg8][GPS ][STAT] ");
		Serial.print(gps.sat_count);
		Serial.print(" sats");

		if (currentFix.valid.location) {
			Serial.print(", Loc: ");
			Serial.print(currentFix.latitude(), 6);
			Serial.print(',');
			Serial.print(currentFix.longitude(), 6);
		}

		if (currentFix.valid.altitude) {
			Serial.print(", Alt: ");
			Serial.print(currentFix.altitude_ft());
			Serial.print(" ft");
		}

		if (currentFix.valid.speed) {
			Serial.print(" :: ");
			Serial.print(currentFix.speed_mph());
			Serial.print(" mph");
		}

		LED_GPS_status();

		// date
		// time
		// heading

		Serial.println();
	}


	// Check if incoming data is available
	if (CAN_MSGAVAIL == CAN.checkReceive()) {
		bool print_msg = 1;

		// Read CAN message data
		// len : data length
		// buf : data buffer
		unsigned char len = 0;
		unsigned char buf[8];

		CAN.readMsgBuf(&len, buf);

		unsigned long arbid = CAN.getCanId();

		switch (arbid) {
			case 0x130 : // Ignition status
				print_msg = 0;

				// Serial.println("[dieslg8][CAN ][PARS] Ignition status received");

				// Test if all bits in mask_ignition_sta are present in buf[0]
				if ((buf[0] & mask_ignition_sta) == mask_ignition_sta) {
					if (ignition_sta != 1) {
						Serial.println("[dieslg8][IGN ][STA ] Active");
					}

					if (ignition_run != 1) {
						Serial.println("[dieslg8][IGN ][RUN ] Active");
						gauge_sweep();
					}

					ignition_off = 0;
					ignition_acc = 0;
					ignition_run = 1;
					ignition_sta = 1;
					break;
				}

				// Test if all bits in mask_ignition_run are present in buf[0]
				if ((buf[0] & mask_ignition_run) == mask_ignition_run) {
					if (ignition_run != 1) {
						Serial.println("[dieslg8][IGN ][RUN ] Active");
						gauge_sweep();
					}

					ignition_off = 0;
					ignition_acc = 0;
					ignition_run = 1;
					ignition_sta = 0;
					break;
				}

				// Test if all bits in mask_ignition_acc are present in buf[0]
				if ((buf[0] & mask_ignition_acc) == mask_ignition_acc) {
					if (ignition_acc != 1) {
						Serial.println("[dieslg8][IGN ][ACC ] Active");
						reset_fuel();
						reset_oil();
					}

					ignition_off = 0;
					ignition_acc = 1;
					ignition_run = 0;
					ignition_sta = 0;
					break;
				}

				// By this point, ignition must be off
				if (ignition_off != 1) {
					Serial.println("[dieslg8][IGN ][OFF ] Active");
					reset_fuel();
					reset_oil();
				}

				temp_flashed = 0;

				ignition_off = 1;
				ignition_acc = 0;
				ignition_run = 0;
				ignition_sta = 0;

				break;

			case 0x612 : // Response to STATUS_MESSWERTE_BLOCK request
				print_msg = 0;

				// TODO: Why the f**k doesnt the case statement work here
				if (data_expected == 0) { // 0 = Ambient + Boost actual
					unsigned int value_01 = (buf[4] << 8) | buf[5];
					unsigned int value_02 = (buf[6] << 8) | buf[7];

					ambient_hpa      = value_01 * 0.030518;
					boost_hpa_actual = (value_02 * 0.091554) - ambient_hpa;
					boost_psi_actual = boost_hpa_actual / hpa2psi;

					// Serial.print("[dieslg8][DATA][AMBh] "); Serial.println(ambient_hpa);
					// Serial.print("[dieslg8][DATA][BSTa] "); Serial.println(boost_psi_actual);

					hijack_fuel_boost();
				}
				else if (data_expected == 1) { // 1 = Coolant temp + Boost target
					unsigned int value_01 = (buf[4] << 8) | buf[5];
					unsigned int value_02 = (buf[6] << 8) | buf[7];

					boost_hpa_target = (value_01 * 0.091554) - ambient_hpa;
					boost_psi_target = boost_hpa_target / hpa2psi;

					coolant_temp_c = (value_02 * 0.01) - 100;

					if (coolant_temp_c > coolant_c_target) temp_flash();

					// Serial.print("[dieslg8][DATA][BSTt] "); Serial.println(boost_psi_target);
					// Serial.print("[dieslg8][DATA][CLTc] "); Serial.println(coolant_temp_c);

					hijack_oil_boost();
					hijack_oil_coolant();
				}
				else if (data_expected == 2) { // 2 = Pedal position + Engine RPM
					unsigned int value_01 = (buf[4] << 8) | buf[5]; // Pedal
					unsigned int value_02 = (buf[6] << 8) | buf[7]; // RPM

					throttle_percent = value_01 * 0.012207;
					engine_rpm       = value_02 * 0.5;

					// Serial.print("[dieslg8][DATA][THRT] "); Serial.println(throttle_percent);
					// Serial.print("[dieslg8][DATA][RPM ] "); Serial.println(engine_rpm);

					hijack_fuel_boost();
				}

				break;

			case 0x660 : // ACK to DIA message
				print_msg = 0;
		}

		if (print_msg == 1) {
			// Serial.println("----------------------------------------");
			Serial.print("[dieslg8][CAN ][RECV] 0x");
			Serial.print(arbid, HEX);
			Serial.print(" => ");

			// Print the data
			for (int i = 0; i < len; i++) {
				Serial.print(buf[i], HEX);
				Serial.print(" ");
			}

			Serial.println();
		}
	}
}


// vim: set syntax=cpp filetype=cpp ts=2 sw=2 tw=0 et :

// Enable SD card performance data logging by defining logging_perf_enable
// #define logging_perf_enable true

// External libraries
#include <SPI.h>
#include "mcp_can.h"
#include "mcp2515_can.h"

#ifdef logging_perf_enable
#include <SD.h>
#endif

// Debug mode
#define DBG false

// Debug macro to print messages to serial
#define DEBUG(x)     if (DBG && Serial) { Serial.print(x);     }
#define DEBUGLN(x)   if (DBG && Serial) { Serial.println(x);   }
#define DEBUG2(x, y) if (DBG && Serial) { Serial.print(x, y);  }


// Config: Control active cruise LEDs
const bool acc_led_enable = 0;

// Config: disable/enable automatic code clearing
const bool code_clear_all_enable       = 0;
const bool code_clear_specific_enable  = 0;

// Config: disable/enable all turn signal LED hijacking
const bool hijack_turn_enable = 1;

// Config: disable/enable turn signal LED flash when coolant temp reaches target
// hijack_turn_enable must also be set to 1
const bool temp_flash_enable = 1;

// Config: disable/enable gauge sweep
const bool gauge_sweep_enable = 1;

// Config: disable/enable fuel gauge hijacking
const bool hijack_fuel_boost_enable   = 1;
const bool hijack_fuel_coolant_enable = 0;

// Config: disable/enable oil gauge hijacking
const bool hijack_oil_boost_enable = 0;
const bool hijack_oil_coolant_enable = 1;


// Config: unit limits for gauges
const int boost_psi_max = 40;
const int coolant_c_max = 150;

// Config: Various value targets
const int coolant_c_target_hi = 76;
const int coolant_c_target_lo = 74;


// Config: step limits for gauges
const int steps_max_large = 4667; // Large gauges (speedo, tach)
const int steps_max_small = 1800; // Small gauges (fuel %, oil)

// 1 psi = 68.9475729318 hPa
// 1 hPA = 0.0145037738 psi
const float hpa2psi = 68.9475729318;
const float psi2hpa = 0.0145037738;

// Config: Pins
const int CAN_INT_PIN = 2;
const int SPI_CS_CAN  = 9;
const int SPI_CS_SD   = 4;


// Ignition bitmask values to match against
const int mask_ignition_acc = 0xC1; // buf[0]
const int mask_ignition_run = 0xC5; // buf[0]
const int mask_ignition_str = 0xD5; // buf[0]



// Status variables
unsigned long loop_count_01 = 0;
unsigned long loop_count_02 = 0;

// 0 = Ambient        + Boost actual
// 1 = Coolant temp   + Boost target
// 2 = Pedal position + Engine RPM
unsigned int data_expected = 0;

// Integer status values
unsigned int engine_rpm;
unsigned int throttle_percent;
int coolant_temp_c;


// Float status values
float ambient_hpa;
float ambient_psi;

float boost_hpa_actual;
float boost_hpa_target;

float boost_psi_actual;
float boost_psi_target;



// Boolean status values
bool temp_flashed = 0;
bool acc_led_on   = 0;

bool hijack_fuel_active = 0;
bool hijack_oil_active  = 0;


bool ignition_off = 1;
bool ignition_acc = 0;
bool ignition_run = 0;
bool ignition_str = 0;


// Set CS pin
mcp2515_can CAN(SPI_CS_CAN);

// Declare File handles for logging
#ifdef logging_perf_enable
File log_file_perf;
#endif



// Send CAN message
void can_send(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
	// DEBUG("[dieslg8][CAN ][SEND] 0x");
	// DEBUG2(address, HEX);
	// DEBUG(" => ");
	// DEBUG2(a, HEX); DEBUG(" ");
	// DEBUG2(b, HEX); DEBUG(" ");
	// DEBUG2(c, HEX); DEBUG(" ");
	// DEBUG2(d, HEX); DEBUG(" ");
	// DEBUG2(e, HEX); DEBUG(" ");
	// DEBUG2(f, HEX); DEBUG(" ");
	// DEBUG2(g, HEX); DEBUG(" ");
	// DEBUG2(h, HEX); DEBUG(" ");
	// DEBUGLN();

	unsigned char DataToSend[8] = {a, b, c, d, e, f, g, h};

	delay(25);
	CAN.sendMsgBuf(address, 0, 8, DataToSend);
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
void code_clear_specific() {
	if (code_clear_specific_enable != 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][DTC ][CLR ] Specific");

	can_send(0x6F1, 0x12, 0x04, 0x18, 0x02, 0xFF, 0xFF, 0x00, 0x00); delay(75);

	hijack_turn(0x01);
	can_send(0x612, 0xF1, 0x10, 0x1A, 0x58, 0x08, 0x4C, 0xAE, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x23, 0x3F, 0xF1, 0x21, 0x4B, 0x39, 0x21); delay(75);
	can_send(0x612, 0xF1, 0x24, 0x4A, 0x24, 0xE1, 0xFF, 0xFF, 0xFF); delay(75);

	hijack_turn(0x02);
	can_send(0x612, 0xF1, 0x21, 0x48, 0x5C, 0xE1, 0x4B, 0x73, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x03, 0x14, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x7F, 0x14, 0x78, 0xFF, 0xFF, 0xFF); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x54, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF); delay(75);

	reset_turn();
}

// Clear all DDE codes
void code_clear_all() {
	if (code_clear_all_enable != 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][DTC ][CLR ] All");

	// hijack_turn(0x01);

	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); hijack_turn(0x02); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); hijack_turn(0x01); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); hijack_turn(0x02); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); hijack_turn(0x01); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);

	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);

	// reset_turn();
}


void temp_flash() {
	if (temp_flash_enable != 1) return;

	// Return if already flashed
	if (temp_flashed == 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][COOL] Turn LED flash");

	hijack_turn(0x01); delay(111);
	hijack_turn(0x03); delay(111);
	hijack_turn(0x02); delay(111);

	hijack_turn(0x01); delay(111);
	hijack_turn(0x03); delay(111);
	hijack_turn(0x02); delay(111);

	hijack_turn(0x01); delay(111);
	hijack_turn(0x03); delay(111);
	hijack_turn(0x02); delay(111);

	reset_turn();

	temp_flashed = 1;
}

// Illuminate turn signal LED(s) in the cluster
// led_mask:
// 0x01 : Left
// 0x02 : Right
// 0x03 : Both
void hijack_turn(unsigned int led_mask) {
	if (hijack_turn_enable != 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][TURN] Set");

	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, led_mask, 0x04, 0x00);
}

// Reset turn signal LED(s) in the cluster
void reset_turn() {
	if (hijack_turn_enable != 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][TURN] Reset");

	can_send(0x6F1, 0x60, 0x03, 0x30, 0x2B, 0x00, 0x00, 0x00, 0x00);
}


// Illuminate active cruise LED(s) in the cluster
// led_mask_1:
// 0x01 :
// 0x02 :
// 0x03 :
void hijack_acc() {
	if (acc_led_enable != 1) return;

	// Return if already on
	if (acc_led_on == 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][COOL] Active cruise LEDs set");

	can_send(0x6F1, 0x60, 0x03, 0x70, 0x27, 0x6A, 0x00, 0x00, 0x00);

	// Mark as on
	acc_led_on = 1;
}

// Reset active cruise LED(s) in the cluster
void reset_acc() {
	if (acc_led_enable != 1) return;

	// Return if already off
	if (acc_led_on == 0) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][COOL] Active cruise LEDs reset");

	can_send(0x6F1, 0x60, 0x03, 0x70, 0x2B, 0x6E, 0x00, 0x00, 0x00);

	// Mark as off
	acc_led_on = 0;
}


void gauge_sweep() {
	if (gauge_sweep_enable != 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][SWP ] Performing");

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

	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(A3, LOW);
}


void hijack_fuel_boost() {
	if (hijack_fuel_boost_enable != 1) return;

	// Return if less than 10 hPa, throttle under 49%, or engine RPM under 1000
	if (boost_hpa_actual < 10 || throttle_percent < 49 || engine_rpm < 1000) {
		reset_fuel();
		return;
	}

	// DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][FUEL] Boost");

	unsigned int steps = boost_hpa_actual * (steps_max_small / (boost_psi_max * hpa2psi));
	hijack_fuel(steps);
}

void hijack_fuel_coolant() {
	if (hijack_fuel_coolant_enable != 1) return;

	// Return if coolant_temp_c is under 0 or above max, or engine RPM is under 400
	if (coolant_temp_c < 0 || coolant_temp_c > coolant_c_max || engine_rpm < 400) {
		return;
	}

	// DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][FUEL] Coolant");

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	hijack_fuel(steps);
}

void hijack_fuel(unsigned int steps) {
	if (hijack_fuel_boost_enable != 1 && hijack_fuel_coolant_enable != 1) return;

	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		reset_fuel();
		return;
	}

	digitalWrite(A3, HIGH);

	hijack_fuel_active = 1;
	hijack_gauge(0x22, steps);
}

void reset_fuel() {
	if (hijack_fuel_boost_enable != 1 && hijack_fuel_coolant_enable != 1) return;

	// Return if hijack is inactive
	if (hijack_fuel_active == 0) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][FUEL] Reset");

	digitalWrite(A3, LOW);

	reset_gauge(0x22);
	hijack_fuel_active = 0;
}


void hijack_oil_boost() {
	if (hijack_oil_boost_enable != 1) return;

	// Return if less than 10 hPa, throttle under 49%, or engine RPM under 1000
	if (boost_hpa_target < 10 || throttle_percent < 49 || engine_rpm < 1000) {
		reset_oil();
		return;
	}

	// DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][OIL ] Boost");

	unsigned int steps = boost_hpa_target * (steps_max_small / (boost_psi_max * hpa2psi));
	hijack_oil(steps);
}

void hijack_oil_coolant() {
	if (hijack_oil_coolant_enable != 1) return;

	// Return if coolant_temp_c is under 0 or above max, or engine RPM is under 400
	if (coolant_temp_c < 0 || coolant_temp_c > coolant_c_max || engine_rpm < 400) {
		return;
	}

	DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][OIL ] Coolant");

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	hijack_oil(steps);
}

void hijack_oil(unsigned int steps) {
	if (hijack_oil_boost_enable != 1 && hijack_oil_coolant_enable != 1) return;

	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		reset_oil();
		return;
	}

	digitalWrite(LED_BUILTIN, HIGH);

	hijack_oil_active = 1;
	hijack_gauge(0x23, steps);
}

void reset_oil() {
	if (hijack_oil_boost_enable != 1 && hijack_oil_coolant_enable != 1) return;

	// Return if hijack is inactive
	if (hijack_oil_active == 0) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][OIL ] Reset");

	digitalWrite(LED_BUILTIN, LOW);

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
	// 0x20 = Speedomter
	// 0x21 = Tachometer
	// 0x22 = Fuel
	// 0x23 = Oil

	can_send(0x6F1, 0x60, 0x03, 0x30, gauge_id, 0x00, 0xFF, 0xFF, 0xFF);
}


// Get data
// 0x01, 0x62 : [IFPWG]  Pedal position (filtered)  %    x*0.012207
// 0x01, 0xF4 : [SPLAD]  Boost pressure, target     hPA  x*0.091554
// 0x05, 0x47 : [ITKUM]  Coolant temp               C    (x/100)-100
// 0x07, 0x6D : [IPLAD]  Boost pressure, actual     hPa  x*0.091554
// 0x0C, 0x1C : [IPUMG]  Ambient pressure           hPa  x*0.030518
// 0x18, 0x81 : [INMOT]  Engine RPM, actual         %    x*0.5
//
// 0x05, 0x79 : [SMIBA]  Limiter, internal torque   Nm   (x/10)
// 0x05, 0x7B : [IMBEG]  Setpoint, internal torque  Nm   (x*0.114443)-2500
// 0x07, 0xD1 : [IMOAK]  Torque, actual             Nm   (x*0.114443)-2500
//
// 0x07, 0x6F : [ITLAL]  Intake air temp, post intercooler  C  (x/100)-100
// 0x0A, 0x8C : [ITOEL]  Oil temp                           C  (x/100)-100
// 0x0A, 0xF1 : [ITMOT]  Engine temp                        C  (x/10)-273.14
// 0x0F, 0xD2 : [ITUMG]  Ambient temp                       C  (x/10)-273.14
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


#ifdef logging_perf_enable
void sdcard_log_perf() {
	// Toggle LED
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

	log_file_perf = SD.open("logfile_perf.csv", FILE_WRITE);

	log_file_perf.print(ignition_off);     log_file_perf.print(",");
	log_file_perf.print(ignition_acc);     log_file_perf.print(",");
	log_file_perf.print(ignition_run);     log_file_perf.print(",");
	log_file_perf.print(ignition_str);     log_file_perf.print(",");
	log_file_perf.print(engine_rpm);       log_file_perf.print(",");
	log_file_perf.print(throttle_percent); log_file_perf.print(",");
	log_file_perf.print(coolant_temp_c);   log_file_perf.print(",");
	log_file_perf.print(boost_psi_target); log_file_perf.print(",");

	log_file_perf.println(boost_psi_actual);

	log_file_perf.close();

	// DEBUGLN("[dieslg8][SD  ][FUNC][WRIT][PERF] Done");

	// Toggle LED
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
#endif


void setup() {
	// Initialize digital pin LED_BUILTIN as an output
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(A3, OUTPUT);

	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(A3, LOW);

	// Wait for serial connection when debugging
	if (DBG) {
		unsigned int serial_counter = 0;

		// Initialize serial output for logging
		Serial.begin(115200);
		while (!Serial) {
			if (serial_counter > 50) break;
			serial_counter++;

			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			digitalWrite(A3, !digitalRead(A3));
			delay(100);
		}

		DEBUG("[dieslg8][INIT][SRL ] serial_counter: "); DEBUGLN(serial_counter);
	}

	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(A3, LOW);

	// Init CAN, baudrate 500k
	while (CAN.begin(CAN_500KBPS) != CAN_OK) {
		DEBUGLN("[dieslg8][INIT][CAN ] Waiting");
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		digitalWrite(A3, !digitalRead(A3));
		delay(100);
	}

	DEBUGLN("[dieslg8][INIT][CAN ] OK");

	// There are 2 mask in MCP2515, you need to set both of them
	CAN.init_Mask(0, 0, 0x3FF);
	CAN.init_Mask(1, 0, 0x3FF);

	// There are 6 filters in MCP2515
	CAN.init_Filt(0, 0, 0x130);
	CAN.init_Filt(1, 0, 0x612);
	// CAN.init_Filt(2, 0, 0x660);

	// Initialize SD card logging
#ifdef logging_perf_enable
	if (!SD.begin(SPI_CS_SD)) {
		DEBUGLN("[dieslg8][INIT][SD  ] Failure");
		delay(1000);
	}

	DEBUGLN("[dieslg8][INIT][SD  ] OK");
#endif

	DEBUGLN("[dieslg8][INIT][MAIN] Ready");
	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(A3, LOW);
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

		if (ignition_run == 1) {
			code_clear_all();
			code_clear_specific();
		}
	}

	// Check if incoming data is available
	if (CAN_MSGAVAIL == CAN.checkReceive()) {
		bool print_msg = 1;

		// Read CAN message data
		// len : data length
		// buf : data buffer
		unsigned char len = 0;
		unsigned char buf[8];

		// Read CAN message data
		// len : data length
		// buf : data buffer
		CAN.readMsgBuf(&len, buf);

		unsigned long arbid = CAN.getCanId();

		switch (arbid) {
			case 0x130 : { // Ignition status
				print_msg = 0;

				// DEBUGLN("[dieslg8][CAN ][PARS] Ignition status received");

				// Test if all bits in mask_ignition_str are present in buf[0]
				if ((buf[0] & mask_ignition_str) == mask_ignition_str) {
					if (ignition_str != 1) {
						DEBUGLN("[dieslg8][CAN ][IGN ][STA ] Active");

						code_clear_all();
						code_clear_specific();
					}

					if (ignition_run != 1) {
						DEBUGLN("[dieslg8][CAN ][IGN ][RUN ] Active");

						gauge_sweep();

						code_clear_all();
						code_clear_specific();
					}

					ignition_off = 0;
					ignition_acc = 0;
					ignition_run = 1;
					ignition_str = 1;

					break;
				}

				// Test if all bits in mask_ignition_run are present in buf[0]
				if ((buf[0] & mask_ignition_run) == mask_ignition_run) {
					if (ignition_run != 1) {
						DEBUGLN("[dieslg8][CAN ][IGN ][RUN ] Active");

						gauge_sweep();

						code_clear_all();
						code_clear_specific();
					}

					ignition_off = 0;
					ignition_acc = 0;
					ignition_run = 1;
					ignition_str = 0;

					break;
				}

				// Test if all bits in mask_ignition_acc are present in buf[0]
				if ((buf[0] & mask_ignition_acc) == mask_ignition_acc) {
					if (ignition_acc != 1) {
						DEBUGLN("[dieslg8][CAN ][IGN ][ACC ] Active");

						reset_fuel();
						reset_oil();

						reset_acc();
						temp_flashed = 0;
					}

					ignition_off = 0;
					ignition_acc = 1;
					ignition_run = 0;
					ignition_str = 0;

					break;
				}

				// By this point, ignition must be off
				if (ignition_off != 1) {
					DEBUGLN("[dieslg8][CAN ][IGN ][OFF ] Active");

					reset_fuel();
					reset_oil();
					reset_acc();

					temp_flashed = 0;
				}

				ignition_off = 1;
				ignition_acc = 0;
				ignition_run = 0;
				ignition_str = 0;

				break;
			} // 0x130

			case 0x612 : { // Response to STATUS_MESSWERTE_BLOCK request
				print_msg = 0;

				// TODO: Why the f**k doesnt the case statement work here
				if (data_expected == 0) { // 0 = Ambient + Boost actual
					unsigned int value_01 = (buf[4] << 8) | buf[5];
					unsigned int value_02 = (buf[6] << 8) | buf[7];

					ambient_hpa = value_01 * 0.030518;
					ambient_psi = ambient_hpa / hpa2psi;

					float boost_hpa_actual_last = boost_hpa_actual;

					boost_hpa_actual = (value_02 * 0.091554) - ambient_hpa;
					if (boost_hpa_actual < 0) boost_hpa_actual = 0;

					boost_psi_actual = boost_hpa_actual / hpa2psi;

					// DEBUG("[dieslg8][CAN ][DATA][AMBh] "); DEBUGLN(ambient_hpa);
					// DEBUG("[dieslg8][CAN ][DATA][BSAh] "); DEBUGLN(boost_hpa_actual);
					// DEBUG("[dieslg8][CAN ][DATA][AMBp] "); DEBUGLN(ambient_psi);
					// DEBUG("[dieslg8][CAN ][DATA][BSAp] "); DEBUGLN(boost_psi_actual);

					if (boost_hpa_actual_last != boost_hpa_actual) {
						hijack_fuel_boost();
					}
				}
				else if (data_expected == 1) { // 1 = Coolant temp + Boost target
					unsigned int value_01 = (buf[4] << 8) | buf[5];
					unsigned int value_02 = (buf[6] << 8) | buf[7];

					float boost_hpa_target_last = boost_hpa_target;

					boost_hpa_target = (value_01 * 0.091554) - ambient_hpa;
					if (boost_hpa_target < 0) boost_hpa_target = 0;

					boost_psi_target = boost_hpa_target / hpa2psi;

					int coolant_temp_c_last = coolant_temp_c;
					coolant_temp_c = (value_02 * 0.01) - 100;

					// DEBUG("[dieslg8][CAN ][DATA][BSTh] "); DEBUGLN(boost_hpa_target);
					// DEBUG("[dieslg8][CAN ][DATA][BSTp] "); DEBUGLN(boost_psi_target);
					// DEBUG("[dieslg8][CAN ][DATA][BS p] "); DEBUG(boost_psi_actual); DEBUG("/"); DEBUGLN(boost_psi_target);
					// DEBUG("[dieslg8][CAN ][DATA][CLTc] "); DEBUGLN(coolant_temp_c);


					if (coolant_temp_c > coolant_c_target_hi) {
						temp_flash();
						hijack_acc();
					}
					else if (coolant_temp_c < coolant_c_target_lo) {
						reset_acc();
					}

					if (boost_hpa_target_last != boost_hpa_target) hijack_oil_boost();

					if (coolant_temp_c_last != coolant_temp_c) hijack_fuel_coolant();
					if (coolant_temp_c_last != coolant_temp_c) hijack_oil_coolant();
				}
				else if (data_expected == 2) { // 2 = Pedal position + Engine RPM
					unsigned int value_01 = (buf[4] << 8) | buf[5]; // Pedal
					unsigned int value_02 = (buf[6] << 8) | buf[7]; // RPM

					throttle_percent = value_01 * 0.012207;
					engine_rpm       = value_02 * 0.5;

					// DEBUG("[dieslg8][CAN ][DATA][THR%] "); DEBUGLN(throttle_percent);
					// DEBUG("[dieslg8][CAN ][DATA][RPM ] "); DEBUGLN(engine_rpm);

					hijack_fuel_boost();
					hijack_oil_boost();
				}

				break;
			} // 0x612

			case 0x660 : { // ACK to DIA message
				print_msg = 0;
			} // 0x660
		}


		if (print_msg == 1) {
			// DEBUGLN("----------------------------------------");
			DEBUG("[dieslg8][CAN ][RECV] 0x");
			DEBUG2(arbid, HEX);
			DEBUG(" => ");

			// Print the data
			for (int i = 0; i < len; i++) {
				DEBUG2(buf[i], HEX);
				DEBUG(" ");
			}

			DEBUGLN();
		}
	}
}


/* vim: set filetype=cpp ts=2 sw=2 tw=0 noet :*/

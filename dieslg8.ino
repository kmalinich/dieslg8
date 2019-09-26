// Debug mode
#define DBG true

// Debug macro to print messages to serial
#define DEBUG(x)     if (DBG && Serial) { Serial.print(x);     }
#define DEBUGLN(x)   if (DBG && Serial) { Serial.println(x);   }
#define DEBUG2(x, y) if (DBG && Serial) { Serial.print(x, y);  }


// Config: disable/enable SD card performance data logging
// #define logging_perf_enable true

// Config: Control active cruise LEDs
// #define acc_led_enable true

// Config: disable/enable automatic code clearing
// #define code_clear_all_enable true
#define code_clear_specific_enable true

// Config: disable/enable turn signal LED flash when coolant temp reaches target
// #define temp_flash_enable true

// Config: disable/enable gauge sweep
#define gauge_sweep_enable true

// Config: disable/enable gauge hijacking
#define hijack_fuel_boost_enable true
// #define hijack_fuel_coolant_enable 1
// #define hijack_oil_boost_enable 1
#define hijack_oil_coolant_enable true


// Config: unit limits for gauges
#define boost_psi_max 40
#define coolant_c_max 150

// Config: Various value targets
#define coolant_c_target_hi 76
#define coolant_c_target_lo 74


// Config: step limits for gauges
#define steps_max_large 4667 // Large gauges (speedo, tach)
#define steps_max_small 1800 // Small gauges (fuel %, oil)

// 1 psi = 68.9475729318 hPa
// 1 hPA = 0.0145037738 psi
#define hpa2psi 68.9475729318
#define psi2hpa 0.0145037738

// Config: SPI pins
#define SPI_CS_CAN 9
#ifdef logging_perf_enable
#define SPI_CS_SD  4
#endif

// External libraries
#include <mcp_can.h>
#include <SPI.h>

#ifdef logging_perf_enable
#include <SD.h>
#endif


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
#ifdef temp_flash_enable
bool temp_flashed = 0;
#endif
#ifdef acc_led_enable
bool acc_led_on = 0;
#endif

#if defined(gauge_sweep_enable) || defined(hijack_fuel_boost_enable) || defined(hijack_fuel_coolant_enable)
bool hijack_fuel_active = 0;
#endif
#if defined(gauge_sweep_enable) || defined(hijack_oil_boost_enable) || defined(hijack_oil_coolant_enable)
bool hijack_oil_active  = 0;
#endif

bool ignition_off = 1;
bool ignition_acc = 0;
bool ignition_run = 0;
bool ignition_sta = 0;


// Set CS pins
MCP_CAN CAN(SPI_CS_CAN);

// Declare File handles for logging
#ifdef logging_perf_enable
File log_file_perf;
#endif



// Send CAN message
void can_send(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
	/*
		 DEBUG("[dieslg8][CAN ][SEND] 0x");
		 DEBUG2(address, HEX);
		 DEBUG(" => ");
		 DEBUG2(a, HEX); DEBUG(" ");
		 DEBUG2(b, HEX); DEBUG(" ");
		 DEBUG2(c, HEX); DEBUG(" ");
		 DEBUG2(d, HEX); DEBUG(" ");
		 DEBUG2(e, HEX); DEBUG(" ");
		 DEBUG2(f, HEX); DEBUG(" ");
		 DEBUG2(g, HEX); DEBUG(" ");
		 DEBUG2(h, HEX); DEBUG(" ");
		 DEBUGLN();
		 */

	unsigned char DataToSend[8] = {a, b, c, d, e, f, g, h};

	delay(25);
	CAN.sendMsgBuf(address, 0, 8, DataToSend);
}


#ifdef code_clear_specific_enable
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
	DEBUGLN("[dieslg8][CAN ][FUNC][DTC ][CLR ] Specific");

	can_send(0x6F1, 0x12, 0x04, 0x18, 0x02, 0xFF, 0xFF, 0x00, 0x00); delay(75);

	// turn_set(0x01);
	can_send(0x612, 0xF1, 0x10, 0x1A, 0x58, 0x08, 0x4C, 0xAE, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x23, 0x3F, 0xF1, 0x21, 0x4B, 0x39, 0x21); delay(75);
	can_send(0x612, 0xF1, 0x24, 0x4A, 0x24, 0xE1, 0xFF, 0xFF, 0xFF); delay(75);

	// turn_set(0x02);
	can_send(0x612, 0xF1, 0x21, 0x48, 0x5C, 0xE1, 0x4B, 0x73, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x03, 0x14, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x7F, 0x14, 0x78, 0xFF, 0xFF, 0xFF); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x54, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF); delay(75);

	// turn_reset();

	// From code_clear_all()
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
}
#endif

#ifdef code_clear_all_enable
// Clear all DDE codes
void code_clear_all() {
	DEBUGLN("[dieslg8][CAN ][FUNC][DTC ][CLR ] All");

	// turn_set(0x01);

	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x02); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x01); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x02); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x01); delay(100);
	// can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);

	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);

	// turn_reset();
}
#endif


#ifdef temp_flash_enable
void temp_flash() {
	// Return if already flashed
	if (temp_flashed == 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][COOL] Turn LED flash");

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
#endif


#ifdef acc_led_enable
// Illuminate active cruise LED(s) in the cluster
// led_mask_1:
// 0x01 :
// 0x02 :
// 0x03 :
void acc_set() {
	// Return if already on
	if (acc_led_on == 1) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][COOL] Active cruise LEDs set");

	can_send(0x6F1, 0x60, 0x03, 0x70, 0x27, 0x6A, 0x00, 0x00, 0x00);

	// Mark as on
	acc_led_on = 1;
}

// Reset active cruise LED(s) in the cluster
void acc_reset() {
	// Return if already off
	if (acc_led_on == 0) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][COOL] Active cruise LEDs reset");

	can_send(0x6F1, 0x60, 0x03, 0x70, 0x2B, 0x6E, 0x00, 0x00, 0x00);

	// Mark as off
	acc_led_on = 0;
}
#endif


#if defined(temp_flash_enable) || defined(code_clear_all_enable) || defined(code_clear_specific_enable)
// Illuminate turn signal LED(s) in the cluster
// led_mask:
// 0x01 : Left
// 0x02 : Right
// 0x03 : Both
void turn_set(unsigned int led_mask) {
	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][TURN] Set");

	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, led_mask, 0x04, 0x00);
}

// Reset turn signal LED(s) in the cluster
void turn_reset() {
	DEBUGLN("[dieslg8][CAN ][FUNC][KOMB][TURN] Reset");

	can_send(0x6F1, 0x60, 0x03, 0x30, 0x2B, 0x00, 0x00, 0x00, 0x00);
}
#endif


#ifdef gauge_sweep_enable
void gauge_sweep() {
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
}
#endif


#ifdef hijack_fuel_boost_enable
void hijack_fuel_boost() {
	// Return if less than 10 hPa, throttle under 39%, or engine RPM under 400
	if (boost_hpa_actual < 10 || throttle_percent < 39 || engine_rpm < 400) {
		reset_fuel();
		return;
	}

	// DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][FUEL] Boost");

	unsigned int steps = boost_hpa_actual * (steps_max_small / (boost_psi_max * hpa2psi));
	hijack_fuel(steps);
}
#endif

#ifdef hijack_fuel_coolant_enable
void hijack_fuel_coolant() {
	// Return if coolant_temp_c is under 0 or above max, or engine RPM is under 400
	if (coolant_temp_c < 0 || coolant_temp_c > coolant_c_max || engine_rpm < 400) {
		return;
	}

	// DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][FUEL] Coolant");

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	hijack_fuel(steps);
}
#endif

#if defined(gauge_sweep_enable) || defined(hijack_fuel_boost_enable) || defined(hijack_fuel_coolant_enable)
void hijack_fuel(unsigned int steps) {
	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		reset_fuel();
		return;
	}

	hijack_fuel_active = 1;
	hijack_gauge(0x22, steps);
}

void reset_fuel() {
	// Return if hijack is inactive
	if (hijack_fuel_active == 0) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][FUEL] Reset");

	reset_gauge(0x22);
	hijack_fuel_active = 0;
}
#endif


#ifdef hijack_oil_boost_enable
void hijack_oil_boost() {
	// Return if less than 10 hPa, throttle under 39%, or engine RPM under 400
	if (boost_hpa_target < 10 || throttle_percent < 39 || engine_rpm < 400) {
		reset_oil();
		return;
	}

	// DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][OIL ] Boost");

	unsigned int steps = boost_hpa_target * (steps_max_small / (boost_psi_max * hpa2psi));
	hijack_oil(steps);
}
#endif

#ifdef hijack_oil_coolant_enable
void hijack_oil_coolant() {
	// Return if coolant_temp_c is under 0 or above max, or engine RPM is under 400
	if (coolant_temp_c < 0 || coolant_temp_c > coolant_c_max || engine_rpm < 400) {
		return;
	}

	DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][OIL ] Coolant");

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	hijack_oil(steps);
}
#endif

#if defined(gauge_sweep_enable) || defined(hijack_oil_boost_enable) || defined(hijack_oil_coolant_enable)
void hijack_oil(unsigned int steps) {
	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		reset_oil();
		return;
	}

	hijack_oil_active = 1;
	hijack_gauge(0x23, steps);
}

void reset_oil() {
	// Return if hijack is inactive
	if (hijack_oil_active == 0) return;

	DEBUGLN("[dieslg8][CAN ][FUNC][HIJK][OIL ] Reset");

	reset_gauge(0x23);
	hijack_oil_active = 0;
}
#endif


// Hijack a gauge
#if defined(gauge_sweep_enable) || defined(hijack_fuel_coolant_enable) || defined(hijack_fuel_coolant_enable) || defined(hijack_oil_boost_enable) || defined(hijack_oil_coolant_enable)
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
#endif


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


#ifdef logging_perf_enable
void sdcard_log_perf() {
	// Toggle LED
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

	log_file_perf = SD.open("logfile_perf.csv", FILE_WRITE);

	log_file_perf.print(ignition_off);     log_file_perf.print(",");
	log_file_perf.print(ignition_acc);     log_file_perf.print(",");
	log_file_perf.print(ignition_run);     log_file_perf.print(",");
	log_file_perf.print(ignition_sta);     log_file_perf.print(",");
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
	digitalWrite(LED_BUILTIN, HIGH);

	// Wait for serial connection when debugging
	if (DBG) {
		// Initialize serial output for logging
		Serial.begin(115200);
		while (!Serial) ;
		delay(1000);
	}

	// Init CAN, baudrate 500k
	while (CAN.begin(CAN_500KBPS) != CAN_OK) {
		DEBUGLN("[dieslg8][INIT][CAN ] Waiting");
		delay(1000);
	}

	DEBUGLN("[dieslg8][INIT][CAN ] OK");

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
#ifdef code_clear_specific_enable
		if (ignition_run == 1) code_clear_specific();
#endif
#ifdef code_clear_all_enable
		if (ignition_run == 1) code_clear_all();
#endif
	}

	// Check if incoming data is available
	if (CAN_MSGAVAIL == CAN.checkReceive()) {
		bool print_msg = 1;

		// Ignition bitmask values to match against
		const int mask_ignition_acc = 0xC1; // buf[0]
		const int mask_ignition_run = 0xC5; // buf[0]
		const int mask_ignition_sta = 0xD5; // buf[0]

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
			case 0x130 : // Ignition status
				print_msg = 0;

				// DEBUGLN("[dieslg8][CAN ][PARS] Ignition status received");

				// Test if all bits in mask_ignition_sta are present in buf[0]
				if ((buf[0] & mask_ignition_sta) == mask_ignition_sta) {
					if (ignition_sta != 1) {
						DEBUGLN("[dieslg8][CAN ][IGN ][STA ] Active");
#ifdef code_clear_all_enable
						code_clear_all();
#endif
#ifdef code_clear_specific_enable
						code_clear_specific();
#endif
					}

					if (ignition_run != 1) {
						DEBUGLN("[dieslg8][CAN ][IGN ][RUN ] Active");
#ifdef gauge_sweep_enable
						gauge_sweep();
#endif
#ifdef code_clear_specific_enable
						code_clear_specific();
#endif
#ifdef code_clear_all_enable
						code_clear_all();
#endif

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
						DEBUGLN("[dieslg8][CAN ][IGN ][RUN ] Active");
#ifdef gauge_sweep_enable
						gauge_sweep();
#endif
#ifdef code_clear_specific_enable
						code_clear_specific();
#endif
#ifdef code_clear_all_enable
						code_clear_all();
#endif
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
						DEBUGLN("[dieslg8][CAN ][IGN ][ACC ] Active");
#if defined(hijack_fuel_boost_enable) || defined(hijack_fuel_coolant_enable)
						reset_fuel();
#endif
#if defined(hijack_oil_boost_enable) || defined(hijack_oil_coolant_enable)
						reset_oil();
#endif
#ifdef acc_led_enable
						acc_reset();
#endif
#ifdef temp_flash_enable
						temp_flashed = 0;
#endif
					}

					ignition_off = 0;
					ignition_acc = 1;
					ignition_run = 0;
					ignition_sta = 0;
					break;
				}

				// By this point, ignition must be off
				if (ignition_off != 1) {
					DEBUGLN("[dieslg8][CAN ][IGN ][OFF ] Active");
#if defined(hijack_fuel_boost_enable) || defined(hijack_fuel_coolant_enable)
					reset_fuel();
#endif
#if defined(hijack_oil_boost_enable) || defined(hijack_oil_coolant_enable)
					reset_oil();
#endif
#ifdef acc_led_enable
					acc_reset();
#endif
#ifdef temp_flash_enable
					temp_flashed = 0;
#endif
				}

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

					ambient_hpa = value_01 * 0.030518;
					ambient_psi = ambient_hpa / hpa2psi;

					boost_hpa_actual = (value_02 * 0.091554) - ambient_hpa;
					if (boost_hpa_actual < 0) boost_hpa_actual = 0;

					boost_psi_actual = boost_hpa_actual / hpa2psi;

					// DEBUG("[dieslg8][CAN ][DATA][AMBh] "); DEBUGLN(ambient_hpa);
					// DEBUG("[dieslg8][CAN ][DATA][BSAh] "); DEBUGLN(boost_hpa_actual);
					// DEBUG("[dieslg8][CAN ][DATA][AMBp] "); DEBUGLN(ambient_psi);
					// DEBUG("[dieslg8][CAN ][DATA][BSAp] "); DEBUGLN(boost_psi_actual);

#ifdef hijack_fuel_boost_enable
					hijack_fuel_boost();
#endif
#ifdef hijack_oil_boost_enable
					hijack_oil_boost();
#endif
				}
				else if (data_expected == 1) { // 1 = Coolant temp + Boost target
					unsigned int value_01 = (buf[4] << 8) | buf[5];
					unsigned int value_02 = (buf[6] << 8) | buf[7];

					boost_hpa_target = (value_01 * 0.091554) - ambient_hpa;
					if (boost_hpa_target < 0) boost_hpa_target = 0;

					boost_psi_target = boost_hpa_target / hpa2psi;

					int coolant_temp_c_new = (value_02 * 0.01) - 100;

					// DEBUG("[dieslg8][CAN ][DATA][BSTh] "); DEBUGLN(boost_hpa_target);
					// DEBUG("[dieslg8][CAN ][DATA][BSTp] "); DEBUGLN(boost_psi_target);
					// DEBUG("[dieslg8][CAN ][DATA][BS p] "); DEBUG(boost_psi_actual); DEBUG("/"); DEBUGLN(boost_psi_target);
					// DEBUG("[dieslg8][CAN ][DATA][CLTc] "); DEBUGLN(coolant_temp_c);

#ifdef temp_flash_enable
					if (coolant_temp_c > coolant_c_target_hi) temp_flash();
#endif
#ifdef acc_led_enable
					if (coolant_temp_c > coolant_c_target_hi) {
						acc_set();
					}
					else if (coolant_temp_c < coolant_c_target_lo) {
						acc_reset();
					}
#endif

#ifdef hijack_fuel_boost_enable
					hijack_fuel_boost();
#endif
#ifdef hijack_fuel_coolant_enable
					if (coolant_temp_c_new != coolant_temp_c) hijack_fuel_coolant();
#endif
#ifdef hijack_oil_boost_enable
					hijack_oil_boost();
#endif
#ifdef hijack_oil_coolant_enable
					if (coolant_temp_c_new != coolant_temp_c) hijack_oil_coolant();
#endif

					coolant_temp_c = coolant_temp_c_new;
				}
				else if (data_expected == 2) { // 2 = Pedal position + Engine RPM
					unsigned int value_01 = (buf[4] << 8) | buf[5]; // Pedal
					unsigned int value_02 = (buf[6] << 8) | buf[7]; // RPM

					throttle_percent = value_01 * 0.012207;
					engine_rpm       = value_02 * 0.5;

					// DEBUG("[dieslg8][CAN ][DATA][THR%] "); DEBUGLN(throttle_percent);
					// DEBUG("[dieslg8][CAN ][DATA][RPM ] "); DEBUGLN(engine_rpm);

#ifdef hijack_fuel_boost_enable
					hijack_fuel_boost();
#endif
#ifdef hijack_oil_boost_enable
					hijack_oil_boost();
#endif
				}

				break;

			case 0x660 : // ACK to DIA message
				print_msg = 0;
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

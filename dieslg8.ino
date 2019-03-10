#include <mcp_can.h>
#include <SD.h>
#include <SPI.h>


const int SPI_CS_CAN = 9;
const int SPI_CS_SD  = 4;

File log_file;

const int mask_ignition_acc = 0xC1; // buf[0]
const int mask_ignition_run = 0xC5; // buf[0]
const int mask_ignition_sta = 0xD5; // buf[0]

unsigned long loop_count_01 = 0;
unsigned long loop_count_02 = 0;

// 0 = ambient + boost actual
// 1 = coolant + boost target
unsigned int data_expected = 0;

// Step limits for gauges
const int steps_max_large = 4667; // Large gauges (speedo, tach)
const int steps_max_small = 1800; // Small gauges (fuel %, oil)

// Unit limits for gauges
const int boost_psi_max = 40;
const int coolant_c_max = 100;

// 1 psi = 68.9475729318 hPa
// 1 hPA = 0.0145037738 psi
const float hpa2psi = 68.9475729318;
const float psi2hpa = 0.0145037738;


unsigned int ambient_hpa;
unsigned int boost_hpa_actual;
unsigned int boost_hpa_target;

float boost_psi_actual;
float boost_psi_target;

float coolant_temp_c;

bool temp_flashed = 0;

bool fuel_hijack_active = 0;
bool oil_hijack_active  = 0;

bool ignition_off = 1;
bool ignition_acc = 0;
bool ignition_run = 0;
bool ignition_sta = 0;

bool sweep_done = 0;


// Set CS pins
MCP_CAN CAN(SPI_CS_CAN);



// Send CAN message
void can_send(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
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
	Serial.println("[dieslg8][CAN ][FUNC] Performing code clear specific");

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
	Serial.println("[dieslg8][CAN ][FUNC] Performing code clear all");

	turn_set(0x01);

	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x02); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x01); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x02); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); turn_set(0x01); delay(100);
	can_send(0x6F1, 0x12, 0x03, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(100);

	turn_reset();
}

void temp_flash() {
	if (temp_flashed == 1) return;

	Serial.println("[dieslg8][CAN ][FUNC] Performing coolant temp LED flash");

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
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, led_mask, 0x04, 0x00);
}

// Reset turn signal LED(s) in the cluster
void turn_reset() {
	can_send(0x6F1, 0x60, 0x03, 0x30, 0x2B, 0x00, 0x00, 0x00, 0x00);
}


void gauge_sweep() {
	Serial.println("[dieslg8][CAN ][FUNC] Performing gauge sweep");

	// Speedo/tach to steps_max_large
	gauge_hijack(0x20, steps_max_large);
	gauge_hijack(0x21, steps_max_large);

	// Fuel/oil to steps_max_small
	fuel_hijack(steps_max_small);
	oil_hijack(steps_max_small);

	delay(1000);

	// Reset gauges
	gauge_reset(0x20);
	gauge_reset(0x21);
	gauge_reset(0x22);
	gauge_reset(0x23);
}


void fuel_hijack_boost() {
	// Return if less than 10 hPa
	if (boost_hpa_actual < 10) {
		// fuel_reset();
		return;
	}

	unsigned int steps = boost_hpa_actual * (steps_max_small / (boost_psi_max * hpa2psi));
	fuel_hijack(steps);
}

void fuel_hijack(unsigned int steps) {
	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		fuel_reset();
		return;
	}

	fuel_hijack_active = 1;
	gauge_hijack(0x22, steps);
}

void fuel_reset() {
	// Return if hijack is inactive
	if (fuel_hijack_active == 0) return;
	gauge_reset(0x22);
	fuel_hijack_active = 0;
}


void oil_hijack_boost() {
	// Return if less than 10 hPa
	if (boost_hpa_target < 10) {
		// oil_reset();
		return;
	}

	unsigned int steps = boost_hpa_target * (steps_max_small / (boost_psi_max * hpa2psi));
	oil_hijack(steps);
}

void oil_hijack_coolant() {
	// Return if less than 10 hPa
	if (coolant_temp_c < 0) {
		oil_reset();
		return;
	}

	unsigned int steps = coolant_temp_c * (steps_max_small / coolant_c_max);
	oil_hijack(steps);
}

void oil_hijack(unsigned int steps) {
	// Return if steps are out of bounds
	if (steps > steps_max_small) {
		oil_reset();
		return;
	}

	oil_hijack_active = 1;
	gauge_hijack(0x23, steps);
}

void oil_reset() {
	// Return if hijack is inactive
	if (oil_hijack_active == 0) return;
	gauge_reset(0x23);
	oil_hijack_active = 0;
}


// Hijack a gauge
void gauge_hijack(int gauge_id, int steps) {
	uint8_t msb = (steps / 256); // MSB
	uint8_t lsb = (steps % 256); // LSB

	can_send(0x6F1, 0x60, 0x05, 0x30, gauge_id, 0x06, msb, lsb, 0xFF);
}


// Un-hijack a gauge
void gauge_reset(unsigned int gauge_id) {
	// Gauge IDs
	// 0x20 = Speedometer
	// 0x21 = Tachometer
	// 0x22 = Fuel
	// 0x23 = Oil

	can_send(0x6F1, 0x60, 0x03, 0x30, gauge_id, 0x00, 0xFF, 0xFF, 0xFF);
}

// Get data
// 0x0C1C : [IPUMG]  Ambient pressure  hPa  x*0.030518
// 0x076D : [IPLAD]  Boost actual      hPa  x*0.091554
// 0x01F4 : [SPLAD]  Boost target      hPA  x*0.091554
// 0x0AF1 : [ITMOT]  Engine temp       C    (x/10)-273.14
void status_messwertblock_lesen() {
	// 0 = ambient + boost actual
	// 1 = coolant + boost target
	if (data_expected == 0) {
		data_expected = 1;
		can_send(0x6F1, 0x12, 0x06, 0x2C, 0x10, 0x01, 0xF4, 0x0A, 0xF1);
	}
	else {
		data_expected = 0;
		can_send(0x6F1, 0x12, 0x06, 0x2C, 0x10, 0x0C, 0x1C, 0x07, 0x6D);
	}
}


void sdcard_log() {
	log_file = SD.open("logfile.csv", FILE_WRITE);

	log_file.print(ignition_off);     log_file.print(",");
	log_file.print(ignition_acc);     log_file.print(",");
	log_file.print(ignition_run);     log_file.print(",");
	log_file.print(ignition_sta);     log_file.print(",");
	log_file.print(coolant_temp_c);   log_file.print(",");
	log_file.print(boost_psi_target); log_file.print(",");
	log_file.println(boost_psi_actual);
}


void setup() {
	// Initialize digital pin LED_BUILTIN as an output
	// pinMode(LED_BUILTIN, OUTPUT);

	// Initialize serial output for logging
	Serial.begin(115200);

	// Init CAN, baudrate 500k
	while (CAN_OK != CAN.begin(CAN_500KBPS)) {
		Serial.println("[dieslg8][CAN ][INIT] FAIL");
		delay(1000);
	}

	Serial.println("[dieslg8][CAN ][INIT] OK");

	// Init SD card
	if (!SD.begin(SPI_CS_SD)) {
		Serial.println("[dieslg8][SD  ][INIT] FAIL");
		while(1);
	}

	Serial.println("[dieslg8][SD ][INIT] OK");
}

void loop() {
	unsigned char len = 0;
	unsigned char buf[8];

	loop_count_01++;
	loop_count_02++;

	if (loop_count_01 == 2000) {
		loop_count_01 = 0;
		if (ignition_run == 1) status_messwertblock_lesen();
	}

	if (loop_count_02 == 10000000) {
		loop_count_02 = 0;
		// if (ignition_run == 1) code_clear();
	}

	// Check if incoming data is available
	if (CAN_MSGAVAIL == CAN.checkReceive()) {
		bool print_msg = 1;

		// Read CAN message data
		// len : data length
		// buf : data buffer
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
						// code_clear();
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
						// code_clear();
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
						fuel_reset();
						oil_reset();
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
					fuel_reset();
					oil_reset();
				}

				temp_flashed = 0;

				ignition_off = 1;
				ignition_acc = 0;
				ignition_run = 0;
				ignition_sta = 0;

				break;

			case 0x612 : // Response to STATUS_MESSWERTE_BLOCK request
				print_msg = 0;

				// Serial.print("[dieslg8][DATA][EXP ] "); Serial.println(data_expected);

				// TODO: Why the f**k doesnt the case statement work here
				if (data_expected == 1) { // coolant + boost target
					unsigned int boost_target = (buf[4] << 8) | buf[5];
					unsigned int coolant_temp = (buf[6] << 8) | buf[7];

					boost_hpa_target = (boost_target * 0.091554) - ambient_hpa;
					boost_psi_target = boost_hpa_target / hpa2psi;

					coolant_temp_c = (coolant_temp / 10) - 273.14;

					if (coolant_temp_c > 75) temp_flash();

					// Serial.print("[dieslg8][DATA][BSTT] "); Serial.println(boost_psi_target);
					// Serial.print("[dieslg8][DATA][CLT1] "); Serial.println(coolant_temp_c);

					oil_hijack_boost();
				}
				else { // ambient + boost actual
					unsigned int ambient      = (buf[4] << 8) | buf[5];
					unsigned int boost_actual = (buf[6] << 8) | buf[7];

					ambient_hpa      = ambient       * 0.030518;
					boost_hpa_actual = (boost_actual * 0.091554) - ambient_hpa;
					boost_psi_actual = boost_hpa_actual / hpa2psi;

					// Serial.print("[dieslg8][DATA][AMBP] "); Serial.println(ambient_hpa);
					// Serial.print("[dieslg8][DATA][BSTA] "); Serial.println(boost_psi_actual);

					fuel_hijack_boost();
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


/* vim: set syntax=cpp filetype=cpp ts=2 sw=2 tw=0 et :*/

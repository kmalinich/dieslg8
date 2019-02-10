#include <mcp_can.h>
#include <SPI.h>


const int SPI_CS_PIN = 9;

const int mask_ignition_acc = 0xC1; // buf[0]
const int mask_ignition_run = 0xC5; // buf[0]
const int mask_ignition_sta = 0xD5; // buf[0]

unsigned int loop_count = 0;

float ambient_hpa;
float boost_hpa;
float boost_psi;

boolean fuel_hijack_active = 0;
boolean oil_hijack_active  = 0;

boolean ignition_off = 1;
boolean ignition_acc = 0;
boolean ignition_run = 0;
boolean ignition_sta = 0;

boolean sweep_done = 0;


// Set CS pin
MCP_CAN CAN(SPI_CS_PIN);


void led_blink() {
	Serial.println("[dieslg8][LED] Blink");

	// turn the LED on (HIGH is the voltage level)
	Serial.println("[dieslg8][LED] HIGH");
	digitalWrite(LED_BUILTIN, HIGH);
	delay(250);

	// turn the LED off by making the voltage LOW
	Serial.println("[dieslg8][LED] LOW");
	digitalWrite(LED_BUILTIN, LOW);
}

// Send CAN message
void can_send(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
	// Serial.print("[dieslg8][CAN][SEND] 0x");
	// Serial.print(address, HEX);
	// Serial.print(" => ");

	// Print the data
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
	Serial.println("[dieslg8][CAN][FUNC] Performing code clear");

	can_send(0x6F1, 0x12, 0x04, 0x18, 0x02, 0xFF, 0xFF, 0x00, 0x00); delay(75);

	turn_l();
	can_send(0x612, 0xF1, 0x10, 0x1A, 0x58, 0x08, 0x4C, 0xAE, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x23, 0x3F, 0xF1, 0x21, 0x4B, 0x39, 0x21); delay(75);
	can_send(0x612, 0xF1, 0x24, 0x4A, 0x24, 0xE1, 0xFF, 0xFF, 0xFF); delay(75);

	turn_r();
	can_send(0x612, 0xF1, 0x21, 0x48, 0x5C, 0xE1, 0x4B, 0x73, 0xE1); delay(75);
	can_send(0x6F1, 0x12, 0x03, 0x14, 0xFF, 0xFF, 0x00, 0x00, 0x00); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x7F, 0x14, 0x78, 0xFF, 0xFF, 0xFF); delay(75);
	can_send(0x612, 0xF1, 0x03, 0x54, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF); delay(75);

	turn_reset();
}


// Illuminate turn signal LED(s) in the cluster
void turn_both() {
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, 0x03, 0x04, 0x00);
}

void turn_l() {
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, 0x01, 0x04, 0x00);
}

void turn_r() {
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x2B, 0x06, 0x02, 0x04, 0x00);
}

// Reset turn signal LED(s) in the cluster
void turn_reset() {
	can_send(0x6F1, 0x60, 0x03, 0x30, 0x2B, 0x00, 0x00, 0x00, 0x00);
}


void gauge_sweep() {
	Serial.println("[dieslg8][CAN][FUNC] Performing gauge sweep");

	// Speedo to 4622 :: 260kph
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x20, 0x06, 0x12, 0x3B, 0xFF);

	// Tach to 4667 :: 7000RPM
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x21, 0x06, 0x12, 0x0E, 0xFF);

	// Fuel to 1800 :: 100%
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x22, 0x06, 0x07, 0x08, 0xFF);

	// Oil/cons to 1800 :: 100%
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x23, 0x06, 0x07, 0x08, 0xFF);


	// Speedo reset
	delay(1000);
	can_send(0x6F1, 0x60, 0x03, 0x30, 0x20, 0x00, 0xFF, 0xFF, 0xFF);

	// Tach reset
	can_send(0x6F1, 0x60, 0x03, 0x30, 0x21, 0x00, 0xFF, 0xFF, 0xFF);

	// Fuel reset
	fuel_reset();

	// Oil/cons reset
	oil_reset();
}


void fuel_hijack(boost_hpa) {
	// Return if less than 15 psi
	if (boost_hpa < 1034) {
		fuel_reset();
		return;
	}

	int steps = ((boost_hpa * 1.4506769825918762) - 40);

	// Return if steps are out of bounds
	if (steps < 0 || steps > 1800) {
		fuel_reset();
		return;
	}

	// Serial.print("[dieslg8][HIJK][FUEL] Steps : ");
	// Serial.println(steps);

	uint8_t lsb = (steps % 256); // LSB
	uint8_t msb = (steps / 256); // MSB

	fuel_hijack_active = 1;
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x22, 0x06, msb, lsb, 0xFF);
}

void oil_hijack() {
	// Return if less than 15 psi
	if (boost_hpa < 1034) {
		oil_reset();
		return;
	}

	int steps = ((boost_hpa * 1.4506769825918762) - 40);

	// Return if steps are out of bounds
	if (steps < 0 || steps > 1800) {
		oil_reset();
		return;
	}

	// Serial.print("[dieslg8][HIJK][OIL] Steps : ");
	// Serial.println(steps);

	uint8_t lsb = (steps % 256); // LSB
	uint8_t msb = (steps / 256); // MSB

	oil_hijack_active = 1;
	can_send(0x6F1, 0x60, 0x05, 0x30, 0x23, 0x06, msb, lsb, 0xFF);
}


void fuel_reset() {
	can_send(0x6F1, 0x60, 0x03, 0x30, 0x22, 0x00, 0xFF, 0xFF, 0xFF);
	fuel_hijack_active = 0;
}

void oil_reset() {
	can_send(0x6F1, 0x60, 0x03, 0x30, 0x23, 0x00, 0xFF, 0xFF, 0xFF);
	oil_hijack_active = 0;
}

// Get data
// 0x0C1C : [IPLAD] Ambient pressure hPA
// 0x076D : [IPUMG] Boost actual     hPA
void status_messwertblock_lesen() {
	can_send(0x6F1, 0x12, 0x06, 0x2C, 0x10, 0x0C, 0x1C, 0x07, 0x6D);
}


void setup() {
	// Initialize digital pin LED_BUILTIN as an output
	pinMode(LED_BUILTIN, OUTPUT);

	// Initialize serial output for logging
	Serial.begin(115200);

	// Initialize CAN baudrate 500k
	while (CAN_OK != CAN.begin(CAN_500KBPS)) {
		Serial.println("[dieslg8][CAN][INIT] FAIL");
		delay(1000);
	}

	Serial.println("[dieslg8][CAN][INIT] OK");
}

void loop() {
	boolean print_msg = 1;

	unsigned char len = 0;
	unsigned char buf[8];

	// Check if incoming data is available
	if (CAN_MSGAVAIL == CAN.checkReceive()) {
		// led_blink();

		// Read CAN message data
		// len : data length
		// buf : data buffer
		CAN.readMsgBuf(&len, buf);

		unsigned long arbid = CAN.getCanId();

		switch (arbid) {
			case 0x130 : // Ignition status
				print_msg = 0;

				// Serial.println("[dieslg8][CAN][PARS] Ignition status received");

				// Test if all bits in mask_ignition_sta are present in buf[0]
				if ((buf[0] & mask_ignition_sta) == mask_ignition_sta) {
					if (ignition_sta != 1) {
						Serial.println("[dieslg8][IGN][STA ] Active");
					}

					if (ignition_run != 1) {
						Serial.println("[dieslg8][IGN][RUN ] Active");
						gauge_sweep();
						code_clear();
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
						Serial.println("[dieslg8][IGN][RUN ] Active");
						gauge_sweep();
						code_clear();
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
						Serial.println("[dieslg8][IGN][ACC ] Active");
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
					Serial.println("[dieslg8][IGN][OFF ] Active");
					fuel_reset();
					oil_reset();
				}

				ignition_off = 1;
				ignition_acc = 0;
				ignition_run = 0;
				ignition_sta = 0;

				break;

			case 0x612 : // Response to STATUS_MESSWERTE_BLOCK request
				print_msg = 0;

				unsigned int ambient = (buf[4] << 8) | buf[5];
				unsigned int boost   = (buf[6] << 8) | buf[7];

				ambient_hpa = ambient * 0.030518;
				boost_hpa   = ((boost  * 0.091554) - 5) - ambient_hpa;
				boost_psi   = boost_hpa / 68.947572932;

				// Serial.print("[dieslg8][BST][PSI] ");
				// Serial.println(boost_psi);

				fuel_hijack(boost_hpa);

				break;

			case 0x660 : // ACK to DIA message
				print_msg = 0;
				break;
		}

		// Serial.print("[dieslg8][IGN][STAT] Off/Acc/Run/Sta ");
		// Serial.print(ignition_off);
		// Serial.print(ignition_acc);
		// Serial.print(ignition_run);
		// Serial.print(ignition_sta);
		// Serial.println();


		if (print_msg == 0) return;

		// Serial.println("----------------------------------------");
		Serial.print("[dieslg8][CAN][RECV] 0x");
		Serial.print(arbid, HEX);
		Serial.print(" => ");

		// Print the data
		for (int i = 0; i < len; i++) {
			Serial.print(buf[i], HEX);
			Serial.print(" ");
		}

		Serial.println();
	}

	loop_count++;

	if (loop_count == 750) {
		loop_count = 0;
		// Serial.println("[dieslg8][LOOP]");
		if (ignition_run == 1) {
			status_messwertblock_lesen();
		}
	}
}

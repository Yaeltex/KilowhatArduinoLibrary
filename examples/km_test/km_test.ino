#include "KM_Data.h"


//PARA PROBAR, DESCOMENTAR ESTE DEFINE LA PRIMERA VEZ QUE SE SUBE/PRUEBA Y LUEGO DEJARLA COMENTADA PARA TESTEAR
//#define WRITE_EEPROM

#define SERIAL_PRINT(lbl, data) { Serial.print(lbl); Serial.print(": "); Serial.println(data); }

byte test_data[] ={ 89, 84, 88, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 32, 8, 2, 0, 100, 0, 127, 0, 5, 0, 40, 2, 0, 0, 0, 127, 1, 2, 0, 1, 0, 127, 1, 2, 0, 2, 0, 127, 1, 2, 0, 3, 0, 127, 1, 2, 0, 4, 0, 127, 1, 2, 0, 5, 0, 127, 1, 2, 0, 6, 0, 127, 1, 2, 0, 7, 0, 127, 1, 2, 0, 16, 0, 127, 1, 2, 0, 17, 0, 127, 1, 2, 0, 18, 0, 127, 1, 2, 0, 19, 0, 127, 1, 2, 0, 8, 0, 127, 1, 2, 0, 9, 0, 127, 1, 2, 0, 10, 0, 127, 1, 2, 0, 11, 0, 127, 1, 1, 0, 0, 0, 127, 0, 1, 0, 1, 0, 127, 0, 1, 0, 2, 0, 127, 0, 1, 0, 3, 0, 127, 0, 1, 0, 4, 0, 127, 0, 1, 0, 5, 0, 127, 0, 7, 0, 0, 0, 127, 2, 7, 0, 1, 0, 127, 2, 1, 0, 20, 0, 127, 1, 2, 0, 21, 0, 127, 1, 2, 0, 22, 0, 127, 1, 2, 0, 23, 0, 127, 1, 2, 0, 12, 0, 127, 1, 2, 0, 13, 0, 127, 1, 2, 0, 14, 0, 127, 1, 2, 0, 15, 0, 127, 1, 0, 0, 42, 84, 0, 1, 42, 84, 0, 2, 42, 84, 0, 3, 42, 84, 0, 4, 42, 84, 0, 5, 42, 84, 2, 0, 42, 84, 2, 1, 42, 84, 2, 0, 101, 0, 127, 0, 5, 0, 45, 2, 0, 24, 0, 127, 1, 2, 0, 25, 0, 127, 3, 2, 0, 26, 0, 127, 3, 2, 0, 27, 0, 127, 3, 2, 0, 28, 0, 127, 3, 2, 0, 29, 0, 127, 3, 2, 0, 30, 0, 127, 3, 2, 0, 31, 0, 127, 3, 2, 0, 40, 0, 127, 3, 2, 0, 41, 0, 127, 3, 2, 0, 42, 0, 127, 3, 2, 0, 43, 0, 127, 3, 2, 0, 32, 0, 127, 3, 2, 0, 33, 0, 127, 3, 2, 0, 34, 0, 127, 3, 2, 0, 35, 0, 127, 3, 1, 0, 6, 0, 127, 0, 1, 0, 7, 0, 127, 0, 1, 0, 8, 0, 127, 0, 1, 0, 9, 0, 127, 0, 1, 0, 10, 0, 127, 0, 1, 0, 11, 0, 127, 0, 7, 0, 0, 0, 127, 2, 7, 0, 1, 0, 127, 2, 2, 0, 44, 0, 127, 3, 2, 0, 45, 0, 127, 3, 2, 0, 46, 0, 127, 3, 2, 0, 47, 0, 127, 3, 2, 0, 36, 0, 127, 3, 2, 0, 37, 0, 127, 3, 2, 0, 38, 0, 127, 3, 2, 0, 39, 0, 127, 3, 9, 16, 42, 84, 9, 17, 42, 84, 9, 18, 42, 84, 9, 19, 42, 84, 8, 20, 42, 84, 8, 21, 42, 84, 10, 0, 42, 84, 10, 1, 42, 84};

void printInputNorm(const char *lbl, KMS::InputNorm &d, int i) {
	SERIAL_PRINT(lbl, i);
	SERIAL_PRINT("\tAnalog", d.AD());
	SERIAL_PRINT("\tMode", d.mode());
	SERIAL_PRINT("\tChannel", d.channel());
	SERIAL_PRINT("\tParam fine (nrpn)", d.param_fine());
    SERIAL_PRINT("\tParam coarse", d.param_coarse());
    SERIAL_PRINT("\tParam full NRPN", (d.param_coarse() << 7) | d.param_fine());
    SERIAL_PRINT("\tMin", d.param_min());
    SERIAL_PRINT("\tMax", d.param_max());
}

void printInputUS(const char *lbl, KMS::InputUS &d, int i) {
	SERIAL_PRINT(lbl, i);
	SERIAL_PRINT("\tMode", d.mode());
	SERIAL_PRINT("\tChannel", d.channel());
	SERIAL_PRINT("\tParam fine (nrpn)", d.param_fine());
    SERIAL_PRINT("\tParam coarse", d.param_coarse());
    SERIAL_PRINT("\tParam full NRPN", (d.param_coarse() << 7) | d.param_fine());
    SERIAL_PRINT("\tMin", d.param_min());
    SERIAL_PRINT("\tMax", d.param_max());
}

void printOutput(const char *lbl, KMS::Output &d, int i) {
    SERIAL_PRINT(lbl, i);
    SERIAL_PRINT("\tChannel", d.channel());
    SERIAL_PRINT("\tNote", d.param());
    SERIAL_PRINT("\tBlink", d.blink());
    SERIAL_PRINT("\ttBlink Min", d.blink_min());
    SERIAL_PRINT("\ttBlink Max", d.blink_max());
}

void setup() {
	Serial.begin(250000);
  KMS::initialize();

//  
//  KMS::io.write(0, test_data, sizeof(test_data));
//  
//  KMS::InputUS ultrasonicSensorData = KMS::ultrasound();
//    
//  KMS::setBank(0);
//    SERIAL_PRINT("US bank ", KMS::bank());
//    SERIAL_PRINT("\tMode", ultrasonicSensorData.mode());
//    SERIAL_PRINT("\tChannel", ultrasonicSensorData.channel());
//    SERIAL_PRINT("\tParam fine (nrpn)", ultrasonicSensorData.param_fine());
//    SERIAL_PRINT("\tParam coarse", ultrasonicSensorData.param_coarse());
//    SERIAL_PRINT("\tParam full NRPN", (ultrasonicSensorData.param_coarse() << 7) | ultrasonicSensorData.param_fine());
//    SERIAL_PRINT("\tMin", ultrasonicSensorData.param_min());
//    SERIAL_PRINT("\tMax", ultrasonicSensorData.param_max());
//  KMS::setBank(1);
//    byte i = 101;
//    Serial.println(ultrasonicSensorData.wr_param_fine(&i));
//    SERIAL_PRINT("US bank ", KMS::bank());
//    SERIAL_PRINT("\tMode", ultrasonicSensorData.mode());
//    SERIAL_PRINT("\tChannel", ultrasonicSensorData.channel());
//    SERIAL_PRINT("\tParam fine (nrpn)", ultrasonicSensorData.param_fine());
//    SERIAL_PRINT("\tParam coarse", ultrasonicSensorData.param_coarse());
//    SERIAL_PRINT("\tParam full NRPN", (ultrasonicSensorData.param_coarse() << 7) | ultrasonicSensorData.param_fine());
//    SERIAL_PRINT("\tMin", ultrasonicSensorData.param_min());
//    SERIAL_PRINT("\tMax", ultrasonicSensorData.param_max());
  
    byte p[1024] = {};
    KMS::io.read(0, p, 1024);
    Serial.println("READ EEPROM");
    for(int r=0; r < 1024; r++){
      Serial.print(*(p+r));Serial.print(" ");
      if (!(r%16)) Serial.println();
    }
  
  
//	for(int i = 0; i<32; i++) {
//      KMS::InputNorm inputData = KMS::input(i);
//			SERIAL_PRINT("Input ", i);
//      SERIAL_PRINT("\tAnalog", inputData.AD());
//      SERIAL_PRINT("\tMode", inputData.mode());
//      SERIAL_PRINT("\tChannel", inputData.channel());
//      SERIAL_PRINT("\tParam fine (nrpn)", inputData.param_fine());
//      SERIAL_PRINT("\tParam coarse", inputData.param_coarse());
//      SERIAL_PRINT("\tParam full NRPN", (inputData.param_coarse() << 7) | inputData.param_fine());
//      SERIAL_PRINT("\tMin", inputData.param_min());
//      SERIAL_PRINT("\tMax", inputData.param_max());
//		}
//		for(int i = 0; i<NUM_OUTPUTS; i++) {
//            printOutput("Output", KMS::outputs[i], i);
//		}
	
}

void loop() {
}


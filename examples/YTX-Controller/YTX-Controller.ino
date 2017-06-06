/*
   Autor: Franco Grassano - YAELTEX
   Version: 0.9.1
   ---
   INFORMACIÓN DE LICENCIA
   Kilo Mux Shield por Yaeltex se distribuye bajo una licencia
   Creative Commons Atribución-CompartirIgual 4.0 Internacional - http://creativecommons.org/licenses/by-sa/4.0/
   ----
   Código para el manejo de los integrados 74HC595 tomado de http://bildr.org/2011/02/74HC595/
   Librería de multiplexado (modificada) tomada de http://mayhewlabs.com/products/mux-shield-2
   Librería para el manejo del sensor de ultrasonido tomada de http://playground.arduino.cc/Code/NewPing

   Este código fue desarrollado para el KILO MUX SHIELD desarrolado en conjunto por Yaeltex y el Laboratorio del Juguete, en Buenos Aires, Argentina,
   apuntando al desarrollo de controladores MIDI con Arduino.
   Está preparado para manejar 2 (expandible) registros de desplazamiento 74HC595 conectados en cadena (16 salidas digitales en total),
   y 2 multiplexores CD4067 de 16 canales cada uno (16 entradas analógicas, y 16 entradas digitales), pero es expandible en el
   caso de utilizar hardware diferente. Para ello se modifican los "define" NUM_MUX, NUM_CANALES_MUX y NUM_595s.
   NOTA: Se modificó la librería MuxShield, para trabajar sólo con 1 o 2 multiplexores. Si se necesita usar más multiplexores, descargar la librería original.

   Para las entradas analógicas, por cuestiones de ruido se recomienda usar potenciómetros o sensores con buena estabilidad, y con preferencia con valores
   cercanos o menores a 10 Kohm.
   Agradecimientos:
   - Jorge Crowe
   - Lucas Leal
   - Dimitri Diakopoulos
*/

/*
   Inclusión de librerías.
*/
#include <KM_Data.h>
#include <Kilomux.h>
#include <NewPing.h>
#include <MIDI.h>

void setup(); // Esto es para solucionar el bug que tiene Arduino al usar los #ifdef del preprocesador

#define MIDI_COMMS

#if defined(MIDI_COMMS)
struct MySettings : public midi::DefaultSettings
{
  static const unsigned SysExMaxSize = 64; // Accept SysEx messages up to 1024 bytes long.
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create a 'MIDI' object using MySettings bound to Serial.
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MySettings);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

#define STATUS_BLINK_INTERVAL 100
#define OUTPUT_BLINK_INTERVAL 300

#define MAX_BANKS     8

static const char * const modeLabels[] = {
  "Off"
  , "Note"
  , "CC"
  , "NRPN"
  , "Program Change"
  , "Shifter"
};
#define MODE_LABEL(m)   ((m) <= KMS::M_SHIFTER ? modeLabels[m] : "???")

// SysEx commands
#define CONFIG_MODE       1    // PC->hw : Activate monitor mode
#define CONFIG_ACK        2    // HW->pc : Acknowledge the config mode
#define DUMP_TO_HW        3    // PC->hw : Partial EEPROM dump from PC
#define DUMP_OK           4    // HW->pc : Ack from dump properly saved
#define EXIT_CONFIG       5    // HW->pc : Deactivate monitor mode
#define EXIT_CONFIG_ACK   6    // HW->pc : Ack from exit config mode

#define CONFIG_OFF 0
#define CONFIG_ON 1

#define ANALOG_UP     1
#define ANALOG_DOWN   0

// Ultrasonic sensor defines
#define US_SENSOR_FILTER_SIZE  3         // Cantidad de valores almacenados para el filtro. Cuanto más grande, mejor el suavizado y  más lenta la lectura.

// Global data in EEPROM containing general config of inputs, outputs, US sensor, LED mode
KMS::GlobalData gD = KMS::globalData();

// Ultrasonic sensor variables
KMS::InputUS ultrasonicSensorData = KMS::ultrasound();
unsigned long minMicroSecSensor = 0;
unsigned long maxMicroSecSensor = 0;
byte pingSensorInterval = 25; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingSensorTimer;     // Holds the next ping time.
bool sensorActive = 0;                  // Inicializo inactivo (variable interna)
uint16_t usSensorPrevValue;                       // Array para el filtrado de la lectura del sensor
uint16_t uSeg = 0;                                // Contador de microsegundos para el ping del sensor.

// Running average filter variables
uint8_t filterIndex = 0;            // Indice que recorre los valores del filtro de suavizado
uint8_t filterCount = 0;
uint16_t filterSum = 0;
uint16_t filterSamples[US_SENSOR_FILTER_SIZE];

Kilomux KMShield;                                       // Objeto de la clase Kilomux
NewPing usSensor(SensorTriggerPin, SensorEchoPin, MAX_SENSOR_DISTANCE); // Instancia de NewPing para el sensor US.

// Digital input and output states
bool digitalInputState[MAX_BANKS][NUM_MUX * NUM_MUX_CHANNELS]; // Estado de los botones - si no se usan entradas digitales, se pueden achicar estos arrays
byte digitalOutState[MAX_BANKS][NUM_MUX * NUM_MUX_CHANNELS]; // Estado de los botones
byte currentProgram[MAX_BANKS][16] = {};

// Contadores, flags //////////////////////////////////////////////////////////////////////
byte mux, muxChannel;                       // Contadores para recorrer los multiplexores
byte numBanks, numInputs, numOutputs;
byte prevBank, currBank = 0;
bool ledModeMatrix, newBank, changeDigOutState, ultrasoundPresent, midiThru;
bool configurationValid = true, firstRead = true;
bool flagBlinkStatusLED = 0, configMode = 0, receivingSysEx = 0;
bool outputBlinkState = 0;
unsigned long millisPrevLED = 0;               // Variable para guardar ms
static byte blinkCountStatusLED = 0;
uint16_t dataPacketSize;
unsigned long prevBlinkMillis = 0;
///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
#if defined(MIDI_COMMS)
  MIDI.begin(MIDI_CHANNEL_OMNI); // Se inicializa la comunicación MIDI.
  MIDI.turnThruOff();            // Por default, la librería de Arduino MIDI tiene el THRU en ON, y NO QUEREMOS ESO!
#else
  Serial.begin(250000);                  // Se inicializa la comunicación serial.
#endif

  // Initialize Kilomux shield
  KMShield.init();
  KMShield.setADCprescaler(PS_16);    // Override initial setting
  // Initialize EEPROM handling library
  KMS::initialize();
  
  prevBlinkMillis = millis();

#if !defined(MIDI_COMMS)
  Serial.print("Kilowhat protocol: v");
  Serial.println(gD.protocolVersion());
#endif
      
  ResetConfig(CONFIG_OFF);
}

/*
    Loop principal.
    Lee MIDI o Serial.
    Si la configuración en EEPROM es válida (configuración por Kilowhat)
      Actualiza el LED de estado.
      Actualiza las salidas del Kilomux.
      Lee el sensor de ultrasonido.
      Lee las entradas del Kilomux y envía mensajes MIDI si corresponde.
    Si la configuración en EEPROM no es válida
      LED de estado intermitente cada medio segundo.
*/
void loop() {
   //unsigned long antMicrosLoop = micros();
#if defined(MIDI_COMMS)
  if (MIDI.read()){
    ReadMidi();
  }
#else
  if (Serial.available()){
    ReadSerial();
  }
#endif

  if (configurationValid) {
    if (flagBlinkStatusLED && blinkCountStatusLED) blinkStatusLED();
    
    if (!receivingSysEx){
      UpdateDigitalOutputs();
      if (millis() - prevBlinkMillis > OUTPUT_BLINK_INTERVAL) {         // Si transcurrieron más de X ms desde la ultima actualización,
        ToggleBlinkOutputs();
      }
      //unsigned long antMicrosLoop = micros();
      if (ultrasoundPresent) ReadSensorUS();
      ReadInputs(); 
      //Serial.println(micros()-antMicrosLoop);
    }
  }
  else{
    if(!(millis() % 500)){
      flagBlinkStatusLED = 1;
      blinkCountStatusLED = 1;
    }
    if (flagBlinkStatusLED && blinkCountStatusLED) blinkStatusLED();
    ReadInputs(); 
  } 
}

/*
    newConfig:  0 - Reset en modo normal.
                1 - Reset em modo de configuración.
*/
void ResetConfig(bool newConfig) {
  pinMode(LED_BUILTIN, OUTPUT);
  // Reload global data from EEPROM
  gD = KMS::globalData();
  
  currBank = 0;
  KMS::setBank(currBank);
  configMode = newConfig;
  firstRead = true;

  if (gD.isValid()) {
    if (gD.protocolVersion() == 1){
      dataPacketSize = 57; // 64 SysEx packets. 7 Header bytes - 57 data bytes   
    }else{
      dataPacketSize = 57; // Default.
    }
   
    ledModeMatrix = gD.hasOutputMatrix();
    numBanks = gD.numBanks();
    numInputs = gD.numInputsNorm();
    numOutputs = gD.numOutputs();
    ultrasonicSensorData = KMS::ultrasound();
    ultrasoundPresent = ultrasonicSensorData.mode() != KMS::M_OFF;
    midiThru = gD.midiThru();

    #if defined(MIDI_COMMS)
    if (midiThru)
      MIDI.turnThruOn();             // Turn midi thru on, because configuration says so
    else
      MIDI.turnThruOff();            // Turn midi thru off, because configuration says so
    #endif
    
     #if !defined(MIDI_COMMS)
    Serial.print("Numero de bancos: "); Serial.println(numBanks);
    Serial.print("Numero de entradas: "); Serial.println(numInputs);
    Serial.print("Numero de salidas: "); Serial.println(numOutputs);
    Serial.print("MIDI thru? "); Serial.println(midiThru ? "SI" : "NO");
    Serial.print("Sensor mode: "); Serial.println(MODE_LABEL(ultrasonicSensorData.mode()));
    #endif
    
    if (ultrasoundPresent) {
      FilterClear();
      minMicroSecSensor = ultrasonicSensorData.dist_min() * US_ROUNDTRIP_CM;
      maxMicroSecSensor = ultrasonicSensorData.dist_max() * US_ROUNDTRIP_CM;
      usSensor.changeMaxDistance(ultrasonicSensorData.dist_max());
      sensorActive = false;
      pingSensorTimer = millis() + pingSensorInterval;
      pinMode(SensorEchoPin, INPUT_PULLUP);
      pinMode(SensorTriggerPin, OUTPUT);
      pinMode(ActivateSensorButtonPin, INPUT_PULLUP);
      pinMode(ActivateSensorLedPin, OUTPUT);
      digitalWrite(ActivateSensorLedPin, LOW);
    }
  }
  else {
    configurationValid = false;
    dataPacketSize = 57;
    #if !defined(MIDI_COMMS)
      Serial.print("Datos en EEPROM no válidos");
    #endif
  }
  
  // Inicializar lecturas
  for (mux = 0; mux < NUM_MUX; mux++) {
    for (muxChannel = 0; muxChannel < NUM_MUX_CHANNELS; muxChannel++) {
      KMShield.muxReadings[mux][muxChannel] = KMShield.analogReadKm(mux, muxChannel);
      KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];
    }
  }
  for (byte bank = 0; bank < MAX_BANKS; bank++) {
    for (byte i = 0; i < NUM_MUX * NUM_MUX_CHANNELS; i++) {
      digitalInputState[bank][i] = 0;
      digitalOutState[bank][i] = 0;
    }
    for (byte i = 0; i < 16; i++) {
      currentProgram[bank][i] = 0;
    }
  }
  changeDigOutState = true;
}

/*
    Esta función es llamada por el loop principal, cuando las variables flagBlinkStatusLED y blinkCountStatusLED son distintas de cero.
    blinkCountStatusLED tiene la cantidad de veces que debe titilar el LED.
    El intervalo es fijo y dado por la etiqueta 'STATUS_BLINK_INTERVAL'
*/ 
void blinkStatusLED() {
  static bool lastLEDState = LOW;
  static unsigned long millisPrev = 0;
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    millisPrev = millis();
  }
  if (millis() - millisPrev > STATUS_BLINK_INTERVAL) {
    millisPrev = millis();
    digitalWrite(13, !lastLEDState);
    lastLEDState = !lastLEDState;
    if (lastLEDState == LOW) blinkCountStatusLED--;
    if (!blinkCountStatusLED) {
      flagBlinkStatusLED = 0;
      firstTime = true;
    }
  }

  return;
}

/*
    Si se los flags 'newBank' o 'changeDigOutState' se marcaron, se actualizan los estados de las salidas digitales.
*/ 
void UpdateDigitalOutputs() {
  uint16_t dOut = 0;
  if (newBank || changeDigOutState) {
    newBank = false; changeDigOutState = false;

    if (!ledModeMatrix) {
      for (dOut = 0; dOut < numOutputs; dOut++) {
        KMS::Output outputData = KMS::output(dOut);
        if (!outputData.shifter() || configMode) {
          if (digitalOutState[currBank][dOut] == OUT_ON)
            KMShield.digitalWriteKm(dOut, HIGH);
          else if (digitalOutState[currBank][dOut] == OUT_OFF)
            KMShield.digitalWriteKm(dOut, LOW);
        } else{
          if (outputData.param() == currBank)
            KMShield.digitalWriteKm(dOut, HIGH);
          else
            KMShield.digitalWriteKm(dOut, LOW);
        }
      }
    } 
    else{
      // Próximamente acá estará el código para manejar salidas en forma matricial, y así controlar hasta 64 salidas!
    }
  }
}

/*
    Actualiza el estado de las salidas intermitentes.
    Se actualizan todas las salidas al mismo tiempo.
    La intermitencia tiene un intervalo fijo, dado por la etiqueta 'OUTPUT_BLINK_INTERVAL' 
*/ 
void ToggleBlinkOutputs(void) {
  uint16_t dOut = 0;
  for (dOut = 0; dOut < numOutputs; dOut++) {                   // Recorrer todos los LEDs
    if (digitalOutState[currBank][dOut] == OUT_BLINK) {     // Si corresponde titilar este LED,
      if (outputBlinkState) {                                   // Y estaba encendido,
        KMShield.digitalWriteKm(dOut, HIGH);                               // Se apaga
      }
      else {                                           // Si estaba apagado,
        KMShield.digitalWriteKm(dOut, LOW);                              // Se enciende
      }
    }
  }
  prevBlinkMillis = millis();                     // Actualizo el contador de ms usado en la comparación
  outputBlinkState = !outputBlinkState;
}

#if defined(MIDI_COMMS)
/*
    Esta función se encarga de recibir mensajes MIDI.
    Notes ->  Actualizar salidas
    SysEx ->  Entrada y salida del modo de configuración
              Nueva configuración en la EEPROM.
*/  
void ReadMidi(void) {
  switch (MIDI.getType()) {
    case midi::NoteOn:
      HandleNotes(); break;
    case midi::NoteOff:
      HandleNotes(); break;
    case midi::SystemExclusive:
      int sysexLength = 0;
      const byte *pMsg;

      sysexLength = (int) MIDI.getSysExArrayLength();
      pMsg = MIDI.getSysExArray();

      char sysexID[3];
      sysexID[0] = (char) pMsg[1];
      sysexID[1] = (char) pMsg[2];
      sysexID[2] = (char) pMsg[3];

      char command = pMsg[4];

      if (sysexID[0] == 'Y' && sysexID[1] == 'T' && sysexID[2] == 'X') {

        if (command == CONFIG_MODE && !configMode) {           // Enter config mode
          //MIDI.turnThruOff();
          flagBlinkStatusLED = 1;
          blinkCountStatusLED = 1;
          const byte configAckSysExMsg[5] = {'Y', 'T', 'X', CONFIG_ACK, 0};
          MIDI.sendSysEx(5, configAckSysExMsg, false);
          ResetConfig(CONFIG_ON);
        }
        else if (command == EXIT_CONFIG && configMode) {       // Enter config mode
          flagBlinkStatusLED = 1;
          blinkCountStatusLED = 1;
          const byte configAckSysExMsg[5] = {'Y', 'T', 'X', EXIT_CONFIG_ACK, 0};
          MIDI.sendSysEx(5, configAckSysExMsg, false);
          //MIDI.turnThruOn();
          ResetConfig(CONFIG_OFF);
        }
        else if (command == DUMP_TO_HW) {           // Save dump data
          if (!receivingSysEx) {
            receivingSysEx = 1;
            MIDI.turnThruOff();
          }

          KMS::io.write(dataPacketSize * pMsg[5], pMsg + 6, sysexLength - 7); // pMsg has index in byte 6, total sysex packet has max.
          // |F0, 'Y' , 'T' , 'X', command, index, F7|
          flagBlinkStatusLED = 1;
          blinkCountStatusLED = 1;

          if (sysexLength < dataPacketSize + 7) { // Last message?
            receivingSysEx = 0;
            flagBlinkStatusLED = 1;
            blinkCountStatusLED = 3;
            const byte dumpOkMsg[5] = {'Y', 'T', 'X', DUMP_OK, 0};
            MIDI.sendSysEx(5, dumpOkMsg, false);
            const byte configAckSysExMsg[5] = {'Y', 'T', 'X', EXIT_CONFIG_ACK, 0};
            MIDI.sendSysEx(5, configAckSysExMsg, false);
            //MIDI.turnThruOn();
            ResetConfig(CONFIG_OFF);
          }
        }
      }
      break;
  }
}
#else
/*
    Usada para poder acceder al modo de configuración a través del terminal Serial.
    Con la tecla 'c' se accede al modo de configuración.
    Con la tecla 'x' se sale del modo de configuración.
*/
void ReadSerial(){
  char inChar = (char) Serial.read();
  if (inChar == 'c' && !configMode) {
    //configMode = true;
    ResetConfig(CONFIG_ON);
  }
  else if (inChar == 'x' && configMode) {
    //configMode = false;
    ResetConfig(CONFIG_OFF);
  }
}
#endif

/*
    Al recibir un mensaje de tipo NOTE, se determina si la nota corresponde a alguna salida del banco actual.
    En ese caso, se actualiza el estado de la misma y se enciente el flag para que la función 'UpdateDigitalOutputs()'
    la encienda o lo apague.
*/
#if defined(MIDI_COMMS)
void HandleNotes() {
  byte data1, data2, channel;
  data1 = MIDI.getData1();
  data2 = MIDI.getData2();
  channel = MIDI.getChannel();    // Channel 1-16
  if (!configMode) {
    for (byte outputIndex = 0; outputIndex < numOutputs; outputIndex++) {
      KMS::Output outputData = KMS::output(outputIndex);
      if (data1 == outputData.param() && channel == outputData.channel()) {
        if (MIDI.getType() == midi::NoteOn) {
          if (outputData.blink() && data2 >= outputData.blink_min() && data2 <= outputData.blink_max()) {
            digitalOutState[currBank][outputIndex] = OUT_BLINK;
          } else if (outputData.blink() && data2 && (data2 < outputData.blink_min() || data2 > outputData.blink_max())) {
            digitalOutState[currBank][outputIndex] = OUT_ON;
          } else if (outputData.blink() && !data2) {
            digitalOutState[currBank][outputIndex] = OUT_OFF;
          } else {       // blink off
            if (data2) {
              digitalOutState[currBank][outputIndex] = OUT_ON;
            } else {
              digitalOutState[currBank][outputIndex] = OUT_OFF;
            }
          }
        } else if (MIDI.getType() == midi::NoteOff) {
          if (!data2) {
            digitalOutState[currBank][outputIndex] = OUT_OFF;
          }
        }
        changeDigOutState = true;
        break;
      }
    }
  }
  else {
    if (data2)
      KMShield.digitalWriteKm(data1, HIGH);
    else
      KMShield.digitalWriteKm(data1, LOW);
  }
}
#endif

/*
    Verifica si el sensor de ultrasonido está activado y maneja el LED indicador.
    Si está activado, la librería NewPing envía un pulso de ultrasonido en forma periódica 
    y espera recibir uno de vuelta, al rebotar en un objeto.
    Se calcula luego el tiempo que tarda en ir y volver y con ello se determina la distancia al objeto.
    Diagrama de conexión del sensor de ultrasonido: http://wiki.yaeltex.com.ar/index.php?title=Kilomux_Shield#Sensor_de_Ultrasonido_.28J1.29
    
*/
void ReadSensorUS() {
  static uint16_t prevMillisUltraSensor = 0;    // Variable usada para almacenar los milisegundos desde el último Ping al sensor
  static bool activateSensorButtonPrevState = HIGH;                // Inicializo inactivo (entrada activa baja)
  static bool activateSensorButtonState = HIGH;                   // Inicializo inactivo (entrada activa baja)
  static bool sensorLEDState = LOW;                       // Inicializo inactivo (salida activa alta)

  // Este codigo verifica si se presionó el botón y activa o desactiva el sensor cada vez que se presiona
  activateSensorButtonState = digitalRead(ActivateSensorButtonPin);
  if (activateSensorButtonState == LOW && activateSensorButtonPrevState == HIGH) {  // Si el botón previamente estaba en estado alto, y ahora esta en estado bajo, quiere decir que paso de estar no presionado a presionado (activo bajo)
    activateSensorButtonPrevState = LOW;                                // Actualizo el estado previo
    sensorActive = !sensorActive;                      // Activo o desactivo el sensor
    sensorLEDState = !sensorLEDState;                                // Cambio el estado del LED
    digitalWrite(ActivateSensorLedPin, sensorLEDState);               // Y actualizo la salida
  }
  else if (activateSensorButtonState == HIGH && activateSensorButtonPrevState == LOW) { // Si el botón previamente estaba en estado bajo, y ahora esta en estado alto, quiere decir que paso de estar presionado a no presionado
    activateSensorButtonPrevState = HIGH;                                  // Actualizo el estado previo
  }
  
  if(firstRead){
    sensorLEDState = LOW; 
    activateSensorButtonPrevState = HIGH;
    activateSensorButtonState = HIGH;
    return;
  }
  
  if (sensorActive) {                                     // Si el sensor está activado
    if (millis() >= pingSensorTimer) {                      // y transcurrió el delay minimo entre lecturas
      pingSensorTimer += pingSensorInterval;
      usSensor.ping_timer(EchoCheck);                       // Sensar el tiempo que tarda el pulso de ultrasonido en volver. Se recibe el valor el us.
    }
  }else
    pingSensorTimer = millis() + pingSensorInterval;
  
}

/*
    Esta función se llama cada un intervalo definido por la variable 'pingSensorInterval'.
    Verifica la lectura de un pulso de ultrasonido, y en caso que exista, formatea el 
    mensaje MIDI a enviar, según la configuración existente en la EEPROM.
*/
void EchoCheck(){
  uint8_t rc = usSensor.check_timer();        
  if(rc != 0){                            // el resultado es distinto de 0 si hay un pulso
    uSeg = usSensor.ping_result;          // me traigo el tiempo en microsegundos de retorno del pulso
    uSeg = constrain(uSeg, minMicroSecSensor, maxMicroSecSensor);     // lo limito dentro del rango posible segun la distancia configurada  
    uint16_t sensorRange = maxMicroSecSensor - minMicroSecSensor;     
    uint16_t usSensorValue = map(uSeg, minMicroSecSensor, maxMicroSecSensor+1, 0, sensorRange+1);  
    uint16_t minMidiNRPN, maxMidiNRPN;
    byte mode = ultrasonicSensorData.mode();
    byte midiChannel = ultrasonicSensorData.channel();
    byte param = ultrasonicSensorData.param_fine();
    byte minMidi = ultrasonicSensorData.param_min();
    byte maxMidi = ultrasonicSensorData.param_max();

    if (mode == KMS::M_NRPN){
      minMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + minMidi);
      maxMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + maxMidi);
      usSensorValue = map(usSensorValue, 0, mode == KMS::M_NRPN ? 1024 : 128, minMidiNRPN, maxMidiNRPN+1);
    }else{
      usSensorValue = map(usSensorValue, 0, 128, minMidi, maxMidi+1); 
    }
    
    // FILTRO DE MEDIA MÓVIL PARA SUAVIZAR LA LECTURA
    usSensorValue = FilterGetNewAverage(usSensorValue);
    
    // Detecto si cambió el valor filtrado, para no mandar valores repetidos
    if (usSensorValue != usSensorPrevValue) {
      usSensorPrevValue = usSensorValue;
      #if defined(MIDI_COMMS)
      if(configMode){
        MIDI.sendControlChange(0, usSensorValue, 15);
      }
      else{
        switch (mode) {
          case (KMS::M_NOTE):
            MIDI.sendNoteOn(param, usSensorValue, midiChannel); break;
          case (KMS::M_CC):
            MIDI.sendControlChange(param, usSensorValue, midiChannel); break;
          case KMS::M_NRPN:
            MIDI.sendControlChange( 101, ultrasonicSensorData.param_coarse(), midiChannel);
            MIDI.sendControlChange( 100, ultrasonicSensorData.param_fine(), midiChannel);
            MIDI.sendControlChange( 6, (usSensorValue >> 7) & 0x7F, midiChannel);
            MIDI.sendControlChange( 38, (usSensorValue & 0x7F), midiChannel); break;
          default: break;
        }
      }
      #else
        Serial.print("Channel: "); Serial.print(midiChannel); Serial.print("\t");
        Serial.print("Modo: "); Serial.print(MODE_LABEL(mode)); Serial.print("\t");
        Serial.print("Parameter: "); Serial.print((ultrasonicSensorData.param_coarse() << 7) | ultrasonicSensorData.param_fine()); Serial.print("  Valor: "); Serial.println(usSensorValue);
      #endif
    }
  }
}

/*
    Filtro de media móvil para el sensor de ultrasonido (librería RunningAverage integrada) (http://playground.arduino.cc/Main/RunningAverage)
*/
uint16_t FilterGetNewAverage(uint16_t newVal){
  filterSum -= filterSamples[filterIndex];
  filterSamples[filterIndex] = newVal;
  filterSum += filterSamples[filterIndex];
  filterIndex++;
  if (filterIndex == US_SENSOR_FILTER_SIZE) filterIndex = 0;  // faster than %
  // update count as last otherwise if( _cnt == 0) above will fail
  if (filterCount < US_SENSOR_FILTER_SIZE) 
    filterCount++;
  if (filterCount == 0) 
    return NAN;
  return filterSum / filterCount;
}

/*
   Limpia los valores del filtro de media móvil para un nuevo uso.
*/
void FilterClear(){
  filterCount = 0;
  filterIndex = 0;
  filterSum = 0;
  for (uint8_t i = 0; i < US_SENSOR_FILTER_SIZE; i++){
    filterSamples[i] = 0; // keeps addValue simpler
  }
}

/*
   Esta función lee todas las entradas análógicas y/o digitales y almacena los valores de cada una en el array de 'lecturas'.
   Compara con los valores previos, almacenados en 'lecturasPrev', y si cambian, y llama a la función InputChanged().
*/
void ReadInputs() {
  static uint16_t currAnalogValue = 0, prevAnalogValue = 0;
  for (int inputIndex = 0; inputIndex < numInputs; inputIndex++) {
    KMS::InputNorm inputData = KMS::input(inputIndex);
    mux = inputIndex < 16 ? MUX_A : MUX_B;           // MUX 0 or 1
    muxChannel = inputIndex % NUM_MUX_CHANNELS;         // CHANNEL 0-15
    if (inputData.mode() != KMS::M_OFF) {
      if (inputData.mode() == KMS::M_SHIFTER && !configMode) {
        // CÓDIGO PARA LECTURA DE SHIFTERS
        KMShield.muxReadings[mux][muxChannel] = KMShield.digitalReadKm(mux, muxChannel, PULLUP);      // Leer entradas digitales 'KMShield.digitalReadKm(N_MUX, N_CANAL)'
        //Serial.print("Mux: "); Serial.print(mux); Serial.print("  Channel: "); Serial.println(channel);
        if (KMShield.muxReadings[mux][muxChannel] != KMShield.muxPrevReadings[mux][muxChannel]) {     // Me interesa la lectura, si cambió el estado del botón,
          KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];          // Almacenar lectura actual como anterior, para el próximo ciclo
          if (firstRead) continue;                                                              // Esto es  para evitar que al resetear se cambie el banco
          byte param = inputData.param_fine();
          byte buttonState = !KMShield.muxReadings[mux][muxChannel];
          static bool isShifterToggle;
          static bool bankButtonPressed;
          currBank = KMS::bank();
          if (buttonState && currBank != param && param <= KMS::realBanks() && !bankButtonPressed) {
            isShifterToggle = inputData.toggle();
            prevBank = currBank;
            KMS::setBank(param);
            currBank = param;
            bankButtonPressed = true;
            newBank = true;
            #if !defined(MIDI_COMMS)
            Serial.println("");
            Serial.print("Current Bank: "); Serial.print(KMS::bank());
            Serial.print("\t   Previous bank: "); Serial.print(prevBank);
            Serial.print("\tToggle: "); Serial.println(isShifterToggle ? "YES" : "NO"); Serial.println("");
            #endif
          }
          else if (!buttonState && !isShifterToggle && param == currBank && bankButtonPressed) {  // button not activated and shifter is momentary
            KMS::setBank(prevBank);                             // reset bank to previous
            prevBank = param;
            currBank = KMS::bank();
            bankButtonPressed = false;
            newBank = true;
            #if !defined(MIDI_COMMS)
            Serial.println("");
            Serial.print("Bank: "); Serial.print(KMS::bank());
            Serial.print("\tPrevious bank: "); Serial.print(prevBank);
            Serial.print("\tBank button mode: "); Serial.println(isShifterToggle); Serial.println("");
            #endif
          }
          else if (!buttonState && isShifterToggle && param == currBank && bankButtonPressed) {
            bankButtonPressed = false;
          }
        }
      }
      else if (inputData.AD() == KMS::T_ANALOG) {
        KMShield.muxReadings[mux][muxChannel] = KMShield.analogReadKm(mux, muxChannel);           // Si es NRPN leer entradas analógicas 'KMShield.analogReadKm(N_MUX,N_CANAL)'
        
        if (!firstRead && KMShield.muxReadings[mux][muxChannel] != KMShield.muxPrevReadings[mux][muxChannel]) {  // Si leo algo distinto a lo anterior
          // Enviar mensaje.
          InputChanged(inputIndex, inputData, KMShield.muxReadings[mux][muxChannel]);
        }
        else if (firstRead) {
          KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];         // Almacenar lectura actual como anterior, para el próximo ciclo
          continue;
        }
        else {
          continue;                                                                          // Sigo con la próxima lectura
        }
        KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];         // Almacenar lectura actual como anterior, para el próximo ciclo
      }

      else if (inputData.AD() == KMS::T_DIGITAL) {
        // CÓDIGO PARA LECTURA DE ENTRADAS DIGITALES Y SHIFTERS
        KMShield.muxReadings[mux][muxChannel] = KMShield.digitalReadKm(mux, muxChannel, PULLUP);      // Leer entradas digitales 'KMShield.digitalReadKm(N_MUX, N_CANAL)'
        bool toggle = inputData.toggle();
        //Serial.print("Mux: "); Serial.print(mux); Serial.print("  Channel: "); Serial.println(channel);
        if (KMShield.muxReadings[mux][muxChannel] != KMShield.muxPrevReadings[mux][muxChannel]) {     // Me interesa la lectura, si cambió el estado del botón,
          KMShield.muxPrevReadings[mux][muxChannel] = KMShield.muxReadings[mux][muxChannel];             // Almacenar lectura actual como anterior, para el próximo ciclo
          if (firstRead) continue;
          if (!KMShield.muxReadings[mux][muxChannel]) {
            digitalInputState[currBank][muxChannel + mux * NUM_MUX_CHANNELS] = !digitalInputState[currBank][muxChannel + mux * NUM_MUX_CHANNELS]; // MODO TOGGLE: Cambia de 0 a 1, o viceversa
            // MODO NORMAL: Cambia de 0 a 1
            InputChanged(inputIndex, inputData, !digitalInputState[currBank][muxChannel + mux * NUM_MUX_CHANNELS]);
          }
          else if (KMShield.muxReadings[mux][muxChannel] && !toggle) {
            digitalInputState[currBank][muxChannel + mux * NUM_MUX_CHANNELS] = 0;
            InputChanged(inputIndex, inputData, !digitalInputState[currBank][muxChannel + mux * NUM_MUX_CHANNELS]);
          }
        }
      }
    }
  }
  firstRead = false;
}

/*
   Esta función se encarga de analizar si el cambio en la entrada es ruido o es un cambio válido.
   Luego se encarga de darle formato al mensaje MIDI a enviar, según la configuración en la EEPROM.
*/
void InputChanged(int numInput, const KMS::InputNorm &inputData, uint16_t rawValue) {
  byte mode = inputData.mode();
  bool analog = inputData.AD();       // 1 is analog
  byte param = inputData.param_fine();
  byte channel = inputData.channel();
  byte minMidi = inputData.param_min();
  byte maxMidi = inputData.param_max();
  uint16_t minMidiNRPN, maxMidiNRPN;
  uint16_t mapValue, constrainedValue;
  uint16_t noiseTh;
  static uint16_t prevValue[NUM_MUX * NUM_MUX_CHANNELS] = {};
  static uint16_t prevRawValue[NUM_MUX * NUM_MUX_CHANNELS] = {};

  #define CONST_LOW_LIMIT   2
  #define CONST_HIGH_LIMIT  1021
                                                                                       
  if (!configMode) {
    if (analog) {
      constrainedValue = constrain(rawValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT);
      if (IsNoise(constrainedValue, prevRawValue[numInput], numInput, 2, true)) 
        return;                                                           
      prevRawValue[numInput] = constrainedValue;  
      //Serial.print("Raw value: "); Serial.print(constrainedValue); Serial.println();
      if (mode == KMS::M_NRPN) {
        minMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + minMidi);
        maxMidiNRPN = pgm_read_word_near(KMS::nrpn_min_max + maxMidi);

        int maxMinDiff = maxMidiNRPN - minMidiNRPN;
        byte maxMidiNRPNadd = abs(maxMinDiff) >> 10;

        mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, minMidiNRPN, maxMidiNRPN); // map() only maps correctly if fromHigh and toHigh are +1 the actual mapped values

        noiseTh = abs(maxMinDiff) >> 8;          // divide range to get noise threshold. Max th is 127/4 = 64 : Min th is 0.
        
        if(maxMinDiff < 1023){
          if (IsNoise(mapValue, prevValue[numInput], numInput, noiseTh, false))
            return;
        }
      } else {
        mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, minMidi, maxMidi);
        int maxMinDiff = maxMidi - minMidi;
        noiseTh = abs(maxMinDiff) >> 6;          // divide range to get noise threshold. Max th is 127/64 = 2 : Min th is 0.
        if (IsNoise(mapValue, prevValue[numInput], numInput, 0, false))
          return;
      }
      prevValue[numInput] = mapValue;   // Save value to previous data array
    }
    else {      // DIGITAL INPUTS
      if (rawValue)  mapValue = minMidi;   // If value is != 0, then button is off
      else           mapValue = maxMidi;   // If value is == 0, the button is on (active LOW)
    }
  }
  
#if defined(MIDI_COMMS)
  if (configMode) { // CONFIG MODE MESSAGES
    if(analog){
      constrainedValue = constrain(rawValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT);
      if (IsNoise(constrainedValue, prevRawValue[numInput], numInput, 2, true)) 
        return;                                                           
      prevRawValue[numInput] = constrainedValue;
      mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, 0, 127);
      if (IsNoise(mapValue, prevValue[numInput], numInput, 0, false))
        return;
      prevValue[numInput] = mapValue;   // Save value to previous data array
    }else {      // DIGITAL INPUTS
      if (rawValue)  mapValue = 0;   // If value is != 0, then button is off
      else           mapValue = 127;   // If value is == 0, the button is on (active LOW)
    }
    
    MIDI.sendControlChange( numInput, mapValue, 1);
  }
  else {    // CONFIG MODE MESSAGES - ONLY CC FOR ANALOG INPUTS AND NOTES FOR DIGITAL INPUTS
    switch (mode) {
      case (KMS::M_NOTE):
        MIDI.sendNoteOn(param, mapValue, channel); break;
      case (KMS::M_CC):
        MIDI.sendControlChange(param, mapValue, channel); break;
      case KMS::M_NRPN:
        MIDI.sendControlChange( 101, inputData.param_coarse(), channel);
        MIDI.sendControlChange( 100, inputData.param_fine(), channel);
        MIDI.sendControlChange( 6, (mapValue >> 7) & 0x7F, channel);
        MIDI.sendControlChange( 38, (mapValue & 0x7F), channel); break;
      case KMS::M_PROGRAM_MINUS:
        if(currentProgram[currBank][channel-1] > 0){
          currentProgram[currBank][channel-1]--;
          MIDI.sendProgramChange(currentProgram[currBank][channel-1], channel); 
        } break;
      case KMS::M_PROGRAM:
        if (!analog && rawValue > 0) {
          MIDI.sendProgramChange( param, channel);
        } 
        else if (analog){
          MIDI.sendProgramChange( mapValue, channel);
        } break;
      case KMS::M_PROGRAM_PLUS:
        if(currentProgram[currBank][channel-1] < 127){
          currentProgram[currBank][channel-1]++;
          MIDI.sendProgramChange(currentProgram[currBank][channel-1], channel);
        } break;
      default: break;
    }
  }
#else    
  if (configMode) { // CONFIG MODE MESSAGES
    if(analog){
      constrainedValue = constrain(rawValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT);
      if (IsNoise(constrainedValue, prevRawValue[numInput], numInput, 2, true)) 
        return;                                                           
      prevRawValue[numInput] = constrainedValue;
      mapValue = map(constrainedValue, CONST_LOW_LIMIT, CONST_HIGH_LIMIT, 0, 127);    
      if (IsNoise(mapValue, prevValue[numInput], numInput, 0, false))
        return;
    }else {      // DIGITAL INPUTS
      if (rawValue)  mapValue = 0;   // If value is != 0, then button is off
      else           mapValue = 127;   // If value is == 0, the button is on (active LOW)
    }
  }
  Serial.print("Channel: "); Serial.print(channel);
  Serial.print("    Tipo: "); Serial.print(inputData.AD() ? "Analog" : "Digital");
  Serial.print("    Min: "); Serial.print(mode == KMS::M_NRPN ? minMidiNRPN : minMidi);
  Serial.print("    Max: "); Serial.print(mode == KMS::M_NRPN ? maxMidiNRPN : maxMidi); 
  Serial.print("    Modo: "); Serial.print(MODE_LABEL(mode)); 
  Serial.print("    Parameter: "); Serial.print((inputData.param_coarse() << 7) | inputData.param_fine());
  Serial.print("    Valor: "); Serial.print(mapValue); 
  Serial.print("    Valor original: "); Serial.println(rawValue);  
  prevValue[numInput] = mapValue;   // Save value to previous data array  
#endif

  return;
}

/*
   Funcion para filtrar el ruido analógico de los pontenciómetros. Analiza si el valor crece o decrece, y en el caso de un cambio de dirección,
   decide si es ruido o no, si hubo un cambio superior al valor anterior más el umbral de ruido.

   Recibe: -
*/
/*
   Funcion para filtrar el ruido analógico de los pontenciómetros. Analiza si el valor crece o decrece, y en el caso de un cambio de dirección,
   decide si es ruido o no, si hubo un cambio superior al valor anterior más el umbral de ruido.

   Recibe: -
*/
uint16_t IsNoise(uint16_t currentValue, uint16_t prevValue, uint16_t input, byte noiseTh, bool raw) {
  static bool upOrDownRaw[NUM_MUX * NUM_MUX_CHANNELS] = {};
  static bool upOrDownOutput[NUM_MUX * NUM_MUX_CHANNELS] = {};

  bool directionOfChange = upOrDownRaw[input] ? raw == true : upOrDownOutput[input];

  if (directionOfChange == ANALOG_UP) {
    if (currentValue > prevValue) {            // Si el valor está creciendo, y la nueva lectura es mayor a la anterior,
      return 0;                        // no es ruido.
    }
    else if (currentValue < prevValue - noiseTh) { // Si el valor está creciendo, y la nueva lectura menor a la anterior menos el UMBRAL
      if(raw) upOrDownRaw[input] = ANALOG_DOWN;        // se cambia el estado a DECRECIENDO y
      else    upOrDownOutput[input] = ANALOG_DOWN;
      return 0;                                       // no es ruido.
    }
  }
  else if (directionOfChange == ANALOG_DOWN) {
    if (currentValue < prevValue) { // Si el valor está decreciendo, y la nueva lectura es menor a la anterior,
      return 0;                                        // no es ruido.
    }
    else if (currentValue > prevValue + noiseTh) {  // Si el valor está decreciendo, y la nueva lectura mayor a la anterior mas el UMBRAL
      if(raw) upOrDownRaw[input] = ANALOG_UP;        // se cambia el estado a CRECIENDO y
      else    upOrDownOutput[input] = ANALOG_UP;
      return 0;                                      // no es ruido.
    }
  }
  return 1;         // Si todo lo anterior no se cumple, es ruido.
}

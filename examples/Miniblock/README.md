# Miniblock Arduino Sketch
Este es el código que usa Miniblock, el controlador MIDI de Yaeltex.

## Instalación

Para usar estos sketchs se requiere la última versión del [Arduino IDE](https://www.arduino.cc/en/main/software) y las siguientes librerías, instaladas en la carpeta Documents/Arduino/libraries:

* [Kilomux Shield Arduino Library](https://github.com/Yaeltex/kilomux-arduino-library/archive/master.zip)
* [Kilowhat SysEx Library](https://github.com/Yaeltex/kilowhat-arduino-library/archive/master.zip)
* [EEPROMex - extensión de la librería EEPROM](https://github.com/Yaeltex/kilowhat-arduino-library/blob/master/examples/libs/EEPROMEx.zip?raw=true)
* [NewPing - modificada por Yaeltex](https://github.com/Yaeltex/kilowhat-arduino-library/blob/master/examples/libs/NewPing.zip?raw=true)
* [Arduino MIDI Library 4.2 o superior](https://github.com/Yaeltex/kilowhat-arduino-library/blob/master/examples/libs/MIDI.zip?raw=true)

## Log

### 1.1
* Reordenamiento de los potenciómetros para que se correspondan con las filas del Kilowhat. Es necesario actualizar el archivo miniblock-default.kwt.
* Bugs en la recepción MIDI arreglados.
* LEDs se apagan con NOTE OFF con cualquier velocity 0-127. Antes solo con velocity 0.
* Funciona con la versión 0.9.1-mb.

### 1.0
* Versión inicial
* Funciona con la versión 0.9.1-mb.

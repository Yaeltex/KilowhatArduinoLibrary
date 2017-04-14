# Kilowhat SysEx

Esta librería fue hecha para facilitar la escritura y el acceso a la configuración guardada en la EEPROM de la Arduino, para ser utilizada por el usuario del Kilomux.
Para poder usarla, colocar los contenidos de la descarga en la carpeta Documents/Arduino/libraries, e incluir en tu sketch desde el Arduino IDE.

Última versión: v0.9.1 (funciona con la misma versión del Kilowhat, o superior)

## YTX-Controller
Este ejemplo es el código necesario para usar el [Kilowhat](http://wiki.yaeltex.com.ar/index.php?title=Kilowhat), preparado para enviar y recibir el protocolo SysEx, en este caso usado para que la Arduino reciba una nueva configuración de entradas y salidas, y guardarlo en la EEPROM interna de la placa.

El código está pensado para ser usado con el [Kilomux shield](http://wiki.yaeltex.com.ar/index.php?title=Kilomux_Shield) para Arduino, desarrollado por Yaeltex, y leerá las entradas activadas, para luego enviar los mensajes MIDI de acuerdo a la configuración guardada en la EEPROM.
Al recibir notas MIDI, la Arduino encenderá las salidas que correspondan.

Para usar este sketch se requieren las siguientes librerías:

* [Kilomux Shield](https://github.com/Yaeltex/kilomux-arduino-library/archive/master.zip)
* [Kilowhat SysEx Library](https://github.com/Yaeltex/kilowhat-arduino-library/archive/master.zip)
* [EEPROMex - extensión de la librería EEPROM](https://github.com/Yaeltex/kilowhat-arduino-library/blob/master/examples/libs/EEPROMEx.zip?raw=true)
* [NewPing - modificada por Yaeltex](https://github.com/Yaeltex/kilowhat-arduino-library/blob/master/examples/libs/NewPing.zip?raw=true)
* [Arduino MIDI Library 4.2](https://github.com/Yaeltex/kilowhat-arduino-library/blob/master/examples/libs/MIDI.zip?raw=true)
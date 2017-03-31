# YTX-Controller
Este ejemplo es el código necesario para usar el [Kilowhat](http://wiki.yaeltex.com.ar/index.php?title=Kilowhat), preparado para enviar y recibir el protocolo SysEx, en este caso usado para que la Arduino reciba una nueva configuración de entradas y salidas, y guardarlo en la EEPROM interna de la placa.

El código está pensado para ser usado con el [Kilomux shield](http://wiki.yaeltex.com.ar/index.php?title=Kilomux_Shield) para Arduino, desarrollado por Yaeltex, y leerá las entradas activadas, para luego enviar los mensajes MIDI de acuerdo a la configuración guardada en la EEPROM.
Al recibir notas MIDI, la Arduino encenderá las salidas que correspondan.

Para usar este sketch se requieren las siguientes librerías:

* [Kilomux Shield]()
* [Kilowhat SysEx Library]()
* [NewPing]()
* [Arduino MIDI Library 4.2]()
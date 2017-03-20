
/* * * * * * * * * * * * * * * * * * * * * * *
* Code by Martin Sebastian Wain for YAELTEX *
* contact@2bam.com                     2016 *
* * * * * * * * * * * * * * * * * * * * * * */

#ifndef _KM_DATA_H_
#define _KM_DATA_H_

#include <Arduino.h>
#include "KM_Accessors.h"
#include "KM_EEPROM.h"

#if 1
  #define DEBUG_PRINT(lbl, data) { Serial.print(lbl); Serial.print(": "); Serial.println(data); }
#else
  #define DEBUG_PRINT(lbl, data)
#endif

namespace KMS {
  enum Mode {
    M_OFF = 0,
    M_NOTE = 1,
    M_CC = 2,
    M_NRPN = 3,
	M_PROGRAM = 4,
    // Only for inputs (save one bit in outputs)
    M_SHIFTER = 5
  };
  enum Type {
    T_DIGITAL = 0,
    T_ANALOG = 1
  };

  const uint16_t nrpn_min_max[] PROGMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 
                                         16, 17, 18, 19, 20, 21, 23, 24, 26, 27, 29, 31, 33, 35, 
                                         38, 40, 43, 45, 48, 51, 55, 58, 62, 66, 70, 75, 79, 85, 
                                         90, 96, 102, 108, 115, 127, 131, 139, 148, 157, 168, 178, 
                                         190, 202, 231, 254, 255, 275, 293, 312, 331, 353, 375, 399, 
                                         425, 452, 481, 511, 544, 579, 616, 655, 697, 742, 789, 839, 
                                         893, 950, 1023, 1075, 1144, 1217, 1295, 1378, 1466, 1559, 
                                         1659, 1765, 1877, 1997, 2125, 2261, 2405, 2558, 2722, 2896, 
                                         3081, 3277, 3487, 3709, 3946, 4198, 4466, 4751, 5055, 5377, 
                                         5721, 6086, 6474, 6888, 7328, 7795, 8293, 8823, 9386, 9985, 
                                         10623, 11301, 12022, 12790, 13606, 14475, 15399, 16383};
										 
  static const byte PROTOCOL_VERSION = 1;
  extern EEPROM_IO io;

  namespace priv {
    extern int _inputsOffset;
    extern int _outputsOffset;
    extern byte _currBank;
    extern byte _maxBank;
    extern int _bankLen;
    extern int _bankOffset;
  }

  //This function should be called before all operations to KMS namespace.
  void initialize();
  
  //Returns number of banks. This is better than GlobalData::numBanks() because the user might have misconfigured using a lower memory device than the config.
  inline byte realBanks() { return priv::_maxBank; }
  inline GlobalData globalData() { return GlobalData(); }  //This is superfluous but keeps the interface
  inline InputUS ultrasound() { return InputUS(priv::_bankOffset); }
  inline InputNorm input(int index) { return InputNorm(priv::_bankOffset + priv::_inputsOffset + InputNorm::length * index); }
  inline Output output(int index) { return Output(priv::_bankOffset + priv::_outputsOffset + Output::length * index); }

  inline byte bank() { return priv::_currBank; }
  void setBank(byte bank);

} //namespace KMS

#endif // _KM_DATA_H_


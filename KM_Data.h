
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
  	M_PROGRAM_MINUS = 4,
  	M_PROGRAM = 5,
  	M_PROGRAM_PLUS = 6,
    // Only for inputs (save one bit in outputs)
    M_SHIFTER = 7
  };
  enum Type {
    T_DIGITAL = 0,
    T_ANALOG = 1
  };
  
  const uint16_t nrpn_min_max[] PROGMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 26, 27, 29, 31, 33, 35, 38, 40, 43, 45, 48, 51, 55, 58, 62, 66, 70, 75, 79, 85, 90, 96, 102, 108, 115, 127, 128, 139, 148, 157, 168, 178, 190, 202, 225, 254, 255, 275, 293, 312, 331, 353, 375, 399, 425, 452, 481, 511, 512, 552, 616, 655, 697, 742, 789, 839, 893, 950, 1023, 1071, 1151, 1215, 1295, 1375, 1471, 1551, 1663, 1759, 1871, 2047, 2127, 2255, 2399, 2559, 2719, 2895, 3087, 3279, 3487, 3711, 3951, 4095, 4463, 4751, 5055, 5375, 5727, 6079, 6479, 6879, 7327, 7791, 8191, 8815, 9391, 9985, 10623, 11295, 12015, 12783, 13599, 14479, 15391, 16383};

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


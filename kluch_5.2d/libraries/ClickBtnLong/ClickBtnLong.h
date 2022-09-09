/*ClickBtnLong.h
*/
#ifndef ClickBtnLong_h
#define ClickBtnLong_h

#include "Arduino.h"
class ClickBtnLong {
  public:
    ClickBtnLong(byte _pin, int _value, int _deviation);
    void run(void (* press)(), void (* longPress)());
	bool ClickBtnLong::VerifiClick ();
  protected:
    byte pin;
	int value =0;
	int deviation = 20;
    uint32_t time_Pressed = 0 ;
    uint32_t last_state = 0 ;
    bool bounce = 0; 		// антидребезговый флаг
	bool button_ln = 0;
    bool button_state = 0;
};

#endif //Cl_do_btn_long_h

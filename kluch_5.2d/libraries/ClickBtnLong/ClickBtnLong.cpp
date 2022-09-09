/*ClickBtnLong.cpp
*/
#include "Arduino.h"
#include "ClickBtnLong.h"
ClickBtnLong::ClickBtnLong(byte _pin, int _value, int _deviation) {
  
  //Setup the input pins
  pinMode(pin, INPUT);
  deviation	= _deviation;
  pin = _pin;
  value = _value;
}
bool ClickBtnLong::VerifiClick () {
		int A_Read = analogRead(pin);
	if (value < (A_Read + deviation) && value >= (A_Read - deviation)) return (true);
	  else return(false);
}

void ClickBtnLong::run(void (* press)(), void (* longPress)()) {
  if ((millis() - last_state) > 5000  ) button_ln = 0;
  if (! bounce && !button_ln && VerifiClick()) {
    bounce = 1;                                     // выставить флаг что кнопка нажата
    time_Pressed = millis();                        // сделать временую засветку
  }
  if (bounce && millis() - time_Pressed >= 50) { 	// если прошло антидребезговое время
    bounce = 0;      								// то снять флаг
    if (VerifiClick() && !button_state) { 			// если кнопка по прежнему нажата 
      button_state = 1;
      last_state = millis();
    }
	if (VerifiClick() && button_state && ((millis() - last_state) >= 500)) {
		button_state = 0;
		button_ln = 1;
		longPress();//длиное нажатие
		}
  }
  if ((!VerifiClick()) && button_state) {
	if ( (millis() - last_state) < 500  ) {
	  button_state = 0;
	  button_ln = 0;
	  press();// короткое нажатие
	}
	else if (button_state && millis() - last_state >= 500) {
			button_state = 0;
			button_ln = 0;
			longPress();//длиное нажатие
		  }
  }
}


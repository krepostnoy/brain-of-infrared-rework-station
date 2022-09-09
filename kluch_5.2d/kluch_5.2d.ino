/* Тема, в которой обсуждается этот проект, https://clck.ru/JNqKg
   link to the topic that discusses this development: https://clck.ru/JNqKg */
//редактирование от 21-06-2022 года.

#pragma GCC optimize ("Os")
#include <EEPROM.h>                                            //библиотека энергонезависимой памяти
#include <LiquidCrystal_I2C.h>                                 //библиотека дисплея LCD2004 i2c
LiquidCrystal_I2C lcd(0x3f, 20, 4);                            //вместо (0x3f) указываем адрес своего дисплея, например,0x27
//LiquidCrystal_I2C lcd(0x27, 20, 4);                            //вместо (0x27) указываем адрес своего дисплея, например,0x3f
#include "Setting.h"                                           //файл начальных установок
#include "Variables.h"                                         //файл переменных и флагов

#ifdef resist_keyboard                                         //если клавиатура резистивная
#include <ClickBtnLong.h>                                      //библиотека аналоговой (резистивной) клавиатуры 
ClickBtnLong Btn_UP(PinKeyboard, PIN_UP, ABERRATION);          //передаём данные кнопок резистивной клавиатуры в библиотеку ClickBtnLong.h
ClickBtnLong Btn_DOWN(PinKeyboard, PIN_DOWN, ABERRATION);      //
ClickBtnLong Btn_LEFT(PinKeyboard, PIN_LEFT, ABERRATION);      //
ClickBtnLong Btn_RIGHT(PinKeyboard, PIN_RIGHT, ABERRATION);    //
ClickBtnLong Btn_OK(PinKeyboard, PIN_OK, ABERRATION);          //
#else
#include <Cl_do_btn_long.h>                                    //библиотека одиночных кнопок
Cl_do_btn_long Btn_UP(PIN_UP);                                 //передаём данные одиночных кнопок в библиотеку Cl_do_btn_long.h
Cl_do_btn_long Btn_DOWN(PIN_DOWN);                             //
Cl_do_btn_long Btn_LEFT(PIN_LEFT);                             //
Cl_do_btn_long Btn_RIGHT(PIN_RIGHT);                           //
Cl_do_btn_long Btn_OK(PIN_OK);                                 //
#endif

void setup() {                                                 //начальные установки
  Serial.begin(9600);

#ifdef resist_keyboard
  pinMode(A6, INPUT);
#else
  pinMode (PIN_UP, INPUT_PULLUP);                              //подключен подтягивающий резистор
  pinMode (PIN_DOWN, INPUT_PULLUP);                            //подключен подтягивающий резистор
  pinMode (PIN_LEFT, INPUT_PULLUP);                            //подключен подтягивающий резистор
  pinMode (PIN_RIGHT, INPUT_PULLUP);                           //подключен подтягивающий резистор
  pinMode (PIN_OK, INPUT_PULLUP);                              //подключен подтягивающий резистор
#endif
#ifdef SetInterrupt 0
  pinMode (2, INPUT_PULLUP);                                   //подтяжка пина ZCC к +5v, для защиты от ложных включений излучателей.
#endif
#ifdef SetInterrupt 1
  pinMode (3, INPUT_PULLUP);                                   //подтяжка пина ZCC к +5v, для защиты от ложных включений излучателей.
#endif
  pinMode(thermoCStop, OUTPUT);                                //задаём состояние пинов, к которым подключены MAX6675
  pinMode(thermoCSbott, OUTPUT);                               //
  pinMode(thermoCLK, OUTPUT);                                  //
  pinMode(thermoSO, INPUT);                                    //
  pinMode(buzzerPin, OUTPUT);                                  //устанавливаем пин пищалки на ВЫХОД
  pinMode (Int_Fan, OUTPUT);                                   //устанавливаем пины охладителей на ВЫХОД
  pinMode (Ext_Fan, OUTPUT);                                   //
  pinMode(RelayPin1, OUTPUT);                                  //задаём состояние пинов управления выходами
  pinMode(RelayPin2, OUTPUT);                                  //

  Hello_guys();                                                //сообщение приветствия
  SongHello();                                                 //Мелодия приветствия
  delay(3000);                                                 //показываем заставку пока стабилизируются MAX6675
  lcd.clear();                                                 //очищаем LCD
  loadProfile();                                               //вызов функции loadProfile для загрузки данных профиля из eeprom
  nextRead1 = millis();                                        //запускаем чтение с термопар

  /*настраиваем работу с прерыванием: LOW вызывает прерывание, когда на порту LOW
    CHANGE прерывание вызывается при смене значения на порту, с LOW на HIGH и наоборот
    RISING прерывание вызывается только при смене значения на порту с LOW на HIGH
    FALLING прерывание вызывается только при смене значения на порту с HIGH на LOW */
#ifdef SetInterrupt                                            //если используется схема ZCC
  attachInterrupt(SetInterrupt, Dimming, RISING);              //настроить порт прерывания(0 или 1) 2й или 3й цифровой пин
#else
  MsTimer2::set(10, Dimming);                                  //10 ms period (настройка таймера)
  MsTimer2::start();                                           //если прерывания по таймеру, а не от схемы ZCC
#endif
}

void loop() {                                                  //---основной цикл программы---//
  unsigned long currentMillis = millis();
  int SP2 = profile.Setpoint2 * 10;                            //множим уставку НИ на 10
  float out_float;                                             //вывод на LCD дробных чисел

  Btn_UP.run(&PushUp, &PushUpLong);                            //указываем обработчики для кнопки Up
  Btn_DOWN.run(&PushDown, &PushDownLong);                      //указываем обработчики для кнопки Down
  Btn_LEFT.run(&PushLeft, &PushLeftLong);                      //указываем обработчики для кнопки Left
  Btn_RIGHT.run(&PushRight, &PushRightLong);                   //указываем обработчики для кнопки Right
  Btn_OK.run(&PushOk, &PushOkLong);                            //указываем обработчики для кнопки Ok

  switch (reflowState) {
    case REFLOW_STATE_IDLE: {                                  //главное меню
        TopStart = false;                                      //ВИ отключен
        currentStep = 1;                                       //шаг установлен №1
        counter = 0;                                           //счетчик обнулили
        setpointRamp = 0;                                      //скорость роста t°C обнулили
        x = 1;                                                 //устанавливаем переменную в исходное состояние
        flag = 0;                                              //после отановки профиля сбрасываем флаг
        Output1 = 0;                                           //выход ПИД ВИ обнулили
        Output2 = 0;                                           //выход ПИД НИ обнулили

        if (updateScreen) {                                    //Настройка экрана в режиме ожидания
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("State:IDLE "));
          lcd.setCursor(1, 1);
          lcd.print(F("Name:"));
          lcd.setCursor(11, 0);
          lcd.print(F("m>(*.*)<m"));
          lcd.setCursor(11, 1);
          lcd.print(F("Step: 1"));
          lcd.setCursor(1, 2);
          lcd.print(F("ThSP:"));
          lcd.setCursor(11, 2);
          lcd.print(F("Real:"));
          lcd.setCursor(1, 3);
          lcd.print(F("BhSP:"));
          lcd.setCursor(11, 3);
          lcd.print(F("Real:"));
          updateScreen = false;
        }
        lcd.setCursor(7, 1);
        lcd.print(currentProfile);
        lcd.print(F("  "));
        lcd.setCursor(7, 2);
        lcd.print(profile.temperatureStep[0]);
        lcd.print(F(" "));
        lcd.setCursor(7, 3);
        lcd.print(SP2);
        lcd.print(F(" "));

        if (millis() > nextRead1) {                            //чтение и вывод на LCD температур, подготовка данных для ПК
          nextRead1 = millis() + SENSOR_SAMPLING_TIME;         //устраняем колебания датчиков температуры
          TempRead();
          if (kluch == 4) {                                    //выводим в порт
            kluch = 0;
            BufferUpload();
          }
          if (kluch == 2) TempDisplay();
          kluch++;
        }
      } break;

    case REFLOW_STATE_PROFILE_INIT: {                          //инициализация тестового профиля
        /* 4 шага,
           уставка НИ 160° (записываем поделив на 10, показываем как обычно),
           min BOTTOM PWR 0%,
           max BOTTOM PWR 50%,
           pTarg 70,
           pPwr 90%,
           min TOP PWR по шагам 10%,10%,10%,10%,
           max TOP PWR по шагам 99%,99%,99%,99%,
           скорости роста t° по шагам 10,3,3,6,
           уставки ВИ по шагам 180,190,200,225,
           длительности полок по шагам 20,15,15,15,
           P.I.D TOP 10,15,40, P.I.D BOTTOM 8,20,40. */
        profile = {4, 16, 0, 50, 70, 90, 10, 10, 10, 10, 99, 99, 99, 99, 10, 3, 3, 6, 180, 190, 200, 225, 20, 15, 15, 15, 10, 15, 40, 8, 20, 40};
        SaveProfile();
        tone(buzzerPin, 1045, 200);                             //звуковой сигнал при сбросе профиля
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print(F("the preset profile"));
        lcd.setCursor(3, 2);
        lcd.print(F("is a uploaded!"));
        delay(2000);
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      } break;

    case REFLOW_STATE_MENU_STEPS: {                            //устанавливаем количество шагов профиля
        if (updateScreen) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Profile:"));
          lcd.setCursor(11, 0);
          lcd.print(F("Steps:"));
          updateScreen = false;
        }
        lcd.setCursor(8, 0);
        lcd.print(currentProfile);
        lcd.print(F(" "));
        lcd.setCursor(17, 0);
        lcd.print(profile.profileSteps);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_BOTTOM_TARGET: {                    //устанавливаем температуру "Нижнего Нагревателя"
        if (updateScreen) {
          lineForm(0, 1, 3);
          updateScreen = false;
        }
        lcd.setCursor(17, 1);
        lcd.print(SP2);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_STEP_TARGET: {                      //устанавливаем температуру "Верхнего Нагревателя"
        if (updateScreen) {
          lineForm(0, 2, 9);
          lcd.setCursor(8, 2);
          lcd.print(editStep + 1);
          lcd.setCursor(9, 2);
          lcd.print(F(" Target:  "));
          updateScreen = false;
        }
        lcd.setCursor(17, 2);
        lcd.print(profile.temperatureStep[editStep]);
        lcd.print(F(" "));
        lineForm(0, 1, 3);
      } break;

    case REFLOW_STATE_MENU_STEP_RAMP: {                        //устанавливаем скорость нагрева "Верхним Нагревателем"
        if (updateScreen) {
          lcd.setCursor(8, 2);
          lcd.print(editStep + 1);
          lcd.setCursor(9, 2);
          lcd.print(F("   Ramp: "));
          updateScreen = false;
        }
        lcd.setCursor(17, 2);
        out_float = profile.rampRateStep[editStep] / 10.0;
        lcd.print(out_float, 1);
        lcd.print(F(" "));
        lineForm(0, 1, 3);
      } break;

    case REFLOW_STATE_MENU_STEP_DWELL: {                       //устанавливаем время перехода на следующий шаг (длительность шага)
        if (updateScreen) {
          lcd.setCursor(8, 2);
          lcd.print(editStep + 1);
          lcd.print(F("  Dwell:"));
          updateScreen = false;
        }
        lcd.setCursor(17, 2);
        lcd.print(profile.dwellTimerStep[editStep]);
        lcd.print(F("  "));
        lineForm(0, 1, 3);
      } break;

    case REFLOW_STATE_MENU_BOTTOM_kf2: {                       //настройка "kf2" ПИД нижнего нагревателя
        if (updateScreen) {
          lcd.setCursor(0, 2);
          lcd.print(F("BotHeater    kf2:"));
          updateScreen = false;
        }
        lcd.setCursor(17, 2);
        lcd.print(kf2);
        lcd.print(F("  "));
        lineForm(0, 1, 5);
      } break;

    case REFLOW_STATE_MENU_BOTTOM_P: {                         //настройка "P" ПИД нижнего нагревателя
        if (updateScreen) {
          lineForm(0, 3, 2);
          updateScreen = false;
        }
        lcd.setCursor(2, 3);
        lcd.print(profile.kp2);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_BOTTOM_I: {                         //настройка "I" ПИД нижнего нагревателя
        lcd.setCursor(9, 3);
        lcd.print(profile.ki2);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_BOTTOM_D: {                         //настройка "D" ПИД нижнего нагревателя
        lcd.setCursor(16, 3);
        if (profile.kd2 >= 100) {
          lcd.print(profile.kd2);
          lcd.print(F(" "));
        } else {
          lcd.setCursor(16, 3);
          lcd.print( (profile.kd2));
          lcd.print(F("  "));
        }
      } break;

    case REFLOW_STATE_MENU_TOP_P: {                            //настройка "P" ПИД верхнего нагревателя
        if (updateScreen) {
          lcd.setCursor(0, 2);
          lcd.print(F("TopHeater          "));
          lineForm(0, 3, 2);
          updateScreen = false;
        }
        lcd.setCursor(2, 3);
        lcd.print(profile.kp1);
        lcd.print(F("  "));
        updateScreen = false;
      } break;

    case REFLOW_STATE_MENU_TOP_I: {                            //настройка "I" ПИД верхнего нагревателя
        lcd.setCursor(9, 3);
        lcd.print(profile.ki1);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_TOP_D: {                            //настройка "D" ПИД верхнего нагревателя
        lcd.setCursor(16, 3);
        if (profile.kd1 >= 100) {
          lcd.print(profile.kd1);
          lcd.print(F(" "));
        } else {
          lcd.setCursor(16, 3);
          lcd.print( (profile.kd1));
          lcd.print(F("  "));
        }
      } break;

    case REFLOW_STATE_MENU_BOTTOM_PWR_MIN: {                   //устанавливаем минимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (updateScreen) {
          lineForm(0, 1, 4);
          lineForm(0, 2, 6);
          lineForm(0, 3, 8);
          updateScreen = false;
        }
        lcd.setCursor(4, 3);
        lcd.print(profile.min_pwr_BOTTOM);
        lcd.print(F("%  "));
        lcd.setCursor(17, 3);
        lcd.print(F("   "));
      } break;

    case REFLOW_STATE_MENU_BOTTOM_PWR_MAX: {                   //устанавливаем максимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        lcd.setCursor(4, 3);
        lcd.print(profile.min_pwr_BOTTOM);
        lcd.print(F("% "));
        lcd.setCursor(17, 3);
        lcd.print(profile.max_pwr_BOTTOM);
        if (profile.max_pwr_BOTTOM < 10) {
          lcd.setCursor(18, 3);
          lcd.print(F("% "));
        } else {
          lcd.setCursor(19, 3);
          lcd.print(F("%"));
        }
      } break;

    case REFLOW_STATE_MENU_BOTTOM_pTarg: {                     //устанавливаем t°C преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (updateScreen) {
          lineForm(0, 1, 4);
          lineForm(0, 2, 6);
          lineForm(0, 3, 7);
          updateScreen = false;
        }
        lcd.setCursor(6, 3);
        lcd.print(profile.pTarg);
        lcd.print(F("   "));
        lcd.setCursor(17, 3);
        lcd.print(F("   "));
      } break;

    case REFLOW_STATE_MENU_BOTTOM_pPwr: {                      //устанавливаем мощность преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        lcd.setCursor(17, 3);
        lcd.print(profile.pPwr);
        if (profile.pPwr < 10) {
          lcd.setCursor(18, 3);
          lcd.print(F("% "));
        } else {
          lcd.setCursor(19, 3);
          lcd.print(F("%"));
        }
      } break;

    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                      //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (updateScreen) {
          lineForm(0, 2, 9);
          lcd.setCursor(8, 2);
          lcd.print(editStep + 1);
          lcd.setCursor(9, 2);
          lcd.print(F(" PWR Limits"));
          lineForm(0, 3, 8);
          updateScreen = false;
        }
        lcd.setCursor(4, 3);
        lcd.print(profile.min_pwr_TOPStep[editStep]);
        lcd.print(F("% "));
        lcd.setCursor(17, 3);
        lcd.print(F("   "));
      } break;

    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                      //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (updateScreen) {
          lcd.setCursor(8, 2);
          lcd.print(editStep + 1);
          updateScreen = false;
        }
        lcd.setCursor(17, 3);
        lcd.print(profile.max_pwr_TOPStep[editStep]);;
        if (profile.max_pwr_TOPStep[editStep] < 10) {
          lcd.setCursor(18, 3);
          lcd.print(F("% "));
        } else {
          lcd.setCursor(19, 3);
          lcd.print(F("%"));
        }
      } break;

    case REFLOW_STATE_PRE_HEATER: {                            //ПРЕДНАГРЕВ НИ
#ifndef inert_heater
        Output2 = 3;
        if (millis() - previousMillis > 5000) {
          Output2 = 0;
          reflowState = REFLOW_STATE_STEP_RAMP;
        }
#else
        Output2 = profile.pPwr;
        if (Input2 >= profile.pTarg) {
          tone(buzzerPin, 1045, 200);                          //звуковой сигнал о завершении этапа "преднагрев НИ"
          Output2 = 0;
          reflowState = REFLOW_STATE_STEP_RAMP;
        }
#endif
      } break;

    /*  case REFLOW_STATE_BOTTOM_PAUSE: {
          if ((millis() - previousMillis) > 120000) {
            previousMillis = millis();
            tone(buzzerPin, 800, 500);                                     //звуковой сигнал
            tone(buzzerPin, 1045, 500);                                     //звуковой сигнал
            TopStart = true;
            reflowState = REFLOW_STATE_STEP_RAMP;
          }
        } break; */

    case REFLOW_STATE_STEP_RAMP: {                                        //"старт и процесс пайки", рост температуры с заданной скоростью
        digitalWrite(Int_Fan, HIGH);                                      //включить охладитель симисторов и контроллера
        if (tc1 >= SP2 - delta && !TopStart) TopStart = true;             //если по данным ДАТЧИКА У ЧИПА низ достиг уставки, или уставки минус delta°С, включаем верхний нагреватель
        //if (tc1 >= SP2 - delta && !TopStart) TopStart = false;          // смотрел t° низа платы при настройке ПИД НИ
        /*  if (tc1 >= SP2 - delta && !TopStart && !BottomPause) {            //если по данным ДАТЧИКА У ЧИПА низ достиг уставки, или уставки минус delta°С, включаем верхний нагреватель
            tc2 = profile.Setpoint2 * 10;                                   //умножаем, потому что profile.Setpoint2 мы ранее поделили на 10
            BottomPause = true;
            tone(buzzerPin, 1000, 500);                                     //звуковой сигнал
            previousMillis = millis();
            reflowState = REFLOW_STATE_BOTTOM_PAUSE;
          } */

        if (TopStart == true) {                                           //открываем скобку (чтоб не запутаться) //если включен верхний нагреватель
          if (flag == 0) {                                                //фиксируем стартовую температуру
            startTemp = tc1;
            flag = 1;
          }
          if (startTemp > profile.temperatureStep[currentStep - 1]) {     //устанавливаем нужный шаг, до которого нагрета плата
            for (x = 1; startTemp > profile.temperatureStep[currentStep - 1]; currentStep++)  x++;
          }
          if (currentStep > x && flag == 1) {
            flag = 0;
            startTemp = profile.temperatureStep[currentStep - 2];
            flag = 1;
          }
          lcd.setCursor(17, 1);
          lcd.print(currentStep);
          lcd.setCursor(6, 3);
          lcd.print(SP2);
          lcd.print(F(" "));                                              //?

          if ((currentMillis - previousMillis) > 1000 / (profile.rampRateStep[currentStep - 1] * 0.1))  {  //счётчик скорости роста температуры от 0.1с. до 3с.
            previousMillis = currentMillis;
            counter = counter + 1;
            setpointRamp = counter + startTemp;
            lcd.setCursor(6, 2);
            lcd.print(setpointRamp);
            lcd.print(F("  "));
            Setpoint1 = setpointRamp;
          }
        }                                                                 //закрывам скобку

        if (setpointRamp >= profile.temperatureStep[currentStep - 1]) {   //если достигли нужной температуры
          lcd.setCursor(6, 2);
          lcd.print(profile.temperatureStep[currentStep - 1]);
          reflowState = REFLOW_STATE_STEP;
        }
      } break;

    case REFLOW_STATE_STEP: {                                             //
        Setpoint1 = profile.temperatureStep[currentStep - 1];
        if (Input1 >= profile.temperatureStep[currentStep - 1]) {
          counter = 0;
          reflowState = REFLOW_STATE_STEP_DWELL;
        }
      } break;

    case REFLOW_STATE_STEP_DWELL: {                                       //считаем время перехода на следующий шаг
        if (currentMillis - previousMillis > 1000) {
          previousMillis = currentMillis;
          counter = counter + 1;
        }
        if (counter >= profile.dwellTimerStep[currentStep - 1]) {         //если счётчик равен установленному времени
          tone(buzzerPin, 1045, 200);                                     //звуковой сигнал о переходе на следующий шаг
          counter = 0;
          setpointRamp = 0;
          if (profile.profileSteps == currentStep) {                      //если достигли последнего шага
            currentStep = 1;
            x = 1;                                                        //устанавливаем переменную в исходное состояние
            flag = 0;                                                     //после завершения профиля сбрасываем флаг
            reflowState = REFLOW_STATE_COMPLETE;
          } else {                                                        //если шаг не последний
            currentStep++;                                                //переходим на следующий шаг
            reflowState = REFLOW_STATE_STEP_RAMP;
          }
        }
      } break;

    case REFLOW_STATE_COMPLETE: {                                         //завершение пайки
        updateScreen = true;
        digitalWrite(Int_Fan, LOW);                                       //выключить охладитель симисторов и контроллера
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
        TopStart = false;                                                 //состояние ВИ - отключен
        SongAlarm();
      } break;

  }                                                                       //здесь закончился switch

  if (reflowStatus == REFLOW_STATUS_ON) {                                 //включение нагревателей
    if (updateScreen) {
      lcd.setCursor(0, 0);
      lcd.print(F("State:RUN!"));
      updateScreen = false;
    }
    if (millis() > nextRead1) {
      nextRead1 = millis() + SENSOR_SAMPLING_TIME;                        //считываем данные с термопар и считаем ПИД
      TempRead();                                                         //считываем данные с термопар
      Count1++;                                                           //увеличиваем счетчик Count1
      chast = Count1 / 4;
      kluch = Count1 % 4;

#ifdef LongBufferPid                                                      //при использовании Pid2buf для инерционного НИ из керамических нагревателей
      if (reflowState != REFLOW_STATE_PRE_HEATER) Output2 = Pid2buf(Input2, SP2, profile.kp2, profile.ki2, profile.kd2);
#else                                                                     //при использовании Pid2 для "быстрых" НИ из галогеновых ламп     
      if (reflowState != REFLOW_STATE_PRE_HEATER) Output2 = Pid2(Input2, SP2, profile.kp2, profile.ki2, profile.kd2);
#endif
      if (TopStart) Output1 = Pid1(Input1, Setpoint1, profile.kp1, profile.ki1, profile.kd1); else Output1 = 0;
    }
    if (kluch == 3) {                                                   //выводим в порт
      kluch = 0;
      BufferUpload();
      //BufferUpload2();
      //BufferUpload3();
    }
    if (kluch == 2) {                                                   //
      TempDisplay();
    }
    if (kluch == 1) {
#ifdef TimePrint
      PrintTime();
#endif
    }
  }
  else {
    digitalWrite(RelayPin1, LOW);
    digitalWrite(RelayPin2, LOW);
  }

}                                                                         //конец основного цикла loop программы

void Dimming() {
  if (reflowStatus == REFLOW_STATUS_ON) {
    OutPWR_TOP();
    OutPWR_BOTTOM();
  }
}

void OutPWR_TOP() {
  reg1 = Output1 + er1;                                                   //pwr- задание выходной мощности в %, er- ошибка округления
  if (reg1 < 50) {
    out1 = LOW;
    er1 = reg1 ;                                                          //reg- переменная для расчетов
  } else {
    out1 = HIGH;
    er1 = reg1 - 100;
  }
  digitalWrite(RelayPin1, out1);                                          //пин через который осуществляется дискретное управление
}

void OutPWR_BOTTOM() {
  reg2 = Output2 + er2;                                                   //pwr- задание выходной мощности в %, er- ошибка округления
  if (reg2 < 50) {
    out2 = LOW;
    er2 = reg2 ;                                                          //reg- переменная для расчетов
  } else {
    out2 = HIGH;
    er2 = reg2 - 100;
  }
  digitalWrite(RelayPin2, out2);                                          //пин через который осуществляется дискретное управление
}

byte Pid1(double temp, byte ust, byte kP, byte kI, byte kd) {             //ПИД для ВИ
  byte out = 0;
  static float ed = 0;
  e1 = (ust - temp);                                                      //ошибка регулирования
  p1 =  (kP * e1) / 10;                                                   //П составляющая
  integra = (integra < i_min) ? i_min : (integra > i_max) ? i_max : integra + (kI * e1) / 1000; //И составляющая
  d1 = kd * (temp - ed);                                                  //Д составляющая
  ed = temp;
  out = (p1 + integra - d1 < profile.min_pwr_TOPStep[currentStep - 1]) ? profile.min_pwr_TOPStep[currentStep - 1] : (p1 + integra - d1 > profile.max_pwr_TOPStep[currentStep - 1]) ? profile.max_pwr_TOPStep[currentStep - 1] : p1 + integra - d1;
  return out;
}

#ifdef LongBufferPid
byte Pid2buf(double temp, int ust, byte kP, byte kI, byte kd) {           //ПИД для "тугих" НИ из керамических и им подобных излучателей
  byte out = 0;
  static const byte buff_depth = 8;
  static float ed[buff_depth];
  static byte buff_position;
  e2 = (ust - temp);                                                      //ошибка регулирования
  p2 =  (kP * e2) / 10;                                                   //П составляющая
  integra2 = (integra2 < i_min) ? i_min : (integra2 > i_max) ? i_max : integra2 + (kI * e2) / 10000; //И составляющая
  //d2 = kd * (temp - ed[buff_position]) / buff_depth * 10;                 //Д составляющая
  //d2 = kd * (temp - ed);                                                //Д составляющая
  d2 = kd * (temp - ed[buff_position]) / buff_depth;                      //Д составляющая
  ed[buff_position] = temp;
  buff_position = (buff_position < buff_depth - 1) ? buff_position + 1 : 0;
  //out = (p2 + integra2 - d2 < 0) ? 0 : (p2 + integra2 - d2 > profile.max_pwr_BOTTOM) ? profile.max_pwr_BOTTOM : p2 + integra2 - d2;
  out = (p2 + integra2 - d2 < profile.min_pwr_BOTTOM) ? profile.min_pwr_BOTTOM : (p2 + integra2 - d2 > profile.max_pwr_BOTTOM) ? profile.max_pwr_BOTTOM : p2 + integra2 - d2;
  return out;
}
#else
byte Pid2(double temp, int ust, byte kP, byte kI, byte kd) {              //ПИД для "шустрых" НИ на галогенках и им подобных излучателях
  byte out = 0;
  static float ed = 0;
  e2 = (ust - temp);                                                      //ошибка регулирования
  p2 =  (kP * e2) / 10;                                                   //П составляющая
  integra2 = (integra2 < i_min) ? i_min : (integra2 > i_max) ? i_max : integra2 + (kI * e2) / 1000; //И составляющая
  d2 = kd * (temp - ed);                                                  //Д составляющая
  ed = temp;
  //out = (p2 + integra2 - d2 < 0) ? 0 : (p2 + integra2 - d2 > profile.max_pwr_BOTTOM) ? profile.max_pwr_BOTTOM : p2 + integra2 - d2;
  out = (p2 + integra2 - d2 < profile.min_pwr_BOTTOM) ? profile.min_pwr_BOTTOM : (p2 + integra2 - d2 > profile.max_pwr_BOTTOM) ? profile.max_pwr_BOTTOM : p2 + integra2 - d2;
  return out;
}
#endif

double max6675_read_temp (int ck, int cs, int so) {              //MAX6675 functions @Dmitrysh
  char i;
  int tmp = 0;
  digitalWrite(cs, LOW);//cs = 0;                                //Stop a conversion in progress
  asm volatile (" nop"  "\n\t");
  for (i = 15; i >= 0; i--) {
    digitalWrite(ck, HIGH);
    asm volatile (" nop"  "\n\t");
    if ( digitalRead(so))
      tmp |= (1 << i);
    digitalWrite(ck, LOW);
    asm volatile (" nop"  "\n\t");
  }
  digitalWrite(cs, HIGH);
  if (tmp & 0x4) {
    return NAN;
  } else
    return ((tmp >> 3)) * 0.25;
}

void loadProfile()  {                                            //чтение профиля из EEPROM
  EEPROM.get((currentProfile - 1)*SizeProfile, profile);
}

void SaveProfile()    {                                          //сохранение текущего профиля
  EEPROM.put((currentProfile - 1)*SizeProfile, profile);
}

void TempRead() {                                                //чтение данных от текущих температурах с MAX6675 (преобразователей сигналов термопар)
  if (Input1 <= 0) Input1 = max6675_read_temp (thermoCLK, thermoCStop, thermoSO);
  else Input1 = Input1 * 0.8 + 0.2 * (max6675_read_temp (thermoCLK, thermoCStop, thermoSO));  //термопара "ВЕРХНЕГО" нагревателя
  if (Input2 <= 0) Input2 = max6675_read_temp (thermoCLK, thermoCSbott, thermoSO);
  else Input2 = Input2 * 0.8 + 0.2 * (max6675_read_temp (thermoCLK, thermoCSbott, thermoSO)); //термопара "НИЖНЕГО"  нагревателя
  tc1 = Input1;
  tc2 = Input2;
  if (reflowState == REFLOW_STATE_STEP_RAMP || reflowState == REFLOW_STATE_STEP
      || reflowState == REFLOW_STATE_STEP_DWELL || reflowState == REFLOW_STATE_PRE_HEATER) {
    if (isnan(Input1) || isnan(Input2)) {
      reflowState = REFLOW_STATE_COMPLETE;
    }
  }
}

void TempDisplay()  {                                            //вывод текущих температур на LCD
  lcd.setCursor(17, 2);
  if (isnan(Input1)) {
    lcd.print("Er");
  } else {
    if (tc1 >= 100) {
      lcd.print(tc1);
      lcd.print(F(""));
    } else {
      lcd.setCursor(17, 2);
      lcd.print( (tc1));
      lcd.print(F(" "));
    }
  }
  lcd.setCursor(17, 3);
  if (isnan(Input2)) {
    lcd.print("Er");
  } else {
    if (tc2 >= 100) {
      lcd.print(tc2);
      lcd.print(F(""));
    } else {
      lcd.setCursor(17, 3);
      lcd.print( (tc2));
      lcd.print(F(" "));
    }
  }
}
void BufferUpload() {                                            //стандартный вывод данных
#ifdef SerialPortPlotter
  sprintf (buf, "$%03d %03d %03d %03d;", int(Output1), int(Output2), tc1, tc2);
#else
  sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2, currentProfile);
#endif
  Serial.println(buf);
}

void BufferUpload2() {                                           //расширенный вывод данных для настройки НИ
#ifdef SerialPortPlotter
  sprintf (buf, "$%03d %03d %03d %03d %03d %03d %03d;", int(Output1), int(Output2), tc1, tc2, int(p2), int(integra2), int(d2));
#else
  sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2, currentProfile);
#endif
  Serial.println(buf);
}

void BufferUpload3() {                                           //расширенный вывод данных для настройки ВИ
#ifdef SerialPortPlotter
  sprintf (buf, "$%03d %03d %03d %03d %03d %03d %03d;", int(Output1), int(Output2), tc1, tc2, int(p1), int(integra), int(d1));
#else
  sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2, currentProfile);
#endif
  Serial.println(buf);
}

void Hello_guys() {                                              //сообщение приветствия
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("brain of IR stations"));
  lcd.setCursor(1, 1);
  lcd.print(F("v1.3.3.   SPP/irsp"));
  lcd.setCursor(1, 2);
  lcd.print(F("/forum.amperka.ru/"));
  lcd.setCursor(0, 3);
  lcd.print(F("everyone loves cats!"));
}

void SongHello() {                                               //Мелодия приветствия
  tone(buzzerPin, 523);
  delay(200);
  tone(buzzerPin, 659);
  delay(200);
  tone(buzzerPin, 784);
  delay(200);
  tone(buzzerPin, 1046);
  delay(200);
  noTone(buzzerPin);
}

void PrintTime() {                                               //длительность пайки в секундах
  lcd.setCursor(11, 0);                                          //
  lcd.print(F("time"));                                          //
  lcd.setCursor(15, 0);                                          //
  lcd.print(F(":    "));                                         //
  lcd.setCursor(16, 0);                                          //
  lcd.print(F(""));                                              //
  lcd.print(chast);                                              // секунды
}

void SongAlarm() {                                               //звук предупреждения
  tone(buzzerPin, 1046);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1046);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1046);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1046);
  delay(100);
  noTone(buzzerPin);
  delay(100);
}

void lineForm(byte col, byte str, byte n) {                      //форма для вывода на LCD определенной информации
  lcd.setCursor(col, str);
  if (n == 1) {
    lcd.print(F("                    "));
  }
  if (n == 2) {
    lcd.print(F("P:     I:     D:    "));
  }
  if (n == 3) {
    lcd.print(F("BotHeater Target:"));
  }
  if (n == 4) {
    lcd.print(F("Power   Parameters  "));
  }
  if (n == 5) {
    lcd.print(F("P. I. D.  Parameters"));
  }
  if (n == 6) {
    lcd.print(F("BotHeater PWR Limits"));
  }
  if (n == 7) {
    lcd.print(F("pTarg:      pPwr:"));
  }
  if (n == 8) {
    lcd.print(F("min:         max:"));
  }
  if (n == 9) {
    lcd.print(F("TopStep:"));
  }
}

void PushOk() {                                                  //обработка короткого нажатия Ok
  if (reflowState == REFLOW_STATE_IDLE) {                        //если коротко нажать Ok в IDLE
    tone(buzzerPin, 1045, 500);                                  //звуковой сигнал при старте профиля
    updateScreen = true;
#ifdef SerialPortPlotter                                         //если ПО SerialPortPlotter версии @geleos27
    sprintf (buf, "$#");
    Serial.println(buf);
#else                                                            //если ПО irsp.exe от @Dmitrysh
    sprintf (buf, "SYNC\r\n");
    Serial.println(buf);
#endif
    nextRead1 = millis();
    digitalWrite(Ext_Fan, LOW);                                  //выключить охладитель платы
    digitalWrite(Int_Fan, HIGH);                                 //включить охладитель симисторов и контроллера
    integra = 0;
    integra2 = 0;
    Count1 = 0;
    p1 = 0;
    p2 = 0;
    d1 = 0;
    d2 = 0;
    previousMillis = millis();
    reflowStatus = REFLOW_STATUS_ON;                             //статус "ПАЙКА" активируется
    reflowState = REFLOW_STATE_PRE_HEATER;                       //переходим в состояние "ПРЕДНАГРЕВ НИ"
    updateScreen = true;
  }
}

void PushOkLong() {                                              //обработка длинного нажатия Ok
  if (reflowState == REFLOW_STATE_STEP_RAMP || reflowState == REFLOW_STATE_STEP
      || reflowState == REFLOW_STATE_STEP_DWELL || reflowState == REFLOW_STATE_PRE_HEATER) { //ничего не делаем в этих состояниях
    return;
  }
  if (reflowState == REFLOW_STATE_IDLE) {                        //если держим долго Ok в режиме IDLE
    reflowState = REFLOW_STATE_MENU_STEPS;                       //то заходим в меню настроек
    updateScreen = true;                                         //обновляем LCD
  } else {
    reflowState = REFLOW_STATE_IDLE;
    updateScreen = true;
    SaveProfile();
    tone(buzzerPin, 1045, 200);                                  //звуковой сигнал
    lcd.clear();
    lcd.setCursor(2, 1);
    lcd.print(F("the User profile"));
    lcd.setCursor(5, 2);
    lcd.print(F("is a Saved!"));
    delay(2000);
  }
}

void PushUp() {                                                  //обработка короткого нажатия Up
  switch (reflowState) {                                                    //всё происходит внутри "машины состояний"
    case REFLOW_STATE_IDLE: {                                               //главное меню
        if (currentProfile <= 9) {
          currentProfile ++;                                                //увеличить № профиля на 1
        } else {
          currentProfile = 10;                                              //если текущий профиль > 10, нужно вернуться к профилю 10
        }
        loadProfile();                                                      //вызов функции loadProfile для загрузки данных профиля из eeprom
        break;
      }
    case REFLOW_STATE_MENU_STEPS: {                                         //устанавливаем количество шагов профиля
        if (profile.profileSteps <= 3) {
          profile.profileSteps ++;                                          //увеличить число шагов профиля на 1
        } else {
          profile.profileSteps = 4;                                         //если число шагов > 4, установить число шагов = 4
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_TARGET: {                                 //устанавливаем температуру "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.Setpoint2 <= 32) {
          profile.Setpoint2 ++;
        } else {
          profile.Setpoint2 = 8;                                            //деленная на 10
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_TARGET: {                                   //устанавливаем температуру "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.temperatureStep[editStep] <= 254) {
          profile.temperatureStep[editStep] ++;
        } else {
          profile.temperatureStep[editStep] = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_RAMP: {                                     //устанавливаем скорость нагрева "ВЕРХНИМ НАГРЕВАТЕЛЕМ"
        if (profile.rampRateStep[editStep] <= 29) {
          profile.rampRateStep[editStep] ++;
        } else {
          profile.rampRateStep[editStep] = 30;                              //максимальная скорость роста температуры умноженная на 10
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_DWELL: {                                    //устанавливаем время перехода на следующий шаг ("полочка" после уставки шага)
        if (profile.dwellTimerStep[editStep] <= 254) {
          profile.dwellTimerStep[editStep] ++;
        } else {
          profile.dwellTimerStep[editStep] = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_kf2: {                                    //настройка "kf2" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (kf2 <= 254) {
          kf2 ++;
        } else {
          kf2 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_P: {                                      //настройка "P" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kp2 <= 254) {
          profile.kp2 ++;
        } else {
          profile.kp2 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_I: {                                      //настройка "I" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.ki2 <= 254) {
          profile.ki2 ++;
        } else {
          profile.ki2 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_D: {                                      //настройка "D" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kd2 <= 254) {
          profile.kd2 ++;
        } else {
          profile.kd2 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_P: {                                         //настройка "P" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kp1 <= 254) {
          profile.kp1 ++;
        } else {
          profile.kp1 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_I: {                                         //настройка "I" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.ki1 <= 254) {
          profile.ki1 ++;
        } else {
          profile.ki1 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_D: {                                         //настройка "D" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kd1 <= 254) {
          profile.kd1 ++;
        } else {
          profile.kd1 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MIN:  {                               //устанавливаем минимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.min_pwr_BOTTOM <= 49) {
          profile.min_pwr_BOTTOM ++;
        } else {
          profile.min_pwr_BOTTOM = 50;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MAX:  {                               //устанавливаем максимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.max_pwr_BOTTOM <= 98) {
          profile.max_pwr_BOTTOM ++;
        } else {
          profile.max_pwr_BOTTOM = 99;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pTarg:  {                                 //устанавливаем t°C преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.pTarg <= 99) {
          profile.pTarg ++;
        } else {
          profile.pTarg = 100;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pPwr:  {                                  //устанавливаем мощность преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.pPwr <= 98) {
          profile.pPwr ++;
        } else {
          profile.pPwr = 99;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                                   //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.min_pwr_TOPStep[editStep] <= 49) {
          profile.min_pwr_TOPStep[editStep] ++;
        } else {
          profile.min_pwr_TOPStep[editStep] = 50;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                                   //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.max_pwr_TOPStep[editStep] <= 98) {
          profile.max_pwr_TOPStep[editStep] ++;
        } else {
          profile.max_pwr_TOPStep[editStep] = 99;
        }
        break;
      }
  }
}

void PushDown() {                                                //обработка короткого нажатия Down
  switch (reflowState) {                                                    //всё происходит внутри "машины состояний"
    case REFLOW_STATE_IDLE: {                                               //главное меню
        if (currentProfile >= 2) {
          currentProfile --;                                                //уменьшить № профиля на 1
        } else {
          currentProfile = 1;                                               //если № профиля <= 0, установить № профиля 14
        }
        loadProfile();                                                      //вызов функции loadProfile для загрузки данных профиля из eeprom
        break;
      }
    case REFLOW_STATE_MENU_STEPS: {                                         //устанавливаем количество шагов профиля
        if (profile.profileSteps >= 2) {
          profile.profileSteps --;
        } else {
          profile.profileSteps = 1;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_TARGET: {                                 //устанавливаем температуру "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.Setpoint2 >= 9) {
          profile.Setpoint2 --;
        } else {
          profile.Setpoint2 = 8;                                            //деленная на 10
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_TARGET: {                                   //устанавливаем температуру "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.temperatureStep[editStep] >= 1) {
          profile.temperatureStep[editStep] --;
        } else {
          profile.temperatureStep[editStep] = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_RAMP: {                                     //устанавливаем скорость нагрева "ВЕРХНИМ НАГРЕВАТЕЛЕМ"
        if (profile.rampRateStep[editStep] >= 1) {
          profile.rampRateStep[editStep] --;
        } else {
          profile.rampRateStep[editStep] = 0;                               //минимальная скорость роста температуры умноженная на 10
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_DWELL: {                                    //устанавливаем время перехода на следующий шаг ("полочка" после уставки шага)
        if (profile.dwellTimerStep[editStep] >= 1) {
          profile.dwellTimerStep[editStep] --;
        } else {
          profile.dwellTimerStep[editStep] = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_kf2: {                                    //настройка "kf2" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (kf2 >= 1) {
          kf2 --;
        } else {
          kf2 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_P: {                                      //настройка "P" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kp2 >= 1) {
          profile.kp2 --;
        } else {
          profile.kp2 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_I: {                                      //настройка "I" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.ki2 >= 1) {
          profile.ki2 --;
        } else {
          profile.ki2 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_D: {                                      //настройка "D" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kd2 >= 1) {
          profile.kd2 --;
        } else {
          profile.kd2 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_P: {                                         //настройка "P" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kp1 >= 1) {
          profile.kp1 --;
        } else {
          profile.kp1 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_I: {                                         //настройка "I" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.ki1 >= 1) {
          profile.ki1 --;
        } else {
          profile.ki1 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_D: {                                         //настройка "D" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.kd1 >= 1) {
          profile.kd1 --;
        } else {
          profile.kd1 = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MIN: {                                //устанавливаем минимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.min_pwr_BOTTOM >= 1) {
          profile.min_pwr_BOTTOM --;
        } else {
          profile.min_pwr_BOTTOM = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MAX: {                                //устанавливаем максимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.max_pwr_BOTTOM >= 1) {
          profile.max_pwr_BOTTOM --;
        } else {
          profile.max_pwr_BOTTOM = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pTarg:  {                                 //устанавливаем t°C преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.pTarg >= 1) {
          profile.pTarg --;
        } else {
          profile.pTarg = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pPwr:  {                                  //устанавливаем мощность преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        if (profile.pPwr >= 1) {
          profile.pPwr --;
        } else {
          profile.pPwr = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                                   //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.min_pwr_TOPStep[editStep] >= 1) {
          profile.min_pwr_TOPStep[editStep] --;
        } else {
          profile.min_pwr_TOPStep[editStep] = 0;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                                   //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (profile.max_pwr_TOPStep[editStep] >= 1) {
          profile.max_pwr_TOPStep[editStep] --;
        } else {
          profile.max_pwr_TOPStep[editStep] = 0;
        }
        break;
      }
  }
}

void PushRight() {                                               //обработка короткого нажатия Right
  switch (reflowState) {
    case REFLOW_STATE_IDLE: {                                               //главное меню, управляем охладителем платы кнопкой ВПРАВО
        digitalWrite(Ext_Fan, HIGH);                                        //включить охладитель платы
        break;
      }
    case REFLOW_STATE_MENU_STEPS: {                                         //устанавливаем количество шагов профиля
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_TARGET;
        lineForm(0, 2, 1);
        lineForm(0, 3, 1);
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_TARGET: {                                 //устанавливаем температуру "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_STEP_TARGET;
        break;
      }
    case REFLOW_STATE_MENU_STEP_TARGET: {                                   //устанавливаем температуру "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_RAMP;
        }  else {
          editStep ++;
          if (profile.temperatureStep[editStep] < profile.temperatureStep[editStep - 1])
            profile.temperatureStep[editStep] = profile.temperatureStep[editStep - 1];
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_RAMP: {                                     //устанавливаем скорость нагрева "ВЕРХНИМ НАГРЕВАТЕЛЕМ"
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_DWELL;
        }  else editStep ++;
        break;
      }
    case REFLOW_STATE_MENU_STEP_DWELL: {                                    //устанавливаем время перехода на следующий шаг ("полочка" после уставки шага)
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps)  {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_BOTTOM_kf2;
        }  else editStep ++;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_kf2: {                                    //настройка "kf2" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
        lineForm(0, 3, 1);
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_P: {                                      //настройка "P" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_I;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_I: {                                      //настройка "I" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_D;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_D: {                                      //настройка "D" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_P;
        break;
      }
    case REFLOW_STATE_MENU_TOP_P: {                                         //настройка "P" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_I;
        break;
      }
    case REFLOW_STATE_MENU_TOP_I: {                                         //настройка "I" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_D;
        break;
      }
    case REFLOW_STATE_MENU_TOP_D: {                                         //настройка "D" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MIN;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MIN:  {                               //устанавливаем мин. мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MAX;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MAX:  {                               //устанавливаем макс. мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_pTarg;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pTarg:  {                                 //устанавливаем t°C преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_pPwr;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pPwr:  {                                  //устанавливаем мощность преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_PWR_MIN;
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                                   //устанавливаем мин. мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_TOP_PWR_MAX;
        }  else editStep ++;
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                                   //устанавливаем макс. мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEPS;
          lineForm(0, 3, 1);
        }   else editStep ++;
        break;
      }
  }                                                                         //закрываем switch                                                                                        //конец обработки нажатия ВПРАВО
}                                                                           //закрываем PushRight

void PushLeft() {                                                //обработка короткого нажатия Left
  if (reflowState == REFLOW_STATE_STEP_RAMP || reflowState == REFLOW_STATE_STEP
      || reflowState == REFLOW_STATE_STEP_DWELL || reflowState == REFLOW_STATE_PRE_HEATER) {
    reflowState = REFLOW_STATE_COMPLETE;
    return;
  }
  switch (reflowState) {
    case REFLOW_STATE_IDLE: {                                               //главный экран, управляем охладителем платы кнопкой ВПРАВО
        digitalWrite(Ext_Fan, LOW);                                         //выключить охладитель платы
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                                   //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        if (editStep - 1 <= 0) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_TOP_PWR_MIN;
        }   else
          editStep --;
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                                   //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        if (editStep - 1 <= 0) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_BOTTOM_pPwr;                      //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
          lineForm(0, 2, 6);
          lineForm(0, 3, 7);
        }  else
          editStep --;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pPwr:  {                                  //устанавливаем мощность преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_pTarg;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_pTarg:  {                                 //устанавливаем t°C преднагрева для "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MAX;
        lineForm(0, 3, 8);
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MAX: {                                //устанавливаем макс. мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MIN;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_PWR_MIN: {                                //устанавливаем мин. мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_D;
        lineForm(0, 1, 5);
        lcd.setCursor(0, 2);
        lcd.print(F("TopHeater           "));
        lineForm(0, 3, 2);
        break;
      }
    case REFLOW_STATE_MENU_TOP_D: {                                         //настройка "D" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_I;
        break;
      }
    case REFLOW_STATE_MENU_TOP_I: {                                         //настройка "I" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_P;
        break;
      }
    case REFLOW_STATE_MENU_TOP_P:  {                                        //настройка "P" ПИД "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_D;
        lcd.setCursor(0, 2);
        lcd.print(F("BotHeater    kf2:"));
        lineForm(0, 3, 2);
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_D: {                                      //настройка "D" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_I;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_I: {                                      //настройка "I" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_P;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_P: {                                      //настройка "P" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_kf2;
        lineForm(0, 3, 1);
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_kf2: {                                    //настройка "kf2" ПИД "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_STEP_DWELL;
        lineForm(0, 1, 3);
        lcd.setCursor(0, 2);
        lcd.print(F("TopStep:   "));
        lineForm(0, 3, 1);
        break;
      }
    case REFLOW_STATE_MENU_STEP_DWELL: {                                    //устанавливаем время перехода на следующий шаг ("полочка" после уставки шага)
        updateScreen = true;
        if (editStep - 1 <= 0) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_RAMP;
        }  else
          editStep --;
        break;
      }
    case REFLOW_STATE_MENU_STEP_RAMP: {                                     //устанавливаем скорость нагрева "ВЕРХНИМ НАГРЕВАТЕЛЕМ"
        updateScreen = true;
        if (editStep - 1 <= 0) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_TARGET;
        }  else
          editStep --;
        break;
      }
    case REFLOW_STATE_MENU_STEP_TARGET: {                                   //устанавливаем температуру "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        if (editStep - 1 <= 0) {
          editStep = 0;
          lineForm(0, 2, 1);
          reflowState = REFLOW_STATE_MENU_BOTTOM_TARGET;
        }  else
          editStep --;
        break;
      }
    case REFLOW_STATE_MENU_BOTTOM_TARGET: {                                 //устанавливаем температуру "НИЖНЕГО НАГРЕВАТЕЛЯ"
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_STEPS;
        break;
      }
    case REFLOW_STATE_MENU_STEPS: {                                         //устанавливаем количество шагов профиля
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
        break;
      }
  }                                                                         //закрываем switch                                                                                        //конец обработки нажатия LEFT
}                                                                           //закрываем PushLeft

void PushLeftLong() {                                            //обработка длинного нажатия Left
  if (reflowState == REFLOW_STATE_IDLE) {                                   //держим долго Left в режиме IDLE
    reflowState = REFLOW_STATE_PROFILE_INIT;                                //и запускается инициализация предустановленного профиля
  }
}

void PushUpLong() {                                              //обработка длинного нажатия Up (пока в резерве)

}

void PushDownLong() {                                            //обработка длинного нажатия Down (пока в резерве)

}

void PushRightLong() {                                           //обработка длинного нажатия Right (пока в резерве)

}

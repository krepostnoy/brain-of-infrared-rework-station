//Крайнее редактирование 01.01.2023.
//---ProMini_LCD1602_ANALOG KEYBOARD SHIELD---//
#include <EEPROM.h>
#include "Setting.h"
#include "Variables.h"
#include <ClickBtnLong.h>                                       //библиотека аналоговой (резистивной) клавиатуры 
ClickBtnLong Btn_UP(PinKeyboard, PIN_UP, ABERRATION);           //передаём данные кнопок резистивной клавиатуры в библиотеку ClickBtnLong.h
ClickBtnLong Btn_DOWN(PinKeyboard, PIN_DOWN, ABERRATION);       //
ClickBtnLong Btn_OK(PinKeyboard, PIN_OK, ABERRATION);           //
#include <LiquidCrystal.h>
const int rs = 4, en = 5, d4 = 6, d5 = 7, d6 = 8, d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {                                                  //начальные установки
  Serial.begin(9600);
  loadProfile();                                                //вызов функции loadProfile для загрузки данных профиля из eeprom
  pinMode(2, INPUT_PULLUP);                                     //фиксируем состояние входя ZCC и подтягиваем его резистором к питанию
  pinMode(A6, INPUT);                                           //устанавливаем пин клавиатуры на ВХОД
  pinMode(thermoCStop, OUTPUT);                                 //задаём состояние пинов, к которым подключены MAX6675
  pinMode(thermoCLK, OUTPUT);                                   //
  pinMode(thermoSO, INPUT);                                     //
  pinMode(buzzerPin, OUTPUT);                                   //устанавливаем пин пищалки на ВЫХОД
  pinMode (Int_Fan, OUTPUT);                                    //устанавливаем пины охладителей на ВЫХОД
  pinMode (BH_OFF, OUTPUT);                                     //
  pinMode(RelayPin1, OUTPUT);                                   //задаём состояние пинов управления выходами
  Hello_guys();                                                 //сообщение приветствия
  SongHello();                                                  //Мелодия приветствия
  Developers();                                                 //разработчики
  /* настраиваем работу с прерыванием: LOW вызывает прерывание, когда на порту LOW
     CHANGE прерывание вызывается при смене значения на порту, с LOW на HIGH и наоборот
     RISING прерывание вызывается только при смене значения на порту с LOW на HIGH
     FALLING прерывание вызывается только при смене значения на порту с HIGH на LOW */
#ifdef SetInterrupt                                             //если используется схема ZCC
  attachInterrupt(SetInterrupt, Dimming, RISING);               //настроить порт прерывания (0) 2й цифровой пин
#else
  MsTimer2::set(10, Dimming);                                   //10ms period (настройка таймера)
  MsTimer2::start();                                            //если прерывания по таймеру, а не от схемы ZCC
#endif
}

void loop() {                                                   //основной цикл программы
  unsigned long currentMillis = millis();
  float out_float;                                              //вывод на LCD дробных чисел
  Btn_UP.run(&PushUp, &PushUpLong);                             //указываем обработчики для кнопки Up
  Btn_DOWN.run(&PushDown, &PushDownLong);                       //указываем обработчики для кнопки Down
  Btn_OK.run(&PushOk, &PushOkLong);                             //указываем обработчики для кнопки Ok

  switch (reflowState) {                                        //машина состояний (swich)
    case REFLOW_STATE_IDLE: {                                   //главный экран
        TopStart = false;                                       //излучатель выключен
        digitalWrite(BH_OFF, HIGH);                             //выключить НИ
        digitalWrite(Int_Fan, LOW);                             //выключить охладитель симисторов
        currentStep = 1;                                        //установлен шаг №1
        counter = 0;                                            //счетчик обнулён
        setpointRamp = 0;                                       //скорость прироста t°C = 0
        x = 1;                                                  //устанавливаем переменную в исходное состояние
        flag = 0;                                               //после отановки профиля сбрасываем флаг
        Output1 = 0;                                            //низкий уровень на выходе управления излучателем
        if (millis() > nextRead) {
          nextRead = millis() + SENSOR_SAMPLING_TIME;           //устраняем колебания датчиков температуры
          TempRead();                                           //считываем информацию с преобразователя термопары
          if (kluch == 4) {                                     //готовим данные для вывода на ПК
            kluch = 0;                                          //
            BufferUpload();                                     //выводим данные в COM-порт
          }
          if (kluch == 2) TempDisplay();                        //выводим температуры на LCD
          kluch++;
        }
        if (updateScreen) {                                     //Настройка экрана в режиме ожидания
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("PTN:"));
          lcd.setCursor(6, 0);
          lcd.print(F("OFF"));
          lcd.setCursor(10, 0);
          lcd.print(F("ST:1"));
          lcd.setCursor(0, 1);
          lcd.print(F("hSP:"));
          lcd.setCursor(10, 1);
          lcd.print(F("PV:"));
          updateScreen = false;
        }
        lcd.setCursor(4, 0);
        lcd.print(currentProfile);
        lcd.print(F(" "));
        lcd.setCursor(4, 1);
        lcd.print(profile.temperatureStep[0]);
        lcd.print(F(" "));
      } break;

    case REFLOW_STATE_PROFILE_INIT: {                           //инициализация тестового профиля
        /* 4 шага,
           уставки ВИ по шагам 180,190,200,225
           скорости роста t° по шагам 10,3,3,6
           длительности полок по шагам 10,15,30,15
           P.I.D TOP 10,1,40
           pwr PREHEAT 5%
           min TOP PWR по шагам 10%,10%,10%,10%
           max TOP PWR по шагам 99%,99%,99%,99% */

        profile = {4, 180, 190, 200, 225, 10, 3, 3, 6, 10, 15, 30, 15, 10, 1, 40, 5, 10, 10, 10, 10, 99, 99, 99, 99};
        SaveProfile();
        tone(buzzerPin, 1045, 500);                              //звуковой сигнал при сбросе профиля
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print(F("preset profile "));
        lcd.setCursor(1, 1);
        lcd.print(F("is a uploaded!"));
        delay(2000);
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      } break;

    case REFLOW_STATE_MENU_STEPS: {                             //устанавливаем количество шагов профиля
        if (updateScreen) {
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print(F("Change Setting"));
          lcd.setCursor(0, 1);
          lcd.print(F("Steps in PTN:"));
          updateScreen = false;
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        lcd.print(profile.profileSteps);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_STEP_TARGET: {                       //устанавливаем температуру ВИ
        if (updateScreen) {
          lcd.setCursor(0, 1);
          lcd.print(F("St:"));
          lcd.print(editStep + 1);
          lcd.setCursor(4, 1);
          lcd.print(F("  Target:  "));
          updateScreen = false;
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        lcd.print(profile.temperatureStep[editStep]);
        lcd.print(F("  "));
      }  break;

    case REFLOW_STATE_MENU_STEP_RAMP: {                         //устанавливаем скорость нагрева ВИ
        if (updateScreen) {
          lcd.setCursor(0, 1);
          lcd.print(F("St:"));
          lcd.print(editStep + 1);
          lcd.setCursor(4, 1);
          lcd.print(F("    Ramp:"));
          updateScreen = false;
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        out_float = profile.rampRateStep[editStep] / 10.0;
        lcd.print(out_float, 1);
        lcd.print(F(" "));
      }  break;

    case REFLOW_STATE_MENU_STEP_DWELL: {                        //устанавливаем время перехода на следующий шаг
        if (updateScreen) {
          lcd.setCursor(0, 1);
          lcd.print(F("St:"));
          lcd.print(editStep + 1);
          lcd.setCursor(4, 1);
          lcd.print(F("   Dwell:  "));
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        lcd.print(profile.dwellTimerStep[editStep]);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_TOP_P: {                             //настройка "P" ПИД верхнего нагревателя
        if (updateScreen) {
          lcd.setCursor(0, 1);
          lcd.print(F(" p"));
          updateScreen = false;
        }
        Skobki(0, !OkSet);
        lcd.setCursor(2, 1);
        lcd.print(profile.kp1);
        lcd.print(F("             "));
      }  break;
    case REFLOW_STATE_MENU_TOP_I: {                             //настройка "I" ПИД верхнего нагревателя
        Skobki(0, !OkSet);
        lcd.setCursor(6, 1);
        lcd.print(F("i"));
        lcd.print(profile.ki1);
        lcd.print(F("  "));
      }  break;
    case REFLOW_STATE_MENU_TOP_D: {                             //настройка "D" ПИД верхнего нагревателя
        Skobki(0, !OkSet);
        lcd.setCursor(11, 1);
        lcd.print(F("d"));
        lcd.print(profile.kd1);
        lcd.print(F("  "));
      } break;

    case REFLOW_STATE_MENU_PREHEAT_PWR: {                       //устанавливаем мощность ВИ на этапе преднагрева
        if (updateScreen) {
          lcd.setCursor(0, 1);
          lcd.print(F("PREHEAT  PWR:"));
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        lcd.print(profile.pwr_PREHEAT);
        lcd.print(F("%"));
        lcd.print(F(" "));
      } break;

    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                       //устанавливаем минимальную мощность ВИ
        if (updateScreen) {
          lcd.setCursor(0, 1);
          lcd.print(F("St:"));
          lcd.print(editStep + 1);
          lcd.setCursor(4, 1);
          lcd.print(F(" min PWR:"));
          updateScreen = false;
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        lcd.print(profile.min_pwr_TOPStep[editStep]);
        lcd.print(F("%"));
        lcd.print(F(" "));
      } break;

    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                       //устанавливаем максимальную мощность ВИ
        if (updateScreen) {
          lcd.setCursor(3, 1);
          lcd.print(editStep + 1);
          lcd.setCursor(4, 1);
          lcd.print(F(" max "));
          updateScreen = false;
        }
        Skobki(0, !OkSet);
        lcd.setCursor(13, 1);
        lcd.print(profile.max_pwr_TOPStep[editStep]);
        lcd.print(F("%"));
        lcd.print(F(" "));
      } break;

    case REFLOW_STATE_PREHEAT: {                                //Подогрев ВИ до момента TopStart = true
        if (tc1 < degree_Celsius && !TopStart) {                //если t° на датчике ВИ НЕ достигла degree_Celsius включаем преднагрев
          Output1 = profile.pwr_PREHEAT;
        } else {
          reflowState = REFLOW_STATE_STEP_RAMP;
        }
      } break;

    case REFLOW_STATE_STEP_RAMP: {                              //"процесс пайки", рост температуры с заданной скоростью
        digitalWrite(Int_Fan, HIGH);                             //включить охладитель симисторов и контроллера
        if (tc1 >= degree_Celsius && !TopStart) TopStart = true; //если температура на датчике ВИ достигла degree_Celsius включаем ВИ
        if (TopStart == true)                                    //если включаем верхний нагреватель:
        { //открываем скобку (чтоб не запутаться)
          if (flag == 0) {                                       //фиксируем стартовую температуру
            startTemp = tc1;
            flag = 1;
          }
          if (startTemp > profile.temperatureStep[currentStep - 1]) {     //устанавливаем нужный шаг, до которого нагрета плата
            for (x = 1; startTemp > profile.temperatureStep[currentStep - 1]; currentStep++) {
              x++;
            }
          }
          if (currentStep > x && flag == 1) {
            flag = 0;
            startTemp = profile.temperatureStep[currentStep - 2];
            flag = 1;
          }
          lcd.setCursor(13, 0);
          lcd.print(currentStep);
          if ((currentMillis - previousMillis) > 1000 / (profile.rampRateStep[currentStep - 1] * 0.1)) { //счётчик скорости роста t° от 0.1с. до 3с.
            previousMillis = currentMillis;
            counter = counter + 1;
            setpointRamp = counter + startTemp;
            lcd.setCursor(4, 1);
            lcd.print(setpointRamp);
            lcd.print(F("  "));
            Setpoint1 = setpointRamp;
          }
        }                                                                 //закрывам скобку
        if (setpointRamp >= profile.temperatureStep[currentStep - 1]) {   //если достигли нужной температуры
          lcd.setCursor(4, 1);
          lcd.print(profile.temperatureStep[currentStep - 1]);
          reflowState = REFLOW_STATE_STEP;
        }
      }  break;
    case REFLOW_STATE_STEP: {
        Setpoint1 = profile.temperatureStep[currentStep - 1];
        if (Input1 >= profile.temperatureStep[currentStep - 1]) {
          counter = 0;
          reflowState = REFLOW_STATE_STEP_DWELL;
        }
      }  break;

    case REFLOW_STATE_STEP_DWELL: {                             //считаем время перехода на следующий шаг
        if (currentMillis - previousMillis > 1000) {
          previousMillis = currentMillis;
          counter = counter + 1;
        }
        //if (counter >= profile.dwellTimerStep[currentStep - 1]) {       //если счётчик больше установленного времени
        /* Строка выше является верной! Для обеспечения возможности использовать контроллер
           для управления не только верхним излучателем, а, при необходимости, вместо этого,
           для управления нагревом и долговременным удержанием заданной уставки термостолом,
           не имевщим изначально таких функций, немного изменено условие в этой строке. Теперь
           при задании длительности любого шага равной 0, профиль никогда не сможет завершиться.
           Это не совсем правильно, но так сделано для упрощения кода. Измененная строка
           расположена ниже этого текстового блока */
        if (counter == profile.dwellTimerStep[currentStep - 1]) {         //если счётчик равен установленному времени
          tone(buzzerPin, 1045, 500);  //звуковой сигнал
          counter = 0;
          setpointRamp = 0;
          if (profile.profileSteps == currentStep) {            //если достигли последнего шага
            currentStep = 1;
            x = 1;                                              //устанавливаем переменную в исходное состояние
            flag = 0;                                           //после завершения профиля сбрасываем флаг
            reflowState = REFLOW_STATE_COMPLETE;
          } else {                                              //если шаг не последний
            currentStep++;                                      //переходим на следующий шаг
            reflowState = REFLOW_STATE_STEP_RAMP;
          }
        }
      } break;

    case REFLOW_STATE_COMPLETE: {                               //завершение пайки
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_IDLE;
        updateScreen = true;
        SongAlarm();
      } break;
  }                                                             //здесь закончился switch

  if (reflowStatus == REFLOW_STATUS_ON) {                       //включение нагревателей
    if (updateScreen) {
      lcd.setCursor(6, 0);
      lcd.print(F("RUN"));
      updateScreen = false;
    }
    if (millis() > nextRead) {
      nextRead = millis() + SENSOR_SAMPLING_TIME;               //считываем данные с термопар и считаем ПИД
      TempRead();                                               //считываем данные с термопар
      Count1++;                                                 //увеличиваем счетчик Count1
      chast = Count1 / 4;
      kluch = Count1 % 4;
      if (TopStart) Output1 = Pid1(Input1, Setpoint1, profile.kp1, profile.ki1, profile.kd1); else Output1 = 0;
    }
    if (kluch == 2) {                                           //готовим данные
      kluch = 0;
#ifdef SppBigdata
      BufferUpload1();                                          //выводим больше данных в COM-порт
#else
      BufferUpload();                                           //выводим данные в COM-порт
#endif
    }
    if (kluch == 3) {
      TempDisplay();                                            //выводим температуры на LCD
    }
  } else
    digitalWrite(RelayPin1, LOW);
}                                                               //конец основного цикла loop программы

void Dimming() {                                                //что делает диммер
  if (reflowStatus == REFLOW_STATUS_ON) {
    Outpwr_TOP();
  }
}

void Outpwr_TOP() {                                             //расчет мощности
  reg1 = Output1 + er1;                                         //er- ошибка округления
  if (reg1 < 50) {
    out1 = LOW;
    er1 = reg1 ;                                                //reg- переменная для расчетов
  } else {
    out1 = HIGH;
    er1 = reg1 - 100;
  }
  digitalWrite(RelayPin1, out1);                                //пин через который осуществляется дискретное управление
}

byte Pid1(double temp, byte ust, byte kP, byte kI, byte kd) {   //ПИД по измерению t° - нет выбросов мощности
  byte out = 0;
  static float ed = 0;
  e = (ust - temp);                                             //ошибка регулирования
  p = (kP * e) / 10;                                            //П составляющая с делителем на 10
  integra = (integra < i_min) ? i_min : (integra > i_max) ? i_max : integra + (kI * e) / 1000; //И составляющая с делителем на 1000
  d = kd * (temp - ed);                                         //Д составляющая
  ed = temp;
  out = (p + integra - d < profile.min_pwr_TOPStep[currentStep - 1]) ? profile.min_pwr_TOPStep[currentStep - 1] : (p + integra - d > profile.max_pwr_TOPStep[currentStep - 1]) ? profile.max_pwr_TOPStep[currentStep - 1] : p + integra - d;
  return out;
}

double max6675_read_temp (int ck, int cs, int so) {             //MAX6675 functions
  char i;
  int tmp = 0;
  digitalWrite(cs, LOW);//cs = 0;                               //Stop a conversion in progress
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
    return 250;
  } else
    return ((tmp >> 3)) * 0.25;
}

void loadProfile() {                                            //считывание профиля из EEPROM
  EEPROM.get((currentProfile - 1)*SizeProfile, profile);
}
void SaveProfile () {                                           //сохранение текущего профиля
  EEPROM.put((currentProfile - 1)*SizeProfile, profile);
}

void TempRead() {                                               //опрос модуля MAX6675
  Input1  = Input1 * 0.6 + 0.4 * (max6675_read_temp(thermoCLK, thermoCStop, thermoSO));   //считываем данные с преобразователя термопары
  tc1 = Input1;
  if (tc1 >= 250 && TopStart) {
    reflowState = REFLOW_STATE_COMPLETE;
  }
}

void TempDisplay() {                                            //вывод температуры на LCD
  lcd.setCursor(13, 1);
  lcd.print(F("   "));
  lcd.setCursor(13, 1);
  lcd.print(tc1);
}

void BufferUpload() {                                           //готовим данные для вывода на ПК
#ifdef SerialPortPlotter
  sprintf (buf, "$%03d %03d %03d %03d;", int(Output1), int(Output2), tc1, tc2);     //стандартный набор данных
#else
  sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2, currentProfile);
#endif
  Serial.print(buf);
}

void BufferUpload1() {                                           //готовим данные для вывода на ПК
#ifdef SerialPortPlotter
  sprintf (buf, "$%03d %03d %03d %03d;", int(Output1), int(Output2), tc1, tc2, int(p), int(integra), int(d));     //расширенный набор данных
#else
  sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2, currentProfile);
#endif
  Serial.print(buf);
}

void Hello_guys() {                                             //сообщение приветствия
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("ARDUINO REWORK"));
  lcd.setCursor(0, 1);
  lcd.print(F("v1.4_Top Channel"));
  delay (3000);
}

void Developers() {                                             //разработчики
  lcd.setCursor(0, 0);
  lcd.print(F("   amperka.ru   "));
  lcd.setCursor(0, 1);
  lcd.print(F("vector99 Cinema "));
  delay (2000);

  lcd.setCursor(0, 0);
  lcd.print(F("Roniks59 Watashi"));
  lcd.setCursor(0, 1);
  lcd.print(F("DmitrySh SOLOway "));
  delay (2000);
}

void SongHello() {                                              //мелодия приветствия
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

void SongAlarm() {                                              //мелодия предупреждения
  tone(buzzerPin, 1046);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1046);
  delay(100);
  noTone(buzzerPin);
}

void Skobki(byte str, bool key) {
  if (key) {
    lcd.setCursor(0, str);
    lcd.print(">");
    lcd.setCursor(15, str);
    lcd.print("<");
  }
  else {
    lcd.setCursor(0, str);
    lcd.print(" ");
    lcd.setCursor(15, str);
    lcd.print(" ");
  }
}

void PushOk() {                                                 //обработка короткого нажатия Ok
  if (reflowState == REFLOW_STATE_IDLE) {
    tone(buzzerPin, 1045, 500);                                 //звуковой сигнал при старте профиля
    updateScreen = true;
#ifdef SerialPortPlotter                                        //если ПО SerialPortPlotter
    sprintf (buf, "$#");
#else                                                           //если ПО irsp.exe от @Dmitrysh
    sprintf (buf, "SYNC\r\n");
#endif
    Serial.println(buf);
    nextRead = millis();
    //digitalWrite(Ext_Fan, LOW);                                 //выключить охлаждение платы
    digitalWrite(Int_Fan, HIGH);                                //включить охладитель симисторов и контроллера
    digitalWrite(BH_OFF, LOW);                                  //включить НИ
    integra = 0;
    p = 0;
    d = 0;
    Count1 = 0;
    previousMillis = millis();
    reflowStatus = REFLOW_STATUS_ON;
    reflowState = REFLOW_STATE_PREHEAT;
    updateScreen = true;
  } else {
    if (reflowState >= REFLOW_STATE_PREHEAT && reflowState <= REFLOW_STATE_STEP_DWELL) {
      reflowState = REFLOW_STATE_COMPLETE;
      reflowStatus = REFLOW_STATUS_OFF;
      return;
    }
  }
  if (reflowState >= REFLOW_STATE_MENU_STEPS && reflowState <= REFLOW_STATE_MENU_TOP_PWR_MAX) {
    OkSet = !OkSet;
    return;
  }
}

void PushOkLong() {                                             //обработка длинного нажатия Ok
  if (reflowState >= REFLOW_STATE_PREHEAT && reflowState <= REFLOW_STATE_STEP_DWELL) { //ничего не делаем в этих состояниях
    return;
  }
  if (reflowState == REFLOW_STATE_IDLE) {                       //если держим долго Ok в режиме IDLE
    reflowState = REFLOW_STATE_MENU_STEPS;                      //то заходим в меню настроек
    updateScreen = true;                                        //обновляем LCD
  }
  else {
    reflowState = REFLOW_STATE_IDLE;
    updateScreen = true;
    SaveProfile();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("parameters Saved"));
    delay(2000);
  }
}

void PushUp() {                                                 //обработка короткого нажатия Up
  switch (reflowState) {
    case REFLOW_STATE_IDLE: {                                   //главное меню
        currentProfile++;
        if (currentProfile >= 6) currentProfile = 1;            //если текущий профиль > 5, нужно вернуться к профилю 1
        loadProfile();                                          //вызов функции loadProfile для загрузки данных профиля из eeprom
        break;
      }
    case REFLOW_STATE_MENU_STEPS: {                             //устанавливаем количество шагов профиля
        if (OkSet) {
          updateScreen = true;
          reflowState = REFLOW_STATE_MENU_STEP_TARGET;
        } else {
          profile.profileSteps++;
          if (profile.profileSteps >= 5) profile.profileSteps = 4;
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_TARGET: {                       //устанавливаем температуру "Верхнего Нагревателя"
        if (OkSet) {
          updateScreen = true;
          if (editStep + 1 == profile.profileSteps) {
            editStep = 0;
            reflowState = REFLOW_STATE_MENU_STEP_RAMP;
          }  else {
            editStep++;
            if (profile.temperatureStep[editStep] < profile.temperatureStep[editStep - 1])
              profile.temperatureStep[editStep] = profile.temperatureStep[editStep - 1];
          }
        } else {
          profile.temperatureStep[editStep] ++;
          if (profile.temperatureStep[editStep] >= 256)  profile.temperatureStep[editStep] = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_RAMP: {                         //устанавливаем скорость нагрева "Верхним Нагревателем"
        if (OkSet) {
          updateScreen = true;
          if (editStep + 1 == profile.profileSteps) {
            editStep = 0;
            reflowState = REFLOW_STATE_MENU_STEP_DWELL;
          }  else {
            editStep++;
          }
        }  else {
          profile.rampRateStep[editStep] ++;
          if (profile.rampRateStep[editStep] >= 31)  profile.rampRateStep[editStep] = 30;     //максимальная скорость роста температуры умноженная на 10
        }
        break;
      }
    case REFLOW_STATE_MENU_STEP_DWELL: {                        //устанавливаем время перехода на следующий шаг (длительность шага)
        if (OkSet) {
          updateScreen = true;
          if (editStep + 1 == profile.profileSteps)  {
            editStep = 0;
            reflowState = REFLOW_STATE_MENU_TOP_P;
          }  else {
            editStep++;
          }
        }  else {
          profile.dwellTimerStep[editStep] ++;
          if (profile.dwellTimerStep[editStep] >= 256)   profile.dwellTimerStep[editStep] = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_P: {                             //настройка "P" ПИД верхнего нагревателя
        if (OkSet) {
          updateScreen = true;
          reflowState = REFLOW_STATE_MENU_TOP_I;
        }  else {
          profile.kp1 ++;
          if (profile.kp1 >= 256)  profile.kp1 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_I: {                             //настройка "I" ПИД верхнего нагревателя
        if (OkSet) {
          updateScreen = true;
          reflowState = REFLOW_STATE_MENU_TOP_D;
        }  else {
          profile.ki1 ++;
          if (profile.ki1 >= 256)  profile.ki1 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_D: {                             //настройка "D" ПИД верхнего нагревателя
        if (OkSet) {
          updateScreen = true;
          reflowState = REFLOW_STATE_MENU_PREHEAT_PWR;
        }  else {
          profile.kd1 ++;
          if (profile.kd1 >= 256)  profile.kd1 = 255;
        }
        break;
      }
    case REFLOW_STATE_MENU_PREHEAT_PWR: {                       //установка мощности "ВЕРХНЕГО НАГРЕВАТЕЛЯ" на этапе преднагрева
        if (OkSet) {
          updateScreen = true;
          reflowState = REFLOW_STATE_MENU_TOP_PWR_MIN;
        }  else {
          profile.pwr_PREHEAT ++;
          if (profile.pwr_PREHEAT >= 26)  profile.pwr_PREHEAT = 25;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                       //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (OkSet) {
          updateScreen = true;
          if (editStep + 1 == profile.profileSteps) {
            editStep = 0;
            reflowState = REFLOW_STATE_MENU_TOP_PWR_MAX;
          }  else {
            editStep++;
          }
        }  else {
          profile.min_pwr_TOPStep[editStep] ++;
          if (profile.min_pwr_TOPStep[editStep] >= 100) profile.min_pwr_TOPStep[editStep] = 99;
        }
        break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                       //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (OkSet) {
          updateScreen = true;
          if (editStep + 1 == profile.profileSteps) {
            editStep = 0;
            reflowState = REFLOW_STATE_MENU_STEPS;
          }   else {
            editStep++;
          }
        }  else {
          profile.max_pwr_TOPStep[editStep] ++;
          if (profile.max_pwr_TOPStep[editStep] >= 100) profile.max_pwr_TOPStep[editStep] = 99;
        }
        break;
      }
  }                                                             //закрываем switch
}                                                               //конец обработки нажатия UP

void PushDown() {                                               //обработка короткого нажатия Down
  switch (reflowState) {
    case REFLOW_STATE_IDLE: {                                   //главное меню
        currentProfile --;
        if (currentProfile <= 0) currentProfile = 1;
        loadProfile();                                          //вызов функции loadProfile для загрузки данных профиля из eeprom
        break;
      }
    case REFLOW_STATE_MENU_STEPS: {                             //устанавливаем количество шагов профиля
        if (!OkSet) {
          profile.profileSteps --;
          if (profile.profileSteps <= 0) profile.profileSteps = 1;
        } break;
      }
    case REFLOW_STATE_MENU_STEP_TARGET: {                       //устанавливаем температуру "Верхнего Нагревателя"
        if (!OkSet) {
          profile.temperatureStep[editStep] --;
          if (profile.temperatureStep[editStep] <= 0)  profile.temperatureStep[editStep] = 0;
          if (editStep > 0 && profile.temperatureStep[editStep] < profile.temperatureStep[editStep - 1])
            profile.temperatureStep[editStep] = profile.temperatureStep[editStep - 1];
        } break;
      }
    case REFLOW_STATE_MENU_STEP_RAMP: {                         //устанавливаем скорость нагрева "Верхним Нагревателем"
        if (!OkSet) {
          profile.rampRateStep[editStep] --;
          if (profile.rampRateStep[editStep] <= 0)  profile.rampRateStep[editStep] = 1;       //минимальная скорость роста температуры умноженная на 10
        } break;
      }
    case REFLOW_STATE_MENU_STEP_DWELL: {                        //устанавливаем время перехода на следующий шаг (длительность шага)
        if (!OkSet) {
          profile.dwellTimerStep[editStep] --;
          if (profile.dwellTimerStep[editStep] <= 0)  profile.dwellTimerStep[editStep] = 0;
        } break;
      }
    case REFLOW_STATE_MENU_TOP_P: {                             //настройка "P" ПИД верхнего нагревателя
        if (!OkSet) {
          profile.kp1 --;
          if (profile.kp1 <= 0)  profile.kp1 = 0;
        } break;
      }
    case REFLOW_STATE_MENU_TOP_I: {                             //настройка "I" ПИД верхнего нагревателя
        if (!OkSet) {
          profile.ki1 --;
          if (profile.ki1 <= 0)  profile.ki1 = 0;
        } break;
      }
    case REFLOW_STATE_MENU_TOP_D: {                             //настройка "D" ПИД верхнего нагревателя
        if (!OkSet) {
          profile.kd1 --;
          if (profile.kd1 <= 0)  profile.kd1 = 0;
        } break;
      }
    case REFLOW_STATE_MENU_PREHEAT_PWR: {                       //устанавливаем мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ" на этапе преднагрева
        if (!OkSet) {
          profile.pwr_PREHEAT --;
          if (profile.pwr_PREHEAT <= 0)  profile.pwr_PREHEAT = 0;
        } break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MIN: {                       //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (!OkSet) {
          profile.min_pwr_TOPStep[editStep] --;
          if (profile.min_pwr_TOPStep[editStep] <= 0) profile.min_pwr_TOPStep[editStep] = 0;
        } break;
      }
    case REFLOW_STATE_MENU_TOP_PWR_MAX: {                       //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
        if (!OkSet) {
          profile.max_pwr_TOPStep[editStep] --;
          if (profile.max_pwr_TOPStep[editStep] <= 0)  profile.max_pwr_TOPStep[editStep] = 0;
        } break;
      }
  }
}

void PushUpLong() {                                             //обработка длинного нажатия Up (пока в резерве)
  if (reflowState == REFLOW_STATE_PREHEAT || reflowState == REFLOW_STATE_STEP_RAMP)  { //включаем ВИ вручную, если идёт пайка
    TopStart = true;
    return;
  }
}

void PushDownLong() {                                           //обработка длинного нажатия Left
  if (reflowState == REFLOW_STATE_IDLE) {                       //держим долго Left в режиме IDLE
    reflowState = REFLOW_STATE_PROFILE_INIT;                    //и запускается инициализация предустановленного профиля
  }
}

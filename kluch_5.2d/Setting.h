#define resist_keyboard 1                         //раскоментировать, если клавиатура резистивная
#define SerialPortPlotter 1                       //раскоментировать, если ПО SerialPortPlotter
//#define inert_heater 1                            //Раскоментировать для инерционного НИ
#define LongBufferPid 1                           //раскоментировать для инерционного НИ
#define delta 10                                  //за сколько °С до уставки НИ включится ВИ
#define TimePrint 1                               //раскомментировать для вывода на LCD секундомера при пайке

/* Выбор источника прерывания! если детектор нуля не используется - закомментировать оба варианта
  (#define SetInterrupt 0 и #define SetInterrupt 1) и будет использоваться программный таймер
  (нужна библиотека MsTimer2.h) скачать по ссылке https://github.com/PaulStoffregen/MsTimer2 */
#define SetInterrupt 0                          //назначение пина детектора нуля ZCC (если используется): подключение к пину 2
//#define SetInterrupt 1                          //назначение пина детектора нуля ZCC (если используется): подключение к пину 3
#ifndef SetInterrupt                              //если не используется схема ZCC
#include <MsTimer2.h>                             //библиотека таймера для работы без схемы ZCC
#endif

#define RelayPin1 10                              //назначаем пин "ВЕРХНЕГО" нагревателя
#define RelayPin2 11                              //назначаем пин "НИЖНЕГО" нагревателя
#define buzzerPin 13                              //назначаем пин пищалки
#define Int_Fan  4                                //назначаем пин охладителя симисторов и контроллера
#define Ext_Fan  5                                //назначаем пин охладителя платы
#define thermoCStop 6                             //CS MAX6675 "ВЕРХНЕГО" канала
#define thermoCSbott 7                            //CS MAX6675 "НИЖНЕГО" канала
#define thermoSO 8                                //DO=(SO) преобразователей термопар MAX6675
#define thermoCLK  9                              //SCK=(CLK) преобразователей термопар MAX6675
#define SENSOR_SAMPLING_TIME 250                  //частота обновления текущей температуры мсек.(1000=1 раз в секунду)

#ifdef resist_keyboard                            //если клавиатура резистивная
#define PinKeyboard A6                            //назначаем пин подключения клавиатуры
//#define PIN_UP 32                                 //значение на кнопке Up
//#define PIN_DOWN 90                               //значение на кнопке DOWN
//#define PIN_LEFT 0                                //значение на кнопке LEFT
//#define PIN_RIGHT 170                             //значение на кнопке RIGHT
//#define PIN_OK 360                                //значение на кнопке OK
#define PIN_UP 184
#define PIN_DOWN 0
#define PIN_LEFT 367
#define PIN_RIGHT 510
#define PIN_OK 250
#define ABERRATION 30
#else
#define PIN_UP A0                                 //пин одиночной кнопки Up
#define PIN_DOWN  A1                              //пин одиночной кнопки DOWN
#define PIN_LEFT  A2                              //пин одиночной кнопки LEFT
#define PIN_RIGHT  A3                             //пин одиночной кнопки RIGHT
#define PIN_OK 12                                 //пин одиночной кнопки OK
#endif

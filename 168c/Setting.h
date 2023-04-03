#define SerialPortPlotter 1        //раскоментировать, если ПО SerialPortPlotter
#define degree_Celsius 130         //температура по датчику ВИ, при которой включится старт пайки
#define SppBigdata 1               //раскомментировать, если ПО SerialPortPlotter и нужен вывод составляющих P.I.D. на время настрорйки работы ВИ
/* Выбор источника прерывания: если детектор нуля не используется - закомментировать "#define SetInterrupt 0",
   то будет использоваться программный таймер (нужна библиотека MsTimer2.h)
   скачать по ссылке https://github.com/PaulStoffregen/MsTimer2 */
//#define SetInterrupt 0             //раскоментировать, если используется детектор нуля ZCC (подключен к пину 2)
#ifndef SetInterrupt               //если не используется схема ZCC
#include <MsTimer2.h>              //библиотека таймера для работы без схемы ZCC
#endif

char buf[32];                      //буфер для вывода в UART

#define RelayPin1 3                //назначаем пин "ВЕРХНЕГО" нагревателя
#define buzzerPin A3               //назначаем пин пищалки
#define Int_Fan  A2                //назначаем пин охладителя симисторов и контроллера
//#define Ext_Fan  A5                //назначаем пин охладителя платы
#define BH_OFF  A0                 //назначаем пин отключателя НИ после завершения профиля
#define thermoCLK  10              //SCK=(CLK) модуля MAX6675 - преобразователя сигнала термопары
#define thermoCStop  11            //CS модуля MAX6675 - преобразователя сигнала термопары
#define thermoSO  12               //DO=(SO) модуля MAX6675 - преобразователя сигнала термопары

#define PinKeyboard A5             //назначаем пин подключения клавиатуры
#define PIN_UP 345                 //значение на кнопке Up
#define PIN_DOWN 412               //значение на кнопке DOWN
#define PIN_OK 212                 //значение на кнопке OK
#define ABERRATION 30              //допустимое отклонение значения потенциала кнопки

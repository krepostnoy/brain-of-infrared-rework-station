# brain-of-infrared-rework-station
This project aims to create an inexpensive controller for controlling a contactless soldering system using infrared radiation.
Корни этого проекта направляются к видеоролику по ссылке: 
(https://www.youtube.com/watch?v=_YlqswsgasU) и, немного позднее, в Грецию по ссылке 
(http://liliumjsn.blogspot.com/2014/10/arduino-rework-station.html).
Масштабная попытка воскресить и развить идеи этого проекта сделана в теме "ИК паяльная станция 
на Arduino Mega 2560. Доработка скетча "ARS_v2_Lilium_JSN"" по ссылке (https://clck.ru/JNqKg). 
В разработке этого устройства принимали непосредственное участие в темах по ссылке 
(https://clck.ru/JNqKg) следующие участники: 
@vector99 (http://forum.amperka.ru/members/vector99.11198/),
@xake (http://forum.amperka.ru/members/xake.11647/),
@Roniks59 (http://forum.amperka.ru/members/roniks59.15321/), 
@Dmitrysh (http://forum.amperka.ru/members/dmitrysh.14292/),
@SNMar4enko (http://forum.amperka.ru/members/snmar4enko.17088/),
@revolover (http://forum.amperka.ru/members/revolover.16485/),
@alsh_0907 (http://forum.amperka.ru/members/alsh_0907.16464/),
@Aleksander1997 (http://forum.amperka.ru/members/aleksander1997.16111/),
@Watashi (http://forum.amperka.ru/members/watashi.19093/),
@geleos27 (http://forum.amperka.ru/members/geleos27.28967/).
Для упрощения процесса разработки, вместо отдельных радиокомпонентов, используются модули: 
плата arduino NANO (AtMega328p/16MHz/5v), LCD 2004 i2c, 2 штуки MAX6675 module for arduino, 
вместо 5 штук одиночных тактовых кнопок можно использовать клавиатурный 
модуль Keyes yes_AD_key K845037.
В качестве основы устройства выбран интегральный 8-битный микроконтроллер AtMega328p 
производства Atmel/Microchip.
Дисплей на контроллере HD44780 используется символьный, монохромный, 4-строчный, по 20 символов 
в каждой строке. 
Дисплей подключается к микроконтроллеру по шине i2c через переходник на микросхеме PCF8574. 
Разрабатываемое устройство имеет 2 канала измерения температуры и 2 канала управления 
нагревательными элементами станции. 
В качестве преобразователей сигналов датчиков температуры применены 2 микросхемы MAX6675. 
В качестве датчиков используются 2 термопары K-Type.
Устройство ввода - 5 одиночных нормально разомкнутых тактовых кнопок, или клавиатура на 
принципе резистивного делителя напряжения, которая подключается к аналоговому входу 
микроконтроллера.
Устройство имеет 2 выхода для подключения силовых твердотельных реле типа SSR-40DA (либо их 
самодельных аналогов на основе оптотриака и мощного симметричного тиристора), через которые 
управляются нагреватели паяльной станции с использованием алгоритма Брезенхема для равномерного 
распределения управляющих импульсов по периоду синусоиды питающей паяльную станцию сети. Для 
повышения точности управления имеется возможность использования внешнего прерывания от схемы 
(Zero Crossing Control) контроля за пересечением нуля вольт синусоидой напряжения питающей 
паяльную станцию сети (в этом случае предпочтительнее, но не обязательно, использование 
самодельных аналогов твердотельных реле на основе оптотриака, не имеющего встроенный контроль 
за переходом синусоиды через ноль). При использовании покупных твердотельных реле типа SSR-40DA 
возможно обойтись без использования внешней схемы Zero Crossing Control, для этого в прошивке 
перед компиляцией нужно выбрать прерывание по таймеру в AtMega328p с управлением библиотекой 
MsTimer2.h (https://github.com/PaulStoffregen/MsTimer2). 
Также имеются 2 выхода для управления двумя вентиляторами (либо группами вентиляторов) через 
модуль электромагнитного реле, либо модуль на мощном полевом транзисторе, либо их 
самодельный аналог.
Управляющие сигналы на всех выходах устройства имеют TTL уровни.
Устройство хранит в энергонезависимой памяти AtMega328p до 10 профилей пайки по 1-4 шага в 
каждом. Каждый профиль можно редактировать с сохранением внесённых изменений. Имеется 
возможность инициализации тестового, встроенного в прошивку, профиля для получения 
представления о значениях параметров в меню устройства. Для каждого профиля можно использовать 
свои собственные значения коэффициентов ПИД, и это позволяет без проблем использовать на 
паяльной станции нагреватели-излучатели различных конструкций и мощностей.
Управление нагревом излучателей паяльной станции производится с помощью алгоритма на основе 
Пропорционально-Интегрально-Дифференциального принципа. 
Для удобства настройки станции реализована возможность вывода различных данных, в том числе для 
построения графиков "температура-время" из устройства в программы на персональные компьютеры 
под управлением операционных систем Windows от Microsoft. Программы, с которыми производится 
тестирование разрабатываемое устройство:
SerialPortPlotter (https://github.com/CieNTi/serial_port_plotter) в редакции @geleos27 
(https://github.com/geleos27/serial_port_plotter) 
(https://drive.google.com/drive/folders/1MJODRkzjEhu9_pDdhrgusmGDYtArU51c), а также программа 
irsp.exe от @Dmitrysh (https://clck.ru/WZCTk), которая выросла из программы Heater.exe 
(https://drive.google.com/file/d/1ybs_o17qxBp1C3WeMLrRBQr2mTUntQIp/view) от @hominidae 
(https://mysku.ru/blog/aliexpress/47529.html).

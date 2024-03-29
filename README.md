# P10-animation
Целью этой работы является создание анимации вращающегося куба с помощью микроконтроллера STM32F429ZITx и подсоединенной к нему светодиодной матрицы LED P10. 
## Описание работы матрицы  
![p10-led-matrix-module](https://github.com/LugenderGeist/P10-animation/assets/155521631/77afae54-c5bd-41d4-b9bd-e27f91a2f568)  
Первым делом необходимо изучить выбранную матрицу, в нашем случае LED P10:  
- Размерность 16х32 светодиода
- Матрица монохромная
- Питается от 5В постоянного тока
- Протокол передачи данных - SPI, о котором подробнее будет написано ниже  

![снимок01](https://github.com/LugenderGeist/P10-animation/assets/155521631/2655e0c6-e822-40b6-b940-67c5264a5b3f)  
На рисунке выше представлена обратная сторона матрицы. На ней расположены:
- 1 x SM74HC245D — неинвертирующий буфер
- 1 x SM74HC04 — 6-канальный инвертор
- 1 x SM74HC138D — 8-битный дешифратор
- 4 x APM4953 — сборка из 2 P-канальных MOSFET
- 16 x 74HC595D — сдвиговый регистр с защёлкой  

Два 16-пиновых разъёма — интерфейсные, один из них входной (к нему подключается контроллер экрана), а второй — выходной (к нему подключается следующая матрица в цепочке). Стрелка на плате направлена от входного разъёма к выходному.
Питание подаётся на клеммы в центре платы. Напряжение питания — 5В, максимальный ток (когда включены все светодиодны матрицы) — 2А (для белой матрицы).  
(*Стоит сказать, что информация выше взята со [статьи на хабре](https://habr.com/ru/articles/372215/). Матрицы могут отличаться внешне, но принцип работы у них одинаковый, так что эта информация актуальна для любой LED матрицы.*)  
Перейдем к электрической схеме:  
![P10_12](https://github.com/LugenderGeist/P10-animation/assets/155521631/9cd37edf-07c7-456a-bbfa-be69bec6876c)  
Схема состоит из двух генераторов – генератора с регулировкой скважности и обычного тактового генератора. Первый генератор, изменяя величину скважности сигнала OE с помощью переменного сопротивления R2, позволяет изменять яркость светодиодов модуля. Второй генератор формирует основные сигналы CKL и SCLK для внутренней электронной схемы модуля, которые обеспечивают последовательный ввод, сдвиг и вывод информации на светодиоды. Переменное сопротивление R5 изменяет частоту второго генератора и соответственно скорость сдвига информации в сдвиговых регистрах модуля.  
По положительному фронту сигнала CKL происходит ввод информации через вход R и сдвиг её по всем разрядам всех регистров модуля (в электронной схеме модуля используются мс 74HC595). По положительному фронту сигнала SCLK сдвигающаяся информация отображается на светодиодах модуля через каждые половину периода после очередного сдвига, что удобно наблюдать по состоянию светодиода VD4.  
(*Здесь уже информация взята из [другой статьи](https://sotvorimvmeste.ru/viewtopic.php?p=613), но в целом, опять же, принцип работа матриц очень похож, если не одинаков, несмотря на возможное отличие в принципиальных схемах.*)  
## Протокол SPI
Как уже было указано выше, данные матрица передает по протоколу SPI, соответственно микроконтроллер обладает возможностью работать по этому протоколу.  
SPI (Serial Peripheral Interface в переводе шина для подключения периферийных устройств) - последовательный синхронный стандарт передачи данных в режиме полного дуплекса, предназначенный для обеспечения простого и недорогого высокоскоростного сопряжения микроконтроллеров и периферии.  
Главное назначение протокола - связать одно главное устройство - Ведущее (Master) - с одним или несколькими Ведомыми (Slave). Ведущий в этом интерфейсе всегда один, только он руководит всем процессом и только он может формировать тактовые импульсы. В нашем случае Ведущим является микроконтроллер, а Ведомыми - LED матрица. Особенно этот интерфейс востребован там, где требуется высокая скорость передачи данных и не менее высокая надежность. Расплатой за это является использование большего количеств проводов, чем для других интерфейсов, а именно:
- MOSI - Master Output Slave Input (Ведущий передает, Ведомый принимает),
- MISO - Master Input Slave Output (Ведущий принимает, Ведомый передает) (*не используется в этой работе*)
- SCLK, иначе SCK - Serial Clock (тактовый сигнал).  

Передача осуществляется пакетами. Длина пакета, как правило, составляет 1 байт (8 бит). Ведущее устройство инициирует цикл связи установкой низкого уровня на выводе выбора подчиненного устройства, с которым необходимо установить соединение. При низком уровне сигнала:
- схемотехника ведомого устройства находится в активном состоянии
- вывод MISO переводится в режим «выход»
- тактовый сигнал SCLK от ведущего устройства воспринимается ведомым и вызывает считывание на входе MOSI значений передаваемых от ведущего битов и сдвиг регистра ведомого устройства  

Схематично принцип передачи данных изображен ниже:  
![Снимок4](https://github.com/LugenderGeist/P10-animation/assets/155521631/8a942177-5624-4514-8022-06be14d4903c)  
Подлежащие передаче данные ведущее и ведомое устройства помещают в сдвиговые регистры. После этого ведущее устройство начинает генерировать импульсы синхронизации на линии SCLK, что приводит к взаимному обмену данными. Передача данных осуществляется бит за битом от ведущего по линии MOSI и от ведомого по линии MISO. Передача осуществляется, как правило, начиная со старших битов. После передачи каждого пакета данных ведущее устройство, в целях синхронизации ведомого устройства, может перевести линию SS в высокое состояние.  

## Подключение к микроконтроллеру
Изучим распиновку кабеля матрицы.  
![Снимок](https://github.com/LugenderGeist/P10-animation/assets/155521631/3d142539-613a-451b-b602-192be63437b0)  
На рисунке видно, что на кабеле подключения матрицы нам нужны 7 пинов: пин nOE - разрешает работу матрицы, один из пинов GND - земля, пины A и B - их комбинации определяют, какая из четвертей матрицы будет работать, пины CLK и R - линии клока и данных синхронного последовательного интерфейса. Они подключаются к SCK и MOSI соответственно интерфейса SPI микроконтроллера. Пин SCLK - по переднему фронту защелкивает переданные в сдвиговые регистры данные на их выходы.  
В работе будет использован микроконтроллер STM32F429ZITx. В среде STM32CubeMX были выбраны все необходимые порты, для подключения LED матрицы, включая порты для передачи данных по протоколу SPI. Названия портов на схеме соответствует пинам на кабеле подключения.  
![Снимок1](https://github.com/LugenderGeist/P10-animation/assets/155521631/f36ed925-9b04-4e23-8a45-0a77de58fc87)  
Также для создания анимации был подключен таймер. На рисунке ниже представлены настройки таймера: был выбран таймер TIM1, который с помощью коэффициентов Prescaler и Counter Period был настроен на период прерывания в 1 секунду. (При условии что HCLK равен 180 МГц.)  
![Снимок2](https://github.com/LugenderGeist/P10-animation/assets/155521631/e6ca94a5-a4f2-4276-b456-76db7734b878)  
## Написание кода
Основной целью работы было понять, как создать простую анимацию вращения куба.  
Первым делом была подключена библиотека P10, разработанная для упрощения отрисовки простых элементов (точки, прямой, прямоугольника). Было принято решение задать вершины куба, которые будут соединяться линиями, отрисованными с помощью библиотеки. Для этого нужно задать массив, в котором будут содержаться координаты вершин: всего их 8.  
Далее были написаны функции для обновления четвертей экрана. Их всего 4 и в каждой из них:
- Устанавливается лог. 0 на ножке nOE.
- Устанавливается лог. уровни на ножках A и B в соответствии с обновляемой группой светодиодов (одной из четырех). 
- По SPI выдаются данные для сдвиговых регистров. Для одной матрицы 32x16 это 16 байт (16 8-битных регистров).
- На ножку SCLK подается короткий положительный импульс. Это подает землю на катоды светодиодов в соответствии с загруженными в регистры байтами.
- Устанавливается лог. 1 на ножке nOE. При этом четверть панели работает до следующего обновления следующей группы светодиодов.  


В функции таймера в файле stm32f4xx_it.c был записан алгоритм, по которому изменяются коэфициенты, которые будут изменять координаты вершин. Их блок-схемы:    
![Снимок5](https://github.com/LugenderGeist/P10-animation/assets/155521631/d2dfd285-2b37-416a-9a1e-4df2bd7437d0)  

В файле main.c после этого было описано влияние коэфициентов на вершины. Блок-схема основного кода:  
![Снимок6](https://github.com/LugenderGeist/P10-animation/assets/155521631/fd9f7b8b-0959-4d1e-b1ee-691a02d3d281)  

## Результат работы
![Новый проект](https://github.com/LugenderGeist/P10-animation/assets/155521631/c1a3d3e4-a46c-4f43-88af-0b1bb3b6fe40)  
(*Из-за того, что частота, с которой снимает телефон не совпадает с частотой обновления матрицы - кажется, что что-то не так. В жизни это выглядит намного лучше.*)

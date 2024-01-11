# P10-animation

## Задание  
С помощью микроконтроллера STM32F429ZITx и подсоединенной к нему светодиодной матрицы LED P10 создать анимацию вращающегося куба. Для передачи данных использовать протокол SPI.  
## LED матрица
### Описание работы матрицы
![p10-led-matrix-module](https://github.com/LugenderGeist/P10-animation/assets/155521631/77afae54-c5bd-41d4-b9bd-e27f91a2f568)  
Первым делом необходимо изучить выбранную матрицу, в нашем случае LED P10:  
- Размерность 16х32 светодиода
- Матрица монохромная
- Питается от 5В постоянного тока
- Протокол передачи данных - SPI, о котором подробнее будет написано ниже
Сама работа матрицы:
![снимок01](https://github.com/LugenderGeist/P10-animation/assets/155521631/2655e0c6-e822-40b6-b940-67c5264a5b3f)  
На рисунке выше представлена обратная сторона матрицы. На ней расположены:
- 1 x SM74HC245D — неинвертирующий буфер
- 1 x SM74HC04 — 6-канальный инвертор
- 1 x SM74HC138D — 8-битный дешифратор
- 4 x APM4953 — сборка из 2 P-канальных MOSFET
- 16 x 74HC595D — сдвиговый регистр с защёлкой
Два 16-пиновых разъёма — интерфейсные, один из них входной (к нему подключается контроллер экрана), а второй — выходной (к нему подключается следующая матрица в цепочке). Стрелка на плате направлена от входного разъёма к выходному.
Питание подаётся на клеммы в центре платы. Напряжение питания — 5В, максимальный ток (когда включены все светодиодны матрицы) — 2А (для белой матрицы).
*Стоит сказать, что информация выше взята со [статьи на хабре](https://habr.com/ru/articles/372215/)*
### Протокол SPI
SPI (Serial Peripheral Interface) - последовательный синхронный стандарт передачи данных в режиме полного дуплекса, предназначенный для обеспечения простого и недорогого высокоскоростного сопряжения микроконтроллеров и периферии.  
## Подключение к микроконтроллеру
Изучим наспиновку кабеля матрицы.
![Снимок](https://github.com/LugenderGeist/P10-animation/assets/155521631/3d142539-613a-451b-b602-192be63437b0)  
На рисунке видно, что на кабеле подключения матрицы нам нужны 7 пинов: пин nOE - разрешает работу матрицы, один из пинов GND - земля, пины A и B - их комбинации определяют, какая из четвертей матрицы будет работать, пины CLK и R - линии клока и данных синхронного последовательного интерфейса. Они подключаются к SCK и MOSI соответственно интерфейса SPI микроконтроллера. Пин SCLK - по переднему фронту защелкивает переданные в сдвиговые регистры данные на их выходы.  
В работе будет использован микроконтроллер STM32F429ZITx. В среде STM32CubeMX были выбраны все необходимые порты, для подключения LED матрицы, включая порты для передачи данных по протоколу SPI. Названия портов на схеме соответствует пинам на кабеле подключения.  
![Снимок1](https://github.com/LugenderGeist/P10-animation/assets/155521631/f36ed925-9b04-4e23-8a45-0a77de58fc87)  
Также для создания анимации был подключен таймер. На рисунке ниже представлены настройки таймера: был выбран таймер TIM1, который с помощью коэффициентов Prescaler и Counter Period был настроен на период прерывания в 1 секунду. (При условии что HCLK равен 180 МГц.)  
![Снимок2](https://github.com/LugenderGeist/P10-animation/assets/155521631/e6ca94a5-a4f2-4276-b456-76db7734b878)  
## Написание кода

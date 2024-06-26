# inverted
**Ознакомительная версия проекта "Голосовой помощник на колесах"**   

Данный проект находится на этапе разработки. На данный момент:   
- написан алгоритм сбора и обработки данных с датчиков (C, МК: STM32F103C8T6): 
    - 10 DOF IMU Sensor (акселерометр, гироскоп)
    - датчики Холла (энкодер);
    - INA219 (датчик тока);
- частично проработана визуальная часть web-приложения для взаимодействия с роботом (HTML, CSS);
- написана программа для обработки речи пользователя (Python).

Наименование корневого каталога «inverted» объясняется тем, что изначально целью являлось достижение баланса системы. Двухколёсный робот – система, представляющая собой перевернутый маятник (inverted pendulum). Баланс такой системы достигается путем определения положения системы в пространстве по показаниям датчиков и оказания на систему компенсирующего воздействия (вращение колес) в случае ее отклонения от положения равновесия.   
Баланса системы удалось достичь, и, более того, удалось достичь управления системой по Bluetooth с компьютера через программу LabVIEW, но система не обладала достаточной устойчивостью, в связи с чем алгоритм проекта претерпел существенные изменения. Поскольку ранее поставленная цель была достигнута, хоть и не в полной мере, было принято решение определить новую цель, более амбициозную.   
По ходу работы над проектом и достижения новых результатов материалы на GitHub будут корректироваться и дополняться.

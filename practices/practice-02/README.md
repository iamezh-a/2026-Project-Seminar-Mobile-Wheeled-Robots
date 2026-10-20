
# Практическая работа №2 — Знакомство с Webots

**Вариант:** 03

## Описание

В рамках практической работы создан ROS 2-пакет `webots_digits` для симуляции робота TurtleBot3 Burger в Webots.

Сцена содержит:

- Арену размером 8 × 8 метров с полом и стенами.
- Два статических препятствия в форме цифр `0` и `3`.
- Отдельные PROTO-файлы для каждой цифры.
- Робота TurtleBot3 Burger, управляемого через ROS 2.

Препятствия имеют серый цвет и физическую геометрию `boundingObject`. Узел `Physics` не используется, поэтому препятствия неподвижны.

## Структура проекта

```text
webots_digits/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── webots_digits
├── launch/
│   └── spawn_robot.launch.py
├── worlds/
│   └── digits_world.wbt
└── protos/
    ├── Digit0.proto
    └── Digit3.proto
```

## Сборка

Команды выполняются в подготовленном окружении ROS 2 Jazzy внутри Docker-контейнера курса.

Перейти в рабочее пространство:

```bash
cd ~/practices_ws
```

Собрать пакет:

```bash
colcon build --packages-select webots_digits --symlink-install
```

Подключить окружение:

```bash
source ~/practices_ws/install/local_setup.zsh
```

## Запуск симуляции

Убедиться, что сервер Webots запущен.

В терминале Docker выполнить:

```bash
ros2 launch webots_digits spawn_robot.launch.py
```

После запуска открывается сцена `digits_world.wbt` с ареной, цифрами `03` и роботом TurtleBot3.

## Управление роботом

Открыть второй терминал внутри Docker и подключить окружение:

```bash
source ~/practices_ws/install/local_setup.zsh
```

Для движения робота по окружности выполнить:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
"{twist: {linear: {x: 0.2}, angular: {z: 0.08}}}"
```

Команда задаёт постоянную линейную скорость 0,2 м/с и угловую скорость 0,08 рад/с.

Расчётный радиус траектории:

R = v / ω = 0,2 / 0,08 = 2,5 м.

Робот движется по окружности вокруг обеих цифр.

Для прекращения публикации команд нажать `Ctrl + C`.

Для явной остановки робота выполнить:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
"{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

## Результат

Создана сцена с двумя статическими препятствиями в форме числа `03`. Робот TurtleBot3 запускается через ROS 2 и выполняет круговое движение вокруг обоих препятствий.

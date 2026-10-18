# Practice 01

Вариант: 03.

Программа рисует цифры 0 и 3 с помощью двух черепах в turtlesim.

Для управления обеими черепахами используется один ROS 2 узел `digit_controller`.

Имя черепахи и цифра передаются в узел через ROS-параметры.

## Сборка

Внутри ROS 2 Docker-контейнера выполнить:


cd ~/practices_ws
colcon build --symlink-install
source ~/practices_ws/install/local_setup.zsh

## Запуск

После сборки внутри ROS 2 Docker-контейнера выполнить:


ros2 launch turtle_digits digits.launch.py

После запуска автоматически:

1. запускается `turtlesim`;
2. удаляется стандартная черепаха `turtle1`;
3. создаётся `turtle_zero`;
4. создаётся `turtle_three`;
5. запускаются два экземпляра `digit_controller`;
6. черепахи рисуют цифры 0 и 3.

После завершения рисования управляющие узлы продолжают работать и публикуют нулевую скорость.

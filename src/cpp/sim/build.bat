g++ main.cpp ^
    ..\nml_hand_exo\nml_hand_exo.cpp ^
    ..\nml_hand_exo\utils.cpp ^
    ..\nml_hand_exo\gesture_controller.cpp ^
    ..\nml_hand_exo\gesture_library.cpp ^
    shim\oled.cpp ^
    -I.\shim ^
    -I..\nml_hand_exo ^
    -std=c++17 -g -o hand_exo_sim.exe

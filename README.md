# microros_pico_ws
Demo workspace for [this tutorial](https://youtu.be/MBKAZ_2P1Sk) on working with micro-ROS on Raspberry Pi Pico. 

You can run `colcon build` from the repo root.

The `src` directory contains two packages (`button_tester` and `my_interfaces`), and there is a separate directory containing the files that should be modified from the [micro-ROS Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk). 


## button_tester

This program subscribes to `/pico_publisher` to receive button presses and publishes to `/button_color` to send the colours.

`ros2 run button_tester button_tester`

## my_interfaces

Containes a single service definition (`SetColor.srv`) and the CMakeLists.txt to build it.

## Changes for the micro-ROS Pico SDK

As per the video, you should clone and build [micro-ROS Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) somewhere else. The `microros_different_bits` directory in this repo, combined with the notes below, covers the changes discussed in the tutorial.

### extern C for pico_uart_transports.h

Adds the following to allow compilation under both C and C++.
```
... original header guards here ...

#ifdef __cplusplus
extern "C"
{
#endif

... original content here ...

#ifdef __cplusplus
}
#endif

... end header guard ...
```

### CMakeLists.txt

Differences to original:

- Add `set(PICO_CXX_ENABLE_EXCEPTIONS 1)` before initialisation.
- Add `include("PicoLED/PicoLed.cmake")` to include PicoLed library.
- Rename `pico_micro_ros_example.c` to `pico_micro_ros_example.cpp`.
- Add `PicoLed` to `target_link_libraries.

### pico_micro_ros_example.cpp

The main file worked on in the video. It is based on `pico_micro_ros_example.c` but renamed with the `.cpp` extension and new code added.

### PicoLED Library
Clone PicoLED into the root directory.

### Add my_interfaces
Place in `microros_static_library/library_generation/extra_packages/` and then rebuild with the Docker command shown in the video.





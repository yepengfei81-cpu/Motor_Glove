# OpenRB-150 高速桥接固件

编译 / 烧录（板子在哪个口用 `arduino-cli board list` 看）：

    arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 firmware/openrb_bridge
    arduino-cli upload  --fqbn OpenRB-150:samd:OpenRB-150 -p /dev/ttyACM2 firmware/openrb_bridge

烧录后板子会重新枚举，串口号可能变，`arduino-cli board list` 再看一次。
刷回原厂桥接：

    cp -r ~/.arduino15/packages/OpenRB-150/hardware/samd/0.2.1/libraries/OpenRB-150/examples/usb_to_dynamixel /tmp/
    arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 /tmp/usb_to_dynamixel
    arduino-cli upload  --fqbn OpenRB-150:samd:OpenRB-150 -p /dev/ttyACM2 /tmp/usb_to_dynamixel

烧录失败/板子没反应：双击 RESET 进 bootloader（LED 慢闪），再 upload 一次。bootloader 在受保护区，sketch 刷不坏它。

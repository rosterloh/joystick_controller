# ESP32S3 Joystick Controller Application

[Adafruit QT Py ESP32-S3](https://learn.adafruit.com/adafruit-qt-py-esp32-s3/overview)

### Initialisation

The first step is to initialise the workspace folder (``esp_ws``) where
the ``joystick_controller`` and all Zephyr modules will be cloned. Run the following
command:

```shell
west init -m https://github.com/rosterloh/joystick_controller --mr main esp_ws
cd esp_ws
west update
west blobs fetch hal_espressif
```

### Building

All commands are implemented as VSCode tasks. press ctrl+shift+b for the build tasks menu

### Debugging

ESP32-S3 support on OpenOCD is available upstream as of version 0.12.0. Download and install OpenOCD from [OpenOCD](https://github.com/openocd-org/openocd)

ESP32-S3 has a built-in JTAG circuitry and can be debugged without any additional chip. Only an USB cable connected to the D+/D- pins is necessary.

Further documentation can be obtained from the SoC vendor in [JTAG debugging for ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/jtag-debugging/)

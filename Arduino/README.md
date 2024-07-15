# Arduino
Programming the microcontroller was done using [PlatformIO](https://platformio.org/) - Arduino framework.

TODO Before uploading the firmware to the microcrontroller, we have to set a few configuration parameters.

I use a custom made script that bundles a few of the PlatformIO together, though in short, these are the steps to be run to program the microcontroller:
```bash
platformio pkg update
platformio run
platformio run --target upload
platformio device monitor --port /dev/O_ttyUSB0 --baud 115200 | ts [%Y-%m-%d %H:%M:%S] | tee -a MainControllerTeensy.log
```

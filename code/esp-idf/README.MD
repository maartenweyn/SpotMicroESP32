ESP-IDF

Installation and tutorials:

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-5-start-a-project



Initiate directory with 

```get_idf```

then for the first time

```
idf.py set-target esp32
idf.py menuconfig

```

then

```
idf.py build
idf.py -p /dev/tty.SLAB_USBtoUART flash
idf.py -p /dev/tty.SLAB_USBtoUART monitor

```






| Supported Targets | ESP32 |
| ----------------- | ----- |

ESP-IDF Gatt Server Demo
========================

This is the demo of APIs to create a GATT service by adding attributes one by one. However, this method is defined by Bluedroid and is difficult for users to use.

Hence, we also allow users to create a GATT service with an attribute table, which releases the user from adding attributes one by one. And it is recommended for users to use. For more information about this method, please refer to [gatt_server_service_table_demo](../gatt_server_service_table).

This demo creates GATT a service and then starts advertising, waiting to be connected to a GATT client. 

To test this demo, we can run the [gatt_client_demo](../gatt_client), which can scan for and connect to this demo automatically. They will start exchanging data once the GATT client has enabled the notification function of the GATT server.

Please check the [tutorial](tutorial/Gatt_Server_Example_Walkthrough.md) for more information about this example.

python /home/name/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 /home/name/esp32/ozono/build/bootloader/bootloader.bin 0x10000 /home/name/esp32/ozono/build/ozono.bin 0x8000 /home/name/esp32/ozono/build/partitions_singleapp.bin

----> flash app only 
python /home/name/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 480600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x10000 ./build/gatt_server_demos.bin
python /home/name/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB1 --baud 230400 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x10000 ./build/gatt_server_demos.bin

------> monitor
~/esp-idf/tools/idf_monitor.py --port /dev/ttyUSB1 ~/esp32/ozono/build/ozono.elf
~/esp-idf/tools/idf_monitor.py --port /dev/ttyUSB2 ~/esp32/ozono/build/ozono.elf
~/esp-idf/tools/idf_monitor.py --port /dev/ttyUSB3 ~/esp32/ozono/build/ozono.elf




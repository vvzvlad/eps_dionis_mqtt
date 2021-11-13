main: build upload

#fqbn:
#1) "--fqbn", not "-fqbn"
#2) format fqbn lib:cpu:board:param1,param2

#

build:
	arduino-cli compile --libraries ./ --fqbn=esp8266:esp8266:d1_mini:xtal=80,baud=921600 mqtt2i2c

upload:
	arduino-cli upload --port /dev/tty.usbserial* --fqbn=esp8266:esp8266:d1_mini:baud=921600 mqtt2i2c

miniterm:
	miniterm.py /dev/tty.usbserial* 115200

clean:
	arduino-cli cache clean

install_env:
	arduino-cli core update-index --additional-urls "http://arduino.esp8266.com/stable/package_esp8266com_index.json"
	arduino-cli core install esp8266:esp8266 --additional-urls "http://arduino.esp8266.com/stable/package_esp8266com_index.json"
  arduino-cli lib install "EspMQTTClient"






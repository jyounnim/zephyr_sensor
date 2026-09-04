# sysbuild.cmake - builds core0 (this app, procpu) and core1 (remote/,
# appcpu) as two independent Zephyr images for one ESP32-S3 chip.
#
# Build with:
#   west build -p always --sysbuild -b esp32s3_devkitc/esp32s3/procpu \
#       01_AHT20_BMP280_MultiSensor/lab
#
# Pattern follows this project's existing ESP32-S3 IPM lab (roadmap
# Lab 18 / samples/drivers/ipm/ipm_esp32 style).

ExternalZephyrProject_Add(
	APPLICATION remote
	SOURCE_DIR ${APP_DIR}/remote
	BOARD esp32s3_devkitc/esp32s3/appcpu
)

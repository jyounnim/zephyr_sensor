# sysbuild.cmake - builds core0 (this app, procpu) and core1 (remote/,
# appcpu) as two independent Zephyr images for one ESP32-S3 chip.
#
# Build with:
#   west build -p always --sysbuild -b esp32s3_devkitc/esp32s3/procpu \
#       02_GY521_MPU6050/lab

ExternalZephyrProject_Add(
	APPLICATION remote
	SOURCE_DIR ${APP_DIR}/remote
	BOARD esp32s3_devkitc/esp32s3/appcpu
)

# Overlays for the Odroid platform

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
	can0.dtbo \
	dht11.dtbo \
	i2c0.dtbo \
	i2c1.dtbo \
	mcp2515.dtbo \
	mcp2515_12mhz.dtbo \
	mcp2515_16mhz.dtbo \
	onewire.dtbo \
	pcf8563.dtbo \
	pwm1.dtbo \
	pwm2.dtbo \
	pwm9.dtbo \
	rknpu.dtbo \
	spi0.dtbo \
	ttyfiq0_115200.dtbo \
	uart0-with-ctsrts.dtbo \
	uart0.dtbo \
	uart1.dtbo

targets += $(dtbo-y)
always-y := $(dtbo-y)

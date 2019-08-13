# ASUS EC driver

ccflags-y := -Werror

obj-$(CONFIG_INPUT_ASUSEC)		+= asus-ec.o
asus-ec-y				= dockram.o controller.o tf_charger.o

obj-$(CONFIG_ASUSPEC)			+= asuspec.o
obj-$(CONFIG_ASUSDEC)			+= asusdec.o elantech.o
obj-$(CONFIG_BATTERY_PAD)		+= pad_battery.o

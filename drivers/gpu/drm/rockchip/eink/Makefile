obj-$(CONFIG_SOFTWARE_EBC)   += software_ebc.o

obj-$(CONFIG_EBC)   += ebc_public.o
obj-$(CONFIG_EBC)   += px30_tcon.o rk3288_tcon.o
obj-$(CONFIG_EBC)   += epdlut/
ifdef CONFIG_ARM64
obj-$(CONFIG_EBC)   += check_auto_image_sARM_arm64.o
else
obj-$(CONFIG_EBC)   += check_auto_image_sARM.o
endif
obj-$(CONFIG_EBC)   += bufmanage/
obj-$(CONFIG_EBC)   += bootani/
obj-$(CONFIG_EBC)   += ebc_dbg/
obj-$(CONFIG_EBC)   += epdpower/

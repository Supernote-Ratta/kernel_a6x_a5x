#obj-$(CONFIG_SOFTWARE_EBC)   += software_ebc.o

#obj-$(CONFIG_EBC)   += ebc_public.o
#obj-$(CONFIG_EBC)   += px30_tcon.o rk3288_tcon.o
#obj-$(CONFIG_EBC)   += epdlut/
#obj-$(CONFIG_EBC)   += bufmanage/
#obj-$(CONFIG_EBC)   += bootani/
#obj-$(CONFIG_EBC)   += ebc_dbg/
#obj-$(CONFIG_EBC)   += epdpower/

obj-$(CONFIG_EBC)   += rk_ebc.o

$(obj)/rk_ebc.o: $(srctree)/$(obj)/rk_ebc.px
	cp $(srctree)/$(obj)/rk_ebc.px $(obj)/rk_ebc.o


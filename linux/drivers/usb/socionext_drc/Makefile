obj-$(CONFIG_USB_F_USB20HDC)	+= f_usb20hdc_drc.o
f_usb20hdc_drc-y			:= f_usb20hdc_phy.o

ifneq ($(filter y,$(CONFIG_USB_F_USB20HDC_HOST_ONLY) $(CONFIG_USB_F_USB20HDC_DUAL_ROLE)),)
	f_usb20hdc_drc-y		+= f_usb20hdc-hcd.o
endif

ifneq ($(filter y,$(CONFIG_USB_F_USB20HDC_GADGET_ONLY) $(CONFIG_USB_F_USB20HDC_DUAL_ROLE)),)
	f_usb20hdc_drc-y		+= f_usb20hdc_udc.o
endif

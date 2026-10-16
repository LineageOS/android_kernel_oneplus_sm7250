#
# Makefile for kernel Silead Fingerprint drivers.
#

ifdef CONFIG_ARCH_QCOM
	 ccflags-y += -DBSP_SIL_PLAT_QCOM
endif

ifeq ($(CONFIG_OPLUS_FINGERPRINT_QCOM),y)
 obj-y	+= silead_fp_platform.o
endif





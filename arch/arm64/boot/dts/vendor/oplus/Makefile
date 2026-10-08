ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
    dtbo-$(CONFIG_ARCH_LITO) += \
        lagoon-ebba-overlay.dtbo \
        lito-avicii-overlay.dtbo

    lagoon-ebba-overlay.dtbo-base := lagoon.dtb
    lito-avicii-overlay.dtbo-base := lito.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo

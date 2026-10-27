# SPDX-License-Identifier: GPL-2.0-only

ccflags-y += -I$(srctree)/techpack/camera-oplus/include/uapi
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_core
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_cpas/include
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_req_mgr
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_sensor_module/cam_cci
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_smmu/
ccflags-y += -I$(srctree)/techpack/camera-oplus/drivers/cam_utils
ccflags-y += -I$(srctree)/techpack/camera-oplus/oplus

obj-$(CONFIG_SPECTRA_CAMERA) += hardware_info.o project_info.o
#obj-$(CONFIG_SPECTRA_CAMERA) += param_read_write.o boot_mode.o device_info.o

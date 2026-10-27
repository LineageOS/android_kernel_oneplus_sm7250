#ifndef __OPLUS_NFC_H__
#define __OPLUS_NFC_H__

#include <stdbool.h>

#define CHECK_NFC_CHIP(chip) \
pr_err("%s : enter\n", __func__); \
if (!is_support_chip(chip)) { \
    pr_err("%s device not support, exit\n", __func__); \
    return -ENOMEM; \
}

/*
 * Family check for the nq-nci driver: passes if the chipset named in the
 * device tree (chipset-<project>) is any chip nq-nci can drive, so the
 * specific chip no longer has to be hardcoded in nq-nci.c.
 */
#define CHECK_NFC_NQ_FAMILY() \
pr_err("%s : enter\n", __func__); \
if (!is_nq_nci_chip()) { \
    pr_err("%s device not support, exit\n", __func__); \
    return -ENODEV; \
}

typedef enum{
    UNKNOWN = 0,
    NQ330,
    SN100T,
    SN100F,
    ST21H,
    ST54H,
    PN557
} chip_type;

bool is_support_chip(chip_type chip);
bool is_nq_nci_chip(void);
bool is_nfc_support(void);

#endif

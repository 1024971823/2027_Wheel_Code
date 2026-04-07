#ifndef USB_DEBUG_H
#define USB_DEBUG_H
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void ModifyDebugDataPackage(uint8_t index, float data, const char * name);

#ifdef __cplusplus
}
#endif

#endif  // USB_DEBUG_H

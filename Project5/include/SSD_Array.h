#ifndef SSD_ARRAY_H
#define SSD_ARRAY_H

#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void SSD_init(void);
    void SSD_update(int digitSelect, int value, int decimalPoint);

#ifdef __cplusplus
}
#endif

#endif // SSD_ARRAY_H
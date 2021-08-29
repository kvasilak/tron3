/* *****************************************************************************
*  A circular buffer.
*  Copyright 2015-2021 Keith Vasilakes
*  ***************************************************************************** */

#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

void TxBuf_Put(uint8_t c);
bool TxBuf_Get(uint8_t *c);
uint8_t TxBuf_ElementSize();
bool TxBuf_IsEmpty();
void TxBuf_Clear();
int TxBuf_CountStored();
int TxBuf_MaxSize();

#endif

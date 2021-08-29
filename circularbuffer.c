/*
 * circularbuffer.c
 *
 * Created: 8/15/2021 7:21:48 PM
 *  Author: keith
 */ 

#include <avr/io.h>
#include <stddef.h>
#include <stdbool.h>

#include "circularbuffer.h"
#define MAX_ENTRIES 2000

static int TxBuffer[MAX_ENTRIES];

// Head is the location of the last value added to the
// circular buffer. 
uint16_t In;
uint16_t Out;

// Track the number of entries stored in the buffer. 
// Never greater than MAX_ENTRIES;
uint16_t Size;


/* Add a new value to the end of the buffer. If the
buffer has exceed its intrinsic capacity, this will
overwrite old entries. */
void TxBuf_Put(uint8_t c)
{
    TxBuffer[In] = c;

    In++;

    if (In >= MAX_ENTRIES) { In = 0; }

    if(Size < MAX_ENTRIES ) { Size++; }
}

//Get a byte from the buffer
//Removes byte
bool TxBuf_Get(uint8_t *c)
{
    //buffer is empty?
    if(In != Out)
    {
	    //not empty yet
	    *c = TxBuffer[Out];
		
	    Out++;
		
	    //handle wrap
	    if(Out >= MAX_ENTRIES)
	    {
		    Out = 0;
	    }
		
	    return true;
    }
	
    return false;
}

uint8_t TxBuf_ElementSize()
{
    return sizeof(uint8_t);
}

bool TxBuf_IsEmpty()
/* Returns true if no data has been stored in the buffer, 
and false if at least one value has been stored. */
{
    return Size == 0;
}

/* Empties the circular buffer. The old data is 
still present, but iterators and count will 
ignore it. */
void TxBuf_Clear()
{
    Size = 0;
    In = 0;
    Out = 0;
}

/* Returns the number of entries that are stored
in the circular buffer. */
int TxBuf_CountStored()
{
    return Size;
}

/* Returns the maximum number of values that can
be stored in the circular buffer. */
int TxBuf_MaxSize()
{
    return MAX_ENTRIES;
}

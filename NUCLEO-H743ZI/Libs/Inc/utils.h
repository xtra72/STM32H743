#ifndef UTILS_H_
#define UTILS_H_

#include "target.h"

bool        strToUint16(char* string, uint16_t* value);
bool        strToUint32(char* string, uint32_t* value);

uint32_t    seperateString(char* string, char* delimitSet, char* seperatedStrings[], uint32_t maxSeperatedCount);
bool        strToIP(char* string, uint8_t *ip);

double IEEE754_Binary32ToDouble(int value); // IEEE integer convert double.

#endif
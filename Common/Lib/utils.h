#ifndef UTILS_H_
#define UTILS_H_

#include "target.h"

bool        strToUint8(char* string, uint8_t* value);
bool        strToUint16(char* string, uint16_t* value);
bool        strToUint32(char* string, uint32_t* value);
bool        strToHex16(char* string, uint16_t* value);
bool        strToHex32(char* string, uint32_t* value);
bool        strToHexArray(char* string, uint8_t* buffer, uint32_t bufferSize, uint32_t* length);

uint32_t    seperateString(char* string, char* delimitSet, char* seperatedStrings[], uint32_t maxSeperatedCount);
bool        strToIP(char* string, uint8_t *ip);

double IEEE754_Binary32ToDouble(int value); // IEEE integer convert double.

uint32_t    ntohUint32(uint32_t value);
uint16_t    ntohUint16(uint16_t value);
#endif
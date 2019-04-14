#include <string.h>
#include <math.h>
#include <ctype.h>
#include "utils.h"

bool    strToUint16(char* string, uint16_t* value)
{
    if ((string == NULL) || (strlen(string) == 0))
    {
        return  false;
    }

    for(int i = 0 ; i < strlen(string) ; i++)
    {
        if (!isdigit(string[i]))
        {
            return  false;
        }
    }

    uint32_t    temp = strtoul(string, 0, 10);
    if (temp > (uint32_t)UINT16_MAX)
    {
        return  false;
    }

    *value = (uint16_t)temp;

    return  true;
}

bool    strToUint32(char* string, uint32_t* value)
{
    if ((string == NULL) || (strlen(string) == 0))
    {
        return  false;
    }

    for(int i = 0 ; i < strlen(string) ; i++)
    {
        if (!isdigit(string[i]))
        {
            return  false;
        }
    }

    *value = strtoul(string, 0, 10);

    return  true;
}

bool    strToHex32(char* string, uint32_t* value)
{
    if ((string == NULL) || (strlen(string) == 0))
    {
        return  false;
    }

    for(int i = 0 ; i < strlen(string) ; i++)
    {
        if (!(isdigit(string[i]) || ('a' <= string[i] && string[i] <= 'f') || ('A' <= string[i] && string[i] <= 'F')))
        {
            return  false;
        }
    }

    *value = strtoul(string, 0, 16);

    return  true;
}

uint32_t    seperateString(char* string, char* delimitSet, char* seperatedStrings[], uint32_t maxSeperatedCount)
{
    uint32_t    count = 0;
    char*   token;

    token = strtok(string, delimitSet);
    if (token != NULL)
    {
        seperatedStrings[count++] = token;
        while(token = strtok(NULL, delimitSet))
        {
            if (count >= maxSeperatedCount)
            {
                break;
            }

            seperatedStrings[count++] = token;
        }
    }

    return  count;
}

bool    strToIP(char* string, uint8_t *ip)
{
    uint32_t    len;
    char        buffer[16];
    char*       seperatedStrings[4];

    len = strlen(string);
    if ((len < 7) || (15 < len))
    {
        return  false;
    }

    strcpy(buffer, string);

    if (seperateString(buffer, ".", seperatedStrings, 4) != 4)
    {
        return  false;
    }

    ip[0] = strtoul(seperatedStrings[0], 0, 10);
    ip[1] = strtoul(seperatedStrings[1], 0, 10);
    ip[2] = strtoul(seperatedStrings[2], 0, 10);
    ip[3] = strtoul(seperatedStrings[3], 0, 10);

    return  true;
}


double IEEE754_Binary32ToDouble(int value) // IEEE integer convert double.
{
        int minus = -1, exponent;
        double fraction, result;

       if((value&0x80000000) == 0)
       {
               minus = 1;
       }
       exponent = ((value & 0x7F800000) >> 23) - 127;
       fraction = (value & 0x7FFFFF) + 0x800000;
       fraction = fraction / 0x800000;
       result = minus * fraction * pow(2, exponent);
       return result;
}

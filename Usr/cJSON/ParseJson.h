#ifndef PARSEJson_H__
#define PARSEJson_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "main.h"

void json_Parse(char* str);
void json_Parse_K210(char* str);

#ifdef __cplusplus
}
#endif
#endif

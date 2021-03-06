//*****************************************************************************
//
// X24015 SD Boot Loader
//
// Copyright (C) 2021, RTZ MICROSYSTEMS LLC
// All Rights Reserved
//
//*****************************************************************************

#ifndef __CUSTOM_H
#define __CUSTOM_H

/* External Debug Function Prototypes */
extern void ConfigureUART(void);
extern void UARTPutch(char ch);
extern void UARTPuts(char* s);
extern void UARTprintf(const char *pcString, ...);

#endif /* __CUSTOM_H */

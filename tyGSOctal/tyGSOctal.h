/**************************************************************************
 Header:        tyGSOctal.h

 Author:        Peregrine M. McGehee

 Description:   Header file for GreenSpring Ip_Octal 232, 422, and 485
 serial I/O modules. This software is somewhat based on the HiDEOS
 device driver developed by Jim Kowalkowski of the Advanced Photon Source.

 History:
 who  when      what
 ---  --------  ------------------------------------------------
 PMM  18/11/96  Original.
 PMM  12/12/96  Added node * client.
 PMM  06/03/97  Increased number of delimiters to 5.
 ANJ  11/11/03  Significant cleanup, added ioc shell stuff
**************************************************************************/

#ifndef INC_TYGSOCTAL_H
#define INC_TYGSOCTAL_H

enum { MAX_SPIN_TIME=2 };
typedef enum { RS485,RS232 } RSmode;

typedef struct ty_gsoctal_dev {
    TY_DEV	    tyDev;
    SCC2698*        regs;
    SCC2698_CHAN*   chan;

    int             created;
    int             block;
    struct quadTable *qt;
    RSmode          mode;
    int             baud;
    int             opts;
    UCHAR           imr;
} TY_GSOCTAL_DEV;

typedef struct quadTable {
    const char *moduleID;
    TY_GSOCTAL_DEV port[8];             /* one per port */
    int modelID;
    ushort_t carrier;
    ushort_t module;
    UCHAR imr[4];			/* one per block */
} QUAD_TABLE;

STATUS tyGSOctalDrv(int);
int tyGSOctalModuleInit(const char *, const char *, int, int, int);
const char *tyGSOctalDevCreate(char *, const char *, int, int, int);
void tyGSOctalConfig(char *, int, char, int, int, char);
void tyGSOctalReport(void);

#endif

/**************************************************************************
 Header:        tyGSOctal.h

 Author:        Peregrine M. McGehee

 Description:   Header file for GreenSpring Ip_Octal 232, 422, and 485
 serial I/O modules. This software is somewhat based on the HiDEOS
 device driver developed by Jim Kowalkowski of the Advanced Photon Source.

 History:
 who            when       what
 ---            --------   ------------------------------------------------
 PMM            18/11/96   Original.
 PMM            12/12/96   Added node * client.
 PMM            06/03/97   Increased number of delimiters to 5.
**************************************************************************/

#ifndef __OCTALUART_H
#define __OCTALUART_H

typedef enum { BYTE_FUNC_ISR=0, BYTE_FUNC_TASK=1 } BYTE_FUNC_TYPES;
typedef enum { SerialWriteTimeOut=-2, SerialReadTimeOut=-1 } SerialRC;
typedef enum { SerialNoWait=0, SerialWaitForever=-1 } SerialReadModes;
typedef enum { SerialNoReply=-2 } SerialWriteModes;
typedef enum { ByteFuncOK=0,ByteFuncEndRead=1,ByteFuncReject=-1 } ByteFuncRC;
enum { MAX_SPIN_TIME=2, CLOCK_HZ=3686400 };
typedef enum { QUAD, OCTAL } TYPE_SIZE;
typedef enum { RS485,RS232 } RSmode;

struct ty_gsoctal_dev
{
    TY_DEV	    tyDev;
    SCC2698*        regs;
    SCC2698_CHAN*   chan;

    int             created;
    UCHAR           opcr,mr1,mr2;
    UCHAR           imr;
    int             port, block;
    void *          qt;
    RSmode          mode;
};
typedef struct ty_gsoctal_dev TY_GSOCTAL_DEV;

struct quadTable
{
    TY_GSOCTAL_DEV port[8];             /* one per port */
    ushort_t carrier;
    ushort_t module;
    UCHAR imr[4];			/* one per block */
};
typedef struct quadTable QUAD_TABLE;

STATUS tyGSOctalDrv(int);
int tyGSOctalModuleInit(char *, int, int, int);
TY_GSOCTAL_DEV *tyGSOctalDevCreate(char *, int, int, int, int);
void tyGSOctalReport(void);
void tyGSOctalConfig(TY_GSOCTAL_DEV *, unsigned int, char,
                     int, int, char);
void tyGSOctalSetcr(TY_GSOCTAL_DEV *, unsigned char);
void tyGSOctalSetopcr(TY_GSOCTAL_DEV *, unsigned char);
int  tyGSOctalOpen(TY_GSOCTAL_DEV *, char *, int);
int  tyGSOctalWrite(TY_GSOCTAL_DEV *, char *, long);
STATUS tyGSOctalIoctl(TY_GSOCTAL_DEV *, int, int);
int tyGSOctalStartup(TY_GSOCTAL_DEV *);

#endif

/**************************************************************************
 CVS/RCS Log information:

**************************************************************************/


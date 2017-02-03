/*
FILENAME...     IP520Int.h
USAGE...        Internal Acromag IP520 header information.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/**************************************************************************
 History:
 who  when       what
 ---  ---------- ------------------------------------------------
 RLS  2014/05/05 Original. Based on Andrew Johnson's "tyGSOctal".
**************************************************************************/

#ifndef INC_IP520_H
#define INC_IP520_H

#include <tyLib.h>  /* For TY_DEV. */
#include <epicsTypes.h>

typedef enum {RS232, RS422, RS485} RSmode;  /* IP520 - RS232 only, IP521 - RS422 or RS485 */

struct regmap {
    union {
        struct {
            epicsUInt8 Off0, rbr;    /* recieve buffer (or dll) */
            epicsUInt8 Off2, ier;    /* interrupt enable */
            epicsUInt8 Off4, isr;    /* interrupt status (or efr) */
            epicsUInt8 Off6, lcr;    /* line control */
            epicsUInt8 Off8, mcr;    /* modem control */
            epicsUInt8 OffA, lsr;    /* line status */
            epicsUInt8 OffC, msr;    /* modem status */
            epicsUInt8 OffE, scr;    /* scratch */
        } read;
        struct {
            epicsUInt8 Off0, thr;    /* transmit holding (or dll) */
            epicsUInt8 Off2, ier;    /* interrupt enable */
            epicsUInt8 Off4, fcr;    /* FIFO control (or efr) */
            epicsUInt8 Off6, lcr;    /* line control */
            epicsUInt8 Off8, mcr;    /* modem control */
            epicsUInt8 OffA, lsr;    /* line status */
            epicsUInt8 OffC, msr;    /* modem status */
            epicsUInt8 OffE, scr;    /* scratch */
        } write;
    } u;
};
typedef volatile struct regmap REGMAP;


typedef struct ty_ip520_dev {
    TY_DEV          tyDev;
    REGMAP          *regs;
    struct modTable *pmod;
    int             created;
    RSmode          mode;
    int             baud;
    int             opts;
    int             overCount;    /* Rx overrun error counter. */
    int             parityCount;  /* Rx parity error counter. */
    int             frameCount;   /* Rx framing error counter. */
    unsigned long   readCount;
    unsigned long   writeCount;
} TY_IP520_DEV;

typedef struct modTable {
    const char    *moduleID;
    TY_IP520_DEV   dev[8];
    epicsUInt16    modelID;
    epicsUInt16    carrier;
    epicsUInt16    slot;
    epicsInt16     irqCount;
} MOD_TABLE;

int IP520Drv(int);
int IP520ModuleInit(const char *, const char *, int, int, int);
const char* IP520DevCreate(char *, const char *, int, int, int);
void IP520Report(void);

#endif

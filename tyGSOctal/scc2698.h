/**************************************************************************
 Header:        scc2698.h

 Author:        Peregrine McGehee

 Description:   SCC2698 UART management - based on the HiDEOS driver for
 the GreenSpring Ip_Octal 232, 422, and 485 serial I/O modules
 developed by Jim Kowalkowski of the Advanced Photon Source.

 History:
 who            when       what
 ---            --------   ------------------------------------------------
 PMM            18/11/96   Original
**************************************************************************/

#ifndef __SCC2698_H
#define __SCC2698_H
/*
 * SCC2698 UART MANAGEMENT
 *
 * (R)=read access, (W)=write access (R/W) read/write access
 *
 * This structure can be used to index to each port when the SCC is set
 * up in the Quad or Octal configuation.
 */
struct scc2698_chan {
    union {
        struct {
            UCHAR d0,mr;	/* a mode register 1/2 (R/W) */
            UCHAR d1,sr;	/* a status (R), a clock select (W) */
            UCHAR d2,r1;	/* a command (W) */
            UCHAR d3,rhr;	/* a receiver hold (R), a transmitter
                                 hold (W) */
            UCHAR junk[8];	/* other stuff for block control */
        } r;
        struct {
            UCHAR d0,mr;	/* a mode register 1/2 (R/W) */
            UCHAR d1,csr;	/* a status (R), a clock select (W) */
            UCHAR d2,cr;	/* a command (W) */
            UCHAR d3,thr;	/* a receiver hold (R), a transmitter
                                 hold (W) */
            UCHAR junk[8];	/* other stuff for block control */
        } w;
    } u;
};
typedef volatile struct scc2698_chan SCC2698_CHAN;

/*
 * This is the entire structure of the SCC.  Note that there are really
 * only four control blocks, each containing two ports.
 */
struct scc2698 {
    union {
        struct {
            UCHAR d0,mra;	/* a mode register 1/2 (R/W) */
            UCHAR d1,sra;	/* a status (R), a clock select (W) */
            UCHAR d2,r1;	/* reserved */
            UCHAR d3,rhra;	/* a receiver hold (R), a transmitter
                                 hold (W) */
            UCHAR d4,ipcr;	/* a Aux cntl (W), a input port change
                                 (R) */ 
            UCHAR d5,isr;	/* a Interrupt status (R), a Interrupt
                                 mask (W) */
            UCHAR d6,ctur;	/* a Counter timer upper (R/W) */
            UCHAR d7,ctlr;	/* a Counter timer upper (R/W) */
            UCHAR d8,mrb;	/* b mode register 1/2 (R/W) */
            UCHAR d9,srb;	/* b status (R), b clock select (W) */
            UCHAR da,r2;	/* reserved */
            UCHAR db,rhrb;	/* b receiver hold (R), b transmitter
                                 hold (W) */
            UCHAR dc,r3;	/* reserved */
            UCHAR dd,ip;	/* a output port conf (W), a input
                                 port (R) */
            UCHAR de,ctg;	/* start counter timer a (R) */
            UCHAR df,cts;	/* stop counter timer a (R) */
        } r;
        struct {
            UCHAR d0,mra;	/* a mode register 1/2 (R/W) */
            UCHAR d1,csra;	/* a status (R), a clock select (W) */
            UCHAR d2,cra;	/* a command (W) */
            UCHAR d3,thra;	/* a receiver hold (R), a transmitter
                                 hold (W) */
            UCHAR d4,acr;	/* a Aux cntl (W), a input port change
                                 (R) */
            UCHAR d5,imr;	/* a Interrupt status (R), a Interrupt
                                 mask (W) */
            UCHAR d6,ctu;	/* a Counter timer upper (R/W) */
            UCHAR d7,ctl;	/* a Counter timer upper (R/W) */
            UCHAR d8,mrb;	/* b mode register 1/2 (R/W) */
            UCHAR d9,csrb;	/* b status (R), b clock select (W) */
            UCHAR da,crb;	/* b command (W) */
            UCHAR db,thrb;	/* b receiver hold (R), b receiver
                                 hold (W) */
            UCHAR dc,r3;	/* reserved */
            UCHAR dd,opcr;	/* a output port conf (W), a input
                                 port (R) */
            UCHAR de,r4;	/* start counter timer a (R) */
            UCHAR df,r5;	/* start counter timer a (R) */
        } w;
    } u;
};
typedef volatile struct scc2698 SCC2698;

/*
 * SCC 2698 ISR/IMR Bit definitions
*/ 
#define SCC_ISR_TXRDY_A 0x01
#define SCC_ISR_RXRDY_A 0x02
#define SCC_ISR_CBRK_A  0x04
#define SCC_ISR_CTRRDY  0x08
#define SCC_ISR_TXRDY_B 0x10
#define SCC_ISR_RXRDY_B 0x20
#define SCC_ISR_CBRK_B  0x40
#define SCC_ISR_MPI     0x80

#endif

/**************************************************************************
 CVS/RCS Log information:

**************************************************************************/

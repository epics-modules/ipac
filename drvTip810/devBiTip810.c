/*******************************************************************************
Project:
    CAN Bus Driver for EPICS

File:
    devBiTip810.c

Description:
    TIP810 Status Binary Input device support

Author:
    Andrew Johnson <anjohnson@iee.org>
Created:
    3 April 1997
Version:
    $Id: devBiTip810.c,v 1.9 2007-02-09 22:09:55 anj Exp $

Copyright (c) 1995-2000 Andrew Johnson

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*******************************************************************************/

#include <vxWorks.h>
#include <string.h>
#include <ctype.h>

#include "dbDefs.h"
#include "dbAccess.h"
#include "recSup.h"
#include "recGbl.h"
#include "alarm.h"
#include "devSup.h"
#include "devLib.h"
#include "biRecord.h"
#include "canBus.h"
#include "drvTip810.h"
#include "pca82c200.h"
#include "epicsExport.h"


/* Create the dset for devBiTip810 */
LOCAL long init_bi(struct biRecord *prec);
LOCAL long read_bi(struct biRecord *prec);

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_bi;
} devBiTip810 = {
	5,
	NULL,
	NULL,
	init_bi,
	NULL,
	read_bi
};
epicsExportAddress(dset, devBiTip810);

LOCAL long init_bi(
    struct biRecord *prec
) {
    static struct {
	char		*string;
	unsigned long	mask;
    } tipState[] = {
	{ "BUS_OFF",	PCA_SR_BS },
	{ "BUS_ERROR",	PCA_SR_ES },
	{ "DATA_OVERRUN",	PCA_SR_DO },
	{ "RECEIVING",	PCA_SR_RS },
	{ "RECEIVED",	PCA_SR_RBS },
	{ "SENDING",	PCA_SR_TS },
	{ "SENT", 	PCA_SR_TCS },
	{ "OK_TO_SEND",	PCA_SR_TBS },
	{ NULL,		0 }
    };

    char *canString;
    char *name;
    char separator;
    int i;
    long status;

    /* bi.inp must be an INST_IO */
    if (prec->inp.type != INST_IO) goto error;

    canString = ((struct instio *)&(prec->inp.value))->string;

    /* Strip leading whitespace & non-alphanumeric chars */
    while (!isalnum(*canString)) {
	if (*canString++ == '\0') goto error;
    }

    /* First part of string is the bus name */
    name = canString;

    /* find the end of the busName */
    canString = strpbrk(canString, "/:");
    if (canString == NULL || *canString == '\0') goto error;

    /* Temporarily truncate string after name and look up t810 device */
    separator = *canString;
    *canString = '\0';
    status = canOpen(name, &prec->dpvt);
    *canString++ = separator;
    if (status) goto error;

    /* After the bus name comes the name of the status bit we're after */
    prec->mask=0;
    for (i=0; tipState[i].string != NULL; i++)
	if (strcmp(canString, tipState[i].string) == 0)
	    prec->mask=tipState[i].mask;

    if (prec->mask == 0) {
        goto error;
    }
    return 0;

error:
    if (canSilenceErrors) {
    	prec->dpvt = NULL;
    	prec->pact = TRUE;
    	return OK;
    } else {
    	recGblRecordError(S_db_badField,(void *)prec,
			  "devBiTip810: Bad INP field type or value");
    	return S_db_badField;
    }
}

LOCAL long read_bi(struct biRecord *prec)
{
	if (prec->dpvt == NULL || prec->mask == 0) {
	    prec->pact = TRUE;
	    return S_dev_noDevice;
	}

	prec->rval = t810Status(prec->dpvt) & prec->mask;
	return 0;
}


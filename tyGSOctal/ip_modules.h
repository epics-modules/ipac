/**************************************************************************
 Header:        ip_modules.h

 Author:        Peregrine McGehee

 Description:   Manufacturer and Model Ids for GreenSpring serial I/O
 modules. Based on the HiDEOS driver for the GreenSpring Ip_Octal 232,
 422, and 485 serial I/O modules developed by Jim Kowalkowski of the
 Advanced Photon Source.

 History:
 who            when       what
 ---            --------   ------------------------------------------------
 PMM            18/11/96   Original
**********************************************************/

#ifndef __IP_MODULES_H
#define __IP_MODULES_H

/* 
	known id_prom values for Green Spring:
		ascii_1='I' ascii_2='P' ascii_3='A' ascii_4='C'
		manufacturer_id = 0xf0 (Green Spring)
		model_id:
			0x22 = IP-Octal232
			0x2a = IP-Octal422
			0x48 = IP-Octal485	
*/

#define GREEN_SPRING_ID	        0xf0

#define GSIP_OCTAL232		0x22
#define GSIP_OCTAL422		0x2a
#define GSIP_OCTAL485		0x48

#endif 

/**************************************************************************
 CVS/RCS Log information:



**************************************************************************/

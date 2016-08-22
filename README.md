# IPAC Carrier and Module Drivers

This README file is part of ipac, also previously known as drvIpac, a collection
of software mainly for vxWorks and RTEMS to drive one or more Industry Pack (IP)
carrier boards and a small set of IP I/O modules. Other modules exist that use
the carrier interface defined here to drive other I/O modules. This software was
written for use with EPICS, the [Experimental Physics and Industrial Control
System toolkit](http://www.aps.anl.gov/epics).

The source code and some Wiki pages for this module are now maintained at
[github.com](https://github.com/), start reading at the [ipac Wiki Home
Page](https://github.com/epics-modules/ipac/wiki) or browsing the code on the
[master branch](https://github.com/epics-modules/ipac).

## EPICS Application Structure

This is a standard EPICS support module using the EPICS build system. Sites may
wish to edit the top-level Makefile to change the list of module drivers that
are built along with the ipac driver. Drivers should be configured to only
compile on the OS platforms for which they will build though, so this step
shouldn't be necessary.

Andrew Johnson


------

## LEGAL STUFF

This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free
Software Foundation; either version 2.1 of the License, or (at your option) any
later version.

This library is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along
with this library; if not, write to the Free Software Foundation, Inc., 59
Temple Place, Suite 330, Boston, MA  02111-1307  USA

#!/bin/env python
#==========================================================================
# (c) 2007-2008  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Examples
# File    : detect.py
#--------------------------------------------------------------------------
# Simple Cheetah Device Detection Example
#--------------------------------------------------------------------------
# Redistribution and use of this file in source and binary forms, with
# or without modification, are permitted.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#==========================================================================

#==========================================================================
# IMPORTS
#==========================================================================
from cheetah_py import *


#==========================================================================
# MAIN PROGRAM
#==========================================================================
print "Searching for Cheetah adapters..."

# Find all the attached devices
(num, ports, unique_ids) = ch_find_devices_ext(16, 16)

if num > 0:
    print "%d device(s) found:" % num

    # Print the information on each device
    for i in range(num):
        port      = ports[i]
        unique_id = unique_ids[i]

        # Determine if the device is in-use
        inuse = "(avail)"
        if (port & CH_PORT_NOT_FREE):
            inuse = "(in-use)"
            port  = port & ~CH_PORT_NOT_FREE

        # Display device port number, in-use status, and serial number
        print "    port = %d   %s  (%04d-%06d)" % \
            (port, inuse, unique_id / 1000000, unique_id % 1000000)

else:
    print "No devices found."

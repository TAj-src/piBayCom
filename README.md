
09/06/2020 - version 0.1.07a

#######################################################################################
#######################################################################################
piBayCom Linux Kernel & KISS modules by G7TAJ. Original BayCom HDLC code by G8BPQ.

NOTE : This is an alpha release. It works but is far from tested in. Please give feedback!


You will need to modify the hardware a little (or at-least the lead).

  * Remove the diodes that power the device from the RS232 lines (may blow up the Pi and wont power it properly)
  * Power it remotely (you could re-wire the TXD pin in the cable to do this, leaving this diode. It depends on the hardware variant you have)
  * Make a 5v to 3v3 resistor divider on the modem TX pin to the Pi. TCM3105 is 5v, Pi is 3v3.

(You will need the Linux kernel headers for the Pi you are using - sudo apt install raspberrypi-kernel-headers)

to compile, type

	make


You end up with :

	piBayComLKM.ko  <- Linux Kernel Module
	BayComKISS      <- KiSS driver 
        cal             <- Calibration routine program
	
copy the file 99-piBayCom.rules to /etc/udev/rules.d/99-piBayCom.rules


Contens of the file for info:

#Rules file for the piBayCom device driver
KERNEL=="piBayCom", SUBSYSTEM=="PiB", MODE="0666"




Kernel Module
-------------

	sudo insmod piBayComLKM.ko RX_PIN=17 TX_PIN=23 PTT_PIN=27  [BAUD=1200 | 300]


The pins are the BCM numbers as per (https://bloggerbrothers.files.wordpress.com/2017/01/screen-shot-2017-11-12-at-2-06-11-pm.png?w=663)

BAUD is optional and assumes 1200 if ommitted.
        This is for use on modems that do not use the TCM3105 (e.g. AM7910 and Op-Amp variants)


to check its running ok, tail the kern.og

	tail  /var/log/kern.log -f

Calibration program
-------------------

With the kernel module loaded, run :
        ./cal

You get 4 options.

	0 = disable PTT
 	1 = Send HIGH tone + PTT
 	2 = Send LOW tone + PTT
 	3 = Send alternate diddle tones for ~15seconds + PTT

Note: standard BayCom modems do not mute the TX line when PTT is LOW, so they always output the last tone.


KISS driver
-----------

	./BayComKISS -k8616 [-b1200|300]

note: sudo is not needed

where -k8616 is the TCP port
      -b1200 or 300 - the BAUD rate. optional. Defaults to 1200

To stop the driver, kill BayComKISS (or CTRL-C if on the console) and then unload the module

	 killall BayComKISS
	 sudo rmmod piBayComLKM.ko


73s de Steve

Good luck !

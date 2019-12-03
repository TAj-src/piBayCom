#######################################################################################
#######################################################################################
PiBayCom Linux Kernel & KISS modules by G7TAJ. Original BayCom HDLC code by G8BPQ.


You will need to modify the hardware a little (or at-least the lead).

  * Remove the diodes that power the device from the RS232 lines (may blow up the Pi and wont power it properly)
  * Power it remotely (you could re-wire the TXD pin in the cable to do this, leaving this diode. It depends on the hardware variant you have)
  * Make a 5v to 3v3 resistor divider on the modem TX pin to the Pi. TCM3105 is 5v, Pi is 3v3.



to compile, type

	make


You end up with :

	piBayComLKM.ko  <- Linux Kernel Module
	BayComKISS      <- KiSS driver 


copy the file 99-piBayCom.rules to /etc/udev/rules.d/99-piBayCom.rules


Contens of the file for info:

#Rules file for the piBayCom device driver
KERNEL=="piBayCom", SUBSYSTEM=="PiB", MODE="0666"




to run LKM

	sudo insmod piBayComLKM.ko RX_PIN=17 TX_PIN=23 PTT_PIN=27


The pins are the BCM numbers as per (https://bloggerbrothers.files.wordpress.com/2017/01/screen-shot-2017-11-12-at-2-06-11-pm.png?w=663)


to check its running ok, tail the kern.og

	tail  /var/log/kern.log -f


To run the KISS driver

	./BayComKISS -k8616

note: sudo is not needed

where -k8616 is the TCP port


To stop the driver, kill BayComKISS and then unload the module

	 killall BayComKISS
	 sudo rmmod piBayComLKM.ko


73s de Steve

Good luck !

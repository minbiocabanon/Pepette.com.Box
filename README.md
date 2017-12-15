# Pepette.com.Box
SMS box for monitoring voltage accumulators, water sensor and GPS position of a sailboat

# Libraries needed
install this lib :
http://playground.arduino.cc/Main/RunningMedian

# compatible with Arduino 1.5.8 + linkitone SDK

## How to prepare files for OTAUpdate

- First modify Arduino IDE options : allow verbose compilation
![Arduino IDE verbose option](/img_wiki/1_verbose.png)

- Compile your project and copy the path of the vxp.cpp file
![Arduino IDE verbose option](/img_wiki/2_file.png)

- In the path, copy the vxp.cpp file and paste it in your server path. Rename it in **update.vxp**

-  Now generate the md5 file. You can use a Linux/machine with this command on the server :

  **# md5sum update.vxp > update.md5**
  
- Be sure that the path server is accessible from the internet and then you can run OTAUpdate software on your LinkItOne


#ToDo List

Use concatenated SMS to accelerate command . Use comma for separating commands:
Exemple :
	1234,1  -> get status

# Version

#15/12/2017
	Check modem by autotest with sending an sms (loopback), if time out it will resets linkitone (reset fonction of OTAUpdate)
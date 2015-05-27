# Pepette.com.Box
SMS box for monitoring voltage accumulators, water sensor and GPS position of a sailboat


## How to prepare files for OTAUpdate

- First modify Arduino IDE options : allow verbose compilation
![Arduino IDE verbose option](/img_wiki/1_verbose.png)

- Compile your project and copy the path of the vxp.cpp file
![Arduino IDE verbose option](/img_wiki/2_file.png)

- In the path, copy the vxp.cpp file and paste it in your server path. Rename it in **update.vxp**

-  Now generate the md5 file. You can use a Linux/machine with this command on the server :

  **# md5sum update.vxp > update.md5**
  
- Be sure that the path server is accessible from the internet and then you can run OTAUpdate software on your LinkItOne


Env:  
  
On ubuntu:  
  
Build, code, flash - everything with [Arduino-Nvim Plugin](https://github.com/yuukiflow/Arduino-Nvim)   
  
%% 
Old
IDE ->  
 - Build arduino IDE with.   
 (because arduino ide has a bug, or more likely ubuntu has hard to fix permission access from arduino ide's appimage 
 to ports. Actually after a wile started to work by its own)  
 (platfromio btw for some reason can't get access too. Plus as of now lags behind and didn't added support for c6 of many necessary libs)  
   
 - Flash with esptool manually.  
 - code in nvim, platfromio, or where you want.   
 %%   
  
  
  
Project setup  
  
// ElegantOTA async mode has bug that it ignores #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1 in project C files, so this param must be set either:
- directly by patching library ~/Arduino/libraries/ElegantOTA/src/ElegantOTA.cpp. Details in lib's git. 
   #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
- Passed as build flag -DELEGANTOTA_USE_ASYNC_WEBSERVER=1 (worked for PlatformIO)
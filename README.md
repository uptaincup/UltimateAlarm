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
  
// Must be set directly by patching library ~/Arduino/libraries/ElegantOTA/src/ElegantOTA.cpp. Details in libs git.  
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
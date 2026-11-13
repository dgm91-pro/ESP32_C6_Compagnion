---
title: "ESP32 C6 COMPAGNION"
github: ""
description: "This is Going to be Experimental..."
created_at: "2026-11-11"
total_time: "28h 6m"
---

# November 11, 2026: First Entry

Looking Around for what components i'm going to use , i want to make a deskbot that i could maybe program to listen to my teacher and then use an IA to take notes for me since i work with a PC at school.

I'll like to add some physical buttons and a led to start recording and everything.

![](https://fabricate.hackclub-assets.com/c50832df457cad83e6021a43d178ac217f1d0fb969ab17db30128f2b692e341d/image-webp/pouce-vers-le-haut-d-%C3%A9motic-ne-47854113.webp)

**Total time spent: 6m**

# November 11, 2026: My First DIY footprint and symbol in kicad ! Yayyyy

So i started designing the footprint and symbol in kicad cuz i wanted to make it my own so i know exactly where are the holes and everything and i don't download a bad footprint ( ex : esp32 c6 without touch )

![](https://fabricate.hackclub-assets.com/5c1628aec893b525809c2bc361a37fa4423b9bf83c683f4148651802a44b2cdd/image-png/Capture%20d%27%C3%A9cran%202026-11-11%20205727.png)
![](https://fabricate.hackclub-assets.com/93c7c44c7d0699de0cbe56711ccf831b146a7a238cc4e4aa1e43ff0f94a53aeb/image-png/Capture%20d%27%C3%A9cran%202026-11-11%20205746.png)

**Total time spent: 3h**

# November 12, 2026: Made Some Research , choose components and made schematics !

Hi , i've made some search and here is what i'll use :

- MAX98357A for audio output ( shared i2s )
- SPH0645LM4H for audio input ( shared i2s )
- MCP23008 which will allow me to connect the button and RGB led
- TPS2116DRL i'll explain why below.

This esp board have an internal battery charging and management module. BUT , the amp i'm using need more than the internal 3.3v regulator can give. So , i'm going to use this power mux to be able to program everything and test only with usb-c without battery , and i want to make sure i'll don't have any problem if usb and battery are giving power at the same time.

Note : I2S connection should be possible by "remapping" the esp32 UART pins.
( flash and debug gonna be on usb )

![](https://fabricate.hackclub-assets.com/74878ef544208108c2ad606be25649a3da2cdac52e8da1ac7dc27a68cedc5bf5/image-jpeg/schematics0.1_pages-to-jpg-0001.jpg)

**Total time spent: 11h 30m**

# November 13, 2026: Routed The PCB ( Good Title Suggestion Hackclub 😂)

So , I Fixed some little problems with the schematics and just ended to route everything/add some decorations to my design.
Next Step Will be to Make The BOM.
I'll try to find as more component as i can to make it cheaper for funding.

Note : I needed to put VIAS at some random places to join GND on both surfaces.

![](https://fabricate.hackclub-assets.com/4eb46d767d9a6ec85e2f3d68af07cec8e4cf2e6cfbd141dcfea6fbe0932b711d/image-png/Capture%20d%27%C3%A9cran%202026-11-13%20165246.png)

**Total time spent: 8h 30m**

# November 13, 2026: Cleaning ts out🧹💨🍃

Fixed some little things on the pcb , 
Made the BOM and Github repo ,
And made checkout tests on different platforms. 

It was pretty hard to do everything and took me a lot of time but here we are.
For the 3d design , i've some idea of how i'll put thing in place , but i wan't to wait for parts 
to have the exact size of components, also, normally i should have a better 3D printer for Xmas so i'll see. Same for software , i need to try on the board directly and anyway i don't have anything to test rn.

I'll put you my BOM here , but please go look on the github repo , because you'll need some informations about the parts. 

https://github.com/dgm91-pro/ESP32_C6_Compagnion/tree/main

( PS : i forgot to put that you might need an sd card , but i'll update later )

![](https://fabricate.hackclub-assets.com/f11f555d9fee09235098de2a3c46dac69725bfca7b3f154c2f77c5a6db46d4a2/image-png/BOM_TXT_IMG.png)

**Total time spent: 5h**

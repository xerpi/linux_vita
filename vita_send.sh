#!/bin/sh

curl -T ./arch/arm/boot/zImage ftp://$PSVITAIP:1337/ux0:/linux/
curl -T "./arch/arm/boot/dts/{vita1000,vita2000,pstv}.dtb" ftp://$PSVITAIP:1337/ux0:/linux/



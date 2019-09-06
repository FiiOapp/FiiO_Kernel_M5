#!/bin/sh
cp sound/soc/ingenic/*.ko ingenic/
adb push ingenic /lib/modules/3.10.14/kernel/sound/soc/
adb reboot

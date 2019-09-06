echo "building mass storage kernel ..."
cd $BUILD_ROOT_PATH/kernel/
make halley2_linux_sfcnand_mass_defconfig
make xImage -j12
make modules -j12
cp arch/mips/boot/compressed/xImage $BUILD_ROOT_PATH/out/product/halley2/image/xImage_TF_card
cp sound/soc/ingenic/snd-asoc-dma-v13.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-asoc-aic-v12.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-asoc-i2s-v13.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-asoc-icdc-d3.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-soc-phoenix-icdc.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-asoc-spdif-v13.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-asoc-spdif-dump.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/
cp sound/soc/ingenic/snd-soc-phoenix-spdif.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/sound/soc/ingenic/

#cp drivers/usb/gadget/g_mass_storage.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
#cp drivers/usb/gadget/libcomposite.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
#cp drivers/usb/gadget/usb_f_mass_storage.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
#cp drivers/usb/gadget/u_serial.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
#cp drivers/usb/gadget/usb_f_serial.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
#cp drivers/usb/gadget/usb_f_acm.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
#cp drivers/usb/gadget/g_serial.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
rm arch/mips/boot/compressed/xImage
rm arch/mips/boot/zcompressed/xImage
mkfs.ubifs -e 0x1f000 -c 2048 -m 0x800 -d $BUILD_ROOT_PATH/out/product/halley2/system -o $BUILD_ROOT_PATH/out/product/halley2/image/system.ubi
#make halley2_linux_sfcnand_ubi_defconfig


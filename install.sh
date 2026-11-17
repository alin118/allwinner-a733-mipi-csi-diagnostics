#!/bin/bash
# Установка драйвера камеры IMX219 (Orange Pi Zero 3W, A733). Запускать: sudo ./install.sh
set -e
cd "$(dirname "$0")"
K=$(uname -r)
[ "$K" = "6.6.98-sun60iw2" ] || { echo "Ядро $K, нужно 6.6.98-sun60iw2: модули не подойдут"; exit 1; }
D=/lib/modules/$K/kernel/bsp/drivers/vin
for m in "vin_io.ko:$D/vin_io.ko" "vin_v4l2.ko:$D/vin_v4l2.ko" "imx219.ko:$D/modules/sensor/imx219.ko"; do
  src=ko/${m%%:*}; dst=${m#*:}
  [ -e "$dst" ] || { echo "нет $dst (другой образ системы?)"; exit 1; }
  [ -e "$dst.orig_vin" ] || cp "$dst" "$dst.orig_vin"
  cp "$src" "$dst"; echo "установлен $dst"
done
# Оверлей второй камеры (vinc8 -> video8, mipi1/csi1, через tdm_rx0)
O=/boot/dtb/allwinner/overlay
dtc -I dts -O dtb -o $O/sun60i-a733-vinc8-offline.dtbo dts/sun60i-a733-vinc8-offline.dts 2>/dev/null
for f in dtbo/*.dtbo; do cp "$f" $O/; n=$(basename "$f" .dtbo); n=${n#sun60i-a733-}
  [ "$n" = vinc8-offline ] && cp $O/sun60i-a733-vinc8-offline.dtbo $O/ 2>/dev/null
  grep -q "$n" /boot/orangepiEnv.txt || { grep -q '^overlays=' /boot/orangepiEnv.txt && sed -i "/^overlays=/ s/\$/ $n/" /boot/orangepiEnv.txt || echo "overlays=$n" >> /boot/orangepiEnv.txt; }
done
if ! grep -q vinc8-offline /boot/orangepiEnv.txt; then
  if grep -q '^overlays=' /boot/orangepiEnv.txt; then sed -i '/^overlays=/ s/$/ vinc8-offline/' /boot/orangepiEnv.txt
  else echo "overlays=vinc8-offline" >> /boot/orangepiEnv.txt; fi
fi
depmod -a
H=${SUDO_USER:+/home/$SUDO_USER}; H=${H:-/home/orangepi}
cp tools/*.sh "$H"/; chmod +x "$H"/*.sh
gcc -O2 -w -o "$H/livec" tools/livec.c -lm -lpthread
gcc -O2 -w -o "$H/arrive" tools/arrive.c
chown $SUDO_USER "$H"/livec "$H"/arrive "$H"/run*.sh 2>/dev/null || true
gcc -O2 -w -o "$H/v4lview" tools/v4lview.c
cp tools/selftest.sh "$H"/ && chmod +x "$H/selftest.sh"
echo "Готово. Перезагрузите плату, затем: sudo $H/selftest.sh и sudo $H/v4lview /dev/video0 /dev/video8"

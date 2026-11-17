# Universal V4L2 driver for IMX219 on Allwinner A733 (Orange Pi Zero 3W)

**Two IMX219 cameras simultaneously, RAW10 Bayer via standard V4L2** — works with `v4l2-ctl`,
GStreamer (`v4l2src`), OpenCV. Kernel 6.6.98-sun60iw2 (vendor BSP). See the full story below (Russian)
and [IMX219_A733_FINAL_REPORT.txt](IMX219_A733_FINAL_REPORT.txt) for the complete diagnostic history:
the vendor BSP declared the A733 MIPI CSI receiver dead — this repo is the proof of life and the fix.

---
# Универсальный V4L2-драйвер IMX219 для Orange Pi Zero 3W (Allwinner A733)

Ядро 6.6.98-sun60iw2 (вендорский BSP). Две камеры одновременно (/dev/video0 и /dev/video8), RAW10, до 60 fps на камеру.

## Что умеет
- Режимы (сенсор): 3280x2464 (~20 fps), 1920x1080 (~47), 1640x1232 (полное поле зрения, аппаратный биннинг 2x2, ~42), 1280x960 (~51), 1280x720 (~72), 640x480 (до ~105 fps).
- Кроп (цифровой зум): окно любого размера и положения через S_SELECTION; биннинг 2x2 включается автоматически, если размер в S_FMT не больше половины окна.
- Форматы: BG10 (10 бит в 16-битном контейнере, порядок BGGR) и BA81 (8 бит, старшие разряды) - последний понимают GStreamer (bayer2rgb) и большинство программ.
- Управление: частота кадров (S_PARM / ENUM_FRAMEINTERVALS, 2..120), экспозиция, усиление.
- Работает с v4l2-ctl, GStreamer (v4l2src), OpenCV (CAP_V4L2, CONVERT_RGB=0).

## Установка
    sudo ./install.sh && sudo reboot
    sudo ~/selftest.sh                       # проверка режимов, fps, кропа
    sudo ~/v4lview /dev/video0 /dev/video8   # просмотр на HDMI (fb0)
install.sh кладёт модули vin_io.ko, vin_v4l2.ko, imx219.ko в /lib/modules (оригиналы сохраняются как *.orig_vin) и подключает оверлеи device tree.

## Примеры
    v4l2-ctl -d /dev/video0 --list-formats-ext
    v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=BG10 --set-parm=30 --stream-mmap --stream-count=100 --stream-to=a.raw
    # кроп 1600x1200 из центра матрицы, затем биннинг (800x600 из этого же окна):
    v4l2-ctl -d /dev/video0 --set-selection=target=crop,left=840,top=632,width=1600,height=1200 --set-fmt-video=width=800,height=600,pixelformat=BG10 --stream-mmap --stream-count=60 --stream-to=b.raw
    # GStreamer:
    gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-bayer,format=bggr,width=1280,height=720 ! bayer2rgb ! videoconvert ! autovideosink
Порядок S_FMT и S_SELECTION любой: драйвер запоминает запрошенный размер и пересчитывает режим.

## Важно знать
- Экспозиция ограничена длиной кадра: чем выше fps, тем короче максимальная выдержка (на 105 fps около 500 строк). Драйвер сам урезает выдержку, а заданное значение запоминает.
- Единицы экспозиции: 1/16 строки (по умолчанию 12800 = 800 строк). Усиление: 16 = 1x ... 2048.
- Экспозицию и усиление можно менять только в процессе, который держит устройство (устройство открывается эксклюзивно).
- Баланс белого аппаратно в IMX219 отсутствует: делайте программно после дебайера.
- Отладочные сообщения драйвера выключены; включить: echo 1 | sudo tee /sys/module/vin_io/parameters/vin_debug
- Предупреждение ядра "WARNING ... sunxi-iommu-v2.c" при открытии устройства безвредно (вендорский IOMMU).
- ffmpeg (-f v4l2) устройство не видит: ядро объявляет только multiplanar-захват, а ffmpeg его не поддерживает.

## Сборка
    cd bsp && make -C /lib/modules/$(uname -r)/build M=$PWD/drivers/vin modules ccflags-y="-I$PWD/include"
Исходники: drivers/vin (дерево BSP с правками), готовые модули: ko/. Требуется тот же образ системы (камера описана в device tree оверлеями из dtbo/).

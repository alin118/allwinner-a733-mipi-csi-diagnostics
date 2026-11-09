# Патчи BSP VIN-драйвера (ядро 6.6.98-sun60iw2, linux-aw2501)

Все правки помечены в исходниках комментариями /* PATCHxxx: ... */.
Активные — необходимы для работы. Пассивные — диагностические printk (можно не применять).

## Активные (необходимые)

| Патч | Файл:строка | Что делает |
|---|---|---|
| PATCH361 | sunxi_mipi.c:937-940 | Корень проекта: num_data_lanes==0 -> форс V4L2_MBUS_CSI2_2_LANE. Без него PHY на 1 лан, поток резался вдвое, тысячи ECC |
| PATCH311 | sunxi_isp.c:822 | RAW-форматы проваливаются в штатное включение ISP (break вместо return 0). ISP всё равно в обходе |
| PATCH247 | vin_tdm.c:519,527 | force RX_TX_EN (if(1||tx_func_en)) — TDM передаёт кадры в свои буферы |
| PATCH350 | vin_tdm.c:926-934 | RAW fall-through при конфиге RX0 (bus_fmt == RAW10 -> break, не return) |
| PATCH393 | vin_tdm.c:2382 | printk «buf0 dma=0x%x buf1 dma=0x%x» — userspace узнаёт физ. адреса DDR-буферов |
| PATCH395 | vin_tdm.c:405-406 | pkg_en=1, lbc_en=0 — ПЛОСКИЙ packed RAW10 вместо LBC (убраны LBC-маркеры, источник дрейфа 1px/строку) |
| PATCH390 | vin_tdm.c:2552 | force TDM_OFFLINE — pkg-буферизация в DDR (не online-обход) |
| PATCH400 | vin_tdm.c:2244 | ignore RX_BUF_FULL (не ронять стрим на переполнение) |
| (без номера) | vin_tdm.c:409,620 | входная высота 720 -> 716 (сенсор шлёт 716 строк) |
| (без номера) | рядом с PATCH393 | включение ДВУХ буферов пинг-понг вместо одного |

## Пассивные (диагностика, не влияют на работу)

PATCH216-219 (speed_dn, RX0_FRM_DONE_INT), PATCH300/320/321/330/340/360
(печать конфигов mcfg/prs/ISP/пайплайна).

## Исторические (уже не применены, оставлено для истории)

- Закрытые TERM/HS/LPDT в sunxi_mipi.c (27.10) — восстановлены OPEN; PHY был мёртв.
- PATCH94 — ISP online по DT — вырезан.
- Таблица ISP t[0x120]=0x02D00500 — взломанная чужой правкой, восстановлена из bak4.
- PATCH370/371 — ISP-хардкод; неважно, ISP в обходе.

## Сборка и установка

```bash
cd ~/linux-aw2501/bsp
make -C /lib/modules/$(uname -r)/build M=$PWD/drivers/vin modules ccflags-y="-I$PWD/include"
# контроль: grep -c error в логе, потом копировать
sudo cp drivers/vin/modules/sensor/imx219.ko /lib/modules/$(uname -r)/kernel/bsp/drivers/vin/modules/sensor/
sudo depmod -a
```
Правило: НИКОГДА не цеплять cp/reboot через && за grep. После ребута сверять
srcversion загруженного модуля с деревом.

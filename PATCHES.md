# Patches to the BSP VIN driver (kernel 6.6.98-sun60iw2, linux-aw2501)

All edits are marked in the sources with /* PATCHxxx: ... */ comments. Active = required.
Passive = diagnostic prints, now behind the runtime knob: `echo 1 | sudo tee /sys/module/vin_io/parameters/vin_debug`.

## The one-line root cause

`sunxi_mipi_g_mbus_config()` received `num_data_lanes = 0` (vendor sensor module built against
pre-5.11 headers). PHY fell to 1 lane, the 2-lane stream was halved, thousands of ECC errors,
empty payloads -> "sensor dead" for 8+ days. PATCH361 forces 2 lanes when 0.

## Patch map

| Patch | File | What |
|---|---|---|
| 311 | sunxi_isp.c:822 | RAW formats fall through to ISP enable (ISP path kept intact) |
| 247 | vin_tdm.c:519,527 | force RX_TX_EN |
| 350 | vin_tdm.c:926-934 | RAW fall-through in RX0 config |
| 360/361 | sunxi_mipi.c:936-940 | diag + force 2 lanes (ROOT CAUSE FIX) |
| 390 | vin_tdm.c:2552 | force TDM_OFFLINE (pkg buffers in DDR) |
| 391/394 | vin_tdm.c | buffer-fill diagnostics (passive, vin_debug) |
| 392 | vin_tdm.c:409,620 | input height 720->716 (sensor sends 716 lines) |
| 393 | vin_tdm.c:2382 | printk "buf0 dma= / buf1 dma=" (userspace learns DDR buffer addresses) |
| 395 | vin_tdm.c:405-406 | pkg_en=1, lbc_en=0 — plain packed RAW10 instead of LBC (kills 1px/line drift) |
| 400 | vin_tdm.c:2244 | ignore RX_BUF_FULL |
| 401 | vin_tdm.c | frame error without TDM reset (stability) |
| 402 | vin_video.c:2029 | safe media-link lookup (list_for_each_entry dangling ptr fix) |
| 403/404 | vin_video.c:3756-3761 | entity naming + enable tdm_rx->isp link |
| 405 | vin_tdm.c | buffer-address print moved to RX1 (tdm_rx[1]) |
| 406 | vin_tdm.c | RX1_FRM_DONE_INT enable |
| 407 | sunxi_mipi.c | PHY for the second port (mipi1) |
| 408 | imx219.c | dual-sensor driver (SENSOR_NUM=2, imx219/imx219_2) |
| 409 | sunxi_csi.c | csi1 parser fixes |
| 410 | vin_tdm.c | per-channel buffer-address printk (rxK:bufN dma_addr) |
| **411** | **vin.c** | **DUAL-CAMERA KEY: `csic_isp_input_select` — "ISP0 input k <- parser of the vinc with tdm_rx_sel=k"** (register CSIC_ISP0_IN0 + 16k) |
| **412-416** | **vin_video.c / vin_tdm.c** | **V4L2 vb2 frame delivery via TDM software completion: RAW10->16-bit unpack in kernel, dma_sync_single_for_cpu invalidate, ioremap; fixes stop-path panic** |
| **417-420** | **imx219.c** | **direct IMX219 exposure/gain/fps controls on the V4L2 nodes (subdev ioctl, applied on stream start)** |

## Dual-camera wiring

cam1: i2c-11, mipi0/csi0, vinc0 (tdm_rx_sel=0) -> /dev/video0.
cam2: i2c-9, mipi1/csi1, vinc8 (tdm_rx_sel=1, overlay sun60i-a733-vinc8-offline.dtbo) -> /dev/video8.
TDM RX0+RX1 are independent channels with own DDR pkg buffers.

## Build

```bash
cd bsp && make -C /lib/modules/$(uname -r)/build M=$PWD/drivers/vin modules ccflags-y="-I$PWD/include"
```

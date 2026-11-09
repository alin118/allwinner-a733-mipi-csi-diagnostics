# allwinner-a733-mipi-csi-diagnostics

**Verdict (updated 2026-11-09): the A733 MIPI CSI receiver is ALIVE. Live video from IMX219 works.**

Minimal RAW10 camera bring-up for Orange Pi Zero 3W (Allwinner A733 / sun60iw2, kernel 6.6.98-sun60iw2)
with a full hardware-level diagnostic history. Two Raspberry Pi Camera V2 clones (IMX219, 22-pin FPC)
were declared dead by the vendor BSP for 8+ days — this repo documents why, and the exact patches
that resurrected the pipeline.

## What works now (verified on hardware, 2026-11-09)

- Live preview on HDMI: ~40 FPS, 1280x720, bayer debayer + AWB + AE + gamma, `/dev/fb0`
- Sensor: IMX219 in mainline-RPi 720p **2x2 binned** mode (2560x1440 window -> 1280x720 out, ~78% sensor width FOV)
- Capture chain: sensor -> MIPI PHY (2 lanes) -> CSI parser -> **TDM RX0 pkg buffer (plain packed RAW10)**
-> userspace `livec` via `/dev/mem` mmap -> fb0. ISP/VIPP/vinc are permanently bypassed (they are broken in BSP).
- Two ping-pong DDR buffers, frame-freshness detection by magic pattern overwrite.

## Root cause (the single real defect)

`sunxi_mipi_g_mbus_config()` received `num_data_lanes = 0`: the vendor sensor module was built against
pre-5.11 kernel headers and never fills the field. Every IS_FLAG check fails, lane count falls through
to 1, the PHY listens on a single lane, the 2-lane stream is halved (644 px), thousands of ECC errors,
empty payloads -> "sensor dead". One patch (`sunxi_mipi.c:937-940`: force 2 lanes when 0) revives the link.

## Repo layout

- `driver/` — patched BSP sources (`imx219_bsp.c`, `vin_tdm.c`, `sunxi_mipi.c`, `sunxi_isp.c`), kernel 6.6.98-sun60iw2
- `userspace/` — `livec.c` (RAW10 unpack + debayer + AWB + AE + fb0 viewer), `runc.sh` (stream orchestrator)
- `PATCHES.md` — every patch with file:line and rationale
- `IMX219_A733_FINAL_REPORT.txt` — full diagnostic history: dead receiver -> root cause -> live video

## Quick start

```bash
# kernel modules (BSP tree ~/linux-aw2501/bsp)
make -C /lib/modules/$(uname -r)/build M=$PWD/drivers/vin modules ccflags-y="-I$PWD/include"
sudo cp drivers/vin/modules/sensor/imx219.ko /lib/modules/$(uname -r)/kernel/bsp/drivers/vin/modules/sensor/
sudo depmod -a

# live preview, 30 s
gcc -O2 -o ~/livec userspace/livec.c -lm
sudo cp userspace/runc.sh ~/
sudo ~/runc.sh 30 3        # 30 = seconds, 3 = bayer phase (RGGB)
```

Buffer addresses are printed by the driver (PATCH393 printk "buf0 dma=... buf1 dma=..."), `runc.sh` picks them up.

## Hardware notes

- Board: Orange Pi Zero 3W, Allwinner A733, 6 GB RAM, 5V/3A
- Both IMX219 clones verified working on Raspberry Pi 5 before the bring-up (i2c 0x10, 24 MHz MCLK, 456 MHz link)
- Kernel: Ubuntu Jammy (Orange Pi 1.0.0), hybrid image with Radxa Cubie A7S GPU/VPU userspace, NPU galcore via MaverickLong repo

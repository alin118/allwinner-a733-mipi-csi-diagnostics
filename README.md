# allwinner-a733-mipi-csi-diagnostics

Minimal RAW10 camera bring-up driver for Orange Pi Zero 3W (Allwinner A733 / sun60iw2)
plus a fully documented verdict: the SoC MIPI CSI receiver does not work on this board.

## TL;DR

We reverse-engineered and brought up the complete camera input chain of a brand-new
Allwinner SoC (clocks, power domains, NSI fabric, IOMMU, CCU gates, CSI top, combo
D-PHY, port, parser, DMA) and proved with an oscilloscope and bit-banged CSI-2
sampling that valid MIPI signals from two different sensors (IMX219 clone, original
OV5647) reach the SoC pads on both camera connectors, yet the combo D-PHY receiver
never decodes a single packet. The vendor driver shows zero frames for the board's
entire lifetime. The defect is hardware (silicon/analog PHY domain); no software can
fix it. This repo preserves the driver (ready to work on a fixed board) and the
complete evidence.

## What works (driver v6.59)

- Full VIN wake-up chain: 16 clock domains, pinctrl (mclk0/1/2 + mipi0/1/2),
  pd_vi + all 12 NSI masters (runtime PM), IOMMU bypass + master_ready + PVT_HANG.
- Vendor-faithful combo-PHY/port/parser config, fixed against five places where the
  vendor BSP sources lie to their own binaries (S2P, ofscal, deskew data lanes,
  DESKEW_EN, unknown +0x084 bit, parser PRS_MODE) by capturing the vendor driver's
  live registers mid-stream.
- Both cameras: auto-detect IMX219 @0x10 / OV5647 @0x36, mainline register tables,
  test pattern, 1/2/4-lane modes, two connectors (CAM1/CAM2, cam2=1).
- XCLR held low by a foreign driver (sensor in eternal reset) raised via
  pinctrl/gpio APIs without ownership.
- Bit-banged CSI-2 capture (PIO bank-K sampling, ~14 Msps APB limit), segment
  activity profile, BMP waveform rendering.
- Diagnostics: live register capture during vendor streaming (do_dump), lane-map /
  polarity / S2P-mask / S2P-delay sweeps, pad probes.

## Evidence matrix

| Sensor | CAM1 (PHY0)                     | CAM2 (PHY1)                      |
|--------|---------------------------------|----------------------------------|
| IMX219 | I2C OK, pads OK (bursts), PHY 0 | -                                |
| OV5647 | I2C OK, pads OK, PHY 0          | I2C OK (bus i2c-9), pads OK, PHY 0 |

- Vendor binary driver: zero frames ever; board historically hangs waiting.
- Scope: data lanes show valid D-PHY HS bursts (LP-11 1.2 V to ~0.2 V common-mode
  toggling); clock lane alive.
- All register-level knobs exhausted. No MBUS/NSI config service in the image.
- Community confirms: no working MIPI CSI driver exists for A733 boards.

## Quick start

    cd imx219_raw
    make
    sudo insmod imx219_raw.ko sensor=auto lanes=2 tpg=1 nodance=1 lpck=0
    dmesg | tail

Params: cam2=1 i2c_bus=9 (second connector), slow=1 op_mpy=1 (bit-bang capture),
single_phy=1, xclr_late=1, s2p_dly=, no_cfg=1, save=/path/cap.bin.

## Files

- imx219_raw.c - the driver (single file, ~2500 lines, commented history of every
  experiment v6.1 to v6.59).
- Makefile - standard kbuild module Makefile.
- IMX219_A733_FINAL_REPORT.txt - full report: register maps, the five BSP
  source-vs-binary divergences, evidence chain, verdict (Russian).

## Verdict

Hardware defect of the MIPI CSI receiver path (combo D-PHY / analog supply /
bonding) on Allwinner A733 as shipped on Orange Pi Zero 3W. Not fixable in
software. The driver is ready for a board with a working receiver.

Driver and investigation by a human plus three AI assistants over ~70 experiments.

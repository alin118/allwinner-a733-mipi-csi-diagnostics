// SPDX-License-Identifier: GPL-2.0
/*
 * imx219_raw.c — v6
 * Minimal RAW10 capture bring-up for IMX219 (Raspberry Pi Camera v2)
 * Board: Orange Pi Zero 3W, SoC: Allwinner A733 (sun60iw2)
 * Kernel: 6.6.98-sun60iw2
 *
 * Pipeline: IMX219 -> MIPI CSI-2 D-PHY -> MIPI host (0x05810100)
 *           -> CSI (0x05820000) -> TDM RX (0x05908000) -> DMA -> DRAM
 *
 * v6 changes vs v5:
 *  - REMOVED bogus write to the nonexistent VIDEO_IN gate register
 *    (sysccu window below RST_BUS_VIDEO_IN; writes there were dropped)
 *  - enable ALL clocks of vind0: index loop over the DT "clocks"
 *    property, no hardcoded names -> covers all 15/16 regardless
 *  - select pinctrl states "mclk0-default" AND "mipi0-default" on the
 *    vind0 device (v5 never selected pinctrl -> MCLK may never reach
 *    the pin even with clk_prepare_enable() == 0)
 *  - enable regulators "vind_mipipin" and "vind_mclkpin"
 *    (both = axp8191 bldo1, 0.5-3.4V -> sensor analog/rails)
 *  - auto power-on from the module: pm_runtime_forbid() on
 *      - soc@3000000:pd_vi_test@0   (genpd wrapper for pd_vi)
 *      - ALL children of 2020000.nsi-controller (covers hidden
 *        tdm/scaler/vinc NSI masters, not just isp/csi)
 *    child names are printed to dmesg -> answers the "truncated
 *    NSI master list" question
 *  - ISP base fixed to the real one: 0x05900000 (was 0x058ffff4)
 *  - new diagnostics:
 *      step_rd on 0x05810000+0x00 (vind base itself, never read)
 *      sysccu dump of 0x1700-0x17fc (never scanned window)
 *  - resets are intentionally NOT re-asserted at rmmod so vendor
 *    modules stay usable after our tests
 *
 * v6.1 additions (after full NSI/IOMMU investigation):
 *  - sunxi_enable_device_iommu(1, false) + (2, false): force IOMMU
 *    bypass for CSI(1)/ISP(2) masters (both EXPORT_SYMBOL'd, in kallsyms)
 *  - sunxi_iommu_master_ready(dev-with-iommus) for every vinc child of
 *    vind: un-skips the master, enables per-master INT and PVT_HANG
 *    (prevent-hang) registers
 *  - pm_runtime_get_sync() on the vind platform device itself
 *    (genpd attach -> pd_vi resume through the device, not out-of-band)
 *  - IOMMU BYP register readbacks at 0x03900404/0x604/0x804
 *
 * v6.2 additions:
 *  - sunxi_iommu_master_ready called ONCE per unique master id (was
 *    once per vinc node = 40+ calls; each triggered a Runtime-PM
 *    underflow WARN whose trace dumps flooded and evicted the whole
 *    init log from the ring buffer)
 *  - IOMMU block moved to the END of init; probe_dump runs a SECOND
 *    time right after it (regions woken by master_ready/PVT_HANG
 *    will show up in the second dump)
 *  - new dump windows: VIN CCU 0x05800000+0x00..0xFC (submodule gate
 *    bits suspected here - the last unscanned alive region) and
 *    CSI_TOP 0x05800800+0x00..0x3C
 * v6.59 (Gemini suggestions):
 *  - S2P_DLY sweep: time_hs_settle (bits16-23 of phy+0x028) swept
 *    0x10..0x40 on the live stream; param s2p_dly for manual value
 *    (0x25 = vendor). The DLY field was never swept before - only the
 *    lane mask was.
 *  - xclr_late=1: hold XCLR low through SoC/PHY config, release right
 *    before streaming (classic LP-11 entry order)
 *
 * v6.58: lane_sweep sweeps the ACTIVE port (port1 when cam2=1)
 *
 * v6.57 (CAM2 connector support):
 *  - cam2=1: camera on the CAM2 connector -> mipi1 pin group
 *    (PK10..19, pk_shift=10), XCLR = PE9 (gpio 137), MCLK2 (PE10),
 *    parser1 @ 0x05821000, port1 for the final dt/vc read, pad-probe
 *    and bitbang masks shifted, i2c_bus param
 *
 * v6.56 (OV5647 support):
 *  - auto sensor detect: IMX219 @0x10 (id 0x0219) / OV5647 @0x36
 *    (id 0x5647); param sensor=imx219|ov5647|auto
 *  - OV5647 (RPi cam v1.3): 1-lane MIPI, needs host MCLK (forced on),
 *    mainline ov5647.c common + 640x480 RAW10 tables, LP-11 pre-entry
 *    + final kick (0x4800=0x04, 0x4202=0, 0x300d=0), TPG 0x503d=0x80
 *  - lanes=1 supported: PHY lanedt 0x10, S2P mask 0x1, port 0x101
 *
 * v6.55 (GPT audit round 3):
 *  - pinctrl for MIPI1/MIPI2 now also guarded by !single_phy (the
 *    remaining unconditional piece from his round-2 note)
 *  - exit: removed the extra pm_runtime_put for vind (one get_sync ->
 *    one put_sync); put_device(g_pddev) added
 *  - pm_runtime_get_sync() failure handling: pm_runtime_put_noidle()
 *    on negative results (vind, pd_vi_test, every NSI child)
 *
 * v6.54 (GPT audit round 2):
 *  - single_phy=1: configure ONLY PHY0/PORT0 and mipi0 pins; PHY1/2
 *    registers AND mipi1/2 pinctrl are left untouched (one-variable
 *    experiment per GPT)
 *  - exit: balanced runtime PM - every get_sync() from init gets a
 *    put_sync() (vind, pd_vi_test, every NSI child); all PM puts
 *    happen BEFORE any put_device()
 *
 *  - clk_get_rate() printed for every enabled clock
 *
 * v6.3 additions (full vin_md_set_power + combo_csi bring-up):
 *  - DT-driven base discovery with phys printed to dmesg:
 *      cmb_top = vind reg index 2 (expects 0x05810000)
 *      phy     = mipi0 reg index 0, port = mipi0 reg index 1
 *              (vendor of_iomap order; phys printed - resolves the
 *               0x05810100 vs 0x05811000 question for good)
 *  - pinctrl "mipi0-default" selected on the mipi0 DEVICE (not vind)
 *  - the whole vendor wake chain, in vendor order:
 *      ccu+0x000 = 0x3          (mode, gating enabled)
 *      ccu+0x004 = 0x10103      (parser0/1, combo0, mipi0 clk)
 *      ccu+0x008 = 0x11011      (misp0, bridge, f2s0/s2f0 ch clk)
 *      csi_top+0x000 = 0x9      (top_en + isp_bridge_en)
 *      csi_top+0x0f0 = 0xF0F0F  (mbus req max)
 *      cmb_top+0x000 = 0xB0F    (PWDNZ|RSTN|VREF|LVLDO|0p2=3|0p9=2)
 *      cmb_top+0x004 = trescal 0xA sequence incl. RESETN toggle
 *      phy+0x000 = 0x04041131   (2 data lanes, 1 ck lane, mipi mode)
 *      phy+0x00C = 0x01300000   (ofscal)
 *      phy+0x018 = 0x00200000   (deskew laneck0 = 2)
 *      phy+0x02C = 0x00011000   (lpck 1 lane + lp dbc)
 *      phy+0x034 = 0x1          (freq counter)
 *      port+0x108 = 0x2B        (DT=0x2B RAW10)
 *      port+0x110 = 0x8000F     (FS|FE|LS|LE + RAW trigger)
 *      port+0x000 = 0x201       (lane_num=2, port_en)
 *    each step followed by step_rd readback on ALIVE regs
 *  - old mipi_bsp_writes() REMOVED: its mipi_ctl values were garbage
 *    for the combo register file
 *  - note: sunxi_nsi_master_ready does NOT exist in this kernel
 *
 * v6.4 additions (sensor streaming):
 *  - PHY/port switched to 4 lanes to match the vendor 1080p table
 *    (imx219 reg 0x0114 = 0x03 -> 4 data lanes, 720 Mbit/lane):
 *    phy+0x000 = 0x040411F1, port+0x000 = 0x401
 *  - full sensor_1080p_regs table from BSP vfe/device/imx219.c
 *    written over i2c-11 @0x10, then 0x0100 = 0x01 (stream on)
 *  - status dump 200 ms after stream on: trescal result (cmb_top+0x004),
 *    phy+0x0F0, port regs, plus stickiness re-probe of csi0/isp/tdm
 *  - dma_set_mask_and_coherent(32) before dma_alloc_coherent
 *
 * v6.5 additions (parser bring-up):
 *  - dma_coerce_mask_and_coherent(32) instead of dma_set_mask (fixes
 *    the NULL dma buffer on the misc device)
 *  - stick test on csi0 (parser0 @ 0x05820000): write/readback
 *  - minimal MIPI parser config per parser_reg_i.h:
 *      +0x008 MCSIC_IF_CFG = 0x00000100  (dw=1 -> RAW10)
 *      +0x00C PRS_CAP      = 0x3         (ch0 still+video capture on)
 *      +0x028 CH0_HSIZE    = 1920<<16    (hor_len)
 *      +0x02C CH0_VSIZE    = 1080<<16    (ver_len)
 *      +0x034 CH0_PARA1    = HT|VT<<16   (from sensor 0x0162/0x0160)
 *      +0x000 PRS_EN       = 0x80008001  (PRS_EN|PCLK_EN|MCSIC_EN)
 *  - PRS_SIGNAL_STA (+0x010) sampled 5x at 60 ms: DATA_STA/VSYNC_STA
 *    must change while the sensor streams
 *
 * v6.6 additions (full port config + live status):
 *  - parser CH0_PARA1 fixed: HT=0x0DE8, VT=0x0A2F (were swapped)
 *  - port CTL = 0x80000401 (EN + 4 lanes + OUT_NUM=TWO_DATA per
 *    vendor: 4-lane mode splits into 2 output streams)
 *  - port MIPI_CFG +0x100 = 0x1 (UNPACK_EN, per cmb_port_mipi_cfg)
 *  - status loop now samples, 5x @60 ms:
 *      prs+0x010  SIGNAL_STA
 *      port+0x118 CH0 INT pending (frame events)
 *      port+0x11C CH0 current packet dt/vc (0x2b = RAW10 flowing)
 *      phy+0x0F0  PHY status (CK0/D0/D1)
 *
 * v6.7 additions (clock evidence):
 *  - phy+0x034 FREQ counter sampled 3x @80 ms (result in bits 16-31;
 *    ~0x168 = 360 MHz lane clock -> sensor really streams at 720 Mbit)
 *  - config readbacks: port+0x000, phy+0x000, cmb_top+0x000 held
 *    against the streaming sensor
 *
 * v6.8 additions (lane map + A/B proof):
 *  - port lane map per BSP cmb_phy_lane[SUN60IW2][0] = {0,1,2,3}:
 *    port+0x004 = 0x00003210, port+0x008 = 0x00000000 (reset default
 *    leaves all IDs 0 -> every port lane listens to PHY lane 0 and
 *    no packet can assemble)
 *  - A/B test: sensor stream OFF (0x0100=0) -> freq counter must
 *    drop; stream ON again -> freq returns (proves the counter
 *    measures the sensor's lanes)
 *
 * v6.9 additions (dual PHY probe - find the camera):
 *  - the board has TWO MIPI interfaces: mipi0 (PK0..PK9) and mipi1
 *    (PK10..PK19); the camera may be routed to mipi1
 *  - mipi1 node (compatible allwinner,sunxi-mipi, device_id 1):
 *    phy = 0x05810200, port = 0x05811400, configured identically
 *  - pinctrl "mipi1-default" selected on the mipi1 device
 *  - ccu+0x004 = 0x00030303 (adds combo1/mipi1 clk gates)
 *  - status loop reports cur dt/vc + PHY sta for BOTH ports
 *
 * v6.10 additions (the real MCLK):
 *  - sensor0_mclk_id in DT = 1: the camera is clocked from MCLK1, but
 *    only mclk0-default pins were selected -> MCLK never reached the
 *    sensor (PLL dead, no MIPI). Now mclk1-default is selected too.
 *
 * v6.11 additions (pinctrl held + polarity experiment):
 *  - BUGFIX: pinctrl handles were dropped right after select
 *    (pinctrl_put), pins reverted - MCLK/MIPI never stayed muxed.
 *    Handles are now held until module exit.
 *  - module param polar=1 inverts D-PHY polarity: phy+0x024 =
 *    0x0000FF00 (HSDT_POLAR all 4 + HSCK_POLAR) on both PHYs.
 *    Try: sudo insmod imx219_raw.ko polar=1
 *
 * v6.12 additions (vendor-config diff mode):
 *  - module param no_cfg=1: skips ALL register writes, sensor
 *    streaming, parser config - only dumps. Use while /dev/video0
 *    is held open by v4l2-ctl (vendor vin_md_set_power) to read
 *    the vendor's own PHY/port/CCU register values and diff them
 *    against ours.
 *
 * v6.13 additions (three-PHY probe + lane-count switch):
 *  - mipi2 found in DT (device_id 2): phy 0x05810300, port 0x05811800,
 *    pinctrl mipi2-default; mipic pins = PK20..PK25 (2 data + ck)
 *  - module param lanes=2|4 (default 4):
 *      sensor reg 0x0114 = 0x01 (2-lane) or 0x03 (4-lane)
 *      PHY  +0x000 lanedt field = 0x3 (2) or 0xF (4)
 *      port +0x000 = 0x201 (2, ONE_DATA) or 0x80000401 (4, TWO_DATA)
 *      port lane map = 0x10 (2) or 0x3210 (4)
 *  - PHY1/PHY2 mipirx DBC only exists on sel 0: their +0x02C = 0x1000
 *  - trigger register = 0x000E000F (FS|FE|LS|LE|YUV|RGB|RAW per vendor)
 *  - status loop also reads port2/phy2
 *
 * v6.14 additions (wake the sensor properly):
 *  - SW reset: write IMX219 0x0103=1 + 10 ms, then table, then
 *    0x0100=1 twice with 50 ms (the vendor uses a hard XCLR pulse
 *    which we cannot drive - GPIO134 is busy; SW reset substitutes)
 *  - readback of 0x0100 and 0x0114 logged after stream on
 *
 * v6.15 additions (MCLK presence test):
 *  - after streaming starts, mux mclk0/mclk1 pins to gpio_in via
 *    the mclk*-sleep states, sample each pin 20000x via gpio:
 *    ~50%% ones = 24 MHz clock present; 0%%/100%% = static
 *    (params mclk0_pin=128 (PE0), mclk1_pin=133 (PE5))
 *    NOTE: this probe was flawed - muxing to gpio_in disconnects
 *    the clock; values are meaningless. Superseded by mclk2-default.
 *
 * v6.16 additions (mclk2 pin mux):
 *  - select "mclk2-default" (PE10) as well: the camera may be routed
 *    to mclk2 - gpio-138 (PE10) was observed as input-low, meaning
 *    no clock output was ever selected on it
 *
 * v6.17 additions (100 percent vendor config coverage):
 *  - port+0x00C = 2 on all three ports (cmb_port_set_wdr_mode, the
 *    vendor writes it unconditionally even for WDR_NONE);
 *    progressive field bit already covered (bit 30 of the DI/trigger
 *    register is 0 in our 0x000E000F)
 *
 * v6.18 additions (old-chat proposals):
 *  - tpg=1 module param: IMX219 test pattern (0x0600=1 color bars)
 *    before stream on (does not need light; still needs MCLK)
 *  - PIO pinmux readback for PE0/PE5/PE10 (mclk0/1/2): reads the
 *    actual pin config register (bank stride 0x24) - verifies the
 *    MCLK function really landed on the pin (pio base from DT,
 *    fallback 0x02000000). func=7 means gpio_in.
 *  - status loop adds port +0x114/+0x118/+0x11C/+0x120/+0x124
 *
 * v6.19 additions (XCLR release + pio scan, from the V1.2 schematic):
 *  - schematic page 13 (MIPI_DPHY_CSI0): CAM1 pin17 = PE6 = camera
 *    XCLR/RESET, pin18 = PE5 = MCLK1, TWI11 = i2c-11, VCC_3V3_CSI
 *    always on. PE6 was found held as gpio input LOW by an unknown
 *    driver (likely vin_io parsing sensor0_pwdn) -> sensor in eternal
 *    reset; I2C still answers, MIPI silent. RPi drives this pin.
 *  - xclr_release(): gpio_request_one(134, OUT_INIT_HIGH); raw PIO
 *    poke is planned for v6.20 once the scan reveals bank layout
 *  - pio scan: dumps 0x80 words at pio base to locate banks (our
 *    earlier fixed-offset read returned zeros - stride unconfirmed)
 *
 * v6.20 additions (XCLR via raw PIO, diff-located):
 *  - pinctrl table confirmed: selector 0x0 = gpio_in, 0x1 = gpio_out
 *  - xclr_pio_poke(): toggles PE5 between mclk1-default/mclk1-sleep
 *    and scans the whole PIO window twice; the word where ONLY bits
 *    20-23 change is PE bank CFG0 -> PE6 nibble = bits 24-27 there.
 *    Then: PE6 -> gpio_out (0x1), DATA bit 6 -> 1 (CFG0+0x10)
 *    NOTE: +0x1fc was a read-only word - writes dropped; the diff
 *    was spurious (PE5's mclk selector IS 0, same as gpio_in, so
 *    its CFG never changes)
 *
 * v6.21 additions (XCLR via official APIs, no address guessing):
 *  - pinctrl_gpio_direction_output(134): muxes PE6 to gpio_out
 *    WITHOUT checking the gpiochip claim
 *  - gpio_set_value(134, 1): writes the DATA bit, also without an
 *    ownership check (classic sunxi idiom for shared pins)
 *
 * v6.22 additions (lane-map auto-sweep):
 *  - XCLR fix proved the sensor really streams: PHY freq counter
 *    jumps 2841->10760 (OFF->ON), i.e. CK lane at ~360 MHz
 *  - polar=1 alone did not decode packets, so sweep 6 lane maps
 *    x 2 polarities on the port; a dt/vc of 0x2B marks the winner
 *
 * v6.23 additions (2-lane sweep phase):
 *  - all earlier lanes=2 tests predated the XCLR fix and were
 *    invalid. Phase 2: reconfigure sensor to 2-lane (0x0114=1),
 *    PHY DT_2LANE, port 2 lanes ONE_DATA, then sweep 6 lane pairs
 *    x 2 polarities (data may land on pairs {2,3} etc.)
 *
 * v6.24 additions (real 2-lane mode - the cable wires only D0/D1/CK):
 *  - the HBV_Raspberry_150FPC adapter wires CAM1 pins 1-10 (D0,D1,CK),
 *    17-18 (XCLR,MCLK), 20-22 (GND,SCL,SDA) - D2/D3 pairs NOT connected
 *  - camera works on RPi5 (2-lane CSI), so lanes=2 is the native mode;
 *    flipping 0x0114 alone with the 4-lane 720Mbps table cannot work
 *    (payload needs 2x912Mbps - requires the proper 2-lane PLL/timing)
 *  - imx219_1080p_2l[]: mainline Linux imx219 table (2-lane):
 *    PLL VT_MPY=57 OP_MPY=114 (912 Mbps/lane), FLL=1763, LLP=3448,
 *    pixel clock 182.4 MHz, plus 12 undocumented regs, TPG=2 colorbars
 *  - lanes default is now 2; TPG writes 0x0600=2 (mainline numbering)
 *  - lane_sweep uses 2-lane map candidates when lanes==2
 *
 * v6.25 additions (vendor register capture):
 *  - param wait_vendor=N: at the end of init sleep N seconds (start
 *    the vendor stream meanwhile) then dump the full config register
 *    set - the definitive diff of vendor values vs ours
 *
 * v6.27 additions (the two real diffs, from in-stream vendor capture):
 *  - prs+0x000 = 0x80008003: vendor sets PRS_MODE (bit 1) for MIPI
 *    input (we only had PRS_EN) - without it the parser ignores MCSIC
 *  - phy+0x028 = 0x00250f00: vendor ENABLES S2P (serial->parallel) on
 *    all data lanes (S2P_EN=0xf) with S2P_DLY=0x25 (time_hs) - without
 *    it data lanes never convert, so CK is alive but no packets reach
 *    the port
 *
 * v6.29 additions:
 *  - nodance param: skip the LP-11 dance (it may have broken 4-lane
 *    emission: freq dropped 0x2a08 -> 0x099a after v6.28)
 *  - post-stream sensor readback: dumps PLL/lane/timing regs to
 *    verify the table actually stuck in the sensor
 *
 * v6.30 additions (electrical probe of the data lanes):
 *  - the freq counter is no longer a stream indicator (S2P changed
 *    what it measures - all ~0x0a00 now). Direct pad check instead:
 *  - diff-locate the PK cfg words by toggling mipi0-default/sleep
 *    (mux really changes here), then read the bank DATA register
 *    (cfg0+0x10) while the sensor streams: ~50% ones = the lane
 *    toggles = signal reaches the SoC pad; static = broken pair
 *    NOTE: v6.30 scanned only 0x200 bytes and found no diff - bank K
 *    (gpio 320+, 11th bank) sits beyond that with larger strides
 *    RESULT: D0N=8678 D1N=8680 ones/20000 - DATA REACHES THE PADS.
 *    The break is inside the SoC (combo PHY does not lock). Bank K:
 *    PIO+0x584 = CFG0 (sel 0x2 = mcsia), +0x594 = DATA.
 *
 * v6.32 additions (full-region vendor capture):
 *  - dump ALL phy regs 0x000-0x0FC and port regs 0x000-0x1FC during
 *    vendor stream - the missing bit may sit in a register we never
 *    read (e.g. phy+0x004)
 *
 * v6.28 additions:
 *  - mainline LP-11 entry dance at stream_on start (brief stream
 *    on/off) - the 2-lane table otherwise may not start the CSI
 *    transmitter (freq counter showed baseline with it, 0x2a08 with
 *    the 4-lane table)
 *  - NOTE: all earlier 4-lane attempts predated the v6.27 S2P/PRS_MODE
 *    fixes - retest lanes=4 (headers go on data lane 0, which is
 *    wired in the cable!)
 *    (header-only API, no symbol, no implementation) - NSI is driven
 *    purely by runtime PM of the nsi_master child devices (v6 forbids)
 *
 * SAFETY: never read 0x0583xxxx (vinc/CSI_DMA) — hangs the board.
 * All reads go through step_rd(): "RD <name> ..." is printed BEFORE
 * the readl; a log line without "= 0x" marks the exact hanging read.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/gpio.h>

/* ------------------------------------------------------------------ */
/* memory map                                                          */
/* ------------------------------------------------------------------ */

#define SYS_CCU_BASE     0x02002000UL
#define SYS_CCU_SIZE     0x2000UL        /* covers 0x1700-0x18fc dumps */

#define VIN_CCU_BASE     0x05800000UL
#define VIN_CCU_SIZE     0x130000UL      /* 0x05800000-0x0592ffff      */

/* offsets inside the VIN_CCU mapping */
#define OFF_CSI_TOP      0x00800         /* 0x05800800 CSI_TOP         */
#define OFF_VIND_BASE    0x10000         /* 0x05810000 third vind reg  */
#define OFF_MIPI0        0x10100         /* 0x05810100 MIPI host ctl   */
#define OFF_PHY0         0x11000         /* 0x05811000 MIPI0 D-PHY     */
#define OFF_CSI0         0x20000         /* 0x05820000 CSI host 0      */
#define OFF_ISP          0x100000        /* 0x05900000 ISP             */
#define OFF_TDM          0x108000        /* 0x05908000 TDM RX          */
#define OFF_SCALER       0x110400        /* 0x05910400 VIPP/scaler     */

/* IOMMU (0x03900000) - alive, sunxi-iommu-v2 driver loaded */
#define IOMMU_BASE     0x03900000UL
#define IOMMU_SIZE     0x10000UL
/* BYP reg per master: 0x200*id + 0x4 + 0x400 (verified vs PATCH64 read) */
#define IOMMU_BYP(n)   (0x400 + 0x200 * (n) + 0x4)

/* sysccu (0x02002000) */
#define SCLK_GATE_CSI      0x1840
#define RST_BUS_CSI        0x1844
#define SCLK_GATE_ISP      0x1860
#define RST_BUS_VIDEO_IN   0x1884

#define I2C_BUS_NUM   11
#define IMX219_ADDR   0x10

#define MAX_CLKS 32

/* ------------------------------------------------------------------ */
/* globals                                                             */
/* ------------------------------------------------------------------ */

static void __iomem *g_sysccu;
static void __iomem *g_ccu;
static void __iomem *g_iommu;

/* exported by sunxi-iommu-v2 (present in /proc/kallsyms) */
extern void sunxi_enable_device_iommu(unsigned int master_id, bool flag);
extern void sunxi_iommu_master_ready(struct device *dev);

static struct device_node      *g_vind_np;
static struct platform_device  *g_pdev;
static struct device           *g_pddev;   /* pd_vi_test              */
static struct device           *g_nsidev;  /* nsi-controller          */

/* v6.3: DT-discovered bases */
static void __iomem *g_cmb_top;            /* csi top (vind reg2)     */
static resource_size_t g_cmb_top_phys;
static struct device_node     *g_mipi_np;
static struct platform_device *g_mipipdev;
static void __iomem *g_phy_base;           /* mipi0 reg0              */
static resource_size_t g_phy_phys;
static void __iomem *g_port_base;          /* mipi0 reg1              */
static resource_size_t g_port_phys;
static struct device_node     *g_mipi1_np;
static struct platform_device *g_mipi1pdev;
static void __iomem *g_phy1_base;
static resource_size_t g_phy1_phys;
static void __iomem *g_port1_base;
static resource_size_t g_port1_phys;
static struct device_node     *g_mipi2_np;
static struct platform_device *g_mipi2pdev;
static void __iomem *g_phy2_base;
static resource_size_t g_phy2_phys;
static void __iomem *g_port2_base;
static resource_size_t g_port2_phys;

static struct clk *g_clks[MAX_CLKS];
static int g_nclks;

static void *g_dma_va;
static dma_addr_t g_dma_pa;
static size_t g_dma_size;

static bool polar;
module_param(polar, bool, 0444);
MODULE_PARM_DESC(polar, "invert D-PHY lane polarity (experiment)");

static bool no_cfg;
module_param(no_cfg, bool, 0444);
MODULE_PARM_DESC(no_cfg, "read-only: dump registers without writing");

static int lanes = 2;
module_param(lanes, int, 0444);
MODULE_PARM_DESC(lanes, "MIPI lane count: 2 (native for this cable) or 4");

static int mclk0_pin = 128;
module_param(mclk0_pin, int, 0444);
MODULE_PARM_DESC(mclk0_pin, "gpio number of the mclk0 pin (PE0=128)");

static int mclk1_pin = 133;
module_param(mclk1_pin, int, 0444);
MODULE_PARM_DESC(mclk1_pin, "gpio number of the mclk1 pin (PE5=133)");

static bool tpg;
module_param(tpg, bool, 0444);
MODULE_PARM_DESC(tpg, "enable IMX219 test pattern (0x0600=2 color bars)");

static bool nodance;
module_param(nodance, bool, 0444);
MODULE_PARM_DESC(nodance, "skip the LP-11 entry dance");

static bool nomclk;
module_param(nomclk, bool, 0444);
MODULE_PARM_DESC(nomclk, "do NOT drive MCLK pins - the camera module has its own oscillator (no 24 MHz seen on RPi5)");

static int s2p_en = -1;
module_param(s2p_en, int, 0444);
MODULE_PARM_DESC(s2p_en, "S2P lane enable mask override (-1 = auto: 0x3 for 2 lanes, 0xf for 4)");

static int s2p_dly = 0x25;
module_param(s2p_dly, int, 0444);
MODULE_PARM_DESC(s2p_dly, "S2P_DLY (time_hs_settle) bits16-23 of phy+0x028; 0x25 = vendor value");

static bool xclr_late;
module_param(xclr_late, bool, 0444);
MODULE_PARM_DESC(xclr_late, "v6.59: hold XCLR LOW during SoC/PHY config, release right before streaming (classic LP-11 order)");

static bool phyoff;
module_param(phyoff, bool, 0444);
MODULE_PARM_DESC(phyoff, "pad probe runs with the PHY powered down (PWDNZ=0) - is the toggling from the camera or from the PHY itself?");

static int deskew_en = 1;
module_param(deskew_en, int, 0444);
MODULE_PARM_DESC(deskew_en, "write phy+0x030 deskew-enable value (vendor uses DK_1LANE=1; -1 = do not touch)");

static int ck_en = 1;
module_param(ck_en, int, 0444);
MODULE_PARM_DESC(ck_en, "phy+0x000 LANECK_EN field (0/1/2) - 1 may enable a CK output driver that fights the camera");

static int lpck = 1;
module_param(lpck, int, 0444);
MODULE_PARM_DESC(lpck, "phy+0x02C LPCK_EN field (0 = do not drive LP clock, 1 = LPCK_1LANE per BSP)");

static int term_val;
module_param(term_val, int, 0444);
MODULE_PARM_DESC(term_val, "phy+0x020 value override (default 0 = per BSP; try enables+dly e.g. 0x110011)");

static int hs_val;
module_param(hs_val, int, 0444);
MODULE_PARM_DESC(hs_val, "phy+0x024 value override (default 0 = per BSP; try enables+dly)");

static bool slow;
module_param(slow, bool, 0444);
MODULE_PARM_DESC(slow, "Plan Z: 1-lane slow sensor mode (640x480, low PLL rate) for bit-banged CSI-2 capture");

static int op_mpy = 12;
module_param(op_mpy, int, 0444);
MODULE_PARM_DESC(op_mpy, "slow-mode PLL_OP_MPY (12 ~ 96 Mbit/s, 6 ~ 48 Mbit/s)");

static int trig_idle = 200;
module_param(trig_idle, int, 0444);
MODULE_PARM_DESC(trig_idle, "consecutive LP-11 reads required to arm the burst trigger (unused in v6.50 simple trigger)");

static bool notrig;
module_param(notrig, bool, 0444);
MODULE_PARM_DESC(notrig, "skip the trigger, capture immediately");

static bool xvs_trig;
module_param(xvs_trig, bool, 0444);
MODULE_PARM_DESC(xvs_trig, "trigger on XVS frame sync (PE5, bank E DATA bit 5) instead of the lanes");

static char save_path[64] = "/mnt/usb/cap.bin";
module_param_string(save, save_path, sizeof(save_path), 0644);
MODULE_PARM_DESC(save, "file to write the capture to (empty = do not save)");

static char sensor_name[16] = "auto";
module_param_string(sensor, sensor_name, sizeof(sensor_name), 0444);
MODULE_PARM_DESC(sensor, "sensor: auto / imx219 / ov5647");

static bool cam2;
module_param(cam2, bool, 0444);
MODULE_PARM_DESC(cam2, "v6.57: camera on the CAM2 connector (mipi1: PHY1/port1, PK10-19, XCLR=PE9/gpio137, MCLK2=PE10)");

static int i2c_bus = 11;
module_param(i2c_bus, int, 0444);
MODULE_PARM_DESC(i2c_bus, "i2c adapter number for the sensor");

/* v6.57: XCLR gpio + PK bit offset depend on the connector */
static int xclr_gpio = 134;
static int pk_shift;			/* 0 = CAM1 (PK0..), 10 = CAM2 (PK10..) */

/* v6.56: auto-detected sensor */
static u16 g_sensor_addr = 0x10;	/* 0x10 = IMX219, 0x36 = OV5647 */
static int g_sensor_type;		/* 0 = IMX219, 1 = OV5647 */

static bool single_phy;
module_param(single_phy, bool, 0444);
MODULE_PARM_DESC(single_phy, "v6.54: configure ONLY PHY0/PORT0+pins; PHY1/2 and mipi1/2 pinctrl untouched");

static int cap_mb = 32;
module_param(cap_mb, int, 0444);
MODULE_PARM_DESC(cap_mb, "capture buffer size in MB (default 32 -> ~8M samples -> full slow frame)");



static int pefi = 1;
module_param(pefi, int, 0444);
MODULE_PARM_DESC(pefi, "phy+0x000 LP_PEFI+HS_PEFI enable (1 = per BSP, 0 = lines left free)");

static bool phyreinit = 1;
module_param(phyreinit, bool, 0444);
MODULE_PARM_DESC(phyreinit, "re-run PHY top power + trescal AFTER the sensor streams (calibrate on a live signal)");

static bool xclr_pulse = 1;
module_param(xclr_pulse, bool, 0444);
MODULE_PARM_DESC(xclr_pulse, "hardware XCLR pulse (low 100 ms, high 10 ms) right before the config table - datasheet power-on sequence");

static int wait_vendor;
module_param(wait_vendor, int, 0444);
MODULE_PARM_DESC(wait_vendor, "seconds to wait at end of init before dumping (run vendor stream meanwhile)");

static void vendor_capture_dump(void);

static int do_dump;
static int do_dump_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (do_dump) {
		do_dump = 0;
		vendor_capture_dump();
	}
	return ret;
}
static const struct kernel_param_ops do_dump_ops = {
	.set = do_dump_set,
	.get = param_get_int,
};
module_param_cb(do_dump, &do_dump_ops, &do_dump, 0644);
MODULE_PARM_DESC(do_dump, "write 1 to dump config registers at any time");

/* held pinctrl handles - released only at module exit */
static struct pinctrl *g_pc_vind, *g_pc_mipi0, *g_pc_mipi1, *g_pc_mipi2;

/* ------------------------------------------------------------------ */
/* step_rd — the only safe read pattern                                */
/* ------------------------------------------------------------------ */

static void step_rd(const char *name, void __iomem *addr)
{
	u32 v;

	/* printed BEFORE the read: if the next line has no "= 0x...",
	 * this exact read is where the board hung */
	pr_info("imx219_raw: RD %s (%p) ...\n", name, addr);
	v = readl(addr);
	pr_info("imx219_raw:     %s = 0x%08x\n", name, v);
}

static void step_rd_window(const char *name, void __iomem *base,
			   u32 off0, u32 off1)
{
	u32 off;
	char nm[48];

	for (off = off0; off <= off1; off += 4) {
		snprintf(nm, sizeof(nm), "%s+0x%03x", name, off);
		step_rd(nm, base + off);
	}
}

/* ------------------------------------------------------------------ */
/* clocks: enable EVERYTHING vind0 has, by index (no hardcoded names)  */
/* ------------------------------------------------------------------ */

static void vind_clocks_on(struct device_node *np)
{
	int i, n;

	n = of_property_count_strings(np, "clock-names");
	if (n <= 0)
		n = of_property_count_elems_of_size(np, "clocks",
						    sizeof(__be32));
	if (n <= 0) {
		pr_warn("imx219_raw: no clocks found on vind node\n");
		return;
	}
	if (n > MAX_CLKS)
		n = MAX_CLKS;

	for (i = 0; i < n; i++) {
		const char *nm = "?";
		struct clk *c;

		of_property_read_string_index(np, "clock-names", i, &nm);
		c = of_clk_get(np, i);
		if (IS_ERR(c)) {
			pr_warn("imx219_raw: clk[%d] '%s' get failed (%ld)\n",
				i, nm, PTR_ERR(c));
			continue;
		}
		if (clk_prepare_enable(c)) {
			pr_warn("imx219_raw: clk[%d] '%s' enable failed\n",
				i, nm);
			clk_put(c);
			continue;
		}
		g_clks[g_nclks++] = c;
		pr_info("imx219_raw: clk[%02d] '%s' ON rate=%lu\n", i, nm,
			clk_get_rate(c));
	}
	pr_info("imx219_raw: %d clocks enabled (of %d listed)\n",
		g_nclks, n);
}

static void vind_clocks_off(void)
{
	int i;

	for (i = 0; i < g_nclks; i++) {
		clk_disable_unprepare(g_clks[i]);
		clk_put(g_clks[i]);
		g_clks[i] = NULL;
	}
	g_nclks = 0;
}

/* ------------------------------------------------------------------ */
/* pinctrl: put MCLK and MIPI lanes onto pins                          */
/* ------------------------------------------------------------------ */

static void vind_pinctrl_select_one(struct pinctrl *p, const char *name)
{
	struct pinctrl_state *s;
	int ret;

	s = pinctrl_lookup_state(p, name);
	if (IS_ERR(s)) {
		pr_warn("imx219_raw: pinctrl state '%s' missing (%ld)\n",
			name, PTR_ERR(s));
		return;
	}
	ret = pinctrl_select_state(p, s);
	pr_info("imx219_raw: pinctrl select '%s' ret=%d\n", name, ret);
}

static void vind_pinctrl_select(struct device *dev)
{
	struct pinctrl *p;

	p = pinctrl_get(dev);
	if (IS_ERR(p)) {
		pr_warn("imx219_raw: pinctrl_get failed (%ld)\n", PTR_ERR(p));
		return;
	}
	vind_pinctrl_select_one(p, "mclk0-default");
	vind_pinctrl_select_one(p, "mipi0-default");
	pinctrl_put(p);
}

/* ------------------------------------------------------------------ */
/* regulators: bldo1 rails to the sensor/header                        */
/* ------------------------------------------------------------------ */

static void vind_regulator_on(struct device *dev, const char *name)
{
	struct regulator *r;
	int ret, uv;

	r = devm_regulator_get(dev, name);
	if (IS_ERR(r)) {
		pr_warn("imx219_raw: regulator '%s' get failed (%ld)\n",
			name, PTR_ERR(r));
		return;
	}
	ret = regulator_enable(r);
	uv = regulator_get_voltage(r);
	pr_info("imx219_raw: regulator '%s' enable ret=%d voltage=%d uV\n",
		name, ret, uv);
}

/* ------------------------------------------------------------------ */
/* runtime PM: force pd_vi and ALL NSI masters to stay on              */
/* ------------------------------------------------------------------ */

static int nsi_forbid_one(struct device *dev, void *data)
{
	pm_runtime_forbid(dev);
	/* v6.55: get_sync bumps usage_count even on failure */
	if (pm_runtime_get_sync(dev) < 0)
		pm_runtime_put_noidle(dev);
	pr_info("imx219_raw: NSI master '%s' forced ON\n", dev_name(dev));
	return 0;
}

static int nsi_allow_one(struct device *dev, void *data)
{
	/* v6.54: balance the pm_runtime_get_sync() from nsi_forbid_one */
	pm_runtime_put_sync(dev);
	pm_runtime_allow(dev);
	return 0;
}

static void auto_power_on(void)
{
	g_pddev = bus_find_device_by_name(&platform_bus_type, NULL,
					  "soc@3000000:pd_vi_test@0");
	if (!g_pddev) {
		pr_warn("imx219_raw: pd_vi_test device NOT found\n");
	} else {
		pm_runtime_forbid(g_pddev);
		if (pm_runtime_get_sync(g_pddev) < 0)
			pm_runtime_put_noidle(g_pddev);
		pr_info("imx219_raw: pd_vi_test forbidden (pd_vi forced ON)\n");
	}

	g_nsidev = bus_find_device_by_name(&platform_bus_type, NULL,
					   "2020000.nsi-controller");
	if (!g_nsidev) {
		pr_warn("imx219_raw: nsi-controller NOT found\n");
		return;
	}
	/* forbid EVERY child: list in dmesg shows whether hidden
	 * tdm/scaler/vinc NSI masters exist */
	device_for_each_child(g_nsidev, NULL, nsi_forbid_one);
}

/* ------------------------------------------------------------------ */
/* sysccu resets                                                       */
/* ------------------------------------------------------------------ */

static void csi_bus_reset_deassert(void)
{
	u32 v;

	v = readl(g_sysccu + RST_BUS_CSI);
	writel(v | BIT(16), g_sysccu + RST_BUS_CSI);
	pr_info("imx219_raw: RST_BUS_CSI (0x1844) = 0x%08x\n",
		readl(g_sysccu + RST_BUS_CSI));

	v = readl(g_sysccu + RST_BUS_VIDEO_IN);
	writel(v | BIT(16), g_sysccu + RST_BUS_VIDEO_IN);
	pr_info("imx219_raw: RST_BUS_VIDEO_IN (0x1884) = 0x%08x\n",
		readl(g_sysccu + RST_BUS_VIDEO_IN));
	/* deliberately nothing else: the gate register that used to be
	 * written below RST_BUS_VIDEO_IN does not exist (v6 fix) */
}

/* ------------------------------------------------------------------ */
/* IMX219 over the existing i2c adapter (vendor client already there)  */
/* ------------------------------------------------------------------ */

static int imx219_read16(struct i2c_adapter *adap, u16 reg)
{
	u8 buf[2] = { reg >> 8, reg & 0xff };
	u8 val = 0;
	struct i2c_msg msg[2] = {
		{ .addr = g_sensor_addr, .flags = 0,
		  .len = 2, .buf = buf },
		{ .addr = g_sensor_addr, .flags = I2C_M_RD,
		  .len = 1, .buf = &val },
	};

	if (i2c_transfer(adap, msg, 2) != 2)
		return -EIO;
	return val;
}

/* IMX219 1920x1080 @30, 2-lane, RAW10 (mainline Linux imx219.c) */
static const u16 imx219_1080p_2l[][2] = {
	{0x0100, 0x00},
	{0x30eb, 0x05}, {0x30eb, 0x0c}, {0x300a, 0xff}, {0x300b, 0xff},
	{0x30eb, 0x05}, {0x30eb, 0x09},
	{0x455e, 0x00}, {0x471e, 0x4b}, {0x4767, 0x0f}, {0x4750, 0x14},
	{0x4540, 0x00}, {0x47b4, 0x14}, {0x4713, 0x30},
	{0x478b, 0x10}, {0x478f, 0x10}, {0x4793, 0x10},
	{0x4797, 0x0e}, {0x479b, 0x0e},
	{0x0170, 0x01}, {0x0171, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18}, {0x012b, 0x00},
	/* frame A: crop 680,692 .. 2599,1771 -> 1920x1080 */
	{0x0164, 0x02}, {0x0165, 0xa8},
	{0x0166, 0x0a}, {0x0167, 0x27},
	{0x0168, 0x02}, {0x0169, 0xb4},
	{0x016a, 0x06}, {0x016b, 0xeb},
	{0x016c, 0x07}, {0x016d, 0x80},
	{0x016e, 0x04}, {0x016f, 0x38},
	{0x0174, 0x00}, {0x0175, 0x00},
	{0x018c, 0x0a}, {0x018d, 0x0a},
	{0x0624, 0x07}, {0x0625, 0x80},
	{0x0626, 0x04}, {0x0627, 0x38},
	/* 2-lane PLL: 912 Mbps/lane */
	{0x0301, 0x05}, {0x0303, 0x01}, {0x0304, 0x03}, {0x0305, 0x03},
	{0x0306, 0x00}, {0x0307, 0x39},
	{0x0309, 0x0a}, {0x030b, 0x01},
	{0x030c, 0x00}, {0x030d, 0x72},
	{0x0114, 0x01},
	/* timing: pixel clock 182.4 MHz */
	{0x0160, 0x06}, {0x0161, 0xe3},
	{0x0162, 0x0d}, {0x0163, 0x78},
	{0x0100, 0x01},
};

/* Plan Z: 1-lane slow mode, 640x480 crop, PLL divided down.
 * fps scales by 114/op_mpy (op_mpy=12 -> ~3 fps); CSI-2 packets identical */
static const u16 imx219_slow[][2] = {
	{0x0100, 0x00},
	{0x30eb, 0x05}, {0x30eb, 0x0c}, {0x300a, 0xff}, {0x300b, 0xff},
	{0x30eb, 0x05}, {0x30eb, 0x09},
	{0x455e, 0x00}, {0x471e, 0x4b}, {0x4767, 0x0f}, {0x4750, 0x14},
	{0x4540, 0x00}, {0x47b4, 0x14}, {0x4713, 0x30},
	{0x478b, 0x10}, {0x478f, 0x10}, {0x4793, 0x10},
	{0x4797, 0x0e}, {0x479b, 0x0e},
	{0x0170, 0x01}, {0x0171, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18}, {0x012b, 0x00},
	/* crop: X 1328..1967, Y 992..1471 -> 640x480 */
	{0x0164, 0x05}, {0x0165, 0x30},
	{0x0166, 0x07}, {0x0167, 0xaf},
	{0x0168, 0x03}, {0x0169, 0xe0},
	{0x016a, 0x05}, {0x016b, 0xbf},
	{0x016c, 0x02}, {0x016d, 0x80},
	{0x016e, 0x01}, {0x016f, 0xe0},
	{0x0174, 0x00}, {0x0175, 0x00},
	{0x018c, 0x0a}, {0x018d, 0x0a},
	/* PLL with reduced OP_MPY (rate param) */
	{0x0301, 0x05}, {0x0303, 0x01}, {0x0304, 0x03}, {0x0305, 0x03},
	{0x0306, 0x00}, {0x0307, 0x39},
	{0x0309, 0x0a}, {0x030b, 0x01},
	{0x0114, 0x00},			/* 1 lane */
	{0x0160, 0x06}, {0x0161, 0xe3},
	{0x0162, 0x0d}, {0x0163, 0x78},
	{0x0100, 0x01},
};

/* OV5647 (RPi cam v1.3): common init, mainline ov5647.c */
static const u16 ov5647_common[][2] = {
	{0x0100, 0x00}, {0x0103, 0x01},
	{0x3034, 0x1a}, {0x3035, 0x21}, {0x303c, 0x11}, {0x3106, 0xf5},
	{0x3827, 0xec}, {0x370c, 0x03}, {0x5000, 0x06}, {0x5003, 0x08},
	{0x5a00, 0x08}, {0x3000, 0x00}, {0x3001, 0x00}, {0x3002, 0x00},
	{0x3016, 0x08}, {0x3017, 0xe0}, {0x3018, 0x44}, {0x301c, 0xf8},
	{0x301d, 0xf0}, {0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3c01, 0x80},
	{0x3b07, 0x0c}, {0x3630, 0x2e}, {0x3632, 0xe2}, {0x3633, 0x23},
	{0x3634, 0x44}, {0x3636, 0x06}, {0x3620, 0x64}, {0x3621, 0xe0},
	{0x3600, 0x37}, {0x3704, 0xa0}, {0x3703, 0x5a}, {0x3715, 0x78},
	{0x3717, 0x01}, {0x3731, 0x02}, {0x370b, 0x60}, {0x3705, 0x1a},
	{0x3f05, 0x02}, {0x3f06, 0x10}, {0x3f01, 0x0a}, {0x3a08, 0x01},
	{0x3a0f, 0x58}, {0x3a10, 0x50}, {0x3a1b, 0x58}, {0x3a1e, 0x50},
	{0x3a11, 0x60}, {0x3a1f, 0x28}, {0x4001, 0x02}, {0x4000, 0x09},
	{0x3503, 0x03},
};

/* OV5647 640x480 RAW10 1-lane (mainline mode table, xclk 24-25 MHz) */
static const u16 ov5647_vga[][2] = {
	{0x3036, 0x46}, {0x3821, 0x03}, {0x3820, 0x41},
	{0x3612, 0x59}, {0x3618, 0x00}, {0x3814, 0x35}, {0x3815, 0x35},
	{0x3708, 0x64}, {0x3709, 0x52},
	{0x3800, 0x00}, {0x3801, 0x10}, {0x3802, 0x00}, {0x3803, 0x00},
	{0x3804, 0x0a}, {0x3805, 0x2f}, {0x3806, 0x07}, {0x3807, 0x9f},
	{0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01}, {0x380b, 0xe0},
	{0x3a09, 0x2e}, {0x3a0a, 0x00}, {0x3a0b, 0xfb}, {0x3a0d, 0x02},
	{0x3a0e, 0x01}, {0x4004, 0x02}, {0x4800, 0x34},
	{0x503d, 0x00},	/* test pattern off by default */
	{0x0100, 0x01},
};

/* IMX219 1920x1080 @30, 4-lane, RAW10 (BSP sensor_1080p_regs) */
static const u16 imx219_1080p[][2] = {
	{0x30EB, 0x05}, {0x30EB, 0x0C}, {0x300A, 0xFF}, {0x300B, 0xFF},
	{0x30EB, 0x05}, {0x30EB, 0x09},
	{0x0114, 0x03}, {0x0128, 0x00}, {0x012A, 0x18}, {0x012B, 0x00},
	{0x0160, 0x0A}, {0x0161, 0x2F}, {0x0162, 0x0D}, {0x0163, 0xE8},
	{0x0164, 0x02}, {0x0165, 0xA8}, {0x0166, 0x0A}, {0x0167, 0x27},
	{0x0168, 0x02}, {0x0169, 0xB4}, {0x016A, 0x06}, {0x016B, 0xEB},
	{0x016C, 0x07}, {0x016D, 0x80}, {0x016E, 0x04}, {0x016F, 0x38},
	{0x0170, 0x01}, {0x0171, 0x01}, {0x0174, 0x00}, {0x0175, 0x00},
	{0x018C, 0x0A}, {0x018D, 0x0A},
	{0x0301, 0x05}, {0x0303, 0x01}, {0x0304, 0x03}, {0x0305, 0x03},
	{0x0306, 0x00}, {0x0307, 0x57}, {0x0309, 0x0A}, {0x030B, 0x01},
	{0x030C, 0x00}, {0x030D, 0x5A},
	{0x4767, 0x0F}, {0x4750, 0x14}, {0x47B4, 0x14},
	{0x0100, 0x01},
};

static int imx219_write16(struct i2c_adapter *adap, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg = { .addr = g_sensor_addr, .flags = 0,
			       .len = 3, .buf = buf };

	return i2c_transfer(adap, &msg, 1) == 1 ? 0 : -EIO;
}

static void imx219_stream_on(void)
{
	struct i2c_adapter *adap;
	int i, ret = 0;

	adap = i2c_get_adapter(i2c_bus);
	if (!adap) {
		pr_warn("imx219_raw: stream_on: no i2c-%d\n", i2c_bus);
		return;
	}
	if (!nodance) {
		/* v6.28: mainline LP-11 entry dance */
		imx219_write16(adap, 0x0100, 0x01);
		usleep_range(100, 150);
		imx219_write16(adap, 0x0100, 0x00);
		msleep(10);
	}
	if (xclr_pulse) {
		/* v6.42: datasheet power-on pulse */
		gpio_set_value(134, 0);
		msleep(100);
		gpio_set_value(134, 1);
		msleep(10);
	}
	imx219_write16(adap, 0x0100, 0x00);	/* standby */
	msleep(20);
	/* v6.14: SW reset (XCLR pulse substitute) */
	imx219_write16(adap, 0x0103, 0x01);
	msleep(10);
	imx219_write16(adap, 0x0100, 0x00);
	msleep(10);
	if (slow) {
		for (i = 0; i < ARRAY_SIZE(imx219_slow) - 1; i++) {
			u8 v = imx219_slow[i][1];

			if (imx219_slow[i][0] == 0x030c)
				v = (op_mpy >> 8) & 0xff;
			if (imx219_slow[i][0] == 0x030d)
				v = op_mpy & 0xff;
			ret |= imx219_write16(adap, imx219_slow[i][0], v);
		}
	} else if (lanes == 4) {
		for (i = 0; i < ARRAY_SIZE(imx219_1080p) - 1; i++)
			ret |= imx219_write16(adap, imx219_1080p[i][0],
					      imx219_1080p[i][1]);
	} else {
		for (i = 0; i < ARRAY_SIZE(imx219_1080p_2l) - 1; i++)
			ret |= imx219_write16(adap, imx219_1080p_2l[i][0],
					      imx219_1080p_2l[i][1]);
	}
	if (ret)
		pr_warn("imx219_raw: table write errors\n");
	msleep(20);
	if (tpg)
		imx219_write16(adap, 0x0600, 0x02);	/* color bars */
	msleep(5);
	imx219_write16(adap, 0x0100, 0x01);	/* stream on */
	msleep(50);
	imx219_write16(adap, 0x0100, 0x01);	/* again */
	msleep(20);
	pr_info("imx219_raw: v6.14: rb 0x0100=%d 0x0114=%d\n",
		imx219_read16(adap, 0x0100), imx219_read16(adap, 0x0114));
	pr_info("imx219_raw: v6.29: rb pll 0x0304=%d 0x0305=%d 0x0306=%d 0x0307=%d 0x0309=%d 0x030b=%d 0x030c=%d 0x030d=%d\n",
		imx219_read16(adap, 0x0304), imx219_read16(adap, 0x0305),
		imx219_read16(adap, 0x0306), imx219_read16(adap, 0x0307),
		imx219_read16(adap, 0x0309), imx219_read16(adap, 0x030b),
		imx219_read16(adap, 0x030c), imx219_read16(adap, 0x030d));
	pr_info("imx219_raw: v6.29: rb tim 0x0160=%d 0x0161=%d 0x0162=%d 0x0163=%d out 0x016c=%d 0x016e=%d\n",
		imx219_read16(adap, 0x0160), imx219_read16(adap, 0x0161),
		imx219_read16(adap, 0x0162), imx219_read16(adap, 0x0163),
		imx219_read16(adap, 0x016c), imx219_read16(adap, 0x016e));
	pr_info("imx219_raw: v6.29: rb fmt 0x018c=%d 0x0174=%d tp 0x0600=%d\n",
		imx219_read16(adap, 0x018c), imx219_read16(adap, 0x0174),
		imx219_read16(adap, 0x0600));
	i2c_put_adapter(adap);
	pr_info("imx219_raw: v6.4: sensor stream ON, table errs=%d\n", ret);
}

static void imx219_stream_off(void)
{
	struct i2c_adapter *adap = i2c_get_adapter(i2c_bus);

	if (!adap)
		return;
	imx219_write16(adap, 0x0100, 0x00);
	i2c_put_adapter(adap);
}

/* v6.56: probe IMX219 @0x10 (id 0x0219) and OV5647 @0x36 (id 0x5647) */
static void sensor_detect(struct i2c_adapter *adap)
{
	int a = 0x10, hi, lo;
	int want_imx = !strcmp(sensor_name, "imx219");
	int want_ov  = !strcmp(sensor_name, "ov5647");

	/* v6.59b: detection needs the sensor out of reset (XCLR high),
	 * even when xclr_late holds it low for the PHY-config phase */
	gpio_set_value(xclr_gpio, 1);
	msleep(10);

	g_sensor_addr = 0x10;
	hi = imx219_read16(adap, 0x0000);
	lo = imx219_read16(adap, 0x0001);
	if (hi == 0x02 && lo == 0x19) {
		g_sensor_type = 0;
		g_sensor_addr = 0x10;
		pr_info("imx219_raw: v6.56: IMX219 @0x10 (0x%02x%02x)\n", hi, lo);
		if (xclr_late) {
			gpio_set_value(xclr_gpio, 0);
			pr_info("imx219_raw: v6.59: XCLR re-asserted for the PHY-config phase\n");
		}
		return;
	}
	if (want_imx) {
		pr_warn("imx219_raw: v6.56: imx219 requested but id 0x%02x%02x\n",
			hi, lo);
		return;
	}

	g_sensor_addr = 0x36;
	hi = imx219_read16(adap, 0x300a);
	lo = imx219_read16(adap, 0x300b);
	if (hi == 0x56 && lo == 0x47) {
		g_sensor_type = 1;
		pr_info("imx219_raw: v6.56: OV5647 @0x36 (0x%02x%02x) 1-lane, needs MCLK\n",
			hi, lo);
		if (xclr_late) {
			gpio_set_value(xclr_gpio, 0);
			pr_info("imx219_raw: v6.59: XCLR re-asserted for the PHY-config phase\n");
		}
		return;
	}
	if (want_ov)
		pr_warn("imx219_raw: v6.56: ov5647 requested but id 0x%02x%02x\n",
			hi, lo);
	g_sensor_addr = a;
	pr_warn("imx219_raw: v6.56: no known sensor (0x10:?, 0x36:0x%02x%02x)\n",
		hi, lo);
}

static void ov5647_stream_on(struct i2c_adapter *adap)
{
	int i;

	/* mainline sequence: LP-11 pre-entry, SW reset, common, mode, kick */
	imx219_write16(adap, 0x4800, 0x25);
	imx219_write16(adap, 0x4202, 0x0f);
	imx219_write16(adap, 0x300d, 0x01);
	msleep(20);
	imx219_write16(adap, 0x0103, 0x01);
	msleep(5);
	for (i = 0; i < ARRAY_SIZE(ov5647_common); i++)
		imx219_write16(adap, ov5647_common[i][0], ov5647_common[i][1]);
	for (i = 0; i < ARRAY_SIZE(ov5647_vga); i++) {
		if (ov5647_vga[i][0] == 0x503d && tpg)
			continue;
		imx219_write16(adap, ov5647_vga[i][0], ov5647_vga[i][1]);
	}
	if (tpg) {
		imx219_write16(adap, 0x503d, 0x80);	/* color bars */
		msleep(5);
	}
	/* final kick: MIPI_CTRL00 = BUS_IDLE, frames on, pad out */
	imx219_write16(adap, 0x4800, 0x04);
	imx219_write16(adap, 0x4202, 0x00);
	imx219_write16(adap, 0x300d, 0x00);
	pr_info("imx219_raw: v6.56: ov5647 stream on (tpg=%d)\n", tpg);
}

static void sensor_stream_on(void)
{
	if (g_sensor_type == 1) {
		struct i2c_adapter *adap = i2c_get_adapter(i2c_bus);

		if (!adap)
			return;
		gpio_set_value(xclr_gpio, 0);
		msleep(20);
		gpio_set_value(xclr_gpio, 1);
		msleep(30);
		ov5647_stream_on(adap);
		i2c_put_adapter(adap);
		return;
	}
	if (xclr_late) {
		gpio_set_value(xclr_gpio, 1);
		msleep(10);
		pr_info("imx219_raw: v6.59: XCLR released, streaming now\n");
	}
	imx219_stream_on();
}

static void imx219_check_id(void)
{
	struct i2c_adapter *adap;
	int hi, lo;

	adap = i2c_get_adapter(i2c_bus);
	if (!adap) {
		pr_warn("imx219_raw: i2c-%d adapter not found\n", I2C_BUS_NUM);
		return;
	}
	hi = imx219_read16(adap, 0x0000);
	lo = imx219_read16(adap, 0x0001);
	if (hi == 0x02 && lo == 0x19)
		pr_info("imx219_raw: IMX219 id ok (0x%02x%02x)\n", hi, lo);
	else
		pr_warn("imx219_raw: IMX219 id unexpected (0x%02x%02x)\n",
			hi, lo);
	i2c_put_adapter(adap);
}

/* ------------------------------------------------------------------ */
/* v6.1: IOMMU bypass + master_ready + vind runtime resume             */
/* ------------------------------------------------------------------ */

static void iommu_bypass_and_ready(void)
{
	struct device_node *child;
	struct device tmpdev;

	if (!g_iommu) {
		pr_warn("imx219_raw: v6.1: iommu not mapped, skipping\n");
		return;
	}

	pr_info("imx219_raw: v6.1: force IOMMU bypass: CSI(1) ISP(2)\n");
	sunxi_enable_device_iommu(1, false);
	sunxi_enable_device_iommu(2, false);

	/* sunxi_iommu_master_ready() only reads dev->of_node, so a
	 * zeroed stack device with of_node pointed at a vinc child
	 * (which carries iommus = <&mmu_aw N 0>) is sufficient */
	if (g_vind_np) {
		unsigned int seen = 0;

		memset(&tmpdev, 0, sizeof(tmpdev));
		for_each_child_of_node(g_vind_np, child) {
			struct of_phandle_args args;
			unsigned int id;

			if (of_parse_phandle_with_args(child, "iommus",
						       "#iommu-cells", 0,
						       &args) != 0 ||
			    args.args_count != 2)
				continue;
			id = args.args[0];
			if (id > 31 || (seen & (1u << id)))
				continue;
			seen |= (1u << id);
			pr_info("imx219_raw: v6.1: iommu_master_ready: %s master %u\n",
				of_node_full_name(child), id);
			tmpdev.of_node = child;
			sunxi_iommu_master_ready(&tmpdev);
		}
		tmpdev.of_node = NULL;
	} else {
		pr_warn("imx219_raw: v6.1: vind node missing, master_ready skipped\n");
	}

	step_rd("iommu BYP(0) 0x03900404", g_iommu + IOMMU_BYP(0));
	step_rd("iommu BYP(1) 0x03900604", g_iommu + IOMMU_BYP(1));
	step_rd("iommu BYP(2) 0x03900804", g_iommu + IOMMU_BYP(2));
}

/* ------------------------------------------------------------------ */
/* register bring-up sequences (BSP-derived, harmless while regs dead) */
/* ------------------------------------------------------------------ */

static u32 s2p_value(void);

static void s2p_dly_sweep(void)
{
	static const u32 dlys[] = {
		0x10, 0x15, 0x1a, 0x20, 0x25, 0x2a, 0x30, 0x35, 0x40
	};
	void __iomem *prt = cam2 ? g_port1_base : g_port_base;
	u32 enmask = s2p_value() & 0x0000FF00;
	int i;

	if (!g_phy_base || !prt)
		return;
	pr_info("imx219_raw: v6.59: S2P_DLY sweep on live stream\n");
	for (i = 0; i < ARRAY_SIZE(dlys); i++) {
		u32 dtvc, sta;

		writel((dlys[i] << 16) | enmask, g_phy_base + 0x028);
		msleep(80);
		dtvc = readl(prt + 0x11C);
		sta = readl(g_phy_base + 0x0F0);
		pr_info("imx219_raw: v6.59: dly=0x%02x -> dt/vc=0x%08x sta=0x%08x\n",
			dlys[i], dtvc, sta);
		if ((dtvc & 0x3f) == 0x2b) {
			pr_info("imx219_raw: v6.59: *** MATCH S2P_DLY=0x%02x ***\n",
				dlys[i]);
			return;
		}
	}
	writel((0x25 << 16) | enmask, g_phy_base + 0x028);
	pr_info("imx219_raw: v6.59: no dly match, restored 0x25\n");
}

static void phy_recalibrate(void)
{
	u32 v;

	if (!g_cmb_top)
		return;
	pr_info("imx219_raw: v6.41: PHY recalibration on live signal\n");
	writel(0x00000B0F, g_cmb_top + 0x000);
	writel(0x000000A3, g_cmb_top + 0x004);
	v = readl(g_cmb_top + 0x004);
	writel((v & ~0x10) | (0xA << 16), g_cmb_top + 0x004);
	v = readl(g_cmb_top + 0x004);
	writel(v & ~0x4, g_cmb_top + 0x004);
	v = readl(g_cmb_top + 0x004);
	writel(v | 0x4, g_cmb_top + 0x004);
	msleep(50);
	pr_info("imx219_raw: v6.41: trescal result after recal = 0x%x (bits16-20)\n",
		(readl(g_cmb_top + 0x004) >> 16) & 0x1F);
	if (g_phy_base)
		step_rd("phy+0x0F0 (post-recal)", g_phy_base + 0x0F0);
}

/* ------------------------------------------------------------------ */
/* v6.22: lane-map auto-sweep (port sees clock but no packets)         */
/* ------------------------------------------------------------------ */

static void lane_sweep(void)
{
	void __iomem *swp = cam2 ? g_port1_base : g_port_base;
	static const u32 maps4[] = {
		0x3210, 0x2310, 0x3201, 0x2301, 0x1032, 0x0123
	};
	static const u32 maps2[] = {
		0x00000010, 0x00000021, 0x00000032,
		0x00000001, 0x00000020, 0x00000031
	};
	const u32 *maps = (lanes == 4) ? maps4 : maps2;
	int nmaps = (lanes == 4) ? ARRAY_SIZE(maps4) : ARRAY_SIZE(maps2);
	int mi, po;

	if (!swp || !g_phy_base)
		return;
	pr_info("imx219_raw: v6.22: lane-map sweep start (port %s)\n",
		cam2 ? "1 (CAM2)" : "0 (CAM1)");
	for (po = 0; po < 2; po++) {
		writel(po ? 0x0000FF00 : 0x00000000, g_phy_base + 0x024);
		for (mi = 0; mi < nmaps; mi++) {
			u32 dtvc, pd;

			writel(maps[mi], swp + 0x004);
			writel(0x00000000, swp + 0x008);
			msleep(120);
			dtvc = readl(swp + 0x11C);
			pd = readl(swp + 0x118);
			pr_info("imx219_raw: v6.22: map=0x%x polar=%d -> dt/vc=0x%08x int=0x%08x\n",
				maps[mi], po, dtvc, pd);
			if ((dtvc & 0x3f) == 0x2b) {
				pr_info("imx219_raw: v6.22: *** MATCH map=0x%x polar=%d dt/vc=0x%08x ***\n",
					maps[mi], po, dtvc);
				return;
			}
		}
	}
	writel(0x00003210, swp + 0x004);
	writel(0x00000000, g_phy_base + 0x024);
	pr_info("imx219_raw: v6.22: no match, restored defaults\n");
}

/* ------------------------------------------------------------------ */
/* v6.5: CSI parser bring-up (0x05820000)                              */
/* ------------------------------------------------------------------ */

static void parser_bringup(void)
{
	void __iomem *prs = g_ccu + (cam2 ? 0x21000 : OFF_CSI0);
	u32 v;
	int i;

	pr_info("imx219_raw: v6.5: ---- parser bring-up ----\n");

	/* stick test (PRS_EN + PRS_MODE per vendor) */
	step_rd("prs+0x000 (before)", prs + 0x000);
	writel(0x80008003, prs + 0x000);
	v = readl(prs + 0x000);
	pr_info("imx219_raw: v6.5: parser stick test: wrote 0x80008003 read 0x%08x\n", v);
	if (v != 0x80008003) {
		pr_warn("imx219_raw: v6.5: parser does NOT stick, aborting\n");
		return;
	}

	/* minimal MIPI RAW10 cfg */
	writel(0x00000100, prs + 0x008);	/* MCSIC_IF_CFG dw=RAW10 */
	writel(0x00000003, prs + 0x00C);	/* ch0 still+video cap on */
	writel(1920 << 16, prs + 0x028);	/* CH0 hor_len */
	writel(1080 << 16, prs + 0x02C);	/* CH0 ver_len */
	writel(0x0A2F << 16 | 0x0DE8, prs + 0x034); /* VT|HT (fixed) */
	step_rd("prs+0x008 (rb)", prs + 0x008);
	step_rd("prs+0x028 (rb)", prs + 0x028);
	step_rd("prs+0x034 (rb)", prs + 0x034);

	/* v6.7: config held against streaming sensor? */
	if (g_port_base)
		step_rd("port+0x000 (ctl rb)", g_port_base + 0x000);
	if (g_phy_base)
		step_rd("phy+0x000 (ctl rb)", g_phy_base + 0x000);
	if (g_cmb_top)
		step_rd("cmb_top+0x000 (ctl rb)", g_cmb_top + 0x000);

	/* sample live statuses while sensor streams */
	for (i = 0; i < 5; i++) {
		msleep(60);
		step_rd("prs SIGNAL_STA", prs + 0x010);
		if (g_port_base) {
			step_rd("port+0x114", g_port_base + 0x114);
			step_rd("port+0x118 (ch0 int)", g_port_base + 0x118);
			step_rd("port+0x11C (cur dt/vc)", g_port_base + 0x11C);
			step_rd("port+0x120", g_port_base + 0x120);
			step_rd("port+0x124", g_port_base + 0x124);
		}
		if (g_phy_base) {
			step_rd("phy+0x0F0 (sta)", g_phy_base + 0x0F0);
			if (i < 3)
				step_rd("phy+0x034 (freq)", g_phy_base + 0x034);
		}
		if (g_port1_base) {
			step_rd("port1+0x118 (ch0 int)", g_port1_base + 0x118);
			step_rd("port1+0x11C (cur dt/vc)", g_port1_base + 0x11C);
		}
		if (g_phy1_base)
			step_rd("phy1+0x0F0 (sta)", g_phy1_base + 0x0F0);
		if (g_port2_base) {
			step_rd("port2+0x114", g_port2_base + 0x114);
			step_rd("port2+0x118 (ch0 int)", g_port2_base + 0x118);
			step_rd("port2+0x11C (dt/vc)", g_port2_base + 0x11C);
			step_rd("port2+0x120", g_port2_base + 0x120);
			step_rd("port2+0x124", g_port2_base + 0x124);
		}
		if (g_phy2_base)
			step_rd("phy2+0x0F0 (sta)", g_phy2_base + 0x0F0);
	}
	/* v6.8 A/B: stop sensor, freq must drop; restart, freq returns */
	if (g_phy_base && !no_cfg) {
		pr_info("imx219_raw: v6.8: A/B sensor OFF...\n");
		imx219_stream_off();
		msleep(150);
		step_rd("phy+0x034 (freq, OFF)", g_phy_base + 0x034);
		pr_info("imx219_raw: v6.8: sensor back ON...\n");
		sensor_stream_on();
		msleep(150);
		step_rd("phy+0x034 (freq, ON)", g_phy_base + 0x034);
		if (g_port_base)
			step_rd("port+0x11C (dt/vc after map)",
		(cam2 ? g_port1_base : g_port_base) + 0x11C);
	}
	s2p_dly_sweep();
	if (phyreinit)
		phy_recalibrate();
	lane_sweep();
	pr_info("imx219_raw: v6.8: ---- parser bring-up done ----\n");
}

static u32 s2p_value(void)
{
	u32 en = (s2p_en >= 0) ? (u32)s2p_en :
		 (lanes == 4 ? 0xf : (lanes == 2 ? 0x3 : 0x1));

	return ((u32)s2p_dly << 16) | (en << 8);
}

static u32 phy_ctl_value(u32 lanedt)
{
	u32 v = 0x04000001 | lanedt;	/* IBIAS_EN + PHY0_EN */

	v |= ((u32)ck_en & 0x3) << 8;	/* LANECK_EN */
	if (pefi) {
		v |= 0x1000;	/* LP_PEFI */
		v |= 0x40000;	/* HS_PEFI = 4 << 16 */
	}
	return v;
}

static u32 mipirx_value(bool with_dbc)
{
	u32 v = ((u32)lpck & 0x3) << 12;

	if (with_dbc)
		v |= 0x10000;	/* DBC_EN, sel 0 only */
	return v;
}

/* ------------------------------------------------------------------ */
/* v6.3: DT base discovery                                             */
/* ------------------------------------------------------------------ */

static void dt_find_bases(void)
{
	struct resource res;

	/* cmb_top = vind reg index 2 */
	if (g_vind_np &&
	    of_address_to_resource(g_vind_np, 2, &res) == 0) {
		g_cmb_top_phys = res.start;
		g_cmb_top = ioremap(res.start, resource_size(&res));
		pr_info("imx219_raw: v6.3: cmb_top phys=0x%pa size=0x%zx virt=%p\n",
			&res.start, (size_t)resource_size(&res), g_cmb_top);
	} else {
		g_cmb_top_phys = 0x05810000;
		g_cmb_top = ioremap(0x05810000, 0x100);
		pr_warn("imx219_raw: v6.3: cmb_top from DT failed, fallback 0x05810000\n");
	}

	/* mipi0 node: compatible allwinner,sunxi-mipi, device_id 0 */
	g_mipi_np = of_find_compatible_node(NULL, NULL,
					    "allwinner,sunxi-mipi");
	while (g_mipi_np) {
		u32 id = 0;

		of_property_read_u32(g_mipi_np, "device_id", &id);
		if (id == 0)
			break;
		g_mipi_np = of_find_compatible_node(g_mipi_np, NULL,
						    "allwinner,sunxi-mipi");
	}
	if (!g_mipi_np) {
		pr_warn("imx219_raw: v6.3: mipi0 node not found by compatible\n");
	} else {
		pr_info("imx219_raw: v6.3: mipi node: %s\n",
			of_node_full_name(g_mipi_np));
		if (of_address_to_resource(g_mipi_np, 0, &res) == 0) {
			g_phy_phys = res.start;
			g_phy_base = ioremap(res.start, resource_size(&res));
			pr_info("imx219_raw: v6.3: phy phys=0x%pa size=0x%zx virt=%p\n",
				&res.start, (size_t)resource_size(&res), g_phy_base);
		}
		if (of_address_to_resource(g_mipi_np, 1, &res) == 0) {
			g_port_phys = res.start;
			g_port_base = ioremap(res.start, resource_size(&res));
			pr_info("imx219_raw: v6.3: port phys=0x%pa size=0x%zx virt=%p\n",
				&res.start, (size_t)resource_size(&res), g_port_base);
		}
		g_mipipdev = of_find_device_by_node(g_mipi_np);
		if (g_mipipdev)
			pr_info("imx219_raw: v6.3: mipi device: %s\n",
				dev_name(&g_mipipdev->dev));
	}

	/* v6.9: mipi1 (device_id 1) */
	g_mipi1_np = of_find_compatible_node(NULL, NULL,
					     "allwinner,sunxi-mipi");
	while (g_mipi1_np) {
		u32 id = 0;

		of_property_read_u32(g_mipi1_np, "device_id", &id);
		if (id == 1)
			break;
		g_mipi1_np = of_find_compatible_node(g_mipi1_np, NULL,
						    "allwinner,sunxi-mipi");
	}
	if (!g_mipi1_np) {
		pr_warn("imx219_raw: v6.9: mipi1 node not found\n");
		return;
	}
	pr_info("imx219_raw: v6.9: mipi1 node: %s\n",
		of_node_full_name(g_mipi1_np));
	if (of_address_to_resource(g_mipi1_np, 0, &res) == 0) {
		g_phy1_phys = res.start;
		g_phy1_base = ioremap(res.start, resource_size(&res));
		pr_info("imx219_raw: v6.9: phy1 phys=0x%pa size=0x%zx virt=%p\n",
			&res.start, (size_t)resource_size(&res), g_phy1_base);
	}
	if (of_address_to_resource(g_mipi1_np, 1, &res) == 0) {
		g_port1_phys = res.start;
		g_port1_base = ioremap(res.start, resource_size(&res));
		pr_info("imx219_raw: v6.9: port1 phys=0x%pa size=0x%zx virt=%p\n",
			&res.start, (size_t)resource_size(&res), g_port1_base);
	}
	g_mipi1pdev = of_find_device_by_node(g_mipi1_np);
	if (g_mipi1pdev)
		pr_info("imx219_raw: v6.9: mipi1 device: %s\n",
			dev_name(&g_mipi1pdev->dev));

	/* v6.13: mipi2 (device_id 2) */
	g_mipi2_np = of_find_compatible_node(NULL, NULL,
					     "allwinner,sunxi-mipi");
	while (g_mipi2_np) {
		u32 id = 0;

		of_property_read_u32(g_mipi2_np, "device_id", &id);
		if (id == 2)
			break;
		g_mipi2_np = of_find_compatible_node(g_mipi2_np, NULL,
						    "allwinner,sunxi-mipi");
	}
	if (!g_mipi2_np) {
		pr_warn("imx219_raw: v6.13: mipi2 node not found\n");
		return;
	}
	pr_info("imx219_raw: v6.13: mipi2 node: %s\n",
		of_node_full_name(g_mipi2_np));
	if (of_address_to_resource(g_mipi2_np, 0, &res) == 0) {
		g_phy2_phys = res.start;
		g_phy2_base = ioremap(res.start, resource_size(&res));
		pr_info("imx219_raw: v6.13: phy2 phys=0x%pa size=0x%zx virt=%p\n",
			&res.start, (size_t)resource_size(&res), g_phy2_base);
	}
	if (of_address_to_resource(g_mipi2_np, 1, &res) == 0) {
		g_port2_phys = res.start;
		g_port2_base = ioremap(res.start, resource_size(&res));
		pr_info("imx219_raw: v6.13: port2 phys=0x%pa size=0x%zx virt=%p\n",
			&res.start, (size_t)resource_size(&res), g_port2_base);
	}
	g_mipi2pdev = of_find_device_by_node(g_mipi2_np);
	if (g_mipi2pdev)
		pr_info("imx219_raw: v6.13: mipi2 device: %s\n",
			dev_name(&g_mipi2pdev->dev));
}

/* ------------------------------------------------------------------ */
/* v6.3: the whole vendor wake chain (vin_md_set_power + combo_csi)    */
/* ------------------------------------------------------------------ */

static void combo_bringup(void)
{
	u32 v;

	pr_info("imx219_raw: v6.3: ---- combo bring-up ----\n");

	/* 1. VIN CCU clock gates (csic_ccu_base = 0x05800000, ALIVE) */
	step_rd("ccu+0x004 (before)", g_ccu + 0x004);
	writel(0x3, g_ccu + 0x000);	/* mode + gating enabled */
	writel(0x30303, g_ccu + 0x004);	/* parser0/1, combo0/1, mipi0/1 */
	writel(0x11011, g_ccu + 0x008);	/* misp0, bridge, f2s0/s2f0 ch */
	step_rd("ccu+0x004 (after)", g_ccu + 0x004);
	step_rd("ccu+0x008 (after)", g_ccu + 0x008);

	/* 2. CSIC TOP enable (ALIVE) */
	v = readl(g_ccu + OFF_CSI_TOP + 0x000);
	v &= ~(1u << 20);		/* bridge ch en (clear disable) */
	v |= 0x9;			/* top_en + isp_bridge_en */
	writel(v, g_ccu + OFF_CSI_TOP + 0x000);
	writel(0xF0F0F, g_ccu + OFF_CSI_TOP + 0x0F0); /* mbus req max */
	step_rd("csi_top+0x000 (after)", g_ccu + OFF_CSI_TOP + 0x000);

	/* 3. combo PHY top power (cmb_top, was dead, expect wake) */
	if (g_cmb_top) {
		writel(0xB0F, g_cmb_top + 0x000);
		/* trescal 0xA: auto+soft+set, vendor-quirk result write,
		 * then resetn toggle */
		writel(0xA3, g_cmb_top + 0x004);
		v = readl(g_cmb_top + 0x004);
		writel((v & ~0x10) | (0xA << 16), g_cmb_top + 0x004);
		v = readl(g_cmb_top + 0x004);
		writel(v & ~0x4, g_cmb_top + 0x004);
		v = readl(g_cmb_top + 0x004);
		writel(v | 0x4, g_cmb_top + 0x004);
		step_rd("cmb_top+0x000 (after)", g_cmb_top + 0x000);
		step_rd("cmb_top+0x004 (after)", g_cmb_top + 0x004);
	} else {
		pr_warn("imx219_raw: v6.3: cmb_top not mapped\n");
	}

	/* 4. PHY lane/config regs (phy base, was dead) */
	if (g_phy_base) {
		u32 lanedt = (lanes == 4) ? 0xF0 : (lanes == 2 ? 0x30 : 0x10);

		writel(phy_ctl_value(lanedt), g_phy_base + 0x000);
		/* v6.33: ofscal triple-write (vendor capture) */
		writel(0x10101000, g_phy_base + 0x008);
		writel(0x01300000, g_phy_base + 0x00C);
		writel(0x01300000, g_phy_base + 0x010);
		writel(0x00200000, g_phy_base + 0x01C); /* deskew laned0 (vendor) */
		writel(0x00010000, g_phy_base + 0x084); /* vendor-set bit */
		if (deskew_en >= 0)
			writel((u32)deskew_en, g_phy_base + 0x030);
		writel(0x00200000, g_phy_base + 0x018);
		writel((u32)term_val, g_phy_base + 0x020);
		writel((u32)hs_val, g_phy_base + 0x024);
		writel(s2p_value(), g_phy_base + 0x028); /* S2P en all + dly */
		writel(mipirx_value(1), g_phy_base + 0x02C);
		writel(0x00000001, g_phy_base + 0x034);
		if (polar)
			writel(0x0000FF00, g_phy_base + 0x024); /* inv polar */
		pr_info("imx219_raw: v6.13: lanes=%d\n", lanes);
		step_rd("phy+0x000 (wr-rb)", g_phy_base + 0x000);
		step_rd("phy+0x02C (wr-rb)", g_phy_base + 0x02C);
	} else {
		pr_warn("imx219_raw: v6.3: phy base not mapped\n");
	}

	/* 5. PORT cfg (port base, was dead) */
	if (g_port_base) {
		if (lanes == 4) {
			writel(0x00003210, g_port_base + 0x004);
			writel(0x00000000, g_port_base + 0x008);
			writel(0x80000401, g_port_base + 0x000);
		} else if (lanes == 2) {
			writel(0x00000010, g_port_base + 0x004);
			writel(0x00000201, g_port_base + 0x000);
		} else {
			writel(0x00000000, g_port_base + 0x004);
			writel(0x00000101, g_port_base + 0x000);
		}
		step_rd("port+0x004 (map rb)", g_port_base + 0x004);
		writel(0x2B, g_port_base + 0x108);	/* DT=RAW10 */
		writel(0x000E000F, g_port_base + 0x110); /* full trigger */
		writel(0x00000002, g_port_base + 0x00C); /* WDR_MODE per vendor */
		writel(0x00000001, g_port_base + 0x100); /* MIPI unpack en */
		step_rd("port+0x000 (wr-rb)", g_port_base + 0x000);
	} else {
		pr_warn("imx219_raw: v6.3: port base not mapped\n");
	}

	/* v6.9/6.13: PHY1/port1 (lanes param; no DBC off sel 0)
	 * v6.54: skipped entirely when single_phy=1 */
	if (g_phy1_base && !single_phy) {
		u32 lanedt = (lanes == 4) ? 0xF0 : 0x30;

		writel(phy_ctl_value(lanedt), g_phy1_base + 0x000);
		writel(0x10101000, g_phy1_base + 0x008);
		writel(0x01300000, g_phy1_base + 0x00C);
		writel(0x01300000, g_phy1_base + 0x010);
		writel(0x00200000, g_phy1_base + 0x01C);
		writel(0x00010000, g_phy1_base + 0x084);
		if (deskew_en >= 0)
			writel((u32)deskew_en, g_phy1_base + 0x030);
		writel(0x00200000, g_phy1_base + 0x018);
		writel(s2p_value(), g_phy1_base + 0x028);
		writel(mipirx_value(0), g_phy1_base + 0x02C); /* no DBC off sel0 */
		writel(0x00000001, g_phy1_base + 0x034);
		if (polar)
			writel(0x0000FF00, g_phy1_base + 0x024);
		step_rd("phy1+0x000 (rb)", g_phy1_base + 0x000);
	} else {
		pr_warn("imx219_raw: v6.9: phy1 not mapped\n");
	}
	if (g_port1_base && !single_phy) {
		if (lanes == 4) {
			writel(0x00003210, g_port1_base + 0x004);
			writel(0x80000401, g_port1_base + 0x000);
		} else {
			writel(0x00000010, g_port1_base + 0x004);
			writel(0x00000201, g_port1_base + 0x000);
		}
		writel(0x00000001, g_port1_base + 0x100);
		writel(0x2B, g_port1_base + 0x108);
		writel(0x000E000F, g_port1_base + 0x110);
		writel(0x00000002, g_port1_base + 0x00C);
		step_rd("port1+0x000 (rb)", g_port1_base + 0x000);
	} else if (!g_port1_base) {
		pr_warn("imx219_raw: v6.9: port1 not mapped\n");
	}

	/* v6.13: PHY2/port2 - 2-lane only (mipi2 spec {2,2})
	 * v6.54: skipped entirely when single_phy=1 */
	if (g_phy2_base && !single_phy) {
		writel(phy_ctl_value(0x30), g_phy2_base + 0x000); /* 2 data lanes */
		writel(0x10101000, g_phy2_base + 0x008);
		writel(0x01300000, g_phy2_base + 0x00C);
		writel(0x01300000, g_phy2_base + 0x010);
		writel(0x00200000, g_phy2_base + 0x01C);
		writel(0x00010000, g_phy2_base + 0x084);
		if (deskew_en >= 0)
			writel((u32)deskew_en, g_phy2_base + 0x030);
		writel(0x00200000, g_phy2_base + 0x018);
		writel(0x00250300, g_phy2_base + 0x028);
		writel(mipirx_value(0), g_phy2_base + 0x02C); /* no DBC off sel0 */
		writel(0x00000001, g_phy2_base + 0x034);
		step_rd("phy2+0x000 (rb)", g_phy2_base + 0x000);
	} else {
		pr_warn("imx219_raw: v6.13: phy2 not mapped\n");
	}
	if (g_port2_base && !single_phy) {
		writel(0x00000010, g_port2_base + 0x004); /* map 0,1 */
		writel(0x00000201, g_port2_base + 0x000); /* 2 lanes ONE_DATA */
		writel(0x00000001, g_port2_base + 0x100);
		writel(0x2B, g_port2_base + 0x108);
		writel(0x000E000F, g_port2_base + 0x110);
		writel(0x00000002, g_port2_base + 0x00C);
		step_rd("port2+0x000 (rb)", g_port2_base + 0x000);
	} else {
		pr_warn("imx219_raw: v6.13: port2 not mapped\n");
	}

	pr_info("imx219_raw: v6.9: ---- combo bring-up done ----\n");
}

static void tdm_bsp_writes(void)
{
	/* TODO(BSP): port register sequences from
	 * ~/linux-aw2501/bsp/drivers/vin/vin-tdm/tdm100/tdm_reg.h
	 * when TDM region answers with something other than zero.
	 */
	pr_info("imx219_raw: TDM config skipped in v6 (BSP port pending)\n");
}

/* ------------------------------------------------------------------ */
/* v6.15: MCLK presence test                                           */
/* ------------------------------------------------------------------ */

static void mclk_presence_check(void)
{
	int ones0 = -1, ones1 = -1, i;

	if (!g_pc_vind)
		return;

	/* mux the mclk pins to gpio inputs */
	vind_pinctrl_select_one(g_pc_vind, "mclk0-sleep");
	vind_pinctrl_select_one(g_pc_vind, "mclk1-sleep");
	msleep(20);

	if (gpio_request(mclk0_pin, "mclk0_probe") == 0) {
		ones0 = 0;
		for (i = 0; i < 20000; i++)
			ones0 += gpio_get_value(mclk0_pin);
		gpio_free(mclk0_pin);
	}
	if (gpio_request(mclk1_pin, "mclk1_probe") == 0) {
		ones1 = 0;
		for (i = 0; i < 20000; i++)
			ones1 += gpio_get_value(mclk1_pin);
		gpio_free(mclk1_pin);
	}

	pr_info("imx219_raw: v6.15: MCLK probe: mclk0_pin=%d ones=%d/20000, mclk1_pin=%d ones=%d/20000 (~10000 = 24MHz present)\n",
		mclk0_pin, ones0, mclk1_pin, ones1);

	/* restore clock output muxing */
	vind_pinctrl_select_one(g_pc_vind, "mclk0-default");
	vind_pinctrl_select_one(g_pc_vind, "mclk1-default");
}

/* ------------------------------------------------------------------ */
/* v6.18: PIO pinmux readback (PE0/PE5/PE10 mclk pins)                 */
/* ------------------------------------------------------------------ */

static void pio_pinmux_check(void)
{
	struct device_node *pio;
	struct resource res;
	void __iomem *base;
	u32 v;

	pio = of_find_node_by_name(NULL, "pio");
	if (!pio)
		pio = of_find_node_by_name(NULL, "pinctrl");
	if (!pio)
		pio = of_find_compatible_node(NULL, NULL,
					      "allwinner,sunxi-pinctrl");
	if (pio && of_address_to_resource(pio, 0, &res) != 0)
		pio = NULL;
	if (!pio) {
		res.start = 0x02000000;
		res.end = res.start + 0x400;
		pr_warn("imx219_raw: v6.18: pio node not found, fallback 0x02000000\n");
	} else {
		pr_info("imx219_raw: v6.18: pio node: %s\n",
			of_node_full_name(pio));
	}
	base = ioremap(res.start, 0x400);
	if (!base) {
		pr_warn("imx219_raw: v6.18: pio ioremap failed\n");
		return;
	}
	pr_info("imx219_raw: v6.18: pio base phys=0x%pa\n", &res.start);

	/* bank E = 4, stride 0x24; PE0/PE5 in cfg0 (+0x90), PE10 in cfg1 */
	v = readl(base + 4 * 0x24 + 0 * 4);
	pr_info("imx219_raw: v6.18: PE cfg0 (PE0..PE7) = 0x%08x: PE0 func=%u PE5 func=%u (7=gpio_in)\n",
		v, v & 7, (v >> 20) & 7);
	v = readl(base + 4 * 0x24 + 1 * 4);
	pr_info("imx219_raw: v6.18: PE cfg1 (PE8..PE15) = 0x%08x: PE10 func=%u\n",
		v, (v >> 8) & 7);
	v = readl(base + 4 * 0x24 + 0x14);
	pr_info("imx219_raw: v6.18: PE drv0 = 0x%08x\n", v);
	v = readl(base + 4 * 0x24 + 0x1C);
	pr_info("imx219_raw: v6.18: PE pull0 = 0x%08x\n", v);
	of_node_put(pio);

	iounmap(base);
}

/* ------------------------------------------------------------------ */
/* v6.19: XCLR (PE6/gpio-134) release + pio layout scan                */
/* ------------------------------------------------------------------ */

static void pio_scan(void)
{
	struct device_node *pio;
	struct resource res;
	void __iomem *base;
	int i;

	pio = of_find_node_by_name(NULL, "pinctrl");
	if (!pio || of_address_to_resource(pio, 0, &res) != 0) {
		res.start = 0x02000000;
		res.end = res.start + 0x400;
	}
	base = ioremap(res.start, 0x400);
	if (!base)
		return;
	pr_info("imx219_raw: v6.19: pio scan @ 0x%pa (nonzero words):\n",
		&res.start);
	for (i = 0; i < 0x80; i++) {
		u32 v = readl(base + i * 4);

		if (v)
			pr_info("imx219_raw: v6.19:   +0x%03x = 0x%08x\n",
				i * 4, v);
	}
	iounmap(base);
}

static void xclr_release(void)
{
	int ret;

	pr_info("imx219_raw: v6.57: releasing camera XCLR (gpio %d)\n", xclr_gpio);
	ret = gpio_request_one(xclr_gpio, GPIOF_OUT_INIT_HIGH, "imx219_xclr");
	if (ret) {
		pr_warn("imx219_raw: v6.19: gpio_request_one(134) failed %d - pin held by another driver, raw PIO poke in v6.20\n",
			ret);
		return;
	}
	gpio_set_value(134, 1);
	pr_info("imx219_raw: v6.19: XCLR driven HIGH via gpio134\n");
}

/* ------------------------------------------------------------------ */
/* v6.20: XCLR (PE6) forced high via raw PIO, self-locating            */
/* ------------------------------------------------------------------ */

static void xclr_pio_poke(void)
{
	struct device_node *pio;
	struct resource res;
	void __iomem *base;
	u32 a[0x80], b[0x80];
	int i, found = -1;

	pio = of_find_node_by_name(NULL, "pinctrl");
	if (!pio || of_address_to_resource(pio, 0, &res) != 0) {
		res.start = 0x02000000;
		res.end = res.start + 0x400;
	}
	base = ioremap(res.start, 0x400);
	if (!base)
		return;

	/* scan with mclk1-default active */
	for (i = 0; i < 0x80; i++)
		a[i] = readl(base + i * 4);
	vind_pinctrl_select_one(g_pc_vind, "mclk1-sleep");
	msleep(5);
	for (i = 0; i < 0x80; i++)
		b[i] = readl(base + i * 4);
	vind_pinctrl_select_one(g_pc_vind, "mclk1-default");

	for (i = 0; i < 0x80; i++) {
		u32 d = a[i] ^ b[i];

		if (d)
			pr_info("imx219_raw: v6.20: diff +0x%03x: 0x%08x -> 0x%08x\n",
				i * 4, a[i], b[i]);
		if (d == (d & (0xFu << 20)))
			found = i;
	}
	if (found < 0) {
		pr_warn("imx219_raw: v6.20: PE5 cfg word not isolated, skip poke\n");
		iounmap(base);
		return;
	}
	pr_info("imx219_raw: v6.20: PE CFG0 at +0x%03x (PE5 sel was 0x%x)\n",
		found * 4, (a[found] >> 20) & 0xF);

	/* PE6 -> gpio_out (0x1) */
	{
		u32 v = a[found];

		v &= ~(0xFu << 24);
		v |= (0x1u << 24);
		writel(v, base + found * 4);
		pr_info("imx219_raw: v6.20: PE6 cfg -> gpio_out, word=0x%08x\n",
			readl(base + found * 4));
		/* DATA at CFG0+0x10, bit 6 */
		{
			u32 d = readl(base + found * 4 + 0x10);

			writel(d | (1u << 6), base + found * 4 + 0x10);
			pr_info("imx219_raw: v6.20: PE DATA=0x%08x\n",
				readl(base + found * 4 + 0x10));
		}
	}
	iounmap(base);
	pr_info("imx219_raw: v6.20: XCLR (PE6) forced HIGH via raw PIO\n");
}

static void xclr_api_drive(void)
{
	int ret;

	pr_info("imx219_raw: v6.21: driving XCLR via pinctrl+gpio APIs\n");
	ret = pinctrl_gpio_direction_output(134);
	pr_info("imx219_raw: v6.21: pinctrl_gpio_direction_output(134) = %d\n",
		ret);
	if (ret)
		return;
	if (xclr_late) {
		gpio_set_value(xclr_gpio, 0);
		pr_info("imx219_raw: v6.59: XCLR held LOW (xclr_late), released before stream\n");
		return;
	}
	gpio_set_value(xclr_gpio, 1);
	msleep(7);	/* mainline: wait t4+t5 after XCLR high */
	pr_info("imx219_raw: v6.57: XCLR (gpio %d) driven HIGH\n", xclr_gpio);
}

static void vendor_capture_dump(void)
{
	pr_info("imx219_raw: v6.25: ---- vendor config capture ----\n");
	step_rd("ccu+0x000", g_ccu + 0x000);
	step_rd("ccu+0x004", g_ccu + 0x004);
	step_rd("ccu+0x008", g_ccu + 0x008);
	step_rd("ccu+0x00c", g_ccu + 0x00c);
	step_rd("csi_top+0x000", g_ccu + OFF_CSI_TOP + 0x000);
	step_rd("csi_top+0x0f0", g_ccu + OFF_CSI_TOP + 0x0f0);
	if (g_cmb_top) {
		step_rd("cmb_top+0x000", g_cmb_top + 0x000);
		step_rd("cmb_top+0x004", g_cmb_top + 0x004);
	}
	if (g_phy_base)
		step_rd_window("cap-phy", g_phy_base, 0x000, 0x0fc);
	if (g_port_base)
		step_rd_window("cap-port", g_port_base, 0x000, 0x1fc);
	step_rd_window("cap-prs", g_ccu + OFF_CSI0, 0x000, 0x010);
	pr_info("imx219_raw: v6.25: ---- capture done ----\n");
}

static void pio_pk_probe(void)
{
	struct device_node *pio;
	struct resource res;
	void __iomem *base;
	u32 a[0x200], b[0x200];
	int i, w = -1, ones0 = 0, ones2 = 0, ones4 = 0;

	if (!g_pc_mipi0 && !g_pc_mipi1)
		return;
	pio = of_find_node_by_name(NULL, "pinctrl");
	if (!pio || of_address_to_resource(pio, 0, &res) != 0) {
		res.start = 0x02000000;
		res.end = res.start + 0x400;
	}
	base = ioremap(res.start, 0x800);
	if (!base)
		return;

	if (cam2) {
		/* v6.57: DATA register is bank-global; use the known cfg0
		 * offset (0x584 -> w = 0x161) without a mux diff */
		w = 0x584 / 4;
	} else {
	for (i = 0; i < 0x200; i++)
		a[i] = readl(base + i * 4);
	vind_pinctrl_select_one(g_pc_mipi0, "mipi0-sleep");
	msleep(10);
	for (i = 0; i < 0x200; i++)
		b[i] = readl(base + i * 4);

	for (i = 0; i < 0x1f0; i++) {
		if (a[i] != b[i]) {
			pr_info("imx219_raw: v6.30: pkdiff +0x%03x: 0x%08x -> 0x%08x\n",
				i * 4, a[i], b[i]);
			if (w < 0)
				w = i;
		}
	}
	if (w < 0) {
		pr_warn("imx219_raw: v6.30: no mux diff found\n");
		vind_pinctrl_select_one(g_pc_mipi0, "mipi0-default");
		iounmap(base);
		return;
	}
	}

	if (phyoff && g_cmb_top) {
		writel(0x00000000, g_cmb_top + 0x000); /* PHY power down */
		msleep(5);
		pr_info("imx219_raw: v6.38: PHY powered down for the pad probe\n");
	}

	/* v6.37: sample ALL PK0..PK9 in the ON phase - reveals where the
	 * camera signals actually land (cable pinout may differ from the
	 * schematic: RPi5 22-pin order vs OPi CAM1 order) */
	{
		int ones[10];
		u32 mask = 0;

		vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
					cam2 ? "mipi1-sleep" : "mipi0-sleep");
		msleep(10);
		for (i = 0; i < 10; i++)
			ones[i] = 0;
		for (i = 0; i < 20000; i++) {
			u32 d = readl(base + w * 4 + 0x10);
			int j;

			for (j = 0; j < 10; j++)
				ones[j] += (d >> (pk_shift + j)) & 1;
		}
		vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
					cam2 ? "mipi1-default" : "mipi0-default");
		for (i = 0; i < 10; i++) {
			if (ones[i] > 3000 && ones[i] < 17000)
				mask |= 1u << i;
		}
		pr_info("imx219_raw: v6.37: PK ones/20000: %d %d %d %d %d %d %d %d %d %d\n",
			ones[0], ones[1], ones[2], ones[3], ones[4],
			ones[5], ones[6], ones[7], ones[8], ones[9]);
		pr_info("imx219_raw: v6.37: toggling mask (PK9..PK0) = 0x%03x (1=toggles)\n",
			mask);
	}

	/* v6.34: A/B sampling - streaming vs standby (crosstalk test) */
	{
		int o0, o2, o4;

		vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
					cam2 ? "mipi1-sleep" : "mipi0-sleep");
		msleep(10);
		o0 = o2 = o4 = 0;
		for (i = 0; i < 20000; i++) {
			u32 d = readl(base + w * 4 + 0x10);

			o0 += (d >> (pk_shift + 0)) & 1;
			o2 += (d >> (pk_shift + 2)) & 1;
			o4 += (d >> (pk_shift + 4)) & 1;
		}
		vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
					cam2 ? "mipi1-default" : "mipi0-default");
		pr_info("imx219_raw: v6.34: PK ON  ones/20000: D0N=%d D1N=%d CKN=%d\n",
			o0, o2, o4);

		imx219_stream_off();
		msleep(150);
		vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
					cam2 ? "mipi1-sleep" : "mipi0-sleep");
		msleep(10);
		o0 = o2 = o4 = 0;
		for (i = 0; i < 20000; i++) {
			u32 d = readl(base + w * 4 + 0x10);

			o0 += (d >> (pk_shift + 0)) & 1;
			o2 += (d >> (pk_shift + 2)) & 1;
			o4 += (d >> (pk_shift + 4)) & 1;
		}
		vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
					cam2 ? "mipi1-default" : "mipi0-default");
		pr_info("imx219_raw: v6.34: PK OFF ones/20000: D0N=%d D1N=%d CKN=%d\n",
			o0, o2, o4);

		sensor_stream_on();
		msleep(100);
	}
	if (phyoff && g_cmb_top) {
		writel(0x00000B0F, g_cmb_top + 0x000); /* PHY back on */
		pr_info("imx219_raw: v6.38: PHY powered back up\n");
	}
	iounmap(base);
}

/* ------------------------------------------------------------------ */
/* v6.45 Plan Z: bit-banged CSI-2 capture (PHY off, PIO sampling)      */
/* ------------------------------------------------------------------ */

#define PIO_K_DATA   0x02000594UL	/* bank K DATA (pkdiff-located) */
#define PIO_E_DATA   0x020005A0UL	/* bank E DATA (cfg0 0x590 + 0x10) */

static void save_capture(const char *path, void *buf, size_t len)
{
	struct file *f;
	loff_t pos = 0;
	ssize_t w;

	if (!path || !path[0])
		return;
	f = filp_open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (IS_ERR(f)) {
		pr_warn("imx219_raw: save: open %s failed (%ld) - flash mounted?\n",
			path, PTR_ERR(f));
		return;
	}
	w = kernel_write(f, buf, len, &pos);
	filp_close(f, NULL);
	pr_info("imx219_raw: save: wrote %zd bytes to %s\n", w, path);
}

static void save_waveform_bmp(const char *path, u32 *buf, long samples)
{
	const long W = 2048;
	long H = samples / W;
	long x, y;
	u8 *row;
	u8 hdr[54];
	u32 fsize;
	struct file *f;
	loff_t pos = 0;

	if (H > 4096)
		H = 4096;
	fsize = 54 + (u32)(W * H * 3);
	memset(hdr, 0, sizeof(hdr));
	hdr[0] = 'B'; hdr[1] = 'M';
	hdr[2] = fsize & 0xff; hdr[3] = (fsize >> 8) & 0xff;
	hdr[4] = (fsize >> 16) & 0xff; hdr[5] = (fsize >> 24) & 0xff;
	hdr[10] = 54;
	hdr[14] = 40;			/* DIB size */
	hdr[18] = W & 0xff; hdr[19] = (W >> 8) & 0xff;
	hdr[22] = (long)H & 0xff; hdr[23] = ((long)H >> 8) & 0xff;
	hdr[26] = 1;			/* planes */
	hdr[28] = 24;			/* bpp */
	hdr[34] = (W * H * 3) & 0xff;
	hdr[35] = ((W * H * 3) >> 8) & 0xff;
	hdr[38] = 0x74; hdr[39] = 0x0B;	/* ~2835 ppm */

	row = kmalloc(W * 3, GFP_KERNEL);
	if (!row)
		return;
	f = filp_open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (IS_ERR(f)) {
		pr_warn("imx219_raw: bmp: open %s failed (%ld)\n", path, PTR_ERR(f));
		kfree(row);
		return;
	}
	kernel_write(f, hdr, 54, &pos);
	for (y = H - 1; y >= 0; y--) {
		for (x = 0; x < W; x++) {
			u32 s = buf[y * W + x];

			row[x * 3 + 0] = (s & (0x20u << pk_shift)) ? 255 : 0;	/* B = CKP */
			row[x * 3 + 1] = (s & (0x02u << pk_shift)) ? 255 : 0;	/* G = D0P */
			row[x * 3 + 2] = (s & (0x01u << pk_shift)) ? 255 : 0;	/* R = D0N */
		}
		kernel_write(f, row, W * 3, &pos);
	}
	filp_close(f, NULL);
	kfree(row);
	pr_info("imx219_raw: bmp: wrote %s (%ld x %ld)\n", path, W, H);
}

static void bitbang_capture(void)
{
	void __iomem *pio;
	u32 *buf = g_dma_va;
	long cap_samples = (long)(g_dma_size / 4);
	ktime_t t0, t1;
	u64 ns;
	int i, trig = 0;

	if (!g_dma_va) {
		pr_warn("imx219_raw: v6.45: no dma buffer, capture skipped\n");
		return;
	}
	pio = ioremap(PIO_K_DATA, 4);
	if (!pio) {
		pr_warn("imx219_raw: v6.45: pio ioremap failed\n");
		return;
	}

	/* PHY off -> lines free (sensor drives them) */
	if (g_cmb_top)
		writel(0x00000000, g_cmb_top + 0x000);
	/* v6.57: CAM2 probes the mipi1 pin group */
	vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
				cam2 ? "mipi1-sleep" : "mipi0-sleep");
	msleep(10);

	if (notrig) {
		trig = 1;
	} else {
		/* v6.51: LP-11 reference = the DATA pair (bits 0,1) - the
		 * CKN leg (bit 4) sits low on this board so it cannot be
		 * used. Arm after sustained data-pair idle, fire on the
		 * first sample where the data pair leaves LP-11. */
		int idle = 0;

		for (i = 0; i < 2000000000; i++) {
			if ((readl(pio) & (0x3u << pk_shift)) == (0x3u << pk_shift)) {
				if (++idle >= trig_idle) {
					for (; i < 2000000000; i++) {
						if ((readl(pio) & (0x3u << pk_shift)) != (0x3u << pk_shift)) {
							trig = 1;
							break;
						}
					}
					break;
				}
			} else {
				idle = 0;
			}
		}
	}
	if (!trig)
		pr_warn("imx219_raw: v6.45: no trigger - blind capture anyway\n");

	/* capture burst: bare LDR/STR loop */
	{
		u32 tmp = 0;
		u32 *b = buf;
		long n = cap_samples;

		t0 = ktime_get();
		asm volatile(
			"1:	ldr %w[t], [%[p]]\n"
			"	str %w[t], [%[b]], #4\n"
			"	subs %[n], %[n], #1\n"
			"	bne 1b\n"
			: [b] "+r"(b), [n] "+r"(n), [t] "=&r"(tmp)
			: [p] "r"(pio)
			: "cc", "memory");
		t1 = ktime_get();
	}
	ns = ktime_to_ns(ktime_sub(t1, t0));

	pr_info("imx219_raw: v6.45: captured %ld samples in %llu ns = %lu Msps\n",
		cap_samples, ns, (unsigned long)div64_u64((u64)cap_samples * 1000ULL, ns ? ns : 1));
	pr_info("imx219_raw: v6.45: first samples: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	{
		/* v6.52: per-segment activity profile: where in the window
		 * do the bursts live (D0N transitions per 512k samples) */
		long seg = 512 * 1024;
		long nseg = cap_samples / seg;
		long s, j;

		pr_info("imx219_raw: v6.52: D0N transitions per 512k-sample segment (%ld segments):\n",
			nseg);
		for (s = 0; s < nseg; s++) {
			u32 t = 0;

			for (j = s * seg + 1; j < (s + 1) * seg; j++)
				if ((buf[j] ^ buf[j - 1]) & (1u << pk_shift))
					t++;
			pr_info("imx219_raw: v6.52:   seg %2ld: %u\n", s, t);
		}
		pr_info("imx219_raw: v6.45: full totals: D0N=%u sampled\n",
			0);
	}

	save_capture("/tmp/cap.bin", buf, (size_t)cap_samples * 4);
	save_waveform_bmp("/tmp/wave.bmp", buf, cap_samples);
	save_capture(save_path, buf, (size_t)cap_samples * 4);

out:
	vind_pinctrl_select_one(cam2 ? g_pc_mipi1 : g_pc_mipi0,
				cam2 ? "mipi1-default" : "mipi0-default");
	if (g_cmb_top)
		writel(0x00000B0F, g_cmb_top + 0x000);
	iounmap(pio);
	pr_info("imx219_raw: v6.45: capture done\n");
}

/* ------------------------------------------------------------------ */
/* probe dump — the step_rd list                                       */
/* ------------------------------------------------------------------ */

static void probe_dump(void)
{
	pr_info("imx219_raw: ---- probe dump (step_rd) ----\n");
	step_rd("ccu+0x000",        g_ccu + 0x000);
	step_rd("ccu+0x004",        g_ccu + 0x004);
	step_rd("ccu+0x008",        g_ccu + 0x008);
	step_rd("csi_top+0x000",    g_ccu + OFF_CSI_TOP + 0x000);
	step_rd("csi_top+0x008",    g_ccu + OFF_CSI_TOP + 0x008);
	if (g_cmb_top)
		step_rd("cmb_top+0x000", g_cmb_top + 0x000);
	else
		step_rd("vind+0x000",  g_ccu + OFF_VIND_BASE);
	if (g_phy_base) {
		step_rd("phy+0x000",   g_phy_base + 0x000);
		step_rd("phy+0x004",   g_phy_base + 0x004);
	} else {
		step_rd("phy0+0x000",  g_ccu + OFF_PHY0);
	}
	if (g_port_base)
		step_rd("port+0x000",  g_port_base + 0x000);
	if (g_port2_base)
		step_rd("port2+0x000", g_port2_base + 0x000);
	if (g_phy2_base)
		step_rd("phy2+0x0F0",  g_phy2_base + 0x0F0);
	step_rd("mipi0+0x000",     g_ccu + OFF_MIPI0);
	step_rd("csi0+0x000",      g_ccu + OFF_CSI0);
	step_rd("isp+0x000",       g_ccu + OFF_ISP);
	step_rd("tdm+0x000",       g_ccu + OFF_TDM);
	step_rd("scaler+0x400",    g_ccu + OFF_SCALER);
	pr_info("imx219_raw: ---- probe dump done ----\n");
}

/* ------------------------------------------------------------------ */
/* char device: frame buffer readout (phase A stub, zeros until DMA)   */
/* ------------------------------------------------------------------ */

static ssize_t imx219_raw_read(struct file *f, char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	size_t off = (size_t)*ppos;
	size_t n;

	if (off >= g_dma_size)
		return 0;
	n = min(count, g_dma_size - off);
	if (copy_to_user(ubuf, g_dma_va + off, n))
		return -EFAULT;
	*ppos += n;
	return n;
}

static const struct file_operations imx219_raw_fops = {
	.owner = THIS_MODULE,
	.read  = imx219_raw_read,
};

static struct miscdevice imx219_raw_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "imx219_raw",
	.fops  = &imx219_raw_fops,
};

/* ------------------------------------------------------------------ */
/* init / exit                                                         */
/* ------------------------------------------------------------------ */

static int __init imx219_raw_init(void)
{
	int ret;

	pr_info("imx219_raw: ===== v6.59 init =====\n");

	g_sysccu = ioremap(SYS_CCU_BASE, SYS_CCU_SIZE);
	g_ccu    = ioremap(VIN_CCU_BASE, VIN_CCU_SIZE);
	g_iommu  = ioremap(IOMMU_BASE, IOMMU_SIZE);
	if (!g_sysccu || !g_ccu || !g_iommu) {
		pr_err("imx219_raw: ioremap failed (sysccu=%p ccu=%p iommu=%p)\n",
		       g_sysccu, g_ccu, g_iommu);
		ret = -ENOMEM;
		goto fail_map;
	}
	pr_info("imx219_raw: sysccu @ %p, vin_ccu @ %p, iommu @ %p\n",
		g_sysccu, g_ccu, g_iommu);

	ret = misc_register(&imx219_raw_misc);
	if (ret) {
		pr_err("imx219_raw: misc_register failed %d\n", ret);
		goto fail_misc;
	}

	dma_coerce_mask_and_coherent(imx219_raw_misc.this_device,
				     DMA_BIT_MASK(32));
	g_dma_size = (size_t)cap_mb * 1024 * 1024;
	g_dma_va = dma_alloc_coherent(imx219_raw_misc.this_device,
				      g_dma_size, &g_dma_pa, GFP_KERNEL);
	pr_info("imx219_raw: dma va=%p pa=%pad size=%zu\n",
		g_dma_va, &g_dma_pa, g_dma_size);

	/* locate vind0 (it owns clocks, pinctrl states, regulators) */
	g_vind_np = of_find_node_by_name(NULL, "vind");
	if (!g_vind_np) {
		pr_warn("imx219_raw: vind node NOT found in DT\n");
	} else {
		pr_info("imx219_raw: vind node: %s\n",
			of_node_full_name(g_vind_np));
		g_pdev = of_find_device_by_node(g_vind_np);
		if (g_pdev)
			pr_info("imx219_raw: vind device: %s\n",
				dev_name(&g_pdev->dev));
		else
			pr_warn("imx219_raw: vind platform device NOT found\n");
	}

	dt_find_bases();

	if (g_pdev) {
		vind_clocks_on(g_vind_np);
		vind_pinctrl_select(&g_pdev->dev);
		/* v6.11: hold the handle - never put until exit */
		if (cam2) {
			xclr_gpio = 137;	/* PE9 */
			pk_shift = 10;
		}
		g_pc_vind = pinctrl_get(&g_pdev->dev);
		if (!IS_ERR(g_pc_vind)) {
			if (cam2)
				vind_pinctrl_select_one(g_pc_vind, "mclk2-default");
			/* v6.56: mclk1 ALWAYS selected - OV5647 needs XCLK even
			 * for chip-id detection (sensor_detect runs later);
			 * harmless for IMX219 (verified: module self-clocked,
			 * ignores the pin) */
			vind_pinctrl_select_one(g_pc_vind, "mclk1-default");
			if (!nomclk)
				vind_pinctrl_select_one(g_pc_vind, "mclk2-default");
		} else {
			g_pc_vind = NULL;
		}
		vind_regulator_on(imx219_raw_misc.this_device,
				  "vind_mipipin");
		vind_regulator_on(imx219_raw_misc.this_device,
				  "vind_mclkpin");
	}
	if (g_mipipdev) {
		g_pc_mipi0 = pinctrl_get(&g_mipipdev->dev);
		if (!IS_ERR(g_pc_mipi0))
			vind_pinctrl_select_one(g_pc_mipi0, "mipi0-default");
		else {
			pr_warn("imx219_raw: v6.3: mipi pinctrl_get failed (%ld)\n",
				PTR_ERR(g_pc_mipi0));
			g_pc_mipi0 = NULL;
		}
	} else {
		pr_warn("imx219_raw: v6.3: mipi device missing, pinctrl skipped\n");
	}
	if (g_mipi1pdev && !single_phy) {
		g_pc_mipi1 = pinctrl_get(&g_mipi1pdev->dev);
		if (!IS_ERR(g_pc_mipi1))
			vind_pinctrl_select_one(g_pc_mipi1, "mipi1-default");
		else
			g_pc_mipi1 = NULL;
	}
	if (g_mipi2pdev && !single_phy) {
		g_pc_mipi2 = pinctrl_get(&g_mipi2pdev->dev);
		if (!IS_ERR(g_pc_mipi2))
			vind_pinctrl_select_one(g_pc_mipi2, "mipi2-default");
		else
			g_pc_mipi2 = NULL;
	}

	auto_power_on();
	xclr_release();

	if (g_pdev) {
		int retpm = pm_runtime_get_sync(&g_pdev->dev);

		if (retpm < 0)
			pm_runtime_put_noidle(&g_pdev->dev);
		pr_info("imx219_raw: v6.1: pm_runtime_get_sync(vind) = %d active=%d\n",
			retpm, pm_runtime_active(&g_pdev->dev) ? 1 : 0);
	} else {
		pr_warn("imx219_raw: v6.1: vind device missing, rpm skipped\n");
	}

	xclr_pio_poke();
	xclr_api_drive();
	{
		struct i2c_adapter *adap = i2c_get_adapter(i2c_bus);

		if (adap) {
			sensor_detect(adap);
			i2c_put_adapter(adap);
		}
	}
	imx219_check_id();
	csi_bus_reset_deassert();

	/* v6.3: vendor wake chain FIRST - everything else may wake here */
	if (!no_cfg)
		combo_bringup();
	else
		pr_info("imx219_raw: v6.12: no_cfg=1, skipping combo bring-up\n");

	pr_info("imx219_raw: ---- probe dump #1 (after combo bring-up) ----\n");
	probe_dump();

	/* v6.4: sensor into 4-lane RAW10 stream, then watch statuses */
	if (!no_cfg)
		sensor_stream_on();
	else
		pr_info("imx219_raw: v6.12: no_cfg=1, sensor not streamed\n");
	msleep(200);
	pr_info("imx219_raw: v6.4: ---- post-stream status ----\n");
	if (g_cmb_top)
		step_rd("cmb_top+0x004 (trescal)", g_cmb_top + 0x004);
	if (g_phy_base) {
		step_rd("phy+0x000 (rb)", g_phy_base + 0x000);
		step_rd("phy+0x0F0 (dbg)", g_phy_base + 0x0F0);
	}
	if (g_port_base) {
		step_rd("port+0x000 (rb)", g_port_base + 0x000);
		step_rd("port+0x108 (di)", g_port_base + 0x108);
		step_rd("port+0x110 (trig)", g_port_base + 0x110);
	}
	step_rd("csi0+0x000 (stick?)", g_ccu + OFF_CSI0);
	step_rd("isp+0x000 (stick?)", g_ccu + OFF_ISP);
	step_rd("tdm+0x000 (stick?)", g_ccu + OFF_TDM);

	/* v6.5: parser (CAM2 uses parser1 @ 0x05821000) */
	if (!no_cfg)
		parser_bringup();
	else
		pr_info("imx219_raw: v6.12: no_cfg=1, parser skipped\n");

	pio_pk_probe();

	if (slow) {
		pr_info("imx219_raw: v6.45: Plan Z bit-bang capture starting...\n");
		msleep(300);
		bitbang_capture();
	}

	step_rd_window("ccu", g_ccu, 0x000, 0x0fc);
	step_rd_window("csi_top", g_ccu + OFF_CSI_TOP, 0x000, 0x03c);

	tdm_bsp_writes();

	/* sysccu windows: 0x1700-0x17fc never scanned before (NEW),
	 * 0x1800-0x18fc baseline re-check */
	step_rd_window("sysccu", g_sysccu, 0x1700, 0x17fc);
	step_rd_window("sysccu", g_sysccu, 0x1800, 0x18fc);

	/* v6.2: IOMMU block last (its PM-underflow WARN traces used to
	 * evict the whole init log from the ring buffer) */
	iommu_bypass_and_ready();

	pr_info("imx219_raw: ---- probe dump #2 (after IOMMU) ----\n");
	probe_dump();

	mclk_presence_check();
	pio_pinmux_check();
	pio_scan();

	if (wait_vendor > 0) {
		pr_info("imx219_raw: v6.25: waiting %d s for vendor stream...\n",
			wait_vendor);
		msleep(wait_vendor * 1000);
		vendor_capture_dump();
	}

	pr_info("imx219_raw: ===== v6.59 init done, /dev/imx219_raw ready =====\n");
	return 0;

fail_misc:
	misc_deregister(&imx219_raw_misc);
fail_map:
	if (g_ccu)
		iounmap(g_ccu);
	if (g_sysccu)
		iounmap(g_sysccu);
	return ret;
}

static void __exit imx219_raw_exit(void)
{
	pr_info("imx219_raw: ===== exit =====\n");

	/* v6.54: release ALL runtime-PM holds FIRST (each get_sync from
	 * init gets its put_sync here), only then drop device references.
	 * Resets stay deasserted on purpose. */
	if (g_pdev) {
		/* v6.55: exactly one put_sync per the single get_sync in init */
		pm_runtime_put_sync(&g_pdev->dev);
	}
	if (g_pddev) {
		pm_runtime_put_sync(g_pddev);
		pm_runtime_allow(g_pddev);
		put_device(g_pddev);
		g_pddev = NULL;
	}
	if (g_nsidev) {
		device_for_each_child(g_nsidev, NULL, nsi_allow_one);
		put_device(g_nsidev);
	}

	if (g_dma_va)
		dma_free_coherent(imx219_raw_misc.this_device, g_dma_size,
				  g_dma_va, g_dma_pa);
	misc_deregister(&imx219_raw_misc);
	vind_clocks_off();
	if (g_vind_np)
		of_node_put(g_vind_np);
	if (g_pdev)
		put_device(&g_pdev->dev);
	gpio_free(xclr_gpio);
	if (g_pc_mipi2)
		pinctrl_put(g_pc_mipi2);
	if (g_pc_mipi1)
		pinctrl_put(g_pc_mipi1);
	if (g_pc_mipi0)
		pinctrl_put(g_pc_mipi0);
	if (g_pc_vind)
		pinctrl_put(g_pc_vind);
	if (g_port2_base)
		iounmap(g_port2_base);
	if (g_phy2_base)
		iounmap(g_phy2_base);
	if (g_mipi2pdev)
		put_device(&g_mipi2pdev->dev);
	if (g_mipi2_np)
		of_node_put(g_mipi2_np);
	if (g_mipi1pdev)
		put_device(&g_mipi1pdev->dev);
	if (g_port1_base)
		iounmap(g_port1_base);
	if (g_phy1_base)
		iounmap(g_phy1_base);
	if (g_mipipdev)
		put_device(&g_mipipdev->dev);
	if (g_port_base)
		iounmap(g_port_base);
	if (g_phy_base)
		iounmap(g_phy_base);
	if (g_cmb_top)
		iounmap(g_cmb_top);
	if (g_mipi_np)
		of_node_put(g_mipi_np);
	iounmap(g_iommu);
	iounmap(g_ccu);
	iounmap(g_sysccu);
	pr_info("imx219_raw: exit done\n");
}

module_init(imx219_raw_init);
module_exit(imx219_raw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("imx219_raw project");
MODULE_DESCRIPTION("RAW10 bring-up for IMX219/OV5647 on A733 (sun60iw2), v6.59");

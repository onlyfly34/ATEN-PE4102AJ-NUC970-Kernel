# ATEN PE4102AJ — NUC970 kernel ports

From-source Linux kernels for the **ATEN PE4102AJ / PE4102G eco PDU** (Nuvoton **NUC977**,
ARM926EJ-S / ARMv5TEJ). Each supported kernel version lives on its own branch; the default branch
(`pe4102aj-6.18-mainline`) carries the latest 6.18 port **and** this index.

## Branches
| branch | base upstream (remote) | status |
|--------|------------------------|--------|
| `pe4102aj-3.10` | `OpenNuvoton/NUC970_Linux_Kernel` (Linux 3.10.108, **EOL**) | ✅ validated on hardware; reference / archival |
| `pe4102aj-5.10` | `OpenNuvoton/NUC970-linux-5.10.y` (Linux 5.10 LTS) | ✅ validated on hardware — Device-Tree port, XZ-compressed, Buildroot rootfs |
| `pe4102aj-6.18-mainline` | kernel.org `linux-6.18.y` stable (mainline LTS) | ✅ validated on hardware — DT-only NUC970 port with full upstream lineage; CVE-tracking target |

Upstreams are git **remotes** (`origin` = 3.10, `up510` = 5.10, `stable`/kernel.org = 6.18), not branches.
Each `pe4102aj-*` branch = its upstream base + a small, rebasable PE4102AJ patch series on top. The
5.10 and 6.18 ports move the board description to **Device Tree** (no more board files).

## The PE4102AJ patch series (what each branch carries)
1. **External hardware watchdog (WDI / GPIO44 = PB12)** kept alive from early boot — without it the
   board hard-resets ~3 s in.
2. **ATEN 14-partition SPI-NOR layout** (MX25L25635, 32 MB) matching the vendor `/proc/mtd`.
3. **`pe4102aj_defconfig`** — boot the on-flash cramfs rootfs (not initramfs); key points:
   `GPIO_SYSFS`, **UART6 on PG (not PB — PB2 is the buzzer)**, `POSIX_MQUEUE`, MTD/CRAMFS/JFFS2/SPI-NOR,
   NUC970 DMA/RTC/crypto, IPv6, netfilter. On 5.10/6.18 also: **XZ** kernel compression (smaller and
   ~2.3× faster to decompress than LZMA on this ARM926), `BLK_DEV_WRITE_MOUNTED`, and a 12 MHz TIMER1
   so the CRNG seeds quickly instead of stalling ~250 s.

## NUC970 drivers — 3.10 / 5.10 / 6.18
What each port enables for the same hardware. `3.10` is the vendor reference (board files, EOL) and is
shown for comparison; `5.10` and `6.18` are the maintained Device-Tree ports. Symbols the vendor Kconfig
turned on with a hidden `default y` are now listed explicitly in each `pe4102aj_defconfig`.

| area | 3.10 (reference) | 5.10 | 6.18 |
|------|------------------|------|------|
| board model | board file + `cmdline` parts | Device Tree (appended DTB) | Device Tree (appended DTB) |
| kernel compression | gzip | **XZ** | **XZ** |
| external WDI (decompress) | built-in (no symbol) | `NUC970_DECOMP_WDI` | `NUC970_DECOMP_WDI` |
| SPI-NOR | `MTD_M25P80` | `MTD_SPI_NOR` | `MTD_SPI_NOR` |
| SPI controller | `SPI_NUC970_P0` | `SPI_NUC970_P0/P1` | `SPI_NUC970` |
| cramfs root | `CRAMFS` | `CRAMFS` | `CRAMFS` + `CRAMFS_MTD` |
| Ethernet MAC | `NUC970_ETH0` | `NUC970_ETH0/1` | `NUC970_EMAC` |
| Ethernet PHY | `ICPLUS_PHY` | `ICPLUS_PHY` | `ICPLUS_PHY` |
| serial console | `SERIAL_NUC970` (UART6/PG) | `SERIAL_NUC970` (DT) | `SERIAL_NUC970` (DT) |
| pin mux | `PINCTRL_NUC970` | `PINCTRL_NUC970` (+`PINCTRL_SINGLE`) | `PINCTRL_SINGLE` |
| GPIO controller | `GPIO_NUC970` | `GPIO_NUC970` | `GPIO_NUC970` |
| sysfs GPIO | `GPIO_SYSFS` | `GPIO_SYSFS` | `GPIO_SYSFS` |
| RTC | `RTC_DRV_NUC970` | `RTC_DRV_NUC970` | `RTC_DRV_NUC970` |
| etimer | `NUC970_ETIMER` | `NUC970_ETIMER` (+WKUP) | `NUC970_ETIMER` |
| internal SoC watchdog | — (not enabled) | `NUC970_WDT` (+WWDT/WKUP) | `NUC970_WATCHDOG` |
| DMA engine | `NUC970_DMA` | — (not built) | `NUC970_GDMA` |
| DMA proxy | `NUC970_DMA_PROXY` | — (not built) | `NUC970_DMA_PROXY` |
| HW crypto / OTP | `CRYPTO_DEV_NUC970` | `CRYPTO_DEV_NUC970` | `CRYPTO_DEV_NUC970` + `_MTP` |
| USB host | on (EHCI/OHCI/storage) | **off** | **off** |

Notes:
- **GPIO** — all three use `GPIO_NUC970` for the gpiochip (lines, direction, IRQ); only the *pin-mux*
  implementation differs. 6.18 uses the generic mainline `pinctrl-single`, so no GPIO functionality is lost.
- **DMA** — 5.10 does not build the NUC970 dmaengine driver or the userspace DMA proxy that 3.10 and 6.18
  carry. The full Buildroot userspace ran green on 5.10 without them; parity is still under review.
- **internal watchdog** — the vendor 3.10 defconfig enabled no `/dev/watchdog` driver at all (only the
  external WDI software timer); 5.10 and 6.18 add the SoC watchdog as an independent hardware layer.

> **No hardware RNG.** `HW_RANDOM` is deliberately *off* — the NUC970 has no TRNG. The CRNG seeds from
> jitter entropy, which is why the 5.10/6.18 ports clock TIMER1 at 12 MHz (fine-grained `get_entropy`)
> so `getrandom()` does not stall for minutes at boot.

## SDK Improvements
Changes this fork makes over the stock OpenNuvoton / ATEN BSP. The 3.10 branch is kept as the untouched
vendor reference; improvements land on the 5.10 and 6.18 ports.

**Boot time & robustness**
- **XZ kernel compression** (5.10, 6.18) — replaces the vendor LZMA; XZ is both smaller and ~2.3× faster
  to self-decompress on this ARM926 (measured on hardware).
- **Fast CRNG seeding** (6.18) — the post-5.18 `random` rework blocks `getrandom()` until 256 bits of
  entropy; with no cycle counter and TIMER1 at 120 kHz, jitter entropy was too coarse and vendor init
  stalled ~252 s. Clocking TIMER1 at 12 MHz (83 ns) cuts CRNG init to ~10 s.
- **Self-reboot on panic** (`PANIC_TIMEOUT=5`, 5.10 & 6.18) — the vendor left it 0 (hang forever on panic).
- **`BLK_DEV_WRITE_MOUNTED=y`** (6.18) — fixes RAM-boot of the cramfs ramdisk (a deferred `fput` kept the
  write side open, so `mount_root` hit `EBUSY`).

**Correctness fixes vs the vendor defconfig**
- **UART6 on PG, not PB** — the vendor option muxed UART6 onto PB, but PB2 is the buzzer, so the buzzer
  was stuck on in NORMAL mode. Moving UART6 to PG fixes it.
- **`GPIO_SYSFS` enabled** — the vendor had it off, which broke the userspace GPIO (`/tmp/property`)
  detection the init scripts rely on.
- **SPI-NOR 64 KB erase** (`# MTD_SPI_NOR_USE_4K_SECTORS is not set`) — matches the MX25L25635 erase
  granularity; the one real gap found during 5.10 flash validation.

**Watchdog**
- **Internal SoC watchdog enabled** (5.10, 6.18) — the vendor 3.10 enabled no `/dev/watchdog` driver at
  all; the ports add `NUC970_WDT` / `NUC970_WATCHDOG` as an independent hardware layer, armed by the
  userspace guard, and document the kernel-panic blind spot of the external-WDI-only design.

**Maintainability & mainline hygiene**
- **Device Tree port** (5.10, 6.18) — the board moves from board files to DT; 6.18 uses the generic
  `pinctrl-single` for pin-mux (the gpiochip stays `GPIO_NUC970`).
- **Full kernel.org lineage** (6.18) — the branch carries complete upstream history, so it rebases onto
  each `linux-6.18.y` stable release and is the CVE-tracking target.
- **No hidden `default y`** (5.10, 6.18) — NUC970 drivers the BSP force-enabled with `default y` (e.g.
  `GPIO_NUC970`, `CRYPTO_DEV_NUC970`, `NUC970_EBI_CS0`) now have that default dropped and are enabled
  explicitly in `pe4102aj_defconfig`, so the defconfig is an honest, auditable list (the resulting
  `.config` is byte-identical).
- **USB disabled** (5.10, 6.18) — the board has no external USB port, so dropping the host controllers,
  storage and PL2303 serial cuts attack surface and image size.

## Build
**3.10 branch** (needs the kernel.org gcc-4.9.4 crosstool — gcc 15 fails on 3.10's ARM `__asmeq`):
```
make ARCH=arm pe4102aj_defconfig
make ARCH=arm CROSS_COMPILE=<arm-linux-gnueabi-gcc-4.9.4-> LOADADDR=0x8000 uImage
```
**5.10 / 6.18 branches** (build with a modern cross-gcc; Device-Tree based):
```
make ARCH=arm pe4102aj_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOADADDR=0x8000 uImage dtbs
```

## Maintenance / CVE policy
- `pe4102aj-6.18-mainline` is the intended CVE-tracking target: based on kernel.org stable, pull each
  stable release with `git fetch stable && git rebase v6.18.<n>` (which bundles CVE fixes), re-applying
  the PE4102AJ series on top. (`git rerere` recommended for repeated rebase conflicts.)
- 3.10.108 is EOL (no upstream fixes) → individual CVE backports only; kept as a working reference.

## License
GPLv2 (Linux core is GPL-2.0-only; Nuvoton NUC970 files GPL-2.0-or-later). Forked from the public
OpenNuvoton BSP.

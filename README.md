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
   ~2.3× faster to decompress than LZMA on this ARM926), `COMPAT_32BIT_TIME` (32-bit time_t rootfs ABI),
   `BLK_DEV_WRITE_MOUNTED`, and a 12 MHz TIMER1 so the CRNG seeds quickly instead of stalling ~250 s.

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

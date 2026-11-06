# ATEN PE4102AJ — NUC970 kernel ports

From-source Linux kernels for the **ATEN PE4102AJ / PE4102G eco PDU** (Nuvoton **NUC977**,
ARM926EJ-S / ARMv5TEJ). Each supported kernel version lives on its own branch; this `main` branch is
just the index.

## Branches
| branch | base upstream (remote) | status |
|--------|------------------------|--------|
| `pe4102aj-3.10` | `OpenNuvoton/NUC970_Linux_Kernel` (Linux 3.10.108, **EOL**) | ✅ validated on hardware; reference / archival |
| `pe4102aj-5.10` | `OpenNuvoton/NUC970-linux-5.10.y` (Linux 5.10 LTS) | 🚧 planned — port the PE4102AJ patches to Device Tree |
| `pe4102aj-6.x`  | kernel.org `linux-6.x.y` stable (mainline LTS) | 💡 future — requires forward-porting NUC970 platform support; target for direct upstream CVE tracking |

Upstreams are git **remotes** (`origin` = 3.10, `up510` = 5.10, `stable` = kernel.org), not branches.
Each `pe4102aj-*` branch = its upstream base + a small, rebasable PE4102AJ patch series on top.

## The PE4102AJ patch series (what each branch carries)
1. **External hardware watchdog (WDI / GPIO44 = PB12)** kept alive from early boot — without it the
   board hard-resets ~3 s in.
2. **ATEN 14-partition SPI-NOR layout** (MX25L25635, 32 MB) matching the vendor `/proc/mtd`.
3. **`pe4102aj_defconfig`** — boot the on-flash cramfs rootfs (not initramfs); key points:
   `GPIO_SYSFS`, **UART6 on PG (not PB — PB2 is the buzzer)**, `POSIX_MQUEUE`, MTD/CRAMFS/JFFS2/SPI-NOR,
   NUC970 DMA/RTC/crypto, IPv6, netfilter.

## Build (3.10 branch)
```
make ARCH=arm pe4102aj_defconfig
make ARCH=arm CROSS_COMPILE=<arm-linux-gnueabi-gcc-4.9.4-> LOADADDR=0x8000 uImage
```
(gcc 15 fails on 3.10's ARM `__asmeq`; use the kernel.org gcc-4.9.4 crosstool.)

## Maintenance / CVE policy
- `pe4102aj-6.x` is the intended CVE-tracking target: base on kernel.org stable and
  `git fetch stable && git rebase v6.x.<n>` to pull each stable release (which bundles CVE fixes),
  re-applying the PE4102AJ series on top. (`git rerere` recommended for repeated rebase conflicts.)
- 3.10.108 is EOL (no upstream fixes) → individual CVE backports only; kept as a working reference.

## License
GPLv2 (Linux core is GPL-2.0-only; Nuvoton NUC970 files GPL-2.0-or-later). Forked from the public
OpenNuvoton BSP.

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

## Drivers enabled for the NUC970 port
The main board-specific symbols turned on in `pe4102aj_defconfig` (see that file for the full list):

| area | key `CONFIG_*` symbols | purpose |
|------|------------------------|---------|
| SoC platform / boot | `ARCH_NUC970`, `NUC970_DECOMP_WDI`, `ARM_APPENDED_DTB`, `ARM_ATAG_DTB_COMPAT` | NUC970 platform; feed the external WDI while the kernel self-decompresses (board resets ~3 s in otherwise); DTB is appended to the uImage (U-Boot passes no Device Tree) |
| Storage / rootfs (SPI-NOR) | `MTD`, `MTD_BLOCK`, `MTD_SPI_NOR`, `SPI_NUC970`, `CRAMFS`(+`CRAMFS_MTD`), `JFFS2_FS`, `BLK_DEV_RAM`, `BLK_DEV_INITRD` | MX25L25635 32 MB, 14-partition map; read-only cramfs root + JFFS2 `/nv`; ramdisk boot path used during bring-up |
| Networking | `NUC970_EMAC`, `ICPLUS_PHY`, `NETFILTER`, `IP_NF_IPTABLES`, `IP6_NF_IPTABLES` | on-chip MAC + IC+ PHY; vendor firewall |
| Serial / console | `SERIAL_NUC970`(+`_CONSOLE`), `USB_SERIAL_PL2303` | UART6/PG console; USB serial |
| GPIO / pinctrl | `PINCTRL_SINGLE`, `GPIO_SYSFS` | outlet relays, panel buttons, WDI and buzzer driven through sysfs GPIO |
| Timers / RTC | `NUC970_ETIMER`, `RTC_DRV_NUC970` | etimer (vendor software watchdog); RTC |
| Watchdog | `WATCHDOG`, `NUC970_WATCHDOG`, `WATCHDOG_NOWAYOUT` | internal SoC WDT at `/dev/watchdog`, nowayout (userspace guard opens it and must keep kicking) |
| DMA | `NUC970_GDMA`, `NUC970_DMA_PROXY` | GDMA plus the userspace DMA-proxy the vendor stack uses |
| Crypto / OTP | `CRYPTO_DEV_NUC970`, `CRYPTO_DEV_NUC970_MTP`, `CRYPTO_USER_API_*` | hardware AES/SHA; MTP / OTP key access for `mtp_tool`; AF_ALG user APIs |
| USB | `USB_EHCI_HCD`, `USB_OHCI_HCD`, `USB_STORAGE` | USB host (EHCI/OHCI) + mass storage |

> **No hardware RNG.** `HW_RANDOM` is deliberately *off* — the NUC970 has no TRNG. The CRNG seeds from
> jitter entropy, which is why the 5.10/6.18 ports clock TIMER1 at 12 MHz (fine-grained `get_entropy`)
> so `getrandom()` does not stall for minutes at boot.

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

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nuvoton NUC970/NUC977 clock controller
 *
 * Forward-ported from the clock tree in Nuvoton's NUC970 Linux BSP
 * (drivers/clk/nuvoton/clk-nuc970-{ccf,apll,upll}.c, Copyright (c) 2014
 * Nuvoton Technology Corporation) and turned into a DT clock provider.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/nuvoton,nuc970-clk.h>

/* register offsets from the CLK block base (0xB0000200) */
#define CLK_HCLKEN	0x10
#define CLK_PCLKEN0	0x18
#define CLK_PCLKEN1	0x1c
#define CLK_DIV(n)	(0x20 + 4 * (n))
#define CLK_APLLCON	0x60
#define CLK_UPLLCON	0x64

#define NO_ID		(-1)

static DEFINE_SPINLOCK(nuc970_clk_lock);

/*
 * APLL / UPLL (read-only; the boot loader programs them).
 *   Fout = Fin * (FB + 1) / (IN + 1) / (OUT + 1)
 *   FB = [6:0], IN = [12:7], OUT = [15:13]
 * This reproduces every entry of the BSP's lookup table
 * (0x15 -> 264 MHz, 0x18 -> 300, 0x1d -> 360, 0x8027 -> 96, 0x8028 -> 98.4,
 *  0x21f0 -> 169.5).
 */
struct nuc970_pll {
	struct clk_hw hw;
	void __iomem *reg;
};

static unsigned long nuc970_pll_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct nuc970_pll *pll = container_of(hw, struct nuc970_pll, hw);
	u32 val = readl(pll->reg);
	u64 rate = (u64)parent_rate * ((val & 0x7f) + 1);

	do_div(rate, ((val >> 7) & 0x3f) + 1);
	do_div(rate, ((val >> 13) & 0x7) + 1);

	return rate;
}

static const struct clk_ops nuc970_pll_ops = {
	.recalc_rate = nuc970_pll_recalc_rate,
};

static struct clk_hw * __init nuc970_register_pll(const char *name,
						  const char *parent,
						  void __iomem *reg)
{
	struct clk_init_data init = {};
	struct nuc970_pll *pll;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &nuc970_pll_ops;
	init.parent_names = &parent;
	init.num_parents = 1;
	pll->reg = reg;
	pll->hw.init = &init;

	ret = clk_hw_register(NULL, &pll->hw);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}
	return &pll->hw;
}

enum nuc970_clk_type { T_MUX, T_DIV, T_GATE, T_FF };

struct nuc970_clk_desc {
	enum nuc970_clk_type type;
	int id;
	const char *name;
	const char *parent;
	const char * const *parents;
	u8 num_parents;
	u16 reg;
	u8 shift;
	u8 width;
	u16 mult;
	u16 div;
	unsigned long flags;
};

#define MUX(_id, _name, _parents, _reg, _shift, _width)			\
	{ .type = T_MUX, .id = _id, .name = _name, .parents = _parents,	\
	  .num_parents = ARRAY_SIZE(_parents), .reg = _reg,		\
	  .shift = _shift, .width = _width }
#define DIV(_id, _name, _parent, _reg, _shift, _width)			\
	{ .type = T_DIV, .id = _id, .name = _name, .parent = _parent,	\
	  .reg = _reg, .shift = _shift, .width = _width }
#define GATE_F(_id, _name, _parent, _reg, _bit, _flags)			\
	{ .type = T_GATE, .id = _id, .name = _name, .parent = _parent,	\
	  .reg = _reg, .shift = _bit,					\
	  .flags = CLK_SET_RATE_PARENT | (_flags) }
#define GATE(_id, _name, _parent, _reg, _bit)				\
	GATE_F(_id, _name, _parent, _reg, _bit, 0)
#define FF(_id, _name, _parent, _mult, _div)				\
	{ .type = T_FF, .id = _id, .name = _name, .parent = _parent,	\
	  .mult = _mult, .div = _div, .flags = CLK_SET_RATE_PARENT }

/*
 * Engine clock with an APLL/UPLL pre-divider, a 4-way source mux
 * (xin, -, apll/div, upll/div), a divider and a gate.
 */
#define ECLK(_p, _id, _reg, _pdiv_shift, _mux_shift, _div_shift,	\
	     _div_width, _gate_reg, _gate_bit)				\
	DIV(NO_ID, _p "_aplldiv", "apll", _reg, _pdiv_shift, 3),	\
	DIV(NO_ID, _p "_uplldiv", "upll", _reg, _pdiv_shift, 3),	\
	{ .type = T_MUX, .id = NO_ID, .name = _p "_eclk_mux",		\
	  .parents = (const char * const []){ "xin", "dummy",		\
			_p "_aplldiv", _p "_uplldiv" },			\
	  .num_parents = 4, .reg = _reg, .shift = _mux_shift,		\
	  .width = 2 },							\
	DIV(NO_ID, _p "_eclk_div", _p "_eclk_mux", _reg, _div_shift,	\
	    _div_width),						\
	GATE(_id, _p "_eclk_gate", _p "_eclk_div", _gate_reg, _gate_bit)

static const char * const sys_sel[] = { "xin", "dummy", "apll", "upll" };
static const char * const xin_32k_sel[] = { "xin", "xin32k" };
static const char * const etimer_sel[] = { "xin", "pclk_div", "pclk4096_div",
					   "xin32k" };
static const char * const wdt_sel[] = { "xin", "xin128_div", "pclk4096_div",
					"xin32k" };

static const struct nuc970_clk_desc nuc970_clks[] __initconst = {
	FF(NUC970_CLK_XIN128_DIV, "xin128_div", "xin", 1, 128),

	/* system */
	MUX(NUC970_CLK_SYS_MUX, "sys_mux", sys_sel, CLK_DIV(0), 3, 2),
	DIV(NUC970_CLK_SYS_DIV, "sys_div", "sys_mux", CLK_DIV(0), 0, 2),
	GATE_F(NUC970_CLK_DDR, "ddr_gate", "sys_div", CLK_HCLKEN, 10,
	       CLK_IS_CRITICAL),
	DIV(NUC970_CLK_CPU_DIV, "cpu_div", "sys_div", CLK_DIV(0), 16, 1),
	GATE_F(NUC970_CLK_CPU, "cpu_gate", "cpu_div", CLK_HCLKEN, 0,
	       CLK_IS_CRITICAL),

	/* HCLK1 */
	FF(NUC970_CLK_HCLK1_DIV, "hclk1_div", "cpu_div", 1, 2),
	GATE(NUC970_CLK_GDMA, "gdma_hclk_gate", "hclk1_div", CLK_HCLKEN, 12),
	GATE(NUC970_CLK_EBI, "ebi_hclk_gate", "hclk1_div", CLK_HCLKEN, 9),
	GATE(NUC970_CLK_TIC, "tic_hclk_gate", "hclk1_div", CLK_HCLKEN, 7),

	/* HCLK / HCLK234 */
	FF(NUC970_CLK_HCLKN_DIV, "hclkn_div", "sys_div", 1, 2),
	GATE_F(NUC970_CLK_DRAM, "dram_gate", "hclkn_div", CLK_HCLKEN, 10,
	       CLK_IS_CRITICAL),
	GATE_F(NUC970_CLK_HCLK, "hclk_gate", "hclkn_div", CLK_HCLKEN, 1,
	       CLK_IS_CRITICAL),
	GATE_F(NUC970_CLK_SRAM, "sram_gate", "hclk_gate", CLK_HCLKEN, 8,
	       CLK_IS_CRITICAL),
	DIV(NUC970_CLK_HCLK234_DIV, "hclk234_div", "hclkn_div", CLK_DIV(0),
	    20, 4),

	/* HCLK3 */
	GATE(NUC970_CLK_USBH, "usbh_hclk_gate", "hclk234_div", CLK_HCLKEN, 18),
	GATE(NUC970_CLK_USBD, "usbd_hclk_gate", "hclk234_div", CLK_HCLKEN, 19),
	GATE(NUC970_CLK_FMI, "fmi_hclk_gate", "hclk234_div", CLK_HCLKEN, 20),
	GATE(NUC970_CLK_NAND, "nand_hclk_gate", "hclk234_div", CLK_HCLKEN, 21),
	GATE(NUC970_CLK_EMMC, "emmc_hclk_gate", "hclk234_div", CLK_HCLKEN, 22),
	GATE(NUC970_CLK_CRYPTO, "crypto_hclk_gate", "hclk234_div",
	     CLK_HCLKEN, 23),
	GATE(NUC970_CLK_EMAC1, "emac1_hclk_gate", "hclk234_div",
	     CLK_HCLKEN, 17),
	DIV(NO_ID, "emac1_eclk_div", "hclk234_div", CLK_DIV(8), 0, 8),
	GATE(NUC970_CLK_EMAC1_ECLK, "emac1_eclk_gate", "emac1_eclk_div",
	     CLK_HCLKEN, 17),
	GATE(NUC970_CLK_JPEG, "jpeg_hclk_gate", "hclk234_div", CLK_HCLKEN, 29),
	DIV(NO_ID, "jpeg_eclk_div", "hclk234_div", CLK_DIV(3), 28, 3),
	GATE(NUC970_CLK_JPEG_ECLK, "jpeg_eclk_gate", "jpeg_eclk_div",
	     CLK_HCLKEN, 29),
	GATE(NUC970_CLK_GE2D, "ge2d_hclk_gate", "hclk234_div", CLK_HCLKEN, 28),
	DIV(NO_ID, "ge2d_eclk_div", "hclk234_div", CLK_DIV(2), 28, 2),
	GATE(NUC970_CLK_GE2D_ECLK, "ge2d_eclk_gate", "ge2d_eclk_div",
	     CLK_HCLKEN, 28),

	/* HCLK4 */
	GATE(NUC970_CLK_SDH, "sdh_hclk_gate", "hclk234_div", CLK_HCLKEN, 30),
	GATE(NUC970_CLK_AUDIO, "audio_hclk_gate", "hclk234_div",
	     CLK_HCLKEN, 24),
	GATE(NUC970_CLK_LCD, "lcd_hclk_gate", "hclk234_div", CLK_HCLKEN, 25),
	GATE(NUC970_CLK_CAP, "cap_hclk_gate", "hclk234_div", CLK_HCLKEN, 26),
	GATE(NUC970_CLK_SENSOR, "sensor_hclk_gate", "hclk234_div",
	     CLK_HCLKEN, 27),
	GATE(NUC970_CLK_EMAC0, "emac0_hclk_gate", "hclk234_div",
	     CLK_HCLKEN, 16),
	DIV(NO_ID, "emac0_eclk_div", "hclk234_div", CLK_DIV(8), 0, 8),
	GATE(NUC970_CLK_EMAC0_ECLK, "emac0_eclk_gate", "emac0_eclk_div",
	     CLK_HCLKEN, 16),

	/* engine clocks */
	ECLK("usb", NUC970_CLK_USB_ECLK, CLK_DIV(2), 0, 3, 8, 4,
	     CLK_HCLKEN, 18),
	ECLK("sdh", NUC970_CLK_SDH_ECLK, CLK_DIV(9), 0, 3, 8, 8,
	     CLK_HCLKEN, 30),
	ECLK("emmc", NUC970_CLK_EMMC_ECLK, CLK_DIV(3), 0, 3, 8, 8,
	     CLK_HCLKEN, 22),
	ECLK("adc", NUC970_CLK_ADC_ECLK, CLK_DIV(7), 16, 19, 24, 8,
	     CLK_PCLKEN1, 24),
	ECLK("lcd", NUC970_CLK_LCD_ECLK, CLK_DIV(1), 0, 3, 8, 8,
	     CLK_HCLKEN, 25),
	ECLK("audio", NUC970_CLK_AUDIO_ECLK, CLK_DIV(1), 16, 19, 24, 8,
	     CLK_HCLKEN, 24),
	ECLK("cap", NUC970_CLK_CAP_ECLK, CLK_DIV(3), 16, 19, 24, 4,
	     CLK_HCLKEN, 26),
	ECLK("uart0", NUC970_CLK_UART0_ECLK, CLK_DIV(4), 0, 3, 5, 3,
	     CLK_PCLKEN0, 16),
	ECLK("uart1", NUC970_CLK_UART1_ECLK, CLK_DIV(4), 8, 11, 13, 3,
	     CLK_PCLKEN0, 17),
	ECLK("uart2", NUC970_CLK_UART2_ECLK, CLK_DIV(4), 16, 19, 21, 3,
	     CLK_PCLKEN0, 18),
	ECLK("uart3", NUC970_CLK_UART3_ECLK, CLK_DIV(4), 24, 27, 29, 3,
	     CLK_PCLKEN0, 19),
	ECLK("uart4", NUC970_CLK_UART4_ECLK, CLK_DIV(5), 0, 3, 5, 3,
	     CLK_PCLKEN0, 20),
	ECLK("uart5", NUC970_CLK_UART5_ECLK, CLK_DIV(5), 8, 11, 13, 3,
	     CLK_PCLKEN0, 21),
	ECLK("uart6", NUC970_CLK_UART6_ECLK, CLK_DIV(5), 16, 19, 21, 3,
	     CLK_PCLKEN0, 22),
	ECLK("uart7", NUC970_CLK_UART7_ECLK, CLK_DIV(5), 24, 27, 29, 3,
	     CLK_PCLKEN0, 23),
	ECLK("uart8", NUC970_CLK_UART8_ECLK, CLK_DIV(6), 0, 3, 5, 3,
	     CLK_PCLKEN0, 24),
	ECLK("uart9", NUC970_CLK_UART9_ECLK, CLK_DIV(6), 8, 11, 13, 3,
	     CLK_PCLKEN0, 25),
	ECLK("uart10", NUC970_CLK_UART10_ECLK, CLK_DIV(6), 16, 19, 21, 3,
	     CLK_PCLKEN0, 26),

	/* system engine clock (no gate in the BSP) */
	DIV(NO_ID, "system_aplldiv", "apll", CLK_DIV(0), 0, 3),
	DIV(NO_ID, "system_uplldiv", "upll", CLK_DIV(0), 0, 3),
	{ .type = T_MUX, .id = NO_ID, .name = "system_eclk_mux",
	  .parents = (const char * const []){ "xin", "dummy",
			"system_aplldiv", "system_uplldiv" },
	  .num_parents = 4, .reg = CLK_DIV(0), .shift = 3, .width = 2 },
	DIV(NUC970_CLK_SYSTEM_ECLK, "system_eclk_div", "system_eclk_mux",
	    CLK_DIV(0), 8, 4),

	/* GPIO / KPI debounce engine clocks */
	MUX(NO_ID, "gpio_eclk_mux", xin_32k_sel, CLK_DIV(7), 7, 1),
	DIV(NO_ID, "gpio_eclk_div", "gpio_eclk_mux", CLK_DIV(7), 0, 7),
	GATE(NUC970_CLK_GPIO_ECLK, "gpio_eclk_gate", "gpio_eclk_div",
	     CLK_PCLKEN0, 3),
	MUX(NO_ID, "kpi_eclk_mux", xin_32k_sel, CLK_DIV(7), 15, 1),
	DIV(NO_ID, "kpi_eclk_div", "kpi_eclk_mux", CLK_DIV(7), 8, 7),
	GATE(NUC970_CLK_KPI_ECLK, "kpi_eclk_gate", "kpi_eclk_div",
	     CLK_PCLKEN1, 25),

	/* ETIMER / WDT engine clocks */
	MUX(NO_ID, "etimer0_eclk_mux", etimer_sel, CLK_DIV(8), 16, 2),
	GATE(NUC970_CLK_ETIMER0_ECLK, "etimer0_eclk_gate", "etimer0_eclk_mux",
	     CLK_PCLKEN0, 4),
	MUX(NO_ID, "etimer1_eclk_mux", etimer_sel, CLK_DIV(8), 20, 2),
	GATE(NUC970_CLK_ETIMER1_ECLK, "etimer1_eclk_gate", "etimer1_eclk_mux",
	     CLK_PCLKEN0, 5),
	MUX(NO_ID, "etimer2_eclk_mux", etimer_sel, CLK_DIV(8), 24, 2),
	GATE(NUC970_CLK_ETIMER2_ECLK, "etimer2_eclk_gate", "etimer2_eclk_mux",
	     CLK_PCLKEN0, 6),
	MUX(NO_ID, "etimer3_eclk_mux", etimer_sel, CLK_DIV(8), 28, 2),
	GATE(NUC970_CLK_ETIMER3_ECLK, "etimer3_eclk_gate", "etimer3_eclk_mux",
	     CLK_PCLKEN0, 7),
	MUX(NO_ID, "wwdt_eclk_mux", wdt_sel, CLK_DIV(8), 10, 2),
	GATE(NUC970_CLK_WWDT_ECLK, "wwdt_eclk_gate", "wwdt_eclk_mux",
	     CLK_PCLKEN0, 1),
	MUX(NO_ID, "wdt_eclk_mux", wdt_sel, CLK_DIV(8), 8, 2),
	GATE(NUC970_CLK_WDT_ECLK, "wdt_eclk_gate", "wdt_eclk_mux",
	     CLK_PCLKEN0, 0),

	/* smartcard engine clocks */
	DIV(NO_ID, "smc0_eclk_div", "xin", CLK_DIV(6), 24, 4),
	GATE(NUC970_CLK_SMC0_ECLK, "smc0_eclk_gate", "smc0_eclk_div",
	     CLK_PCLKEN1, 12),
	DIV(NO_ID, "smc1_eclk_div", "xin", CLK_DIV(6), 28, 4),
	GATE(NUC970_CLK_SMC1_ECLK, "smc1_eclk_gate", "smc1_eclk_div",
	     CLK_PCLKEN1, 13),

	/* PCLK */
	DIV(NUC970_CLK_PCLK_DIV, "pclk_div", "hclk1_div", CLK_DIV(0), 24, 4),
	FF(NUC970_CLK_PCLK4096_DIV, "pclk4096_div", "pclk_div", 1, 4096),
	GATE(NUC970_CLK_I2C0, "i2c0_gate", "pclk_div", CLK_PCLKEN1, 0),
	GATE(NUC970_CLK_I2C1, "i2c1_gate", "pclk_div", CLK_PCLKEN1, 1),
	GATE(NUC970_CLK_SPI0, "spi0_gate", "pclk_div", CLK_PCLKEN1, 4),
	GATE(NUC970_CLK_SPI1, "spi1_gate", "pclk_div", CLK_PCLKEN1, 5),
	GATE(NUC970_CLK_UART0, "uart0_gate", "pclk_div", CLK_PCLKEN0, 16),
	GATE(NUC970_CLK_UART1, "uart1_gate", "pclk_div", CLK_PCLKEN0, 17),
	GATE(NUC970_CLK_UART2, "uart2_gate", "pclk_div", CLK_PCLKEN0, 18),
	GATE(NUC970_CLK_UART3, "uart3_gate", "pclk_div", CLK_PCLKEN0, 19),
	GATE(NUC970_CLK_UART4, "uart4_gate", "pclk_div", CLK_PCLKEN0, 20),
	GATE(NUC970_CLK_UART5, "uart5_gate", "pclk_div", CLK_PCLKEN0, 21),
	GATE(NUC970_CLK_UART6, "uart6_gate", "pclk_div", CLK_PCLKEN0, 22),
	GATE(NUC970_CLK_UART7, "uart7_gate", "pclk_div", CLK_PCLKEN0, 23),
	GATE(NUC970_CLK_UART8, "uart8_gate", "pclk_div", CLK_PCLKEN0, 24),
	GATE(NUC970_CLK_UART9, "uart9_gate", "pclk_div", CLK_PCLKEN0, 25),
	GATE(NUC970_CLK_UART10, "uart10_gate", "pclk_div", CLK_PCLKEN0, 26),
	GATE(NUC970_CLK_WDT, "wdt_gate", "pclk_div", CLK_PCLKEN0, 0),
	GATE(NUC970_CLK_WWDT, "wwdt_gate", "pclk_div", CLK_PCLKEN0, 1),
	GATE(NUC970_CLK_RTC, "rtc_gate", "pclk_div", CLK_PCLKEN0, 2),
	GATE(NUC970_CLK_GPIO, "gpio_gate", "pclk_div", CLK_PCLKEN0, 3),
	GATE(NUC970_CLK_ADC, "adc_gate", "pclk_div", CLK_PCLKEN1, 24),
	GATE(NUC970_CLK_KPI, "kpi_gate", "pclk_div", CLK_PCLKEN1, 25),
	GATE(NUC970_CLK_MTPC, "mtpc_gate", "pclk_div", CLK_PCLKEN1, 26),
	GATE(NUC970_CLK_PWM, "pwm_gate", "pclk_div", CLK_PCLKEN1, 27),
	GATE(NUC970_CLK_ETIMER0, "etimer0_gate", "pclk_div", CLK_PCLKEN0, 4),
	GATE(NUC970_CLK_ETIMER1, "etimer1_gate", "pclk_div", CLK_PCLKEN0, 5),
	GATE(NUC970_CLK_ETIMER2, "etimer2_gate", "pclk_div", CLK_PCLKEN0, 6),
	GATE(NUC970_CLK_ETIMER3, "etimer3_gate", "pclk_div", CLK_PCLKEN0, 7),
	GATE(NUC970_CLK_CAN0, "can0_gate", "pclk_div", CLK_PCLKEN1, 8),
	GATE(NUC970_CLK_CAN1, "can1_gate", "pclk_div", CLK_PCLKEN1, 9),
	GATE(NUC970_CLK_TIMER0, "timer0_gate", "xin", CLK_PCLKEN0, 8),
	GATE(NUC970_CLK_TIMER1, "timer1_gate", "xin", CLK_PCLKEN0, 9),
	GATE(NUC970_CLK_TIMER2, "timer2_gate", "xin", CLK_PCLKEN0, 10),
	GATE(NUC970_CLK_TIMER3, "timer3_gate", "xin", CLK_PCLKEN0, 11),
	GATE(NUC970_CLK_TIMER4, "timer4_gate", "xin", CLK_PCLKEN0, 12),
	GATE(NUC970_CLK_SMC0, "smc0_gate", "pclk_div", CLK_PCLKEN1, 12),
	GATE(NUC970_CLK_SMC1, "smc1_gate", "pclk_div", CLK_PCLKEN1, 13),
};

static void __init nuc970_clk_init(struct device_node *np)
{
	struct clk_hw_onecell_data *data;
	void __iomem *base;
	struct clk_hw *hw;
	unsigned int i;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%pOF: failed to map clock registers\n", np);
		return;
	}

	data = kzalloc(struct_size(data, hws, NUC970_CLK_NR), GFP_KERNEL);
	if (!data)
		goto err_unmap;
	data->num = NUC970_CLK_NR;
	for (i = 0; i < NUC970_CLK_NR; i++)
		data->hws[i] = ERR_PTR(-ENOENT);

	data->hws[NUC970_CLK_XIN] =
		clk_hw_register_fixed_rate(NULL, "xin", NULL, 0, 12000000);
	data->hws[NUC970_CLK_XIN32K] =
		clk_hw_register_fixed_rate(NULL, "xin32k", NULL, 0, 32768);
	data->hws[NUC970_CLK_APLL] =
		nuc970_register_pll("apll", "xin", base + CLK_APLLCON);
	data->hws[NUC970_CLK_UPLL] =
		nuc970_register_pll("upll", "xin", base + CLK_UPLLCON);

	for (i = 0; i < ARRAY_SIZE(nuc970_clks); i++) {
		const struct nuc970_clk_desc *d = &nuc970_clks[i];

		switch (d->type) {
		case T_MUX:
			hw = clk_hw_register_mux(NULL, d->name, d->parents,
						 d->num_parents, d->flags,
						 base + d->reg, d->shift,
						 d->width, 0,
						 &nuc970_clk_lock);
			break;
		case T_DIV:
			hw = clk_hw_register_divider(NULL, d->name, d->parent,
						     d->flags, base + d->reg,
						     d->shift, d->width, 0,
						     &nuc970_clk_lock);
			break;
		case T_GATE:
			hw = clk_hw_register_gate(NULL, d->name, d->parent,
						  d->flags, base + d->reg,
						  d->shift, 0,
						  &nuc970_clk_lock);
			break;
		case T_FF:
			hw = clk_hw_register_fixed_factor(NULL, d->name,
							  d->parent, d->flags,
							  d->mult, d->div);
			break;
		default:
			hw = ERR_PTR(-EINVAL);
		}

		if (IS_ERR(hw)) {
			pr_err("%pOF: failed to register %s: %ld\n", np,
			       d->name, PTR_ERR(hw));
			continue;
		}
		if (d->id != NO_ID)
			data->hws[d->id] = hw;
	}

	for (i = 0; i < NUC970_CLK_NR; i++)
		if (IS_ERR(data->hws[i]))
			pr_warn("%pOF: clock id %u not registered\n", np, i);

	if (of_clk_add_hw_provider(np, of_clk_hw_onecell_get, data))
		pr_err("%pOF: failed to add clock provider\n", np);
	return;

err_unmap:
	iounmap(base);
}
CLK_OF_DECLARE(nuc970_clk, "nuvoton,nuc970-clk", nuc970_clk_init);

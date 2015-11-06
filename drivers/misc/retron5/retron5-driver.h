#ifndef _RETRON5_DRIVER_H
#define _RETRON5_DRIVER_H

#include <linux/module.h>
#define KERNEL
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/rfkill-rk.h>
#include <mach/iomux.h>

#define DRIVER_VER				0x0001
#define DEVICE_NAME				"retron5"
#define DEVICE_FILE				"retron5"
#define TAG						"Retron5: "

#define D_PRINTF(...)			printk(TAG __VA_ARGS__)
#define BAIL(errno)				{ rv = errno; goto end; }
#define LIMIT(in, max)			(((in) > (max)) ? (max) : (in))
#define GET_PROCESS_PID			(task_tgid_nr(current))
#define bswap_16(x) 			(((x) & 0x00ff) << 8 | ((x) & 0xff00) >> 8)

#define GPIO_PORT_FPGA			0x00
#define	GPIO_PORT_JOY0			0x01
#define	GPIO_PORT_JOY1			0x02

#define GPIO_IN					0x00
#define GPIO_OUT				0x01
#define GPIO_HIGH				0x01
#define GPIO_LOW				0x00

#define IOCTL_IDENT 'G'
#define IOCTL_GPIO_SET_BITS 		_IOWR(IOCTL_IDENT, 0, int)
#define IOCTL_GPIO_GET_BITS 		_IOWR(IOCTL_IDENT, 1, int)
#define IOCTL_GPIO_SET_DIRECTION 	_IOWR(IOCTL_IDENT, 2, int)
#define IOCTL_GPIO_SET_PULL	 		_IOWR(IOCTL_IDENT, 3, int)
#define IOCTL_GPIO_ACCESS_CTRL		_IOWR(IOCTL_IDENT, 4, int)
#define IOCTL_SET_LEDS				_IOWR(IOCTL_IDENT, 6, int)
#define IOCTL_GPIO_PORT_MUTEX_OP	_IOWR(IOCTL_IDENT, 11, int)
#define IOCTL_GPIO_PORT_MUTEX_RESET _IOWR(IOCTL_IDENT, 12, int)
#define IOCTL_DRIVER_VERSION 		_IOWR(IOCTL_IDENT, 13, int)
#define IOCTL_PCBA_VERSION 			_IOWR(IOCTL_IDENT, 14, int)

#define GPIO_ACCESS_CTRL_OFF		0
#define GPIO_ACCESS_CTRL_ON			1
#define GPIO_ACCESS_CTRL_LOCKED		2

#define RETRON_MUTEX_UNLOCK			0
#define RETRON_MUTEX_LOCK			1

#define FPGA_DATA_MASK			0xFF	// PD0-PD7 = data bus

/*
 * Data bus direct access:
 * pins = GPIO4C0 - GPIO4C7 = rk30_gpio_banks[4], offset 24 - 31
 */

#define DATA_BUS_BANK			rk30_gpio_banks[4]
#define DATA_BUS_BIT_OFFSET		(16)
#define DATA_BUS_MASK			(0xff)

#define POWER_CTRL_ON			1
#define POWER_CTRL_OFF			0
#define POWER_CTRL_GPIO			303		// GPIO4_B7

/*
 * PCB-A version detection
 *
 * Ver 0: B2=0,B3=0,B4=1 (0x04)
 * Ver 1: B2=1,B3=1,B4=0 (0x03)
 */
#define PCBA_VER_GPIO0			(330)		// GPIO6_B2
#define PCBA_VER_GPIO1			(331)		// GPIO6_B3
#define PCBA_VER_GPIO2			(332)		// GPIO6_B4

struct rk30_gpio_bank {
	struct gpio_chip chip;
	unsigned short id;
	short irq;
	void __iomem *regbase;	/* Base of register bank */
	struct clk *clk;
	u32 suspend_wakeup;
	u32 saved_wakeup;
	spinlock_t lock;
};

struct tracked_mutex
{
	struct mutex *m;
	pid_t pid;
};

extern struct rk30_gpio_bank rk30_gpio_banks[6];
extern struct tracked_mutex port_locks[3];
extern pid_t retron5_locked_pid;
extern int retron5_access_flag;

void configure_pcba_hardware(void);

int gpio_access_ctrl(int flag);
int gpio_check_access(void);
void gpio_set_port_direction(int port, unsigned int mask, unsigned int direction_flags);
void gpio_set_port_bits(int port, unsigned int mask, int value);
unsigned int gpio_get_port_bits(int port, unsigned int mask);

//
// Inline code begins here
//

static inline int check_port_access(uint32_t port)
{
	if(port >= (sizeof(port_locks) / sizeof(struct tracked_mutex)))
	{
		D_PRINTF("invalid port on lock: %d\n", port);
		return -1;
	}

	// TODO: better semantics to handle this case..
	if((retron5_access_flag == GPIO_ACCESS_CTRL_LOCKED) && (retron5_locked_pid == GET_PROCESS_PID))
		return 0;

	if(mutex_is_locked(port_locks[port].m) == 0)
		return -1;

	if(port_locks[port].pid != GET_PROCESS_PID)
		return -1;

	return 0;
}

static inline void lock_port(uint32_t port)
{
	int rv;
	if(port >= (sizeof(port_locks) / sizeof(struct tracked_mutex)))
	{
		D_PRINTF("invalid port on lock: %d\n", port);
		return;
	}

	rv = mutex_lock_interruptible(port_locks[port].m);
	if(rv < 0)
	{
		D_PRINTF("failed to lock mutex for port %d\n", port);
		return;
	}
	port_locks[port].pid = GET_PROCESS_PID;
}

static inline void unlock_port(uint32_t port)
{
	if(port >= (sizeof(port_locks) / sizeof(struct tracked_mutex)))
	{
		D_PRINTF("invalid port on unlock: %d\n", port);
		return;
	}

	mutex_unlock(port_locks[port].m);
}

static inline unsigned int rk30_get_databus(void)
{
	void __iomem *regbase = DATA_BUS_BANK.regbase;
	unsigned int ret = __raw_readl(regbase + GPIO_EXT_PORT);
	return (ret >> DATA_BUS_BIT_OFFSET) & DATA_BUS_MASK;
}

static inline void rk30_set_databus(unsigned long data)
{
	unsigned long flags, val;
	void __iomem *regbase = DATA_BUS_BANK.regbase;
	spin_lock_irqsave(&DATA_BUS_BANK.lock, flags);
	val = __raw_readl(regbase + GPIO_SWPORT_DR);
	val &= ~(DATA_BUS_MASK << DATA_BUS_BIT_OFFSET);
	val |= (data & DATA_BUS_MASK) << DATA_BUS_BIT_OFFSET;
	__raw_writel(val, regbase + GPIO_SWPORT_DR);
	spin_unlock_irqrestore(&DATA_BUS_BANK.lock, flags);
}
#endif // _RETRON5_DRIVER_H

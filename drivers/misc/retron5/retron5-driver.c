#include "retron5-driver.h"

void retron_set_leds(int ledMask);
void retron_hw_powerctrl(int enable);
static int file_write(char *file_path, char *buf);
static void sleep_early_resume(struct early_suspend *h);
static void sleep_early_suspend(struct early_suspend *h);

static DEFINE_MUTEX(access_lock);
static DEFINE_MUTEX(led_lock);
static DEFINE_MUTEX(fpga_lock);
static DEFINE_MUTEX(joy0_lock);
static DEFINE_MUTEX(joy1_lock);
struct tracked_mutex port_locks[3] = {
	{ &fpga_lock, 0 },
	{ &joy0_lock, 0 },
	{ &joy1_lock, 0 },
};
static int dev_major;
static dev_t device_handle;
static struct class *device_class;
static int last_led_mask = 0x1f;
int retron5_access_flag = GPIO_ACCESS_CTRL_OFF;
pid_t retron5_locked_pid = -1;
EXPORT_SYMBOL(port_locks);
EXPORT_SYMBOL(retron5_access_flag);
EXPORT_SYMBOL(retron5_locked_pid);

const static int gpio_port_max_bits[3] = { 28, 18, 18 };

#define BIT_MAP_TOTAL_SIZE	(sizeof(gpio_bit_mapping) / (2*sizeof(int)))
#define BIT_MAP_PORT_SIZE	(sizeof(gpio_bit_mapping) / 3 / (2*sizeof(int)))
const static int gpio_bit_mapping[3][32][2] = {
	{
		// GPIO_PORT_FPGA
		{ 0x0000001, 304 }, // DATA 0
		{ 0x0000002, 305 },
		{ 0x0000004, 306 },
		{ 0x0000008, 307 },
		{ 0x0000010, 308 },
		{ 0x0000020, 309 },
		{ 0x0000040, 310 },
		{ 0x0000080, 311 }, // DATA 7
		{ 0x0000100, 315 }, // CPU_DOUT_BUSY - GPIO4_D3
		{ 0x0000200, 318 }, // CPU_INIT_B - GPIO4_D6
		{ 0x0000400, 314 }, // CPU_CSI_B - GPIO4_D2
		{ 0x0000800, 316 }, // CPU_PROG_B - GPIO4_D4
		{ 0x0001000, 317 }, // CPU_DONE - GPIO4_D5
		{ 0x0002000, 312 }, // CPU_CCLK - GPIO4_D0
		{ 0x0004000,  319 }, // CPU_FPGA_IO_07 - GPIO4_D7
		{ 0x0008000, 313 }, // CPU_RDRW - GPIO4_D1
		{ 0x0010000, 185 }, // CPU_FPGA_IO_08 - GPIO0_D1
		{ 0x0020000, 186 }, // CPU_FPGA_IO_09 - GPIO0_D2
		{ 0x0040000, 187 }, // CPU_FPGA_IO_10 - GPIO0_D3
		{ 0x0080000, 188 }, // CPU_FPGA_IO_11 - GPIO0_D4		
	},
	{
		// GPIO_PORT_JOY0
		{ 0x00001, 202 }, 
		{ 0x00002, 203 },
		{ 0x00004, 204 },
		{ 0x00008, 205 },
		{ 0x00010, 206 },
		{ 0x00020, 207 },
		{ 0x00040, 208 },
		{ 0x00080, 209 }, 
		{ 0x00100, 210 },
		{ 0x00200, 211 },
		{ 0x00400, 212 },
		{ 0x00800, 213 },
		{ 0x01000, 214 },
		{ 0x02000, 215 },
		{ 0x04000, 216 },
		{ 0x08000, 217 },
		{ 0x10000, 218 },
		{ 0x20000, 222 },
	},
	{
		// GPIO_PORT_JOY1
		{ 0x00001, 225 }, 
		{ 0x00002, 226 },
		{ 0x00004, 237 },
		{ 0x00008, 238 },
		{ 0x00010, 239 },
		{ 0x00020, 240 },
		{ 0x00040, 241 },
		{ 0x00080, 242 }, 
		{ 0x00100, 243 },
		{ 0x00200, 244 },
		{ 0x00400, 245 },
		{ 0x00800, 246 },
		{ 0x01000, 247 },
		{ 0x02000, 256 },
		{ 0x04000, 257 },
		{ 0x08000, 258 },
		{ 0x10000, 259 },
		{ 0x20000, 260 },
	},
/*
	{
		// GPIO_PORT_EXP1
		{ 0x00001, 164 }, 
		{ 0x00002, 167 },
		{ 0x00004, 168 },
		{ 0x00008, 182 },
		{ 0x00010, 183 },
		{ 0x00020, 184 },
		{ 0x00040, 189 },
		{ 0x00080, 190 }, 
		{ 0x00100, 176 },
		{ 0x00200, 196 },
		{ 0x00400, 197 },
		{ 0x00800, 198 },
		{ 0x01000, 199 },
		{ 0x02000, 223 },
		{ 0x04000, 252 },
		{ 0x08000, 253 },
		{ 0x10000, 261 },
		{ 0x20000, 262 },
	},
	{
		// GPIO_PORT_EXP2
		{ 0x00001, 283 }, 
		{ 0x00002, 284 },
		{ 0x00004, 285 },
		{ 0x00008, 286 },
		{ 0x00010, 287 },
		{ 0x00020, 272 },
		{ 0x00040, 273 },
		{ 0x00080, 274 }, 
		{ 0x00100, 275 },
		{ 0x00200, 276 },
		{ 0x00400, 277 },
		{ 0x00800, 320 },
		{ 0x01000, 325 },
		{ 0x02000, 326 },
		{ 0x04000, 327 },
		{ 0x08000, 330 },
		{ 0x10000, 331 },
		{ 0x20000, 332 },
	},
*/
};

void retron_set_leds(int ledMask);
void retron_hw_powerctrl(int enable);
unsigned int rk30_get_databus(void);
void rk30_set_databus(unsigned long data);

static int file_read(char *file_path, char *buf);
static int file_write(char *file_path, char *buf);
static void sleep_early_resume(struct early_suspend *h);
static void sleep_early_suspend(struct early_suspend *h);

static void reset_port_mutexes(void)
{
	int port;
	for(port = 0; port < (sizeof(port_locks) / sizeof(struct tracked_mutex)); port++)
		if(mutex_is_locked(port_locks[port].m))
			mutex_unlock(port_locks[port].m);
}

int gpio_check_access(void)
{
	int rv = -1;
	mutex_lock(&access_lock);
	switch(retron5_access_flag)
	{
		case GPIO_ACCESS_CTRL_OFF:
			rv = -1;
			break;
		case GPIO_ACCESS_CTRL_ON:
			rv = 0;
			break;
		case GPIO_ACCESS_CTRL_LOCKED:
		{
			if(retron5_locked_pid == GET_PROCESS_PID)
				rv = 0;
			else
			{
//				D_PRINTF("blocked access from pid %d (owner %d)\n", GET_PROCESS_PID, retron5_locked_pid);
				rv = -1;
			}
			break;
		}
	}
	mutex_unlock(&access_lock);
	return rv;
}
EXPORT_SYMBOL(gpio_check_access);

int gpio_access_ctrl(int flag)
{
	int i;
	if(flag == GPIO_ACCESS_CTRL_ON)
	{
		if(retron5_access_flag == GPIO_ACCESS_CTRL_ON)
			return 0;
		else if(retron5_access_flag == GPIO_ACCESS_CTRL_LOCKED)
		{
			if(GET_PROCESS_PID != retron5_locked_pid)
			{
				D_PRINTF("GPIO locked, cannot enable from other process\n");
				return -1;
			}
			
			reset_port_mutexes();
		}
			
		for(i = 0; i < BIT_MAP_TOTAL_SIZE; i++)
		{
			const int *thisGpio = gpio_bit_mapping[i / BIT_MAP_PORT_SIZE][i % BIT_MAP_PORT_SIZE];
			if(thisGpio[1] == 0)
				continue;
				
			gpio_direction_input(thisGpio[1]);
			gpio_pull_updown(thisGpio[1], PullDisable);
		}
		
		D_PRINTF("GPIO access enabled\n");
		retron5_access_flag = GPIO_ACCESS_CTRL_ON;
	}
	else if(flag == GPIO_ACCESS_CTRL_OFF)
	{
		if(retron5_access_flag == GPIO_ACCESS_CTRL_OFF)
			return 0;
		else if(retron5_access_flag == GPIO_ACCESS_CTRL_LOCKED)
		{
			if(GET_PROCESS_PID != retron5_locked_pid)
			{
				D_PRINTF("GPIO locked, cannot disable from other process\n");
				return -1;
			}

			reset_port_mutexes();
		}

		for(i = 0; i < BIT_MAP_TOTAL_SIZE; i++)
		{
			const int *thisGpio = gpio_bit_mapping[i / BIT_MAP_PORT_SIZE][i % BIT_MAP_PORT_SIZE];
			if(thisGpio[1] == 0)
				continue;
			
			gpio_direction_input(thisGpio[1]);
		}
			
		D_PRINTF("GPIO access disabled\n");
		retron5_access_flag = GPIO_ACCESS_CTRL_OFF;
		retron5_locked_pid = -1;
	}
	else if(flag == GPIO_ACCESS_CTRL_LOCKED)
	{
		if(retron5_access_flag != GPIO_ACCESS_CTRL_ON)
		{
			D_PRINTF("cannot lock GPIO access, invalid state\n");
			return -1;
		}
		
		retron5_locked_pid = GET_PROCESS_PID;
		retron5_access_flag = GPIO_ACCESS_CTRL_LOCKED;
		reset_port_mutexes();
		D_PRINTF("GPIO access locked to process: %d\n", retron5_locked_pid);
	}
	else
	{
		D_PRINTF("invalid access flag: %d\n", flag);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(gpio_access_ctrl);

static inline int get_gpio_for_bit(int port, int bit)
{
	int i;
	for(i = 0; i < gpio_port_max_bits[port]; i++)
		if(gpio_bit_mapping[port][i][0] & bit)
			return gpio_bit_mapping[port][i][1];
			
//	D_PRINTF("bit not found in array: %X\n", bit);
	return 0;
}

void gpio_set_port_pull(int port, unsigned int mask, unsigned int pull_flags)
{
	int i;
	for(i = 0; i < gpio_port_max_bits[port]; i++)
	{
		if(mask & (1 << i))
		{
			int enable, gpio = get_gpio_for_bit(port, 1 << i);
			if(!gpio)
				continue;
			enable = (pull_flags >> i) & 0x01;
			gpio_pull_updown(gpio, (enable) ? 1 : 0);
		}	
	}		
}
EXPORT_SYMBOL(gpio_set_port_pull);

void gpio_set_port_direction(int port, unsigned int mask, unsigned int direction_flags)
{
	int i;
	for(i = 0; i < gpio_port_max_bits[port]; i++)
	{
		if(mask & (1 << i))
		{
			int dir, gpio = get_gpio_for_bit(port, 1 << i);
			if(!gpio)
				continue;
			dir = (direction_flags >> i) & 0x01;
			if(dir == GPIO_IN)
				gpio_direction_input(gpio);
			else
				gpio_direction_output(gpio, 1);
		}	
	}	
}
EXPORT_SYMBOL(gpio_set_port_direction);

void gpio_set_port_bits(int port, unsigned int mask, int value)
{
	int i;
	// speed optimisation
	if(port == GPIO_PORT_FPGA && mask == DATA_BUS_MASK)
	{
		rk30_set_databus(value);
		return;
	}

	for(i = 0; i < gpio_port_max_bits[port]; i++)
	{
		if(mask & (1 << i))
		{
			int bit, gpio = get_gpio_for_bit(port, 1 << i);
			if(!gpio)
				continue;
			bit = (value >> i) & 0x01;
			gpio_set_value(gpio, bit);
		}
	}
}
EXPORT_SYMBOL(gpio_set_port_bits);

unsigned int gpio_get_port_bits(int port, unsigned int mask)
{
	int i;
	unsigned int value = 0;

	// speed optimisation
	if(port == GPIO_PORT_FPGA && mask == DATA_BUS_MASK)
		return rk30_get_databus();

	for(i = 0; i < gpio_port_max_bits[port]; i++)
	{
		if(mask & (1 << i))
		{
			int bit, gpio = get_gpio_for_bit(port, 1 << i);
			if(!gpio)
				continue;
			bit = gpio_get_value(gpio) & 0x01;
			value |= (bit << i);		
		}		
	}
	return value;
}
EXPORT_SYMBOL(gpio_get_port_bits);

static int get_pcba_revision(void)
{
	int raw_ver =	(gpio_get_value(PCBA_VER_GPIO2) << 2) |
					(gpio_get_value(PCBA_VER_GPIO1) << 1) |
					(gpio_get_value(PCBA_VER_GPIO0) << 0);
	return raw_ver;
}

long retron5_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	u8 cmd_buf[1024];
	unsigned int *cmd32 = (unsigned int *)cmd_buf;
	int rv = -EINVAL, port;
	
//	D_PRINTF("ioctl: %X\n", cmd);
	
	switch(cmd)
	{
		case IOCTL_DRIVER_VERSION:
			rv = RETRO_VERSION_TAG | DRIVER_VER;
			break;
		case IOCTL_PCBA_VERSION:
			cmd32[0] = get_pcba_revision();
			if(copy_to_user((void *)arg, cmd_buf, 1*sizeof(int)))
				BAIL(-1);
			rv = 0;
			break;
		case IOCTL_GPIO_SET_BITS:
			if(gpio_check_access())
				BAIL(-1);
			if(copy_from_user(cmd_buf, (const void *)arg, 3*sizeof(int)))
				BAIL(-1);
//			D_PRINTF("IOCTL_GPIO_SET_BITS: %X, %X, %X\n", cmd32[0], cmd32[1], cmd32[2]);
			port = cmd32[0];
			if(check_port_access(port))
				BAIL(-1);
			gpio_set_port_bits(cmd32[0], cmd32[1], cmd32[2]);
			rv = 0;
			break;
		case IOCTL_GPIO_GET_BITS:
			if(gpio_check_access())
				BAIL(-1);
			if(copy_from_user(cmd_buf, (const void *)arg, 2*sizeof(int)))
				BAIL(-1);
//			D_PRINTF("IOCTL_GPIO_GET_BITS: %X, %X\n", cmd32[0], cmd32[1]);
			port = cmd32[0];
			if(check_port_access(port))
				BAIL(-1);
			cmd32[0] = gpio_get_port_bits(cmd32[0], cmd32[1]);
			if(copy_to_user((void *)arg, cmd_buf, 1*sizeof(int)))
				BAIL(-1);
			rv = 0;
			break;
		case IOCTL_GPIO_SET_DIRECTION:
			if(gpio_check_access())
				BAIL(-1);
			if(copy_from_user(cmd_buf, (const void *)arg, 3*sizeof(int)))
				BAIL(-1);
//			D_PRINTF("IOCTL_GPIO_SET_DIRECTION: %X, %X, %X, pid=%d\n", cmd32[0], cmd32[1], cmd32[2], GET_PROCESS_PID);
			port = cmd32[0];
			if(check_port_access(port))
				BAIL(-1);
			gpio_set_port_direction(cmd32[0], cmd32[1], cmd32[2]);
			rv = 0;
			break;
		case IOCTL_GPIO_SET_PULL:
			if(gpio_check_access())
				BAIL(-1);
			if(copy_from_user(cmd_buf, (const void *)arg, 3*sizeof(int)))
				BAIL(-1);
//			D_PRINTF("IOCTL_GPIO_SET_PULL: %X, %X, %X\n", cmd32[0], cmd32[1], cmd32[2]);
			port = cmd32[0];
			if(check_port_access(port))
				BAIL(-1);
			gpio_set_port_pull(cmd32[0], cmd32[1], cmd32[2]);
			rv = 0;
			break;
		case IOCTL_GPIO_ACCESS_CTRL:
			mutex_lock(&access_lock);		
//			D_PRINTF("IOCTL_GPIO_ACCESS_CTRL: %X, pid=%d\n", arg, GET_PROCESS_PID);
			rv = gpio_access_ctrl(arg);
			mutex_unlock(&access_lock);
			break;
/*			
		case IOCTL_SET_LEDS:
			if(gpio_check_access())
				BAIL(-1);		
			mutex_lock(&led_lock);	
			retron_set_leds(arg);
			rv = 0;	
			mutex_unlock(&led_lock);
			break;
*/			
		case IOCTL_GPIO_PORT_MUTEX_OP:
			if(gpio_check_access())
				BAIL(-1);
			if(copy_from_user(cmd_buf, (const void *)arg, 2*sizeof(int)))
				BAIL(-1);
			if(cmd32[1] == RETRON_MUTEX_LOCK)
				lock_port(cmd32[0]);
			else
				unlock_port(cmd32[0]);
			rv = 0;
			break;
		case IOCTL_GPIO_PORT_MUTEX_RESET:
			reset_port_mutexes();
			rv = 0;
			break;
		default:
			D_PRINTF("unknown ioctl: %X\n", cmd);
			BAIL(-1);
	}
	
end:	
	return rv;
}
EXPORT_SYMBOL(retron5_ioctl);

void retron_set_leds(int ledMask)
{
	int i;
	
//	lock_port(GPIO_PORT_JOY1);
	
	gpio_set_port_bits(GPIO_PORT_JOY1, 0x10 | 0x02, 0x10 | 0x02);
	gpio_set_port_direction(GPIO_PORT_JOY1, 0x10 | 0x02, 0x10 | 0x02);

	for(i = 0; i < 5; i++)
	{
		gpio_set_port_bits(GPIO_PORT_JOY1, 0x02, 0x02);
		gpio_set_port_bits(GPIO_PORT_JOY1, 0x10, ((1 << i) & ledMask) ? 0 : 0x10);
		gpio_set_port_bits(GPIO_PORT_JOY1, 0x02, 0);
	}
//	unlock_port(GPIO_PORT_JOY1);
	
//	last_led_mask = ledMask;
}

void retron_hw_powerctrl(int enable)
{
	D_PRINTF("retron_hw_powerctrl: %d\n", enable);
	gpio_direction_output(POWER_CTRL_GPIO, 1);
	if(enable)
		gpio_set_value(POWER_CTRL_GPIO, POWER_CTRL_ON);
	else
		gpio_set_value(POWER_CTRL_GPIO, POWER_CTRL_OFF);
}

static struct early_suspend early_suspend_handler = {
	.suspend = sleep_early_suspend,
	.resume = sleep_early_resume,
};

static struct file_operations fops = {
	.unlocked_ioctl = retron5_ioctl,
};

static int __init retron5_module_init(void)
{	
	int i, rv;
	// reserve all the GPIO
	for(i = 0; i < BIT_MAP_TOTAL_SIZE; i++)
	{
		const int *thisGpio = gpio_bit_mapping[i / BIT_MAP_PORT_SIZE][i % BIT_MAP_PORT_SIZE];
		if(thisGpio[1] == 0)
			continue;			
		rv = gpio_request(thisGpio[1], NULL);
	}
	gpio_request(POWER_CTRL_GPIO, NULL);
	
	retron_hw_powerctrl(1);
	
	dev_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (dev_major < 0) 
	{
		D_PRINTF("Registering the character device failed with %d\n", dev_major);
		return -1;
	}
	
	device_handle = MKDEV(dev_major, 0);

    device_class = class_create(THIS_MODULE, DEVICE_FILE);
    if(device_class == NULL)
    {
		unregister_chrdev(dev_major, DEVICE_NAME);
		D_PRINTF("Class creation failed\n");
		return -1;
    }

    if(device_create(device_class, NULL, device_handle, NULL, DEVICE_FILE) == NULL)
    {
		class_destroy(device_class);
		unregister_chrdev(dev_major, DEVICE_NAME);
		D_PRINTF("Device creation failed\n");
		return -1;
    }
    
	register_early_suspend(&early_suspend_handler);
	for(i = 0; i < 8; i++)
		retron_set_leds(0x1f);

	// setup PCB-A version checking
	gpio_direction_input(PCBA_VER_GPIO0);
	gpio_direction_input(PCBA_VER_GPIO1);
	gpio_direction_input(PCBA_VER_GPIO2);
	gpio_pull_updown(PCBA_VER_GPIO0, 1);
	gpio_pull_updown(PCBA_VER_GPIO1, 1);
	gpio_pull_updown(PCBA_VER_GPIO2, 1);
	printk(TAG "PCB-A revision: %X\n", get_pcba_revision());

	// perform the PCB-A hardware configuration, which currently consists of auto-detection
	// of the connected BT chipset (MT6622 or RDA5876) and registering the detected device with
	// the kernel subsystem
	configure_pcba_hardware();

    printk(TAG "driver initialised, major = %d\n", dev_major);
 	return 0;
}

static void __exit retron5_module_exit(void)
{
	D_PRINTF("driver exit\n");
	device_destroy(device_class, device_handle);
	class_destroy(device_class);	
	unregister_chrdev(dev_major, DEVICE_NAME);
	unregister_early_suspend(&early_suspend_handler);
}  

// Power management stuff

#define FILE_CPU_MAX_FREQ 	"/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"
#define NORMAL_CPU_FREQ		(1608 * 1000)
#define SLEEP_CPU_FREQ		(252 * 1000)

static int file_read(char *file_path, char *buf)
{
	struct file *file = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

	D_PRINTF("file_read %s\n", file_path);
	file = filp_open(file_path, O_RDONLY, 0);

	if (IS_ERR(file)) {
		D_PRINTF("%s error open file  %s\n", __func__, file_path);
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	file->f_op->read(file, (char *)buf, 32, &offset);
	sscanf(buf, "%s", buf);

	set_fs(old_fs);
	filp_close(file, NULL);  
	file = NULL;
	return 0;

}

static int file_write(char *file_path, char *buf)
{
	struct file *file = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

	D_PRINTF("file_write %s %s size = %d\n", file_path, buf, strlen(buf));
	file = filp_open(file_path, O_RDWR, 0);

	if (IS_ERR(file)) {
		D_PRINTF("%s error open file  %s\n", __func__, file_path);
		return -1;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	file->f_op->write(file, (char *)buf, strlen(buf), &offset);

	set_fs(old_fs);
	filp_close(file, NULL);  
	file = NULL;
	return 0;

}

static void set_cpu_freq(int freq)
{
	char buf[32];
	snprintf(buf, 32, "%d", freq);
	file_write(FILE_CPU_MAX_FREQ, buf);
}

static void sleep_early_suspend(struct early_suspend *h)
{
	set_cpu_freq(SLEEP_CPU_FREQ);
	retron_hw_powerctrl(0);
}

static void sleep_early_resume(struct early_suspend *h)
{
	retron_hw_powerctrl(1);
	set_cpu_freq(NORMAL_CPU_FREQ);
//	retron_set_leds(last_led_mask); // resume last LED state
}

module_init(retron5_module_init);
module_exit(retron5_module_exit);
MODULE_LICENSE("GPL");

#include "retron5-driver.h"

#define I2C_SCL_RATE				100000
#define I2C_RETRY_DELAY				5
#define I2C_RETRIES					3
#define RDA5400_I2C_ADDR			0x16

extern struct platform_device device_mt6622;
extern struct platform_device device_rda_bt;

static struct i2c_client *rda_i2c_client = NULL;

static int __devinit rda5876_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	D_PRINTF("rda5876_i2c_probe\n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	rda_i2c_client = client;
	return 0;
}

static int __devexit rda5876_i2c_remove(struct i2c_client *client)
{
	D_PRINTF("rda5876_i2c_remove\n");
	rda_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id rda5876_id[] = {
	{ "rda5876", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rda5876_id);

static struct i2c_driver rda5876_i2c_driver = {
	.driver		= {
		.name	= "rda5876",
		.owner	= THIS_MODULE,
	},
	.probe		= rda5876_i2c_probe,
	.remove		= __devexit_p(rda5876_i2c_remove),
	.id_table	= rda5876_id,
};

static int RDA5400_WriteData(struct i2c_client *client, u8 regaddr, u16* data, u32 datalen)
{
    u8	temp[3];
    u32	i=0;
    int	err = 0;
    int	tries = 0;

    for(i=0;i<datalen;i++)
    {
		struct i2c_msg msg[] = {
			{
				.addr	= RDA5400_I2C_ADDR,
				.flags	= 0,
				.len	= 3,
				.buf	= temp,
				.scl_rate = I2C_SCL_RATE,
			}
		};
        temp[0]=regaddr;
        temp[1]=(u8)(((*data) & 0xff00)>>8);
        temp[2]=(u8)((*data) & 0x00ff);

		do {
			err = i2c_transfer(client->adapter, msg, 1);
			if (err != 1)
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != 1) && (++tries < I2C_RETRIES));

		if (err != 1) {
			dev_err(&client->dev, "5400 write transfer error\n");
			err = -EIO;
		} else {
			err = 0;
		}
    }
	return err;
}

static int RDA5400_ReadData(struct i2c_client *client, u8 regaddr, u16* data,int len)
{
	u8 temp[3]={0};
    int err = 0;
    int tries = 0;
    struct i2c_msg	msgs[] =
    {
        {
            .addr = RDA5400_I2C_ADDR,
            .flags = 0,
            .len = 1,
            .buf = temp,
            .scl_rate = I2C_SCL_RATE,
        },
        {
            .addr = RDA5400_I2C_ADDR,
            .flags = I2C_M_RD,
            .len = 2,
            .buf = temp,
            .scl_rate = I2C_SCL_RATE,
        },
    };

	temp[0]=regaddr;

    do {
	    err = i2c_transfer(client->adapter, msgs, 2);
        	if (err != 2)
				msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2)
        {
	    dev_err(&client->dev, "5400 write transfer error\n");
        	err = -EIO;
	}
        else
        {
	    *data=temp[1]+(temp[0]<<8);
        	err = 0;
	}

    return err;
}

static int rda_get_chip_id(unsigned short *_chipid, unsigned short *_revid)
{
    unsigned char pageaddr = 0x3f;
    unsigned char chipidaddr = 0x20;
    unsigned char revidaddr =0x21;
    unsigned short pagevalue = 0x0001;
    unsigned short chipid=0xffff;
    unsigned short revid=0;

    if(rda_i2c_client == NULL)
		return -1;

    if(RDA5400_WriteData(rda_i2c_client, pageaddr, &pagevalue,1)) return -1;
    if(RDA5400_ReadData(rda_i2c_client, chipidaddr,&chipid,1)) return -1;
    if(RDA5400_ReadData(rda_i2c_client, revidaddr,&revid,1)) return -1;

	*_chipid = chipid;
	*_revid = revid;
	return 0;
}

int bt_configure_thread(void *p)
{
    int ret;
    unsigned short chipid = 0, revid = 0;
    struct rda_bt_platform_data *pdata = (struct rda_bt_platform_data *)device_rda_bt.dev.platform_data;

	D_PRINTF("bt_configure_thread\n");

	gpio_direction_output(pdata->power_gpio.io, pdata->power_gpio.enable);
	msleep(25);
    ret = i2c_add_driver(&rda5876_i2c_driver);
    if(ret)
    {
		D_PRINTF("rda i2c add failed: %d\n", ret);
		return ret;
	}
	ret = rda_get_chip_id(&chipid, &revid);
	D_PRINTF("rda_get_chip_id: ret=%d, chipid=%04X, revid=%04X\n", ret, chipid, revid);
	i2c_del_driver(&rda5876_i2c_driver);
	gpio_direction_input(pdata->power_gpio.io);

	if((ret == 0) && (chipid == 0x5876))
	{
		struct platform_device *devices[] = { &device_rda_bt };
		D_PRINTF("adding RDA5876 device\n");
		platform_add_devices(devices, ARRAY_SIZE(devices));
	}
	else
	{
		struct platform_device *devices[] = { &device_mt6622 };
		D_PRINTF("adding MT6622 device\n");
		platform_add_devices(devices, ARRAY_SIZE(devices));
	}

	return 0;
}

void configure_pcba_hardware(void)
{
	struct task_struct *th = kthread_create(bt_configure_thread, NULL, "bt_configure_thread");
	if(th) wake_up_process(th);
}

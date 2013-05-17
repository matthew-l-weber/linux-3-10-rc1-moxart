/* MOXART watchdog driver (based on MOXA sources)
 * Copyright (C) 2013 Jonas Jensen <jonas.jensen@gmail.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version. */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/seq_file.h>

#include <asm/system_misc.h>

#define MOXA_WATCHDOG_MINOR			WATCHDOG_MINOR

#define APB_CLK						48000000

/* 30 seconds */
#define DEFAULT_WATCHDOG_TIME		(30UL*1000UL)

/* 50 msec */
#define WATCHDOG_MIN_TIME			50UL

/* 60 seconds */
#define WATCHDOG_MAX_TIME			(60UL*1000UL)

/* 500 msec, for watchdog timer polling */
#define WATCHDOG_TOL_TIME			(500UL)

/* 1 seconds, for watchdog timer count */
#define WATCHDOG_TOL_COUNT_TIME		(1000UL)

#define WATCHDOG_COUNTER(x)	((APB_CLK/1000UL)*(x))
#define WATCHDOG_JIFFIES(x)	((((x)+WATCHDOG_TOL_TIME)*HZ)/1000UL)

/*
#define DEBUG_TEST_WATCHDOG_SOFT
#define DEBUG_TEST_WATCHDOG_HARD
*/

/* With both DEBUG_TEST_WATCHDOG and DEBUG_TEST_WATCHDOG_HARD set,
 * watchdog reset can be verified.
 * after 10 seconds wdt_timer calls wdt_poll, another
 * 10 seconds a hardware reset is forced.

 * user mode reboot is preferred (terminate processes and shut down cleanly)
 * (but hardware watchdog should still work)
 * (unless lockup happens after wdt_disable) */

struct wdt_set_struct {
	int mode;
	unsigned long time;
};

static int opencounts;
static int wdt_user_enabled;
static unsigned long wdt_time = DEFAULT_WATCHDOG_TIME;
static struct timer_list wdt_timer;
static struct work_struct rebootqueue;
static void __iomem *reg_wdt;

static void wdt_enable(void)
{
	writel(WATCHDOG_COUNTER(wdt_time + WATCHDOG_TOL_COUNT_TIME),
		reg_wdt + 4);
	writel(0x5ab9, reg_wdt + 8);
	writel(0x03, reg_wdt + 12);
}

static void wdt_restart(char str, const char *cmd)
{
	writel(1, reg_wdt + 4);
    writel(0x5ab9, reg_wdt + 8);
    writel(0x03, reg_wdt + 12);
}

static void wdt_disable(void)
{
	writel(0, reg_wdt + 12);
}

static int wdt_set_timeout(int new_time)
{
	if ((new_time <= 0) || (new_time > WATCHDOG_MAX_TIME))
		return -EINVAL;

	wdt_time = new_time;

	return 0;
}

static void wdt_start(void)
{
	unsigned long flags;

	local_irq_save(flags);
	if (wdt_user_enabled) {
		wdt_disable();
		del_timer(&wdt_timer);
	}
	wdt_user_enabled = 1;
	wdt_timer.expires = jiffies + WATCHDOG_JIFFIES(wdt_time);
	add_timer(&wdt_timer);
	local_irq_restore(flags);
}

static void wdt_enable_card(void)
{
	unsigned long flags;

	local_irq_save(flags);
	wdt_enable();
	local_irq_restore(flags);
}

static void wdt_disable_card(void)
{
	unsigned long flags;

	local_irq_save(flags);
	if (wdt_user_enabled) {
		wdt_disable();
		del_timer(&wdt_timer);
		wdt_user_enabled = 0;
	}
	local_irq_restore(flags);
}

static void wdt_reload(void)
{
	unsigned long flags;

	local_irq_save(flags);
	if (wdt_user_enabled) {
		wdt_disable();
		del_timer(&wdt_timer);
		wdt_timer.expires = jiffies +
			WATCHDOG_JIFFIES(wdt_time);
		add_timer(&wdt_timer);
		wdt_enable();
	}
	local_irq_restore(flags);
}

static const struct watchdog_info moxart_wdt_info = {
	.identity   = "moxart watchdog",
	.options    = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};

static long wdt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &moxart_wdt_info,
				sizeof(moxart_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;
		if (new_value & WDIOS_DISABLECARD)
			wdt_disable_card();
		if (new_value & WDIOS_ENABLECARD)
			wdt_enable_card();
		return 0;
	case WDIOC_KEEPALIVE:
		wdt_reload();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		if (wdt_set_timeout(new_value))
			return -EINVAL;
		wdt_start();
		return put_user(wdt_time, p);
	case WDIOC_GETTIMEOUT:
		return put_user(wdt_time, p);
	default:
		return -ENOTTY;
	}
}

static int wdt_open(struct inode *inode, struct file *file)
{
	unsigned long flags;

	pr_debug("%s\n", __func__);
	if (MINOR(inode->i_rdev) != MOXA_WATCHDOG_MINOR)
		return -ENODEV;
	local_irq_save(flags);
	opencounts++;
	local_irq_restore(flags);
	return 0;
}

static int wdt_release(struct inode *inode, struct file *file)
{
	unsigned long flags;

	local_irq_save(flags);
	pr_debug("%s\n", __func__);
	opencounts--;
	if (opencounts <= 0) {
		if (wdt_user_enabled) {
			wdt_disable();
			del_timer(&wdt_timer);
			wdt_user_enabled = 0;
		}
		opencounts = 0;
	}
	local_irq_restore(flags);
	return 0;
}

/* pat the watchdog whenever device is written to. */
static ssize_t wdt_write(struct file *file, const char *data,
	size_t len, loff_t *ppos)
{
	wdt_reload();      /* pat the watchdog */
	return len;
}

static void wdt_reboot(struct work_struct *work)
{
	char *argv[2], *envp[5];

	pr_debug("%s\n", __func__);

	if (in_interrupt())
		return;
	/* if (!current->fs->root) return; */
	argv[0] = "reboot";
	argv[1] = 0;
	envp[0] = "HOME=/";
	envp[1] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
	envp[2] = 0;

	pr_debug("%s: call_usermodehelper reboot\n", __func__);
	call_usermodehelper(argv[0], argv, envp, 0);
}

static void wdt_poll(unsigned long ignore)
{
	unsigned long flags;

	pr_debug("%s\n", __func__);

	local_irq_save(flags);
	wdt_disable();
#ifdef DEBUG_TEST_WATCHDOG_HARD
	wdt_enable();
#else
	del_timer(&wdt_timer);
	pr_debug("%s: rebooting the system.\n", __func__);
	schedule_work(&rebootqueue);
	local_irq_restore(flags);
#endif
}

static int wdt_show(struct seq_file *m, void *v)
{
	seq_printf(m, "user enable\t: %d\nack time\t: %d msec\n",
		wdt_user_enabled, (int) wdt_time);

	return 0;
}

static int wdt_show_open(struct inode *inode, struct file *file)
{
	return single_open(file, wdt_show, NULL);
}

static const struct file_operations moxart_wdt_show_fops = {
	.open		= wdt_show_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations moxart_wdt_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.unlocked_ioctl	= wdt_ioctl,
	.open			= wdt_open,
	.release		= wdt_release,
	.write			= wdt_write,
};

static struct miscdevice wdt_dev = {
	MOXA_WATCHDOG_MINOR,
	"moxart_watchdog",
	&moxart_wdt_fops
};

static int moxart_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource res_wdt;
	int err;

	err = of_address_to_resource(node, 0, &res_wdt);
	if (err) {
		dev_err(dev, "could not get watchdog base resource\n");
		return err;
	}

	reg_wdt = devm_ioremap_resource(dev, &res_wdt);
	if (IS_ERR(reg_wdt)) {
		dev_err(dev, "%s: devm_ioremap_resource res_wdt failed\n",
			__func__);
		return PTR_ERR(reg_wdt);
	}

	arm_pm_restart = wdt_restart;

	if (misc_register(&wdt_dev)) {
		dev_err(&pdev->dev, "%s: failed to register device.\n",
			__func__);
		goto moxart_wdt_init_err;
	}

	opencounts = 0;
	wdt_user_enabled = 0;

	INIT_WORK(&rebootqueue, wdt_reboot);
	init_timer(&wdt_timer);
	wdt_timer.function = wdt_poll;

#ifdef DEBUG_TEST_WATCHDOG_SOFT
	wdt_time = 10 * 1000;
	wdt_user_enabled = 1;
	wdt_timer.expires = jiffies + WATCHDOG_JIFFIES(wdt_time);
	add_timer(&wdt_timer);
#endif

	proc_create("moxart_wdt_show", 0400, NULL, &moxart_wdt_show_fops);

	dev_info(&pdev->dev, "finished %s base=%p\n", __func__, reg_wdt);

	return 0;

moxart_wdt_init_err:
	misc_deregister(&wdt_dev);
	return -ENOMEM;
}

static int moxart_wdt_remove(struct platform_device *pdev)
{
	unsigned long	flags;

	local_irq_save(flags);
	if (wdt_user_enabled) {
		wdt_disable();
		del_timer(&wdt_timer);
		wdt_user_enabled = 0;
		opencounts = 0;
	}
	local_irq_restore(flags);
	misc_deregister(&wdt_dev);

	return 0;
}

static const struct of_device_id moxart_watchdog_match[] = {
	{ .compatible = "moxa,moxart-watchdog" },
	{ },
};

static struct platform_driver moxart_wdt_driver = {
	.probe      = moxart_wdt_probe,
	.remove     = moxart_wdt_remove,
	.driver     = {
		.name			= "moxart-watchdog",
		.owner			= THIS_MODULE,
		.of_match_table = moxart_watchdog_match,
	},
};

static int __init moxart_wdt_init(void)
{
	return platform_driver_register(&moxart_wdt_driver);
}

static void __exit moxart_wdt_exit(void)
{
	platform_driver_unregister(&moxart_wdt_driver);
}

module_init(moxart_wdt_init);
module_exit(moxart_wdt_exit);

MODULE_DESCRIPTION("MOXART watchdog driver");
MODULE_LICENSE("GPL");

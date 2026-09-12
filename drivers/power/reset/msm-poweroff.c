// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/of_address.h>

static void __iomem *msm_ps_hold;
static void __iomem *restart_reason;

#define MSM_RESTART_REASON_NORMAL	0x77665501
#define MSM_RESTART_REASON_BOOTLOADER	0x77665500
#define MSM_RESTART_REASON_RECOVERY	0x77665502

static void msm_set_restart_reason(const char *cmd)
{
	u32 reason = MSM_RESTART_REASON_NORMAL;

	if (!restart_reason)
		return;

	if (cmd && !strcmp(cmd, "bootloader"))
		reason = MSM_RESTART_REASON_BOOTLOADER;
	else if (cmd && !strcmp(cmd, "recovery"))
		reason = MSM_RESTART_REASON_RECOVERY;

	writel_relaxed(reason, restart_reason);

	/* The bootloader reads IMEM immediately after PS_HOLD drops. */
	wmb();
}

static int do_msm_poweroff(struct sys_off_data *data)
{
	msm_set_restart_reason(data->cmd);

	writel(0, msm_ps_hold);
	mdelay(10000);

	return NOTIFY_DONE;
}

static int msm_restart_probe(struct platform_device *pdev)
{
	struct device_node *np;

	msm_ps_hold = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(msm_ps_hold))
		return PTR_ERR(msm_ps_hold);

	np = of_find_compatible_node(NULL, NULL,
				     "qcom,msm-imem-restart_reason");
	if (np) {
		restart_reason = devm_of_iomap(&pdev->dev, np, 0, NULL);
		of_node_put(np);
		if (IS_ERR(restart_reason)) {
			dev_warn(&pdev->dev, "failed to map IMEM restart reason\n");
			restart_reason = NULL;
		} else {
			/* Do not let a consumed bootloader request survive this boot. */
			msm_set_restart_reason(NULL);
		}
	}

	devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART,
				      128, do_msm_poweroff, NULL);

	devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF,
				      SYS_OFF_PRIO_DEFAULT, do_msm_poweroff,
				      NULL);

	return 0;
}

static const struct of_device_id of_msm_restart_match[] = {
	{ .compatible = "qcom,pshold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msm_restart_match);

static struct platform_driver msm_restart_driver = {
	.probe = msm_restart_probe,
	.driver = {
		.name = "msm-restart",
		.of_match_table = of_match_ptr(of_msm_restart_match),
	},
};
builtin_platform_driver(msm_restart_driver);

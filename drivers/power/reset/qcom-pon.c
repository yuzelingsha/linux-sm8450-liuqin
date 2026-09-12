// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-18 Linaro Limited

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reboot-mode.h>
#include <linux/regmap.h>

#define PON_SOFT_RB_SPARE		0x8f

#define PON_GEN3_PS_HOLD_RST_CTL		0x52
#define PON_GEN3_PS_HOLD_RST_CTL2	0x53
#define PON_GEN3_PS_HOLD_ENABLE		BIT(7)
#define PON_GEN3_PS_HOLD_TYPE_MASK	GENMASK(3, 0)
#define PON_GEN3_PS_HOLD_TYPE_WARM_RESET	0x1

#define GEN1_REASON_SHIFT		2
#define GEN2_REASON_SHIFT		1

#define NO_REASON_SHIFT			0

struct qcom_pon {
	struct device *dev;
	struct regmap *regmap;
	u32 baseaddr;
	u32 pbs_baseaddr;
	struct reboot_mode_driver reboot_mode;
	struct notifier_block reboot_nb;
	long reason_shift;
};

static int qcom_pon_gen3_reboot_notify(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct qcom_pon *pon = container_of(nb, struct qcom_pon, reboot_nb);
	const char *cmd = data;
	bool reset_disabled = false;
	int enable_ret;
	int ret;

	if (action != SYS_RESTART || !cmd ||
	    (strcmp(cmd, "bootloader") && strcmp(cmd, "recovery")))
		return NOTIFY_DONE;

	ret = regmap_update_bits(pon->regmap,
				 pon->pbs_baseaddr + PON_GEN3_PS_HOLD_RST_CTL2,
				 PON_GEN3_PS_HOLD_ENABLE, 0);
	if (ret)
		goto err;
	reset_disabled = true;

	/* The vendor sequence waits for at least ten PMIC sleep-clock cycles. */
	usleep_range(500, 750);

	ret = regmap_update_bits(pon->regmap,
				 pon->pbs_baseaddr + PON_GEN3_PS_HOLD_RST_CTL,
				 PON_GEN3_PS_HOLD_TYPE_MASK,
				 PON_GEN3_PS_HOLD_TYPE_WARM_RESET);
	if (ret)
		goto restore_enable;

	ret = regmap_update_bits(pon->regmap,
				 pon->pbs_baseaddr + PON_GEN3_PS_HOLD_RST_CTL2,
				 PON_GEN3_PS_HOLD_ENABLE,
				 PON_GEN3_PS_HOLD_ENABLE);
	if (ret)
		goto err;
	reset_disabled = false;

	dev_info(pon->dev, "configured warm PS_HOLD reset for %s\n", cmd);
	return NOTIFY_DONE;

restore_enable:
	enable_ret = regmap_update_bits(pon->regmap,
					pon->pbs_baseaddr + PON_GEN3_PS_HOLD_RST_CTL2,
					PON_GEN3_PS_HOLD_ENABLE,
					PON_GEN3_PS_HOLD_ENABLE);
	if (enable_ret)
		dev_err(pon->dev,
			"failed to restore PS_HOLD reset enable for %s: %d\n",
			cmd, enable_ret);
	else
		reset_disabled = false;
err:
	dev_err(pon->dev,
		"failed to configure warm PS_HOLD reset for %s: %d%s\n",
		cmd, ret, reset_disabled ? "; reset remains disabled" : "");
	return NOTIFY_DONE;
}

static int qcom_pon_reboot_mode_write(struct reboot_mode_driver *reboot,
				    unsigned int magic)
{
	struct qcom_pon *pon = container_of
			(reboot, struct qcom_pon, reboot_mode);
	int ret;

	ret = regmap_update_bits(pon->regmap,
				 pon->baseaddr + PON_SOFT_RB_SPARE,
				 GENMASK(7, pon->reason_shift),
				 magic << pon->reason_shift);
	if (ret < 0)
		dev_err(pon->dev, "update reboot mode bits failed\n");

	return ret;
}

static int qcom_pon_probe(struct platform_device *pdev)
{
	struct qcom_pon *pon;
	long reason_shift;
	int error;

	pon = devm_kzalloc(&pdev->dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	pon->dev = &pdev->dev;

	pon->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pon->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	error = of_property_read_u32(pdev->dev.of_node, "reg",
				     &pon->baseaddr);
	if (error)
		return error;

	reason_shift = (long)of_device_get_match_data(&pdev->dev);

	if (reason_shift != NO_REASON_SHIFT) {
		pon->reboot_mode.dev = &pdev->dev;
		pon->reason_shift = reason_shift;
		pon->reboot_mode.write = qcom_pon_reboot_mode_write;
		error = devm_reboot_mode_register(&pdev->dev, &pon->reboot_mode);
		if (error) {
			dev_err(&pdev->dev, "can't register reboot mode\n");
			return error;
		}
	}

	if (of_device_is_compatible(pdev->dev.of_node, "qcom,pmk8350-pon")) {
		int pbs_index;

		pbs_index = of_property_match_string(pdev->dev.of_node,
						     "reg-names", "pbs");
		if (pbs_index < 0)
			return dev_err_probe(&pdev->dev, pbs_index,
					     "failed to find PBS PON register\n");

		error = of_property_read_u32_index(pdev->dev.of_node, "reg",
						   pbs_index,
						   &pon->pbs_baseaddr);
		if (error)
			return dev_err_probe(&pdev->dev, error,
					     "failed to read PBS PON register\n");

		pon->reboot_nb.notifier_call = qcom_pon_gen3_reboot_notify;
		pon->reboot_nb.priority = 192;
		error = devm_register_reboot_notifier(&pdev->dev,
						      &pon->reboot_nb);
		if (error)
			return dev_err_probe(&pdev->dev, error,
					     "failed to register Gen3 restart notifier\n");
	}

	platform_set_drvdata(pdev, pon);

	return devm_of_platform_populate(&pdev->dev);
}

static const struct of_device_id qcom_pon_id_table[] = {
	{ .compatible = "qcom,pm8916-pon", .data = (void *)GEN1_REASON_SHIFT },
	{ .compatible = "qcom,pm8941-pon", .data = (void *)NO_REASON_SHIFT },
	{ .compatible = "qcom,pms405-pon", .data = (void *)GEN1_REASON_SHIFT },
	{ .compatible = "qcom,pm8998-pon", .data = (void *)GEN2_REASON_SHIFT },
	{ .compatible = "qcom,pmk8350-pon", .data = (void *)GEN2_REASON_SHIFT },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_pon_id_table);

static struct platform_driver qcom_pon_driver = {
	.probe = qcom_pon_probe,
	.driver = {
		.name = "qcom-pon",
		.of_match_table = qcom_pon_id_table,
	},
};
module_platform_driver(qcom_pon_driver);

MODULE_DESCRIPTION("Qualcomm Power On driver");
MODULE_LICENSE("GPL v2");

#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Host-only regression tests for the WN8030 transport and power model."""

import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[4]
DRIVER = (ROOT / "drivers/hid/hid-nanosic.c").read_text()
BINDING = (
    ROOT / "Documentation/devicetree/bindings/input/nanosic,wn8030.yaml"
).read_text()
DTS = (
    ROOT / "arch/arm64/boot/dts/qcom/sm8475-xiaomi-liuqin.dts"
).read_text()
I2C_CORE = (ROOT / "drivers/i2c/i2c-core-base.c").read_text()


def function_body(name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^;]*?\)\s*\{{", DRIVER, re.S)
    if not match:
        raise AssertionError(f"cannot isolate {name}()")
    opening = match.end() - 1
    depth = 0
    for position in range(opening, len(DRIVER)):
        if DRIVER[position] == "{":
            depth += 1
        elif DRIVER[position] == "}":
            depth -= 1
            if depth == 0:
                return DRIVER[opening + 1 : position]
    raise AssertionError(f"unterminated {name}()")


class PowerModel:
    """Small executable model of the lock-serialised driver state."""

    timeout = 20

    def __init__(self):
        self.now = 0
        self.deadline = self.timeout
        self.awake = True
        self.suspended = False
        self.pending = True
        self.data_pending = False
        self.wakeup_events = 0
        self.reads = 0

    def wake(self):
        self.awake = True
        self.deadline = self.now + self.timeout
        self.pending = True

    def idle_worker_after_lock(self):
        if self.suspended:
            self.pending = False
        elif self.now < self.deadline:
            self.pending = True
        else:
            self.awake = False
            self.pending = False

    def data_irq(self):
        if self.suspended:
            self.data_pending = True
            self.wakeup_events += 1
            return
        self.wake()
        self.reads += 1

    def suspend(self):
        self.suspended = True
        self.awake = False
        self.pending = False

    def resume(self):
        self.suspended = False
        self.wake()
        if self.data_pending:
            self.data_pending = False
            self.reads += 1


PACKETS = {
    0x02: (8, 8),
    0x05: (9, 9),
    0x06: (5, 3),
    0x19: (21, 21),
    0x22: (16, 0),
    0x23: (32, 0),
}

VERSION_COMMAND = bytes([
    0x4C, 0x32, 0x00, 0x4F, 0x30, 0x80, 0x18, 0x01, 0x00, 0x18,
]) + bytes(57)


def parse_fixture(frame: bytes):
    if len(frame) < 3 or frame[0] != 0x57 or frame[2] == 0:
        return []
    if frame[2] not in (0x39, 0x4A, 0x5B, 0x6C):
        return []

    reports = []
    pos = 3
    while pos < len(frame):
        packet = PACKETS.get(frame[pos])
        if packet is None:
            break
        wire_len, report_len = packet
        if not wire_len or report_len > wire_len or wire_len > len(frame) - pos:
            break
        if report_len:
            reports.append(frame[pos : pos + report_len])
        pos += wire_len
    return reports


def extract_version_fixture(frame: bytes):
    if len(frame) < 3 or frame[0] != 0x57:
        return None
    if frame[2] not in (0x39, 0x4A, 0x5B, 0x6C):
        return None

    pos = 3
    while pos < len(frame):
        packet = PACKETS.get(frame[pos])
        if packet is None or packet[0] > len(frame) - pos:
            return None
        if (frame[pos] == 0x23 and frame[pos + 2:pos + 5] ==
                bytes([0x18, 0x80, 0x01])):
            raw_version = frame[pos + 7:pos + 27].split(b"\x00", 1)[0]
            if not raw_version or any(byte < 0x20 or byte > 0x7E
                                      for byte in raw_version):
                return None
            return raw_version.decode("ascii")
        pos += packet[0]
    return None


def source_version_command() -> bytes:
    size_match = re.search(
        r"#define\s+NANOSIC_VERSION_COMMAND_LEN\s+(\d+)", DRIVER
    )
    array_match = re.search(
        r"static const u8 nanosic_version_command"
        r"\[NANOSIC_VERSION_COMMAND_LEN\]\s*=\s*\{(.*?)\};",
        DRIVER,
        re.S,
    )
    if not size_match or not array_match:
        raise AssertionError("cannot isolate firmware-version command")
    size = int(size_match.group(1))
    prefix = bytes(int(token, 16) for token in
                   re.findall(r"0x([0-9a-fA-F]{2})", array_match.group(1)))
    if len(prefix) > size:
        raise AssertionError("firmware-version initializer exceeds array")
    return prefix + bytes(size - len(prefix))


def version_query_recovery_fixture(write_results):
    """Model the bounded write/reset portion of an on-demand query."""
    reset_attempted = False
    resets = 0

    for attempt, result in enumerate(write_results):
        retries_left = attempt + 1 < len(write_results)
        if (result in ("ENXIO", "EIO", "ETIMEDOUT") and
                not reset_attempted and retries_left):
            resets += 1
            reset_attempted = True
            continue
        if result == "OK":
            return True, resets, attempt + 1

    return False, resets, len(write_results)


class NanosicProtocolFixtures(unittest.TestCase):
    def test_keyboard_and_consumer_use_wire_lengths(self):
        keyboard = bytes([0x05]) + bytes(8)
        consumer_wire = bytes([0x06, 1, 2, 0xAA, 0xBB])
        frame = bytes([0x57, 7, 0x39]) + keyboard + consumer_wire
        self.assertEqual(parse_fixture(frame), [keyboard, consumer_wire[:3]])

    def test_short_header_and_short_report_are_rejected(self):
        self.assertEqual(parse_fixture(b"\x57\x00"), [])
        self.assertEqual(parse_fixture(bytes([0x57, 0, 0x39, 0x05, 0])), [])

    def test_unknown_report_ends_frame_without_resynchronising(self):
        frame = bytes([0x57, 0, 0x39, 0x24, 0x05]) + bytes(8)
        self.assertEqual(parse_fixture(frame), [])

    def test_vendor_firmware_command_is_byte_exact_and_zero_padded(self):
        self.assertEqual(len(VERSION_COMMAND), 67)
        self.assertEqual(source_version_command(), VERSION_COMMAND)

    def test_vendor_version_report_is_extracted_without_hid_dependency(self):
        report = bytearray(32)
        report[0] = 0x23
        report[2:5] = bytes([0x18, 0x80, 0x01])
        report[7:18] = b"WN8030-1.2\x00"
        frame = bytes([0x57, 4, 0x39]) + bytes(report) + bytes(33)
        self.assertEqual(extract_version_fixture(frame), "WN8030-1.2")

    def test_version_report_rejects_wrong_direction_and_control_bytes(self):
        report = bytearray(32)
        report[0] = 0x23
        report[2:5] = bytes([0x80, 0x18, 0x01])
        report[7:11] = b"bad\x00"
        frame = bytes([0x57, 4, 0x39]) + bytes(report) + bytes(33)
        self.assertIsNone(extract_version_fixture(frame))

        report[2:5] = bytes([0x18, 0x80, 0x01])
        report[7:11] = b"a\x01b\x00"
        frame = bytes([0x57, 4, 0x39]) + bytes(report) + bytes(33)
        self.assertIsNone(extract_version_fixture(frame))

    def test_version_write_timeout_gets_one_reset_then_retries(self):
        self.assertEqual(
            version_query_recovery_fixture(["ETIMEDOUT", "OK"]),
            (True, 1, 2),
        )
        self.assertEqual(
            version_query_recovery_fixture(
                ["ETIMEDOUT", "ETIMEDOUT", "ETIMEDOUT"]
            ),
            (False, 1, 3),
        )


class NanosicPowerInterleavings(unittest.TestCase):
    def test_stale_idle_worker_cannot_undo_concurrent_data_wake(self):
        model = PowerModel()
        model.now = model.deadline
        # The worker has started but is waiting for the lock. The IRQ wins it.
        model.data_irq()
        model.idle_worker_after_lock()
        self.assertTrue(model.awake)
        self.assertTrue(model.pending)
        self.assertEqual(model.deadline, model.now + model.timeout)

    def test_irq_during_suspend_only_requests_resume(self):
        model = PowerModel()
        model.suspend()
        model.data_irq()
        self.assertFalse(model.awake)
        self.assertFalse(model.pending)
        self.assertEqual(model.reads, 0)
        self.assertEqual(model.wakeup_events, 1)
        model.resume()
        self.assertTrue(model.awake)
        self.assertTrue(model.pending)
        self.assertEqual(model.reads, 1)
        self.assertFalse(model.data_pending)


class NanosicSourceContracts(unittest.TestCase):
    def test_idle_worker_rechecks_deadline_and_suspend_state(self):
        body = function_body("nanosic_idle_work")
        self.assertIn("if (nano->suspended)", body)
        self.assertIn("time_before(jiffies, nano->wake_deadline)", body)
        self.assertLess(body.index("time_before"), body.index("wake_gpio, 0"))

    def test_suspend_blocks_rearm_before_cancelling_work(self):
        body = function_body("nanosic_suspend")
        self.assertLess(body.index("WRITE_ONCE(nano->suspended, true)"),
                        body.index("cancel_delayed_work_sync"))
        self.assertLess(body.index("wake_gpio, 0"),
                        body.index("cancel_delayed_work_sync"))
        self.assertIn("goto restore_runtime", body)

    def test_driver_and_i2c_core_do_not_double_claim_dedicated_wake_irq(self):
        suspend = function_body("nanosic_suspend")
        resume = function_body("nanosic_resume")
        probe = function_body("nanosic_probe")
        irqs_get = function_body("nanosic_irqs_get")
        self.assertIn("enable_irq_wake(nano->data_irq)", suspend)
        self.assertIn("disable_irq_wake(nano->data_irq)", resume)
        self.assertNotIn("wakeup_irq", suspend)
        self.assertNotIn("wakeup_irq", resume)
        self.assertNotIn("nanosic-wakeup", probe)
        self.assertNotIn('fwnode_irq_get_byname(dev_fwnode(dev), "wakeup")', irqs_get)
        self.assertIn('fwnode_irq_get_byname(fwnode, "wakeup")', I2C_CORE)
        self.assertIn("dev_pm_set_dedicated_wake_irq(dev, wakeirq)", I2C_CORE)

    def test_normal_data_irq_has_no_unconditional_vendor_delay(self):
        body = function_body("nanosic_receive_frame")
        first_read = body.index("ret = nanosic_read_frame")
        retry_sleep = body.index("usleep_range")
        self.assertLess(first_read, retry_sleep)
        self.assertIn("if (ret)", body[first_read:retry_sleep])
        self.assertIn("NANOSIC_I2C_RETRY_US", body)
        self.assertNotIn("mdelay", body)

    def test_transport_identity_and_frame_lengths_remain_defensive(self):
        self.assertNotIn("hid->driver_data", DRIVER)
        parse = function_body("nanosic_parse_frame")
        self.assertIn("len < NANOSIC_FRAME_HDR_LEN", parse)
        self.assertIn("pkt->wire_len > left", parse)
        self.assertIn("pkt->report_len > pkt->wire_len", parse)

    def test_dt_contract_has_two_irqs_and_soc_regulator(self):
        self.assertEqual(BINDING.count("minItems: 2"), 2)
        self.assertRegex(DTS, r"&soc\s*\{[\s\S]*?keyboard_vdd_regulator:")
        keyboard = DTS.split("keyboard@4c {", 1)[1].split("\n\t};", 1)[0]
        self.assertNotIn("hall-", keyboard.replace("hall-*-gpios", ""))
        self.assertIn('interrupt-names = "data", "wakeup";', keyboard)
        self.assertIn("wakeup-source;", keyboard)
        self.assertRegex(keyboard,
                         r"interrupts-extended\s*=\s*<&tlmm 50[^>]*>,\s*"
                         r"<&tlmm 46[^>]*>;")

    def test_diagnostics_are_on_demand_bounded_and_lock_serialised(self):
        show = function_body("firmware_version_show")
        query = function_body("nanosic_query_firmware_version")
        write = function_body("nanosic_write_version_command")
        probe = function_body("nanosic_probe")
        self.assertIn("guard(mutex)(&nano->lock)", show)
        self.assertIn("lockdep_assert_held(&nano->lock)", query)
        self.assertIn("attempt < NANOSIC_VERSION_RETRIES", query)
        self.assertIn("nanosic_read_frame(nano, NANOSIC_I2C_VERSION_READ)",
                      query)
        self.assertIn("NANOSIC_I2C_VERSION_WRITE", write)
        self.assertNotIn("nanosic_query_firmware_version", probe)
        self.assertNotIn("nanosic_write_version_command", probe)

    def test_version_transport_failure_gets_one_vendor_exact_reset_recovery(self):
        query = function_body("nanosic_query_firmware_version")
        reset = function_body("nanosic_reset_bridge")
        needs_reset = function_body("nanosic_version_error_needs_reset")
        receive = function_body("nanosic_receive_frame")
        self.assertIn("nanosic_version_error_needs_reset(ret)", query)
        self.assertIn(
            "ret == -ENXIO || ret == -EIO || ret == -ETIMEDOUT",
            needs_reset,
        )
        self.assertNotIn("-EPROTO", needs_reset)
        self.assertNotIn("-EAGAIN", needs_reset)
        self.assertIn("!reset_attempted", query)
        self.assertEqual(query.count("nanosic_reset_bridge(nano)"), 1)
        self.assertLess(reset.index("reset_gpio, 1"),
                        reset.index("NANOSIC_RESET_ASSERT_MS"))
        self.assertLess(reset.index("NANOSIC_RESET_ASSERT_MS"),
                        reset.index("reset_gpio, 0"))
        self.assertLess(reset.index("reset_gpio, 0"),
                        reset.index("NANOSIC_RESET_READY_MS"))
        self.assertNotIn("nanosic_reset_bridge", receive)

    def test_health_counters_cover_transport_and_reports(self):
        health = function_body("bridge_health_show")
        transfer = function_body("nanosic_i2c_transfer")
        parse = function_body("nanosic_parse_frame")
        irq = function_body("nanosic_data_irq")
        for field in ("irq", "i2c_transfers", "i2c_errors", "bad_magic",
                      "valid_reports", "last_i2c_errno",
                      "last_i2c_operation", "last_i2c_error_transfer",
                      "reset_recoveries"):
            self.assertIn(field, health)
        self.assertIn("i2c_transfer_count++", transfer)
        self.assertIn("i2c_error_count++", transfer)
        self.assertIn("last_i2c_errno = ret", transfer)
        self.assertIn("last_i2c_operation = operation", transfer)
        self.assertIn("bad_magic_count++", parse)
        self.assertIn("valid_report_count++", parse)
        self.assertIn("irq_count++", irq)

    def test_read_only_diagnostics_are_removed_before_devm_teardown(self):
        remove = function_body("nanosic_remove")
        self.assertEqual(DRIVER.count("DEVICE_ATTR_RO("), 2)
        self.assertNotRegex(DRIVER, r"DEVICE_ATTR_(?:RW|WO)\(")
        self.assertIn("sysfs_remove_group", remove)
        self.assertIn(".remove = nanosic_remove", DRIVER)

    def test_no_private_control_or_firmware_loading_surface(self):
        code = re.sub(r"/\*.*?\*/", "", DRIVER, flags=re.S)
        self.assertNotIn("misc_register", code)
        self.assertNotIn("request_firmware", code)
        self.assertNotIn("DEVICE_ATTR_RW", code)
        self.assertNotIn("DEVICE_ATTR_WO", code)


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""Reject a claimed CUDA run whose policy parameters are actually on CPU."""

from collections import namedtuple
from pathlib import Path
import sys
import types
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ood_suite.runtime import policy_parameter_device


class Device(namedtuple("DeviceBase", "type index")):
    __slots__ = ()

    def __str__(self):
        return self.type if self.index is None else "{}:{}".format(self.type, self.index)


def model(*devices):
    return types.SimpleNamespace(parameters=lambda: iter(
        types.SimpleNamespace(device=device) for device in devices))


class DeviceTests(unittest.TestCase):
    def test_cuda_report_uses_actual_device_index(self):
        actual = Device("cuda", 1)
        models = {name: model(actual, actual) for name in ("left", "right", "articulation")}
        self.assertEqual(policy_parameter_device(models, Device("cuda", None)), actual)

    def test_cpu_parameters_cannot_be_reported_as_cuda(self):
        with self.assertRaisesRegex(RuntimeError, "requested cuda, actual cpu"):
            policy_parameter_device({"left": model(Device("cpu", None))}, Device("cuda", None))

    def test_mixed_devices_or_wrong_gpu_are_rejected(self):
        cpu, gpu = Device("cpu", None), Device("cuda", 0)
        with self.assertRaisesRegex(RuntimeError, "one device"):
            policy_parameter_device({"left": model(gpu), "right": model(cpu)}, gpu)
        with self.assertRaisesRegex(RuntimeError, "device mismatch"):
            policy_parameter_device({"left": model(gpu)}, Device("cuda", 1))


if __name__ == "__main__":
    unittest.main()

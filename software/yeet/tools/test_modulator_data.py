#!/usr/bin/env python3
"""Test ModulatorData attributes for telemetry protocol."""

import sys
sys.path.insert(0, 'monitor_client')

from core.telemetry_protocol import ModulatorData

# Test that ModulatorData has both firing angle fields
mod = ModulatorData(
    is_enabled=True,
    firing_angle_input_pu=0.5,
    firing_angle_output_pu=0.7,
    force_commutation_ack=False,
    is_requesting_pi_reset=False
)

print("ModulatorData test:")
print(f"  is_enabled: {mod.is_enabled}")
print(f"  firing_angle_input_pu: {mod.firing_angle_input_pu}")
print(f"  firing_angle_output_pu: {mod.firing_angle_output_pu}")
print(f"  force_commutation_ack: {mod.force_commutation_ack}")
print(f"  is_requesting_pi_reset: {mod.is_requesting_pi_reset}")

# Verify attributes exist
try:
    assert hasattr(mod, 'firing_angle_input_pu'), "Missing firing_angle_input_pu"
    assert hasattr(mod, 'firing_angle_output_pu'), "Missing firing_angle_output_pu"
    print("\n✅ All required ModulatorData attributes exist!")
except AssertionError as e:
    print(f"\n❌ {e}")
    sys.exit(1)

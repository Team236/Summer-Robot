package com.team236.lib.subsystems.cancoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

/**
 * The {@code CanCoderConfig} stores hardware identifiers and configuration profiles required to
 * initialize a CANcoder device.
 */
public class CanCoderConfig {
    public CanDeviceId canId;
    public CANcoderConfiguration config = new CANcoderConfiguration();
}

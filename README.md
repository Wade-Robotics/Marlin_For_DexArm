# Wade Robotics Marlin

[![Build & Release](https://github.com/Wade-Robotics/Marlin_For_DexArm/actions/workflows/build-firmware.yml/badge.svg)](https://github.com/Wade-Robotics/Marlin_For_DexArm/actions/workflows/build-firmware.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

Custom firmware for DexArm robotic arms, maintained by Wade Robotics.

## Features

- 🤖 **Real-time Position Reporting** - M114 R support for live position feedback
- ⚡ **Over-the-Air Updates** - Flash firmware directly from Wade Robotics Studio
- 🔧 **DexArm Optimizations** - Tuned for SCARA kinematics and pick-and-place operations
- 🛡️ **Safe Bootloader** - Software-triggered updates with rollback protection

## Quick Start

### Option 1: Wade Robotics Studio (Recommended)

1. Download [Wade Robotics Studio](https://github.com/Wade-Robotics/Wade-Robotics-Studio/releases)
2. Connect your DexArm
3. Go to **Settings → Firmware Update**
4. Click "Check for Updates" and follow the prompts

### Option 2: Download Pre-built Firmware

Download the latest `firmware.bin` from [Releases](https://github.com/Wade-Robotics/Marlin_For_DexArm/releases).

### Option 3: Build from Source

```bash
# Install PlatformIO
pip install platformio

# Clone and build
git clone https://github.com/Wade-Robotics/Marlin_For_DexArm.git
cd Marlin_For_DexArm
pio run -e DEXARM_V3_2
```

Firmware will be at `.pio/build/DEXARM_V3_2/firmware.bin`

## Flashing Firmware

### Via Wade Robotics Studio
The easiest method - handles everything automatically including bootloader entry.

### Manual Method
1. Connect to DexArm via serial terminal (115200 baud)
2. Send `M2002` then `M2003` to enter bootloader
3. Use YMODEM protocol to transfer `firmware.bin`
4. Send `3` to execute the new firmware

## Supported Hardware

| Board | MCU | Status |
|-------|-----|--------|
| DexArm V3.2 | STM32F407ZGT6 | ✅ Primary |
| DexArm V3.1 | STM32F407ZGT6 | ✅ Supported |

## Configuration

Key configuration files:
- `Marlin/Configuration.h` - Basic settings
- `Marlin/Configuration_adv.h` - Advanced features (M114_REALTIME enabled here)

## Support

For help with Wade Robotics Marlin firmware:

- 📋 [Open an Issue](https://github.com/Wade-Robotics/Marlin_For_DexArm/issues) - Bug reports and feature requests
- 💬 [Discussions](https://github.com/Wade-Robotics/Marlin_For_DexArm/discussions) - Questions and community help
- 📧 Contact Wade Robotics directly for commercial support

## Contributing

1. Fork this repository
2. Create a feature branch from `DexArm_Dev`
3. Submit a Pull Request to `main`

Builds are automatically triggered on push. Merges to `main` create new releases.

## License

This project is licensed under the **GNU General Public License v3.0** - see the [LICENSE](LICENSE) file for details.

This firmware is based on the open-source Marlin project. Per the GPL license, the source code is provided and must remain open.

---

<p align="center">
  <strong>Wade Robotics</strong><br>
  Making robotics accessible
</p>

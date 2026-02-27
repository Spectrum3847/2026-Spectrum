# 2026-Spectrum

Robot code for Team Spectrum 3847's 2026 season.

## Quick Start

### Prerequisites
- Java 17
- WPILib 2026
- Git

### Setup
1. Clone the repo:
   ```bash
   git clone https://github.com/Spectrum3847/2026-Spectrum.git
   ```
2. Open the folder in WPILib VS Code.

## Project Structure
```text
├── src/
│   └── main/
│       ├── java/
│       │   └── frc/
│       │       ├── robot/              # main robot application code
│       │       │   ├── auton/          # autonomous routines and commands
│       │       │   ├── configs/        # robot config selection/constants
│       │       │   ├── swerve/
│       │       │   ├── vision/
│       │       │   ├── launcher/
│       │       │   └── ...             # other subsystems
│       │       ├── spectrumLib/        # shared Spectrum utilities
│       │       └── rebuilt/            # field/targeting helper logic
│       └── deploy/                     # files copied to RoboRIO
│           └── pathplanner/
├── vendordeps/                         # vendor dependency JSONs
└── build.gradle                        # GradleRIO project config
```

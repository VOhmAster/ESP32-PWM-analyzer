# PWM Signal Analyzer

## Description
This project demonstrates a method of analyzing PWM signals with a focus on graphical, real-time visualization of the waveform. The main goal is to educate users on PWM analysis and provide a visual representation of signals as they change in real-time.

## Features
- Real-time graphical display of PWM waveforms.
- Adjustable frequency zoom levels for different ranges (300Hz - 2MHz).
- Simple user interface with interactive menu options.
- Supports multiple PWM channels (CH1, CH2).
- Visual glitch detection in the signals.
- Freeze mode to stop updates for stable readings.

## Additional Feature
The onboard RGB LED acts as a status indicator, adapting its color and behavior (pulsing or blinking) according to the current channel activity, frequency ranges, and duty cycle states.

## Hardware  
This project is built on an **ESP32-S3** and uses the following components:  
- **Two PWM signal inputs** (with optional RC delay network and 10k pull-down to GND).  
- **SSD1322 OLED display** (SPI interface).  
- **Rotary encoder** to adjust the update interval.  
- **Button interface** for menu navigation (3 dedicated buttons).  
- **RGB LED** as a status indicator (color and blink patterns show channel activity and duty states).  
- **220V → 5V (≥700 mA) power supply** power supply module.  
- **STDN-3A24-ADJ step-down module** for voltage regulation.  

### Pin Configuration
| Name             | Pin | Description              |
|------------------|-----|--------------------------|
| `PWM1_PIN`       | 45  | Channel 1 PWM input      |
| `PWM2_PIN`       | 46  | Channel 2 PWM input      |
| `ENC_A`          | 17  | Rotary encoder pin A     |
| `ENC_B`          | 18  | Rotary encoder pin B     |
| `ENC_SW`         | 9   | Rotary encoder switch    |
| `RED_PIN`        | 37  | Red LED for status       |
| `GREEN_PIN`      | 36  | Green LED for status     |
| `BLUE_PIN`       | 38  | Blue LED for status      |
| `SPI_MOSI_PIN`   | 11  | OLED MOSI                |
| `SPI_SCK_PIN`    | 12  | OLED SCK                 |
| `SPI_CS_PIN`     | 10  | OLED CS                  |
| `SPI_RES_PIN`    | 13  | OLED Reset               |
| `SPI_DC_PIN`     | 14  | OLED DC                  |
| `BTN_MENU1`      | 20  | Main menu                |
| `BTN_MENU2`      | 21  | Detailed data            |
| `BTN_MENU3`      | 47  | Graphical view           |

### Important Notes
- An RC circuit is used for delayed start, and a 10kΩ pull-down resistor is placed on the PWM input pins to ensure proper signal reading.

## Installation
1. Clone the repository to your local machine:
    ```bash
    git clone https://github.com/VOhmAster/ESP32-PWM-analyzer.git
    ```
2. Open the project in Arduino IDE.
3. Upload the code to your ESP32 device.
4. Connect the PWM signal sources to the appropriate pins.

## Usage
- **Encoder Rotation**: Adjust the update interval between 50ms and 2000ms or toggle Freeze mode.
- **Encoder Button**: Change the display unit (Hz → kHz → MHz → Hz).
- **Display Navigation**: Use the dedicated buttons to switch between:
- **Main Display**: Displays real-time PWM signal data.
- **Detailed Data**: Shows more in-depth measurements such as high and low times.
- **Graph View**: Displays the waveform graph of the signals.
    
## Expansion
This project can be expanded by adding more hardware, allowing for higher-frequency PWM analysis and additional features like:
- Signal filtering and smoothing.
- Support for higher frequency ranges beyond the default setup.

## Files Included
- `PWM_analyzer.ino`: Main Arduino code.
- `Schematics/`: EasyEDA design files.
- `3D_Models/`: STL files for 3D printing the enclosure.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


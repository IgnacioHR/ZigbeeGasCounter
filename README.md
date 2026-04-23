# Zigbee GAS counter

This project is a DIY Zigbee-enabled gas meter that measures and tracks gas consumption. The device integrates with Zigbee2MQTT and Home Assistant, providing accurate readings for energy management and analysis. It’s designed to be battery-powered and operate autonomously, addressing common challenges of similar solutions.

## Features

- Real-time gas consumption tracking in cubic meters (m³).
- Real-time gas flow tracking in cubit meters per hour (m³/h)
- Zigbee communication for seamless integration with Zigbee2MQTT and Home Assistant.
- Battery-powered for installation flexibility.
- Double battery connector to allow battery replacement without counting interruption.
- Built-in counter to prevent data loss if the network or Home Assistant goes offline.
- Easy setup and minimal hardware requirements.
- OTA firmware updatable.
- Device button with rich functionality 

## Screenshots

Below are some screenshots of the device integrated into Zigbee2MQTT:

![Main device](images/Zigbee2MQTT-1.png)
![Exposes](images/Zigbee2MQTT-2.png)
![Bind](images/Zigbee2MQTT-3.png)
![Reporting](images/Zigbee2MQTT-4.png)
![Clusters](images/Zigbee2MQTT-5.png)

## Why Zigbee for DIY Projects?

Zigbee is a low-power, reliable wireless protocol ideal for IoT devices. Unlike Wi-Fi, Zigbee consumes minimal energy, making it perfect for battery-operated devices. While commercial Zigbee gas meters aren’t readily available, this project demonstrates how to build one from scratch using affordable components and the ESP32-C6 module.

## How It Works

1. The gas meter’s rotating wheel has a built-in magnet.
1. A magnetic sensor detects each full rotation of the wheel, corresponding to a predefined volume of gas.
1. The ESP32-C6 processes the data, maintaining a cumulative counter (currentSummDelivered) and calculating instantaneous demand (instantaneousDemand) in m³/h.
1. These metrics are sent via Zigbee to Zigbee2MQTT, which forwards them to Home Assistant.

## Motivation

Before this project, I used a modified door sensor to monitor gas usage. While functional, it had several drawbacks:

1. **Short battery life**: The door sensor required frequent battery replacements.
2. **Data loss**: If Zigbee2MQTT or Home Assistant went offline, pulse data was lost, leading to discrepancies.
3. **Complexity**: Managing counters and automations in Home Assistant was cumbersome.

After searching for commercial Zigbee gas meters and finding none, I decided to create a custom device tailored to my needs.

![Old device](images/small_gas_counter.png)

## Device button functionality

The device button has implemented gesture recognition. The gestures signaled are PRESS, RELEASE, SINGLE_CLICK, DOUBLE_CLICK and HOLD. In the code there are more gestures such as NONE (No current gesture set) and UNKNOWN_CLICK (set when failing to recognize any of the properly defined gestures).

Once a gesture is recognized, the functionality attached, if any, is raised:

- PRESS: The button is pressed. The internal led is switched on. Zigbee radio is enabled.
- SINGLE_CLICK: The button is pressed and released in less than 400ms and not pressed again in 400ms. Force report of the Current Summation Delivered, Battery percentage, Status and extended Status registers. The Battery voltage is updated internally at this time. Note it is not reported because it is not possible in the Zigbee specifications.
- DOUBLE_CLICK: The button is pressed and releasses in less than 400ms, then pressed again in less than 400ms and released in less than 400ms and not pressed in less than 400ms. The device is restarted (counter is not reset)
- HOLD: The button is keep pressed for 3 seconds. The device leaves the network and starts comission process to find another network.
- NONE: The led is turned off, note the led is turned off for multiple other causes such as entering sleep mode etc.

## Project Status

April 23rd 2026 - First results of battery total duration in Light-Sleep and Deep-Sleep configurations. See below. OTA issues are now fixed
March 2nd 2026 - Now testing in LIGHT_SLEEP mode.This project is in testing phase. It is now deployed for real gas consumption measurement at home.

Remaining issues are:

* (Voltage should be transmitted now, but this is not fully tested) Voltage value is not transmitted to the server as part of the value report (in zigbee terms) I actually I know i can force read the value from the server but I don't know how to push the value from the device to the server.

* (This is now a lesson learned, not an issue) The initial decission about using a 2S LiPo battery causes more troubles than benefits. I finally decided to go 1S LiPo battery and use the internal Seed-Studio 3v switching regulator and charger module.

* (April 2026 this is now fixed) OTA is no longer functional at all. I know about changes around OTA functionality in zigbee2mqtt and more testing is required.

### What is missing and how you can help

See the **Project Status** section above for specific issues. If you can help I'll be more than happy.

For discussions and contributions, please join the ongoing thread on the Home Assistant Community: 👉 [Zigbee Gas Counter](https://community.home-assistant.io/t/zigbee-gas-counter/833557)

One of my main concerns is **power consumption and battery life**. The software includes battery status measuring and reporting.
#### First comparison analysis of the battery duration under Light-Sleep and Deep-Sleep configurations
![Main device](images/BatteryConsumptionRealLightSleepVSDeepSleep.png)

I've recently seen [this video](https://youtu.be/KE7bOYCYETM?si=DaY46TMaGhLbYbze) from @"Microamp Home" on YT and it looks worth testing using one of the devices from Nordic Semiconductors.

Currently, the testing prototype runs on a Seed Studio ESP32-C6, but the long-term goal is to use only the **ESP32-C6 chip**, eliminating unnecessary components. To achieve this, I will need help reviewing schematics and PCB designs, as **I’m not an expert in hardware design—just an enthusiast**. If you have experience in this area, your input would be greatly appreciated.


## Getting Started

### Required Hardware

To build this project, you’ll need:

- **Seed Studio ESP32-C6** or a compatible development board.
- **Magnetic reed switch** for pulse detection.
- **10kΩ resistors** (pull-down).
- **3.6 LiPo battery**. TBD
- Optional: Custom 3D-printed enclosure for the hardware.
- Optional: External 2.4 Ghz antena.

### Software Requirements

- **ESP-IDF (Espressif IoT Development Framework)**: [Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
- **Zigbee2MQTT**: [Setup Guide](https://www.zigbee2mqtt.io/)
- **Home Assistant**: [Official Website](https://www.home-assistant.io/)

## **Setup Instructions**

### **1. Clone the Repository**

First, download the project code from the GitHub repository:

```bash
git clone https://github.com/IgnacioHR/ZigbeeGasCounter.git
cd ZigbeeGasCounter
```

### **2. Configure the target chip**

Set the corrent target for the ESP32-C6:

```bash
idf.py --preview set-target ESP32C6
```

### **3. Install Dependencies**

Ensure ESP-IDF and its tools are installed on your machine.

### **4. Erase Flash Memory**

Clear previous configurations:

```bash
idf.py -p PORT erase-flash
```

### **4. Build and Flash**

Compile the code and flash it to the ESP32-C6 board:

```bash
idf.py build
idf.py -p PORT flash
idf.py -p PORT monitor
```

### **5. Hardware configuration in the source code

Edit the file *esp_zb_gas_meter.h* and head to the section **FEATURES CONFIGURATION** and the section **HARDWARE CONFIGURATION** you can change the chip pins to be used or leave them as they are. You can also decide what features shall be enabled depending, for example, in the power source for your specific unig.

## Device operation and main button actions

The device has just one button, but multiple gestures are recognized

- Button press: When the main button is pressed, if the device was is sleep mode, zigbee radio is enabled. Note, button press is the first gesture of a possible list of gestures recognized
- Button release: No specific action is associated to this gesture. The time passed between the button press and release is used to determine more gestures as defined below.
- Single click: A single click is detected when the time between press and release is shorter than 250ms. When single click gesture is detected, the device shall refresh data on the server side, including the battery status and voltage value (note the voltage value is not refresh in the server side automatically, there is a note on this topic), device error conditions are reset and reevaluated again.
- Double click: A double click is detected when the time between press and release is sorter then 250ms and the button cycle is executed twice. When this gesture is recognized the device is reset.
- Unknown click: This happens when the user triple-click or press-click-press in a way that does not fit in a single or double click. No action is associated to this gesture.
- Long press: When the button is pressed and not released for 3 seconds, the device is ordered to leave the network and start the commision process to join an open network again.

### NVS (Non-Volatile Storage)

The cumulative gas consumption (currentSummDelivered) is stored in the device’s NVS to prevent data loss. The counter is automatically restored upon reboot.

### Battery Optimization

The device uses sleep modes to conserve energy. It wakes up:

- When gas consumption is detected.
- Periodically (e.g., every hour) to send reports.

## Customization

You can adapt this project for other pulse-based meters (e.g., water or electricity) by modifying the code to reflect the appropriate measurement units and formulas.

## Manufacturer information

This project includes a custom manufacturer name and code:

- Name: “MICASA” – A playful mix of “mi casa” (my home) in Spanish.
- Code: 0x8888 – Chosen arbitrarily for this DIY project.

## SDK Resources

- [ESP Zigbee SDK Docs](https://docs.espressif.com/projects/esp-zigbee-sdk)
- [ESP Zigbee SDK Repo](https://github.com/espressif/esp-zigbee-sdk)

## Contributing

If you want to contribute or improve this project, feel free to fork the repository and open a pull request. Suggestions are welcome!

## TODO

Here are the planned improvements:

1. Design a custom PCB for the ESP32-C6, connectors and resistors.
2. Create a 3D-printed enclosure for secure installation.
3. Light-Sleep mode, the chip is not entering Light-Sleep

## Acknowledgments

By improving the content structure and emphasizing actionable steps, this README not only serves as a guide for your project but also inspires others to create their own Zigbee devices.

## License

[License](https://creativecommons.org/licenses/by-nc-sa/4.0/deed.en)

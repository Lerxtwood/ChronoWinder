# watch_winder
3D printed watch winder, using 28BYJ and ESP32C3

## Building

This project uses PlatformIO with the standard `src/main.cpp` layout.

```powershell
platformio run -e esp32-c3-devkitm-1
```

The PlatformIO configuration installs the `AccelStepper` library automatically.

## Configuration

On first boot, or if Wi-Fi cannot connect, the firmware starts a setup access point named `WatchWinder-Setup`.
Connect to that network and open:

```text
http://192.168.4.1/
```

The configuration page lets you set:

- Wi-Fi SSID
- Wi-Fi password
- Rotation direction: clockwise, counter clockwise, or both
- Turns per day (TPD)
- Active RPM, default `6`
- Turns per burst, default `10`
- Rest minutes between bursts, default `5`
- Steps per full rotation
- Centering speed for long-touch position 0 calibration
- Whether the motor is disabled during rest
- Whether manual starts count toward the daily TPD limit
- Automatic daily winding

The page also includes quick action buttons:

- `Run one turn` starts a one-turn test without consuming the daily TPD counter.
- `Return to center` moves to the nearest position 0 and stops.
- `Reset today's turns` clears the completed-turn counter for the current day.
- `Pause for today` stops the winder and prevents automatic daily restarts until the next daily reset.

Three watch profile slots can save and restore the winding settings for different watches. Profiles store direction, TPD, RPM, turns per burst, rest minutes, and steps per full rotation. They do not change Wi-Fi settings.

When rotation direction is set to `Both`, each new burst alternates direction after the rest period. This starts with a clockwise burst, then counter clockwise, then repeats.

After saving settings, the ESP32-C3 restarts automatically so Wi-Fi and winding behavior are applied. The serial log prints the config page IP address after connection.

Automatic daily winding uses local Mountain time and requires Wi-Fi so the ESP32-C3 can sync time with NTP. When enabled, the winder runs in slow bursts until the daily TPD target is reached. On the next local day, the daily counter resets and automatic winding can start again.

Touch-started manual runs use the same burst/rest behavior as automatic runs. By default they also use the daily TPD counter, but that can be changed on the configuration page. Touch again to stop a manual run. Once the configured TPD target has been completed for the local day, automatic starts wait until the next daily reset.

## Firmware Updates

After a firmware version with web updates has been installed over USB, future updates can be installed from the configuration site.

Build the firmware:

```powershell
platformio run -e esp32-c3-devkitm-1
```

Open the winder's configuration page, choose `Firmware update`, and upload:

```text
.pio/build/esp32-c3-devkitm-1/firmware.bin
```

The ESP32-C3 restarts after a successful update. Keep USB upload available as a fallback if Wi-Fi or the web server is not working.

### Wiring Schematic
![SCHEMATIC](https://github.com/user-attachments/assets/371eaa53-4a67-41c6-b905-31f8d02fc284)

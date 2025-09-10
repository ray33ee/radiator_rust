# Intro

# Flash

# Config

On first boot, wifi connection will fail as there are no valid credential stored, so the device will enter a wifi warning (flashing 5 times).

When this happens, the device will automatically enter USB DCD mode, using your favorite terminal app connect to the USB serial port. 

Press enter, this should promt you to enter the ssid. Type it, press enter again and enter the password, then press enter. 

You should see a 'Done.' message which means that the credentials are stored. 

If you made a mistake or need to change the WiFi information again, simply hold the MODE button during boot and the device will enter USB CDC mode, then repeat the above steps again.

# Web server

Once the device connects to the WiFi, it will host a web server which can be used to control the device from any browser on the same network. Just type the IP address into your web browser on the same network and it will show!

# Calibrate

On first use, the device will quickly flash yellow. This means that the motor needs calibrating.

Calibrate by retracting the motor (either by pressing the MODE button or pressing Calibrate Pull in web server) until it stalls. Then unlock and zero (either by holding the MODE button and releasing when it is cyan or pressing the Unlock & Zero button in web server). This teaches the motor where the 0 position is.

Now the device is fully retracted, it can be safely installed onto the valve.

Next the max position needs to be calibrated. This can only be done my Calibrate Push in the web server by incremeents. Keep track of the total pushed until stalling from the 0 position. The total movement from 0 until stall is the max position. Enter the max position into the web server and press the Set Max button.

At any time you can manually move the motor and set the max position but the device MUST be in calibrate mode.

If the motor is interrupted during movement, there is a chance that a recalibration is needed. This does not affect the max position however. Users only need to retract until stall, then unlock and zero, then reset.

Once the following conditions have been met:

- Device is zeroed and unlocked
- Max position is larger than 0
- Max position is less than absolute max (50)

the user can safely leave calibrate mode by pressing cancel in the web server or cancelling via the MODE button.

If during normal use the aformentioned three conditions are not met when motor movement starts, the device will enter calibrate mode. The web server gives the reason for entering calibrate mode, so this can be used to correct any issues.

# Safe mode

When installing, uninstalling or powering down the valve in the future, the user should out the device in safe mode. This will ensure that a) the motor is fully retracted when installing and b) that the device is not powered down during motor movement and c) locks the motor so that on startup the device is not moved unexpectedly. 

On startup the device will enter calibrate mode (due to being locked) to protect the motor. When it is safe, the user can unlock the motor, cancel out of calibrate mode and continue using.

# Mode button

# Status

The RGB LED indicates the status of the device:

## Startup

- Solid Cyan: Initialising peripherals
- Rapid flashing cyan: Connecting to the internet
- Rapid flashing cyan: Connecting to time server
- Solid green: Startup sequence complete

## Normal

- Slow fading orange: Temperature is being monitored and the valve opened/closed according to the thermostat temperature
- Slow fading blue: Radiator valve is off 
- Fast fading orange: Boost mode, radiator valve is on until a period of time expires
- Rainbow: RAINBOW! Please note that valve position and schedule are ignored in this mode
- Slow Yellow/Green fading: Descale mode. Opens and closes valve.

## Maintainence 

- Slow fading white: Safe mode
- Rapid fading yellow: Calibrate mode

## Error

- Flashing red: Fault
- Fading yellow and flashing yellow: Warning

# Descale

When the device is not in use, such as during the summer months, a special mode Descale is created that can be used to engage the valve on a regular basis to descale.

# Warning

Certain events may happen that are recoverable but might need attention. These are warnings. Warnings can be ignored with the Cancel web server or a single press of the MODE button during warning. The number of flashes indicate the warning type:

- 3: Temperature sensor error
- 4: WiFi error (device defaults to OFF and can be turned on with boost via MODE button)
- 5: Time error (device detaults to off and can be controlled via MODE button. Also the time can be manually set via settings in web server)

# Fault

The device may fault in two ways

## Rust Panic

## CPU Exception

In either case the diagnostic board can be connected to show lights which indicate the state.

Also the diag board can connect via USB UART to show the error message.


# Intro

# Flash

# Config

# Web server

Once the device connects to the WiFi, it will host a web server which can be used to control the device from any browser on the same network. Just type the IP address into your web browser on the same network and it will show!

# Calibrate

On first use, the device will quickly flash yellow. This means that the motor needs calibrating.

Calibrate by retracting the motor (either by pressing the MODE button or pressing Calibrate Pull in web server) until it stalls. Then unlock and zero (either by holding the MODE button and releasing when it is cyan or pressing the Unlock & Zero button in web server). This teaches the motor where the 0 position is.

Now the device is fully retracted, it can be safely installed onto the valve.

Next the max position needs to be calibrated. This can only be done my Calibrate Push in the web server by incremeents. Keep track of the total pushed until stalling from the 0 position. The total movement from 0 until stall is the max position. Enter the max position into the web server and press the Set Max button.

At any time you can manually move the motor and set the max position but the device MUST be in calibrate mode.

If the motor is interrupted during movement, there is a chance that a recalibration is needed. This does not affect the max position however. Users only need to retract until stall, then unlock and zero, then reset.

# Safe mode

When installing, uninstalling or powering down the valve in the future, the user should out the device in safe mode. This will ensure that a) the motor is fully retracted when installing and b) that the device is correctly powered down. 

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


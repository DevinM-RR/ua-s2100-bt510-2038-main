# BT510 CT Firmware

## Overview
This firmware is a Bluetooth contact tracing application built on the Zephyr RTOS.

## Release History

### v0.3.0 - Update release for BT510-CT with battery life improvements
- Improve battery voltage measurement readings to be more accurate (performed with LED load)
- Remove non-active mode and (unused) provisioning mode (device will only use Ship mode and Active mode)
- Reduce LED patterns for better battery life
	- Removed DFU patterns (alternating short red/green blinks and 1 second orange blink)
	- Removed provisioning mode patterns (orange blinking pattern)
	- Removed non-active mode pattern (short red blink)
	- Kept power up and ship mode exit pattern of 1 second green LED blink
	- Kept low battery pattern of 3 short blinks of red LED
	- Kept factory reset pattern of 1 second red LED blink
	- Modified datalog full pattern to be 3 short blinks of green LED (changed from orange blinks)
	- Added heartbeat pattern of 1 short green LED blink
- One of the patterns between heartbeat, datalog full, and low battery, will be triggered at the scan interval (to be used with battery voltage measurement as well as indication of normal operation). These LEDs will not trigger when scanning is not on (i.e. during a connection) to reduce possibility of large current spikes. 
- Factory reset activated after 10 seconds of button press, down from 20 seconds 
- Watchdogs implemented
	- 1 minute hardware watchdog
	- Scan result monitoring (software watchdog). Forces a device reset (back into active mode) if scan result count doesn't change for 1 hour. The duration can be configured by re-purposed setting 0x13 (which defaults to 1). This software watchdog can be disabled with a value of 0. 
- Update default parameter values
	- Default RSSI threshold set to -85 (was -99)
	- Has data threshold changed to 5 (was 1)
	- Accelerometer threshold values set to 0
	- Low battery threshold set to 2500 (2.5V as a conservative estimate of low battery under load of the green or red LED)
- POF comparator enabled once device is in low battery state.
	- POF Threshold set to 1.9V
	- If a current spike causes battery voltage to droop to 1.9V or lower, device will reset into ship mode and will continue to stay in ship mode until battery is replaced. A button press will wake the device, and will show the low battery LED pattern (and then return to ship mode).  
- Added battery voltage to advertisement (data index 23 of manufacturing specific data field)
- Added software version to advertisement 
	- Major (data index 24, upper nibble - 4 MSBits)
	- Minor (data index 24, lower nibble - 4 LSBits)
	- Build (data index 25)


### v0.2.8 - Manufacturing release for BT510-CT
- Update ship mode exit to require a 5-second button press
- Update LED pattern of "off sleep" to a red LED blink
- Change button sequence of "on sleep" to a 2-second press (up from 100ms press)
- RTC timekeeping fixes implemented

### v0.2.4 - Security release for BT510-CT
- Add AES-CBC encryption of log data over SMP
- Add authentication procedure for remote devices when an AES Key is set
- Allow RTC re-synch from MG100 every hour
- Add check of device ID table and set to BLE MAC when reverting settings to default values (to prevent cases of device ID of 0xFFs)
- Add check of new rotating device ID at the rotation interval and set to BLE MAC (to prevent cases of device ID of 0xFFs)
- Added datalog test entry command to SMP ("/sys/dlog_test.cmd")
- Added JTAG read back protection

### v0.2.3 - Bug fix release for BT510-CT

- Updates to set HAS DATA flag when there are expired (from contact period) and unacked entries in the log. 
- Update advertising data to use correct Advertising Data Record Type
- Increased SMP timeout from 10 seconds to 30 seconds
- Handle unexpected disconnects by aborting any in-process datalog reads


### v0.2.0 - Bug fix release for BT510-CT

- Added full datalog threshold setting with an associated LED pattern (short orange blink every 5 seconds)
- Fix a bug relating to datalog downloads between the MG100-CT and the BT510-CT.


### v0.1.1 - HAS DATA metric update for BT510-CT

- HAS DATA metric has been updated to indicate when the record count of any device reaches a specified threshold (defined by the ID\_HAS\_DATA\_THRESHOLD setting).
- Set TX power for scanning and connections.


### v0.1.0 - Interim release for BT510-CT

- New Settings Implemented:
	- 001B: ID\_CONTACT\_PERIOD\_SEC - Period being monitored for a contact event. Default: 86400
    - 001C: ID\_DISCOVERY\_DURATION\_SEC - Time a device must be observed before tracking. Default: 300
    - 001D: ID\_TRACKING\_DURATION\_SEC - Maximum log time for each device per contact period Default: 1800
    - 001E: ID\_DISTANCE\_FILTER - Filtering algorithm selection (0: None, 1: Average RSSI). Default: 1 (Average RSSI)
    - 001F: ID\_RECORD\_TYPE - Record type to be used in the datalog (0x10: AD Tracking, 0x11: AD Tracking with "timestamp offset"). Default: 0x11
    - 0020: ID\_MAINTENANCE\_PERIOD\_SEC - Period where datalog performs maintenance actions. Default: 300
    - 0021: ID\_OVERWRITE\_FULL\_DATALOG - Overwrite oldest entries when the datalog is full or stop logging new data (not yet implemented)
    - 0022: ID\_AD\_PROTOCOL\_INTERVAL\_SEC - Interval to switch from tracking advertisement to download advertisement (experimental). Default: 0 (only advertise tracking adverts)
- Implemented new logging scheme using for “Discovery Duration” (default 5 minutes), “Tracking Duration” (default 30 minutes) and “Contact Period” (default 24 hours). A new remote device starts in a discovery state. After the remote device is “seen” for at least the “Discovery Duration”, the device will be moved to the non-volatile datalog, and data will be stored until the "Tracking Duration" is reached. After that time, the remote device will not generate new records in the datalog until the “Contact Period” expires.
- Implemented a running average filter on RSSI data (from a single scan interval).
- Implemented rotating ID used in the BLE advertisements.
- Implemented a new SMP command to trigger one of the LED blink sequences
- Implemented an SMP stats request to retrieve battery level in millivolts for reporting to the user
- Improved Settings file parsing to support sending a subset of settings, rather than requiring the full file

### v0.0.8 - Interim release for BT510-CT pre-production programming

- Implement TM pin to control UART (and whether to go into deep sleep on POR)

### v0.0.7 - Bug fix release

- Fixed datalogging bug related to a full log
- Fixed button initialization after OTA

### v0.0.6 - Initial release for BT510-CT testing

- Implements basic RSSI scanning and logging functionality
- For use with MG100-CT firmware v0.0.3 or higher

## SMP Server Compatibility
This  application supports the following mcumgr transports by default:

* Shell
* Bluetooth

``smp_svr``  enables support for the following command groups:

* ``fs_mgmt``
* ``img_mgmt``
* ``os_mgmt``
* ``stat_mgmt``

This application primarily only leverages support for the ``img_mgmt`` features necessary to perform OTA firmware updates via the SMP protocol.

## Caveats
* The MCUboot bootloader is required for ``img_mgmt`` to function
  properly. Check documentation for ``mcuboot`` for more information.

* The nRFConnect mobile app now supports DFU over smp. When connecting to this
  device select the 'DFU' icon and choose a .bin file to upload.

* The``mcumgr`` command-line tool only works with Bluetooth Low Energy (BLE)
  on Linux and macOS. On Windows there is no support for Device Firmware
  Upgrade over BLE yet.

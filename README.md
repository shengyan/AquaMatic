# AquaMatic
An aquarium controller based on ESP8266 module.

### What it does?
* Monitor air temperature / humidity using a BME280 module;
* Monitor water temperature using a NTC sensor;
* Schedule a dimmable LED Strip;
* Cooling fan speed control (currently manual control, planning to implement PID control);
* A WebUI dashboard for showing trends and settings.

### Known issue:
* Not connected to wifi on rare condition;
* LED not stick to schedule and stay on full-load (mostly at night and associated with wifi problem).

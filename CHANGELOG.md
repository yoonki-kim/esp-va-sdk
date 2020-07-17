## ChangeLog

### v1.0b2 - 2020-07-17

**Enhancements**
* Various restructurings to support custom audio_boards.
* Support for integrating ESP RainMaker.
* Adding hollow_dsp for using custom dsp driver.
* Adding led_patterns for customisation.
* Adding va_ui to support UIs other than LEDs.

**API Changes**
* Added examples/common/ directory.
* The audio_board files path is changed from board_support_pkgs to components/audio_hal

**Bug Fixes**
* Fixed continuous disconnect from the server.
* Handled multiple acknowledgements from server for connect request which in-turn caused disconnection.
* Fixed various bugs which caused audio to stop during barge-in.

### v1.0b1 (Beta) - 2020-03-06

**Enhancements**

* Basic conversation
* Multi-turn conversations
* Audio Streaming and Playback: Amazon music
* Audio Book Support: Kindle, Audible
* Volume control via voice command
* Alarms, Timers, Reminders, Notifications
* Companion Phone app for configuration and control: Android, iOS
* AWS IoT (MQTT and Shadow) support

**Known Issues**

* You have to reboot the device after setup for the wake-word to work.

# EnvisionedFutures
Audio recording and playback system for the Envisioned Futures art project

Envisioned Futures is an interactive art project created by the multitalented artist Katya Ivanova as part of her larger [Emerging Futures project](http://eivanova.com/emergingfutures/).  Envisioned Futures consists of four dioramas of future cityscapes, each lit with LEDs and sitting inside a mirrored infinity cube.  In addition to presenting a vision of the future, the diorama provides the opportunity for viewers to communicate with the future by recording and listening to audio messages.

## Functions and Features
* Press a button to record a message
* Press a button to play a random, previously recorded message
* Messages stored on SD card for later playback and analysis
* Low power consumption - battery powered

## Hardware Platform
* [PJRC Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
* [PJRC Audio Adapter Board for Teensy](https://www.pjrc.com/store/teensy3_audio.html)
* [Adafruit Mono 2.5W Class D Audio Amplifier - PAM8302](https://www.adafruit.com/product/2130)

## Principle Libraries
* [PJRC Audio](https://github.com/PaulStoffregen/Audio)
* [SdFat](https://github.com/greiman/SdFat)

## Converting Audio Files
EFAudioController uses a raw audio format (i.e., no file headers).  Audio data are stored as a single (mono) channel at 44.1 kpbs using little-endian, signed, 16-bit values.  Using the [SoX](https://en.wikipedia.org/wiki/SoX) sound processing program, the .raw files created by EFAudioController can be converted to WAV format with this command:

    sox -r 44100 -e signed-integer -b 16 -c 1 input_file.raw output_file.wav

A handy way to convert all the files in a directory:

    for f in `ls -1 *.raw`; do raw2wav $f $(basename $f .raw).wav; done


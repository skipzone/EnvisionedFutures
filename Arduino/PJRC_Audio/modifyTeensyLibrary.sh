#! /bin/bash
export tld=/Applications/Teensyduino.app/Contents/Java/hardware/teensy/avr/libraries
shopt -s extglob
for f in +(*.cpp|*.h); do mv -v $tld/Audio/$f $tld/Audio/$f.original && cp -v $f $tld/Audio/; done
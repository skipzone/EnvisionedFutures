/**********************************************************************
 *                                                                    *
 * EFAudioController - Audio Record and Playback Controller           *
 * for the Envisioned Futures diorama project                         *
 *                                                                    *
 * Platform:  Teensy 3.2 + PJRC Audio Adapter Board                   *
 *                                                                    *
 * by Ross Butler, May 2018                                       )'( *
 *                                                                    *
 * Uses the recording method demonstrated in the Recorder.ino         *
 * example that accompanies the PJRC Audio library.                   *
 *                                                                    *
 * Copyright (c) 2018 Ross Butler                                     *
 * This source code is released under the terms of the MIT License.   *
 * See the LICENSE file at the top level of this project for details. *
 *                                                                    *
 **********************************************************************/

#include <SerialFlash.h>
#include <SdFat.h>
#include <Bounce.h>
#include <Audio.h>
#include <stdio.h>
#include <math.h>
#include <WS2812Serial.h>
#define USE_WS2812SERIAL
#include "FastLED.h"



/*****************
 * Configuration *
 *****************/

//#define ENABLE_DEBUG_PRINT

// SD card pins (on the audio board)
#define SDCARD_CS_PIN    10
#define SDCARD_MISO_PIN  12
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14
// TODO:  add a comment listing the other audio board pins so that you don't fuck up the assignments again

// input pins
#define PLAY_LAST_BUTTON_PIN 0
#define PLAY_RANDOM_BUTTON_PIN 1
#define RECORD_BUTTON_PIN 2
#define LIGHT_SENSOR_PIN 17
#define VOLUME_PIN A1

// output pins
#define PLAY_LAST_LED_PIN 3
#define PLAY_RANDOM_LED_PIN 4
#define RECORD_LED_PIN 5
#define ADDRESSABLE_LEDS_DATA_PIN 8
#define STATUS_LED_PIN 20   // was 9, but that conflicts with BCLK for the audio board
#define AMP_SHDN_PIN 21

#define SPI_SPEED SD_SCK_MHZ(50)

#define NUM_LEDS 96

static constexpr int32_t validRecordingMinLengthMs = 1000;

static constexpr int inputSource = AUDIO_INPUT_MIC;           // AUDIO_INPUT_LINEIN or AUDIO_INPUT_MIC
static constexpr unsigned int micInputGain = 55;              // 0 to 63 dB; anything much over 55 seems distorted
static constexpr uint8_t lineOutputLevel = 29;                // 29 (default) = 1.29 V p-p; range is 13 (3.16 V) to 31 (1.16 V)
static constexpr float headphoneVolume = 0.5;                 // 0 to 1.0; 0.5 is comfortable, 0.8 is max. undistored
static constexpr int16_t volumeChangeDetectionThreshold = 3;  // suppresses unnecessary gain updates due to analog reading noise

static constexpr unsigned int numAudioBufferBlocks = 160;

static constexpr char recordingBufferFileName[] = "recbuf.u16";
static constexpr char recordingArchiveDirName[] = "recordingArchive";
static constexpr char recordingSerialNumberFileName[] = "sernum.dat";
static constexpr char rawAudioFileNameExtension[] = "u16";
static constexpr char genericErrorMessageFileName[] = "somethingWentWrong.u16";

static constexpr uint8_t minPanelLedThrobIntensity = 16;
static constexpr uint8_t maxPanelLedThrobIntensity = 128;
static constexpr uint32_t ledThrobStepIntervalMs = 10;

static constexpr uint32_t addressableLedsUpdateIntervalMs = 100;
static constexpr uint8_t addressableLedsIntensity = 128;

static constexpr uint8_t lightSensorSeesDark = HIGH;



/****************
 * Nasty Macros *
 ****************/

#ifdef ENABLE_DEBUG_PRINT
  #define debugPrint(a...) cout << a << endl
#else
  #define debugPrint(a...)  /* don't do anything */
#endif



/*************
 * Constants *
 *************/

enum class OperatingState {
  INIT,
  IDLE_START,
  IDLE,
  RECORDING_START,
  RECORDING,
  RECORDING_STOP,
  PLAY_LAST,
  PLAY_RANDOM,
  PLAYING_START,
  PLAYING,
  ERROR_HALT
};

static constexpr unsigned int maxPathLength = sizeof(recordingArchiveDirName)
                                              + 1    // path separator
                                              + 5    // 5-digit file name
                                              + 1    // a dot
                                              + sizeof(rawAudioFileNameExtension);



/***********
 * Globals *
 ***********/

#ifdef ENABLE_DEBUG_PRINT
ArduinoOutStream cout(Serial);
#endif

// GUItool: begin automatically generated code
// Note:  Remove static keyword before importing this code into GUI tool.
static AudioInputI2S            i2s2;           //xy=221,154
static AudioAnalyzePeak         peak1;          //xy=394,199
static AudioRecordQueue         queue1;         //xy=397,154
static AudioPlaySdRaw           playRaw1;       //xy=418,248
static AudioMixer4              mixer1;         //xy=637,244
static AudioOutputI2S           i2s1;           //xy=812,250
static AudioConnection          patchCord1(i2s2, 0, queue1, 0);
static AudioConnection          patchCord2(i2s2, 0, peak1, 0);
static AudioConnection          patchCord3(playRaw1, 0, mixer1, 0);
static AudioConnection          patchCord4(mixer1, 0, i2s1, 0);
static AudioConnection          patchCord5(mixer1, 0, i2s1, 1);
static AudioControlSGTL5000     sgtl5000_1;     //xy=381,303
// GUItool: end automatically generated code

SdFatSoftSpiEX<SDCARD_MISO_PIN, SDCARD_MOSI_PIN, SDCARD_SCK_PIN> sd;

static Bounce buttonRecord = Bounce(RECORD_BUTTON_PIN, 8);
static Bounce buttonPlayLast =   Bounce(PLAY_LAST_BUTTON_PIN, 8);
static Bounce buttonPlayRandom = Bounce(PLAY_RANDOM_BUTTON_PIN, 8);

static OperatingState opState;

static File recordingBufferFile;

// TODO:  need to initialize this to the last recorded message or a generic message
static char lastRecordingArchiveFilePath[maxPathLength];

CRGB leds[NUM_LEDS];

static uint8_t panelLedThrobIntensity;
static uint32_t ledThrobNextUpdateMs;

static uint32_t addressableLedsNextUpdateMs;



/***********
 * Helpers *
 ***********/

void testAddressableLeds(CRGB rgbColor, bool forceUpdate = false);

void testAddressableLeds(CRGB rgbColor, bool forceUpdate)
{
  uint32_t now = millis();

  if (forceUpdate) addressableLedsNextUpdateMs = now;

  // TODO:  are the signed/unsigned parts right?
  if ((int32_t) (now - addressableLedsNextUpdateMs) >= 0) {
    addressableLedsNextUpdateMs += addressableLedsUpdateIntervalMs;
    for (int i = 0; i < NUM_LEDS; ++i) {
      leds[i] = rgbColor;
    }
    FastLED.show(digitalRead(LIGHT_SENSOR_PIN) == lightSensorSeesDark ? addressableLedsIntensity : 0);
  }
}


bool startAudioRecordQueue()
{
  // Returns true if recording to the recording buffer file has started successfully.

  // The file should not be open already when this function is called,
  // but let's do some defensive driving.
  if (recordingBufferFile.isOpen()) {
    queue1.end();
    recordingBufferFile.close();
  }

  recordingBufferFile.open(&sd, recordingBufferFileName, O_CREAT | O_WRITE | O_TRUNC);

  if (recordingBufferFile.isOpen()) {
    queue1.begin();
  }

  return recordingBufferFile.isOpen();
}


void writeAudioRecordQueueToFile()
{
  static byte buffer[512];
  // The Arduino SD library is most efficient when full 512 byte
  // sector size writes are used.  Not sure if that is true with
  // SdFat, but we'll leave things this way for now.
  while (queue1.available() >= 3) {
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
//    memcpy(buffer+512, queue1.readBuffer(), 256);
//    queue1.freeBuffer();
//    memcpy(buffer+768, queue1.readBuffer(), 256);
//    queue1.freeBuffer();
    recordingBufferFile.write(buffer, 512);
  }
//  while (queue1.available() > 2) {
//  if (queue1.available() > 1) {
//    recordingBufferFile.write((byte*) queue1.readBuffer(), 256);
//    queue1.freeBuffer();
//  }
}


void stopAudioRecordQueue()
{
  if (recordingBufferFile.isOpen()) {
    queue1.end();
    while (queue1.available() > 0) {
      recordingBufferFile.write((byte*) queue1.readBuffer(), 256);
      queue1.freeBuffer();
    }
    recordingBufferFile.close();
  }
}


bool createRecordingArchiveDirectory()
{
  // Returns true if the recording archive directory exists (was
  // already there, or a new one was created), false otherwise.

  if (sd.exists(recordingArchiveDirName)) {
    debugPrint(recordingArchiveDirName << F(" exists"));
    // We need to make sure it really is a directory.
    FatFile archiveDirFile;
    archiveDirFile.open(recordingArchiveDirName, O_READ);
    if (!archiveDirFile.isOpen()) {
      // It exists but we can't open it?!??
      debugPrint(F("can't open archive dir"));
      return false;
    }
    bool isDir = archiveDirFile.isDir();
    archiveDirFile.close();
    if (isDir) {
      // Directory already exists so we'll assume we're good to go.
      debugPrint(F("existing archive dir is a dir"));
      return true;
    }
    // It isn't a directory, it's a file.  Get rid of it so we can create the directory.
    debugPrint(F("removing existing file with same name as archive dir"));
    if (!sd.remove(recordingArchiveDirName)) {
        debugPrint(F("can't remove existing archive dir file"));
        return false;
    }
  }

  // Create the directory and any missing parent directories.
  debugPrint(F("creating archive dir"));
  return sd.mkdir(recordingArchiveDirName, true);
}


void writeLastRecordingSerialNumber(uint16_t serialNumber)
{
  SdFile serialNumberFile;
  if (serialNumberFile.open(&sd, recordingSerialNumberFileName, O_CREAT | O_WRITE | O_TRUNC)) {
    serialNumberFile.println(serialNumber);
    serialNumberFile.close();
    debugPrint(F("wrote serial number ") << serialNumber);
  }
  else {
    debugPrint(F("can't open the serial number file for writing"));
  }
}


uint16_t readLastRecordingSerialNumber()
{
  // Returns the last recording serial number as stored in the serial number file.
  // Returns 0 if the file doesn't exist or if an error occurs.

  uint16_t serialNumber = 0;
  SdFile serialNumberFile;

  if (sd.exists(recordingSerialNumberFileName)) {
    if (serialNumberFile.open(&sd, recordingSerialNumberFileName, O_READ)) {
      char snbuf[12];
      if (serialNumberFile.fgets(snbuf, sizeof(snbuf)) > 0) {
        serialNumber = strtoul(snbuf, NULL, 10);
        debugPrint(F("read serial number ") << serialNumber);
      }
      else {
        debugPrint(F("can't read serial number"));
      }
      serialNumberFile.close();
    }
    else {
      debugPrint(F("can't open the serial number file for reading"));
    }
  }
  else {
    debugPrint(recordingSerialNumberFileName << F(" doesn't exist"));
    // Create and initialize the serial number file.
    writeLastRecordingSerialNumber(serialNumber);
  }

  return serialNumber;
}


void buildRecordingArchiveFilePath(uint16_t serialNumber, char* path)
{
  // The buffer pointed to by path must be at least maxPathLength bytes long.
  sprintf(path, "%s/%05u.%s", recordingArchiveDirName, serialNumber, rawAudioFileNameExtension);
}


bool generateRecordingArchiveFilePath(char* path)
{
  // Returns true if a valid path+name was placed in the buffer pointed to by path.
  // There is no guarantee about preserving the existing contents of the buffer otherwise.
  // The buffer must be at least maxPathLength bytes long.

  uint16_t serialNumber = readLastRecordingSerialNumber();

  // Find the next ununsed file name.
  do {
    ++serialNumber;
    buildRecordingArchiveFilePath(serialNumber, path);
    debugPrint(F("trying path = ") << path);
  } while (sd.exists(path));

  // Save the serial number we're using so that the next file name
  // generation can avoid iterating over existing file names.
  writeLastRecordingSerialNumber(serialNumber);

  return true;
}


bool archiveCurrentRecording()
{
  if (!sd.exists(recordingBufferFileName)) {
    debugPrint(recordingBufferFileName << F(" doesn't exist"));
    return false;
  }

  char recordingArchiveFilePath[maxPathLength];
  if (!generateRecordingArchiveFilePath(recordingArchiveFilePath)) {
    debugPrint(F("couldn't generate archive path+name"));
    return false;
  }

  if (!sd.rename(recordingBufferFileName, recordingArchiveFilePath)) {
    debugPrint(F("couldn't rename ") << recordingBufferFileName << F(" to ") << recordingArchiveFilePath);
    return false;
  }

  strcpy(lastRecordingArchiveFilePath, recordingArchiveFilePath);

  return true;
}


bool selectRandomRecordingArchiveFile(char* path)
{
  uint16_t serialNumber = readLastRecordingSerialNumber();

  // Try a reasonable number of times to select an existing recording file.
  // (We might have to try more than once if there are gaps in the sequence
  // because files have been deleted.)
  for (unsigned int tryCount = 0; tryCount < 10; ++tryCount) {
    uint16_t randomSerialNumber = random(1, serialNumber + 1);
    buildRecordingArchiveFilePath(randomSerialNumber, path);
    if (sd.exists(path)) {
      debugPrint(F("selected file ") << path);
      return true;
    }
  }
  debugPrint(F("unable to find a random archive file after 10 tries"));

  return false;
}


void updateOutputGain(bool forceUpdate)
{
  static int16_t lastVolumeReading;

  int16_t volumeReading = analogRead(VOLUME_PIN);

  // Update the mixer's gain setting if explicitly requested
  // to do so or if the volume control has been moved.
  if (forceUpdate || abs(volumeReading - lastVolumeReading) > volumeChangeDetectionThreshold) {
    lastVolumeReading = volumeReading;

    // A setting very close to or at minimum shuts off the audio output.
    // Also, convert a linear pot to an approximately audio-taper pot
    // with y=a*exp(b*x), where a = 0.001 and b = 6.908.
    // See http://www.dr-lex.be/info-stuff/volumecontrols.html
    float outputGain = volumeReading > volumeChangeDetectionThreshold
                     ? 0.001 * expf(6.908 * ((float) volumeReading / 1023.0))
                     : 0.0;
    debugPrint(F("volumeReading=") << volumeReading << F(", outputGain=") << outputGain);

    mixer1.gain(0, outputGain);
  }
}


void startThrobLights()
{
  ledThrobNextUpdateMs = millis();
  panelLedThrobIntensity = minPanelLedThrobIntensity;
  analogWrite(PLAY_LAST_LED_PIN, panelLedThrobIntensity);
  analogWrite(PLAY_RANDOM_LED_PIN, panelLedThrobIntensity);
  analogWrite(RECORD_LED_PIN, panelLedThrobIntensity);
}


void stopThrobLights()
{
  panelLedThrobIntensity = minPanelLedThrobIntensity;
  analogWrite(PLAY_LAST_LED_PIN, panelLedThrobIntensity);
  analogWrite(PLAY_RANDOM_LED_PIN, panelLedThrobIntensity);
  analogWrite(RECORD_LED_PIN, panelLedThrobIntensity);
}


void doThrobLights()
{
  static bool steppingUp;
  static uint8_t hue;

  uint32_t now = millis();

  // TODO:  are the signed/unsigned parts right?
  if ((int32_t) (now - ledThrobNextUpdateMs) < 0) {
    return;
  }
  ledThrobNextUpdateMs += ledThrobStepIntervalMs;

  if (panelLedThrobIntensity >= maxPanelLedThrobIntensity) {
    steppingUp = false;
    panelLedThrobIntensity = maxPanelLedThrobIntensity;
  }
  else if (panelLedThrobIntensity <= minPanelLedThrobIntensity) {
    steppingUp = true;
    panelLedThrobIntensity = minPanelLedThrobIntensity;
  }

  analogWrite(PLAY_LAST_LED_PIN, panelLedThrobIntensity);
  analogWrite(PLAY_RANDOM_LED_PIN, panelLedThrobIntensity);
  analogWrite(RECORD_LED_PIN, panelLedThrobIntensity);

  if (steppingUp) {
    ++panelLedThrobIntensity;
  }
  else {
    --panelLedThrobIntensity;
  }

  testAddressableLeds(CHSV(hue++, 255, 255));
}



/******************
 * Initialization *
 ******************/

void setup()
{
  pinMode(PLAY_LAST_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_RANDOM_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(VOLUME_PIN, INPUT);

  pinMode(PLAY_LAST_LED_PIN, OUTPUT);
  pinMode(PLAY_RANDOM_LED_PIN, OUTPUT);
  pinMode(RECORD_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(AMP_SHDN_PIN, OUTPUT);

  analogWrite(PLAY_LAST_LED_PIN, 255);
  analogWrite(PLAY_RANDOM_LED_PIN, 255);
  analogWrite(RECORD_LED_PIN, 255);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(AMP_SHDN_PIN, LOW);

#ifdef ENABLE_DEBUG_PRINT
  Serial.begin(9600);
#endif
  debugPrint(F("Starting..."));

  //FastLED.addLeds<WS2812SERIAL, ADDRESSABLE_LEDS_DATA_PIN, BRG>(leds, NUM_LEDS);    // 144 px/m strip
  FastLED.addLeds<WS2812SERIAL, ADDRESSABLE_LEDS_DATA_PIN, BGR>(leds,NUM_LEDS); // 5 mm thru-hole single pixel, Box Project pixel strings
  //LEDS.setBrightness(32);

  // Allocate buffer blocks for recorded audio.
  AudioMemory(numAudioBufferBlocks);

  // Enable and configure the Audio board.
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(inputSource);
  sgtl5000_1.micGain(micInputGain);
  sgtl5000_1.volume(headphoneVolume);
  sgtl5000_1.lineOutLevel(lineOutputLevel);

  updateOutputGain(true);

  testAddressableLeds(CRGB::White, true);

  opState = OperatingState::INIT;
}



/************
 * Run Loop *
 ************/

void loop()
{
  static OperatingState returnState;
  static uint32_t recordingStartMs;
  static char rawAudioFilePath[maxPathLength];

  uint32_t now = millis();

  buttonRecord.update();
  buttonPlayLast.update();
  buttonPlayRandom.update();

  switch(opState) {

    case OperatingState::INIT:
      if (!sd.begin(SDCARD_CS_PIN, SPI_SPEED)) {
        debugPrint(F("sd.begin failed"));
        digitalWrite(STATUS_LED_PIN, HIGH);
        opState = OperatingState::ERROR_HALT;
      }
      else if (!createRecordingArchiveDirectory()) {
        debugPrint(F("createRecordingArchiveDirectory failed"));
        digitalWrite(STATUS_LED_PIN, HIGH);
        opState = OperatingState::ERROR_HALT;
      }
      else {
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::IDLE_START:
      startThrobLights();
      opState = OperatingState::IDLE;
      break;

    case OperatingState::IDLE:
      if (buttonRecord.fallingEdge()) {
        stopThrobLights();
        digitalWrite(STATUS_LED_PIN, LOW);
        opState = OperatingState::RECORDING_START;
      }
      else if (buttonPlayLast.fallingEdge()) {
        stopThrobLights();
        digitalWrite(STATUS_LED_PIN, LOW);
        opState = OperatingState::PLAY_LAST;
      }
      else if (buttonPlayRandom.fallingEdge()) {
        stopThrobLights();
        digitalWrite(STATUS_LED_PIN, LOW);
        opState = OperatingState::PLAY_RANDOM;
      }
      else {
        doThrobLights();
      }
      break;

    case OperatingState::RECORDING_START:
      testAddressableLeds(CRGB::Red, true);
      if (startAudioRecordQueue()) {
        recordingStartMs = now;
        analogWrite(RECORD_LED_PIN, 255);
        opState = OperatingState::RECORDING;
      }
      else {
        // Couldn't create/open the recording buffer file.
        debugPrint(F("startAudioRecordQueue failed"));
        digitalWrite(STATUS_LED_PIN, HIGH);
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::RECORDING:
      writeAudioRecordQueueToFile();
      if (buttonRecord.risingEdge()) {
        opState = OperatingState::RECORDING_STOP;
      }
      break;

    case OperatingState::RECORDING_STOP:
      stopAudioRecordQueue();
      analogWrite(RECORD_LED_PIN, 0);
      if ((int32_t) (now - recordingStartMs) >= validRecordingMinLengthMs) {
        if (!archiveCurrentRecording()) {
          digitalWrite(STATUS_LED_PIN, HIGH);
        }
      }
      debugPrint(F("AudioMemoryUsageMax:  ") << AudioMemoryUsageMax());
      AudioMemoryUsageMaxReset();
      opState = OperatingState::IDLE_START;
      break;

    case OperatingState::PLAY_LAST:
      testAddressableLeds(CRGB::Green, true);
      if (sd.exists(lastRecordingArchiveFilePath)) {
        strcpy(rawAudioFilePath, lastRecordingArchiveFilePath);
        analogWrite(PLAY_LAST_LED_PIN, 255);
      }
      else {
        strcpy(rawAudioFilePath, genericErrorMessageFileName);
      }
      opState = OperatingState::PLAYING_START;
      break;

    case OperatingState::PLAY_RANDOM:
      testAddressableLeds(CRGB::Blue, true);
      if (selectRandomRecordingArchiveFile(rawAudioFilePath)) {
        analogWrite(PLAY_RANDOM_LED_PIN, 255);
      }
      else {
        strcpy(rawAudioFilePath, genericErrorMessageFileName);
      }
      opState = OperatingState::PLAYING_START;
      break;

    case OperatingState::PLAYING_START:
      if (sd.exists(rawAudioFilePath)) {
        digitalWrite(AMP_SHDN_PIN, HIGH);
        playRaw1.play(sd, rawAudioFilePath);
        opState = OperatingState::PLAYING;
      }
      else {
        debugPrint(rawAudioFilePath << F(" doesn't exist"));
        digitalWrite(STATUS_LED_PIN, HIGH);
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::PLAYING:
      if (playRaw1.isPlaying()) {
        updateOutputGain(false);
      }
      else {
        playRaw1.stop();
        digitalWrite(AMP_SHDN_PIN, LOW);
        analogWrite(PLAY_LAST_LED_PIN, 0);
        analogWrite(PLAY_RANDOM_LED_PIN, 0);
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::ERROR_HALT:
      break;
  }

}

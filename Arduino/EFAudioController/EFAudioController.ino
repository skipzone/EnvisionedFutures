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

#include <SdFat.h>
#include <Bounce.h>
#include <Audio.h>


#define ENABLE_DEBUG_PRINT


/*****************
 * Configuration *
 *****************/

#define SDCARD_CS_PIN    10
#define SDCARD_MISO_PIN  12
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14
#define RECORD_BUTTON_PIN 2
#define PLAY_BUTTON_PIN 0
#define PLAY_LED_PIN 3
#define IDLE_LED_PIN 4
#define RECORD_LED_PIN 5
#define ERROR_LED_PIN 16

#define SPI_SPEED SD_SCK_MHZ(50)

static constexpr uint32_t validRecordingMinLengthMs = 1000;

static constexpr int inputSource = AUDIO_INPUT_MIC;   // AUDIO_INPUT_LINEIN or AUDIO_INPUT_MIC
static constexpr unsigned int micInputGain = 55;      // 0 to 63 dB; anything much over 55 seems distorted
static constexpr float lineOutputGain = 0.1;          // 0 to 1.0
static constexpr uint8_t lineOutputLevel = 29;        // 29 (default) = 1.29 V p-p; range is 13 (3.16 V) to 31 (1.16 V)
static constexpr float headphoneVolume = 0.5;         // 0 to 1.0; 0.5 is comfortable, 0.8 is max. undistored

static constexpr unsigned int numAudioBufferBlocks = 60;

static constexpr char recordingBufferFileName[] = "recbuf.u16";
static constexpr char recordingArchiveDirName[] = "recordingArchive";
static constexpr char recordingSerialNumberFileName[] = "sernum.dat";
static constexpr char rawAudioFileNameExtension[] = "u16";


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
  PLAYING_START,
  PLAYING,
  ERROR_HALT
};


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

SdFatSoftSpi<SDCARD_MISO_PIN, SDCARD_MOSI_PIN, SDCARD_SCK_PIN> sd;

static Bounce buttonRecord = Bounce(RECORD_BUTTON_PIN, 8);
static Bounce buttonPlay =   Bounce(PLAY_BUTTON_PIN, 8);

static OperatingState opState;

static File recordingBufferFile;

static char lastRecordingArchiveFilePathName[sizeof(recordingArchiveDirName)
                                             + 1    // path separator
                                             + 5    // 5-digit file name
                                             + 1    // a dot
                                             + sizeof(rawAudioFileNameExtension)];


/***********
 * Helpers *
 ***********/

bool startAudioRecordQueue()
{
  // Returns true if recording to the recording buffer file has started successfully.

  // The file should not be open already when this function is called,
  // but let's do some defensive driving.
  if (recordingBufferFile.isOpen()) {
    queue1.end();
    recordingBufferFile.close();
  }

  recordingBufferFile.open(&sd, recordingBufferFileName,  O_CREAT | O_WRITE | O_TRUNC);

  if (recordingBufferFile.isOpen()) {
    queue1.begin();
  }

  return recordingBufferFile.isOpen();
}


void writeAudioRecordQueueToFile()
{
  // The Arduino SD library is most efficient when full 512 byte
  // sector size writes are used.  Not sure if that is true with
  // SdFat, but we'll leave things this way for now.
  while (queue1.available() >= 2) {
    byte buffer[512];
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    recordingBufferFile.write(buffer, 512);
  }
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
  if (sd.exists(recordingArchiveDirName)) {
    debugPrint(recordingArchiveDirName << F(" exists"));
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


uint16_t getLastRecordingSerialNumber()
{
}


///uint16_t generateRecordingSerialNumber()
///{
///}


bool generateRecordingArchiveFilePathName(char* pathBuf, uint8_t bufLen)
{
}


bool archiveCurrentRecording()
{
  if (!sd.exists(recordingBufferFileName)) {
    return false;
  }

  char recordingArchiveFilePathName[sizeof(lastRecordingArchiveFilePathName)];
  if (!generateRecordingArchiveFilePathName(recordingArchiveFilePathName, sizeof(recordingArchiveFilePathName))) {
      return false;
  }

  if (!sd.rename(recordingBufferFileName, recordingArchiveFilePathName)) {
      return false;
  }

  strcpy(lastRecordingArchiveFilePathName, recordingArchiveFilePathName);

  return true;
}


/******************
 * Initialization *
 ******************/

void setup()
{
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_LED_PIN, OUTPUT);
  pinMode(IDLE_LED_PIN, OUTPUT);
  pinMode(RECORD_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  digitalWrite(PLAY_LED_PIN, LOW);
  digitalWrite(IDLE_LED_PIN, LOW);
  digitalWrite(RECORD_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);

#ifdef ENABLE_DEBUG_PRINT
  Serial.begin(9600);
#endif
  debugPrint(F("Starting..."));

  // Allocate buffer blocks for recorded audio.
  AudioMemory(numAudioBufferBlocks);

  // Enable and configure the Audio board.
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(inputSource);
  sgtl5000_1.micGain(micInputGain);
  sgtl5000_1.volume(headphoneVolume);
  sgtl5000_1.lineOutLevel(lineOutputLevel);
  mixer1.gain(0, lineOutputGain);

  opState = OperatingState::INIT;
}


/************
 * Run Loop *
 ************/

void loop()
{
  static uint32_t recordingStartMs;

  uint32_t now = millis();

///  if ((int32_t) (now - ledNextUpdateMs) >= 0) {
///    ledNextUpdateMs += ledUpdateIntervalMs; 

  buttonRecord.update();
  buttonPlay.update();

  switch(opState) {

    case OperatingState::INIT:
      if (!sd.begin(SDCARD_CS_PIN, SPI_SPEED)) {
        debugPrint(F("sd.begin failed"));
        digitalWrite(ERROR_LED_PIN, HIGH);
        opState = OperatingState::ERROR_HALT;
      }
      else if (!createRecordingArchiveDirectory()) {
        debugPrint(F("createRecordingArchiveDirectory failed"));
        digitalWrite(ERROR_LED_PIN, HIGH);
        opState = OperatingState::ERROR_HALT;
      }
      else {
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::IDLE_START:
      digitalWrite(IDLE_LED_PIN, HIGH);
      opState = OperatingState::IDLE;
      break;

    case OperatingState::IDLE:
      if (buttonRecord.fallingEdge()) {
        digitalWrite(IDLE_LED_PIN, LOW);
        digitalWrite(ERROR_LED_PIN, LOW);
        opState = OperatingState::RECORDING_START;
      }
      else if (buttonPlay.fallingEdge()) {
        digitalWrite(IDLE_LED_PIN, LOW);
        digitalWrite(ERROR_LED_PIN, LOW);
        opState = OperatingState::PLAYING_START;
      }
      break;

    case OperatingState::RECORDING_START:
      if (startAudioRecordQueue()) {
        recordingStartMs = now;
        digitalWrite(RECORD_LED_PIN, HIGH);
        opState = OperatingState::RECORDING;
      }
      else {
        // Couldn't create/open the recording buffer file.
        debugPrint(F("startAudioRecordQueue failed"));
        digitalWrite(ERROR_LED_PIN, HIGH);
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
      digitalWrite(RECORD_LED_PIN, LOW);
      if ((int32_t) (now - recordingStartMs) >= validRecordingMinLengthMs) {
        if (!archiveCurrentRecording()) {
          digitalWrite(ERROR_LED_PIN, LOW);
        }
      }
      opState = OperatingState::IDLE_START;
      break;

    case OperatingState::PLAYING_START:
      if (sd.exists(recordingBufferFileName)) {
        playRaw1.play(sd, recordingBufferFileName);
        digitalWrite(PLAY_LED_PIN, HIGH);
        opState = OperatingState::PLAYING;
      }
      else {
        // Couldn't open the recording buffer file.
        debugPrint(recordingBufferFileName << F(" doesn't exist"));
        digitalWrite(ERROR_LED_PIN, HIGH);
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::PLAYING:
      if (!playRaw1.isPlaying()) {
        playRaw1.stop();
        digitalWrite(PLAY_LED_PIN, LOW);
        opState = OperatingState::IDLE_START;
      }
      break;

    case OperatingState::ERROR_HALT:
      break;
  }

}


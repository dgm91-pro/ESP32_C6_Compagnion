/*
  MAX98357A + SPH0645LM4H Test Program ESP32 C6 COMPAGNION

  This program was made for the ESP32-C6-TOUCH-LCD-1.47 Waveshare Board
  It Capture the microphone data and sends it to the DAC ( Simple Passthrough ).
  Please take a look at the program bellow to avoid any configuration mistake.
  This program is avaiable on the project github repo.

  Note : if the speaker is too close to the Microphone , it might make a Larsen effect.
  Please keep them away or isolate the sound of the Speaker

  Arduino IDE Config :
    - Board           : ESP32C6 Dev Module
    - USB CDC on Boot : Enabled
    - Flash Size      : 8MB ( 64 Mb )
    - Erase All Flash : Enabled
    - Flash mode      : QIO
    - Flash Frequency : 80MHz
 
  https://github.com/dgm91-pro/ESP32_C6_Compagnion

  Created 15 Nov 2026 By Dgm9.1
  This Test code is in the public domain.

*/

#include <Arduino.h>
#include <Wire.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"

// Global definitions
#define PIN_I2S_BCLK  16 // SHARED
#define PIN_I2S_WS    17 // SHARED
#define PIN_I2S_DOUT   7 // MAX98357A
#define PIN_I2S_DIN    6 // SPH0645LM4H

// Needed to disable QMI INT2 Pin and put it to High-Z mode.
#define PIN_I2C_SDA   18 
#define PIN_I2C_SCL   19 


/*
  QMI8658A : Please read this carefully :
    Waveshare docs tell us that the I2C adresses of this module should be 0x51 ( RTC ) , 0x6B ( QMI ) and 0x7E ( AXS5106L ).
    But For Safety reasons please run this program before soldering the board on the pcb or run a I2C scanner / test before to avoid mistakes.

  For more information take a look at this : 
  https://docs.waveshare.com/ESP32-C6-Touch-LCD-1.47
  https://files.waveshare.com/upload/5/5f/QMI8658A_Datasheet_Rev_A.pdf

*/ 

#define QMI_ADDR       0x6B // ToDo: Waveshare FAQ info , might need some change !
#define QMI_CTRL1      0x02 // Hex 02 is for CTRL1 Register


// Audio config
#define SAMPLE_RATE    32000 // 32 Khz - SPH0645LM4H specify an BCLK interval of [2,048 ; 4,096] MHz , With 32000*2*32 we got 2,048 MHz here.
#define FRAMES         128   // Samples of 32bit - SPH0645LM4H uses 32 bit I2S registers , this allows us to keep their original format.

/*
 1 If SPH0645LM4H set to Right canal
 0 If SPH0645LM4H set to Left canal
 In the pcb , SEL→10K→GND , so it's using the left canal
*/
#define MIC_SLOT       0

i2s_chan_handle_t tx_handle = NULL;
i2s_chan_handle_t rx_handle = NULL;

// FRAMES are Timed by 2 because we're using stereo for now.
int32_t rx_buffer[FRAMES * 2];
int32_t tx_buffer[FRAMES * 2];


// I2C QMI Functions
bool qmiRead(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(QMI_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom(QMI_ADDR, (uint8_t)1) != 1)
    return false;
  value = Wire.read();
  return true;
}

bool qmiWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool disableIMUInt2() {
  uint8_t ctrl1;
  if (!qmiRead(QMI_CTRL1, ctrl1)) {
    Serial.println("[-] Cannot Read QMI CTRL1");
    return false;
  }
  Serial.printf("[+] QMI CTRL1 Read : 0x%02X\n", ctrl1);
  // INT2_EN = bit 4
  // 0 = INT2 in High Z 
  ctrl1 &= ~(1 << 4);
  if (!qmiWrite(QMI_CTRL1, ctrl1)) {
    Serial.println("[-] Write Error QMI CTRL1");
    return false;
  }

  uint8_t ctrl1_check;
  if (!qmiRead(QMI_CTRL1, ctrl1_check))
    return false;
  Serial.printf("[+] CTRL1 apres : 0x%02X\n", ctrl1_check);
  if (ctrl1_check & (1 << 4)) {
    Serial.println("[-] Error : INT2 Still Enabled");
    return false;
  }
  Serial.println("[+] Patch Applied : INT2 Set to High-Z");
  return true;
}


// Init I2S in Full Duplex 
void initAudio() {
  i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(
      I2S_NUM_AUTO,
      I2S_ROLE_MASTER
    );
  // Simultaneous allocation TX + RX
  ESP_ERROR_CHECK(
    i2s_new_channel(
      &chan_cfg,
      &tx_handle,
      &rx_handle
    )
  );
  i2s_std_config_t std_cfg = {
    .clk_cfg =
      I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg =
      I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_STEREO
      ),

    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,          // Not Required Here. Bellow Are Defaults pin for the PCB :
      .bclk = (gpio_num_t)PIN_I2S_BCLK, // GPIO 16 By Default
      .ws = (gpio_num_t)PIN_I2S_WS,     // GPIO 17 By Default 
      .dout = (gpio_num_t)PIN_I2S_DOUT, // GPIO 7  By Default ( MAX98357A DIN ) - ESP32
      .din = (gpio_num_t)PIN_I2S_DIN,   // GPIO 6  By Default ( SPH0645LM4H DOUT ) - ESP32

      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  // Configuration of the two channels
  ESP_ERROR_CHECK(
    i2s_channel_init_std_mode(
      tx_handle,
      &std_cfg
    )
  );

  ESP_ERROR_CHECK(
    i2s_channel_init_std_mode(
      rx_handle,
      &std_cfg
    )
  );

  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
  Serial.println("[+] I2S full Duplex Ok !");
}

// Microphone input and speaker output
void audioLoop() {
  size_t bytes_read = 0;
  size_t bytes_written = 0;
  esp_err_t err = i2s_channel_read(
    rx_handle,
    rx_buffer,
    sizeof(rx_buffer),
    &bytes_read,
    100
  );
  if (err != ESP_OK || bytes_read == 0)
    return;

  size_t samples = bytes_read / sizeof(int32_t);
  if (samples % 2 != 0) {
    Serial.println("[-] I2S Error : incomplete stereo frame");
    return;
  }
  size_t frames = samples / 2;

  // Audio Passthrough
  for (size_t i = 0; i < frames; i++) {
    int32_t mic_sample = rx_buffer[i * 2 + MIC_SLOT];

    tx_buffer[i * 2] = mic_sample;
    tx_buffer[i * 2 + 1] = mic_sample;
  }
  esp_err_t write_err = i2s_channel_write(
    tx_handle,
    tx_buffer,
    frames * 2 * sizeof(int32_t),
    &bytes_written,
    100
  );
  ESP_ERROR_CHECK(write_err);
  if (bytes_written != frames * 2 * sizeof(int32_t)) {
    Serial.println("[-] I2S Error: incomplete write");
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  delay(1000);  // 1 sec delay to let Serial initialise
  Serial.println();
  Serial.println("[+] Running Setup");

  // I2C Bus Initialisation
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000); // 100Khz , for better compatibility , Todo: Test in Fast Mode (400Khz)
  delay(50);

  // Disabling INT2 of QMI
  if (!disableIMUInt2()) {
    Serial.println("[-] Error , check QMI8658A connexion !");
    while (true)
      delay(1000);
  }

  // I2S Initialisation
  initAudio();
  Serial.println("[+] I2S Audio ready !");
}


// Loop
void loop() {
  audioLoop(); // Just Keep Audio Passthrough
}

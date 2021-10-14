#include <Arduino.h>
#include <ADS1115_WE.h>
#include<Wire.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2C_ADDRESS 0x48


typedef struct struct_receive_message{
  char flag;

}struct_receive_message;

char chReceiveFlag;

typedef struct struct_send_message{
  float accumulatedValue100times;
  int counter;

}struct_send_message;

struct_send_message message2send;
struct_receive_message incomingMessage;



String success;





// MAC address of our receiving device
uint8_t broadcastAddress[] = {0xE0, 0xE2, 0xE6, 0x4F, 0x27, 0xC8};

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}


// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  Serial.print("Bytes received: ");
  Serial.println(len);
  chReceiveFlag = incomingMessage.flag;

}




/* There are several ways to create your ADS1115_WE object:
   ADS1115_WE adc = ADS1115_WE()             -> uses Wire / I2C Address = 0x48
   ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS)  -> uses Wire / I2C_ADDRESS
   ADS1115_WE adc = ADS1115_WE(&wire2)       -> uses the TwoWire object wire2 / I2C_ADDRESS
   ADS1115_WE adc = ADS1115_WE(&wire2, I2C_ADDRESS) -> all together
   Successfully tested with two I2C busses on an ESP32
*/
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS); // creating an ADS1115_WE object, with SPI address 0x48// to je address spojen na GND
int counter = 0;
float voltage = 0.0;
float values[10];
float midVal = 0;
bool flag = false;
int timediff;
int now;


void ADConverterInit();

void IRAM_ATTR ISR() {
  flag = true;
}




void setup() {
  Wire.begin(); //initializing I2C
  Serial.begin(230400); //koja je ovo serijska komunikacija...:-0
  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }
  pinMode(23, INPUT_PULLUP);
  attachInterrupt(23, ISR, FALLING);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  ADConverterInit();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}


  void loop() {

    //Serial.print("0: ");
    //voltage = readChannel(ADS1115_COMP_0_GND);
    //adc.setCompareChannels(ADS1115_COMP_0_GND);
    //voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
    //adc.setCompareChannels(ADS1115_COMP_1_GND);
    if (flag)
    {
      voltage = adc.getResult_mV(); // alternative: getResult_mV for Millivolt
      midVal = midVal + voltage;
      counter++;
      //if (counter==7704)
      if (counter == 100)
      {
        now = millis();
        timediff = now - timediff;
        Serial.println(" ");
        //midVal=midVal/7704;
        midVal = midVal / 100;
        Serial.println(midVal);
        message2send.accumulatedValue100times = midVal;
        message2send.counter = counter;
        Serial.println(" ");
        midVal = 0;
        counter = 0;
        Serial.println(timediff);
        timediff = now;
          // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message2send, sizeof(message2send));
        
        if (result == ESP_OK){
          Serial.println("Ne treba cuvati podatke");
        }
        else{
          Serial.println("Treba napisati neko logovanje");
        }
        
        /*
          timediff=millis()-timediff;
          Serial.println(timediff);
          timediff=millis();
        */
      }
      flag = false;
    }
    //delay(3);
  }

  float readChannel(ADS1115_MUX channel) {
    //float voltage = 0.0;
    adc.setCompareChannels(channel);
    voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
    return voltage;
  }



void ADConverterInit() {
    /* Set the voltage range of the ADC to adjust the gain
      Please note that you must not apply more than VDD + 0.3V to the input pins!

      ADS1115_RANGE_6144  ->  +/- 6144 mV
      ADS1115_RANGE_4096  ->  +/- 4096 mV
      ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
      ADS1115_RANGE_1024  ->  +/- 1024 mV
      ADS1115_RANGE_0512  ->  +/- 512 mV
      ADS1115_RANGE_0256  ->  +/- 256 mV
    */
    adc.setVoltageRange_mV(ADS1115_RANGE_4096); //Postavljen opseg do 4096mV sto znaci da je rezolucija 125uV (Ovo treba proveriti sta je optimalno sa hardverom)

    /* Set the inputs to be compared

        ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
        ADS1115_COMP_0_3    ->  compares 0 with 3
        ADS1115_COMP_1_3    ->  compares 1 with 3
        ADS1115_COMP_2_3    ->  compares 2 with 3
        ADS1115_COMP_0_GND  ->  compares 0 with GND
        ADS1115_COMP_1_GND  ->  compares 1 with GND
        ADS1115_COMP_2_GND  ->  compares 2 with GND
        ADS1115_COMP_3_GND  ->  compares 3 with GND
    */
    adc.setCompareChannels(ADS1115_COMP_0_GND); //stavljeno da uporedjujemo ulaz A0 sa GND. Ovde bi trebalo proveriti sta radi funkcija setSingleChannel(0...3)

    /* Set number of conversions after which the alert pin asserts
       - or you can disable the alert

        ADS1115_ASSERT_AFTER_1  -> after 1 conversion
        ADS1115_ASSERT_AFTER_2  -> after 2 conversions
        ADS1115_ASSERT_AFTER_4  -> after 4 conversions
        ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default)
    */
    adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //radimo u modu da podizemo Alert svaki put kada je odradjena konverzija i spremna za citanje. Na osnovu ovoga dizemo interrupt. Treba videti da li idemo na HIGH ili na LOW

    /* Set the conversion rate in SPS (samples per second)
       Options should be self-explaining:

        ADS1115_8_SPS
        ADS1115_16_SPS
        ADS1115_32_SPS
        ADS1115_64_SPS
        ADS1115_128_SPS (default)
        ADS1115_250_SPS
        ADS1115_475_SPS
        ADS1115_860_SPS
    */
    adc.setConvRate(ADS1115_8_SPS); //Broj odbiraka u sekundi trenutno je 8 za testiranje, a za projekat treba 128 sto je default

    /* Set continuous or single shot mode:

        ADS1115_CONTINUOUS  ->  continuous mode
        ADS1115_SINGLE     ->  single shot mode (default)
    */
    adc.setMeasureMode(ADS1115_CONTINUOUS); //trenutno smo stavljeni u Continous mode, ali sam ubedjen da je bolje u Singel-shot modu

    /* Choose maximum limit or maximum and minimum alert limit (window) in Volt - alert pin will
       assert when measured values are beyond the maximum limit or outside the window
       Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
       In max limit mode the minimum value is the limit where the alert pin assertion will be
       cleared (if not latched)

       ADS1115_MAX_LIMIT
       ADS1115_WINDOW

    */
    //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); // zakomentarisano da pustamo alarm kada izadje iz zadane vrednosti, ali ovo nas ne interesuje

    /* Enable or disable latch. If latch is enabled the alert pin will assert until the
       conversion register is read (getResult functions). If disabled the alert pin assertion will be
       cleared with next value within limits.

        ADS1115_LATCH_DISABLED (default)
        ADS1115_LATCH_ENABLED
    */
    adc.setAlertLatch(ADS1115_LATCH_ENABLED); //ova funkcija nam treba da bi mogli da ynamo da li treba da citamo vrednost ili ne. 

    /* Sets the alert pin polarity if active:

       ADS1115_ACT_LOW  ->  active low (default)
       ADS1115_ACT_HIGH ->  active high
    */
    adc.setAlertPol(ADS1115_ACT_LOW); // za pravljenje interapta, ali potrebno je videti da li zelimo HIGH ili LOW

    /* With this function the alert pin will assert, when a conversion is ready.
       In order to deactivate, use the setAlertLimit_V function
    */
    adc.setAlertPinToConversionReady(); // samo sa pozivanjem ove funkcije govorimo da zelimo da ALARM pin radi tako da menja stanje svaki put kada je vrednost u registru spremna za citanje. 

    Serial.println("ADS1115 Example Sketch - Continuous Mode");
    Serial.println("All values in volts");
    Serial.println();
    adc.setCompareChannels(ADS1115_COMP_0_GND); //ne znam sto opet zovemo ovu funkciju, proveriti
 

  /* If you change the compare channels you can immediately read values from the conversion
     register, although they might belong to the former channel if no precautions are taken.
     It takes about the time needed for two conversions to get the correct data. In single
     shot mode you can use the isBusy() function to wait for data from the new channel. This
     does not work in continuous mode.
     To solve this issue the library adds a delay after change of channels if you are in contunuous
     mode. The length of the delay is adjusted to the conversion rate. But be aware that the output
     rate will be much lower that the conversion rate if you change channels frequently.
  */


}

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin

// CS-pinnit 16, 14, 12

Adafruit_MAX31865 hltSensor = Adafruit_MAX31865(16, 0, 4, 5);
Adafruit_MAX31865 mltSensor = Adafruit_MAX31865(14, 0, 4, 5);
// Adafruit_MAX31865 bkSensor = Adafruit_MAX31865(12, 0, 4, 5);

// RTD Module reference resistor value and sensor 0c nominal resistance
#define RREF 430.0
#define RNOMINAL 100.0

// SSR Relay Pins
#define HLT_SSR 13
#define BK_SSR 15

// PID Autotune
byte ATuneModeRemember = 2;

double outputStart = 5;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

bool mashing = false;
bool sparging = false;
bool boiling = false;

struct vessel
{
  double Setpoint = 0;
  double Input;
  double Output;
  double Kp;
  double Ki;
  double Kd;
  bool At = false;
  bool On = false;
  int windowSize = 2000;
  unsigned long windowStartTime;
};

typedef struct vessel Vessel;

// Brewing Vessels
Vessel HLT;
Vessel MLT;
Vessel BK;

PID hltPID(&HLT.Input, &HLT.Output, &HLT.Setpoint, HLT.Kp, HLT.Ki, HLT.Kd, DIRECT);
PID mltPID(&MLT.Input, &MLT.Output, &MLT.Setpoint, MLT.Kp, MLT.Ki, MLT.Kd, DIRECT);
PID bkPID(&BK.Input, &BK.Output, &BK.Setpoint, BK.Kp, BK.Ki, BK.Kd, DIRECT);

PID_ATune hltATune(&HLT.Input, &HLT.Output);
PID_ATune mltATune(&MLT.Input, &MLT.Output);
PID_ATune bkATune(&BK.Input, &BK.Output);

// ArduinoJSON Communication
StaticJsonDocument<200> doc;
JsonObject hlt, mlt, bk;

void sendJSON()
{
  doc["mashing"] = mashing;
  doc["sparging"] = sparging;
  doc["boiling"] = boiling;

  hlt["sp"] = HLT.Setpoint;
  hlt["pv"] = HLT.Input;
  hlt["pow"] = HLT.Output;
  hlt["P"] = HLT.Kp;
  hlt["I"] = HLT.Ki;
  hlt["D"] = HLT.Kd;
  hlt["tune"] = HLT.At;
  hlt["on"] = HLT.On;

  mlt["sp"] = MLT.Setpoint;
  mlt["pv"] = MLT.Input;
  mlt["pow"] = MLT.Output;
  mlt["P"] = MLT.Kp;
  mlt["I"] = MLT.Ki;
  mlt["D"] = MLT.Kd;
  mlt["tune"] = MLT.At;

  bk["sp"] = BK.Setpoint;
  bk["pv"] = BK.Input;
  bk["pow"] = BK.Output;
  bk["P"] = BK.Kp;
  bk["I"] = BK.Ki;
  bk["D"] = BK.Kd;
  bk["tune"] = BK.At;
  bk["on"] = BK.On;

  serializeJson(doc, Serial);
}

void AutoTuneHelper(boolean start, PID &pid)
{
  if (start)
    ATuneModeRemember = pid.GetMode();
  else
    pid.SetMode(ATuneModeRemember);
}

void toggleAutotune(Vessel &vessel, PID &pid, PID_ATune &aTune)
{
  if (!vessel.At)
  {
    // Set the output to the desired starting frequency.
    vessel.Output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true, pid);
    vessel.At = true;
  }
  else
  { // cancel autotune
    aTune.Cancel();
    vessel.At = false;
    AutoTuneHelper(false, pid);
  }
}

void sendError(DeserializationError err)
{
  // Print error back to serial port
  Serial.print(F("{error:"));
  Serial.print(err.c_str());
  Serial.println(F("}"));
}

void readJSON()
{
  if (Serial.available())
  {
    DeserializationError err = deserializeJson(doc, Serial);

    if (err == DeserializationError::Ok)
    {
      // States
      mashing = doc["mashing"] ? doc["mashing"].as<bool>() : mashing;
      sparging = doc["sparging"] ? doc["sparging"].as<bool>() : sparging;
      boiling = doc["boiling"] ? doc["boiling"].as<bool>() : boiling;

      // HLT
      StaticJsonDocument<100> hltDoc;
      deserializeJson(hltDoc, doc["hlt"]);
      JsonObject hlt = hltDoc.as<JsonObject>();

      HLT.Setpoint = hlt["sp"].as<double>();
      HLT.Kp = hlt["P"] ? hlt["P"].as<double>() : HLT.Kp;
      HLT.Ki = hlt["I"] ? hlt["I"].as<double>() : HLT.Ki;
      HLT.Kd = hlt["D"] ? hlt["D"].as<double>() : HLT.Kd;

      // Power On/Off
      if (hlt["on"] && hlt["on"].as<bool>() != HLT.On)
      {
        HLT.On = hlt["on"].as<bool>();
      }

      // Toggle autotune
      if (hlt["tune"] && hlt["tune"].as<bool>() != HLT.At)
      {
        HLT.At = hlt["tune"].as<bool>();
        mashing = false;
        sparging = true;

        // Static temperature for PID Autotune
        HLT.Setpoint = 71.0;
        toggleAutotune(HLT, hltPID, hltATune);
      }

      // MLT
      StaticJsonDocument<100> mltDoc;
      deserializeJson(mltDoc, doc["mlt"]);
      JsonObject mlt = mltDoc.as<JsonObject>();

      MLT.Setpoint = mlt["sp"].as<double>();
      MLT.Kp = mlt["P"] ? mlt["P"].as<double>() : MLT.Kp;
      MLT.Ki = mlt["I"] ? mlt["I"].as<double>() : MLT.Ki;
      MLT.Kd = mlt["D"] ? mlt["D"].as<double>() : MLT.Kd;

      if (mlt["tune"] && mlt["tune"].as<bool>() != MLT.At)
      {
        MLT.At = mlt["tune"].as<bool>();
        mashing = true;
        sparging = false;

        // Static temperature for PID Autotune
        MLT.Setpoint = 66.6;
        toggleAutotune(MLT, mltPID, mltATune);
      }

      // BK
      StaticJsonDocument<100> bkDoc;
      deserializeJson(bkDoc, doc["bk"]);
      JsonObject bk = bkDoc.as<JsonObject>();

      BK.Setpoint = bk["sp"].as<double>();
      BK.Kp = bk["P"] ? bk["P"].as<double>() : BK.Kp;
      BK.Ki = bk["I"] ? bk["I"].as<double>() : BK.Ki;
      BK.Kd = bk["D"] ? bk["D"].as<double>() : BK.Kd;

      // Power On/Off
      if (bk["on"] && bk["on"].as<bool>() != BK.On)
      {
        BK.On = bk["on"].as<bool>();
      }

      // Toggle autotune
      if (bk["tune"] && bk["tune"].as<bool>() != BK.At)
      {
        BK.At = bk["tune"].as<bool>();
        boiling = true;

        // Static temperature for PID Autotune
        BK.Setpoint = 99.8;
        toggleAutotune(BK, bkPID, bkATune);
      }
    }
    else
    {
      sendError(err);

      // Flush all bytes in the serial port buffer
      while (Serial.available() > 0)
        Serial.read();
    }
  }
}

void setup()
{
  Serial.begin(115200);

  hltSensor.begin(MAX31865_4WIRE);
  mltSensor.begin(MAX31865_4WIRE);
  // bkSensor.begin(MAX31865_4WIRE);

  pinMode(HLT_SSR, OUTPUT);
  pinMode(BK_SSR, OUTPUT);

  hlt = doc.createNestedObject("hlt");
  mlt = doc.createNestedObject("mlt");
  bk = doc.createNestedObject("bk");

  Serial.println(hltPID.GetMode());
}

void loop()
{
  // Read temperature sensors
  hltSensor.readRTD();
  mltSensor.readRTD();
  // bkSensor.readRTD();

  HLT.Input = hltSensor.temperature(RNOMINAL, RREF);
  MLT.Input = mltSensor.temperature(RNOMINAL, RREF);
  BK.Input = hltSensor.temperature(RNOMINAL, RREF);

  // Send data as JSON through Serial
  // sendJSON();
  Serial.println(HLT.Input);
  uint8_t fault = hltSensor.readFault();

  if (fault)
  {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH)
    {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH)
    {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW)
    {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH)
    {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW)
    {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV)
    {
      Serial.println("Under/Over voltage");
    }
    hltSensor.clearFault();
  }

  Serial.println(MLT.Input);
  fault = mltSensor.readFault();

  if (fault)
  {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH)
    {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH)
    {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW)
    {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH)
    {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW)
    {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV)
    {
      Serial.println("Under/Over voltage");
    }
    mltSensor.clearFault();
  }

  if (mashing)
  {

    // PID Auto Tuning
    if (MLT.At)
    {
      if (mltATune.Runtime() != 0)
      {
        MLT.At = false;
        MLT.Kp = mltATune.GetKp();
        MLT.Ki = mltATune.GetKi();
        MLT.Kd = mltATune.GetKd();
        mltPID.SetTunings(MLT.Kp, MLT.Ki, MLT.Kd);
        mashing = false;
        AutoTuneHelper(false, mltPID);
      }
    }
    else
    {
      // Calculate Mash Master PID (MLT)...
      mltPID.Compute();
    }

    //...and supply (output * 100c) as setpoint for Slave PID (HLT)
    HLT.Setpoint = (MLT.Output * 100);

    // Calculate Mash Slave PID (HLT)
    hltPID.Compute();
  }
  else if (sparging)
  {

    // PID Auto Tuning
    if (HLT.At)
    {
      if (hltATune.Runtime() != 0)
      {
        HLT.At = false;
        HLT.Kp = hltATune.GetKp();
        HLT.Ki = hltATune.GetKi();
        HLT.Kd = hltATune.GetKd();
        hltPID.SetTunings(HLT.Kp, HLT.Ki, HLT.Kd);
        sparging = false;
        AutoTuneHelper(false, hltPID);
      }
    }
    else
    {
      // Calculate HLT PID with a manual setpoint
      hltPID.Compute();
    }
  }

  if (boiling)
  {

    // PID Auto Tuning
    if (BK.At)
    {
      if (bkATune.Runtime() != 0)
      {
        BK.At = false;
        BK.Kp = bkATune.GetKp();
        BK.Ki = bkATune.GetKi();
        BK.Kd = bkATune.GetKd();
        bkPID.SetTunings(BK.Kp, BK.Ki, BK.Kd);
        boiling = false;
        AutoTuneHelper(false, bkPID);
      }
    }
    else
    {
      // Calculate Boil PID
      bkPID.Compute();
    }
  }

  if (HLT.On)
  {
    if (millis() - HLT.windowStartTime > HLT.windowSize)
    {
      HLT.windowStartTime += HLT.windowSize;
    }
    if (HLT.Output > millis() - HLT.windowStartTime)
    {
      digitalWrite(HLT_SSR, HIGH);
    }
    else
    {
      digitalWrite(HLT_SSR, LOW);
    }
  }

  if (BK.On)
  {
    if (millis() - BK.windowStartTime > BK.windowSize)
    {
      BK.windowStartTime += BK.windowSize;
    }
    if (BK.Output > millis() - BK.windowStartTime)
    {
      digitalWrite(HLT_SSR, HIGH);
    }
    else
    {
      digitalWrite(HLT_SSR, LOW);
    }
  }
}
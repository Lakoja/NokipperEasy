/*
 * Copyright (C) 2018 Lakoja on github.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __LINE_STEPPER_H__
#define __LINE_STEPPER_H__

class LineStepper
{
private:
  const double MIN_FREQUENCY = 0.5;
  const double MAX_FREQUENCY = 20000.0;
  const uint16_t MIN_LEVEL_MICROS = 2;
  
  uint8_t stepPin;
  uint32_t lastDriveMicros = 0;
  double currentLiveFrequency = 0;
  uint32_t currentLiveFrequencyDoubledMicros = 0;
  double currentDesiredFrequency = 0;
  uint32_t lastLevelChangeMicros = 0;
  bool levelIsHigh = false;
  uint32_t stepCount = 0;

public:

  void setup(uint8_t pin)
  {
    stepPin = pin;
    outputPin(stepPin);
  }

  void drive(bool debug = false)
  {
    uint32_t nowMicros = esp_timer_get_time();

    if (nowMicros - lastDriveMicros >= 500) {
      //Serial.println("LOoPing SLOW micros "+String(nowMicros-lastDriveMicros));
    }

    if (nowMicros - lastDriveMicros >= 50) {
      bool frequencyChange = currentDesiredFrequency != currentLiveFrequency;

      if (frequencyChange) {
        currentLiveFrequency = currentDesiredFrequency;
        if (currentLiveFrequency != 0) {
          currentLiveFrequencyDoubledMicros = getFrequencyDoubledMicros(currentLiveFrequency);
        }
      }

      if (currentLiveFrequency != 0) {
        uint32_t nextDesiredMicros = lastLevelChangeMicros + currentLiveFrequencyDoubledMicros;

        if (nowMicros >= nextDesiredMicros) {
          changeLevel();
          if (frequencyChange) {
            lastLevelChangeMicros = nowMicros;
          } else {
            lastLevelChangeMicros = nextDesiredMicros;
          }

          if (lastLevelChangeMicros + currentLiveFrequencyDoubledMicros < nowMicros) {
            if (debug) {
              Serial.print("TOO FAST "+String(currentLiveFrequencyDoubledMicros)+" "+String(currentLiveFrequency)+" "+String(lastLevelChangeMicros - currentLiveFrequencyDoubledMicros)+" "+String(nowMicros));
            }
            
            lastLevelChangeMicros = nowMicros;
          }
        }
        
      } else if (levelIsHigh) {
        changeLevel();
        lastLevelChangeMicros = nowMicros;
      }
      
      lastDriveMicros = nowMicros;
    }
  }

  void setFrequency(double freq)
  {
    if (freq != 0) {
      freq = max(abs(freq), MIN_FREQUENCY);
    }

    if (freq > MAX_FREQUENCY) {
      Serial.println("\n!!!!!!! Frequency too high!!!!! "+String(freq));
    }

    currentDesiredFrequency = freq;
  }

  int32_t getCurrentSteps(bool reset)
  {
    int32_t current = stepCount;
    if (reset) {  
      stepCount = 0;
    }

    return current;
  }

private:
  void outputPin(int num)
  {
    digitalWrite(num, LOW);
    pinMode(num, OUTPUT);
  }

  void changeLevel()
  {
    digitalWrite(stepPin, levelIsHigh ? LOW : HIGH);
    levelIsHigh = !levelIsHigh;
    if (levelIsHigh) {
      ++stepCount;
    }
  }

  uint32_t getFrequencyDoubledMicros(double frequency)
  {
    if (frequency == 0) {
      Serial.println("\n!!!!!!!!!!!! Division by ZERO frequency!!!!!");
      return 1000000;
    }
    
    return round(1000000.0 / (frequency * 2));
  }
};

#endif

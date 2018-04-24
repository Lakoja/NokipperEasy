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

#ifndef __BUCKET_STEP_COUNTER_H__
#define __BUCKET_STEP_COUNTER_H__

class BucketStepCounter
{
private:
  int16_t *buckets;
  uint16_t bucketPointer = 0;
  int32_t totalSteps = 0;
  uint16_t bucketCount = 0;
  uint16_t bucketWidthMillis = 50;
  uint32_t lastAddMillis = 0;

public:
  BucketStepCounter(uint16_t count, uint16_t millisWidth)
  {
    bucketCount = count;
    bucketWidthMillis = millisWidth;

    buckets = new int16_t[bucketCount];
    for (uint16_t i=0; i<bucketCount; ++i) {
      buckets[i] = 0;
    }
  }

  ~BucketStepCounter()
  {
    delete buckets;
  }

  bool debug = false;

  int32_t addSteps(int16_t steps)
  {
    if (0 == steps) {
      return totalSteps;
    }
    
    uint32_t now = millis();
    uint16_t bucketPointerNow = round((now % (bucketCount * bucketWidthMillis)) / bucketWidthMillis);

    if (bucketPointer != bucketPointerNow) {
      if (buckets[bucketPointerNow] != 0) {
        totalSteps -= buckets[bucketPointerNow];
        buckets[bucketPointerNow] = 0;
        if (debug)
          Serial.println("--");
      }
    }
    
    if (debug)
          Serial.print("a"+String(steps)+" p"+String(bucketPointerNow)+" v"+String(buckets[bucketPointerNow]));

    buckets[bucketPointerNow] += steps;
    totalSteps += steps;

    if (debug)
          Serial.print(" b"+String(buckets[bucketPointerNow])+" >"+String(totalSteps)+"  ");

    bucketPointer = bucketPointerNow;
    lastAddMillis = now;

    return totalSteps;
  }

  int32_t getTotalSteps()
  {
    return totalSteps;
  }
};

#endif

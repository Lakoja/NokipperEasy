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

#ifndef __WIFI_TELNET_SERVER_H__
#define __WIFI_TELNET_SERVER_H__

#include <WiFi.h>
#include "Task.h"

const int CHANNEL = 8;
const int LED_PIN = 2;

class WifiTelnetServer : public WiFiServer, public Task
{
private:
  WiFiClient client;
  uint32_t clientConnectTime;
  bool clientNowConnected = false;
  bool waitForRequest = false;
  String currentLine = "";
  
public:
  static WifiTelnetServer* instance;
  
  static void begin(uint32_t speed)
  {
    pinMode(LED_PIN, OUTPUT);
    
    bool wifiSuccess = instance->setupWifi();
    if (!wifiSuccess) {
      instance = NULL;
    } else {
      digitalWrite(LED_PIN, HIGH);
    }

    Serial.begin(speed);
  }
  
  static void print(String str)
  {
    if (instance != NULL && instance->client.connected()) {
      instance->client.print(str);
    }
    
    Serial.print(str);
  }

  static void println(String str)
  {
    if (instance != NULL && instance->client.connected()) {
      instance->client.println(str);
    }
      
    Serial.println(str);
  }

  WifiTelnetServer(int port) : WiFiServer(port)
  {
    instance = this; 
  }

  void startServer()
  {
    WiFiServer::begin();
  }

  virtual void run()
  {
    while (true) {
      uint32_t loopStart = millis();
      if (!client.connected()) {
        if (clientNowConnected) {
          Serial.println("Disconnected");
        }
        
        clientNowConnected = false;
      
        client = accept();

        if (client.connected()) {
          Serial.print("New ");
        }
      }

      if (client.connected()) {
        if (!clientNowConnected) {
          clientNowConnected = true;
          clientConnectTime = millis();
          currentLine = "";
      
          waitForRequest = true;
        }

        if (waitForRequest) {
          String requested = parseRequest();

          if (requested.length() > 0) {
            waitForRequest = false;
            
            if (requested.startsWith("/ ")) {
              String responseHeader = "HTTP/1.1 200 OK\n";
              responseHeader += "Content-Type: text/plain\n";
              responseHeader += "\n";
              responseHeader += "Welcome watcher...\n";
              client.print(responseHeader);
    
              Serial.println("Serving new client the HTTP response header");
            } else {
              Serial.println("Ignoring request "+requested);
              
              client.println("HTTP/1.1 404 Not Found");
              client.println();
              client.stop();
            }
          }
        }
      }
      
      delay(1);
    }
  }
  
private:
  bool setupWifi()
  {
    WiFi.mode(WIFI_AP);
    
    bool b1 = WiFi.softAPConfig(IPAddress(192,168,191,1), IPAddress(192,168,191,254), IPAddress(255,255,255,0));
    bool b2 = WiFi.softAP("Nokipper", NULL, CHANNEL, 0, 1);
    delay(100);
  
    if (b1 && b2) {
      Serial.print("WiFi AP started ");
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("Could not start AP. config: "+String(b1)+" start:"+String(b2));
      return false;
    }
  
    return true;
  }

  String parseRequest()
  {
    if (!waitForRequest) {
      return "";
    }

    if (millis() - clientConnectTime > 2000) {
      waitForRequest = false;
      client.stop();
      Serial.println("Waited too long for a client request. Current line content: "+currentLine);
    }
    
    uint32_t methodStartTime = micros();

    // TODO simplify time check
    while (client.connected() && micros() - methodStartTime < 2000) {
      if (client.available() == 0)
        delayMicroseconds(200);

      while (client.available() > 0 && micros() - methodStartTime < 2000) {
        /* This does not read past the first line and results in a "broken connection" in Firefox
        String oneRequestLine = client.readStringUntil('\n');
        Serial.println(oneRequestLine);
        if (currentLine.length() == 0) {
          break;
        }*/
        
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            Serial.print("B");
            break;
          } else {
            if (currentLine.startsWith("GET ")) {
              return currentLine.substring(4);
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    return "";
  }
};

#endif

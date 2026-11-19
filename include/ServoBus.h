#pragma once
#include <SCServo.h>
#include "config.h"

// Takao Base SCS0009では、送信したデータが受信側にも折り返される。
// このエコーだけを除去し、後続のサーボ応答は受信済みでも残しておく。
class ServoBus : public SMS_STS {
 public:
  bool echoOk = true;

 protected:
  void rFlushSCS() override {
    SCSerial::rFlushSCS();
    sentLength = 0;
    echoOk = true;
  }

  int writeSCS(unsigned char* data, int length) override {
    if (!data || length < 0 || sentLength + length > sizeof(sent)) {
      echoOk = false;
      return 0;
    }
    const int written = SCSerial::writeSCS(data, length);
    for (int i = 0; i < written; ++i) sent[sentLength++] = data[i];
    if (written != length) echoOk = false;
    return written;
  }

  int writeSCS(unsigned char data) override { return writeSCS(&data, 1); }

  void wFlushSCS() override {
    pSerial->flush(); if (!kExpectTxEcho) return;
    for (unsigned int i = 0; i < sentLength; ++i) {
      unsigned char received = 0;
      if (SCSerial::readSCS(&received, 1) != 1 || received != sent[i]) {
        echoOk = false;
        break;
      }
    }
  }

  int readSCS(unsigned char* data, int length) override {
    // 不一致のエコーや不完全な通信データをサーボの状態として解釈しない。
    return echoOk ? SCSerial::readSCS(data, length) : 0;
  }

 private:
  unsigned char sent[32] = {};
  unsigned int sentLength = 0;
};


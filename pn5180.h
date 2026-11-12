//
// NAME: pn5180.h
//
// DESC: NFC Communication with NXP Semiconductors PN5180 module for Arduino.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// Modified by Junsei Hanada
// Modification date: 2026-11-13
//
// Modifications:
// - Removed unnecessary declarations.
// - Modified class definitions and function declarations.
// - Added and revised Japanese comments.
//


#pragma once
#include <SPI.h>

//PN5180 コマンドリスト
#define PN5180_WRITE_REGISTER           (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK   (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK  (0x02)
#define PN5180_READ_REGISTER            (0x04)
#define PN5180_WRITE_EEPROM             (0x06)
#define PN5180_READ_EEPROM              (0x07)
#define PN5180_SEND_DATA                (0x09)
#define PN5180_READ_DATA                (0x0A)
#define PN5180_SWITCH_MODE              (0x0B)
#define PN5180_LOAD_RF_CONFIG           (0x11)
#define PN5180_RF_ON                    (0x16)
#define PN5180_RF_OFF                   (0x17)

//PN5180 レジスタリスト
#define SYSTEM_CONFIG       (0x00)
#define IRQ_ENABLE          (0x01)
#define IRQ_STATUS          (0x02)
#define IRQ_CLEAR           (0x03)
#define TRANSCEIVE_CONTROL  (0x04)
#define TIMER1_RELOAD       (0x0c)
#define TIMER1_CONFIG       (0x0f)
#define RX_WAIT_CONFIG      (0x11)
#define CRC_RX_CONFIG       (0x12)
#define RX_STATUS           (0x13)
#define CRC_TX_CONFIG       (0x19)
#define RF_STATUS           (0x1d)
#define SYSTEM_STATUS       (0x24)
#define TEMP_CONTROL        (0x25)

//PN5180 EEPROMアドレス
#define DIE_IDENTIFIER      (0x00)
#define PRODUCT_VERSION     (0x10)
#define FIRMWARE_VERSION    (0x12)
#define EEPROM_VERSION      (0x14)
#define IRQ_PIN_CONFIG      (0x1A)

// PN5180 IRQ_STAT 割り込みフラグ(32bit)
#define RX_IRQ_STAT         (1<<0)  // RF受信終了割り込み
#define TX_IRQ_STAT         (1<<1)  // RF送信終了割り込み
#define IDLE_IRQ_STAT       (1<<2)  // IDLE状態割り込み
#define RFOFF_DET_IRQ_STAT  (1<<6)  // RFフィールドOFF検出割り込み
#define RFON_DET_IRQ_STAT   (1<<7)  // RFフィールドON検出割り込み
#define TX_RFOFF_IRQ_STAT   (1<<8)  // PCDによるRFフィールドOFF割り込み
#define TX_RFON_IRQ_STAT    (1<<9)  // PCDによるRFフィールドON割り込み
#define RX_SOF_DET_IRQ_STAT (1<<14) // RF受信開始信号 検出割り込み

enum PN5180TransceiveStat {
  PN5180_TS_Idle = 0,
  PN5180_TS_WaitTransmit = 1,
  PN5180_TS_Transmitting = 2,
  PN5180_TS_WaitReceive = 3,
  PN5180_TS_WaitForData = 4,
  PN5180_TS_Receiving = 5,
  PN5180_TS_LoopBack = 6,
  PN5180_TS_RESERVED = 7
};

class PN5180 {
private:
  uint8_t PN5180_NSS; //LOWで通信開始
  uint8_t PN5180_BUSY; //監視用
  uint8_t PN5180_RST; //リセット用
  //設定用インスタンス
  SPISettings PN5180_SPI_Settings;
  static uint8_t readBuf[508]; //実体無し・変数の存在だけ定義
public:
  /*コンストラクタ*/
  PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin);
  /*開始終了用*/
  void begin();
  void end();
  /*------------PN5180用コマンド-------------*/
  /*レジスタ書き込み cmd (0x00)*/
  bool writeRegister(uint8_t addr, uint32_t val);
  /*レジスタにORマスク cmd (0x01)*/
  bool writeRegisterWithOrMask(uint8_t addr, uint32_t mask);
  /*レジスタにANDマスク cmd (0x02)*/
  bool writeRegisterWithAndMask(uint8_t addr, uint32_t mask);
  /*レジスタ読み込み cmd (0x04)*/
  bool readRegister(uint8_t addr, uint32_t *val); //8*4=32を先に確保？
  /*EEPROM読み込み cmd (0x07)*/
  bool readEEPROM(uint8_t addr, uint8_t *buf, size_t len);
  /*タグへデータ転送 cmd (0x09)*/
  bool sendData(uint8_t *data, size_t len, uint8_t validBits = 0);  //最後の1Byteの有効bit数(0ならALL送信・上位ではなく下位(右))
  /*タグからデータ読み取り cmd (0x0a)*/
  uint8_t* readData(size_t len, uint8_t *buf = nullptr);
  /*RF(電波)設定のロード cmd (0x11)*/
  bool loadRFconfig(uint8_t txConf, uint8_t rxConf);
  /*RFオン cmd (0x16)*/
  bool setRF_on();
  /*Rfオフ cmd (0x17)*/
  bool setRF_off();
  /*-------------------------------------------*/

  /*----------ユーティリティ--------------------*/
  /*pn5180リセット*/
  void reset();
  /*割り込み状態の取得*/
  uint32_t getIRQStatus();
  /*割り込み状態の消去*/
  bool clearIRQStatus(uint32_t irqMask);
  /*送受信状態取得*/
  PN5180TransceiveStat getTRstate();
  /*------------------------------------------*/
private:
  /*Transceive Command(送受信用)*/
  bool trCmd(uint8_t *sendBuf, size_t sendBufLen, uint8_t *recvBuf = nullptr, size_t recvBufLen = 0);
};
//
// NAME: pn5180.cpp
//
// DESC: Implementation of PN5180 class.
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
// - Removed unnecessary functions.
// - Modified SPI communication routines.
// - Added and revised Japanese comments.
//

#include <sys/_stdint.h>
#include <sys/types.h>
#include <Arduino.h>
#include "pn5180.h"

uint8_t PN5180::readBuf[508];  //実体を作成(領域を取る)

/*コンストラクタ*/
PN5180::PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin) {
  PN5180_NSS = SSpin; //LOWで通信開始
  PN5180_BUSY = BUSYpin; //監視用
  PN5180_RST = RSTpin; //リセット用
  /*クロック周波数7MHz・最上位ビットから転送*/
  PN5180_SPI_Settings = SPISettings(7000000, MSBFIRST, SPI_MODE0);
}

/*-----ユーティリティ------*/
void PN5180::begin() {
  pinMode(PN5180_NSS, OUTPUT);
  pinMode(PN5180_BUSY, INPUT);
  pinMode(PN5180_RST, OUTPUT);

  digitalWrite(PN5180_NSS, HIGH);  //disable
  digitalWrite(PN5180_RST, HIGH);  //no reset

  SPI.begin();
}

void PN5180::end() {
  digitalWrite(PN5180_NSS, HIGH); //disable
  SPI.end();
}

/*レジスタ書き込み*/
bool PN5180::writeRegister(uint8_t addr, uint32_t val) {
  uint8_t *p = (uint8_t*)&val;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, addr, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(buf, 6);
  SPI.endTransaction();

  return true;
}

/*レジスタにORマスク*/
bool PN5180::writeRegisterWithOrMask(uint8_t addr, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, addr, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(buf, 6);
  SPI.endTransaction();

  return true;
}

/*レジスタにANDマスク*/
bool PN5180::writeRegisterWithAndMask(uint8_t addr, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, addr, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(buf, 6);
  SPI.endTransaction();

  return true;
}

/*レジスタ読み込み*/
bool PN5180::readRegister(uint8_t addr, uint32_t *val) { //8*4=32を先に確保？
  /*コマンド+addr*/
  uint8_t cmd[2] = { PN5180_READ_REGISTER, addr };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(cmd, 2, (uint8_t*)val, 4);
  SPI.endTransaction();

  return true;
}

/*EEPROM読み込み*/
bool PN5180::readEEPROM(uint8_t addr, uint8_t *buf, size_t len) {
  //EEPROMのアドレス範囲かの確認(datasheatより)
  if((addr > 254) || ((addr + len) > 255 || len == 0)) {
    return false;
  }

  /*コマンド+アドレス+data数*/
  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, static_cast<uint8_t>(len) };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(cmd, 3, buf, len);
  SPI.endTransaction();

  return true;
}

/*タグへデータ転送*/
bool PN5180::sendData(uint8_t *data, size_t len, uint8_t validBits /* = 0 */) {
  if(len > 260 || validBits > 7) return false; //1~260 datasheatに記載
  uint8_t buf[len+2];
  buf[0] = PN5180_SEND_DATA;  //先頭にコマンド
  buf[1] = validBits;         ////最後の1Byteの有効bit数(0ならALL送信・上位ではなく下位(右))
  for(int i = 0; i < len; i++) {
    buf[i + 2] = data[i];
  }
  
  writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  //通信状態をIdleに移行
  writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   //通信状態をTRモードに移行

  PN5180TransceiveStat transceiveState = getTRstate();
  if(transceiveState != PN5180_TS_WaitTransmit) return false;

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(buf, len+2);
  SPI.endTransaction();

  return true;
}

/*タグからデータ読み取り*/
uint8_t* PN5180::readData(size_t len, uint8_t *buf /*= nullptr*/) {
  if(len > 508) return 0L;   //0をlong型(4バイト)で返している
  if(buf == nullptr) buf = readBuf;   //bufが与えられなかった場合，readBufに格納する

  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };  //0x00はdatasheatで指定
  
  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(cmd, 2, buf, len);
  SPI.endTransaction();

  return buf;
}

/*RF(電波)設定のロード*/
bool PN5180::loadRFconfig(uint8_t txConf, uint8_t rxConf) {
  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(cmd, 3);
  SPI.endTransaction();

  return true;
}

/*RFオン*/
bool PN5180::setRF_on() {
  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(cmd, 2);
  SPI.endTransaction();

  while((getIRQStatus() & TX_RFON_IRQ_STAT) == 0);  //RFON割り込み状態になるまで待機
  clearIRQStatus(TX_RFON_IRQ_STAT); //割り込み状態のクリア
  return true;
}

/*Rfオフ*/
bool PN5180::setRF_off() {
  uint8_t cmd[2] = { PN5180_RF_OFF, 0x00 };

  SPI.beginTransaction(PN5180_SPI_Settings);
  trCmd(cmd, 2);
  SPI.endTransaction();

  while((getIRQStatus() & TX_RFOFF_IRQ_STAT) == 0); //RFOFF割り込み状態になるまで待機
  clearIRQStatus(TX_RFOFF_IRQ_STAT);
  return true;
}

/*----------ユーティリティ--------------------*/
/*pn5180リセット*/
void PN5180::reset() {
  digitalWrite(PN5180_RST, LOW);  //リセットするときは最低10μsのLOW
  delayMicroseconds(20); //20μs
  digitalWrite(PN5180_RST, HIGH); //待機が必要かは不明
  delay(2);              //2ms

  while((IDLE_IRQ_STAT & getIRQStatus()) == 0); //立ち上がり待機
  clearIRQStatus(0xffffffff);  //フラグのクリア
}

/*割り込み状態の取得*/
uint32_t PN5180::getIRQStatus() {
  uint32_t irqStatus;
  readRegister(IRQ_STATUS, &irqStatus);
  return irqStatus;
}

/*割り込み状態の消去*/
bool PN5180::clearIRQStatus(uint32_t irqMask) {
  return writeRegister(IRQ_CLEAR, irqMask);
}
/*送受信状態取得*/
PN5180TransceiveStat PN5180::getTRstate() {
  uint32_t rfStatus;
  if(!readRegister(RF_STATUS, &rfStatus)) {
    return PN5180TransceiveStat(0); //PN5180_TS_Idleを返す
  }
  /*0:31のうち26:24がTRStatus*/
  uint8_t state = ((rfStatus >> 24) & 0x07);  //上位8bitに&マスク0x07 -> 0000 0111
  
  return PN5180TransceiveStat(state); //0~7までのSTATUS
}

/*Transceive Command*/
bool PN5180::trCmd(uint8_t *sendBuf, size_t sendBufLen, uint8_t *recvBuf /* = nullptr */, size_t recvBufLen /* = 0 */) {
  /*送信部*/
  while(digitalRead(PN5180_BUSY) != LOW);   //PN5180準備完了待機
  digitalWrite(PN5180_NSS, LOW);  delay(2); //Enable
  for(int i = 0; i < sendBufLen; i++) {
    SPI.transfer(sendBuf[i]);               //送信
  }
  while(digitalRead(PN5180_BUSY) != HIGH);  //処理開始待機
  digitalWrite(PN5180_NSS, HIGH); delay(1); //disable
  while(digitalRead(PN5180_BUSY) != LOW);   //処理完了待機

  /*受信部*/
  /*受信する必要がないなら終了*/
  if((recvBuf == nullptr) || (recvBufLen == 0)) return true; 
  digitalWrite(PN5180_NSS, LOW);  delay(2); //Enable
  for(int i = 0; i < recvBufLen; i++) {
    recvBuf[i] = SPI.transfer(0xff);        //とりえあず1Byte送って受信する
  }
  while(digitalRead(PN5180_BUSY) != HIGH);  //処理開始待機
  digitalWrite(PN5180_NSS, HIGH); delay(1); //disable
  while(digitalRead(PN5180_BUSY) != LOW);   //処理完了待機

  return true;
}
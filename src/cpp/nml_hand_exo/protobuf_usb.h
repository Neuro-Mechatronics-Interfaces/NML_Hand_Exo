#pragma once
#ifndef EXO_USB_PROTOBUF
#define EXO_USB_PROTOBUF 0
#endif
#ifndef EXO_USB_PROTOBUF_V2
#define EXO_USB_PROTOBUF_V2 0
#endif
#if EXO_USB_PROTOBUF_V2
#undef EXO_USB_PROTOBUF
#define EXO_USB_PROTOBUF 1
#endif
#if EXO_USB_PROTOBUF
class NMLHandExo;
class GestureController;
class Adafruit_BNO055;
class String;
void pollProtobufUsb(NMLHandExo& exo, GestureController& gc);
void pollProtobufUsbV2(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu);
void protobufV2EmitEvent(const String& message);
#endif

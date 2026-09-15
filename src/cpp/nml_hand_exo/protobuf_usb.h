#pragma once
#ifndef EXO_USB_PROTOBUF
#define EXO_USB_PROTOBUF 0
#endif
#if EXO_USB_PROTOBUF
class NMLHandExo;
class GestureController;
void pollProtobufUsb(NMLHandExo& exo, GestureController& gc);
#endif

#pragma once

// One selector owns the USB transport choice. The default OpenRB build is
// the newest transport; non-OpenRB/SINGLE_CDC builds remain ASCII because
// Protobuf requires the OpenRB dual-CDC hardware.
#define EXO_USB_PROTOCOL_ASCII 0
#define EXO_USB_PROTOCOL_PROTOBUF_V1 1
#define EXO_USB_PROTOCOL_PROTOBUF_V2 2

#ifndef EXO_USB_PROTOCOL
  // Accept the old flags as compatibility aliases while callers migrate to
  // EXO_USB_PROTOCOL. An explicitly supplied old flag always wins over the
  // board default, including -DEXO_USB_PROTOBUF_V2=0 for legacy ASCII.
  #if defined(EXO_USB_PROTOBUF_V2)
    #if EXO_USB_PROTOBUF_V2
      #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_PROTOBUF_V2
    #elif defined(EXO_USB_PROTOBUF) && EXO_USB_PROTOBUF
      #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_PROTOBUF_V1
    #else
      #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_ASCII
    #endif
  #elif defined(EXO_USB_PROTOBUF)
    #if EXO_USB_PROTOBUF
      #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_PROTOBUF_V1
    #else
      #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_ASCII
    #endif
  #elif defined(ARDUINO_OpenRB) && !defined(SINGLE_CDC) && !EXO_AXON_USB
    #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_PROTOBUF_V2
  #else
    #define EXO_USB_PROTOCOL EXO_USB_PROTOCOL_ASCII
  #endif
#endif

#if EXO_USB_PROTOCOL < EXO_USB_PROTOCOL_ASCII || EXO_USB_PROTOCOL > EXO_USB_PROTOCOL_PROTOBUF_V2
  #error "EXO_USB_PROTOCOL must be 0 (ASCII), 1 (Protobuf-v1), or 2 (Protobuf-v2)"
#endif

// Compatibility aliases for external sketches and older build scripts. New
// firmware code should use EXO_USB_PROTOCOL and the named values above.
#ifdef EXO_USB_PROTOBUF
  #undef EXO_USB_PROTOBUF
#endif
#define EXO_USB_PROTOBUF (EXO_USB_PROTOCOL != EXO_USB_PROTOCOL_ASCII)
#ifdef EXO_USB_PROTOBUF_V2
  #undef EXO_USB_PROTOBUF_V2
#endif
#define EXO_USB_PROTOBUF_V2 (EXO_USB_PROTOCOL == EXO_USB_PROTOCOL_PROTOBUF_V2)

#if EXO_USB_PROTOCOL != EXO_USB_PROTOCOL_ASCII
class NMLHandExo;
class GestureController;
class Adafruit_BNO055;
class String;
void pollProtobufUsb(NMLHandExo& exo, GestureController& gc);
void pollProtobufUsbV2(NMLHandExo& exo, GestureController& gc, Adafruit_BNO055& imu);
void protobufV2EmitEvent(const String& message);
#endif

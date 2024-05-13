/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: LED_pb.proto */

#ifndef PROTOBUF_C_LED_5fpb_2eproto__INCLUDED
#define PROTOBUF_C_LED_5fpb_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _SwitchStatus SwitchStatus;
typedef struct _LEDControl LEDControl;
typedef struct _LEDStatus LEDStatus;


/* --- enums --- */


/* --- messages --- */

struct  _SwitchStatus
{
  ProtobufCMessage base;
  protobuf_c_boolean pressed;
};
#define SWITCH_STATUS__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&switch_status__descriptor) \
    , 0 }


struct  _LEDControl
{
  ProtobufCMessage base;
  protobuf_c_boolean led_on;
};
#define LEDCONTROL__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ledcontrol__descriptor) \
    , 0 }


struct  _LEDStatus
{
  ProtobufCMessage base;
  protobuf_c_boolean led1_on;
  protobuf_c_boolean led2_on;
};
#define LEDSTATUS__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ledstatus__descriptor) \
    , 0, 0 }


/* SwitchStatus methods */
void   switch_status__init
                     (SwitchStatus         *message);
size_t switch_status__get_packed_size
                     (const SwitchStatus   *message);
size_t switch_status__pack
                     (const SwitchStatus   *message,
                      uint8_t             *out);
size_t switch_status__pack_to_buffer
                     (const SwitchStatus   *message,
                      ProtobufCBuffer     *buffer);
SwitchStatus *
       switch_status__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   switch_status__free_unpacked
                     (SwitchStatus *message,
                      ProtobufCAllocator *allocator);
/* LEDControl methods */
void   ledcontrol__init
                     (LEDControl         *message);
size_t ledcontrol__get_packed_size
                     (const LEDControl   *message);
size_t ledcontrol__pack
                     (const LEDControl   *message,
                      uint8_t             *out);
size_t ledcontrol__pack_to_buffer
                     (const LEDControl   *message,
                      ProtobufCBuffer     *buffer);
LEDControl *
       ledcontrol__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ledcontrol__free_unpacked
                     (LEDControl *message,
                      ProtobufCAllocator *allocator);
/* LEDStatus methods */
void   ledstatus__init
                     (LEDStatus         *message);
size_t ledstatus__get_packed_size
                     (const LEDStatus   *message);
size_t ledstatus__pack
                     (const LEDStatus   *message,
                      uint8_t             *out);
size_t ledstatus__pack_to_buffer
                     (const LEDStatus   *message,
                      ProtobufCBuffer     *buffer);
LEDStatus *
       ledstatus__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ledstatus__free_unpacked
                     (LEDStatus *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*SwitchStatus_Closure)
                 (const SwitchStatus *message,
                  void *closure_data);
typedef void (*LEDControl_Closure)
                 (const LEDControl *message,
                  void *closure_data);
typedef void (*LEDStatus_Closure)
                 (const LEDStatus *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor switch_status__descriptor;
extern const ProtobufCMessageDescriptor ledcontrol__descriptor;
extern const ProtobufCMessageDescriptor ledstatus__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_LED_5fpb_2eproto__INCLUDED */

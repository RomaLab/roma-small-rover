#ifndef _GENERIC_H_
#define _GENERIC_H_

/* Generated by WriteClassHeaderVisitor: Wed Apr 28 2021 17:24:23 GMT-0600 (Mountain Daylight Time) */

#ifdef INCLUDE_PRIVATE
typedef struct _PhidgetGeneric *PhidgetGenericHandle;

/* Methods */
API_PRETURN_HDR PhidgetGeneric_create(PhidgetGenericHandle *ch);
API_PRETURN_HDR PhidgetGeneric_delete(PhidgetGenericHandle *ch);
API_PRETURN_HDR PhidgetGeneric_readChannelPacket(PhidgetGenericHandle ch, int packetType, int index,
  uint8_t *packet, size_t *packetLen);
API_PRETURN_HDR PhidgetGeneric_readDevicePacket(PhidgetGenericHandle ch, int packetType, uint8_t *packet,
  size_t *packetLen);
API_PRETURN_HDR PhidgetGeneric_sendChannelPacket(PhidgetGenericHandle ch, int packetType, int index,
  const uint8_t *packet, size_t packetLen);
API_VRETURN_HDR PhidgetGeneric_sendChannelPacket_async(PhidgetGenericHandle ch, int packetType, int index,
  const uint8_t *packet, size_t packetLen, Phidget_AsyncCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetGeneric_sendDevicePacket(PhidgetGenericHandle ch, int packetType,
  const uint8_t *packet, size_t packetLen);
API_VRETURN_HDR PhidgetGeneric_sendDevicePacket_async(PhidgetGenericHandle ch, int packetType,
  const uint8_t *packet, size_t packetLen, Phidget_AsyncCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetGeneric_sendPacket(PhidgetGenericHandle ch, const uint8_t *packet,
  size_t packetLen);
API_VRETURN_HDR PhidgetGeneric_sendPacket_async(PhidgetGenericHandle ch, const uint8_t *packet,
  size_t packetLen, Phidget_AsyncCallback fptr, void *ctx);

/* Properties */
API_PRETURN_HDR PhidgetGeneric_getINPacketLength(PhidgetGenericHandle ch, uint32_t *INPacketLength);
API_PRETURN_HDR PhidgetGeneric_getOUTPacketLength(PhidgetGenericHandle ch, uint32_t *OUTPacketLength);

/* Events */
typedef void (CCONV *PhidgetGeneric_OnPacketCallback)(PhidgetGenericHandle ch, void *ctx,
  const uint8_t *packet, size_t packetLen);

API_PRETURN_HDR PhidgetGeneric_setOnPacketHandler(PhidgetGenericHandle ch,
  PhidgetGeneric_OnPacketCallback fptr, void *ctx);

#endif /* INCLUDE_PRIVATE */
#endif /* _GENERIC_H_ */

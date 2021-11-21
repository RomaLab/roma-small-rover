/* Generated: Thu Feb 04 2016 10:58:25 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/spatial.gen.h"
#include "class/spatial.gen.c"

static void CCONV
PhidgetSpatial_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetSpatial_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetSpatial_create(PhidgetSpatialHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetSpatial_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetSpatialHandle ch;

	TESTPTR(phid);
	ch = (PhidgetSpatialHandle)phid;

	switch (bp->vpkt) {
	case BP_DATAINTERVALCHANGE:
		ch->dataInterval = getBridgePacketUInt32(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "DataInterval");
		break;
	}

	return (_bridgeInput(phid, bp));
}

static void
PhidgetSpatial_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetSpatial_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

/* Generated: Wed Apr 28 2021 17:24:23 GMT-0600 (Mountain Daylight Time) */

#include "device/encoderdevice.h"
#include "device/motorcontroldevice.h"
static void CCONV PhidgetEncoder_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetEncoder_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetEncoder_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetEncoder_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetEncoder_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetEncoder_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetEncoder_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetEncoder_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetEncoder_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetEncoder {
	struct _PhidgetChannel phid;
	int enabled;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	int64_t indexPosition;
	Phidget_EncoderIOMode IOMode;
	int64_t position;
	uint32_t positionChangeTrigger;
	uint32_t minPositionChangeTrigger;
	uint32_t maxPositionChangeTrigger;
	PhidgetEncoder_OnPositionChangeCallback PositionChange;
	void *PositionChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetEncoderHandle ch;
	int version;

	ch = (PhidgetEncoderHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}

	if(version >= 0)
		ch->enabled = getBridgePacketInt32ByName(bp, "enabled");
	if(version >= 0)
		ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	if(version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if(version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if(version >= 0)
		ch->indexPosition = getBridgePacketInt64ByName(bp, "indexPosition");
	if(version >= 0)
		ch->IOMode = getBridgePacketInt32ByName(bp, "IOMode");
	if(version >= 0)
		ch->position = getBridgePacketInt64ByName(bp, "position");
	if(version >= 1)
		ch->positionChangeTrigger = getBridgePacketUInt32ByName(bp, "positionChangeTrigger");
	if(version >= 1)
		ch->minPositionChangeTrigger = getBridgePacketUInt32ByName(bp, "minPositionChangeTrigger");
	if(version >= 1)
		ch->maxPositionChangeTrigger = getBridgePacketUInt32ByName(bp, "maxPositionChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetEncoderHandle ch;

	ch = (PhidgetEncoderHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",enabled=%d"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",indexPosition=%l"
	  ",IOMode=%d"
	  ",position=%l"
	  ",positionChangeTrigger=%u"
	  ",minPositionChangeTrigger=%u"
	  ",maxPositionChangeTrigger=%u"
	  ,1 /* class version */
	  ,ch->enabled
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->indexPosition
	  ,ch->IOMode
	  ,ch->position
	  ,ch->positionChangeTrigger
	  ,ch->minPositionChangeTrigger
	  ,ch->maxPositionChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetEncoderHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetEncoderHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETENABLED:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->enabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Enabled");
		break;
	case BP_SETDATAINTERVAL:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDataInterval,
		  ch->maxDataInterval);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->dataInterval = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "DataInterval");
		break;
	case BP_SETIOMODE:
		if (!supportedEncoderIOMode(phid, (Phidget_EncoderIOMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified EncoderIOMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->IOMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "IOMode");
		break;
	case BP_SETCHANGETRIGGER:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minPositionChangeTrigger,
		  ch->maxPositionChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->positionChangeTrigger = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "PositionChangeTrigger");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetEncoderHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetEncoderHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_1047_ENCODER_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_1047_ENCODER_200:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_1052_ENCODER_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_1052_ENCODER_101:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_1052_ENCODER_110:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_1057_ENCODER_300:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_1057_ENCODER_400:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		break;
	case PHIDCHUID_1065_ENCODER_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 10000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DCC1000_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		break;
	case PHIDCHUID_DCC1000_ENCODER_200:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		break;
	case PHIDCHUID_DCC1000_ENCODER_210:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		break;
	case PHIDCHUID_DCC1004_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DCC1005_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DCC1001_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 50;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DCC1001_ENCODER_120:
		ch->dataInterval = 250;
		ch->minDataInterval = 50;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DCC1002_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 50;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DCC1002_ENCODER_110:
		ch->dataInterval = 250;
		ch->minDataInterval = 50;
		ch->maxDataInterval = 60000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	case PHIDCHUID_ENC1000_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->indexPosition = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		break;
	case PHIDCHUID_HIN1101_ENCODER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 1000;
		ch->maxPositionChangeTrigger = 60000000;
		ch->minPositionChangeTrigger = 0;
		ch->position = 0;
		ch->positionChangeTrigger = 0;
		ch->enabled = 1;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetEncoderHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetEncoderHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1047_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->enabled);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1047_ENCODER_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->enabled);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1052_ENCODER_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1052_ENCODER_101:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1052_ENCODER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1057_ENCODER_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1057_ENCODER_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->enabled);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETIOMODE, NULL, NULL, "%d", ch->IOMode);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_1065_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_DCC1000_ENCODER_100:
		break;
	case PHIDCHUID_DCC1000_ENCODER_200:
		break;
	case PHIDCHUID_DCC1000_ENCODER_210:
		break;
	case PHIDCHUID_DCC1004_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_DCC1005_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_DCC1001_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_DCC1001_ENCODER_120:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_DCC1002_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_DCC1002_ENCODER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_ENC1000_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->enabled);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETIOMODE, NULL, NULL, "%d", ch->IOMode);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_HIN1101_ENCODER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->positionChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->enabled);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetEncoder));
}

static PhidgetReturnCode CCONV
_create(PhidgetEncoderHandle *phidp) {

	CHANNELCREATE_BODY(Encoder, PHIDCHCLASS_ENCODER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_delete(PhidgetEncoderHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetEncoder_setEnabled(PhidgetEncoderHandle ch, int enabled) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENABLED, NULL, NULL, "%d", enabled));
}

API_PRETURN
PhidgetEncoder_getEnabled(PhidgetEncoderHandle ch, int *enabled) {

	TESTPTR_PR(ch);
	TESTPTR_PR(enabled);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*enabled = ch->enabled;
	if (ch->enabled == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_setDataInterval(PhidgetEncoderHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetEncoder_getDataInterval(PhidgetEncoderHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_getMinDataInterval(PhidgetEncoderHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_getMaxDataInterval(PhidgetEncoderHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_getIndexPosition(PhidgetEncoderHandle ch, int64_t *indexPosition) {

	TESTPTR_PR(ch);
	TESTPTR_PR(indexPosition);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1052_ENCODER_000:
	case PHIDCHUID_1052_ENCODER_101:
	case PHIDCHUID_1052_ENCODER_110:
	case PHIDCHUID_1057_ENCODER_300:
	case PHIDCHUID_HIN1101_ENCODER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*indexPosition = ch->indexPosition;
	if (ch->indexPosition == (int64_t)PUNK_INT64)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_setIOMode(PhidgetEncoderHandle ch, Phidget_EncoderIOMode IOMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETIOMODE, NULL, NULL, "%d", IOMode));
}

API_PRETURN
PhidgetEncoder_getIOMode(PhidgetEncoderHandle ch, Phidget_EncoderIOMode *IOMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(IOMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1047_ENCODER_100:
	case PHIDCHUID_1047_ENCODER_200:
	case PHIDCHUID_1052_ENCODER_000:
	case PHIDCHUID_1052_ENCODER_101:
	case PHIDCHUID_1052_ENCODER_110:
	case PHIDCHUID_1057_ENCODER_300:
	case PHIDCHUID_1065_ENCODER_100:
	case PHIDCHUID_DCC1004_ENCODER_100:
	case PHIDCHUID_DCC1005_ENCODER_100:
	case PHIDCHUID_DCC1001_ENCODER_100:
	case PHIDCHUID_DCC1001_ENCODER_120:
	case PHIDCHUID_DCC1002_ENCODER_100:
	case PHIDCHUID_DCC1002_ENCODER_110:
	case PHIDCHUID_HIN1101_ENCODER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*IOMode = ch->IOMode;
	if (ch->IOMode == (Phidget_EncoderIOMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_getPosition(PhidgetEncoderHandle ch, int64_t *position) {

	TESTPTR_PR(ch);
	TESTPTR_PR(position);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*position = ch->position;
	if (ch->position == (int64_t)PUNK_INT64)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_setPositionChangeTrigger(PhidgetEncoderHandle ch, uint32_t positionChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
	  positionChangeTrigger));
}

API_PRETURN
PhidgetEncoder_getPositionChangeTrigger(PhidgetEncoderHandle ch, uint32_t *positionChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(positionChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*positionChangeTrigger = ch->positionChangeTrigger;
	if (ch->positionChangeTrigger == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_getMinPositionChangeTrigger(PhidgetEncoderHandle ch,
  uint32_t *minPositionChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minPositionChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*minPositionChangeTrigger = ch->minPositionChangeTrigger;
	if (ch->minPositionChangeTrigger == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_getMaxPositionChangeTrigger(PhidgetEncoderHandle ch,
  uint32_t *maxPositionChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxPositionChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	*maxPositionChangeTrigger = ch->maxPositionChangeTrigger;
	if (ch->maxPositionChangeTrigger == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetEncoder_setOnPositionChangeHandler(PhidgetEncoderHandle ch,
  PhidgetEncoder_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);

	ch->PositionChange = fptr;
	ch->PositionChangeCtx = ctx;

	return (EPHIDGET_OK);
}

/* Generated: Wed Apr 28 2021 17:24:23 GMT-0600 (Mountain Daylight Time) */

#include "device/phsensordevice.h"
static void CCONV PhidgetPHSensor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetPHSensor_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetPHSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPHSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPHSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetPHSensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetPHSensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetPHSensor_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetPHSensor_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetPHSensor {
	struct _PhidgetChannel phid;
	double correctionTemperature;
	double minCorrectionTemperature;
	double maxCorrectionTemperature;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double PH;
	double minPH;
	double maxPH;
	double PHChangeTrigger;
	double minPHChangeTrigger;
	double maxPHChangeTrigger;
	PhidgetPHSensor_OnPHChangeCallback PHChange;
	void *PHChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPHSensorHandle ch;
	int version;

	ch = (PhidgetPHSensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 0 - functionality may be limited.", phid, version);
	}

	if(version >= 0)
		ch->correctionTemperature = getBridgePacketDoubleByName(bp, "correctionTemperature");
	if(version >= 0)
		ch->minCorrectionTemperature = getBridgePacketDoubleByName(bp, "minCorrectionTemperature");
	if(version >= 0)
		ch->maxCorrectionTemperature = getBridgePacketDoubleByName(bp, "maxCorrectionTemperature");
	if(version >= 0)
		ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	if(version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if(version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if(version >= 0)
		ch->PH = getBridgePacketDoubleByName(bp, "PH");
	if(version >= 0)
		ch->minPH = getBridgePacketDoubleByName(bp, "minPH");
	if(version >= 0)
		ch->maxPH = getBridgePacketDoubleByName(bp, "maxPH");
	if(version >= 0)
		ch->PHChangeTrigger = getBridgePacketDoubleByName(bp, "PHChangeTrigger");
	if(version >= 0)
		ch->minPHChangeTrigger = getBridgePacketDoubleByName(bp, "minPHChangeTrigger");
	if(version >= 0)
		ch->maxPHChangeTrigger = getBridgePacketDoubleByName(bp, "maxPHChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetPHSensorHandle ch;

	ch = (PhidgetPHSensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",correctionTemperature=%g"
	  ",minCorrectionTemperature=%g"
	  ",maxCorrectionTemperature=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",PH=%g"
	  ",minPH=%g"
	  ",maxPH=%g"
	  ",PHChangeTrigger=%g"
	  ",minPHChangeTrigger=%g"
	  ",maxPHChangeTrigger=%g"
	  ,0 /* class version */
	  ,ch->correctionTemperature
	  ,ch->minCorrectionTemperature
	  ,ch->maxCorrectionTemperature
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->PH
	  ,ch->minPH
	  ,ch->maxPH
	  ,ch->PHChangeTrigger
	  ,ch->minPHChangeTrigger
	  ,ch->maxPHChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPHSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetPHSensorHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETCORRECTIONTEMPERATURE:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minCorrectionTemperature,
		  ch->maxCorrectionTemperature);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->correctionTemperature = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "CorrectionTemperature");
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
	case BP_SETCHANGETRIGGER:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minPHChangeTrigger,
		  ch->maxPHChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->PHChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "PHChangeTrigger");
		break;
	case BP_PHCHANGE:
		ch->PH = getBridgePacketDouble(bp, 0);
		FIRECH(ch, PHChange, ch->PH);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetPHSensorDeviceHandle parentPHSensor;
	PhidgetPHSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetPHSensorHandle)phid;

	ret = EPHIDGET_OK;

	parentPHSensor = (PhidgetPHSensorDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1058_PHADAPTER_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxPH = 14;
		ch->maxPHChangeTrigger = 14;
		ch->minDataInterval = 80;
		ch->minPH = 0;
		ch->minPHChangeTrigger = 0;
		ch->PH = parentPHSensor->PH[ch->phid.index];
		ch->PHChangeTrigger = 0;
		ch->correctionTemperature = 25;
		ch->minCorrectionTemperature = 0;
		ch->maxCorrectionTemperature = 100;
		break;
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxPH = 14;
		ch->maxPHChangeTrigger = 14;
		ch->minDataInterval = 50;
		ch->minPH = 0;
		ch->minPHChangeTrigger = 0;
		ch->PH = PUNK_DBL;
		ch->PHChangeTrigger = 0;
		ch->correctionTemperature = 25;
		ch->minCorrectionTemperature = 0;
		ch->maxCorrectionTemperature = 100;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetPHSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetPHSensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1058_PHADAPTER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g", ch->PHChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCORRECTIONTEMPERATURE, NULL, NULL, "%g",
		  ch->correctionTemperature);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g", ch->PHChangeTrigger);
		if (ret != EPHIDGET_OK) {
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCORRECTIONTEMPERATURE, NULL, NULL, "%g",
		  ch->correctionTemperature);
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
	PhidgetPHSensorHandle ch;

	ch = (PhidgetPHSensorHandle)phid;

	if(ch->PH != PUNK_DBL)
		FIRECH(ch, PHChange, ch->PH);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetPHSensorHandle ch;

	ch = (PhidgetPHSensorHandle)phid;

	if(ch->PH == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetPHSensor));
}

static PhidgetReturnCode CCONV
_create(PhidgetPHSensorHandle *phidp) {

	CHANNELCREATE_BODY(PHSensor, PHIDCHCLASS_PHSENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_delete(PhidgetPHSensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetPHSensor_setCorrectionTemperature(PhidgetPHSensorHandle ch, double correctionTemperature) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCORRECTIONTEMPERATURE, NULL, NULL, "%g",
	  correctionTemperature));
}

API_PRETURN
PhidgetPHSensor_getCorrectionTemperature(PhidgetPHSensorHandle ch, double *correctionTemperature) {

	TESTPTR_PR(ch);
	TESTPTR_PR(correctionTemperature);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*correctionTemperature = ch->correctionTemperature;
	if (ch->correctionTemperature == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinCorrectionTemperature(PhidgetPHSensorHandle ch,
  double *minCorrectionTemperature) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minCorrectionTemperature);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*minCorrectionTemperature = ch->minCorrectionTemperature;
	if (ch->minCorrectionTemperature == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxCorrectionTemperature(PhidgetPHSensorHandle ch,
  double *maxCorrectionTemperature) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxCorrectionTemperature);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*maxCorrectionTemperature = ch->maxCorrectionTemperature;
	if (ch->maxCorrectionTemperature == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_setDataInterval(PhidgetPHSensorHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetPHSensor_getDataInterval(PhidgetPHSensorHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinDataInterval(PhidgetPHSensorHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxDataInterval(PhidgetPHSensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getPH(PhidgetPHSensorHandle ch, double *PH) {

	TESTPTR_PR(ch);
	TESTPTR_PR(PH);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*PH = ch->PH;
	if (ch->PH == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinPH(PhidgetPHSensorHandle ch, double *minPH) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minPH);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*minPH = ch->minPH;
	if (ch->minPH == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxPH(PhidgetPHSensorHandle ch, double *maxPH) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxPH);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*maxPH = ch->maxPH;
	if (ch->maxPH == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_setPHChangeTrigger(PhidgetPHSensorHandle ch, double PHChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  PHChangeTrigger));
}

API_PRETURN
PhidgetPHSensor_getPHChangeTrigger(PhidgetPHSensorHandle ch, double *PHChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(PHChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*PHChangeTrigger = ch->PHChangeTrigger;
	if (ch->PHChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinPHChangeTrigger(PhidgetPHSensorHandle ch, double *minPHChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minPHChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*minPHChangeTrigger = ch->minPHChangeTrigger;
	if (ch->minPHChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxPHChangeTrigger(PhidgetPHSensorHandle ch, double *maxPHChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxPHChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHED_PR(ch);

	*maxPHChangeTrigger = ch->maxPHChangeTrigger;
	if (ch->maxPHChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_setOnPHChangeHandler(PhidgetPHSensorHandle ch, PhidgetPHSensor_OnPHChangeCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_PHSENSOR);

	ch->PHChange = fptr;
	ch->PHChangeCtx = ctx;

	return (EPHIDGET_OK);
}

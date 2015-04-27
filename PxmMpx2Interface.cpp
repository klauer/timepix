
#include "PxmMpx2Interface.h"

PxmMpx2Interface::PxmMpx2Interface(Mpx2Interface *mpx)
{
    findDevices=mpx->findDevices;
    init=mpx->init;
    closeDevice=mpx->closeDevice;
    setCallback=mpx->setCallback;
    setCallbackData=mpx->setCallbackData;
    getHwInfoCount=mpx->getHwInfoCount;
    getHwInfoFlags=mpx->getHwInfoFlags;
    getHwInfo=mpx->getHwInfo;
    setHwInfo=mpx->setHwInfo;
    getDevInfo=mpx->getDevInfo;
    reset=mpx->reset;
    setDACs=mpx->setDACs;
    getMpxDacVal=mpx->getMpxDacVal;
    setExtDacVal=mpx->setExtDacVal;
    setPixelsCfg=mpx->setPixelsCfg;
    setAcqPars=mpx->setAcqPars;
    startAcquisition=mpx->startAcquisition;
    stopAcquisition=mpx->stopAcquisition;
    getAcqTime=mpx->getAcqTime;
    resetMatrix=mpx->resetMatrix;
    readMatrix=mpx->readMatrix;
    writeMatrix=mpx->writeMatrix;
    sendTestPulses=mpx->sendTestPulses;
    isBusy=mpx->isBusy;
    getLastError=mpx->getLastError;
    getLastDevError=mpx->getLastDevError;
    chipCount=mpx->chipCount;
    setPixelsCfgF=mpx->setPixelsCfgF;
    setFsr=mpx->setFsr;
    enableTimer=mpx->enableTimer;
    timerExpired=mpx->timerExpired;
    openShutter=mpx->openShutter;
    closeShutter=mpx->closeShutter;
    readMatrixRaw=mpx->readMatrixRaw;
    readReg=mpx->readReg;
    writeReg=mpx->writeReg;
    newFrame=mpx->newFrame;
    stream2Data=mpx->stream2Data;
    parStream2Data=mpx->parStream2Data;
}

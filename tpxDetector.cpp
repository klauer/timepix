/* tpxDetector.cpp
 *
 * This is a driver for a ASI QTPX-262k and STPX-65k pixel array detectors.
 *
 * Author: Dmitry Byelov
 *          ASI
 *
 * Created:  July 5, 2013
 *
 */

#include <sys/stat.h>

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <unistd.h>
#include <time.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsExport.h>

#include "ADDriver.h"

#include "mpxmodule.h"
#include "limits.h"
#include <dlfcn.h>
#include <glbgvm.h>


//______________________________________________________________________________________________

static const char *driverName = "tpxDetector";

//______________________________________________________________________________________________



#define MAX_BAD_PIXELS 100
#define MAX_MPX_DEV_COUNT 512
//
#define RAW_DATA_READOUT

//#define DEV 3

// QTPX
#define DIM_X 512
#define DIM_Y 512
// comment above two lines when using STPX

// STPX
//#define SINGLE
//#define DIM_X 256
//#define DIM_Y 256
/////////////////////////////////////

#define PREVIEW_UPDATE_PERIOD 500 //[ms]
#define PATH_STRING_LEN 1024

#define BIT23           0x800000
#define BIT24           0x1000000

typedef enum {
    TPX_ACQUIRE_ACQUISITION,
} TPXAcquisitionMode_t;


typedef enum {
    TPXImageSingle     = ADImageSingle,
    TPXImageMultiple   = ADImageMultiple,
    TPXImageContinuous = ADImageContinuous,
} TPXImageMode_t;

typedef enum {
    TPX_INTERNAL_TRIGGER,
    TPX_EXTERNAL_TRIGGER
} TPXTimingMode_t;

typedef enum {
    TPX_STATUS_OK,
    TPX_STATUS_NOTLOADED,
    TPX_STATUS_INITIALIZING,
    TPX_STATUS_NOTINITIALIZED,
    TPX_STATUS_ERROR
} TPXStatus_t;

typedef enum {
    NOT_AVAILABLE,
    AVAILABLE
} Avalability_t;

typedef enum {
    NO,
    YES
} YesNo_t;

//______________________________________________________________________________________________

class tpxDetector;

/** Driver for the Timepix detectors */

class tpxDetector : public ADDriver {
  public:
    tpxDetector(const char *portName, int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    // These should really be private, but they are called from C so must be public
    // void endFrameCallback(HACQDESC hAcqDesc);
    // void endAcqCallback(HACQDESC hAcqDesc);

    epicsThreadId acquireTaskId;
    epicsThreadId statusTaskId;
    //epicsThreadId SoPhyTaskId;

    void acquireTask(void);
    //void SoPhyThread(void);
//    void testThread();
    ~tpxDetector();

    // MpxModule *relaxd;
    // short* myarray;

    /** Opens data file for storing of the raw data.
     * The file is opened in the directory with baseName extended by ID number.
     * It increases the ID untill it finds file name that does not exist yet.
     * Return 0 on success
     */
    int openRawDataFile(char* directory, char* baseName);
    int closeRawDataFile();
    int writeRawData(u8* datai, u32 nbytes, int lostRows);

    u32 checkStatus(void);
    u32 checkConfiguration(void);

protected:
    // Re-orders pixels from the MPX native order (256*(4*256)) to 512*512 in case of quad
    void reorderPixels(epicsInt16 *pInput, epicsInt16 *pOutput);
    void copyPixels(epicsInt16 *pInput, epicsInt16 *pOutput);
    void extendPixels(epicsInt16 *pInput, epicsInt16 *pOutput);
    void setupTriggering(bool internal);

    int TPX_SystemID;
#define TPX_FIRST_PARAM TPX_SystemID
    int TPX_Initialize;
    int TPX_InitializeRBV;


    //int TPX_StartSoPhy;
    //int TPX_StartSoPhyRBV;
    int TPX_DevIp;
    int TPX_DevIpRBV;


    int TPX_StatusRBV;
    int TPX_NumFrameBuffers;
    int TPX_NumFrameBuffersRBV;
    int TPX_SyncMode;
    int TPX_SyncModeRBV;
    int TPX_Trigger;
    int TPX_SyncTime;
    int TPX_SyncTimeRBV;
    int TPX_CorrectionsDirectory;

    int TPX_DataSaveDirectory;
    int TPX_DataFilePrefix;

    int TPX_FrameBufferIndex;
    int TPX_ImageNumber;
    int TPX_HWFile;

/////////////////////////////////////////////////////////////////////////
    int TPX_SaveToFileYes;
    int TPX_SaveToFileNo;
    int TPX_SaveToFile;
    int TPX_SaveToFileRBV;
////////////////////////////////////////////////////////////////////////
    int TPX_ExtendedFrameYes;
    int TPX_ExtendedFrameNo;
    int TPX_ExtendedFrame;
    int TPX_ExtendedFrameRBV;
    ////////////////////////////////////////////////////////////////////////
    int TPX_LoadDACFile;
    int TPX_resetDetector;
    int TPX_DACFile;
    int TPX_DACAvailable;
    int TPX_DACRBV;
    int TPX_PixConfigFile;
#define TPX_LAST_PARAM TPX_PixConfigFile
    bool writeRawDataFlag;
    unsigned long lastPreviewUpdate;
    bool isTimeForPreviewUpdate();

private:
    u32 lastStatus_;
    u32 lastConfiguration_;

    int mark;
//   MpxModule *relaxd;
//    EpicsInterface *relaxd;
    short* myarray;
    NDArray *pMatrix;
    int* TDD;
    int ndevs;
    char cpFileName_PC[256];
    char cpFileName_HW[256];
    int ids[MAX_MPX_DEV_COUNT];
    int devnum;

    int raw_frame_num;

    epicsEventId startEventId;
    epicsEventId stopEventId;

    // epicsEventId  acquireStopEvent_;
    epicsInt16   *pAcqBuffer_;
    epicsInt16   *pAcqBufferExt_;

    int           *pPixelCorrectionList_;
    unsigned int  uiNumFrameBuffers_;

    unsigned int  uiDevIp_;

    unsigned int  uiNumBuffersInUse_;
    int           iAcqMode_;
    // Keep a copy of parmeters for Acquistion_EnumSensors
    unsigned int  uiNumSensors_;
    bool          bInitAlways_;
    // Keep a copy of parmeters for Acquisition_GetCommChannel
    unsigned int  uiChannelType_;
    int           iChannelNum_;
    // Keep a copy of parmeters from Acquisition_GetConfiguration
    unsigned int  uiDevFrames_;
    unsigned int  uiRows_;
    unsigned int  uiColumns_;
    unsigned int  uiRowsAdd_;
    unsigned int  uiColumnsAdd_;
    unsigned int  uiDataType_;
    unsigned int  uiSortFlags_;
    BOOL          bEnableIRQ_;
    double        dAcqTimeReq_;
    double        dAcqTimeAct_;
    int           iTrigModeReq_;
    int           iTrigModeAct_;

    bool initializeDetector (void);
    bool setDetectorSet(void);
    void resetDetector(void);
    //bool StartSoPhy (void);
    bool testShoot(void);
    void setBinning(void);
    void loadDACsettings (void);
    void saveTofileYes (void);
    void saveTofileNo (void);
    void extendFrameYes (void);
    void extendFrameNo (void);

    asynStatus setTriggerMode(void);
    asynStatus setExposureTime(void);

    FILE *rawDataFile;
    bool driverReady;
};

//______________________________________________________________________________________________
#define TPX_SystemIDString                   "TPX_SYSTEMID"
#define TPX_InitializeString                 "TPX_INITIALIZE"
#define TPX_InitializeRBVString              "TPX_INITIALIZE_RBV"

//#define TPX_StartSoPhyString                 "TPX_STARTSOPHY"
//#define TPX_StartSoPhyRBVString              "TPX_STARTSOPHY_RBV"
#define TPX_DevIpString                      "TPX_DEV_IP"
#define TPX_DevIpRBVString                   "TPX_DEV_IP_RBV"



#define TPX_StatusRBVString                  "TPX_STATUS_RBV"
#define TPX_NumFrameBuffersString            "TPX_NUM_FRAME_BUFFERS"
#define TPX_NumFrameBuffersRBVString         "TPX_NUM_FRAME_BUFFERS_RBV"
#define TPX_SyncModeString                   "TPX_SYNC_MODE"
#define TPX_SyncModeRBVString                "TPX_SYNC_MODE_RBV"
#define TPX_TriggerString                    "TPX_TRIGGER"
#define TPX_SyncTimeString                   "TPX_SYNC_TIME"
#define TPX_SyncTimeRBVString                "TPX_SYNC_TIME_RBV"
#define TPX_CorrectionsDirectoryString       "TPX_CORRECTIONS_DIRECTORY"
#define TPX_FrameBufferIndexString           "TPX_FRAME_BUFFER_INDEX"
#define TPX_ImageNumberString                "TPX_IMAGE_NUMBER"
#define TPX_DataSaveDirectoryString          "TPX_DATASAVE_DIRECTORY"
#define TPX_DataFilePrefixString	     "TPX_DATAFILE_PREFIX"
#define TPX_HWFileString        	     "TPX_HW_FILE"
//////////////////////////////////////////////////////////////////////////////////
#define TPX_SaveToFileYesString		     "TPX_SAVE_TO_FILEYES"
#define TPX_SaveToFileNoString		     "TPX_SAVE_TO_FILENO"
#define TPX_SaveToFileString		     "TPX_SAVE_TO_FILE"
#define TPX_SaveToFileRBVString		     "TPX_SAVE_TO_FILE_RBV"
//////////////////////////////////////////////////////////////////////////////////
#define TPX_ExtendedFrameYesString		         "TPX_EXTENDED_FRAME_YES"
#define TPX_ExtendedFrameNoString		         "TPX_EXTENDED_FRAME_NO"
#define TPX_ExtendedFrameString		             "TPX_EXTENDED_FRAME"
#define TPX_ExtendedFrameRBVString		         "TPX_EXTENDED_FRAME_RBV"
//////////////////////////////////////////////////////////////////////////////////
#define TPX_resetDetectorString     	 "TPX_RESET_DETECTOR"
#define TPX_LoadDACFileString        	 "TPX_LOAD_DAC_FILE"
#define TPX_DACFileString		         "TPX_DAC_FILE"
#define TPX_DACAvailableString		     "TPX_DAC_AVAILABLE"
#define TPX_PixConfigFileString	         "TPX_PIX_CONFIG_FILE"
#define TPX_DACRBVString		         "TPX_DAC_RBV"

#define NUM_TPX_PARAMS (&TPX_LAST_PARAM - &TPX_FIRST_PARAM + 1)
//______________________________________________________________________________________________


static void acquireTaskC(void *drvPvt);
static void exitCallbackC(void *drvPvt);
static void statusCheckTask(void *drvPvt);
//static void SoPhyThreadC(void *drvPvt);
//_____________________________________________________________________________________________

extern "C" int tpxDetectorConfig(const char *portName, int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize ) {
    new tpxDetector(portName, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

//_____________________________________________________________________________________________
/** Constructor for this driver */
tpxDetector::tpxDetector(const char *portName, int maxBuffers,
                         size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, (int)NUM_TPX_PARAMS, maxBuffers, maxMemory, 0, 0, ASYN_CANBLOCK, 1, priority, stackSize), 
        writeRawDataFlag(false), lastPreviewUpdate(0),
        ndevs(0), devnum(0), rawDataFile(NULL), driverReady(false) {
    int status = asynSuccess;
    static const char *functionName = "tpxDetector";
    
    lastStatus_ = 0;
    lastConfiguration_ = 0;
    pAcqBuffer_           = NULL;
    pAcqBufferExt_           = NULL;
    bInitAlways_          = false;
    uiRows_=DIM_X;
    uiColumns_=DIM_Y;

    /* Add parameters for this driver */
    createParam(TPX_SystemIDString,                    asynParamInt32,   &TPX_SystemID);
    createParam(TPX_InitializeString,                  asynParamInt32,   &TPX_Initialize);
    createParam(TPX_InitializeRBVString,               asynParamInt32,   &TPX_InitializeRBV);

    //createParam(TPX_StartSoPhyString,                  asynParamInt32,   &TPX_StartSoPhy);
    //createParam(TPX_StartSoPhyRBVString,               asynParamInt32,   &TPX_StartSoPhyRBV);
    createParam(TPX_DevIpString,                       asynParamInt32,   &TPX_DevIp);
    createParam(TPX_DevIpRBVString,                    asynParamInt32,   &TPX_DevIpRBV);
    createParam(TPX_StatusRBVString,                   asynParamInt32,   &TPX_StatusRBV);
    createParam(TPX_NumFrameBuffersString,             asynParamInt32,   &TPX_NumFrameBuffers);
    createParam(TPX_NumFrameBuffersRBVString,          asynParamInt32,   &TPX_NumFrameBuffersRBV);
    createParam(TPX_SyncModeString,                    asynParamInt32,   &TPX_SyncMode);
    createParam(TPX_SyncModeRBVString,                 asynParamInt32,   &TPX_SyncModeRBV);
    createParam(TPX_TriggerString,                     asynParamInt32,   &TPX_Trigger);
    createParam(TPX_SyncTimeString,                    asynParamInt32,   &TPX_SyncTime);
    createParam(TPX_SyncTimeRBVString,                 asynParamInt32,   &TPX_SyncTimeRBV);
    createParam(TPX_CorrectionsDirectoryString,        asynParamOctet,   &TPX_CorrectionsDirectory);

    createParam(TPX_DataSaveDirectoryString,           asynParamOctet,   &TPX_DataSaveDirectory);
    createParam(TPX_DataFilePrefixString,              asynParamOctet,   &TPX_DataFilePrefix);

    createParam(TPX_FrameBufferIndexString,            asynParamInt32,   &TPX_FrameBufferIndex);
    createParam(TPX_ImageNumberString,                 asynParamInt32,   &TPX_ImageNumber);
    createParam(TPX_HWFileString,       	           asynParamOctet,   &TPX_HWFile);
//////////////////////////////////////////////////////////////////////////////////
    createParam(TPX_SaveToFileYesString,               asynParamInt32,   &TPX_SaveToFileYes);
    createParam(TPX_SaveToFileNoString,                asynParamInt32,   &TPX_SaveToFileNo);
    createParam(TPX_SaveToFileString,                  asynParamInt32,   &TPX_SaveToFile);
    createParam(TPX_SaveToFileRBVString,               asynParamInt32,   &TPX_SaveToFileRBV);
//////////////////////////////////////////////////////////////////////////////////
    createParam(TPX_ExtendedFrameYesString,            asynParamInt32,   &TPX_ExtendedFrameYes);
    createParam(TPX_ExtendedFrameNoString,             asynParamInt32,   &TPX_ExtendedFrameNo);
    createParam(TPX_ExtendedFrameString,               asynParamInt32,   &TPX_ExtendedFrame);
    createParam(TPX_ExtendedFrameRBVString,            asynParamInt32,   &TPX_ExtendedFrameRBV);
//////////////////////////////////////////////////////////////////////////////////
    createParam(TPX_LoadDACFileString,                 asynParamInt32,   &TPX_LoadDACFile);
    createParam(TPX_resetDetectorString,               asynParamInt32,   &TPX_resetDetector);
    createParam(TPX_DACFileString,       	           asynParamOctet,   &TPX_DACFile);
    createParam(TPX_DACAvailableString,                asynParamInt32,   &TPX_DACAvailable);
    createParam(TPX_PixConfigFileString,       	       asynParamOctet,   &TPX_PixConfigFile);
    createParam(TPX_DACRBVString,       	           asynParamInt32,   &TPX_DACRBV);

    /* Set some default values for parameters */
    status =  setStringParam (ADManufacturer, "ASI");
    status |= setStringParam (ADModel, "ASI_QUAD");
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType, NDUInt16);

    //Detector parameter defaults
    status |= setIntegerParam(TPX_DevIp, 0);
    status |= setIntegerParam(TPX_DevIpRBV, 0);
    status |= setIntegerParam(TPX_NumFrameBuffers, 10);
    status |= setIntegerParam(TPX_NumFrameBuffersRBV, 10);
    status |= setIntegerParam(TPX_SyncTime, 100);
    status |= setIntegerParam(TPX_SystemID, 0);
    status |= setIntegerParam(TPX_Initialize, 0);
    status |= setIntegerParam(TPX_InitializeRBV, NO);

    //status |= setIntegerParam(TPX_StartSoPhy, 0);
    //status |= setIntegerParam(TPX_StartSoPhyRBV, NO);

    status |= setIntegerParam(TPX_StatusRBV, TPX_STATUS_OK);
    status |= setIntegerParam(TPX_SyncModeRBV, TPX_INTERNAL_TRIGGER);
    status |= setStringParam (TPX_CorrectionsDirectory, "none");

    status |= setStringParam (TPX_DataSaveDirectory, "none");
    status |= setStringParam (TPX_DataFilePrefix, "none");

    status |= setStringParam (TPX_HWFile, "none");
//////////////////////////////////////////////////////////////////////////////////
    status |= setIntegerParam (TPX_SaveToFile, 0);
//    status |= setIntegerParam (TPX_SaveToFileYes, 0);
//    status |= setIntegerParam (TPX_SaveToFileNo, 1);
    status |= setIntegerParam (TPX_SaveToFileRBV, 0);
//////////////////////////////////////////////////////////////////////////////////
    status |= setIntegerParam (TPX_ExtendedFrame, NO);
    status |= setIntegerParam (TPX_ExtendedFrameRBV, NO);
//////////////////////////////////////////////////////////////////////////////////
    status |= setStringParam (TPX_DACFile, "none");
    status |= setIntegerParam(TPX_DACAvailable, NOT_AVAILABLE);
    status |= setStringParam (TPX_PixConfigFile, "none");
    status |= setIntegerParam (TPX_DACRBV, TPX_STATUS_NOTLOADED);
    status |= setIntegerParam (TPX_FrameBufferIndex, 0);
    status |= setIntegerParam (TPX_ImageNumber, 0);


    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set camera parameters\n",
                  driverName, functionName);
        return;
    }



    /* Create the epicsEvents for signaling to the acquisition task when acquisition starts and stops */
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: epicsEventCreate failure for start event\n",
                  driverName, functionName);
        return;
    }
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: epicsEventCreate failure for stop event\n",
                  driverName, functionName);
        return;
    }

    /* Create the thread that updates the images */
    acquireTaskId = epicsThreadCreate("acquireTaskC",
                                      epicsThreadPriorityHigh,
                                      epicsThreadGetStackSize(epicsThreadStackMedium),
                                      (EPICSTHREADFUNC)acquireTaskC,
                                      this);

    if (acquireTaskId == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: epicsThreadCreate failure for image task\n",
                  driverName, functionName);
        return;
    }

    /* Create the thread that updates the images */
    statusTaskId = epicsThreadCreate("statusCheck",
                                      epicsThreadPriorityHigh,
                                      epicsThreadGetStackSize(epicsThreadStackMedium),
                                      (EPICSTHREADFUNC)statusCheckTask,
                                      this);

    if (statusTaskId == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: epicsThreadCreate failure for image task\n",
                  driverName, functionName);
        return;
    }

    /* initialize internal variables uses to hold time delayed information.*/
    status |= getDoubleParam(ADAcquireTime, &dAcqTimeReq_);
    dAcqTimeAct_ = dAcqTimeReq_;
    status |= getIntegerParam(ADTriggerMode, &iTrigModeReq_);
    iTrigModeAct_ = iTrigModeReq_;

    // Set exit handler to clean up
    epicsAtExit(exitCallbackC, this);
    mark = 0;
}

//_____________________________________________________________________________________________

/** Callback function that is called by EPICS when the IOC exits */
static void exitCallbackC(void *pPvt) {
    tpxDetector *ptpxDetector = (tpxDetector*) pPvt;
    delete(ptpxDetector);
}

static void acquireTaskC(void *drvPvt) {
    tpxDetector *pPvt = (tpxDetector *)drvPvt;

    pPvt->acquireTask();
}


static void statusCheckTask(void *drvPvt) {
    tpxDetector *pPvt = (tpxDetector *)drvPvt;
    
    epicsTimeStamp t0, t1;
    double elapsedTime;

    while(true) {
        epicsTimeGetCurrent(&t0);
    
        pPvt->checkStatus();

        epicsTimeGetCurrent(&t1);

        elapsedTime = epicsTimeDiffInSeconds(&t1, &t0);

        if (elapsedTime > 0.002) {
            printf("Status check slow! elapsedTime=%f\n", elapsedTime);
        }
        epicsThreadSleep(0.001);
    }
}


//_____________________________________________________________________________________________

tpxDetector::~tpxDetector() {

    if (pAcqBuffer_ != NULL)
        free(pAcqBuffer_);
    if (pAcqBufferExt_ != NULL)
        free(pAcqBufferExt_);
//    delete relaxd;
    mark=0;
    delete[] TDD;
}

//_____________________________________________________________________________________________


static void hw_callback(INTPTR userData, int eventType, void *data) {
    // tpxDetector *tpx = (tpxDetector*)userData;
    // 
    // tpx->lock();
    printf("** Hardware callback: ");
    switch (eventType) {
    case HWCB_ACQSTARTED: printf("HWCB_ACQSTARTED");
    case HWCB_ACQFINISHED: printf("HWCB_ACQFINISHED");
    case HWCB_DEVDISCONNECTED: printf("HWCB_DEVDISCONNECTED");
    default:
        printf("Unknown?");
    }
    printf("\n");
    //tpx->unlock();
}

//_____________________________________________________________________________________________

bool tpxDetector::initializeDetector(void) {

    int status = asynSuccess;
    //int i=0;
    //int ndp=14;
    int numberdevs=0;
    //int SoPhyStatus;
    //FILE  *pInputFile;
    char cpCorrectionsDirectory[256];
    char cpPixConfigFile[256];
    char cpHWFile[256];
    uiRows_=DIM_X;
    uiColumns_=DIM_Y;
    uiRowsAdd_=uiRows_+4;
    uiColumnsAdd_=uiColumns_+4;
    //char str [25];
    //unsigned long data_lu=0;
    //float data_f=0;

    //HwInfoItem hwitm;
    //char tmp[255];
    //hwitm.data = tmp;


    static const char* functionName = "initializeDetector";


    status |= setIntegerParam(TPX_StatusRBV, TPX_STATUS_INITIALIZING);
    callParamCallbacks();

    status |= getIntegerParam(TPX_DevIp, (int *)&uiDevIp_);
    status |= getIntegerParam(TPX_NumFrameBuffers, (int *)&uiNumFrameBuffers_);
    status |= getStringParam(TPX_CorrectionsDirectory, sizeof(cpCorrectionsDirectory), cpCorrectionsDirectory);
    status |= getStringParam(TPX_PixConfigFile, sizeof(cpPixConfigFile), cpPixConfigFile);
    status |= getStringParam(TPX_HWFile, sizeof(cpHWFile), cpHWFile);


    sprintf (cpFileName_PC, "%s%s", cpCorrectionsDirectory, cpPixConfigFile);
    sprintf (cpFileName_HW, "%s%s", cpCorrectionsDirectory, cpHWFile);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Pixel configuration file is: %s\n",
              driverName, functionName, cpFileName_PC);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: HW configuration file is: %s\n",
              driverName, functionName, cpFileName_HW);



    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Attempting to initialize Timepix detector ...\n",
              driverName, functionName);

    // set Medipix devices and load onboard configuration (if any) to the devices


    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Mark equal to %d \n",driverName, functionName, mark);
    if (mark != 0) {
        if(numberdevs!=0) {
            if ((*relaxd->reset)(ids[uiDevIp_])) asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Cannot reset chips !!!\n",driverName, functionName);
        }
//    delete relaxd;
    }




    char relaxdlib[256];
    char *relaxdPath=NULL;

    relaxdPath = getenv( "RELAXD" );

    if ( relaxdPath==NULL ) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Cannot not get relaxd driver library path!\n",
                  driverName, functionName);
        return false;
    }


    sprintf (relaxdlib, "%s/lib/libmpxhwrelaxd.so", relaxdPath);





    void* rmpx = dlopen(relaxdlib, RTLD_NOW);
    if (!rmpx) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error in %s! Cannot load library!\n",
                  driverName, functionName, dlerror());
        return false;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: Library loaded dynamically!\n",
                  driverName, functionName);
    }


    Mpx2Interface *(*funcp)();
    * (void **)(&funcp)=dlsym(rmpx,"getMpx2Interface");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error in %s!Cannot load interface!\n",
                  driverName, functionName, dlerror());
        return false;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: interface loaded dynamically!\n",
                  driverName, functionName);
    }



    relaxd = new PxmMpx2Interface((*funcp)());
    mark=1;

    (*relaxd->findDevices)(ids, &numberdevs);
    if (numberdevs==0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error: Cannot find any device\n",
                  driverName, functionName);
        return false;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Total nimber of found devices  -> %d\n",
                  driverName, functionName, numberdevs);
    }

//    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Reset ALL = %d \n", driverName, functionName,(*relaxd->reset)(ids[uiDevIp_]));

    int ret=(*relaxd->init)(ids[uiDevIp_]);
    if (ret) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error (%d): Chip initialization failed!\n",
                  driverName, functionName, ret);
        return false;
    }
    
    // (*relaxd->setCallbackData)(uiDevIp_, (INTPTR)this);
    (*relaxd->setCallback)(&hw_callback);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Timepix detector initialized\n",
              driverName, functionName);


    ndevs = (*relaxd->chipCount)(ids[uiDevIp_]);
    if (ndevs == 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error: %d  Chip counting failed!\n",
                  driverName, functionName, ndevs);
        return false;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Total of %d chips found.\n",
                  driverName, functionName, ndevs);
    }



    /*
        devtype = (*relaxd->chipType)(ids[uiDevIp_]);

        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: The type of a chip is: %d \n",
                  driverName, functionName, devtype);

      */

    if(this->setDetectorSet()) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error: Something went wrong with setting detector!\n",
                  driverName, functionName);
        return false;
    }



    // allocate frame memory
    if ((uiNumFrameBuffers_ <= 0) || (uiNumFrameBuffers_ > 500)) {
        uiNumFrameBuffers_ = 500;
        status = setIntegerParam(TPX_NumFrameBuffers, uiNumFrameBuffers_);
    }
    if (pAcqBuffer_ != NULL)
        free (pAcqBuffer_);
    pAcqBuffer_ = (epicsInt16 *) malloc(uiNumFrameBuffers_ * uiRows_ * uiColumns_ * sizeof(epicsInt16));
    if (pAcqBuffer_ == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error:  Memory allocation failed for %d frames!\n",
                  driverName, functionName, uiNumFrameBuffers_);
        return false;
    }


    if (pAcqBufferExt_ != NULL)
        free (pAcqBufferExt_);
    pAcqBufferExt_ = (epicsInt16 *) malloc(uiRows_ * uiColumns_ * sizeof(epicsInt16));
    if (pAcqBufferExt_ == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error:  Memory allocation failed for extenmded buffer!\n",
                  driverName, functionName);
        return false;
    }




    //Update readback values
    status = 0;
    status |= setIntegerParam(ADMaxSizeX, uiColumns_);
    status |= setIntegerParam(ADMaxSizeY, uiRows_);

    status |= setIntegerParam(ADSizeX, uiColumns_);
    status |= setIntegerParam(ADSizeY, uiRows_);
    status |= setIntegerParam(NDArraySizeX, uiColumns_);
    status |= setIntegerParam(NDArraySizeY, uiRows_);
    //  status |= setIntegerParam(TPX_SystemID, dwSystemID_);
    status |= setIntegerParam(TPX_StatusRBV, TPX_STATUS_OK);
    status |= setIntegerParam(TPX_InitializeRBV, 1);
    status |= setIntegerParam(TPX_NumFrameBuffersRBV, uiNumFrameBuffers_);
    status |= setIntegerParam(TPX_DevIpRBV, uiDevIp_);
//////////////////////////////////////////////////////////////////////////////////
    status |= setIntegerParam(TPX_SaveToFile, 0);
    status |= setIntegerParam(TPX_SaveToFileRBV, 0);
//////////////////////////////////////////////////////////////////////////////////
    status |= setIntegerParam (TPX_ExtendedFrame, NO);
    status |= setIntegerParam (TPX_ExtendedFrameRBV, NO);
//////////////////////////////////////////////////////////////////////////////////
    callParamCallbacks();

    driverReady=true;

    return true;
}


void tpxDetector::setupTriggering(bool internal) {
    u32 reg=0;
    int bit23=0,bit24=0;
    
    lock();

    (*relaxd->readReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, &reg );
    // Reset 'use Timer'
    reg &= ~MPIX2_CONF_TIMER_USED;

    // Reset 'burst readout' bit, just in case
    reg &= ~MPIX2_CONF_BURST_READOUT;

    // Close shutter (re-enabled below when appropriate)
    reg |= MPIX2_CONF_SHUTTER_CLOSED;

    if (internal) {
        // Disable external trigger
        reg &= ~MPIX2_CONF_EXT_TRIG_ENABLE;
    } else {
        reg &= ~BIT23;
        reg &= ~BIT24;

        // Enable external trigger
        reg |= MPIX2_CONF_EXT_TRIG_ENABLE;
        // Trigger on rising edge
        reg &= ~MPIX2_CONF_EXT_TRIG_FALLING_EDGE;
        // Ready for next (external) trigger
        reg &= ~MPIX2_CONF_EXT_TRIG_INHIBIT;

        if (bit23==1) {
            reg |= BIT23;
        }
        if (bit24==1) {
            reg |= BIT24;
        }
    
    }

    (*relaxd->writeReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, reg );
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "Trigger setup completed (reg set=%x, read back:\n", reg);

    epicsThreadSleep(0.1);
    (*relaxd->readReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, &reg );
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "=%x)\n", reg);

    unlock();
}

void timestamp()
{
    epicsTimeStamp ts;
    epicsTimeGetCurrent(&ts);

    char buf[256];

    epicsTime t(ts);
    size_t numChar = t.strftime(buf, sizeof(buf), "%H:%M:%S.%09f");
    if (numChar > 0) {
        printf((const char*)buf);
    }
}

u32 tpxDetector::checkStatus(void) {
    if(!driverReady) {
        return 0;
    }

    u32 reg=0;

    checkConfiguration();

    lock();
    (*relaxd->readReg)(ids[uiDevIp_], MPIX2_STATUS_REG_OFFSET, &reg);

    if (reg != lastStatus_) {
        printf("-!- "); 
        timestamp();
        printf(" Status changed %x: ", reg);
        if ((reg & MPIX2_STATUS_MPIX_IDLE) == MPIX2_STATUS_MPIX_IDLE) {
            printf("MPIX_IDLE ");
        }
        if ((reg & MPIX2_STATUS_MPIX_BUSY) == MPIX2_STATUS_MPIX_BUSY) {
            printf("MPIX_BUSY ");
        }
        if ((reg & MPIX2_STATUS_XMT_READY) == MPIX2_STATUS_XMT_READY) {
            printf("XMT_READY ");
        }
        if ((reg & MPIX2_STATUS_RCV_READY) == MPIX2_STATUS_RCV_READY) {
            printf("RCV_READY ");
        }
        if ((reg & MPIX2_STATUS_DATARCV_READY) == MPIX2_STATUS_DATARCV_READY) {
            printf("DATARCV_READY ");
        }
        if ((reg & MPIX2_STATUS_SHUTTER_READY) == MPIX2_STATUS_SHUTTER_READY) {
            printf("SHUTTER_READY ");
        }
        if ((reg & MPIX2_STATUS_BURST_READY_INT) == MPIX2_STATUS_BURST_READY_INT) {
            printf("BURST_READY_INT ");
        }
        if ((reg & MPIX2_STATUS_BUS_ERR_INT) == MPIX2_STATUS_BUS_ERR_INT) {
            printf("BUS_ERR_INT ");
        }
        if ((reg & MPIX2_STATUS_CONNECT_ERR_INT) == MPIX2_STATUS_CONNECT_ERR_INT) {
            printf("CONNECT_ERR_INT ");
        }
        if ((reg & MPIX2_STATUS_WATCHDOG_ERR_INT) == MPIX2_STATUS_WATCHDOG_ERR_INT) {
            printf("WATCHDOG_ERR_INT ");
        }
        if ((reg & MPIX2_STATUS_FATAL_ERR_INT) == MPIX2_STATUS_FATAL_ERR_INT) {
            printf("FATAL_ERR_INT ");
        }
        if ((reg & MPIX2_STATUS_EXT_TRIG_INT) == MPIX2_STATUS_EXT_TRIG_INT) {
            printf("EXT_TRIG_INT "); 
        }
        printf("\n");
        lastStatus_ = reg;

    }

    unlock();

    static const char *last_err=NULL;
    static const char *last_dev_err=NULL;
    
    lock();
    if ((*relaxd->getLastError)() != last_err) {
        last_err = (*relaxd->getLastError)();
        printf("Error status: %s\n", last_err);
    }
    unlock();

    lock();
    if ((*relaxd->getLastDevError)(ids[uiDevIp_]) != last_dev_err) {
        last_dev_err = (*relaxd->getLastDevError)(ids[uiDevIp_]);
        printf("Dev error status: %s\n", last_dev_err);
    }
    unlock();

    return reg;
}

u32 tpxDetector::checkConfiguration(void) {
    if(!driverReady) {
        return 0;
    }

    u32 reg=0;

    lock();
    (*relaxd->readReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, &reg);

    if (reg != lastConfiguration_) {

        printf("-!- "); 
        timestamp();
        printf(" Configuration changed %x: ", reg);

        if ((reg & MPIX2_CONF_START) == MPIX2_CONF_START) {
            printf("START ");
        }
        if ((reg & MPIX2_CONF_MODE_MASK) == MPIX2_CONF_MODE_MASK) {
            printf("MODE_MASK ");
        }
        if ((reg & MPIX2_CONF_MODE_READOUT) == MPIX2_CONF_MODE_READOUT) {
            printf("MODE_READOUT ");
        }
        if ((reg & MPIX2_CONF_MODE_SET_MATRIX) == MPIX2_CONF_MODE_SET_MATRIX) {
            printf("MODE_SET_MATRIX ");
        }
        if ((reg & MPIX2_CONF_MODE_SET_DAC) == MPIX2_CONF_MODE_SET_DAC) {
            printf("MODE_SET_DAC ");
        }
        if ((reg & MPIX2_CONF_POLARITY) == MPIX2_CONF_POLARITY) {
            printf("POLARITY ");
        }
        if ((reg & MPIX2_CONF_SPARE_FSR) == MPIX2_CONF_SPARE_FSR) {
            printf("SPARE_FSR ");
        }
        if ((reg & MPIX2_CONF_ENABLE_TPULSE) == MPIX2_CONF_ENABLE_TPULSE) {
            printf("ENABLE_TPULSE ");
        }
        if ((reg & MPIX2_CONF_ENABLE_CST) == MPIX2_CONF_ENABLE_CST) {
            printf("ENABLE_CST ");
        }
        if ((reg & MPIX2_CONF_P_S) == MPIX2_CONF_P_S) {
            printf("P_S ");
        }
        if ((reg & MPIX2_CONF_SHUTTER_CLOSED) == MPIX2_CONF_SHUTTER_CLOSED) {
            printf("SHUTTER_CLOSED ");
        }
        if ((reg & MPIX2_CONF_TIMER_USED) == MPIX2_CONF_TIMER_USED) {
            printf("TIMER_USED ");
        }
        if ((reg & MPIX2_CONF_RESET_MPIX) == MPIX2_CONF_RESET_MPIX) {
            printf("RESET_MPIX ");
        }
        if ((reg & MPIX2_CONF_BURST_READOUT) == MPIX2_CONF_BURST_READOUT) {
            printf("BURST_READOUT ");
        }
        if ((reg & MPIX2_CONF_CHIP_ALL) == MPIX2_CONF_CHIP_ALL) {
            printf("CHIP_ALL ");
        }
        if ((reg & MPIX2_CONF_CHIP_SEL_MASK) == MPIX2_CONF_CHIP_SEL_MASK) {
            printf("CHIP_SEL_MASK ");
        }
        if ((reg & MPIX2_CONF_CHIP_SEL_SHIFT) == MPIX2_CONF_CHIP_SEL_SHIFT) {
            printf("CHIP_SEL_SHIFT ");
        }
        if ((reg & MPIX2_CONF_EXT_TRIG_ENABLE) == MPIX2_CONF_EXT_TRIG_ENABLE) {
            printf("EXT_TRIG_ENABLE ");
        }
        if ((reg & MPIX2_CONF_EXT_TRIG_FALLING_EDGE) == MPIX2_CONF_EXT_TRIG_FALLING_EDGE) {
            printf("EXT_TRIG_FALLING_EDGE ");
        }
        if ((reg & MPIX2_CONF_EXT_TRIG_INHIBIT) == MPIX2_CONF_EXT_TRIG_INHIBIT) {
            printf("EXT_TRIG_INHIBIT ");
        }
        if ((reg & MPIX2_CONF_TPX_CLOCK_MASK) == MPIX2_CONF_TPX_CLOCK_MASK) {
            printf("TPX_CLOCK_MASK ");
        }
        if ((reg & MPIX2_CONF_TPX_CLOCK_SHIFT) == MPIX2_CONF_TPX_CLOCK_SHIFT) {
            printf("TPX_CLOCK_SHIFT ");
        }
        if ((reg & MPIX2_CONF_TPX_CLOCK_100MHZ) == MPIX2_CONF_TPX_CLOCK_100MHZ) {
            printf("TPX_CLOCK_100MHZ ");
        }
        if ((reg & MPIX2_CONF_TPX_CLOCK_50MHZ) == MPIX2_CONF_TPX_CLOCK_50MHZ) {
            printf("TPX_CLOCK_50MHZ ");
        }
        if ((reg & MPIX2_CONF_TPX_CLOCK_10MHZ) == MPIX2_CONF_TPX_CLOCK_10MHZ) {
            printf("TPX_CLOCK_10MHZ ");
        }
        if ((reg & MPIX2_CONF_TPX_CLOCK_EXT) == MPIX2_CONF_TPX_CLOCK_EXT) {
            printf("TPX_CLOCK_EXT ");
        }
        if ((reg & MPIX2_CONF_RO_CLOCK_125MHZ) == MPIX2_CONF_RO_CLOCK_125MHZ) {
            printf("RO_CLOCK_125MHZ ");
        }
        if ((reg & MPIX2_CONF_TPX_PRECLOCKS) == MPIX2_CONF_TPX_PRECLOCKS) {
            printf("TPX_PRECLOCKS ");
        }
        if ((reg & MPIX2_CONF_TPX_TRIG_TYPE_SHIFT) == MPIX2_CONF_TPX_TRIG_TYPE_SHIFT) {
            printf("TPX_TRIG_TYPE_SHIFT ");
        }
        if ((reg & MPIX2_CONF_TPX_TRIG_TYPE_NEG) == MPIX2_CONF_TPX_TRIG_TYPE_NEG) {
            printf("TPX_TRIG_TYPE_NEG ");
        }
        if ((reg & MPIX2_CONF_TPX_TRIG_TYPE_POS) == MPIX2_CONF_TPX_TRIG_TYPE_POS) {
            printf("TPX_TRIG_TYPE_POS ");
        }
        if ((reg & MPIX2_CONF_TPX_TRIG_TYPE_PULSE) == MPIX2_CONF_TPX_TRIG_TYPE_PULSE) {
            printf("TPX_TRIG_TYPE_PULSE ");
        }
        if ((reg & MPIX2_CONF_TPX_TRIG_TYPE_MASK) == MPIX2_CONF_TPX_TRIG_TYPE_MASK) {
            printf("TPX_TRIG_TYPE_MASK ");
        }
        printf("\n");
        lastConfiguration_ = reg;

    }
    unlock();
    return reg;
}


void tpxDetector::acquireTask(void) {
    int status = asynSuccess;
    const int mtrx= uiRows_*uiColumns_;
    const int multp=2;
    int extime;
    int imageCounter;
    int numImages, numImagesCounter, savetofile, ExtendedFrame;
    int imageMode;
    int trigmode;
    int arrayCallbacks=1;
    size_t dims[2];
    int acquire;
    epicsTimeStamp startTime, endTime;
    int poll_count;

    NDArray       *pImage;
    NDDataType_t  dataType;
    NDArrayInfo   arrayInfo;
    epicsInt16   *pInput = NULL;
    epicsInt16   *pRord = NULL;

#ifdef RAW_DATA_READOUT
    u8* myarray8;
    myarray8 = new u8[mtrx*multp];
    u32 nbytes = 0;
    int lostRows = 0;
#endif

    const char *functionName = "acquireTask";
    double elapsedTime,acquireTime;
    epicsEventWaitStatus estatus;
    this->lock();

    // Loop forever
    while (1) {

        poll_count = 0;

        // Is acquisition active?
        getIntegerParam(ADAcquire, &acquire);

        // If we are not acquiring then wait for a semaphore that is given when acquisition is started
        if (!acquire) {
            if (driverReady) {
                u32 reg=0;
                (*relaxd->readReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, &reg );
                reg |= MPIX2_CONF_EXT_TRIG_INHIBIT;
                reg &= ~MPIX2_CONF_EXT_TRIG_ENABLE;
                (*relaxd->writeReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, reg );
            }
        
            this->closeRawDataFile();

            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: waiting for acquire to start\n", driverName, functionName);

            // Release the lock while we wait for an event that says acquire has started, then lock again

            // resetting the detector takes ~2 seconds
            if(driverReady) {
                this->resetDetector();
            }

            this->unlock();
            status = epicsEventWait(this->startEventId);
            this->lock();
            setIntegerParam(ADNumImagesCounter, 0);

            status |= getIntegerParam(TPX_SaveToFile, &savetofile);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: ---------> save to file = %d\n", driverName, functionName,savetofile);

#ifdef RAW_DATA_READOUT
            if(savetofile==1) {
                char dir[PATH_STRING_LEN];
                char baseName[PATH_STRING_LEN];

                status |= getStringParam(TPX_DataSaveDirectory, sizeof(dir), dir);
                status |= getStringParam(TPX_DataFilePrefix, sizeof(baseName), baseName);

                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: Opening file with base name %s in directory: %s\n", driverName, functionName, baseName, dir);
                writeRawDataFlag=false;
                if(strlen(dir)>0 && savetofile==1) {
                    if(this->openRawDataFile((char *) dir, (char *) baseName)!=0) {
                        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Could not open file with base name %s in directory: %s\n", driverName, functionName, baseName, dir);
                    } else {
                        writeRawDataFlag=true;
                    }
                }
            } else {
                writeRawDataFlag=false;
                if (rawDataFile) {
                    closeRawDataFile();
                }
            }
#endif
        } // if acquiring

        callParamCallbacks();
        status |= getIntegerParam(ADTriggerMode, &trigmode);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);

        if (trigmode==TPX_EXTERNAL_TRIGGER) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "---- EXTERNAL TRIGGER MODE (image %d) ----\n", 
                      numImagesCounter);

            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            if(numImagesCounter==0) {
                setupTriggering(false);
            }

            epicsTimeStamp t0, t1;

            epicsTimeGetCurrent(&startTime);
            // Open the ADShutter
            setShutter(1);
            bool read_frame = false;
            while(true) {
                poll_count++;
                if ((*relaxd->newFrame)(ids[uiDevIp_], true, true)) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                              "** new frame **\n");
                    read_frame = true;
                    pInput = &pAcqBuffer_[0];

#ifdef RAW_DATA_READOUT
                    (*relaxd->readMatrixRaw)(ids[uiDevIp_], myarray8, &nbytes, &lostRows);

                    if(writeRawDataFlag && writeRawData(myarray8, mtrx*multp, lostRows)!=0) {
                        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                                  "%s:%s: Could not write raw data\n",
                                  driverName, functionName);
                    }
#else
                    (*relaxd->readMatrix)(ids[uiDevIp_], pInput,mtrx);
#endif
                
                    // Close the ADshutter
                    setShutter(0);
                    break;
                }
                
                epicsTimeGetCurrent(&t0);
                
                this->unlock();
                // estatus = epicsEventTryWait(this->stopEventId);
                estatus = epicsEventWaitWithTimeout(this->stopEventId, 0.0002);
                this->lock();
                epicsTimeGetCurrent(&t1);

                elapsedTime = epicsTimeDiffInSeconds(&t1, &t0);

                if (elapsedTime > 0.01) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: unlock elapsedTime?=%f\n",
                              driverName, functionName, elapsedTime);
                }

                if (estatus==epicsEventWaitOK) {
                    (*relaxd->closeShutter)(ids[uiDevIp_]);
                    (*relaxd->resetMatrix)(ids[uiDevIp_]);

                    // Close the ADshutter
                    setShutter(0);
                    setIntegerParam(ADAcquire, 0);
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                              "%s:%s: acquisition broken (user clicked stop) (poll_count=%d)\n", 
                              driverName, functionName, poll_count);
                    callParamCallbacks();
                    break;
                }
            }

            getIntegerParam(ADNumImages, &numImages);
            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            getIntegerParam(ADImageMode, &imageMode);
            if (!read_frame ||
                (imageMode == ADImageSingle) ||
                ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
                u32 reg=0;
                (*relaxd->readReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, &reg );
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "(Read frame: %d Mode=%d Image counter %d of %d)\n",
                          read_frame, imageMode, numImagesCounter, numImages);
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "End of acquisition/broken acquisition, inhibit triggers (reg=%x, ", reg);
                reg |= MPIX2_CONF_EXT_TRIG_INHIBIT;
                reg &= ~MPIX2_CONF_EXT_TRIG_ENABLE;
                (*relaxd->writeReg)(ids[uiDevIp_], MPIX2_CONF_REG_OFFSET, reg );
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                           "set %x)\n", reg);
            }

            if (!read_frame) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "Read frame failed; continuing loop\n");
                continue;
            }
            
        } else if (trigmode==TPX_INTERNAL_TRIGGER) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "---- INTERNAL TRIGGER MODE (image %d) ----\n",
                      numImagesCounter);

            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            if(numImagesCounter==0) {
                setupTriggering(true);
            }

            status |= getDoubleParam(ADAcquireTime, &dAcqTimeReq_);
            acquireTime=dAcqTimeReq_/1000000.;
            extime=(int)(dAcqTimeReq_);

            // Open the ADShutter
            setShutter(1);
            callParamCallbacks();
            if (dAcqTimeReq_ > 0.0) {
                epicsTimeGetCurrent(&startTime);

                // Camera acquisition
                (*relaxd->enableTimer)(ids[uiDevIp_], true, extime );
                (*relaxd->openShutter)(ids[uiDevIp_],1);

                // EPICS timer
                this->unlock();
                estatus = epicsEventWaitWithTimeout(this->stopEventId, acquireTime);
                this->lock();

                // time to attach debugger
                //          epicsThreadSleep(60.0);
                if (estatus==epicsEventWaitOK) {
                    (*relaxd->closeShutter)(ids[uiDevIp_]);
                    (*relaxd->resetMatrix)(ids[uiDevIp_]);
                    // Close the ADshutter
                    setShutter(0);
                    setIntegerParam(ADAcquire, 0);
                    callParamCallbacks();
                    continue;
                } else {
                    // Check with camera internal timer
                    while(!(*relaxd->timerExpired)(ids[uiDevIp_])) {
                        this->unlock();
                        estatus = epicsEventWaitWithTimeout(this->stopEventId, acquireTime/10.0);
                        this->lock();
                        // Close the ADshutter
                        setShutter(0);

                        if (estatus==epicsEventWaitOK) {
                            (*relaxd->closeShutter)(ids[uiDevIp_]);
                            (*relaxd->resetMatrix)(ids[uiDevIp_]);
                            setIntegerParam(ADAcquire, 0);
                            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                                      "%s:%s: acquisition broken (user stopped acquisition)\n", driverName, functionName);
                            callParamCallbacks();
                            break;
                        }
                    }
                    if (estatus==epicsEventWaitOK) {
                        continue;
                    }
                }

                pInput = &pAcqBuffer_[0];
                //       pInput = &pAcqBuffer_[uiColumns_ * uiRows_ * numImagesCounter];

#ifdef RAW_DATA_READOUT
                (*relaxd->readMatrixRaw)(ids[uiDevIp_], myarray8, &nbytes, &lostRows);
                if(writeRawDataFlag && this->writeRawData(myarray8, mtrx*multp, lostRows)!=0) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Could not write raw data\n", driverName, functionName);
                }
#else
                (*relaxd->readMatrix)(ids[uiDevIp_], pInput,mtrx);
#endif
            }
        } else {
            // Wait until acquire is true again and acquire mode in a valid state
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "Unknown trigger mode %d\n", trigmode);
            continue;
        }
        
        // ** Push acquired frame out to EPICS
        dataType = NDInt16;
        if (this->pArrays[0]) {
            this->pArrays[0]->release();
        }

        // Allocate the array
#ifdef SINGLE
        dims[0] = uiColumns_;
        dims[1] = uiRows_;
#else
        getIntegerParam(TPX_ExtendedFrame, &ExtendedFrame);
        if(ExtendedFrame==1) {
            dims[0] = uiColumnsAdd_;
            dims[1] = uiRowsAdd_;
            pRord = &pAcqBufferExt_[0];
        } else {
            dims[0] = uiColumns_;
            dims[1] = uiRows_;
        }
#endif

        this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
        if (this->pArrays[0] == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error allocating buffer\n",
                      driverName, functionName);
            unlock();
            return;
        }
        pImage = this->pArrays[0];
        pImage->getInfo(&arrayInfo);

        setIntegerParam(NDArraySize,  arrayInfo.totalBytes);
        setIntegerParam(NDArraySizeX, pImage->dims[0].size);
        setIntegerParam(NDArraySizeY, pImage->dims[1].size);

        // Get the current parameters
        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        // Put the frame number and time stamp into the buffer
        pImage->uniqueId = imageCounter;
        pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

        // Get any attributes that have been defined for this driver
        this->getAttributes(pImage->pAttributeList);

        if(isTimeForPreviewUpdate()) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: updating arrays\n", driverName, functionName);

#ifdef RAW_DATA_READOUT
#  ifdef SINGLE
            (*relaxd->stream2Data)(ids[uiDevIp_], myarray8,pInput);
#  else
            (*relaxd->parStream2Data)(ids[uiDevIp_], myarray8,pInput);

#  endif
#endif

#ifdef SINGLE
            //   asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: dim_x %d,  dim_y %d, totbytes %d\n", driverName, functionName, uiColumns_, uiRows_,arrayInfo.totalBytes);
            memcpy(((epicsInt16*)pImage->pData), pInput, arrayInfo.totalBytes);
            //         this->copyPixels(pInput,(epicsInt16*)pImage->pData);
#else
            //	        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: test 1 \n", driverName, functionName);

            if(ExtendedFrame==1) {
                this->reorderPixels(pInput,pRord);
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s:%s: reorder pixels done\n", driverName, functionName);
                this->extendPixels(pRord,(epicsInt16*)pImage->pData);
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s:%s: extend pixels done\n", driverName, functionName);

            } else {
                this->reorderPixels(pInput,(epicsInt16*)pImage->pData);
            }
#endif

            if (arrayCallbacks) {
                // Call the NDArray callback
                // Must release the lock here, or we can get into a deadlock, because we can
                //  block on the plugin lock, and the plugin can be calling us
                this->unlock();
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s:%s: calling imageData callback\n", driverName, functionName);
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                this->lock();
            }
        }

        // See if acquisition is done
        if ((imageMode == ADImageSingle) ||
            ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
            setIntegerParam(ADAcquire, 0);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: acquisition completed\n", driverName, functionName);
        }

        // Call the callbacks to update any changes
        callParamCallbacks();
        getIntegerParam(ADAcquire, &acquire);
        if (acquire) {
            epicsTimeGetCurrent(&endTime);
            elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: frame elapsedTime=%f\n",
                      driverName, functionName, elapsedTime);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: frame poll_count=%d\n",
                      driverName, functionName, poll_count);
        }
    }
}

//_____________________________________________________________________________________________

asynStatus tpxDetector::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    int adstatus;
    int detstatus;
    int status = asynSuccess;
    int retstat;
    static const char *functionName = "writeInt32";

    // Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
    // status at the end, but that's OK

    status = setIntegerParam(function, value);

    // getIntegerParam(ADStatus, &adstatus);

    // This is where the parameter is sent to the hardware
    if (function == ADAcquire) {
        getIntegerParam(ADStatus, &adstatus);
        getIntegerParam(TPX_InitializeRBV, &detstatus);

        if (value && (adstatus == ADStatusIdle) && detstatus) {
            // Send an event to wake up the acquisition task.
            // It won't actually start getting new images until we release the lock below
            epicsEventSignal(this->startEventId);
        }
        if (!value && (adstatus == ADStatusIdle)) {
            // This was a command to stop acquisition
            // Send the stop event
            epicsEventSignal(this->stopEventId);
        }
    } else if ((function == ADBinX) ||
               (function == ADBinY)) {
        if ( adstatus == ADStatusIdle ) {
//           setBinning();
        }
    } else if (function == TPX_Initialize) {
        getIntegerParam(ADStatus, &adstatus);
        if ( adstatus == ADStatusIdle ) {
            this->initializeDetector();
        }
    }


    else if (function == ADTriggerMode) {
        getIntegerParam(ADStatus, &adstatus);
        retstat |= getIntegerParam(ADTriggerMode, &iTrigModeReq_);
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "%s:%s: Setting Requested Trigger Mode: %d\n",
                  driverName, functionName, iTrigModeReq_);
        retstat |= setIntegerParam(ADTriggerMode, iTrigModeAct_);
        //if not running go ahead and set the trigger mode
        if ( adstatus == ADStatusIdle ) {
            setTriggerMode();
        }
    }

    else if (function == TPX_LoadDACFile) {
        getIntegerParam(ADStatus, &adstatus);
        if ( adstatus == ADStatusIdle ) {
            this->loadDACsettings();
//       this->testThread();
        }
    } else if (function == TPX_resetDetector) {
        getIntegerParam(ADStatus, &adstatus);
        if ( adstatus == ADStatusIdle ) {
            this->resetDetector();
//       this->testThread();
        }
    }
//////////////////////////////////////////////////////////////////////////////////
    else if (function == TPX_SaveToFileYes) {
        saveTofileYes();
    } else if (function == TPX_SaveToFileNo) {
        saveTofileNo();
    }
//////////////////////////////////////////////////////////////////////////////////
    else if (function == TPX_ExtendedFrameYes) {
        extendFrameYes();
    } else if (function == TPX_ExtendedFrameNo) {
        extendFrameNo();
    }
//////////////////////////////////////////////////////////////////////////////////
    else {
        /* If this parameter belongs to a base class call its method */
        if (function < TPX_FIRST_PARAM) {
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }

    // Do callbacks so higher layers see any changes
    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);

    return (asynStatus) status;
}


//____________________________________________________________________________________________

asynStatus tpxDetector::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int retstat;
    int adstatus;
    static const char *functionName = "writeFloat64";

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    /* Changing any of the following parameters requires recomputing the base image */
    if (function == ADAcquireTime) {
        getIntegerParam(ADStatus, &adstatus);

        retstat |= getDoubleParam(ADAcquireTime, &dAcqTimeReq_);
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "%s:%s: Setting Requested Acquisition Time: %f\n",
                  driverName, functionName, dAcqTimeReq_);
        retstat |= setDoubleParam(ADAcquireTime, dAcqTimeAct_);
        // if the detector is idle then go ahead and set the value
        if ( adstatus == ADStatusIdle ) {
            setExposureTime();
        }

    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < TPX_FIRST_PARAM) {
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%f\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%f\n",
                  driverName, functionName, function, value);
    return status;
}


//_____________________________________________________________________________________________
///////////////////////////////////////////////////////////////////////////////////////////////

void tpxDetector::setBinning(void) {
    int binX, binY, sizeX, sizeY;
//    static const char *functionName = "setBinning";

    getIntegerParam(ADBinX, &binX);
    getIntegerParam(ADBinY, &binY);
    getIntegerParam(ADMaxSizeX, &sizeX);
    getIntegerParam(ADMaxSizeY, &sizeY);
    uiColumns_ = sizeX/binX;
    uiRows_    = sizeY/binY;
}
///////////////////////////////////////////////////////////////////////////////////
//_____________________________________________________________________________________________

void tpxDetector::reorderPixels(epicsInt16 *pInput, epicsInt16 *pOutput) {
    int b=0;
    for(int r=0; r<256; r++) {
        for(int c=0; c<256; c++) {
            pOutput[b]=pInput[c+r*256];
            pOutput[256+b]=pInput[256*256+c+r*256];
            pOutput[256*512+b]=pInput[256*512+256*256+(255-c)+(255-r)*256];
            pOutput[256*512+256+b++]=pInput[256*512+(255-c)+(255-r)*256];
        }
        b+=256;
    }
}

void tpxDetector::copyPixels(epicsInt16 *pInput, epicsInt16 *pOutput) {
    int border=DIM_X*DIM_Y;

    for(int r=0; r<border; r++) {
        pOutput[r]=pInput[r];
    }
}


void tpxDetector::extendPixels(epicsInt16 *pInput, epicsInt16 *pOutput) {
    int ii,jj;
    double cc=1.0/3.0;
    double cc2=cc*cc;


    for(ii=0; ii<3; ii++) {
        for(jj=0; jj<3; jj++) {
            pOutput[516*(255+ii)+255+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*255+255])*cc2);
            pOutput[516*(255+ii)+258+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*255+256])*cc2);
            pOutput[516*(258+ii)+255+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*256+255])*cc2);
            pOutput[516*(258+ii)+258+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*256+256])*cc2);

        }
    }
    for(ii=0; ii<255; ii++) {
        for(jj=0; jj<3; jj++) {
            pOutput[516*ii+255+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*ii+255])*cc);
            pOutput[516*ii+258+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*ii+256])*cc);

            pOutput[516*(ii+261)+255+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*(ii+257)+255])*cc);
            pOutput[516*(ii+261)+258+jj]=static_cast<epicsInt16>(static_cast<double>(pInput[512*(ii+257)+256])*cc);

            pOutput[516*(255+jj)+ii]=static_cast<epicsInt16>(static_cast<double>(pInput[512*255+ii])*cc);
            pOutput[516*(258+jj)+ii]=static_cast<epicsInt16>(static_cast<double>(pInput[512*256+ii])*cc);

            pOutput[516*(255+jj)+ii+261]=static_cast<epicsInt16>(static_cast<double>(pInput[512*255+ii+257])*cc);
            pOutput[516*(258+jj)+ii+261]=static_cast<epicsInt16>(static_cast<double>(pInput[512*256+ii+257])*cc);
        }
        for(jj=0; jj<255; jj++) {
            pOutput[516*ii+jj]=pInput[512*ii+jj];
            pOutput[516*(ii+261)+jj]=pInput[512*(ii+257)+jj];
            pOutput[516*ii+jj+261]=pInput[512*ii+jj+257];
            pOutput[516*(ii+261)+jj+261]=pInput[512*(ii+257)+jj+257];
        }
    }

}

//////////////////////////////////////////////////////////////////////////////////
void tpxDetector::saveTofileYes (void) {
    int savetofile;
    int status = asynSuccess;
    const char *functionName = "saveTofileYes";

    status |= getIntegerParam(TPX_SaveToFile, &savetofile);
    status |= setIntegerParam(TPX_SaveToFile, 1);
    status |= setIntegerParam(TPX_SaveToFileRBV, 1);
    callParamCallbacks();
    status |= getIntegerParam(TPX_SaveToFile, &savetofile);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Save to file -> %d\n",
              driverName, functionName, savetofile);

}

void tpxDetector::saveTofileNo (void) {
    int savetofile;
    int status = asynSuccess;
    const char *functionName = "saveTofileNo";
    status |= getIntegerParam(TPX_SaveToFile, &savetofile);
    status |= setIntegerParam(TPX_SaveToFile, 0);
    status |= setIntegerParam(TPX_SaveToFileRBV, 0);
    callParamCallbacks();
    status |= getIntegerParam(TPX_SaveToFile, &savetofile);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Save to file -> %d\n",
              driverName, functionName, savetofile);
}
//////////////////////////////////////////////////////////////////////////////////
void tpxDetector::extendFrameYes(void) {
    int extendframe;
    int status = asynSuccess;
    const char *functionName = "extendFrameYes";

    status |= getIntegerParam(TPX_ExtendedFrame, &extendframe);
    status |= setIntegerParam(TPX_ExtendedFrame, 1);
    status |= setIntegerParam(TPX_ExtendedFrameRBV, 1);
    callParamCallbacks();
    status |= getIntegerParam(TPX_ExtendedFrame, &extendframe);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Extended frame -> %d\n",
              driverName, functionName, extendframe);
}

void tpxDetector::extendFrameNo (void) {
    int extendframe;
    int status = asynSuccess;
    const char *functionName = "extendFrameNo";

    status |= getIntegerParam(TPX_ExtendedFrame, &extendframe);
    status |= setIntegerParam(TPX_ExtendedFrame, 0);
    status |= setIntegerParam(TPX_ExtendedFrameRBV, 0);
    callParamCallbacks();
    status |= getIntegerParam(TPX_ExtendedFrame, &extendframe);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Extended Frame -> %d\n",
              driverName, functionName, extendframe);
}
////////////////////////////////////////////////////////////////////////////////////

//_____________________________________________________________________________________________
void tpxDetector::loadDACsettings (void) {
    int status = asynSuccess;
    char cpCorrectionsDirectory[256];
    char cpDACFile[256];
    char cpFileName[256];
    char str [20];
    int ii;
    FILE  *pInputFile;
    struct stat stat_buffer;
    static const char *functionName = "loadDACsettings";
    int err;

    TDD =  new int[56];

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Loading DAC settings...\n",
              driverName, functionName);

    callParamCallbacks();

    status |= getStringParam(TPX_CorrectionsDirectory, sizeof(cpCorrectionsDirectory), cpCorrectionsDirectory);
    status |= getStringParam(TPX_DACFile, sizeof(cpDACFile), cpDACFile);


    //load DAC file
    sprintf (cpFileName, "%s%s", cpCorrectionsDirectory, cpDACFile);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: DAC file name: %s\n",
              driverName, functionName, cpFileName);
    if ((stat (cpFileName, &stat_buffer) == 0) && (stat_buffer.st_mode & S_IFREG)) {
        pInputFile = fopen (cpFileName, "r");
        if (pInputFile != NULL) {
            for (int kk=0; kk<4; kk++) {
                fscanf (pInputFile, "%s", str);
                for (ii=0; ii<14; ii++) {
                    fscanf (pInputFile, "%s", str);
                    for(int n=0; n<20; ++n) {
                        if (isdigit(str[n])) {
                            TDD[ii+kk*14] = atoi(&str[n]);
                            break;
                        }
                    }
                }
            }

            if ((err = ferror(pInputFile))) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: Failed to read DAC file %s (err=%d)\n",
                          driverName, functionName, cpFileName, err);
                fclose (pInputFile);
                return;

            }

            fclose (pInputFile);

            /*
             for( ii=0; ii<56; ++ii )
             {
             asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "d\n", TDD[ii]);
             }
             */

            status = 0;
            status |= setIntegerParam(TPX_DACAvailable, AVAILABLE);
            status |= setIntegerParam(TPX_DACRBV, TPX_STATUS_OK);

            callParamCallbacks();
            //     for( ii=0; ii<56; ++ii ) printf("TDD[%d] = %d \n",ii, TDD[ii]);

            //    callParamCallbacks();
        } else
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Failed to open DAC correction file %s\n",
                      driverName, functionName, cpFileName);
    } else
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Failed to find DAC correction file\n",
                  driverName, functionName);


    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: DAC file loaded: %s\n",
              driverName, functionName, cpFileName);


}

//-------------------------------------------------------------
asynStatus tpxDetector::setTriggerMode() {
    int mode;

    mode = iTrigModeReq_;

    switch (mode) {
    case TPX_EXTERNAL_TRIGGER: {
        //      error = Acquisition_SetFrameSyncMode(hAcqDesc_, HIS_SYNCMODE_EXTERNAL_TRIGGER);
        break;
    }
    case TPX_INTERNAL_TRIGGER: {
        //      error = Acquisition_SetFrameSyncMode(hAcqDesc_, HIS_SYNCMODE_INTERNAL_TIMER);
        break;
    }
    }
    iTrigModeAct_ = mode;
    setIntegerParam(ADTriggerMode, iTrigModeAct_);
    callParamCallbacks();

    return asynSuccess;

}


//-------------------------------------------------------------
asynStatus tpxDetector::setExposureTime() {
    int status = asynSuccess;
    //  int error;
    const char *functionName = "setExposureTime";

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Setting AcquireTime %f\n",
              driverName, functionName, dAcqTimeReq_);
    dAcqTimeAct_ = dAcqTimeReq_* 1000000.;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: internal timer: %f ms\n",
              driverName, functionName, dAcqTimeAct_);
    status |= setDoubleParam(ADAcquireTime, dAcqTimeAct_);
    callParamCallbacks();

    return asynSuccess;
}

//_____________________________________________________________________________________________
bool tpxDetector::isTimeForPreviewUpdate() {
    struct timeval cTime;
    struct timezone zone;
    gettimeofday(&cTime,&zone);
    bool ret=true;
    unsigned long tmpTime=cTime.tv_sec*1e6+cTime.tv_usec;
    if(tmpTime-lastPreviewUpdate>=PREVIEW_UPDATE_PERIOD*1000) {
        ret=true;
        lastPreviewUpdate=tmpTime;
    } else ret=false;

    return ret;
}

//_____________________________________________________________________________________________

int tpxDetector::openRawDataFile(char* directory, char* baseName) {
    int ret;
    char strBuff[1024];

    if(rawDataFile!=NULL) {
        ret=closeRawDataFile();
        if(ret!=0) return ret;
    }


    for(unsigned long i=0; i<ULONG_MAX; i++) {
        sprintf(strBuff,"%s/%s_%lu.tpxRaw",directory,baseName,i);
        FILE *tst=fopen(strBuff,"r");
        if (tst) {
            fclose(tst); // file exists, get another one
        } else {
            break; // file name does not exist yet, so use it
        }
    }

    rawDataFile=fopen(strBuff,"w");

    printf("Opened raw data file: %s\n", strBuff);
    raw_frame_num = 0;
    if(rawDataFile==NULL) return -1;

    return 0;
}

//_____________________________________________________________________________________________

int tpxDetector::writeRawData(u8* data, u32 nbytes, int lostRows) {
    if(rawDataFile==NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: data file not open\n", driverName, "writeRawData");
        return -1;
    }

    u32 ret;
    ret=fwrite((void *) &lostRows,sizeof(int),1,rawDataFile);
    if(ret!=1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: raw data header/Failed to write raw data to file: ret=%d\n", driverName, "writeRawData", ret);
        return -1;
    }


    ret=fwrite((void *) &nbytes,sizeof(u32),1,rawDataFile);
    if(ret!=1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: nbytes/Failed to write raw data to file: ret=%d\n", driverName, "writeRawData", ret);
        return -1;
    }

    ret=fwrite((void *) data,sizeof(u8),nbytes,rawDataFile);
    if(ret!=nbytes) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: data/Failed to write raw data to file: ret=%d\n", driverName, "writeRawData", ret);
        return -1;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s:%s: Raw data written to file\n", driverName, "writeRawData");
    printf("%s:%s: Raw data frame #%d written to file\n", driverName, "writeRawData", raw_frame_num);

    raw_frame_num += 1;
    return 0;
}



//_____________________________________________________________________________________________

int tpxDetector::closeRawDataFile() {
    if(rawDataFile!=NULL) {
        int ret=fclose(rawDataFile);
        if(ret==0) {
            rawDataFile=NULL;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "Raw data file closed\n");
        }
        return ret;
    }
    return 0;
}
//_____________________________________________________________________________________________

bool tpxDetector::setDetectorSet() {

    static const char* functionName = "setDetectorSet";

    FILE  *pInputFile;
    char str [25];
    unsigned long data_lu=0;
    float data_f=0;

    //HwInfoItem hwitm;
    //char tmp[255];
    //hwitm.data = tmp;

    int i=0;
    int ndp=14;

    pInputFile = fopen (cpFileName_HW, "r");

    if (pInputFile != NULL) {
        fscanf(pInputFile, "%s", str);
        sscanf(&str[13], "%f", &data_f);
        if( (*relaxd->setHwInfo)(ids[uiDevIp_], HW_ITEM_TESTPULSELO, &data_f, 4) ) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: setup of hw parameter TestPulseLow failed:\n",
                      driverName, functionName);
            fclose(pInputFile);
            return true;
        }
        fscanf(pInputFile, "%s", str);
        sscanf(&str[14], "%f", &data_f);
        if( (*relaxd->setHwInfo)(ids[uiDevIp_], HW_ITEM_TESTPULSEHI, &data_f, 4) ) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: setup of hw parameter TestPulseHigh failed:\n",
                      driverName, functionName);
            fclose(pInputFile);
            return true;
        }

        for (i=0; i<3; i++) {
            fscanf (pInputFile, "%s", str);
            for(int n=0; n<25; ++n) {
                if (isdigit(str[n])) {
                    data_lu = atoi(&str[n]);
                    break;
                }
            }

            // (0) HW_ITEM_TESTPULSEFREQ          11
            // (1) HW_ITEM_BIAS_VOLTAGE_ADJUST    12
            // (2) HW_ITEM_VDD_ADJUST             13
            if( (*relaxd->setHwInfo)(ids[uiDevIp_], (HW_ITEM_TESTPULSEFREQ+i), &data_lu, 4) ) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: setup of hw parameter number %d failed:\n",
                          driverName, functionName, (i+2));
                fclose(pInputFile);
                return true;
            }
        }

        for (i=0; i<10; i++) {
            fscanf (pInputFile, "%s", str);
            for(int n=0; n<25; ++n) {
                if (isdigit(str[n])) {
                    data_lu = atoi(&str[n]);
                    break;
                }
            }

            // (0) HW_ITEM_FIRSTCHIPNR            18
            // (1) HW_ITEM_LOGVERBOSE             19
            // (2) HW_ITEM_PARREADOUT             20
            // (3) HW_ITEM_STOREPIXELSCFG         21
            // (4) HW_ITEM_STOREDACS              22
            // (5) HW_ITEM_ERASE_STORED_CFG       23
            // (6) HW_ITEM_CONF_TPX_CLOCK         24
            // (7) HW_ITEM_CONF_RO_CLOCK_125MHZ   25
            // (8) HW_ITEM_CONF_TPX_PRECLOCKS     26
            // (9) HW_ITEM_RESERVED               27
            if( (*relaxd->setHwInfo)(ids[uiDevIp_], (HW_ITEM_FIRSTCHIPNR+i), &data_lu, 4) ) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: setup of hw parameter number %d failed:\n",
                          driverName, functionName, (i+5));
                fclose(pInputFile);
                return true;
            }
        }

        fclose(pInputFile);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: opening hw parameters file failed\n",
                  driverName, functionName);
        return true;
    }



    // set pixel configuration

    if( (*relaxd->setPixelsCfgF)(ids[uiDevIp_], cpFileName_PC, TPX_MODE_DONT_SET) ) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error: setPixelsCfg() failed\n",
                  driverName, functionName);
        return true;
    }

    if((*relaxd->resetMatrix)(ids[uiDevIp_])) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Error: resetMatrix() failed\n",
                  driverName, functionName);
        return true;
    }


    // set readout configuration
    for( i=0; i<ndevs; i++ ) {
        if( (*relaxd->setFsr)(ids[uiDevIp_], i, &TDD[i*ndp], 0) ) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: setFsr() chip %d failed\n",
                      driverName, functionName, i);
            return true;
        }
    }
    return false;
}



void tpxDetector::resetDetector (void) {
    //int status = asynSuccess;
    int acquire;
    static const char *functionName = "detectorReset";
    getIntegerParam(ADAcquire, &acquire);
    if (driverReady) {
        if ((*relaxd->reset)(ids[uiDevIp_])) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Cannot reset chips!\n",driverName, functionName);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Reset chips\n",driverName, functionName);
            if (this->setDetectorSet()) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Error: Problem with setting detector!\n", driverName, functionName);
            }
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Detector was reset!\n",driverName, functionName);
        }
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Driver ready = %d; acquire = %d;\n",driverName, functionName, driverReady, acquire);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Check that detector is initialized and not acquiring!\n",driverName, functionName);
    }
}




//_____________________________________________________________________________________________

/* Code for iocsh registration */

/* tpxDetectorConfig */
static const iocshArg tpxDetectorConfigArg0 = {"Port name", iocshArgString};
static const iocshArg tpxDetectorConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg tpxDetectorConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg tpxDetectorConfigArg3 = {"priority", iocshArgInt};

static const iocshArg tpxDetectorConfigArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const tpxDetectorConfigArgs[] =  {&tpxDetectorConfigArg0,
                                                          &tpxDetectorConfigArg1,
                                                          &tpxDetectorConfigArg2,
                                                          &tpxDetectorConfigArg3,
                                                          &tpxDetectorConfigArg4
                                                         };
static const iocshFuncDef configtpxDetector = {"tpxDetectorConfig", 5, tpxDetectorConfigArgs};
static void configtpxDetectorCallFunc(const iocshArgBuf *args) {
    tpxDetectorConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}


static void tpxDetectorRegister(void) {
    iocshRegister(&configtpxDetector, configtpxDetectorCallFunc);
}

extern "C" {
    epicsExportRegistrar(tpxDetectorRegister);
}


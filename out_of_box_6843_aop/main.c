// =============================
// FILE: main_range_only.c
// =============================
//
// Range-only variant of mmWave Demo main.c
// - Range profile & stats/temperature over UART
// - Added detailed timing capture (uses gRangeTiming from DPC)
//

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#include <ti/sysbios/utils/Load.h>

/* mmWave SDK */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/dpm/dpm.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>
#include <ti/demo/xwr64xx/mmw/mmw_output.h>

/* Demo includes */
#include <ti/demo/xwr64xx/mmw/mmw.h>
#include <ti/demo/xwr64xx/mmw/data_path.h>
#include <ti/demo/xwr64xx/mmw/mmw_config.h>
#include <ti/demo/xwr64xx/mmw/mmw_res.h>
#include <ti/board/antenna_geometry.h>
#include <ti/demo/utils/mmwdemo_flash.h>
#define NOW_TICKS()     (Cycleprofiler_getTimeStamp())
#define TICKS_TO_US(dt) ((float)((float)(dt) / R4F_CLOCK_MHZ))
/* Range profile output scaling (we send u16 to GUI but keep u32 internally) */
#define MMWDEMO_RANGE_PROFILE_BITS   32
typedef uint32_t mmw_range_elem32_t;
typedef uint16_t mmw_range_elem16_t;
/* Timing struct from DPC */
typedef struct {
    uint32_t t_frame_isr, t_exec_enter, t_range_begin, t_range_end, t_mag_begin, t_mag_end, t_uart_begin, t_uart_end, t_exec_leave, t_win_begin, t_win_end;
    float us_window_gen, us_range_proc, us_mag_build, us_uart_tx, us_total_execute, us_ramp_from_frame_to_range;
    /* NEW: RF "active frame (burst)" time and period */
    float us_active_frame_burst;
    float framePeriod_ms;
    float us_chirp_interval;
} RangeOnlyTiming;



extern volatile RangeOnlyTiming gRangeTiming[1/*RL_MAX_SUBFRAMES*/];

struct timeMeasure{
    float winTime, fftTime, rampTime, adcStartTime, cfarTime;
};

typedef struct
{
    /* mmWave_execute() in ctrl task */
    uint32_t mmwaveExecCnt;
    /* DPM_execute() in DPC task (per frame) */
    uint32_t dpmExecCnt;
    /* Pending dynamic config processing */
    uint32_t pendCfgCnt;
    /* Total processing in DPC task for one result (from result ready to done) */
    uint32_t frameProcCnt;
    /* Count of processed frames (results) */
    uint32_t frameCount;
    uint32_t objReportCnt;
    uint32_t cfartoLinerCnt;

} CycleProfileStats;

volatile CycleProfileStats gCycleStats = {0};



/** Task priorities */
#define MMWDEMO_CLI_TASK_PRIORITY                 3
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY      4
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY         5
#if (MMWDEMO_CLI_TASK_PRIORITY >= MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY)
#error CLI task priority must be < Object Detection DPM task priority
#endif

/* CQ address offsets/alignments */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET          0x800U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET           0x1000U
#define MMW_DEMO_CQ_DATA_ALIGNMENT              16U

/* L3/TCM working buffers for DPC */
uint8_t gMmwL3[SOC_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram")

#define MMWDEMO_OBJDET_TCM_SIZE (49U * 1024U)
uint8_t gDPC_ObjDetTCM[MMWDEMO_OBJDET_TCM_SIZE];
#pragma DATA_SECTION(gDPC_ObjDetTCM, ".DPC_objDetTcmbHeap")

/**************************************************************************
 *************************** Globals **************************************
 **************************************************************************/
MmwDemo_MCB    gMmwMCB;

/* LDO BYPASS config */
rlRfLdoBypassCfg_t gRFLdoBypassCfg = { .ldoBypassEnable=0, .supplyMonIrDrop=0, .ioSupplyIndicator=0 };

/* Calib store/restore */
#define MMWDEMO_CALIB_FLASH_SIZE 4096
#define MMWDEMO_CALIB_STORE_MAGIC (0x7CB28DF9U)
MmwDemo_calibData gCalibDataStorage;
#pragma DATA_ALIGN(gCalibDataStorage, 8);

extern void MmwDemo_CLIInit(uint8_t taskPriority);

/* Prototypes */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj);
int32_t MmwDemo_dataPathConfig(void);
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathStart (void);
void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj);
void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                     DPC_ObjectDetection_ExecuteResult *result,
                                     MmwDemo_output_message_stats *timingInfo);
void MmwDemo_initTask(UArg arg0, UArg arg1);
static int32_t MmwDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx);
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);
void MmwDemo_sleep(void);
static int32_t MmwDemo_calibInit(void);
static int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData);
static int32_t MmwDemo_calibRestore(MmwDemo_calibData *calibrationData);
static void MmwDemo_DPC_ObjectDetection_reportFxn(DPM_Report reportType, uint32_t instanceId, int32_t errCode, uint32_t arg0, uint32_t arg1);
void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx);
void MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx);
void MmwDemo_platformInit(MmwDemo_platformCfg *config);
void MmwDemo_getTemperatureReport();
/* Debug assert routed to CLI */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{ if (!expression) { CLI_write("Exception: %s, line %d.\n", file, line); } }

/* ---------------- Pending-state helpers ---------------- */
/* (unchanged helper implementations from your file)... */

static void MmwDemo_setSubFramePendingState(MmwDemo_SubFrameCfg *subFrameCfg, uint32_t offset)
{
    switch (offset)
    {
        case MMWDEMO_GUIMONSEL_OFFSET:
            subFrameCfg->objDetDynCfg.isPrepareRangeAzimuthHeatMapPending = 1;
        break;
        case MMWDEMO_CFARCFGRANGE_OFFSET:
            subFrameCfg->objDetDynCfg.isCfarCfgRangePending = 1;
        break;
        case MMWDEMO_FOVRANGE_OFFSET:
            subFrameCfg->objDetDynCfg.isFovRangePending = 1;
        break;
        case MMWDEMO_CALIBDCRANGESIG_OFFSET:
            subFrameCfg->objDetDynCfg.isCalibDcRangeSigCfg = 1;
        break;
        case MMWDEMO_ADCBUFCFG_OFFSET:
            subFrameCfg->isAdcBufCfgPending = 1;
        break;
        case MMWDEMO_LVDSSTREAMCFG_OFFSET:
            subFrameCfg->isLvdsStreamCfgPending = 1;
        break;
        default:
        break;
    }
}
static uint8_t MmwDemo_isDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    return (cfg->isCompRxChannelBiasCfgPending == 1) && (cfg->isMeasureRxChannelBiasCfgPending == 1);
}
static uint8_t MmwDemo_isDynObjDetCommonCfgInNonPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    return (cfg->isCompRxChannelBiasCfgPending == 0) && (cfg->isMeasureRxChannelBiasCfgPending == 0);
}
static uint8_t MmwDemo_isDynObjDetCfgPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    return (cfg->isCalibDcRangeSigCfg==1) && (cfg->isCfarCfgRangePending==1) && (cfg->isFovRangePending==1);
}
static uint8_t MmwDemo_isDynObjDetCfgInNonPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    return (cfg->isCalibDcRangeSigCfg==0) && (cfg->isCfarCfgRangePending==0) && (cfg->isFovRangePending==0);
}
static void MmwDemo_resetDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    cfg->isCompRxChannelBiasCfgPending = 0; cfg->isMeasureRxChannelBiasCfgPending = 0;
}
static void MmwDemo_resetDynObjDetCfgPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    cfg->isCalibDcRangeSigCfg=0;
    cfg->isCfarCfgRangePending=0;
    cfg->isFovRangePending=0;
    cfg->isPrepareRangeAzimuthHeatMapPending=0;
}
void MmwDemo_resetStaticCfgPendingState(void)
{
    uint8_t i;
    for(i=0;i<gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames;i++){
        gMmwMCB.subFrameCfg[i].isAdcBufCfgPending=0;
        gMmwMCB.subFrameCfg[i].isLvdsStreamCfgPending=0;
    }
    gMmwMCB.isAnaMonCfgPending=0;
}
uint8_t MmwDemo_isAllCfgInPendingState(void)
{
    uint8_t i,flag=1;
    for(i=0;i<gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames;i++){
        flag=flag&&MmwDemo_isDynObjDetCfgPendingState(&gMmwMCB.subFrameCfg[i].objDetDynCfg);
        flag=flag&&(gMmwMCB.subFrameCfg[i].isAdcBufCfgPending==1);
        flag=flag&&(gMmwMCB.subFrameCfg[i].isLvdsStreamCfgPending==1);
    }
    flag=flag&&MmwDemo_isDynObjDetCommonCfgPendingState(&gMmwMCB.dataPathObj.objDetCommonCfg);
    flag=flag&&(gMmwMCB.isAnaMonCfgPending==1);
    return flag;
}
uint8_t MmwDemo_isAllCfgInNonPendingState(void)
{
    uint8_t i,flag=1;
    for(i=0;i<gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames;i++){
        flag=flag&&MmwDemo_isDynObjDetCfgInNonPendingState(&gMmwMCB.subFrameCfg[i].objDetDynCfg);
        flag=flag&&(gMmwMCB.subFrameCfg[i].isAdcBufCfgPending==0);
        flag=flag&&(gMmwMCB.subFrameCfg[i].isLvdsStreamCfgPending==0);
    }
    flag=flag&&MmwDemo_isDynObjDetCommonCfgInNonPendingState(&gMmwMCB.dataPathObj.objDetCommonCfg);
    flag=flag&&(gMmwMCB.isAnaMonCfgPending==0);
    return flag;
}
void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{
    if(subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG){
        uint8_t i;
        for( i=0;i<1;/*RL_MAX_SUBFRAMES;*/i++){
            memcpy((void*)((uint32_t)&gMmwMCB.subFrameCfg[i]+offset), srcPtr, size);
            MmwDemo_setSubFramePendingState(&gMmwMCB.subFrameCfg[i], offset);
        }
    }
    else {
        memcpy((void*)((uint32_t)&gMmwMCB.subFrameCfg[subFrameNum]+offset), srcPtr, size);
        MmwDemo_setSubFramePendingState(&gMmwMCB.subFrameCfg[subFrameNum], offset);
    }
}

/* ---------------- MMWave start/stop helpers ---------------- */
void MmwDemo_MMWave_stop(void)
{
    int32_t errCode; DebugP_log0("App: Issuing MMWave_stop\n");
    if (MMWave_stop (gMmwMCB.ctrlHandle, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel; int16_t mmWaveErrorCode, subsysErrorCode;
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR){
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n", mmWaveErrorCode, subsysErrorCode);
            MmwDemo_debugAssert(0);
        } else {
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n", mmWaveErrorCode, subsysErrorCode);
        }
    }
}

void MmwDemo_sensorStopEpilog(void)
{
    Task_Stat stat; Hwi_StackInfo stackInfo; Bool stackOverflow; System_printf("Data Path Stopped (last frame processing done)\n");
    System_printf("Task Stack Usage (Note: CLI and sensor Management Task not reported) ==========\n");
    System_printf("%20s %12s %12s %12s\n", "Task Name","Size","Used","Free");
    Task_stat(gMmwMCB.taskHandles.initTask,&stat);
    System_printf("%20s %12d %12d %12d\n","Init",stat.stackSize,stat.used,stat.stackSize-stat.used);
    Task_stat(gMmwMCB.taskHandles.mmwaveCtrl,&stat);
    System_printf("%20s %12d %12d %12d\n","Mmwave Control",stat.stackSize,stat.used,stat.stackSize-stat.used);
    Task_stat(gMmwMCB.taskHandles.objDetDpmTask,&stat);
    System_printf("%20s %12d %12d %12d\n","ObjDet DPM",stat.stackSize,stat.used,stat.stackSize-stat.used);
    System_printf("HWI Stack (same as System Stack) Usage ============\n");
    stackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (stackOverflow == TRUE){ System_printf("HWI Stack overflowed\n"); MmwDemo_debugAssert(0);} else {
        System_printf("%20s %12s %12s %12s\n"," ","Size","Used","Free");
        System_printf("%20s %12d %12d %12d\n"," ",stackInfo.hwiStackSize,stackInfo.hwiStackPeak,stackInfo.hwiStackSize-stackInfo.hwiStackPeak);
    }
}

int32_t MmwDemo_mssSetHsiClk(void)
{
    rlDevHsiClk_t hsiClkgs;
    int32_t retVal;
    memset(&hsiClkgs,0,sizeof(hsiClkgs));
    hsiClkgs.hsiClk = 0x9;
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_CASCADED_1,&hsiClkgs);
    if(retVal != RL_RET_CODE_OK){ System_printf("Error: Setting up the HSI Clock Failed [Error %d]\n", retVal);
    return -1;
    }
    Task_sleep(HSI_DCA_MIN_DELAY_MSEC);
    return 0;
}

/* ---------------- Open/config/start sensor ---------------- */
int32_t MmwDemo_openSensor(bool isFirstTimeOpen)
{
    int32_t retVal, errCode;
    if(isFirstTimeOpen){
        System_printf ("Debug: Sending rlRfSetLdoBypassConfig with %d %d %d\n", gRFLdoBypassCfg.ldoBypassEnable, gRFLdoBypassCfg.supplyMonIrDrop, gRFLdoBypassCfg.ioSupplyIndicator);
        retVal = rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlRfLdoBypassCfg_t*)&gRFLdoBypassCfg);
        if(retVal!=0){ System_printf("Error: rlRfSetLdoBypassConfig retVal=%d\n", retVal); return -1; }
        gMmwMCB.cfg.openCfg.freqLimitLow=600U;
        gMmwMCB.cfg.openCfg.freqLimitHigh=640U;
        gMmwMCB.cfg.openCfg.disableFrameStartAsyncEvent=false;
        gMmwMCB.cfg.openCfg.disableFrameStopAsyncEvent=false;
        gMmwMCB.cfg.openCfg.useCustomCalibration=false;
        gMmwMCB.cfg.openCfg.customCalibrationEnableMask=0x0;
        gMmwMCB.cfg.openCfg.calibMonTimeUnit=1;
        if((gMmwMCB.calibCfg.saveEnable!=0)&&(gMmwMCB.calibCfg.restoreEnable!=0)){
            System_printf("Error: save and restore both enabled.\n");
            return -1;
        }
        MMWave_CalibrationData *ptrCalibrationDataCfg = NULL;
        MMWave_CalibrationData calibrationDataCfg;
        if(gMmwMCB.calibCfg.restoreEnable!=0){
            if(MmwDemo_calibRestore(&gCalibDataStorage)<0){
                System_printf("Error: restoring calibration data from flash.\n");
                return -1;
            }
            gMmwMCB.cfg.openCfg.useCustomCalibration=true;
            gMmwMCB.cfg.openCfg.customCalibrationEnableMask = 0x1F0U;
            calibrationDataCfg.ptrCalibData=&gCalibDataStorage.calibData;
            calibrationDataCfg.ptrPhaseShiftCalibData=&gCalibDataStorage.phaseShiftCalibData;
            ptrCalibrationDataCfg=&calibrationDataCfg;
        }
        if (MMWave_open(gMmwMCB.ctrlHandle,&gMmwMCB.cfg.openCfg,ptrCalibrationDataCfg,&errCode) < 0){
            MMWave_ErrorLevel errorLevel;
            int16_t mmWaveErrorCode, subsysErrorCode;
            MMWave_decodeError(errCode,&errorLevel,&mmWaveErrorCode,&subsysErrorCode);
            System_printf("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n", mmWaveErrorCode, subsysErrorCode);
            return -1;
        }
        if(gMmwMCB.calibCfg.saveEnable!=0){
            retVal = rlRfCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS,&gCalibDataStorage.calibData);
            if(retVal!=RL_RET_CODE_OK){ System_printf("MSS demo failed rlRfCalibDataStore with Error[%d]\n", retVal); return -1; }
            retVal = MmwDemo_calibSave(&gMmwMCB.calibCfg.calibDataHdr,&gCalibDataStorage);
            if(retVal<0){ return retVal; }
        }
        if(MmwDemo_mssSetHsiClk()<0){ System_printf("Error: MmwDemo_mssSetHsiClk failed.\n"); return -1; }
    }
    return 0;
}

int32_t MmwDemo_configSensor(void)
{
    int32_t errCode=0;
    if (MMWave_config(gMmwMCB.ctrlHandle,&gMmwMCB.cfg.ctrlCfg,&errCode) < 0){
        MMWave_ErrorLevel errorLevel; int16_t mmWaveErrorCode,subsysErrorCode;
        MMWave_decodeError(errCode,&errorLevel,&mmWaveErrorCode,&subsysErrorCode);
        System_printf("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n", mmWaveErrorCode, subsysErrorCode);
    }
    else { errCode = MmwDemo_dataPathConfig(); }
    return errCode;
}

int32_t MmwDemo_startSensor(void)
{
    int32_t errCode;
    MmwDemo_dataPathStart();
    MMWave_CalibrationCfg calibrationCfg;
    memset(&calibrationCfg,0,sizeof(calibrationCfg));
    calibrationCfg.dfeDataOutputMode = gMmwMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 1U;
    DebugP_log0("App: MMWave_start Issued\n");
    System_printf("Starting Sensor (issuing MMWave_start)\n");
    if(MMWave_start(gMmwMCB.ctrlHandle,&calibrationCfg,&errCode) < 0){
        MMWave_ErrorLevel errorLevel; int16_t mmWaveErrorCode,subsysErrorCode;
        MMWave_decodeError(errCode,&errorLevel,&mmWaveErrorCode,&subsysErrorCode);
        System_printf("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
        MmwDemo_debugAssert(0);
        return -1;
    }
    GPIO_write(gMmwMCB.cfg.platformCfg.SensorStatusGPIO,1U);
    gMmwMCB.sensorStartCount++;
    return 0;
}

/* Convert CFAR dB threshold to linear if needed (kept) */
static uint16_t MmwDemo_convertCfarToLinear(uint16_t codedCfarVal, uint8_t numVirtualAntennas)
{
    gCycleStats.cfartoLinerCnt++;
    float dbVal=(float)(codedCfarVal / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
    float linVal = dbVal * (256.0f/6.0f) * ((float)numVirtualAntennas/(float)(1<<mathUtils_ceilLog2(numVirtualAntennas)));
    if(linVal<0) linVal=0;
    if(linVal>65535.0f) linVal=65535.0f;
    return (uint16_t)linVal;
}

void MmwDemo_stopSensor()
{
    int32_t errCode;
    MmwDemo_MMWave_stop();
    Semaphore_pend(gMmwMCB.DPMstopSemHandle, BIOS_WAIT_FOREVER);
    if(gMmwMCB.lvdsStream.hwSessionHandle != NULL){
        int32_t r;
        if ((gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames == 1) ||
            ((gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) &&
             (gMmwMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))){
            if((r=CBUFF_deactivateSession(gMmwMCB.lvdsStream.hwSessionHandle,&errCode))<0){
                System_printf("CBUFF_deactivateSession failed err=%d\n", errCode);
                MmwDemo_debugAssert(0);
            }
        }
        MmwDemo_LVDSStreamDeleteHwSession();
    }
    if(gMmwMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession();
    }
    MmwDemo_sensorStopEpilog();
    GPIO_write(gMmwMCB.cfg.platformCfg.SensorStatusGPIO,0U);
    gMmwMCB.sensorStopCount++;
    System_printf("Sensor has been stopped: startCount: %d stopCount %d\n", gMmwMCB.sensorStartCount,gMmwMCB.sensorStopCount);
}

/* ---------------- Data path init/open ---------------- */
void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj)
{
    MmwDemo_dataPathObjInit(obj);
    MmwDemo_hwaInit(obj);
    MmwDemo_edmaInit(obj);
}
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj)
{
    MmwDemo_hwaOpen(obj, gMmwMCB.socHandle);
    MmwDemo_edmaOpen(obj);
    obj->adcbufHandle = MmwDemo_ADCBufOpen(gMmwMCB.socHandle);
}

/* ---------------- CQ config (unchanged core) ---------------- */
static int32_t MmwDemo_configCQ(MmwDemo_SubFrameCfg *subFrameCfg, uint8_t numChirpsPerChirpEvent, uint8_t validProfileIdx)
{
    MmwDemo_AnaMonitorCfg* ptrAnaMonitorCfg = &gMmwMCB.anaMonCfg;
    ADCBuf_CQConf cqConfig;
    rlRxSatMonConf_t* ptrSatMonCfg;
    rlSigImgMonConf_t* ptrSigImgMonCfg;
    int32_t retVal=0;
    uint16_t cqChirpSize;
    ptrSatMonCfg = &gMmwMCB.cqSatMonCfg[validProfileIdx];
    ptrSigImgMonCfg = &gMmwMCB.cqSigImgMonCfg[validProfileIdx];
    if(ptrAnaMonitorCfg->rxSatMonEn){
        if(ptrSatMonCfg->profileIndx!=validProfileIdx){
            System_printf("Error: Saturation mon not configured for profile(%d)\n", validProfileIdx);
            MmwDemo_debugAssert(0);
        }
        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if(retVal!=0){
            System_printf("Error: rlRfRxIfSatMonConfig error=%d profile(%d)\n", retVal, ptrSatMonCfg->profileIndx);
            return retVal;
        }
    }

    if(ptrAnaMonitorCfg->sigImgMonEn){
        if(ptrSigImgMonCfg->profileIndx!=validProfileIdx){
            System_printf("Error: SigImg mon not configured for profile(%d)\n", validProfileIdx);
            MmwDemo_debugAssert(0);
        }
        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal!=0){
            System_printf("Error: rlRfRxSigImgMonConfig error=%d profile(%d)\n", retVal, ptrSigImgMonCfg->profileIndx);
            return retVal;
        }
    }
    retVal = mmwDemo_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if(retVal!=0){
        System_printf("Error: rlRfAnaMonConfig error=%d\n", retVal);
        return retVal;
    }
    if(ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn){
        memset(&cqConfig,0,sizeof(cqConfig));
        cqConfig.cqDataWidth=0;
        cqConfig.cq1AddrOffset=MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET;
        cqConfig.cq2AddrOffset=MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;
        retVal = ADCBuf_control(gMmwMCB.dataPathObj.adcbufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if(retVal<0){
            System_printf("Error: Unable to configure CQ\n");
            MmwDemo_debugAssert(0);
        }
    }
    if(ptrAnaMonitorCfg->sigImgMonEn){
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->sigImgMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }
    if(ptrAnaMonitorCfg->rxSatMonEn){
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->satMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }
    return retVal;
}
//
///* ---------------- Data path config (range-only tweaks) ---------------- */
static MmwDemo_SubFrameCfg *subFrameCfg;
static MMWave_CtrlCfg      *ptrCtrlCfg;
static MmwDemo_RFParserOutParams RFparserOutParams;
static MmwDemo_DataPathObj *dataPathObj;

int32_t MmwDemo_dataPathConfig (void)
{
    int32_t errCode;
    dataPathObj = &gMmwMCB.dataPathObj;
    int8_t subFrameIndx;
    ptrCtrlCfg = &gMmwMCB.cfg.ctrlCfg;
    gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames = MmwDemo_RFParser_getNumSubFrames(ptrCtrlCfg);
    DebugP_log0("App: Issuing Pre-start Common Config IOCTL\n");
    gMmwMCB.rfFreqScaleFactor = SOC_getDeviceRFFreqScaleFactor(gMmwMCB.socHandle,&errCode);
    if(errCode<0){
        System_printf("Error: Unable to get RF scale factor [Error:%d]\n", errCode);
        return errCode;
    }

    extern ANTDEF_AntGeometry gAntDef_IWR6843AOP;
    dataPathObj->objDetCommonCfg.preStartCommonCfg.antDef = gAntDef_IWR6843AOP;

    errCode = DPM_ioctl (dataPathObj->objDetDpmHandle,
                         DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         &dataPathObj->objDetCommonCfg.preStartCommonCfg,
                         sizeof (DPC_ObjectDetection_PreStartCommonCfg));
    if (errCode < 0){
        System_printf("Error: PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        return errCode;
    }
    MmwDemo_resetDynObjDetCommonCfgPendingState(&dataPathObj->objDetCommonCfg);

    for(subFrameIndx = gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames - 1; subFrameIndx >= 0; subFrameIndx--)
    {
        subFrameCfg  = &gMmwMCB.subFrameCfg[subFrameIndx];
        errCode = MmwDemo_RFParser_parseConfig(&RFparserOutParams, subFrameIndx,
                                         &gMmwMCB.cfg.openCfg, ptrCtrlCfg,
                                         &subFrameCfg->adcBufCfg,
                                         gMmwMCB.rfFreqScaleFactor,
                                         false);
        if (errCode != 0){
            System_printf("Error: RFParser_parseConfig [Error:%d]\n", errCode);
            return errCode;
        }

        /* Force a single virtual antenna for range-only. */
        subFrameCfg->numRangeBins = RFparserOutParams.numRangeBins;
        if ((RFparserOutParams.numVirtualAntennas == 12) && (RFparserOutParams.numRangeBins == 1024))
        {
            subFrameCfg->numRangeBins = 1022; RFparserOutParams.numRangeBins = 1022;
        }

        /* Force range-only, single virtual antenna, and single Doppler bin */
        RFparserOutParams.numVirtualAntennas = 1;
        subFrameCfg->numVirtualAntennas      = 1;
        subFrameCfg->numDopplerBins          = RFparserOutParams.numDopplerBins; //1;

        /* Do NOT touch adcBufChanDataSize or numChirpsPerFrame here */
        subFrameCfg->numChirpsPerSubFrame    = RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numChirpsPerChirpEvent  = RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize      = RFparserOutParams.adcBufChanDataSize;
        subFrameCfg->numAdcSamples           = RFparserOutParams.numAdcSamples;
        subFrameCfg->numVirtualAntennas      = RFparserOutParams.numVirtualAntennas;
        subFrameCfg->numChirpsPerSubFrame    = RFparserOutParams.numChirpsPerFrame;
        {
            float burst_us =
                (float)RFparserOutParams.numChirpsPerFrame * (RFparserOutParams.chirpInterval * 1e6f);
            gRangeTiming[subFrameIndx].us_active_frame_burst = burst_us;
            //gRangeTiming[subFrameIndx].framePeriod_ms        = RFparserOutParams.framePeriod;
        }
        // seconds -> microseconds
        gRangeTiming[subFrameIndx].us_chirp_interval = RFparserOutParams.chirpInterval * 1e6f;

        DPC_ObjectDetection_PreStartCfg objDetPreStartCfg;
        DPC_ObjectDetection_StaticCfg *staticCfg = &objDetPreStartCfg.staticCfg;

        int32_t adcbufRet = MmwDemo_ADCBufConfig(gMmwMCB.dataPathObj.adcbufHandle,
                                             gMmwMCB.cfg.openCfg.chCfg.rxChannelEn,
                                             subFrameCfg->numChirpsPerChirpEvent,
                                             subFrameCfg->adcBufChanDataSize,
                                             &subFrameCfg->adcBufCfg,
                                             &staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
        if (adcbufRet < 0){ System_printf("Error: ADCBuf config failed [%d]\n", adcbufRet); return adcbufRet; }

        errCode = MmwDemo_configCQ(subFrameCfg, RFparserOutParams.numChirpsPerChirpEvent, RFparserOutParams.validProfileIdx);
        if (errCode < 0){ return errCode; }



        /* DPC pre-start cfg */
        objDetPreStartCfg.subFrameNum = subFrameIndx;
        //staticCfg->ADCBufData.data = (void *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
        staticCfg->ADCBufData.data = (int16_t *)((uintptr_t)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS + staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
        staticCfg->ADCBufData.dataProperty.adcBits = 2; /* 16-bit */
        MmwDemo_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 0); /* complex */
        staticCfg->ADCBufData.dataProperty.dataFmt = (subFrameCfg->adcBufCfg.iqSwapSel==1)? DPIF_DATAFORMAT_COMPLEX16_IMRE : DPIF_DATAFORMAT_COMPLEX16_REIM;
        staticCfg->ADCBufData.dataProperty.interleave = (subFrameCfg->adcBufCfg.chInterleave==0)? DPIF_RXCHAN_INTERLEAVE_MODE : DPIF_RXCHAN_NON_INTERLEAVE_MODE;
        RFparserOutParams.numRxAntennas = 1;
        staticCfg->ADCBufData.dataProperty.numAdcSamples = RFparserOutParams.numAdcSamples;
        staticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
        staticCfg->ADCBufData.dataProperty.numRxAntennas = RFparserOutParams.numRxAntennas;
        staticCfg->ADCBufData.dataSize = RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
        staticCfg->dopplerStep = RFparserOutParams.dopplerStep;
        staticCfg->isValidProfileHasOneTxPerChirp = RFparserOutParams.validProfileHasOneTxPerChirp;
        staticCfg->numChirpsPerFrame = RFparserOutParams.numChirpsPerFrame;
        staticCfg->numDopplerBins = RFparserOutParams.numDopplerBins;
        staticCfg->numDopplerChirps = RFparserOutParams.numDopplerChirps;
        staticCfg->numRangeBins = RFparserOutParams.numRangeBins;
        staticCfg->numTxAntennas = RFparserOutParams.numTxAntennas;
        staticCfg->numVirtualAntAzim = RFparserOutParams.numVirtualAntAzim;
        staticCfg->numVirtualAntElev = RFparserOutParams.numVirtualAntElev;
        staticCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
        staticCfg->rangeStep = RFparserOutParams.rangeStep;
        staticCfg->centerFreq = RFparserOutParams.centerFreq;
        staticCfg->rangeFFTtuning.fftOutputDivShift = 3;
        staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 0;
        staticCfg->rxAntOrder[0] = RFparserOutParams.rxAntOrder[0];
        staticCfg->txAntOrder[0] = RFparserOutParams.txAntOrder[0];

        subFrameCfg->objDetDynCfg.dynCfg.cfarCfgRange.thresholdScale =
            MmwDemo_convertCfarToLinear(subFrameCfg->objDetDynCfg.dynCfg.cfarCfgRange.thresholdScale, staticCfg->numVirtualAntennas);

        objDetPreStartCfg.dynCfg = subFrameCfg->objDetDynCfg.dynCfg;

        DebugP_log1("App: Issuing Pre-start Config IOCTL (subFrameIndx = %d)\n", subFrameIndx);
        errCode = DPM_ioctl (dataPathObj->objDetDpmHandle,
                             DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                             &objDetPreStartCfg,
                             sizeof (DPC_ObjectDetection_PreStartCfg));
        MmwDemo_resetDynObjDetCfgPendingState(&subFrameCfg->objDetDynCfg);
        if (errCode < 0){
            System_printf("Error: PRE_START_CFG [Error:%d]\n", errCode);
            return errCode;
        }
    }
    return 0;
}

void MmwDemo_dataPathStart (void)
{
    int32_t errCode; DebugP_log0("App: Issuing DPM_start\n");
    if (gMmwMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED){
        MmwDemo_configLVDSHwData(0);
    }
    if ((errCode = DPM_start(gMmwMCB.dataPathObj.objDetDpmHandle)) < 0){
        System_printf("Error: Unable to start DPM [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
    }
    Semaphore_pend(gMmwMCB.DPMstartSemHandle, BIOS_WAIT_FOREVER);
    DebugP_log0("App: DPM_start Done\n");
}

void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj)
{
    int32_t retVal;
    DebugP_log0("App: Issuing DPM_stop\n");
    retVal = DPM_stop (obj->objDetDpmHandle);
    if (retVal < 0){
        System_printf ("DPM_stop failed[Error %d]\n", retVal);
        MmwDemo_debugAssert(0);
    }
}
uint8_t cycle_cnt;
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    while(1){
        if(MMWave_execute(gMmwMCB.ctrlHandle,&errCode) < 0){
            MmwDemo_debugAssert(0);
        }
        gCycleStats.mmwaveExecCnt++;
    }
}


int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);
    gCycleStats.pendCfgCnt++;
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                    case RL_RF_AE_ESMFAULT_SB:
                        case RL_RF_AE_ANALOG_FAULT_SB:
                            MmwDemo_debugAssert(0);
                            break;
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t calibrationStatus;
                    ptrRFInitCompleteMessage=(rlRfInitComplete_t*)payload;
                    calibrationStatus=ptrRFInitCompleteMessage->calibStatus & 0x1FFFU;
                    CLI_write("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                    gMmwMCB.stats.frameTriggerReady++;
                    break;
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                    gMmwMCB.stats.failedTimingReports++;
                    break;
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                    gMmwMCB.stats.calibrationReports++;
                    break;
                case RL_RF_AE_FRAME_END_SB:
                {
                    gMmwMCB.stats.sensorStopped++;
                    DebugP_log0("App: BSS stop (frame end) received\n");
                    MmwDemo_dataPathStop(&gMmwMCB.dataPathObj);
                    break;
                }
                default:
                    System_printf("Error: Async SB Id %d not handled\n", asyncSB);
                    break;
            }
            break;
        }
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            { case RL_MMWL_AE_MISMATCH_REPORT:
                case RL_MMWL_AE_INTERNALERR_REPORT:
                    MmwDemo_debugAssert(0);
                    break;
            }
            break;
        }
        default:
            System_printf("Error: Async message %d NOT handled\n", msgId);
            break;
    }
    return 0;
}

/* ---------------- Scaling helpers ---------------- */
static inline uint32_t clamp_u32(uint32_t x, uint32_t a, uint32_t b)
{
    return (x < a) ? a : (x > b) ? b : x;
}

static inline uint16_t scale_u32_to_u16_linear(uint32_t in,
                                               uint32_t in_min,
                                               uint32_t in_max,
                                               uint16_t out_min,
                                               uint16_t out_max)
{
    if (in_max == in_min) return out_min;
    uint32_t in_clamped = clamp_u32(in, in_min, in_max);
    uint64_t num = (uint64_t)(in_clamped - in_min) * (uint64_t)(out_max - out_min);
    uint32_t den = (in_max - in_min);
    return (uint16_t)(out_min + (uint16_t)(num / den));
}
static inline uint16_t u32_full_to_u16(uint32_t x)
{
    return scale_u32_to_u16_linear(x, 0u, 0xFFFFFFFFu, 0u, 0xFFFFu);
}

/* ---------------- UART TX (range profile + stats + temp) ---------------- */
static MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];
uint32_t transmitBufferHWA32[128];
//uint16_t transmitBufferHWA16[128];

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                     DPC_ObjectDetection_ExecuteResult *result,
                                     MmwDemo_output_message_stats *timingInfo)
{
    MmwDemo_output_message_header header;
    MmwDemo_SubFrameCfg *sf = &gMmwMCB.subFrameCfg[result->subFrameIdx];
//    MmwDemo_GuiMonSel *pGuiMonSel = &sf->guiMonSel;

    uint32_t tlvIdx = 0, i;
    uint32_t numPaddingBytes, packetLen;
    uint8_t  padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    const mmw_range_elem32_t *det32 = (const mmw_range_elem32_t*)result->detMatrix.data;
    //const mmw_range_elem16_t *det16 = (const mmw_range_elem16_t*)result->detMatrix.data;

    memset(&header, 0, sizeof(header));
    header.platform = 0xA6843;
    header.magicWord[0]=0x0102;
    header.magicWord[1]=0x0304;
    header.magicWord[2]=0x0506;
    header.magicWord[3]=0x0708;
    header.numDetectedObj = result->numObjOut;
    header.version =  MMWAVE_SDK_VERSION_BUILD |
                      (MMWAVE_SDK_VERSION_BUGFIX<<8) |
                      (MMWAVE_SDK_VERSION_MINOR<<16) |
                      (MMWAVE_SDK_VERSION_MAJOR<<24);

    packetLen = sizeof(MmwDemo_output_message_header);

    /* RANGE PROFILE (we send as u16 for bandwidth) */
    tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
    tl[tlvIdx].length = sf->numRangeBins * sizeof(uint16_t);
    packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
    tlvIdx++;

    /* STATS + TEMP (optional) */
//    if (pGuiMonSel->statsInfo){
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;

        MmwDemo_getTemperatureReport();
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_temperatureStats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
//    }

    header.numTLVs        = tlvIdx;
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
                            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1)) /
                              MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles  = Pmu_getCount(0);
    header.frameNumber    = result->stats->frameStartIntCounter;
    header.subFrameNumber = result->subFrameIdx;

    UART_writePolling(uartHandle, (uint8_t*)&header, sizeof(header));

    /* TLV: RANGE PROFILE (u16) */
    tlvIdx = 0;
    UART_writePolling(uartHandle, (uint8_t*)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl));
    for (i = 0; i < sf->numRangeBins; i++){
        transmitBufferHWA32[i] = det32[i]; /* keep full for debug if needed */
        //transmitBufferHWA16[i] = det16[i];
        uint16_t  cnvtDta = u32_full_to_u16(det32[i]);
        //uint16_t  cnvtDta = (uint16_t)det32[i];//u32_full_to_u16(det32[i]);
        UART_writePolling(uartHandle, (uint8_t*)&cnvtDta, sizeof(uint16_t));
        //UART_writePolling(uartHandle, (uint8_t*)&transmitBufferHWA16, sizeof(uint16_t));
    }
    tlvIdx++;

    /* STATS + TEMP */
//    if (pGuiMonSel->statsInfo){
        UART_writePolling(uartHandle, (uint8_t*)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl));
        UART_writePolling(uartHandle, (uint8_t*)timingInfo, tl[tlvIdx].length);
        tlvIdx++;

        UART_writePolling(uartHandle, (uint8_t*)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl));
        UART_writePolling(uartHandle, (uint8_t*)&gMmwMCB.temperatureStats, tl[tlvIdx].length);
        tlvIdx++;
//    }

    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN -
                      (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN){
        UART_writePolling(uartHandle, padding, numPaddingBytes);
    }
}

/* ---------------- Pending dynamic cfg (unchanged) ---------------- */
static int32_t MmwDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx)
{
    gCycleStats.mmwaveExecCnt++;
    int32_t retVal=0;
    MmwDemo_DPC_ObjDet_CommonCfg *commonCfg=&gMmwMCB.dataPathObj.objDetCommonCfg;
    (void)subFrameIndx;
    if (0==0){
        if (commonCfg->isMeasureRxChannelBiasCfgPending==1){
            retVal = DPM_ioctl (gMmwMCB.dataPathObj.objDetDpmHandle, DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE
                                , &commonCfg->preStartCommonCfg.measureRxChannelBiasCfg, sizeof(DPC_ObjectDetection_MeasureRxChannelBiasCfg));
            if(retVal!=0){ return retVal; }
            commonCfg->isMeasureRxChannelBiasCfgPending=0;
        }
        if (commonCfg->isCompRxChannelBiasCfgPending==1){
            retVal = DPM_ioctl (gMmwMCB.dataPathObj.objDetDpmHandle, DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE
                                , &commonCfg->preStartCommonCfg.compRxChanCfg, sizeof(DPU_AoAProc_compRxChannelBiasCfg));
            if(retVal!=0){ return retVal; }
            commonCfg->isCompRxChannelBiasCfgPending=0;
        }
    }
    return retVal;
}

/* ---------------- DPM report + Execute task ---------------- */
static void MmwDemo_DPC_ObjectDetection_reportFxn(DPM_Report reportType, uint32_t instanceId, int32_t errCode, uint32_t arg0, uint32_t arg1)
{
    gCycleStats.objReportCnt++;
    if (errCode != 0)
        {
            System_printf("Error: DPM Report %d err:%d arg0:0x%x arg1:0x%x\n",
                           reportType, errCode, arg0, arg1);
            /* Do not assert here; let the specific cases below print more info. */
        }
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            if (arg0 == DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG)
            {
                DPC_ObjectDetection_PreStartCfg *cfg = (DPC_ObjectDetection_PreStartCfg*)arg1;
                DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage *memUsage = &cfg->memUsage; Memory_Stats stats; Memory_getStats(NULL,&stats);
                System_printf(" ========== Memory Stats ==========\n");
                System_printf("%20s %12s %12s %12s %12s\n"," ","Size","Used","Free","DPCUsed");
                System_printf("%20s %12d %12d %12d %12d\n","System Heap(TCM)", memUsage->SystemHeapTotal, memUsage->SystemHeapUsed, memUsage->SystemHeapTotal-memUsage->SystemHeapUsed, memUsage->SystemHeapDPCUsed);
                System_printf("%20s %12d %12d %12d\n","L3", (int)sizeof(gMmwL3), memUsage->L3RamUsage, (int)sizeof(gMmwL3)-memUsage->L3RamUsage);
                System_printf("%20s %12d %12d %12d\n","TCM", (int)sizeof(gDPC_ObjDetTCM), memUsage->CoreLocalRamUsage, (int)sizeof(gDPC_ObjDetTCM)-memUsage->CoreLocalRamUsage);
            }
            break;
        }
        case DPM_Report_DPC_STARTED:
            DebugP_log0("App: DPC Started\n");
            Semaphore_post(gMmwMCB.DPMstartSemHandle);
            break;
        case DPM_Report_NOTIFY_DPC_RESULT:
            MmwDemo_debugAssert(0);
            break;
        case DPM_Report_DPC_ASSERT:
                {
                    DPM_DPCAssert* a = (DPM_DPCAssert*)arg0;
                    CLI_write("ObjDet DPC ASSERT at %s:%d (arg0=0x%x arg1=0x%x)\n",
                              a->fileName, a->lineNum, a->arg0, a->arg1);
                    break;
                }
        case DPM_Report_DPC_STOPPED:
            DebugP_log0("App: DPC Stopped\n");
            Semaphore_post(gMmwMCB.DPMstopSemHandle);
            break;
        case DPM_Report_DPC_INFO:
            break;
        default:
            DebugP_assert(0);
            break;
    }
}

void MmwDemo_measurementResultOutput(DPU_AoAProc_compRxChannelBiasCfg *compRxChanCfg)
{
    gCycleStats.dpmExecCnt++;
    CLI_write ("compRangeBiasAndRxChanPhase %.7f %.5f %.5f\n", compRxChanCfg->rangeBias, (float)compRxChanCfg->rxChPhaseComp[0].real/32768., (float)compRxChanCfg->rxChPhaseComp[0].imag/32768.);
}

void MmwDemo_getTemperatureReport()
{
    gMmwMCB.temperatureStats.tempReportValid = rlRfGetTemperatureReport(RL_DEVICE_MAP_INTERNAL_BSS, (rlRfTempData_t*)&gMmwMCB.temperatureStats.temperatureReport);
}

MmwDemo_SubFrameStats *currStats, *prevStats;
static void MmwDemo_DPC_ObjectDetection_dpmTask(UArg arg0, UArg arg1)
{
    int32_t retVal;
    DPM_Buffer resultBuffer;
    DPC_ObjectDetection_ExecuteResultExportedInfo exportInfo;
    while(1)
    {
        retVal = DPM_execute (gMmwMCB.dataPathObj.objDetDpmHandle, &resultBuffer);
        if (retVal < 0){
            System_printf("Error: DPM execute failed [Error %d]\n", retVal);
            MmwDemo_debugAssert(0);
        }
        else if (resultBuffer.size[0] == sizeof(DPC_ObjectDetection_ExecuteResult))
        {
            uint32_t startTime, currentInterFrameProcessingEndTime;
            uint8_t currSubFrameIndx;
            uint8_t nextSubFrameIndx;
            DPC_ObjectDetection_ExecuteResult *result = (DPC_ObjectDetection_ExecuteResult *)resultBuffer.ptrBuffer[0];
            currSubFrameIndx = result->subFrameIdx;
            currStats = &gMmwMCB.subFrameStats[currSubFrameIndx];
            currentInterFrameProcessingEndTime = Cycleprofiler_getTimeStamp();

            currStats->outputStats.interFrameProcessingTime =
                (currentInterFrameProcessingEndTime - result->stats->interFrameStartTimeStamp)/R4F_CLOCK_MHZ;

            gRangeTiming->t_exec_leave       = NOW_TICKS();
            gRangeTiming->us_total_execute   = TICKS_TO_US(gRangeTiming->t_exec_leave - gRangeTiming->t_exec_enter);

            prevStats = &gMmwMCB.subFrameStats[0];
            prevStats->outputStats.interFrameProcessingMargin =
                (result->stats->frameStartTimeStamp - prevStats->interFrameProcessingEndTime)/R4F_CLOCK_MHZ
                - (result->stats->subFramePreparationCycles/R4F_CLOCK_MHZ + prevStats->subFramePreparationTime)
                - prevStats->pendingConfigProcTime;

            currStats->interFrameProcessingEndTime = currentInterFrameProcessingEndTime;
            /* UART timing (begin) */
            startTime = Cycleprofiler_getTimeStamp();
            gRangeTiming[result->subFrameIdx].t_uart_begin = startTime;
            MmwDemo_transmitProcessedOutput(gMmwMCB.loggingUartHandle, result, &currStats->outputStats);

            /* UART timing (end) + record */
            uint32_t t_uart_end = Cycleprofiler_getTimeStamp();
            gRangeTiming[result->subFrameIdx].t_uart_end = t_uart_end;
            gRangeTiming[result->subFrameIdx].us_uart_tx =
                    TICKS_TO_US(t_uart_end - gRangeTiming[result->subFrameIdx].t_uart_begin);//R4F_CLOCK_MHZ;

            currStats->outputStats.transmitOutputTime = gRangeTiming[result->subFrameIdx].us_uart_tx;
            float period_us = gRangeTiming[result->subFrameIdx].framePeriod_ms * 1000.0f;
            float duty_pct  = (period_us > 0.0f) ? (100.0f * gRangeTiming[result->subFrameIdx].us_active_frame_burst / period_us) : 0.0f;

            /* Pending config */
            startTime = Cycleprofiler_getTimeStamp();
            nextSubFrameIndx = 0;
            retVal = MmwDemo_processPendingDynamicCfgCommands(nextSubFrameIndx);
            if(retVal!=0){
                System_printf("Error: Pending DynCfg (nextSubFrame=%d) err=%d\n", nextSubFrameIndx, retVal);
                MmwDemo_debugAssert(0);
            }
            currStats->pendingConfigProcTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
            currStats->subFramePreparationTime = 0;

            exportInfo.subFrameIdx = currSubFrameIndx;
            retVal = DPM_ioctl (gMmwMCB.dataPathObj.objDetDpmHandle, DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED, &exportInfo, sizeof (DPC_ObjectDetection_ExecuteResultExportedInfo));
            if(retVal<0){
                System_printf("Error: RESULT_EXPORTED err=%d\n", retVal);
                MmwDemo_debugAssert(0);
            }
        }
    }
}

void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    gCycleStats.frameProcCnt++;
    MmwDemo_SubFrameStats *stats=&gMmwMCB.subFrameStats[subFrameIndx];
    Load_update();
    stats->outputStats.interFrameCPULoad = Load_getCPULoad();
}
void MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    gCycleStats.frameCount++;
    MmwDemo_SubFrameStats *stats=&gMmwMCB.subFrameStats[subFrameIndx];
    Load_update();
    stats->outputStats.activeFrameCPULoad = Load_getCPULoad();
}

/* ---------------- Platform init ---------------- */
void MmwDemo_platformInit(MmwDemo_platformCfg *config)
{
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINK13_PADAZ, SOC_XWR68XX_PINK13_PADAZ_GPIO_2);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR12_PADAP, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR12_PADAP, SOC_XWR68XX_PINR12_PADAP_QSPI_CLK);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP11_PADAQ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP11_PADAQ, SOC_XWR68XX_PINP11_PADAQ_QSPI_CSN);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR13_PADAL, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR13_PADAL, SOC_XWR68XX_PINR13_PADAL_QSPI_D0);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN12_PADAM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN12_PADAM, SOC_XWR68XX_PINN12_PADAM_QSPI_D1);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINR14_PADAN, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINR14_PADAN, SOC_XWR68XX_PINR14_PADAN_QSPI_D2);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP12_PADAO, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINP12_PADAO, SOC_XWR68XX_PINP12_PADAO_QSPI_D3);
    config->SensorStatusGPIO = SOC_XWR68XX_GPIO_2;
    config->sysClockFrequency = MSS_SYS_VCLK;
    config->loggingBaudRate = 921600;
    config->commandBaudRate = 115200;
    GPIO_setConfig(config->SensorStatusGPIO, GPIO_CFG_OUTPUT);
}

/* ---------------- Calib save/restore ---------------- */
/* (unchanged from your file) */
static int32_t MmwDemo_calibInit(void)
{
    int32_t retVal=0;
    rlVersion_t verArgs;
    memset(&verArgs,0,sizeof(verArgs));
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, &verArgs);
    if(retVal<0){
        System_printf("Error: get device version [Error %d]\n", retVal);
        return -1;
    }
    gMmwMCB.calibCfg.sizeOfCalibDataStorage = sizeof(MmwDemo_calibData);
    gMmwMCB.calibCfg.calibDataHdr.magic = MMWDEMO_CALIB_STORE_MAGIC;
    memcpy(&gMmwMCB.calibCfg.calibDataHdr.linkVer, &verArgs.mmWaveLink, sizeof(rlSwVersionParam_t));
    memcpy(&gMmwMCB.calibCfg.calibDataHdr.radarSSVer, &verArgs.rf, sizeof(rlFwVersionParam_t));
    if(gMmwMCB.calibCfg.sizeOfCalibDataStorage <= MMWDEMO_CALIB_FLASH_SIZE){
        gMmwMCB.calibCfg.calibDataHdr.hdrLen = sizeof(MmwDemo_calibDataHeader);
        gMmwMCB.calibCfg.calibDataHdr.dataLen = sizeof(MmwDemo_calibData) - sizeof(MmwDemo_calibDataHeader);
        memset(&gCalibDataStorage,0,sizeof(gCalibDataStorage));
        retVal = mmwDemo_flashInit();
    }
    else {
        System_printf("Error: Calibration data too large\n");
        retVal = -1;
    }
    return retVal;
}
static int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData)
{
    uint32_t flashOffset=gMmwMCB.calibCfg.flashOffset;
    int32_t retVal=0;
    memcpy(&ptrCalibrationData->calibDataHdr, ptrCalibDataHdr, sizeof(MmwDemo_calibDataHeader));
    retVal = mmwDemo_flashWrite(flashOffset, (uint32_t*)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if(retVal<0){
        System_printf("Error: flashing calibration data err[%d].\n", retVal);
    }
    return retVal;
}
static int32_t MmwDemo_calibRestore(MmwDemo_calibData *ptrCalibData)
{
    MmwDemo_calibDataHeader *pDataHdr=&(ptrCalibData->calibDataHdr);
    int32_t retVal=0;
    uint32_t flashOffset=gMmwMCB.calibCfg.flashOffset;
    if(mmwDemo_flashRead(flashOffset,(uint32_t*)pDataHdr,sizeof(MmwDemo_calibData))<0){
        System_printf("Error: reading calibration data from flash.\n");
        return -1;
    }
    if( (pDataHdr->magic != MMWDEMO_CALIB_STORE_MAGIC) || (pDataHdr->hdrLen != gMmwMCB.calibCfg.calibDataHdr.hdrLen) || (pDataHdr->dataLen != gMmwMCB.calibCfg.calibDataHdr.dataLen) ) {
        System_printf("Error: calibration data header validation failed.\n");
        retVal=-1;
    }
    else if(memcmp(&pDataHdr->linkVer, &gMmwMCB.calibCfg.calibDataHdr.linkVer, sizeof(rlSwVersionParam_t))!=0){
        System_printf("Error: mmwLink version mismatch when restoring calibration data.\n");
        retVal=-1;
    }
    else if(memcmp(&pDataHdr->radarSSVer, &gMmwMCB.calibCfg.calibDataHdr.radarSSVer, sizeof(rlFwVersionParam_t))!=0){
        System_printf("Error: RF FW version mismatch when restoring calibration data.\n");
        retVal=-1;
    }
    return retVal;
}


/* ---------------- Init Task ---------------- */
void MmwDemo_initTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    MMWave_InitCfg initCfg;
    UART_Params uartParams;
    Task_Params taskParams;
    Semaphore_Params semParams;
    DPM_InitCfg dpmInitCfg;
    DPC_ObjectDetection_InitParams objDetInitParams;

    System_printf("Debug: Launched the Initialization Task\n");
    UART_init();
    GPIO_init();
    Mailbox_init(MAILBOX_TYPE_MSS);
    MmwDemo_platformInit(&gMmwMCB.cfg.platformCfg);
    MmwDemo_dataPathInit(&gMmwMCB.dataPathObj);

    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate = gMmwMCB.cfg.platformCfg.commandBaudRate;
    uartParams.isPinMuxDone = 1;
    gMmwMCB.commandUartHandle = UART_open(0,&uartParams);
    if(gMmwMCB.commandUartHandle == NULL){
        MmwDemo_debugAssert(0);
        return;
    }
    System_printf("Debug: Command UART %p opened\n", gMmwMCB.commandUartHandle);

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate = gMmwMCB.cfg.platformCfg.loggingBaudRate;
    uartParams.isPinMuxDone = 1U;
    gMmwMCB.loggingUartHandle = UART_open(1,&uartParams);
    if(gMmwMCB.loggingUartHandle == NULL){
        MmwDemo_debugAssert(0);
        return;
    }
    System_printf("Debug: Logging UART %p opened\n", gMmwMCB.loggingUartHandle);
    Semaphore_Params_init(&semParams);
    semParams.mode=Semaphore_Mode_BINARY;
    gMmwMCB.DPMstartSemHandle = Semaphore_create(0,&semParams,NULL);
    gMmwMCB.DPMstopSemHandle = Semaphore_create(0,&semParams,NULL);

    memset(&initCfg,0,sizeof(initCfg));
    initCfg.domain = MMWave_Domain_MSS;
    initCfg.socHandle = gMmwMCB.socHandle;
    initCfg.eventFxn = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel = CRC_Channel_CH1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode = MMWave_ExecutionMode_ISOLATION;
    gMmwMCB.ctrlHandle = MMWave_init(&initCfg,&errCode);
    if(gMmwMCB.ctrlHandle == NULL){
        MmwDemo_debugAssert(0);
        return;
    }
    System_printf("Debug: mmWave Control Initialization OK\n");
    if(MMWave_sync(gMmwMCB.ctrlHandle,&errCode)<0){
        System_printf("Error: mmWave Control Sync failed [%d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    System_printf("Debug: mmWave Control Sync OK\n");

    MmwDemo_dataPathOpen(&gMmwMCB.dataPathObj);
    if((errCode = MmwDemo_LVDSStreamInit()) < 0){
        System_printf("Error: LVDS stream init failed [%d]\n", errCode);
        return;
    }
    gMmwMCB.cqSatMonCfg[0].profileIndx = (1);
    Cycleprofiler_init();

    Task_Params_init(&taskParams);
    taskParams.priority=MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY;
    taskParams.stackSize=3*1024;
    gMmwMCB.taskHandles.mmwaveCtrl = Task_create(MmwDemo_mmWaveCtrlTask,&taskParams,NULL);

    memset(&objDetInitParams,0,sizeof(objDetInitParams));
    objDetInitParams.hwaHandle = gMmwMCB.dataPathObj.hwaHandle;
    objDetInitParams.edmaHandle[0] = gMmwMCB.dataPathObj.edmaHandle[0];
    objDetInitParams.L3ramCfg.addr = (void*)&gMmwL3[0];
    objDetInitParams.L3ramCfg.size = sizeof(gMmwL3);
    objDetInitParams.CoreLocalRamCfg.addr = &gDPC_ObjDetTCM[0];
    objDetInitParams.CoreLocalRamCfg.size = sizeof(gDPC_ObjDetTCM);
    objDetInitParams.processCallBackCfg.processFrameBeginCallBackFxn = MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn;
    objDetInitParams.processCallBackCfg.processInterFrameBeginCallBackFxn = MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn;

    memset(&dpmInitCfg,0,sizeof(dpmInitCfg));
    dpmInitCfg.socHandle = gMmwMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg = &gDPC_ObjectDetectionCfg;
    dpmInitCfg.instanceId = 0xFEEDFEED;
    dpmInitCfg.domain = DPM_Domain_LOCALIZED;
    dpmInitCfg.reportFxn = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.arg = &objDetInitParams;
    dpmInitCfg.argSize = sizeof(DPC_ObjectDetection_InitParams);
    gMmwMCB.dataPathObj.objDetDpmHandle = DPM_init(&dpmInitCfg,&errCode);
    if(gMmwMCB.dataPathObj.objDetDpmHandle == NULL){
        System_printf("Error: DPM init failed [%d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 4*1024;
    gMmwMCB.taskHandles.objDetDpmTask = Task_create(MmwDemo_DPC_ObjectDetection_dpmTask,&taskParams,NULL);

    if(MmwDemo_calibInit()<0){
        System_printf("Error: Calib init failed\n");
        MmwDemo_debugAssert(0);
    }
    MmwDemo_CLIInit(MMWDEMO_CLI_TASK_PRIORITY);

    //autoStart();
}

void MmwDemo_sleep(void){
    asm(" WFI ");
}

int main (void)
{
    Task_Params taskParams;
    int32_t errCode;
    SOC_Handle socHandle;
    SOC_Cfg socCfg;
    ESM_init(0U);
    memset(&socCfg,0,sizeof(SOC_Cfg));
    socCfg.clockCfg=SOC_SysClock_INIT;
    socCfg.dssCfg=SOC_DSSCfg_HALT;
    socCfg.mpuCfg=SOC_MPUCfg_CONFIG;
    socHandle = SOC_init(&socCfg,&errCode);
    if(socHandle==NULL){
        MmwDemo_debugAssert(0);
        return -1;
    }
    if(SOC_waitBSSPowerUp(socHandle,&errCode) < 0){
        System_printf("Debug: SOC_waitBSSPowerUp failed [%d]\n", errCode);
        return 0;
    }
    if (SOC_isSecureDevice(socHandle,&errCode)) {
        SOC_controlSecureFirewall(socHandle, (uint32_t)(SOC_SECURE_FIREWALL_JTAG | SOC_SECURE_FIREWALL_LOGGER), SOC_SECURE_FIREWALL_DISABLE, &errCode);
    }
    memset(&gMmwMCB,0,sizeof(MmwDemo_MCB));
    gMmwMCB.socHandle=socHandle;
    System_printf("**********************************************\n");
    System_printf("Debug: Launching Range-Only Demo\n");
    System_printf("**********************************************\n");
    Task_Params_init(&taskParams);
    gMmwMCB.taskHandles.initTask = Task_create(MmwDemo_initTask,&taskParams,NULL);

    BIOS_start();

    return 0;
}

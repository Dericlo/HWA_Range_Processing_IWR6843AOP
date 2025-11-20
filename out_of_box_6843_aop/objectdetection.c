// =============================
// FILE: objectdetection_range_only.c
// =============================
//
// Object Detection DPC (Range-only)
// - Uses HWA RangeProc only
// - Builds a 32-bit magnitude^2 range profile from radar cube (0-Doppler)
// - Fixed static buffer: 128 bins per subframe (assert if config exceeds)
// - Added detailed timing (window gen, HWA proc, magnitude build, UART, total, ramp proxy)
//

#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* Build switches */
#ifndef DPC_OBJDET_RANGE_ONLY
#define DPC_OBJDET_RANGE_ONLY 1
#endif
#define DBG_DPC_OBJDET

/* TI / mmWave SDK includes */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/control/dpm/dpm.h>
#include <ti/board/antenna_geometry.h>

#if defined(USE_2D_AOA_DPU)
#include <ti/datapath/dpc/dpu/aoa2dproc/aoa2dprochwa.h>
#else
#include <ti/datapath/dpc/dpu/aoaproc/aoaprochwa.h>
#endif

#ifdef SUBSYS_DSS
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop
#endif

/* App provides resource mapping */
#include APP_RESOURCE_FILE

/* ObjDet HWA internal/public API */
#include <ti/datapath/dpc/objectdetection/objdethwa/include/objectdetectioninternal.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

/* ============================
 * Config & local helpers
 * ============================ */

#ifdef DBG_DPC_OBJDET
ObjDetObj *gObjDetObj;
#endif

/* Alignment used for radarCube L3 buffer */
#define DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT   (sizeof(int16_t))

/* HWA limits */
#define DPC_OBJDET_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES  SOC_HWA_WINDOW_RAM_SIZE_IN_SAMPLES
#define DPC_OBJDET_HWA_NUM_PARAM_SETS                  SOC_HWA_NUM_PARAM_SETS

/* Range window */
#define DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE              MATHUTILS_WIN_BLACKMAN
#define DPC_OBJDET_QFORMAT_RANGE_FFT                   17

/* Your fixed FFT size for range profile */
#define MAX_RANGE_BINS                                 128

#ifndef R4F_CLOCK_MHZ
#define R4F_CLOCK_MHZ 200U
#endif
#define NOW_TICKS()     (Cycleprofiler_getTimeStamp())
#define TICKS_TO_US(dt) ((float)((float)(dt) / R4F_CLOCK_MHZ))

/* Timing struct (shared with main) */
typedef struct {
    uint32_t t_frame_isr, t_exec_enter, t_range_begin, t_range_end, t_mag_begin, t_mag_end, t_uart_begin, t_uart_end, t_exec_leave, t_win_begin, t_win_end;
    float us_window_gen, us_range_proc, us_mag_build, us_uart_tx, us_total_execute, us_ramp_from_frame_to_range;
    /* NEW: RF "active frame (burst)" time and period */
    float us_active_frame_burst;
    float framePeriod_ms;
    float us_chirp_interval;
} RangeOnlyTiming;

typedef struct {
    uint32_t cnt_frame, cnt_exec, cnt_range, cnt_mag, cnt_uart, cnt_win, cnt_obj;
} ProcCnt;

volatile ProcCnt mProcCnt;
volatile RangeOnlyTiming gRangeTiming[1/*RL_MAX_SUBFRAMES*/];
static uint8_t gTimingConfigSubframe = 0; /* subframe for window timing */
static uint32_t s_prevFrameStartTicks[1/*RL_MAX_SUBFRAMES*/] = {0};  // match your current single-subframe build

/* Legacy compatibility struct you requested earlier */
struct timeMeasure{
    float winTime, fftTime, rampTime, adcStartTime, cfarTime;
};
static struct timeMeasure gTimeMeasure[1/*RL_MAX_SUBFRAMES*/];

/* ============================
 * DPM registration
 * ============================ */

static DPM_DPCHandle DPC_ObjectDetection_init   (DPM_Handle dpmHandle, DPM_InitCfg* ptrInitCfg, int32_t* errCode);
static int32_t       DPC_ObjectDetection_start  (DPM_DPCHandle handle);
static int32_t       DPC_ObjectDetection_execute(DPM_DPCHandle handle, DPM_Buffer* ptrResult);
static int32_t       DPC_ObjectDetection_ioctl  (DPM_DPCHandle handle, uint32_t cmd, void* arg, uint32_t argLen);
static int32_t       DPC_ObjectDetection_stop   (DPM_DPCHandle handle);
static int32_t       DPC_ObjectDetection_deinit (DPM_DPCHandle handle);
static void          DPC_ObjectDetection_frameStart (DPM_DPCHandle handle);

DPM_ProcChainCfg gDPC_ObjectDetectionCfg =
{
    DPC_ObjectDetection_init,
    DPC_ObjectDetection_start,
    DPC_ObjectDetection_execute,
    DPC_ObjectDetection_ioctl,
    DPC_ObjectDetection_stop,
    DPC_ObjectDetection_deinit,
    NULL,                          /* inject data */
    NULL,                          /* chirp available */
    DPC_ObjectDetection_frameStart
};

/* ============================
 * Debug/assert bridge to DPM
 * ============================ */

void _DPC_Objdet_Assert(DPM_Handle handle, int32_t expression, const char *file, int32_t line)
{
    DPM_DPCAssert fault;
    if (!expression)
    {
        fault.lineNum = (uint32_t)line;
        fault.arg0    = 0U;
        fault.arg1    = 0U;
        strncpy (fault.fileName, file, (DPM_MAX_FILE_NAME_LEN-1));
        (void)DPM_ioctl (handle, DPM_CMD_DPC_ASSERT, (void*)&fault, sizeof(DPM_DPCAssert));
    }
}

/* ============================
 * Simple mem-pool helpers
 * ============================ */
static void DPC_ObjDet_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr   = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr= pool->currAddr;
}
static void *DPC_ObjDet_MemPoolGet(MemPoolObj *pool) { return (void*)pool->currAddr; }
static void DPC_ObjDet_MemPoolSet(MemPoolObj *pool, void *addr)
{
    pool->currAddr   = (uintptr_t)addr;
    pool->maxCurrAddr= MAX(pool->maxCurrAddr, pool->currAddr);
}
static uint32_t DPC_ObjDet_MemPoolGetMaxUsage(MemPoolObj *pool)
{
    return (uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr);
}
static void *DPC_ObjDet_MemPoolAlloc(MemPoolObj *pool, uint32_t size, uint8_t align)
{
    void *ret   = NULL;
    uintptr_t a = MEM_ALIGN(pool->currAddr, align);
    if ((a + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        ret = (void*)a;
        pool->currAddr = a + size;
        pool->maxCurrAddr = MAX(pool->maxCurrAddr, pool->currAddr);
    }
    return ret;
}

/* ============================
 * Range window generation
 * ============================ */
static uint32_t DPC_ObjDet_GetRangeWinGenLen(DPU_RangeProcHWA_Config *cfg)
{
    uint16_t N = cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples;
#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    return (N + 1U)/2U;
#else
    return N;
#endif
}
/* at top */
#undef  DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE  MATHUTILS_WIN_RECT

static void DPC_ObjDet_GenRangeWindow(DPU_RangeProcHWA_Config *cfg)
{
    uint32_t t0 = NOW_TICKS();
    mathUtils_genWindow((uint32_t *)cfg->staticCfg.window,
                        cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples,
                        DPC_ObjDet_GetRangeWinGenLen(cfg),
                        DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                        DPC_OBJDET_QFORMAT_RANGE_FFT);
    uint32_t t1 = NOW_TICKS();
    gRangeTiming[0].us_window_gen = TICKS_TO_US(t1 - t0);
    mProcCnt.cnt_win++;
}

/* ============================
 * Static range-profile buffer
 * ============================ */

/* ============================
 * Frame start ISR hook
 * ============================ */
static void DPC_ObjectDetection_frameStart (DPM_DPCHandle handle)
{
    ObjDetObj *obj = (ObjDetObj *) handle;
    mProcCnt.cnt_frame++;
    obj->stats.frameStartTimeStamp = Cycleprofiler_getTimeStamp();
    gRangeTiming[obj->subFrameIndx].t_frame_isr = obj->stats.frameStartTimeStamp; /* timing */
    uint32_t prev = s_prevFrameStartTicks[obj->subFrameIndx];
    if (prev != 0U)
        {
            gRangeTiming[obj->subFrameIndx].framePeriod_ms =
                TICKS_TO_US(obj->stats.frameStartTimeStamp - prev);
        }
    s_prevFrameStartTicks[obj->subFrameIndx] = obj->stats.frameStartTimeStamp;
    DebugP_log2("ObjDet DPC: Frame Start, frameIndx=%d, subFrame=%d\n",
                obj->stats.frameStartIntCounter, obj->subFrameIndx);

    obj->interSubFrameProcToken++;
    if (obj->subFrameIndx == 0)
    {
        obj->stats.frameStartIntCounter++;
    }

    DebugP_assert (DPM_notifyExecute (obj->dpmHandle, handle, true) == 0);
}

static int32_t DPC_ObjDet_rangeConfig(
    DPU_RangeProcHWA_Handle           dpuHandle,
    DPC_ObjectDetection_StaticCfg    *staticCfg,
    DPC_ObjectDetection_DynCfg       *dynCfg,
    EDMA_Handle                       edmaHandle,
    DPIF_RadarCube                   *radarCube,
    MemPoolObj                       *CoreLocalRamObj,
    uint32_t                         *hwaWindowOffset,
    uint32_t                         *CoreLocalScratchUsage,
    DPU_RangeProcHWA_Config          *cfgSave)
{
    int32_t retVal = 0;
    DPU_RangeProcHWA_Config rangeCfg;
    memset(&rangeCfg, 0, sizeof(rangeCfg));

    DPU_RangeProcHWA_HW_Resources     *hwRes   = &rangeCfg.hwRes;
    DPU_RangeProcHWA_EDMAInputConfig  *edmaIn  = &hwRes->edmaInCfg;
    DPU_RangeProcHWA_EDMAOutputConfig *edmaOut = &hwRes->edmaOutCfg;
    DPU_RangeProcHWA_HwaConfig        *hwaCfg  = &hwRes->hwaCfg;

    /* Dynamic & static copy */
    rangeCfg.dynCfg.calibDcRangeSigCfg = &dynCfg->calibDcRangeSigCfg;
    //rangeCfg.staticCfg                  = *staticCfg;
    rangeCfg.staticCfg.ADCBufData         = staticCfg->ADCBufData;
    rangeCfg.staticCfg.numChirpsPerFrame  = staticCfg->numChirpsPerFrame;
    rangeCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    rangeCfg.staticCfg.numTxAntennas      = staticCfg->numTxAntennas;
    rangeCfg.staticCfg.numVirtualAntennas = staticCfg->numVirtualAntennas;
    rangeCfg.staticCfg.resetDcRangeSigMeanBuffer = 1;
    rangeCfg.staticCfg.rangeFFTtuning.fftOutputDivShift             = staticCfg->rangeFFTtuning.fftOutputDivShift;
    rangeCfg.staticCfg.rangeFFTtuning.numLastButterflyStagesToScale = staticCfg->rangeFFTtuning.numLastButterflyStagesToScale;
    /* Radar cube in L3 */
    rangeCfg.hwRes.radarCube = *radarCube;

    /* ===== Window buffer in local TCM ===== */
    {
        uint16_t N = rangeCfg.staticCfg.ADCBufData.dataProperty.numAdcSamples;
#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
        uint32_t winGenLen = (N + 1U)/2U;
#else
        uint32_t winGenLen = N;
#endif
        rangeCfg.staticCfg.windowSize = winGenLen * sizeof(uint32_t);
        int32_t *winBuf = (int32_t *)DPC_ObjDet_MemPoolAlloc(
                CoreLocalRamObj, rangeCfg.staticCfg.windowSize, sizeof(uint32_t));
        if (winBuf == NULL)
            return DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;

        rangeCfg.staticCfg.window = winBuf;

        /* Generate window now (Blackman by default) */
        mathUtils_genWindow((uint32_t *)rangeCfg.staticCfg.window,
                            N, winGenLen,
                            DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_RANGE_FFT);
        *CoreLocalScratchUsage = rangeCfg.staticCfg.windowSize;
    }

    /* ===== DC removal buffer ===== */
    hwRes->dcRangeSigMeanSize = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE *
                                rangeCfg.staticCfg.numTxAntennas *
                                rangeCfg.staticCfg.ADCBufData.dataProperty.numRxAntennas *
                                sizeof(cmplx32ImRe_t);
    hwRes->dcRangeSigMean = (cmplx32ImRe_t *) MemoryP_ctrlAlloc (
                                hwRes->dcRangeSigMeanSize, 0);

    /* ===== EDMA In (CEPs) ===== */
    hwRes->edmaHandle = edmaHandle;
    edmaIn->dataIn.channel                = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    edmaIn->dataIn.channelShadow          = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW;
    edmaIn->dataIn.eventQueue             = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE;
    edmaIn->dataInSignature.channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH;
    edmaIn->dataInSignature.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW;
    edmaIn->dataInSignature.eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE;

    /* ===== EDMA Out (FMT1: radar cube) ===== */
    edmaOut->u.fmt1.dataOutPing.channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_CH;
    edmaOut->u.fmt1.dataOutPing.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_SHADOW;
    edmaOut->u.fmt1.dataOutPing.eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_EVENT_QUE;

    edmaOut->u.fmt1.dataOutPong.channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_CH;
    edmaOut->u.fmt1.dataOutPong.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_SHADOW;
    edmaOut->u.fmt1.dataOutPong.eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_EVENT_QUE;

    edmaOut->dataOutSignature.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_CH;
    edmaOut->dataOutSignature.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_SHADOW;
    edmaOut->dataOutSignature.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_EVENT_QUE;

    /* === SPECIAL CASE: single chirp per frame ===
       Some SDK builds assume ping+pong. Mirror ping into pong so driver stays happy. */
    if (rangeCfg.staticCfg.numChirpsPerFrame == 1U)
    {
        edmaOut->u.fmt1.dataOutPong.channel       = edmaOut->u.fmt1.dataOutPing.channel;
        edmaOut->u.fmt1.dataOutPong.channelShadow = edmaOut->u.fmt1.dataOutPing.channelShadow;
        edmaOut->u.fmt1.dataOutPong.eventQueue    = edmaOut->u.fmt1.dataOutPing.eventQueue;
    }

    /* ===== HWA ===== */
    hwaCfg->dataInputMode   = DPU_RangeProcHWA_InputMode_ISOLATED;
#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    hwaCfg->hwaWinSym       = HWA_FFT_WINDOW_SYMMETRIC;
#else
    hwaCfg->hwaWinSym       = HWA_FFT_WINDOW_NONSYMMETRIC;
#endif
    hwaCfg->hwaWinRamOffset = (uint16_t)(*hwaWindowOffset);

    /* Make sure window fits */
    {
        uint16_t N = rangeCfg.staticCfg.ADCBufData.dataProperty.numAdcSamples;
#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
        uint32_t winGenLen = (N + 1U)/2U;
#else
        uint32_t winGenLen = N;
#endif
        if ((hwaCfg->hwaWinRamOffset + winGenLen) >
            DPC_OBJDET_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES)
        {
            return DPC_OBJECTDETECTION_ENOMEM_HWA_WINDOW_RAM;
        }
        *hwaWindowOffset += winGenLen;
    }

    hwaCfg->numParamSet      = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    hwaCfg->paramSetStartIdx = DPC_OBJDET_DPU_RANGEPROC_PARAMSET_START_IDX;

    /* ===== Apply Range HWA config ===== */
    retVal = DPU_RangeProcHWA_config(dpuHandle, &rangeCfg);
    if (retVal != 0)
        return retVal;

    /* Save for quick reconfig (do not reset DC buffer next time) */
    rangeCfg.staticCfg.resetDcRangeSigMeanBuffer = 0;
    *cfgSave = rangeCfg;

    return 0;
}


/* Place near top of the file or next to other defines */
#define DPC_OBJDET_TCM_RADARCUBE_THRESHOLD   (16U * 1024U)

static int32_t DPC_ObjDet_preStartConfig(SubFrameObj *obj,
                                         DPC_ObjectDetection_PreStartCommonCfg *commonCfg,
                                         DPC_ObjectDetection_StaticCfg *staticCfg,
                                         DPC_ObjectDetection_DynCfg    *dynCfg,
                                         EDMA_Handle                   edmaHandle[EDMA_NUM_CC],
                                         MemPoolObj                    *L3ramObj,
                                         MemPoolObj                    *CoreLocalRamObj,
                                         uint32_t                      hwaMemBankAddr[4],
                                         uint16_t                      hwaMemBankSize,
                                         uint32_t                      *L3RamUsage,
                                         uint32_t                      *CoreLocalRamUsage)
{
    (void)commonCfg; (void)hwaMemBankAddr; (void)hwaMemBankSize;

    int32_t retVal = 0;
    DPIF_RadarCube radarCube;
    uint32_t hwaWindowOffset = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET;
    uint32_t rangeScratchUsage;
    void *coreLocalStart;

    obj->staticCfg = *staticCfg;
    obj->dynCfg    = *dynCfg;

    obj->log2NumDopplerBins = mathUtils_floorLog2(staticCfg->numDopplerBins);

    DPC_ObjDet_MemPoolReset(L3ramObj);
    DPC_ObjDet_MemPoolReset(CoreLocalRamObj);

    /* ---- Prefer TCM for small radarCube ---- */
    uint32_t cubeBytes = staticCfg->numRangeBins *
                         staticCfg->numDopplerChirps *
                         staticCfg->numVirtualAntennas *
                         sizeof(cmplx16ReIm_t);

    void *cubeMem = NULL;
    if (cubeBytes <= DPC_OBJDET_TCM_RADARCUBE_THRESHOLD) {
        cubeMem = DPC_ObjDet_MemPoolAlloc(CoreLocalRamObj,
                                          cubeBytes,
                                          DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);
    }
    if (cubeMem == NULL) {
        cubeMem = DPC_ObjDet_MemPoolAlloc(L3ramObj,
                                          cubeBytes,
                                          DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);
        if (cubeMem == NULL)
            return DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
    }

    radarCube.data     = cubeMem;
    radarCube.dataSize = cubeBytes;
    radarCube.datafmt  = DPIF_RADARCUBE_FORMAT_1;

    /* Keep scratch start so we can rewind after config */
    coreLocalStart = DPC_ObjDet_MemPoolGet(CoreLocalRamObj);

    /* Configure Range DPU */
    retVal = DPC_ObjDet_rangeConfig(obj->dpuRangeObj,
                                    &obj->staticCfg,
                                    &obj->dynCfg,
                                    edmaHandle[DPC_OBJDET_DPU_RANGEPROC_EDMA_INST_ID],
                                    &radarCube,
                                    CoreLocalRamObj,
                                    &hwaWindowOffset,
                                    &rangeScratchUsage,
                                    &obj->dpuCfg.rangeCfg);
    if (retVal != 0)
        return retVal;

    /* Rewind scratch (only window persists) */
    DPC_ObjDet_MemPoolSet(CoreLocalRamObj, coreLocalStart);

    *CoreLocalRamUsage = DPC_ObjDet_MemPoolGetMaxUsage(CoreLocalRamObj);
    *L3RamUsage        = DPC_ObjDet_MemPoolGetMaxUsage(L3ramObj);

    return 0;
}


/* ============================
 * DPM: init / deinit
 * ============================ */
static DPM_DPCHandle DPC_ObjectDetection_init
(
    DPM_Handle   dpmHandle,
    DPM_InitCfg* ptrInitCfg,
    int32_t*     errCode
)
{
    *errCode = 0;

    if ((ptrInitCfg == NULL) || (ptrInitCfg->arg == NULL))
    {
        *errCode = DPC_OBJECTDETECTION_EINVAL;
        return NULL;
    }
    if (ptrInitCfg->argSize != sizeof(DPC_ObjectDetection_InitParams))
    {
        *errCode = DPC_OBJECTDETECTION_EINVAL__INIT_CFG_ARGSIZE;
        return NULL;
    }

    DPC_ObjectDetection_InitParams *initParams =
        (DPC_ObjectDetection_InitParams *)ptrInitCfg->arg;

    ObjDetObj *obj = (ObjDetObj*)MemoryP_ctrlAlloc(sizeof(ObjDetObj), 0);
    if (obj == NULL)
    {
        *errCode = DPC_OBJECTDETECTION_ENOMEM;
        return NULL;
    }
    memset(obj, 0, sizeof(*obj));
#ifdef DBG_DPC_OBJDET
    gObjDetObj = obj;
#endif

    memcpy(&obj->dpmInitCfg, ptrInitCfg, sizeof(DPM_InitCfg));
    obj->dpmHandle        = dpmHandle;
    obj->socHandle        = ptrInitCfg->socHandle;
    obj->L3RamObj.cfg     = initParams->L3ramCfg;
    obj->CoreLocalRamObj.cfg = initParams->CoreLocalRamCfg;
    int i;
    for (i = 0; i < EDMA_NUM_CC; i++)
        obj->edmaHandle[i] = initParams->edmaHandle[i];
    obj->processCallBackCfg = initParams->processCallBackCfg;

    /* HWA mem info */
    HWA_MemInfo hwaInfo;
    *errCode = HWA_getHWAMemInfo(initParams->hwaHandle, &hwaInfo);
    if (*errCode != 0)
    {
        MemoryP_ctrlFree(obj, sizeof(ObjDetObj));
        return NULL;
    }
    //int i;
    obj->hwaMemBankSize = hwaInfo.bankSize;
    for (i = 0; i < hwaInfo.numBanks; i++)
        obj->hwaMemBankAddr[i] = hwaInfo.baseAddress + i * hwaInfo.bankSize;

    /* Init Range DPU per subframe */
    DPU_RangeProcHWA_InitParams rangeInitParams;
    rangeInitParams.hwaHandle = initParams->hwaHandle;

    for (i = 0; i < 1;/*RL_MAX_SUBFRAMES;*/ i++)
    {
        obj->subFrameObj[i].dpuRangeObj = DPU_RangeProcHWA_init(&rangeInitParams, errCode);
        if (*errCode != 0)
        {
            int j;
            for (j = 0; j < i; j++)
                (void)DPU_RangeProcHWA_deinit(obj->subFrameObj[j].dpuRangeObj);
            MemoryP_ctrlFree(obj, sizeof(ObjDetObj));
            return NULL;
        }
        /* clear timing per subframe */
        memset((void*)&gRangeTiming[i], 0, sizeof(gRangeTiming[i]));
    }

    DebugP_log1("ObjDet DPC: init OK, obj=%u\n", (uint32_t)obj);
    return (DPM_DPCHandle)obj;
}

static int32_t DPC_ObjectDetection_deinit (DPM_DPCHandle handle)
{
    if (handle == NULL)
        return DPC_OBJECTDETECTION_EINVAL;

    ObjDetObj *obj = (ObjDetObj*)handle;
    int32_t ret = 0;
    int i;
    for (i = 0; i < 1;/*RL_MAX_SUBFRAMES;*/ i++)
    {
        int32_t r = DPU_RangeProcHWA_deinit(obj->subFrameObj[i].dpuRangeObj);
        if (r != 0) ret = r;
        if (obj->subFrameObj[i].dpuCfg.rangeCfg.hwRes.dcRangeSigMean)
        {
            MemoryP_ctrlFree(obj->subFrameObj[i].dpuCfg.rangeCfg.hwRes.dcRangeSigMean,
                             obj->subFrameObj[i].dpuCfg.rangeCfg.hwRes.dcRangeSigMeanSize);
            obj->subFrameObj[i].dpuCfg.rangeCfg.hwRes.dcRangeSigMean = NULL;
        }
    }
    MemoryP_ctrlFree(obj, sizeof(ObjDetObj));
    return ret;
}

/* ============================
 * DPM: start / stop
 * ============================ */
static int32_t DPC_ObjDet_reconfigSubFrame(ObjDetObj *obj, uint8_t subFrameIndx)
{
    SubFrameObj *sf = &obj->subFrameObj[subFrameIndx];
    DPC_ObjDet_GenRangeWindow(&sf->dpuCfg.rangeCfg);
    return DPU_RangeProcHWA_config(sf->dpuRangeObj, &sf->dpuCfg.rangeCfg);
}

static int32_t DPC_ObjectDetection_start (DPM_DPCHandle handle)
{
    ObjDetObj *obj = (ObjDetObj*)handle;
    DebugP_assert(obj != NULL);
    mProcCnt.cnt_obj++;
    obj->stats.frameStartIntCounter = 0;
    obj->isCommonCfgReceived = false;

    DebugP_assert(obj->subFrameIndx == 0);

    int32_t ret = DPC_ObjDet_reconfigSubFrame(obj, obj->subFrameIndx);
    if (ret != 0) return ret;

    SubFrameObj *sf = &obj->subFrameObj[obj->subFrameIndx];
    ret = DPU_RangeProcHWA_control(sf->dpuRangeObj, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    if (ret != 0) return ret;

    DebugP_log0("ObjDet DPC: Start done\n");
    return 0;
}

static int32_t DPC_ObjectDetection_stop (DPM_DPCHandle handle)
{
    ObjDetObj *obj = (ObjDetObj*)handle;
    DebugP_assert(obj != NULL);
    DebugP_log0("ObjDet DPC: Stop done\n");
    return 0;
}

/* ============================
 * DPM: execute
 * ============================ */
SubFrameObj *subFrameExecute;
DPU_RangeProcHWA_OutParams outRange;
static int32_t DPC_ObjectDetection_execute(DPM_DPCHandle handle, DPM_Buffer* ptrResult)
{

    mProcCnt.cnt_exec++;
    ObjDetObj   *objexecute = (ObjDetObj*)handle;
    DebugP_assert(objexecute && ptrResult);
    objexecute->stats.interFrameStartTimeStamp = NOW_TICKS();
    RangeOnlyTiming *tm = &gRangeTiming[objexecute->subFrameIndx];
    tm->t_exec_enter = NOW_TICKS();


    subFrameExecute = &objexecute->subFrameObj[objexecute->subFrameIndx];

    if (objexecute->processCallBackCfg.processFrameBeginCallBackFxn)
        objexecute->processCallBackCfg.processFrameBeginCallBackFxn(objexecute->subFrameIndx);

    /* HWA range proc */
//    tm->t_range_begin = NOW_TICKS();
    int32_t ret = DPU_RangeProcHWA_process(subFrameExecute->dpuRangeObj, &outRange);
//    tm->t_range_end = NOW_TICKS();
    if (ret != 0) return ret;
    DebugP_assert(outRange.endOfChirp == true);

//    tm->us_range_proc = TICKS_TO_US(tm->t_range_end - tm->t_range_begin);
//    tm->us_ramp_from_frame_to_range =
//        TICKS_TO_US(tm->t_range_begin - gRangeTiming[objexecute->subFrameIndx].t_frame_isr);

    if (objexecute->processCallBackCfg.processInterFrameBeginCallBackFxn)
        objexecute->processCallBackCfg.processInterFrameBeginCallBackFxn(objexecute->subFrameIndx);

    /* Magnitude build (zero-Doppler, ant0) */
    {
        uint32_t numRangeBins   = subFrameExecute->staticCfg.numRangeBins;
        DebugP_assert(numRangeBins <= MAX_RANGE_BINS);

        //cmplx16ReIm_t *cube = (cmplx16ReIm_t *)subFrameExecute->dpuCfg.rangeCfg.hwRes.radarCube.data;
//        uint32_t *cube = (uint32_t *)subFrameExecute->dpuCfg.rangeCfg.hwRes.radarCube.data;
        uint16_t *cube = (uint16_t *)subFrameExecute->dpuCfg.rangeCfg.hwRes.radarCube.data;

//        uint32_t *dst       = &gRangeProfileBuf[objexecute->subFrameIndx][0];
//        uint32_t strideDopp = 1;//subFrameExecute->staticCfg.numDopplerChirps;
//        uint32_t i;
//        for (i = 0; i < numRangeBins; i++)
//        {
//            cmplx16ReIm_t v = cube[i * strideDopp]; /*[i * strideDopp] doppler=0, ant=0 */
//            int32_t re = (int32_t)v.real;
//            int32_t im = (int32_t)v.imag;
//            dst[i] = (uint32_t)(re*re + im*im);
//        }
        //for (i =0; i <numRangeBins; i++)gRangeProfileBuf[objexecute->subFrameIndx][i] = cube[i];
        objexecute->executeResult.detMatrix.data     = (uint8_t *)cube;
        //objexecute->executeResult.detMatrix.dataSize = numRangeBins * sizeof(uint32_t);
        objexecute->executeResult.detMatrix.dataSize = numRangeBins * sizeof(uint16_t);
    }

    /* Minimal result */
    objexecute->executeResult.numObjOut  = 0;
    objexecute->executeResult.subFrameIdx= objexecute->subFrameIndx;

    objexecute->stats.interChirpProcessingMargin = 0;
    objexecute->stats.interFrameEndTimeStamp     = objexecute->stats.interFrameStartTimeStamp;
    objexecute->executeResult.stats              = &objexecute->stats;

    ptrResult->ptrBuffer[0] = (uint8_t *)&objexecute->executeResult;
    ptrResult->size[0]      = sizeof(DPC_ObjectDetection_ExecuteResult);
    int i;
    for (i = 1; i < DPM_MAX_BUFFER; i++) { ptrResult->ptrBuffer[i]=NULL; ptrResult->size[i]=0; }

    //tm->t_exec_leave       = NOW_TICKS();
    //tm->us_total_execute   = TICKS_TO_US(tm->t_exec_leave - tm->t_exec_enter);

    /* legacy struct fill */
//    gTimeMeasure[objexecute->subFrameIndx].winTime      = gRangeTiming[objexecute->subFrameIndx].us_window_gen;
//    gTimeMeasure[objexecute->subFrameIndx].fftTime      = tm->us_range_proc;
//    gTimeMeasure[objexecute->subFrameIndx].rampTime     = tm->us_ramp_from_frame_to_range;
//    gTimeMeasure[objexecute->subFrameIndx].adcStartTime = tm->us_ramp_from_frame_to_range; /* proxy */
//    gTimeMeasure[objexecute->subFrameIndx].cfarTime     = 0;

//    DebugP_log4("TIMING(us): win=%u, range=%u, mag=%u, exec=%u\n",
//                gTimeMeasure[objexecute->subFrameIndx].winTime,
//                tm->us_range_proc, tm->us_mag_build, tm->us_total_execute);

    return 0;
}


/* ============================
 * DPM: ioctl
 * ============================ */
static int32_t DPC_ObjectDetection_ioctl(DPM_DPCHandle handle,
                                         uint32_t cmd, void* arg, uint32_t argLen)
{
    ObjDetObj *obj = (ObjDetObj*)handle;
    DebugP_assert(obj != NULL);
    if (cmd == DPC_OBJDET_IOCTL__TRIGGER_FRAME)
    {
        DPC_ObjectDetection_frameStart(handle);
        return 0;
    }
    else if (cmd == DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG)
    {
        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_PreStartCommonCfg));
        DPC_ObjectDetection_PreStartCommonCfg *cfg =
            (DPC_ObjectDetection_PreStartCommonCfg*)arg;
        int i;
        for ( i = 0; i < obj->commonCfg.numSubFrames; i++)
        {
            SubFrameObj *sf = &obj->subFrameObj[i];
            if (sf->dpuCfg.rangeCfg.hwRes.dcRangeSigMean)
            {
                MemoryP_ctrlFree(sf->dpuCfg.rangeCfg.hwRes.dcRangeSigMean,
                                 sf->dpuCfg.rangeCfg.hwRes.dcRangeSigMeanSize);
                sf->dpuCfg.rangeCfg.hwRes.dcRangeSigMean = NULL;
            }
        }
        obj->commonCfg = *cfg;
        obj->isCommonCfgReceived = true;

        DebugP_log0("ObjDet DPC: Pre-start Common Config processed\n");
        return 0;
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED)
    {
        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_ExecuteResultExportedInfo));
        DPC_ObjectDetection_ExecuteResultExportedInfo *info =
            (DPC_ObjectDetection_ExecuteResultExportedInfo *)arg;

        DebugP_assert(info->subFrameIdx == obj->subFrameIndx);

        if (obj->commonCfg.numSubFrames > 1)
        {
            obj->subFrameIndx++;
            if (obj->subFrameIndx == obj->commonCfg.numSubFrames)
                obj->subFrameIndx = 0;

            (void)DPC_ObjDet_reconfigSubFrame(obj, obj->subFrameIndx);
        }

        SubFrameObj *sf = &obj->subFrameObj[obj->subFrameIndx];
        (void)DPU_RangeProcHWA_control(sf->dpuRangeObj, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);

        obj->stats.subFramePreparationCycles = 0;
        obj->interSubFrameProcToken--;

        DebugP_log0("ObjDet DPC: RESULT_EXPORTED -> next trigger\n");
        return 0;
    }
    else
    {
        DebugP_assert(arg != NULL);
        uint8_t subFrameNum = *(uint8_t*)arg;
        SubFrameObj *sf = &obj->subFrameObj[subFrameNum];

        switch (cmd)
        {
            case DPC_OBJDET_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG:
            {
                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_CalibDcRangeSigCfg));
                DPC_ObjectDetection_CalibDcRangeSigCfg *cfg =
                    (DPC_ObjectDetection_CalibDcRangeSigCfg*)arg;

                int32_t ret = DPU_RangeProcHWA_control(sf->dpuRangeObj,
                                  DPU_RangeProcHWA_Cmd_dcRangeCfg,
                                  &cfg->cfg,
                                  sizeof(DPU_RangeProc_CalibDcRangeSigCfg));
                if (ret != 0) return ret;

                sf->dynCfg.calibDcRangeSigCfg = cfg->cfg;
                return 0;
            }

            case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
            {
                if (!obj->isCommonCfgReceived)
                    return DPC_OBJECTDETECTION_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_PreStartCfg));
                DPC_ObjectDetection_PreStartCfg *cfg =
                    (DPC_ObjectDetection_PreStartCfg*)arg;

                /* Remember which subframe we are configuring so the window timing
                   stores in the correct slot. */
                gTimingConfigSubframe = subFrameNum;

                DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage *mem = &cfg->memUsage;
                mem->L3RamTotal        = obj->L3RamObj.cfg.size;
                mem->CoreLocalRamTotal = obj->CoreLocalRamObj.cfg.size;

                MemoryP_Stats s0, s1;
                MemoryP_getStats(&s0);

                int32_t ret = DPC_ObjDet_preStartConfig(sf,
                                  &obj->commonCfg,
                                  &cfg->staticCfg,
                                  &cfg->dynCfg,
                                  &obj->edmaHandle[0],
                                  &obj->L3RamObj,
                                  &obj->CoreLocalRamObj,
                                  &obj->hwaMemBankAddr[0],
                                  obj->hwaMemBankSize,
                                  &mem->L3RamUsage,
                                  &mem->CoreLocalRamUsage);
                if (ret != 0) return ret;

                MemoryP_getStats(&s1);
                mem->SystemHeapTotal   = s1.totalSize;
                mem->SystemHeapUsed    = s1.totalSize - s1.totalFreeSize;
                mem->SystemHeapDPCUsed = s0.totalFreeSize - s1.totalFreeSize;

                DebugP_log1("ObjDet DPC: Pre-start CFG processed (subFrame=%d)\n", subFrameNum);
                return 0;
            }

            default:
                return DPC_OBJECTDETECTION_EINVAL__COMMAND;
        }
    }
}

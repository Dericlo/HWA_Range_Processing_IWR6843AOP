/*
 *   @file  mmw_cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <ti/demo/xwr64xx/mmw/mmw.h>
#include <ti/demo/xwr64xx/mmw/mmw_config.h>
#include <ti/demo/utils/mmwdemo_adcconfig.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICfarFovCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern MmwDemo_MCB    gMmwMCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

#define MMWDEMO_DATAUART_MAX_BAUDRATE_SUPPORTED 3125000
/* mmw_cli.c */

float calculateCenterFreq(float f0_GHz, float slope_MHzPerUs,
                          uint32_t numAdcSamples, uint32_t sampleRate_ksps,
                          float adcStartTime_us)
{
    float f0_Hz   = f0_GHz * 1e9f;
    float S_Hzps  = slope_MHzPerUs * 1e12f;
    float Fs_Hz   = sampleRate_ksps * 1e3f;
    float Ts      = numAdcSamples / Fs_Hz;
    float tStart  = adcStartTime_us * 1e-6f;

    float f_start = f0_Hz + S_Hzps * tStart;
    float f_end   = f_start + S_Hzps * Ts;

    return 0.5f * (f_start + f_end); // Hz
}
int32_t centerFreq = 0;
int32_t autoStart(void)
{
    uint32_t retVal;
    uint8_t  subFrameNum   = 0;   /* your cfg uses subFrameIdx 0 for guiMonitor etc */
    uint32_t procDirection = 0;   /* range */

    /************ LvdsStreamCfg  (lvdsStreamCfg -1 0 0 0) ************/
    {
        MmwDemo_LvdsStreamCfg cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.isHeaderEnabled = 0;
        cfg.dataFmt         = 0;  /* MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED */
        cfg.isSwEnabled     = 0;

        /* Save configuration */
        MmwDemo_CfgUpdate((void *)&cfg,
                          MMWDEMO_LVDSSTREAMCFG_OFFSET,
                          sizeof(MmwDemo_LvdsStreamCfg),
                          subFrameNum);
    }

    /************ analogMonitor 0 0 ************/
    gMmwMCB.anaMonCfg.rxSatMonEn  = 0;
    gMmwMCB.anaMonCfg.sigImgMonEn = 0;
    gMmwMCB.isAnaMonCfgPending    = 1;

    /************ measureRangeBiasAndRxChanPhase 0 5.30 0.0 ************/
    {
        DPC_ObjectDetection_MeasureRxChannelBiasCfg BiasCfg;
        memset(&BiasCfg, 0, sizeof(BiasCfg));

        BiasCfg.enabled        = 0;
        BiasCfg.targetDistance = 5.30f;
        BiasCfg.searchWinSize  = 0.0f;

        memcpy(&gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg,
               &BiasCfg,
               sizeof(BiasCfg));
        gMmwMCB.dataPathObj.objDetCommonCfg.isMeasureRxChannelBiasCfgPending = 1;
    }

    /************ compRangeBiasAndRxChanPhase 0.0 1 0 ************/
    {
        DPU_AoAProc_compRxChannelBiasCfg ChannelCfg;
        int32_t Re, Im;

        memset(&ChannelCfg, 0, sizeof(ChannelCfg));
        ChannelCfg.rangeBias = 0.0f;

        /* First (TX,RX) = 1 + j0 */
        Re = (int32_t)(1.0f * 32768.0f);
        MATHUTILS_SATURATE16(Re);
        ChannelCfg.rxChPhaseComp[0].real = (int16_t)Re;

        Im = (int32_t)(0.0f * 32768.0f);
        MATHUTILS_SATURATE16(Im);
        ChannelCfg.rxChPhaseComp[0].imag = (int16_t)Im;

        memcpy(&gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.compRxChanCfg,
               &ChannelCfg,
               sizeof(ChannelCfg));
        gMmwMCB.dataPathObj.objDetCommonCfg.isCompRxChannelBiasCfgPending = 1;
    }

    /************ adcbufCfg -1 0 1 1 1 ************/
    {
        MmwDemo_ADCBufCfg adcBufCfg;
        memset(&adcBufCfg, 0, sizeof(adcBufCfg));

        adcBufCfg.adcFmt         = 0;  /* complex */
        adcBufCfg.iqSwapSel      = 1;
        adcBufCfg.chInterleave   = 1;
        adcBufCfg.chirpThreshold = 1;  /* HWA requires single-chirp */

        MmwDemo_CfgUpdate((void *)&adcBufCfg,
                          MMWDEMO_ADCBUFCFG_OFFSET,
                          sizeof(MmwDemo_ADCBufCfg),
                          subFrameNum);
    }

    /************ calibDcRangeSig -1 0 -1 1 1 ************/
    {
        DPU_RangeProc_CalibDcRangeSigCfg SigCfg;
        memset(&SigCfg, 0, sizeof(SigCfg));

        SigCfg.enabled        = 0;   /* off, like cfg */
        SigCfg.negativeBinIdx = -1;
        SigCfg.positiveBinIdx =  1;
        SigCfg.numAvgChirps   =  1;

        /* (sanity check only if enabled) */
        if (SigCfg.enabled)
        {
            uint32_t log2NumAvgChirps;
            if (SigCfg.negativeBinIdx > 0 || SigCfg.positiveBinIdx < 0)
            {
                CLI_write("Error: Invalid DC range bin index\n");
                return -1;
            }
            if ((SigCfg.positiveBinIdx - SigCfg.negativeBinIdx + 1) >
                DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE)
            {
                CLI_write("Error: Number of bins exceeds the limit\n");
                return -1;
            }
            log2NumAvgChirps = (uint32_t)mathUtils_ceilLog2(SigCfg.numAvgChirps);
            if (SigCfg.numAvgChirps != (uint16_t)(1U << log2NumAvgChirps))
            {
                CLI_write("Error: numAvgChirps must be power of two (or 1)\n");
                return -1;
            }
        }

        MmwDemo_CfgUpdate((void *)&SigCfg,
                          MMWDEMO_CALIBDCRANGESIG_OFFSET,
                          sizeof(SigCfg),
                          subFrameNum);
    }

    /************ multiObjBeamForming -1 0 0.0 ************/
    {
        DPU_AoAProc_MultiObjBeamFormingCfg BeamFormingCfg;
        memset(&BeamFormingCfg, 0, sizeof(BeamFormingCfg));

        BeamFormingCfg.enabled           = 0;
        BeamFormingCfg.multiPeakThrsScal = 0.0f;

        MmwDemo_CfgUpdate((void *)&BeamFormingCfg,
                          MMWDEMO_MULTIOBJBEAMFORMING_OFFSET,
                          sizeof(BeamFormingCfg),
                          subFrameNum);
    }

    /************ cfarFovCfg -1 0 0 5.35  (range) ************/
    {
        DPU_CFARCAProc_FovCfg fovCfg;
        memset(&fovCfg, 0, sizeof(fovCfg));

        procDirection = 0;  /* 0 = range */
        fovCfg.min    = 0.0f;
        fovCfg.max    = 5.35f;

        MmwDemo_CfgUpdate((void *)&fovCfg,
                          MMWDEMO_FOVRANGE_OFFSET,
                          sizeof(fovCfg),
                          subFrameNum);
    }

    /************ cfarCfg -1 0 2 1 1 1 0 15 1  (range CFAR) ************/
    {
        DPU_CFARCAProc_CfarCfg cfarCfg;
        float threshold = 15.0f;

        memset(&cfarCfg, 0, sizeof(cfarCfg));
        procDirection         = 0;  /* 0 = range */
        cfarCfg.averageMode   = 2;
        cfarCfg.winLen        = 1;
        cfarCfg.guardLen      = 1;
        cfarCfg.noiseDivShift = 1;
        cfarCfg.cyclicMode    = 0;
        cfarCfg.peakGroupingEn= 1;

        threshold            *= MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR;
        cfarCfg.thresholdScale = (uint16_t)threshold;

        MmwDemo_CfgUpdate((void *)&cfarCfg,
                          MMWDEMO_CFARCFGRANGE_OFFSET,
                          sizeof(cfarCfg),
                          subFrameNum);
    }

    /************ guiMonitor 0 0 1 0 0 0 1 ************/
    {
        MmwDemo_GuiMonSel guiMonSel;
        memset(&guiMonSel, 0, sizeof(guiMonSel));

        guiMonSel.detectedObjects     = 0;
        guiMonSel.logMagRange         = 1;
        guiMonSel.noiseProfile        = 0;
        guiMonSel.rangeAzimuthHeatMap = 0;
        guiMonSel.rangeDopplerHeatMap = 0;
        guiMonSel.statsInfo           = 1;

        MmwDemo_CfgUpdate((void *)&guiMonSel,
                          MMWDEMO_GUIMONSEL_OFFSET,
                          sizeof(MmwDemo_GuiMonSel),
                          subFrameNum);
    }

    /************ Open + config + start (like sensorStart) ************/

    /* 1) Open sensor (equivalent to first part of sensorStart) */
    if (gMmwMCB.sensorState == MmwDemo_SensorState_INIT)
    {
        CLI_getMMWaveExtensionOpenConfig(&gMmwMCB.cfg.openCfg);
        retVal = MmwDemo_openSensor(true);
        if (retVal != 0) return -1;
        gMmwMCB.sensorState = MmwDemo_SensorState_OPENED;
    }
    else if (gMmwMCB.sensorState == MmwDemo_SensorState_OPENED)
    {
        MMWave_OpenCfg openCfg;
        CLI_getMMWaveExtensionOpenConfig(&openCfg);
        if (memcmp(&gMmwMCB.cfg.openCfg.chCfg, &openCfg.chCfg, sizeof(rlChanCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
        if (memcmp(&gMmwMCB.cfg.openCfg.adcOutCfg, &openCfg.adcOutCfg, sizeof(rlAdcOutCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
    }

    /* 2) RFParser sanity (like in MmwDemo_CLISensorStart) */
    {
        MMWave_CtrlCfg ctrlCfg;
        CLI_getMMWaveExtensionConfig(&gMmwMCB.cfg.ctrlCfg);
        CLI_getMMWaveExtensionConfig(&ctrlCfg);

        MmwDemo_RFParserOutParams rfTmp;
        rfTmp.adcBufChanDataSize = 512;
        rfTmp.chirpInterval = 55.14e-06;
        rfTmp.dopplerResolution = 2;
        rfTmp.dopplerStep = 2;
        rfTmp.framePeriod = 5;
        rfTmp.numAdcSamples = 128;
        rfTmp.numChirpsPerChirpEvent = 1;
        rfTmp.numChirpsPerFrame = 2;
        rfTmp.numDopplerBins = 2;
        rfTmp.numDopplerChirps = 2;
        rfTmp.numRangeBins = 128;
        rfTmp.numRxAntennas = 1;
        rfTmp.numTxAntennas = 1;
        rfTmp.numVirtualAntAzim = 0;
        rfTmp.numVirtualAntElev = 0;
        rfTmp.numVirtualAntennas = 1;
        rfTmp.rangeResolution = 0.0418526791;
        rfTmp.rangeStep = 0.0418526791;
        rfTmp.rxAntOrder[0] = 0;
        rfTmp.rxAntOrder[1] = 0;
        rfTmp.rxAntOrder[2] = 0;
        rfTmp.rxAntOrder[3] = 0;
        rfTmp.txAntOrder[0] = 0;
        rfTmp.txAntOrder[1] = 0;
        rfTmp.txAntOrder[2] = 0;
        rfTmp.validProfileHasOneTxPerChirp = 1;
        rfTmp.validProfileIdx = 0;

        centerFreq = calculateCenterFreq(63.95,-70.00,128,2500,1);
        rfTmp.centerFreq = 62.088e+09;

        gMmwMCB.cfg.openCfg.freqLimitHigh = 640;
        gMmwMCB.cfg.openCfg.freqLimitLow = 600;
        gMmwMCB.cfg.openCfg.calibMonTimeUnit = 1;
        gMmwMCB.cfg.openCfg.disableFrameStartAsyncEvent = 0;
        gMmwMCB.cfg.openCfg.disableFrameStopAsyncEvent = 0;
        gMmwMCB.cfg.openCfg.customCalibrationEnableMask = 0;
        gMmwMCB.cfg.openCfg.useCustomCalibration = 0;
        gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b14Reserved1 = 0;
        gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcBits = 2;
        gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt = 1;
        gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b6Reserved0 = 0;
        gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b8FullScaleReducFctr = 0;
        gMmwMCB.cfg.openCfg.adcOutCfg.reserved0 = 0;
        gMmwMCB.cfg.openCfg.adcOutCfg.reserved1 = 0;
        gMmwMCB.cfg.openCfg.chCfg.cascading = 0;
        gMmwMCB.cfg.openCfg.chCfg.cascadingPinoutCfg = 0;
        gMmwMCB.cfg.openCfg.chCfg.rxChannelEn = 1;
        gMmwMCB.cfg.openCfg.chCfg.txChannelEn = 1;
        gMmwMCB.cfg.openCfg.lowPowerMode.lpAdcMode = 0;
        gMmwMCB.cfg.openCfg.lowPowerMode.reserved = 0;
        gMmwMCB.cfg.ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
        gMmwMCB.rfFreqScaleFactor = 2.7000000000000002;
//        gMmwMCB.cfg.openCfg.defaultAsyncEventHandler = ;
//        int32_t ok = MmwDemo_RFParser_parseConfig(&rfTmp,
//                                                  0,
//                                                  &gMmwMCB.cfg.openCfg,
//                                                  &gMmwMCB.cfg.ctrlCfg,
//                                                  &gMmwMCB.subFrameCfg[0].adcBufCfg,
//                                                  gMmwMCB.rfFreqScaleFactor,
//                                                  false);
//        if (ok != 0)
//        {
//            CLI_write("Error: RFParser failed\n");
//            return -1;
//        }
    }

    /* 3) Configure sensor + datapath */
    retVal = MmwDemo_configSensor();
    if (retVal != 0) return -1;

    /* 4) Start sensor (equivalent to sensorStart) */
    retVal = MmwDemo_startSensor();
    if (retVal != 0) return -1;

    gMmwMCB.sensorState = MmwDemo_SensorState_STARTED;
    return 0;
}




//int32_t autoStart(void){
//    uint8_t procDirection;
//    uint32_t retVal;
//    uint8_t subFrameNum =0;
///* ---------------- LvdsStreamCfg ---------------- */
//    MmwDemo_LvdsStreamCfg   cfg;
//    /* Populate configuration: */
//    cfg.isHeaderEnabled = 0;
//    cfg.dataFmt         = 0;
//    cfg.isSwEnabled     = 0;
//
//    if ((cfg.isSwEnabled == true) && (cfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
//    {
//        if (cfg.isHeaderEnabled == false)
//        {
//            CLI_write("Error: header must be enabled when both h/w and s/w streaming are enabled\n");
//            return -1;
//        }
//    }
//
//    /* Save Configuration to use later */
//    MmwDemo_CfgUpdate((void *)&cfg,
//                      MMWDEMO_LVDSSTREAMCFG_OFFSET,
//                      sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);
///* ---------------- LvdsStreamCfg ---------------- */
//
///* ---------------- AnalogMonitorCfg  ---------------- */
//   /* Save Configuration to use later */
//   gMmwMCB.anaMonCfg.rxSatMonEn = 0;
//   gMmwMCB.anaMonCfg.sigImgMonEn = 0;
//   gMmwMCB.isAnaMonCfgPending = 1;
///* ---------------- AnalogMonitorCfg  ---------------- */
//
///* ---------------- MeasureRangeBiasAndRxChanPhaseCfg  ---------------- */
//    DPC_ObjectDetection_MeasureRxChannelBiasCfg   BiasCfg;
//    /* Populate configuration: */
//    BiasCfg.enabled          = 0;
//    BiasCfg.targetDistance   = 5.30;
//    BiasCfg.searchWinSize   = 0.0;
//
//    /* Save Configuration to use later */
//    memcpy((void *) &gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg,
//           &cfg, sizeof(cfg));
//
//    gMmwMCB.dataPathObj.objDetCommonCfg.isMeasureRxChannelBiasCfgPending = 1;
///* ---------------- MeasureRangeBiasAndRxChanPhaseCfg  ---------------- */
//
///* ---------------- CompRangeBiasAndRxChanPhaseCfg  ---------------- */
//   DPU_AoAProc_compRxChannelBiasCfg   ChannelCfg;
//   int32_t Re, Im;
//
//   /* Populate configuration: */
//   ChannelCfg.rangeBias          = 0.0;
//
//   Re = (int32_t) (1* 32768);
//   MATHUTILS_SATURATE16(Re);
//   ChannelCfg.rxChPhaseComp[0].real = (int16_t) Re;
//
//   Im = (int32_t) (0 * 32768);
//   MATHUTILS_SATURATE16(Im);
//   ChannelCfg.rxChPhaseComp[0].imag = (int16_t) Im;
//
//   /* Save Configuration to use later */
//   memcpy((void *) &gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.compRxChanCfg,
//          &cfg, sizeof(cfg));
//
//   gMmwMCB.dataPathObj.objDetCommonCfg.isCompRxChannelBiasCfgPending = 1;
//
///* ---------------- CompRangeBiasAndRxChanPhaseCfg  ---------------- */
//
///* ---------------- ADCBufCfg  ---------------- */
//    MmwDemo_ADCBufCfg   adcBufCfg;
//    /* Populate configuration: */
//    adcBufCfg.adcFmt          = 0;
//    adcBufCfg.iqSwapSel       = 1;
//    adcBufCfg.chInterleave    = 1;
//    adcBufCfg.chirpThreshold  = 1; //chirpThreshold must be 1, multi-chirp is not allowed
//
//    /* Save Configuration to use later */
//    MmwDemo_CfgUpdate((void *)&adcBufCfg,
//                      MMWDEMO_ADCBUFCFG_OFFSET,
//                      sizeof(MmwDemo_ADCBufCfg), subFrameNum);
///* ---------------- ADCBufCfg  ---------------- */
//
///* ---------------- CalibDcRangeSig  ---------------- */
//    DPU_RangeProc_CalibDcRangeSigCfg SigCfg;
//    uint32_t log2NumAvgChirps;
//    SigCfg.enabled        = 0;
//    SigCfg.negativeBinIdx = -1;
//    SigCfg.positiveBinIdx = 1;
//    SigCfg.numAvgChirps   = 1;
//
//    if (SigCfg.enabled)
//    {
//        /* Basic bounds checks; we don’t look at numLoops here */
//        if (SigCfg.negativeBinIdx > 0)
//        {
//            CLI_write ("Error: Invalid negative bin index\n");
//            return -1;
//        }
//        if (SigCfg.positiveBinIdx < 0)
//        {
//            CLI_write ("Error: Invalid positive bin index\n");
//            return -1;
//        }
//        if ((SigCfg.positiveBinIdx - SigCfg.negativeBinIdx + 1) > DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE)
//        {
//            CLI_write ("Error: Number of bins exceeds the limit\n");
//            return -1;
//        }
//        /* For multi-loop cases, SDK expects power-of-two averaging.
//           For 1-loop we later force numAvgChirps=1 in dataPathConfig. */
//        log2NumAvgChirps = (uint32_t) mathUtils_ceilLog2(SigCfg.numAvgChirps);
//        if (SigCfg.numAvgChirps != (uint16_t)(1U << log2NumAvgChirps))
//        {
//            CLI_write ("Error: numAvgChirps must be power of two (or 1; 1-loop will be clamped later)\n");
//            return -1;
//        }
//    }
//
//    MmwDemo_CfgUpdate((void *)&SigCfg,
//                      MMWDEMO_CALIBDCRANGESIG_OFFSET,
//                      sizeof(SigCfg),
//                      subFrameNum);
///* ---------------- CalibDcRangeSig ---------------- */
//
///* ---------------- MultiObjBeamForming ---------------- */
//    DPU_AoAProc_MultiObjBeamFormingCfg BeamFormingCfg;
//    /* Populate configuration: */
//    BeamFormingCfg.enabled                     = 0;
//    BeamFormingCfg.multiPeakThrsScal           = 0.0;
//
//    /* Save Configuration to use later */
//    MmwDemo_CfgUpdate((void *)&BeamFormingCfg, MMWDEMO_MULTIOBJBEAMFORMING_OFFSET,
//                      sizeof(BeamFormingCfg), subFrameNum);
///* ---------------- MultiObjBeamForming ---------------- */
//
///* ---------------- cfarfovCfg ---------------- */
//    DPU_CFARCAProc_FovCfg   fovCfg;
//    procDirection             = 0;
//    fovCfg.min                = 0;
//    fovCfg.max                = 5.35;
//
//    /* Save Configuration to use later */
//    MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVRANGE_OFFSET,
//                      sizeof(fovCfg), subFrameNum);
///* ---------------- cfarfovCfg ---------------- */
///* ---------------- cfarCfg ---------------- */
//    DPU_CFARCAProc_CfarCfg cfarCfg;
//    float threshold;
//    procDirection             = 0;
//    cfarCfg.averageMode       = 2;
//    cfarCfg.winLen            = 1;
//    cfarCfg.guardLen          = 1;
//    cfarCfg.noiseDivShift     = 1;
//    cfarCfg.cyclicMode        = 0;
//    threshold                 = 15; //Maximum value for CFAR thresholdScale is 100.0 dB.
//    cfarCfg.peakGroupingEn    = 1;
//
//    threshold = threshold * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR;
//    cfarCfg.thresholdScale    = (uint16_t) threshold;
//
//    /* Save Configuration to use later */
//    MmwDemo_CfgUpdate((void *)&cfarCfg, MMWDEMO_CFARCFGRANGE_OFFSET,
//                      sizeof(cfarCfg), subFrameNum);
///* ---------------- cfarCfg ---------------- */
///* ---------------- GuiMonSel ---------------- */
//    MmwDemo_GuiMonSel guiMonSel;
//    guiMonSel.detectedObjects           = 0;
//    guiMonSel.logMagRange               = 1;
//    guiMonSel.noiseProfile              = 0;
//    guiMonSel.rangeAzimuthHeatMap       = 0;
//    guiMonSel.rangeDopplerHeatMap       = 0;
//    guiMonSel.statsInfo                 = 1;
//
//    MmwDemo_CfgUpdate((void *)&guiMonSel, MMWDEMO_GUIMONSEL_OFFSET,
//        sizeof(MmwDemo_GuiMonSel), subFrameNum);
///* ---------------- GuiMonSel ---------------- */
//    MMWave_CtrlCfg ctrlCfg;
//    CLI_getMMWaveExtensionConfig(&ctrlCfg);
//    gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames =
//        MmwDemo_RFParser_getNumSubFrames(&ctrlCfg);
//
//    if (MmwDemo_isAllCfgInPendingState() == 0)
//    {
//        CLI_write("Error: Full configuration must be provided before first start\n");
//        gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames = 0;
//        return -1;
//    }
//    /* ---------------- No-reconfig path sanity ---------------- */
//
//    if (!MmwDemo_isAllCfgInNonPendingState())
//    {
//        if (MmwDemo_isAllCfgInPendingState())
//        {
//            CLI_write("Error: New configuration present, use \"sensorStart\" (no argument)\n");
//        }
//        else
//        {
//            CLI_write("Error: Partial configuration present; issue full config then sensorStart\n");
//        }
//        return -1;
//    }
//
//    /* ---------------- First-time open or re-open consistency ---------------- */
//    if (gMmwMCB.sensorState == MmwDemo_SensorState_INIT)
//    {
//        CLI_getMMWaveExtensionOpenConfig(&gMmwMCB.cfg.openCfg);
//        retVal = MmwDemo_openSensor(true);
//        if (retVal != 0) return -1;
//        gMmwMCB.sensorState = MmwDemo_SensorState_OPENED;
//    }
//    else
//    {
//        /* Verify openCfg has not changed since first start */
//        MMWave_OpenCfg openCfg;
//        CLI_getMMWaveExtensionOpenConfig(&openCfg);
//        if (memcmp(&gMmwMCB.cfg.openCfg.chCfg,        &openCfg.chCfg,        sizeof(rlChanCfg_t))        != 0) { MmwDemo_debugAssert(0); }
//        //if (memcmp(&gMmwMCB.cfg.openCfg.lowPowerMode, &openCfg.lowPowerMode, sizeof(rlLowPowerModeCfg_t))!= 0) { MmwDemo_debugAssert(0); }
//        if (memcmp(&gMmwMCB.cfg.openCfg.adcOutCfg,    &openCfg.adcOutCfg,    sizeof(rlAdcOutCfg_t))       != 0) { MmwDemo_debugAssert(0); }
//    }
//
//    /* ---------------- Configure & Start ---------------- */
//    /* Keep a local copy to inspect frame layout (for info only). */
//    MMWave_CtrlCfg mctrlCfg;
//    CLI_getMMWaveExtensionConfig(&gMmwMCB.cfg.ctrlCfg);
//    CLI_getMMWaveExtensionConfig(&mctrlCfg);
//
//    /* Re-parse RF to get effective doppler chirps (numLoops * active TX per burst). */
//    {
//        MmwDemo_RFParserOutParams rfTmp;
//        int32_t ok = MmwDemo_RFParser_parseConfig(&rfTmp,
//                                                  0,
//                                                  &gMmwMCB.cfg.openCfg,
//                                                  &gMmwMCB.cfg.ctrlCfg,
//                                                  &gMmwMCB.subFrameCfg[0].adcBufCfg,
//                                                  gMmwMCB.rfFreqScaleFactor,
//                                                  false);
//        if (ok != 0)
//        {
//            CLI_write("Error: RFParser failed\n");
//            return -1;
//        }
//    }
//
//    /* Configure sensor as usual; Range/HWA path must already support numLoops=1. */
//    retVal = MmwDemo_configSensor();
//    //if (retVal != 0) return -1;
//    retVal = MmwDemo_startSensor();
//    //if (retVal != 0) return -1;
//    gMmwMCB.sensorState = MmwDemo_SensorState_STARTED;
//}




static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool    doReconfig = true;
    int32_t retVal     = 0;

    /* Accept: sensorStart  OR sensorStart 0 (no reconfig) */
    if (argc == 2)
    {
        doReconfig = (bool)atoi(argv[1]);
        if (doReconfig == true)
        {
            CLI_write("Error: Reconfig is not supported, only argument of 0 is valid\n");
            return -1;
        }
    }
    else
    {
        doReconfig = true; /* default: reconfig */
    }

    /* ---------------- Sensor state checks (non-fatal) ---------------- */
    if ((gMmwMCB.sensorState == MmwDemo_SensorState_INIT) ||
        (gMmwMCB.sensorState == MmwDemo_SensorState_OPENED))
    {
        MMWave_CtrlCfg ctrlCfg;
        CLI_getMMWaveExtensionConfig(&ctrlCfg);
        gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            MmwDemo_RFParser_getNumSubFrames(&ctrlCfg);

        if (MmwDemo_isAllCfgInPendingState() == 0)
        {
            CLI_write("Error: Full configuration must be provided before first start\n");
            gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.numSubFrames = 0;
            return -1;
        }
    }

    if (gMmwMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write("Ignored: Sensor is already started\n");
        return 0;
    }

    /* ---------------- No-reconfig path sanity ---------------- */
    if (!doReconfig)
    {
        if (!MmwDemo_isAllCfgInNonPendingState())
        {
            if (MmwDemo_isAllCfgInPendingState())
            {
                CLI_write("Error: New configuration present, use \"sensorStart\" (no argument)\n");
            }
            else
            {
                CLI_write("Error: Partial configuration present; issue full config then sensorStart\n");
            }
            return -1;
        }
    }

    /* ---------------- First-time open or re-open consistency ---------------- */
    if (gMmwMCB.sensorState == MmwDemo_SensorState_INIT)
    {
        CLI_getMMWaveExtensionOpenConfig(&gMmwMCB.cfg.openCfg);
        retVal = MmwDemo_openSensor(true);
        if (retVal != 0) return -1;
        gMmwMCB.sensorState = MmwDemo_SensorState_OPENED;
    }
    else
    {
        /* Verify openCfg has not changed since first start */
        MMWave_OpenCfg openCfg;
        CLI_getMMWaveExtensionOpenConfig(&openCfg);
        if (memcmp(&gMmwMCB.cfg.openCfg.chCfg,        &openCfg.chCfg,        sizeof(rlChanCfg_t))        != 0) { MmwDemo_debugAssert(0); }
        if (memcmp(&gMmwMCB.cfg.openCfg.lowPowerMode, &openCfg.lowPowerMode, sizeof(rlLowPowerModeCfg_t))!= 0) { MmwDemo_debugAssert(0); }
        if (memcmp(&gMmwMCB.cfg.openCfg.adcOutCfg,    &openCfg.adcOutCfg,    sizeof(rlAdcOutCfg_t))       != 0) { MmwDemo_debugAssert(0); }
    }

    /* ---------------- Configure & Start ---------------- */
    if (doReconfig)
    {
        /* Keep a local copy to inspect frame layout (for info only). */
        MMWave_CtrlCfg ctrlCfg;
        CLI_getMMWaveExtensionConfig(&gMmwMCB.cfg.ctrlCfg);
        CLI_getMMWaveExtensionConfig(&ctrlCfg);

        /* Re-parse RF to get effective doppler chirps (numLoops * active TX per burst). */
        {
            MmwDemo_RFParserOutParams rfTmp;
            int32_t ok = MmwDemo_RFParser_parseConfig(&rfTmp,
                                                      0,
                                                      &gMmwMCB.cfg.openCfg,
                                                      &gMmwMCB.cfg.ctrlCfg,
                                                      &gMmwMCB.subFrameCfg[0].adcBufCfg,
                                                      gMmwMCB.rfFreqScaleFactor,
                                                      false);
            if (ok != 0)
            {
                CLI_write("Error: RFParser failed\n");
                return -1;
            }

            /* If single-chirp (e.g., 1 TX and numLoops=1 => 1 doppler chirp), allow it and inform. */
            if (rfTmp.numDopplerChirps < 2U)
            {
                CLI_write("Info: Single-chirp frame detected (numLoops=1). Range FFT will run; Doppler processing features that\n");
                CLI_write("      rely on multiple chirps will be limited/disabled by the datapath as applicable.\n");
            }
        }

        /* Configure sensor as usual; Range/HWA path must already support numLoops=1. */
        retVal = MmwDemo_configSensor();
        if (retVal != 0) return -1;
    }

    retVal = MmwDemo_startSensor();
    if (retVal != 0) return -1;

    gMmwMCB.sensorState = MmwDemo_SensorState_STARTED;
    return 0;
}




/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    if ((gMmwMCB.sensorState == MmwDemo_SensorState_STOPPED) ||
        (gMmwMCB.sensorState == MmwDemo_SensorState_INIT) ||
        (gMmwMCB.sensorState == MmwDemo_SensorState_OPENED))
    {
        CLI_write ("Ignored: Sensor is already stopped\n");
        return 0;
    }
    
    MmwDemo_stopSensor();
    
    MmwDemo_resetStaticCfgPendingState();

    gMmwMCB.sensorState = MmwDemo_SensorState_STOPPED;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to get sub-frame number
 *
 *  @param[in] argc  Number of arguments
 *  @param[in] argv  Arguments
 *  @param[in] expectedArgc Expected number of arguments
 *  @param[out] subFrameNum Sub-frame Number (0 based)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc,
                                       int8_t* subFrameNum)
{
    int8_t subframe;
    
    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }
    
    *subFrameNum = (int8_t)subframe;

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
MmwDemo_GuiMonSel   guiMonSel;
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{

    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 8, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[2]);
    guiMonSel.logMagRange               = atoi (argv[3]);
    guiMonSel.noiseProfile              = atoi (argv[4]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    guiMonSel.statsInfo                 = atoi (argv[7]);

    MmwDemo_CfgUpdate((void *)&guiMonSel, MMWDEMO_GUIMONSEL_OFFSET,
        sizeof(MmwDemo_GuiMonSel), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
DPU_CFARCAProc_CfarCfg   cfarCfg;
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
    //DPU_CFARCAProc_CfarCfg   cfarCfg;
    uint32_t            procDirection;
    int8_t              subFrameNum;
    float               threshold;

    if(MmwDemo_CLIGetSubframe(argc, argv, 10, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(cfarCfg));

    /* Populate configuration: */
    procDirection             = (uint32_t) atoi (argv[2]);
    cfarCfg.averageMode       = (uint8_t) atoi (argv[3]);
    cfarCfg.winLen            = (uint8_t) atoi (argv[4]);
    cfarCfg.guardLen          = (uint8_t) atoi (argv[5]);
    cfarCfg.noiseDivShift     = (uint8_t) atoi (argv[6]);
    cfarCfg.cyclicMode        = (uint8_t) atoi (argv[7]);
    threshold                 = (float) atof (argv[8]);
    cfarCfg.peakGroupingEn    = (uint8_t) atoi (argv[9]);

    if (threshold > 100.0)
    {
        CLI_write("Error: Maximum value for CFAR thresholdScale is 100.0 dB.\n");
        return -1;
    }   
    
    /* threshold is a float value from 0-100dB. It needs to
       be later converted to linear scale (conversion can only be done
       when the number of virtual antennas is known) before passing it
       to CFAR DPU.
       For now, the threshold will be coded in a 16bit integer in the following
       way:
       suppose threshold is a float represented as XYZ.ABC
       it will be saved as a 16bit integer XYZAB       
       that is, 2 decimal cases are saved.*/
    threshold = threshold * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR;   
    cfarCfg.thresholdScale    = (uint16_t) threshold;
    
    /* Save Configuration to use later */     
    if (procDirection == 0)
    {
        MmwDemo_CfgUpdate((void *)&cfarCfg, MMWDEMO_CFARCFGRANGE_OFFSET,
                          sizeof(cfarCfg), subFrameNum);
    }
    else
    {
        MmwDemo_CfgUpdate((void *)&cfarCfg, MMWDEMO_CFARCFGDOPPLER_OFFSET,
                          sizeof(cfarCfg), subFrameNum);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR FOV (Field Of View) configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICfarFovCfg (int32_t argc, char* argv[])
{
    DPU_CFARCAProc_FovCfg   fovCfg;
    uint32_t            procDirection;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&fovCfg, 0, sizeof(fovCfg));

    /* Populate configuration: */
    procDirection             = (uint32_t) atoi (argv[2]);
    fovCfg.min                = (float) atof (argv[3]);
    fovCfg.max                = (float) atof (argv[4]);

    /* Save Configuration to use later */
    if (procDirection == 0)
    {
        MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVRANGE_OFFSET,
                          sizeof(fovCfg), subFrameNum);
    }
    else
    {
        MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVDOPPLER_OFFSET,
                          sizeof(fovCfg), subFrameNum);
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for multi object beam forming configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[])
{
    DPU_AoAProc_MultiObjBeamFormingCfg cfg;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 4, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled                     = (uint8_t) atoi (argv[2]);
    cfg.multiPeakThrsScal           = (float) atof (argv[3]);

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_MULTIOBJBEAMFORMING_OFFSET,
                      sizeof(cfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DC range calibration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

/* CLI: calibDcRangeSig <subFrameIdx> <enabled> <negBin> <posBin> <numAvgChirps> */
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    DPU_RangeProc_CalibDcRangeSigCfg cfg;
    uint32_t log2NumAvgChirps;
    int8_t   subFrameNum;

    if (MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
        return -1;

    memset(&cfg, 0, sizeof(cfg));
    cfg.enabled        = (uint16_t) atoi (argv[2]);
    cfg.negativeBinIdx = (int16_t)  atoi (argv[3]);
    cfg.positiveBinIdx = (int16_t)  atoi (argv[4]);
    cfg.numAvgChirps   = (uint16_t) atoi (argv[5]);

    if (cfg.enabled)
    {
        /* Basic bounds checks; we don’t look at numLoops here */
        if (cfg.negativeBinIdx > 0)
        {
            CLI_write ("Error: Invalid negative bin index\n");
            return -1;
        }
        if (cfg.positiveBinIdx < 0)
        {
            CLI_write ("Error: Invalid positive bin index\n");
            return -1;
        }
        if ((cfg.positiveBinIdx - cfg.negativeBinIdx + 1) > DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE)
        {
            CLI_write ("Error: Number of bins exceeds the limit\n");
            return -1;
        }
        /* For multi-loop cases, SDK expects power-of-two averaging.
           For 1-loop we later force numAvgChirps=1 in dataPathConfig. */
        log2NumAvgChirps = (uint32_t) mathUtils_ceilLog2(cfg.numAvgChirps);
        if (cfg.numAvgChirps != (uint16_t)(1U << log2NumAvgChirps))
        {
            CLI_write ("Error: numAvgChirps must be power of two (or 1; 1-loop will be clamped later)\n");
            return -1;
        }
    }

    MmwDemo_CfgUpdate((void *)&cfg,
                      MMWDEMO_CALIBDCRANGESIG_OFFSET,
                      sizeof(cfg),
                      subFrameNum);
    return 0;
}





/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    int8_t              subFrameNum;

    if (gMmwMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(adcBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* This demo is using HWA for 1D processing which does not allow multi-chirp
     * processing */
    if (adcBufCfg.chirpThreshold != 1)
    {
        CLI_write("Error: chirpThreshold must be 1, multi-chirp is not allowed\n");
        return -1;
    }
    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&adcBufCfg,
                      MMWDEMO_ADCBUFCFG_OFFSET,
                      sizeof(MmwDemo_ADCBufCfg), subFrameNum);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPU_AoAProc_compRxChannelBiasCfg   cfg;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
//    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
//    {
//        CLI_write ("Error: Invalid usage of the CLI command\n");
//        return -1;
//    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < 1/*SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*/; i++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;

    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.compRxChanCfg,
           &cfg, sizeof(cfg));

    gMmwMCB.dataPathObj.objDetCommonCfg.isCompRxChannelBiasCfgPending = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPC_ObjectDetection_MeasureRxChannelBiasCfg   cfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMCB.dataPathObj.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg,
           &cfg, sizeof(cfg));

    gMmwMCB.dataPathObj.objDetCommonCfg.isMeasureRxChannelBiasCfgPending = 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling analog monitors
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    if (gMmwMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMCB.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gMmwMCB.anaMonCfg.sigImgMonEn = atoi (argv[2]);

    gMmwMCB.isAnaMonCfgPending = 1;

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the High Speed Interface
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{
    MmwDemo_LvdsStreamCfg   cfg;
    int8_t                  subFrameNum;
    if (gMmwMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool)    atoi(argv[2]);
    cfg.dataFmt         = (uint8_t) atoi(argv[3]);
    cfg.isSwEnabled     = (bool)    atoi(argv[4]);

    /* If both h/w and s/w are enabled, HSI header must be enabled, because
     * we don't allow mixed h/w session without HSI header
     * simultaneously with s/w session with HSI header (s/w session always
     * streams HSI header) */
    if ((cfg.isSwEnabled == true) && (cfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
    {
        if (cfg.isHeaderEnabled == false)
        {
            CLI_write("Error: header must be enabled when both h/w and s/w streaming are enabled\n");
            return -1;
        }
    }

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg,
                      MMWDEMO_LVDSSTREAMCFG_OFFSET,
                      sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);

    return 0;
}





/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (uint8_t taskPriority)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR64xx MMW Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );


    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMCB.commandUartHandle;
    cliCfg.taskPriority                 = taskPriority;
    cliCfg.socHandle                    = gMmwMCB.socHandle;
    cliCfg.mmWaveHandle                 = gMmwMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.overridePlatform             = true;
#if defined(USE_2D_AOA_DPU)
    cliCfg.overridePlatformString       = "xWR68xx_AOP";
#else
    cliCfg.overridePlatformString       = "xWR64xx";
#endif
    
    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <procDirection> <averageMode> <winLen> <guardLen> <noiseDiv> <cyclicMode> <thresholdScale> <peakGroupingEn>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "multiObjBeamForming";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <threshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMultiObjBeamForming;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "calibDcRangeSig";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <negativeBinIdx> <positiveBinIdx> <numAvgFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICalibDcRangeSig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarFovCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <procDirection> <min (meters or m/s)> <max (meters or m/s)>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICfarFovCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIAnalogMonitorCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLILvdsStreamCfg;
    cnt++;


    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}



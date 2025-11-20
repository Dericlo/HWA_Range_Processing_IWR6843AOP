/**
 *   @file  mmwdemo_rfparser.c
 *
 *   @brief
 *      Implements rf parser.
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
#include <math.h>

#ifdef MMWDEMO_RFPARSER_DBG
/* enable float extended format in BIOS cfg file using System.extendedFormats
   to able to print %f values */
#include <xdc/runtime/System.h>
#endif

/* mmWave SDK Include files */
#include <ti/common/sys_common.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>

/* MATH utils library Include files */
#include <ti/utils/mathutils/mathutils.h>

/** @defgroup MMWDEMO_RFPARSER_INTERNAL       Mmwdemo RFparser Internal
 */

/**
@defgroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION            RF Parser Internal Functions
@ingroup MMWDEMO_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup MMWDEMO_RFPARSER_INTERNAL_DATA_STRUCTURE      RF Parser Internal Data Structures
@ingroup MMWDEMO_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal data structures which are used internally
*   by the RF Parser module.
*/
/**
@defgroup MMWDEMO_RFPARSER_INTERNAL_DEFINITION          RF Parser Internal Definitions
@ingroup MMWDEMO_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal definitions which are used internally
*   by the RF Parser.
*/


/** @addtogroup MMWDEMO_RFPARSER_INTERNAL_DEFINITION
 *
 @{ */

/*! Speed of light in m/s expressed in float */
#define MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

/** @} */

/**
 * @brief
 *  Data Source Hardware configuration definition
 *
 * @details
 *  The structure describes the hardware related configuration for device and evm
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_DATA_STRUCTURE
 */
typedef struct MmwDemo_RFParserHwAttr_t
{
    /**
     * @brief   ADC buffer size
     */
    uint32_t      adcBufSize;

    /**
     * @brief   Tx Antenna mask for elevation, antenna pattern specific
     */
    uint8_t       elevTxAntMask;

    /**
     * @brief   Tx Antenna mask for azimuth, antenna pattern specific
     */
    uint8_t       azimTxAntMask;
} MmwDemo_RFParserHwAttr;

/*================================================================
               RF Parser platform dependent params
 ================================================================*/
#ifdef SOC_XWR14XX
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_XWR14XX_MSS_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SOC_XWR16XX
#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0,
    0x3
};
#endif

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0,
    0x3
};
#endif

#endif

#ifdef SOC_XWR18XX

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg = 
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#endif

#ifdef SOC_XWR68XX

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#endif
/*================================================================
               RF Parser internal APIs
 ================================================================*/

/**
 *  @b Description
 *  @n
 *      Help Function to get frame period
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Frame period
 */
static float MmwDemo_RFParser_getFramePeriod_ms(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].subFramePeriodicity * 0.000005f);
    }
    else
    {
        return((float)ctrlCfg->u.frameCfg.frameCfg.framePeriodicity * 0.000005f);
    }
}


/**
 *  @b Description
 *  @n
 *      Help Function to get chirp start Index
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Chirp start index
 */
static uint16_t MmwDemo_RFParser_getChirpStartIdx(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.chirpStartIdx);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get chirp stop Index
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Chirp stop index
 */
static uint16_t MmwDemo_RFParser_getChirpEndIdx(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx +
              (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfChirps - 1));
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.chirpEndIdx);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get number of loops
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of loops
 */
static uint16_t MmwDemo_RFParser_getNumLoops(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numLoops);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.numLoops);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get profile handle
 *
 *  @param[in] ctrlCfg          Handle to MMWave control configuration
 *  @param[in] profileLoopIdx   Profile index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Profile handle
 */
static MMWave_ProfileHandle MmwDemo_RFParser_getProfileHandle(MMWave_CtrlCfg *ctrlCfg, uint32_t profileLoopIdx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.profileHandle[profileLoopIdx]);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.profileHandle[profileLoopIdx]);
    }
}

/**
 *  @b Description
 *  @n
 *      Helper function that parses chirp Tx antenna configuration and extracts parameters
 *      needed for AoA configuration depending on the AoA DPU needs
 *
 *  @param[inout]   outParams               Pointer to parameters generated after parsing configuration
 *  @param[inout]   pFoundValidProfile      flag to indicate if this is valid profile; if not, advance to the next one
 *  @param[in]      frameTotalChirps        Total chirps in the frame (used for looping through validChirpTxEnBits)
 *  @param[in]      validChirpTxEnBits      Array of chirp's txEn bits. Dimension is provided by frameTotalChirps
 *  @param[in]      bpmEnabled              BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        <0, one of error codes @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
static int32_t MmwDemo_RFParser_setAoAParams
(
    MmwDemo_RFParserOutParams  *outParams,
    bool                       *pFoundValidProfile,
    int16_t                     frameTotalChirps,
    uint16_t                   *validChirpTxEnBits,
    bool                        bpmEnabled
)
{
    uint32_t    chirpLoopIdx;
    bool        validProfileHasOneTxPerChirp = false;
    uint16_t    validProfileTxEn = 0;
    int32_t     retVal = 0;
    int32_t     i;

    
//    /* for this antenna pattern we require that all Rx's are enabled */
//    if (outParams->numRxAntennas != SYS_COMMON_NUM_RX_CHANNEL)
//    {
//        retVal = MMWDEMO_RFPARSER_EINVAL__NUM_RX_ANTENNAS;
//        goto exit;
//    }
    
    /* now loop through unique chirps and check if we found all of the ones
       needed for the frame and then determine the azimuth/elevation antenna
       configuration
     */
    if (*pFoundValidProfile==true) {
        for (chirpLoopIdx = 0; chirpLoopIdx < frameTotalChirps; chirpLoopIdx++)
        {
            bool        validChirpHasOneTxPerChirp = false;
            uint8_t     log2TxEn;
            uint16_t    chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
            if (chirpTxEn == 0) {
                /* this profile doesn't have all the needed chirps */
                *pFoundValidProfile = false;
                break;
            }

            /* If only TX antenna is enabled, then its 0x1<< log2TxEn should equal chirpTxEn */
            log2TxEn = mathUtils_floorLog2(chirpTxEn);
            validChirpHasOneTxPerChirp = (chirpTxEn == (1 << log2TxEn));

            /* for this antenna pattern we require that only one Tx per chirp is enabled*/
            if (validChirpHasOneTxPerChirp != true)
            {
                /* this profile doesn't have chirps with one TxAnt enabled per Chirp */
                *pFoundValidProfile = false;
                break;
            }

            /* if this is the first chirp, record the chirp's
            MIMO config as profile's MIMO config. We dont handle intermix
            at this point */
            if (chirpLoopIdx == 0)
            {
                validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
            }
            /* check the chirp's MIMO config against Profile's MIMO config */
            if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
            {
                /* this profile doesnt have all chirps with same MIMO config */
                *pFoundValidProfile = false;
                break;
            }

            /* save the antennas actually enabled in this profile */
            validProfileTxEn |= chirpTxEn;
        }
    }

    /* found valid chirps for the frame; mark this profile valid */
    if (*pFoundValidProfile == true) {
        uint16_t        tempValidProfileTxEn=0;
        
        /* set Tx Antenna and Virtual antenna related config */
        outParams->numTxAntennas = 0;
        tempValidProfileTxEn = validProfileTxEn;
        while (tempValidProfileTxEn!=0) {
            if ((tempValidProfileTxEn&0x1)==0x1)
            {
                outParams->numTxAntennas++;
            }
            tempValidProfileTxEn=tempValidProfileTxEn>>1;
        }
        if (outParams->numTxAntennas > SYS_COMMON_NUM_TX_ANTENNAS)
        {
            retVal = MMWDEMO_RFPARSER_EINVAL_NUM_TX_ANTENNAS;
            goto exit;
        }
        outParams->numVirtualAntAzim = 0; // unused
        outParams->numVirtualAntElev = 0; // unused
        outParams->numVirtualAntennas = 1;//outParams->numTxAntennas * outParams->numRxAntennas;
        /* Sanity Check: Ensure that the number of antennas is within system limits */
//        if ((outParams->numVirtualAntennas <= 0) ||
//           (outParams->numVirtualAntennas > (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL)))
//        {
//            retVal = MMWDEMO_RFPARSER_EINVAL__NUM_VIRTUAL_ANTENNAS;
//        }

        /* txAntOrder[] will be needed for Rx Channel Phase Measurement/Compensation routines */
        for (i = 0; i < outParams->numTxAntennas; i++)
        {
            outParams->txAntOrder[i] = mathUtils_floorLog2(validChirpTxEnBits[i]);
        }

        outParams->validProfileHasOneTxPerChirp = validProfileHasOneTxPerChirp;
    }
    
exit:
    return (retVal);    
}


static inline uint32_t fold_bin(uint32_t k, uint32_t N)
{
    return (k <= (N>>1)) ? k : (N - k);
}


#include <math.h>   /* fabsf */
rlProfileCfg_t  profileCfg;
static int32_t MmwDemo_RFParser_parseCtrlConfig
(
    MmwDemo_RFParserOutParams  *outParams,
    uint8_t              subFrameIdx,
    MMWave_OpenCfg      *openCfg,
    MMWave_CtrlCfg      *ctrlCfg,
    float               rfFreqScaleFactor,
    bool                bpmEnabled
)
{
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    uint32_t    profileLoopIdx, chirpLoopIdx;
    bool        foundValidProfile = false;
    uint16_t    channelTxEn = openCfg->chCfg.txChannelEn;
    uint8_t     channel;
    uint8_t     numRxAntennas;
    uint16_t    numLoops;
    int32_t     retVal = 0;
    int32_t     errCode;

    /***********************************************
     * ADC sanity (16-bit complex only)
     ***********************************************/
    if (openCfg->adcOutCfg.fmt.b2AdcBits != 2U)
        { retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_16BITS_ADC; goto exit; }
    if ((openCfg->adcOutCfg.fmt.b2AdcOutFmt != 1U) &&
        (openCfg->adcOutCfg.fmt.b2AdcOutFmt != 2U))
        { retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_COMPLEX_ADC_FORMAT; goto exit; }

    /***********************************************
     * RX channel map (your cfg uses RX0 only)
     ***********************************************/
    numRxAntennas = 0U;
    for (channel = 0U; channel < 1U /* SYS_COMMON_NUM_RX_CHANNEL if needed */; channel++)
    {
        if (openCfg->chCfg.rxChannelEn & (1U << channel))
        {
            outParams->rxAntOrder[numRxAntennas] = channel;
            numRxAntennas++;
        }
        else
        {
            outParams->rxAntOrder[channel] = 0U;
        }
    }
    outParams->numRxAntennas = numRxAntennas;

    /***********************************************
     * Frame fields (from cfg)
     ***********************************************/
    frameChirpStartIdx  = MmwDemo_RFParser_getChirpStartIdx(ctrlCfg, subFrameIdx);  /* 0 */
    frameChirpEndIdx    = MmwDemo_RFParser_getChirpEndIdx  (ctrlCfg, subFrameIdx);  /* 0 */
    numLoops            = MmwDemo_RFParser_getNumLoops     (ctrlCfg, subFrameIdx);  /* 2 */
    outParams->framePeriod = MmwDemo_RFParser_getFramePeriod_ms(ctrlCfg, subFrameIdx);
    frameTotalChirps    = (int16_t)(frameChirpEndIdx - frameChirpStartIdx + 1);      /* 1 */

    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        if (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIdx].numOfBurst != 1U)
        { retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_ONE_NUMOFBURST_FOR_ADVANCED_FRAME; goto exit; }
    }
    if (frameTotalChirps > 32)
    { retVal = MMWDEMO_RFPARSER_ENOIMPL__NUM_UNIQUE_CHIRPS_MORE_THAN_32; goto exit; }

    /***********************************************
     * Find the (single) valid profile & chirp
     ***********************************************/
    for (profileLoopIdx = 0;
         (profileLoopIdx < MMWAVE_MAX_PROFILE) && (foundValidProfile == false);
         profileLoopIdx++)
    {
        MMWave_ProfileHandle profileHandle = MmwDemo_RFParser_getProfileHandle(ctrlCfg, profileLoopIdx);
        if (profileHandle == NULL) continue;

        uint32_t mmWaveNumChirps = 0;
        uint16_t validChirpTxEnBits[32] = {0};

        retVal = MMWave_getNumChirps(profileHandle, &mmWaveNumChirps, &errCode);
        if (retVal != 0) { retVal = errCode; goto exit; }

        /* Mark chirps that fall inside [frameChirpStartIdx .. frameChirpEndIdx] and that enable a TX in channelTxEn */
        for (chirpLoopIdx = 1; chirpLoopIdx <= (int32_t)mmWaveNumChirps; chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            rlChirpCfg_t chirpCfg;

            retVal = MMWave_getChirpHandle(profileHandle, chirpLoopIdx, &chirpHandle, &errCode);
            if (retVal != 0) { retVal = errCode; goto exit; }

            retVal = MMWave_getChirpCfg(chirpHandle, &chirpCfg, &errCode);
            if (retVal != 0) { retVal = errCode; goto exit; }

            if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                (chirpCfg.chirpEndIdx   <= frameChirpEndIdx)   &&
                ((chirpCfg.txEnable & channelTxEn) != 0U))
            {
                uint16_t idx;
                for (idx = (chirpCfg.chirpStartIdx - frameChirpStartIdx);
                     idx <= (chirpCfg.chirpEndIdx   - frameChirpStartIdx);
                     idx++)
                {
                    validChirpTxEnBits[idx] = chirpCfg.txEnable;
                    foundValidProfile = true;
                }
            }
        }

        /* AoA related (still needed even with 1TX/1RX to fill orders) */
        retVal = MmwDemo_RFParser_setAoAParams(outParams,
                                               &foundValidProfile,
                                               frameTotalChirps,
                                               validChirpTxEnBits,
                                               bpmEnabled);
        if (retVal != 0) { goto exit; }

        if (foundValidProfile)
        {

            retVal = MMWave_getProfileCfg(profileHandle, &profileCfg, &errCode);
            if (retVal != 0) { retVal = errCode; goto exit; }

            /* -------- Values read DIRECTLY from your cfg --------
               profileCfg.numAdcSamples     = 128
               profileCfg.digOutSampleRate  = 2500  (ksps)
               profileCfg.freqSlopeConst    = -70    (MHz/us units, via mmwavelink scaling)
               profileCfg.adcStartTimeConst = 1     (10 ns ticks)
               profileCfg.idleTimeConst     = 5     (10 ns ticks)
               profileCfg.rampEndTime       = 55.14 (us -> internally coded; we convert generically)
               profileCfg.startFreqConst    = 60 GHz (Q26)
            ------------------------------------------------------ */
            profileCfg.numAdcSamples     = 128;
            profileCfg.digOutSampleRate  = 2500;
            profileCfg.freqSlopeConst    = -70;
            profileCfg.adcStartTimeConst = 1;
            profileCfg.idleTimeConst     = 5;
            profileCfg.rampEndTime       = 55.14;
            profileCfg.startFreqConst    = 60;

            /* --- Hard override for derived computations ONLY (does NOT program RF) --- */
            const float startFreq_GHz_cfg   = 63.95f;
            const float slope_MHzPerUs_cfg  = -70.0f;

            profileCfg.numAdcSamples        = 128;
            profileCfg.digOutSampleRate     = 2500;   /* ksps */
            profileCfg.adcStartTimeConst    = 1;      /* 10 ns ticks */
            profileCfg.idleTimeConst        = 5;      /* 10 ns ticks */
            profileCfg.rampEndTime          = 55.14f; /* us */
            profileCfg.freqSlopeConst       =-70;
            /* Use your direct units for the math */
            const float Fs_Hz          = profileCfg.digOutSampleRate * 1e3f;
            const float T_s            = (float)profileCfg.numAdcSamples / Fs_Hz;
            const float t_adc_start_s  = profileCfg.adcStartTimeConst * 10.0e-9f;
            const float slope_HzPerSec = slope_MHzPerUs_cfg * 1e12f;     /* MHz/us -> Hz/s */
            const float f0_Hz          = startFreq_GHz_cfg * 1e9f;

            const float f_start_samp_Hz = f0_Hz + slope_HzPerSec * t_adc_start_s;
            const float f_end_samp_Hz   = f_start_samp_Hz + slope_HzPerSec * T_s;

            outParams->centerFreq      = 0.5f * (f_start_samp_Hz + f_end_samp_Hz);
            outParams->chirpInterval   = (profileCfg.idleTimeConst * 10.0e-9f) + (profileCfg.rampEndTime * 1e-6f);

            const float absSlope_HzPerSec = fabsf(slope_HzPerSec);
            outParams->numRangeBins    = mathUtils_pow2roundup(profileCfg.numAdcSamples);
            outParams->rangeStep       = (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * Fs_Hz)
                                         / (2.0f * absSlope_HzPerSec * (float)outParams->numRangeBins);
            outParams->rangeResolution = (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * Fs_Hz)
                                         / (2.0f * absSlope_HzPerSec * (float)profileCfg.numAdcSamples);

            /* Basic dimensions */
            outParams->numAdcSamples   = 128;//profileCfg.numAdcSamples;
            outParams->numRangeBins    = 128;//mathUtils_pow2roundup(outParams->numAdcSamples);

            /* after computing frameTotalChirps and numLoops */
            outParams->numChirpsPerFrame = frameTotalChirps * numLoops;

            /* Single-chirp path: no Doppler */
            if (outParams->numChirpsPerFrame == 1U) {
                outParams->numDopplerChirps = 1U;
                outParams->numDopplerBins   = 1U;
            } else {
                outParams->numDopplerChirps = outParams->numChirpsPerFrame / outParams->numTxAntennas;
                outParams->numDopplerBins   = mathUtils_pow2roundup(outParams->numDopplerChirps);
            }

#ifdef MMWDEMO_RFPARSER_DBG
            System_printf("RX=%u, TX=%u, numLoops=%u\n",
                          outParams->numRxAntennas, outParams->numTxAntennas, numLoops);
            System_printf("Fs=%.0f Hz, Ns=%u, Tadc=%.3fus\n", Fs_Hz, outParams->numAdcSamples, T_s*1e6f);
            System_printf("f0=%.3f GHz, slope=%.3f MHz/us, BW=%.3f GHz\n",
                          f_start_ramp_Hz*1e-9f, slope_HzPerUs*1e-6f, bandwidth_Hz*1e-9f);
            System_printf("fc=%.3f GHz, Tchirp=%.3fus\n",
                          centerFreq_Hz*1e-9f, outParams->chirpInterval*1e6f);
            System_printf("ΔR=%.3f m, Rstep=%.3f m, Δv=%.3f m/s\n",
                          outParams->rangeResolution, outParams->rangeStep, outParams->dopplerResolution);
#endif
        }
    }

    if (foundValidProfile == false)
    {
        retVal = MMWDEMO_RFPARSER_EINVAL__VALID_PROFILECFG_NOT_FOUND;
        goto exit;
    }

exit:
    return retVal;
}



/**
 *  @b Description
 *  @n
 *      Helper function that parses ADCBuf configuration to be used to configure ADCBuf driver
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  adcBufCfg            Pointer to ADCBuf configuration
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        < 0, one of @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
static int32_t MmwDemo_RFParser_parseADCBufCfg
(
    MmwDemo_RFParserOutParams   *outParams,
    MmwDemo_ADCBufCfg     *adcBufCfg
)
{
    uint16_t            numBytePerSample = 0;
    uint32_t            chirpThreshold;
    uint32_t            maxChirpThreshold;
    uint32_t            bytesPerChirp;
    int32_t             retVal = 0;

    /* Only support Complex */
    if (adcBufCfg->adcFmt != 0)
    {
        retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NONCOMPLEX_ADC_FORMAT;
        goto exit;
    }

    /* Complex dataFormat has 4 bytes */
    numBytePerSample = 4U;

    /* Calculate RX channel data size and make it align to 16 bytes */
    outParams->adcBufChanDataSize =  outParams->numAdcSamples * numBytePerSample;
    outParams->adcBufChanDataSize = (outParams->adcBufChanDataSize + 15U) / 16U * 16U;

    /* Calculate max possible chirp threshold */
    bytesPerChirp = outParams->adcBufChanDataSize * outParams->numRxAntennas;

    /* find maximum number of full chirps that can fit in the ADCBUF memory, while
       also being able to divide numChirpsPerFrame, we do not want remainder processing */
    maxChirpThreshold = MmwDemo_RFParserHwCfg.adcBufSize/ bytesPerChirp;
    if (maxChirpThreshold >= outParams->numChirpsPerFrame)
    {
        maxChirpThreshold = outParams->numChirpsPerFrame;
        if (maxChirpThreshold > SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD)
        {
            /* If CQ monitor is enabled, then check max chirpthreshold */
            maxChirpThreshold = SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD;
        }
    }
    else
    {
        /* Find largest divisor of numChirpsPerFrame no bigger than maxChirpThreshold */
        while (outParams->numChirpsPerFrame % maxChirpThreshold)
        {
            maxChirpThreshold--;
        }
    }

    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = adcBufCfg->chirpThreshold;

    /* if automatic, set to the calculated max */
    if (chirpThreshold == 0)
    {
        chirpThreshold = maxChirpThreshold;
    }
    else
    {
        if (chirpThreshold > maxChirpThreshold)
        {
#ifdef MMWDEMO_RFPARSER_DBG
            System_printf("Desired chirpThreshold %d higher than max possible of %d, setting to max\n",
                chirpThreshold, maxChirpThreshold);
#endif
            retVal = MMWDEMO_RFPARSER_EINVAL__CHIRP_THRESH_GREATER_THAN_MAX_ALLOWED;
            goto exit;
        }
        else
        {
            /* check for divisibility of the user provided threshold */
            if((outParams->numChirpsPerFrame % chirpThreshold) != 0)
            {
                retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_DIVISIBILITY_OF_CHIRP_THRESH;
                goto exit;
            }
            else
            {
                /* Chirp threshold has a valid configuration */
            }
        }
    }

#ifdef MMWDEMO_RFPARSER_DBG
    System_printf(" chirpThreshold %d max = %d, \n",
                  chirpThreshold, maxChirpThreshold);
#endif

    /* Save chirpThreshold */
    outParams->numChirpsPerChirpEvent = chirpThreshold;

exit:
    return (retVal);
}

/**************************************************************
 * Exposed APIs
 **************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to parse chirp/profile configurations and 
 *  save configuration and derived parameter in datapath parameter structure.
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  subFrameIdx    Sub-frame index
 *  @param[in]  openCfg        Pointer to the MMWave Open configuration
 *  @param[in]  ctrlCfg        Pointer to the MMWave Control configuration
 *  @param[in]  adcBufCfg      Pointer to ADCBuf configuration
 *  @param[in]  rfFreqScaleFactor RF frequency scale factor, see SOC_getDeviceRFFreqScaleFactor API
 *  @param[in]  bpmEnable      BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup MMWDEMO_RFPARSER_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - = 0
 *  @retval
 *      Error       - < 0, one of @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
int32_t MmwDemo_RFParser_parseConfig
(
    MmwDemo_RFParserOutParams    *outParams,
    uint8_t              subFrameIdx,
    MMWave_OpenCfg       *openCfg,
    MMWave_CtrlCfg       *ctrlCfg,
    MmwDemo_ADCBufCfg    *adcBufCfg,
    float                rfFreqScaleFactor,
    bool                 bpmEnable
)
{
    int32_t retVal;

    if(subFrameIdx < RL_MAX_SUBFRAMES)
    {
        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        retVal = MmwDemo_RFParser_parseCtrlConfig(outParams, subFrameIdx,
                                                  openCfg, ctrlCfg, rfFreqScaleFactor, bpmEnable);

        if (retVal != 0)
        {
            goto exit;
        }
        retVal = MmwDemo_RFParser_parseADCBufCfg(outParams, adcBufCfg);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    else
    {
        retVal = MMWDEMO_RFPARSER_EINVAL__NUM_SUBFRAMES;
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get number of sub-frames from the input
 *      chirp/profile configuration.
 *
 *  @param[in]  ctrlCfg        Pointer to the MMWave Control configuration
 *
 *  \ingroup MMWDEMO_RFPARSER_EXTERNAL_FUNCTION
 *
 *  @retval  Number of Sub-frames
 */
uint8_t MmwDemo_RFParser_getNumSubFrames(MMWave_CtrlCfg *ctrlCfg)
{
    if(ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames);
    }
    else
    {
        return (1);
    }
}

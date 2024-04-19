/* Dynamics Processor (Gate, Compressor & Limiter)
 * Copyright (c) 2023, Marc Paquette (https://github.com/MarkzP)
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.	 Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#if !defined(KINETISL)

#include "effect_dynamics_F32.h"
#include <Arduino.h>


AudioEffectDynamics_F32::AudioEffectDynamics_F32()
	: AudioStream_F32(2, inputQueueArray)
{
	sample_rate_Hz = AUDIO_SAMPLE_RATE_EXACT;
	init();
}


AudioEffectDynamics_F32::AudioEffectDynamics_F32(const AudioSettings_F32 &settings)
	: AudioStream_F32(2, inputQueueArray)
{
	sample_rate_Hz = settings.sample_rate_Hz;
	init();
}


void AudioEffectDynamics_F32::detector(DetectorTypes detectorType, float time, float voltageDrop)
{
	aDetectorDecay = timeToAlpha(time);
	aVoltageDrop = constrain(fabsf(voltageDrop), 0.0f, 0.2f);
	if (detectorType != aDetector) aRMSLevel = 0.0;
	aDetector = detectorType;  
}


void AudioEffectDynamics_F32::gate(float threshold, float attack, float release, float hysterisis, float attenuation, bool enable)
{
	if (enable)
	{
		hysterisis = fabsf(hysterisis);
		aGateThresholdOpen = db2ln(threshold + (hysterisis * 0.5f));
		aGateThresholdClose = db2ln(threshold - (hysterisis * 0.5f));
		aGateAttenuation = db2ln(attenuation);
		aGateAttack = timeToAlpha(attack);
		aGateRelease = timeToAlpha(release);

		if (!aGateEnabled) aGateln = 0.0f;
	}
	
	aGateEnabled = enable;
}


void AudioEffectDynamics_F32::compression(float threshold, float attack, float release, float ratio, float kneeWidth, float wet, bool enable)
{
	if (enable)
	{
		aCompThreshold = db2ln(threshold);
		if (ratio < 0.5f) ratio = 0.5f;
		aCompRatio = 1.0f / ratio;

		float compKneeWidth = db2ln(fabsf(kneeWidth), 0.0f, 126.0f);
		aCompAttack = timeToAlpha(attack);
		aCompRelease = timeToAlpha(release);
		if (compKneeWidth > 0.0f)
		{
			aCompHalfKneeWidth = compKneeWidth * 0.5f;
			aCompTwoKneeWidth = 0.5f / compKneeWidth;
			aCompKneeRatio = aCompRatio - 1.0f;
			aCompLowKnee = aCompThreshold - aCompHalfKneeWidth;
			aCompHighKnee = aCompThreshold + aCompHalfKneeWidth;
		}
		else
		{
			aCompLowKnee = aCompThreshold;
			aCompHighKnee = aCompThreshold;
		}
		
		if (wet < 0.0f) wet = 0.0f;
		else if (wet > 1.0f) wet = 1.0f;
		aCompWet = wet;
	
		if (!aCompEnabled) aCompln = 0.0f;
	}
	
	aCompEnabled = enable;
	computeMakeupGain();
}


void AudioEffectDynamics_F32::limit(float threshold, float attack, float release, bool enable)
{
	if (enable)
	{
		aLimitThreshold = db2ln(threshold);
		aLimitAttack = timeToAlpha(attack);
		aLimitRelease = timeToAlpha(release);

		if (!aLimitEnabled) aLimitln = 0.0f;
	}
	
	aLimitEnabled = enable;
}


void AudioEffectDynamics_F32::autoMakeupGain(float headroom, bool enable)
{
	mgHeadroom = -db2ln(headroom, -60.0f, 60.0f);
	mgAutoEnabled = enable;
	computeMakeupGain();
}


void AudioEffectDynamics_F32::makeupGain(float gain)
{
	mgManual = db2ln(gain, -60.0f, 60.0f);
	computeMakeupGain();
}


void AudioEffectDynamics_F32::init()
{
	detector();
	gate();
	compression();
	limit();
	autoMakeupGain();
	makeupGain();
}


// ***********************************************************************************************/
// derived from https://github.com/romeric/fastapprox
inline float AudioEffectDynamics_F32::unit2ln(float u)
{
	union { float f; uint32_t i; } vx = { u };
	union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
	float y = vx.i;
	y *= 1.1920928955078125e-7f;
	return y - 124.22551499f - 1.498030302f * mx.f - 1.72587999f / (0.3520887068f + mx.f);
}


inline float AudioEffectDynamics_F32::ln2unit(float ln)
{
	float offset = (ln < 0.0f) ? 1.0f : 0.0f;
	float clipp = (ln < -126.0f) ? -126.0f : ln;
	int w = clipp;
	float z = clipp - w + offset;
	union { uint32_t i; float f; } v = { uint32_t ( (1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z) ) };
	return v.f;
}

float AudioEffectDynamics_F32::db2ln(float db, float min, float max)
{
	if (db < min) db = min;
	else if (db > max) db = max;
	return db * EFFECT_DYNAMICS_DB_LN;
}

// ***********************************************************************************************/


float AudioEffectDynamics_F32::timeToAlpha(float time)
{
	if (time <= 0.0f) return 1.0f;
	if (time > 10.0f) time = 10.0f;
	return 1.0f - expf(-3.1699f / (sample_rate_Hz * time));
}


void AudioEffectDynamics_F32::computeMakeupGain()
{
	aMakeupln = mgManual + (mgAutoEnabled && aCompEnabled ? 0.5f * aCompWet * (-aCompThreshold + (aCompThreshold * aCompRatio) + mgHeadroom) : 0.0f);
}


void AudioEffectDynamics_F32::update(void)
{
	audio_block_f32_t *block;
	audio_block_f32_t *sidechain;
	audio_block_f32_t *detectorOut;
	
	float samp, inputln, attln, finalln, knee, outln;
	double rms;

	block = AudioStream_F32::receiveWritable_f32(0);

	if (!block) return;

	sidechain = AudioStream_F32::receiveReadOnly_f32(1);
	if (!sidechain) sidechain = block;
	
	detectorOut = AudioStream_F32::allocate_f32();

	for (int i = 0; i < block->length; i++)
	{
		samp = sidechain->data[i];
		
		// Track & remove dc offset
		aDcOffset += (samp - aDcOffset) * aDcOffsetAlpha;
		samp -= aDcOffset;

		if (aDetector == DetectorType_RMS)
		{
			// Square sample before smoothing
			rms = samp * samp;
			aRMSLevel += (rms - aRMSLevel) * (rms > aRMSLevel ? 1.0 : (double)aDetectorDecay);
			// Un-square
			aDetectorLevel = (float)sqrt(aRMSLevel);
		}
		else
		{
			// Rectify if detector is DiodeBridge
			if (samp < 0.0f) samp = (aDetector == DetectorType_HalfWave ? 0.0f : -samp);
		
			// Account for diode voltage drop				
			samp -= aVoltageDrop;
			if (samp < 0.0f) samp = 0.0f;
			
			aDetectorLevel += (samp - aDetectorLevel) * (samp > aDetectorLevel ? 1.0f : aDetectorDecay);
		}
		
		if (detectorOut) detectorOut->data[i] = aDetectorLevel;
		inputln = unit2ln(aDetectorLevel);
		finalln = aMakeupln;

		//Gate
		if (aGateEnabled)
		{
			if (inputln > aGateThresholdOpen) aGateln -= aGateln * aGateAttack;
			if (inputln < aGateThresholdClose) aGateln += (aGateAttenuation - aGateln) * aGateRelease;

			finalln += aGateln;
		}

		//Compressor
		if (aCompEnabled)
		{		
			attln = 0.0f; //Below knee
			if (inputln >= aCompLowKnee)
			{
				if (inputln < aCompHighKnee)
				{
					//Knee transition
					knee = inputln - aCompLowKnee;
					attln = aCompKneeRatio * knee * knee * aCompTwoKneeWidth;
				}
				else
				{
					//Above knee
					attln = aCompThreshold + ((inputln - aCompThreshold) * aCompRatio) - inputln;
				}
			}
			
			aCompln += (attln - aCompln) * (attln < aCompln ? aCompAttack : aCompRelease);

			finalln += (aCompln * aCompWet);
		}

		//Brickwall Limiter
		if (aLimitEnabled)
		{
			outln = inputln + finalln;
			attln = (outln > aLimitThreshold ? aLimitThreshold - outln : 0.0f);
			aLimitln += (attln - aLimitln) * (attln < aLimitln ? aLimitAttack : aLimitRelease);

			finalln += aLimitln;
		}

		//Compute linear gain
		block->data[i] *= ln2unit(finalln);

		aFinalln = finalln;
	}

	//Transmit & release
	AudioStream_F32::transmit(block, 0);
	AudioStream_F32::release(block);

	if (detectorOut)
	{
		AudioStream_F32::transmit(detectorOut, 1);
		AudioStream_F32::release(detectorOut);
	}

	if (sidechain != block) AudioStream_F32::release(sidechain);
}

#endif
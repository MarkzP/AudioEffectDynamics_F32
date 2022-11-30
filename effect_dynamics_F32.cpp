/* Dynamics Processor (Gate, Compressor & Limiter)
 * Copyright (c) 2022, Marc Paquette (https://github.com/MarkzP)
 * RMS detection by Nic Newdigate (https://github.com/newdigate)
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
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
	if (detectorType == DetectorType_RMS)
	{
		unsigned rmsSampleSize = (unsigned int)(sample_rate_Hz * fabsf(time)) + 1;
		if (rmsSampleSize > aRmsSampleSize) {
			free(apRmsSamplesSquared);
			apRmsSamplesSquared = (double*)malloc(rmsSampleSize * sizeof(double));
			if (apRmsSamplesSquared == nullptr) return;
		}

		aRmsSampleSize = rmsSampleSize;
		aRmsSquaresSum = 0.0;
		for (unsigned int i = 0; i < aRmsSampleSize; i++) apRmsSamplesSquared[i] = 0.0;
		aRmsOneOverSampleSize = 1.0 / (double)aRmsSampleSize;
		aSquareIndex = 0;
	}
	else
	{
		free(apRmsSamplesSquared);
		apRmsSamplesSquared = nullptr;
		aRmsSampleSize = 0;
		aDetectorLevel = 0.0f;
		aDetectorDecay = timeToAlpha(time);
		aVoltageDrop = fabsf(voltageDrop);
	}
	
	aDetector = detectorType;
}


void AudioEffectDynamics_F32::gate(float threshold, float attack, float release, float hysterisis, float attenuation)
{
	threshold = constrain(-fabsf(threshold), EFFECT_DYNAMICS_MIN_DB, EFFECT_DYNAMICS_MAX_DB);
	attenuation = constrain(-fabsf(attenuation), EFFECT_DYNAMICS_MIN_DB, EFFECT_DYNAMICS_MAX_DB);
	hysterisis = constrain(fabsf(hysterisis), 0.0f, 12.0f) / 2.0f;

	if (threshold > EFFECT_DYNAMICS_MIN_DB && attenuation < EFFECT_DYNAMICS_MAX_DB) {

		aGateThresholdOpen = threshold + hysterisis;
		aGateThresholdClose = threshold - hysterisis;
		aGateAttenuation = attenuation;
		aGateAttack = timeToAlpha(attack);
		aGateRelease = timeToAlpha(release);

		if (!aGateEnabled) aGatedb = 0.0f;

		aGateEnabled = true;
	}
	else
	{
		aGateEnabled = false;
	}
}


void AudioEffectDynamics_F32::compression(float threshold, float attack, float release, float ratio, float kneeWidth)
{
	threshold = constrain(-fabsf(threshold), EFFECT_DYNAMICS_MIN_DB, EFFECT_DYNAMICS_MAX_DB);

	if (threshold < EFFECT_DYNAMICS_MAX_DB) {

		aCompThreshold = threshold;
		aCompRatio = 1.0f / constrain(abs(ratio), 1.0f, 60.0f);

		float compKneeWidth = constrain(abs(kneeWidth), 0.0f, 32.0f);
		aCompAttack = timeToAlpha(attack);
		aCompRelease = timeToAlpha(release);
		if (compKneeWidth > 0.0f)
		{
			aCompHalfKneeWidth = compKneeWidth / 2.0f;
			aCompTwoKneeWidth = 1.0f / (compKneeWidth * 2.0f);
			aCompKneeRatio = aCompRatio - 1.0f;
			aCompLowKnee = aCompThreshold - aCompHalfKneeWidth;
			aCompHighKnee = aCompThreshold + aCompHalfKneeWidth;
		}
		else
		{
			aCompLowKnee = aCompThreshold;
			aCompHighKnee = aCompThreshold;
		}

		if (!aCompEnabled) aCompdb = 0.0f;

		aCompEnabled = true;
	}
	else
	{
		aCompThreshold = EFFECT_DYNAMICS_MAX_DB;
		aCompRatio = 1.0f;
		aCompEnabled = false;
	}

	computeMakeupGain();
}


void AudioEffectDynamics_F32::limit(float threshold, float attack, float release)
{
	threshold = constrain(-fabsf(threshold), EFFECT_DYNAMICS_MIN_DB, EFFECT_DYNAMICS_MAX_DB);

	if (threshold < EFFECT_DYNAMICS_MAX_DB) {

		aLimitThreshold = threshold;
		aLimitAttack = timeToAlpha(attack);
		aLimitRelease = timeToAlpha(release);

		if (!aLimitEnabled) aLimitdb = 0.0f;

		aLimitEnabled = true;
	}
	else
	{
		aLimitEnabled = false;
		aLimitThreshold = EFFECT_DYNAMICS_MAX_DB;
	}

	computeMakeupGain();
}


void AudioEffectDynamics_F32::autoMakeupGain(float headroom)
{
	mgAutoEnabled = true;
	mgHeadroom = constrain(-fabsf(headroom), EFFECT_DYNAMICS_MIN_DB, EFFECT_DYNAMICS_MAX_DB);
	computeMakeupGain();
}


void AudioEffectDynamics_F32::makeupGain(float gain)
{
	mgAutoEnabled = false;
	aMakeupdb = constrain(gain, -24.0f, 24.0f);
}


void AudioEffectDynamics_F32::init()
{
	detector();
	gate();
	compression();
	limit();
	autoMakeupGain();	
}


// ***********************************************************************************************/
// from https://github.com/romeric/fastapprox
inline float fasterlog2(float x)
{
	union { float f; uint32_t i; } vx = { x };
	float y = vx.i;
	y *= 1.1920928955078125e-7f;
	return y - 126.94269504f;
}


inline float fasterexp2f (float p)
{
	float clipp = (p < EFFECT_DYNAMICS_MIN_DB) ? EFFECT_DYNAMICS_MIN_DB : p;
	union { uint32_t i; float f; } v = { uint32_t( (1 << 23) * (clipp + 126.94269504f) ) };
	return v.f;
}
// ***********************************************************************************************/


inline float AudioEffectDynamics_F32::unit2db(float u)
{
	if (u < 5.001554864521944e-7f) return EFFECT_DYNAMICS_MIN_DB;
	return 6.02f * fasterlog2(u);
}


inline float AudioEffectDynamics_F32::db2unit(float db)
{
	return fasterexp2f(0.16612f * db);
}


float AudioEffectDynamics_F32::timeToAlpha(float time)
{
	if (time <= 0.0f) return 1.0f;
	if (time > 10.0f) time = 10.0f;
	return 1.0f - expf(-3.1699f / (sample_rate_Hz * time));
}


void AudioEffectDynamics_F32::computeMakeupGain()
{
	if (mgAutoEnabled)
	{
		aMakeupdb = -aCompThreshold + (aCompThreshold * aCompRatio) + mgHeadroom;
	}
}


void AudioEffectDynamics_F32::update(void)
{
	audio_block_f32_t *block;
	audio_block_f32_t *sidechain;

	block = AudioStream_F32::receiveWritable_f32(0);

	if (!block) return;

	sidechain = AudioStream_F32::receiveReadOnly_f32(1);
	if (!sidechain) sidechain = block;

	for (int i = 0; i < block->length; i++)
	{
		float samp = sidechain->data[i];
		
		switch (aDetector)
		{
			case DetectorType_DiodeBridge:
				if (samp < 0.0f) samp = -samp;
				// Fall through to DetectorType_HalfWave...				
			case DetectorType_HalfWave:
				// Account for diode voltage drop				
				samp -= aVoltageDrop;
				if (samp < 0.0f) samp = 0.0f;
				
				// Convert peak to approximate equivalent RMS level
				samp *= 0.707f;
				
				aDetectorLevel += (samp - aDetectorLevel) * (samp > aDetectorLevel ? 1.0f : aDetectorDecay);
				break;
				
			case DetectorType_RMS:
				double squareSamp = (double)samp;
				squareSamp *= squareSamp;
				// Replace sample in window
				aRmsSquaresSum -= apRmsSamplesSquared[aSquareIndex];
				aRmsSquaresSum += squareSamp;
				apRmsSamplesSquared[aSquareIndex] = squareSamp;
				if (++aSquareIndex >= aRmsSampleSize) aSquareIndex = 0;

				// Compute square root of squares
				aDetectorLevel = sqrtf(aRmsSquaresSum * aRmsOneOverSampleSize);
				break;
		}
		
		float inputdb = unit2db(aDetectorLevel);
		float finaldb = aMakeupdb;

		//Gate
		if (aGateEnabled)
		{
			if (inputdb > aGateThresholdOpen) aGatedb -= aGatedb * aGateAttack;
			if (inputdb < aGateThresholdClose) aGatedb += (aGateAttenuation - aGatedb) * aGateRelease;

			finaldb += aGatedb;
		}

		//Compressor
		if (aCompEnabled)
		{		
			float attdb = 0.0f; //Below knee
			if (inputdb >= aCompLowKnee)
			{
				if (inputdb < aCompHighKnee)
				{
					//Knee transition
					float knee = inputdb - aCompLowKnee;
					attdb = aCompKneeRatio * knee * knee * aCompTwoKneeWidth;
				}
				else
				{
					//Above knee
					attdb = aCompThreshold + ((inputdb - aCompThreshold) * aCompRatio) - inputdb;
				}
			}
			
			aCompdb += (attdb - aCompdb) * (attdb < aCompdb ? aCompAttack : aCompRelease);

			finaldb += aCompdb;
		}

		//Brickwall Limiter
		if (aLimitEnabled)
		{
			float outdb = inputdb + finaldb;
			aLimitdb += (aLimitThreshold - outdb - aLimitdb) * (outdb > aLimitThreshold ? aLimitAttack : aLimitRelease);

			finaldb += aLimitdb;
		}

		//Compute linear gain
		block->data[i] *= db2unit(finaldb);
		
		aDetectordb = inputdb;
		aCurrentdb = finaldb;
	}

	//Transmit & release
	AudioStream_F32::transmit(block);
	if (sidechain != block) AudioStream_F32::release(sidechain);
	AudioStream_F32::release(block);
}

#endif
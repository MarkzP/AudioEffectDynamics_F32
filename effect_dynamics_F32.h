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

#ifndef effect_dynamics_f32_h_
#define effect_dynamics_f32_h_

#if !defined(KINETISL)

#include "OpenAudio_ArduinoLibrary.h"
#include "AudioStream_F32.h"

#define EFFECT_DYNAMICS_MIN_DB	(-126.0f)	 //21 bits effictive, limited by approximations
#define EFFECT_DYNAMICS_MAX_DB	(0.0f)

#define EFFECT_DYNAMICS_DB_LN	(0.1660964f)

class AudioEffectDynamics_F32 : public AudioStream_F32
{
//GUI: inputs:2, outputs:2	//this line used for automatic generation of GUI node
//GUI: shortName:effect_Dynamics	
public:
	typedef enum
	{
		DetectorType_DiodeBridge,		// Full wave peak rectification with optional filtering & diode vf drop
		DetectorType_HalfWave,			// Single diode peak rectification with optional filtering & diode vf drop
		DetectorType_RMS
	} DetectorTypes;

	AudioEffectDynamics_F32();
	AudioEffectDynamics_F32(const AudioSettings_F32 &settings);
	
	// Sets the detector characteristics to mimic real life applications
	// time is in seconds
	// voltage drop is in unit level
	void detector(DetectorTypes detectorType = DetectorType_DiodeBridge, float time = 0.02f, float voltageDrop = 0.0f);

	//Sets the gate parameters.
	//threshold is in dbFS
	//attack & release are in seconds
	void gate(float threshold = EFFECT_DYNAMICS_MIN_DB, float attack = 0.001f, float release = 1.0f, float hysterisis = 6.0f, float attenuation = -12.0f, bool enable = false);

	//Sets the compression parameters.
	//threshold & kneeWidth are in db(FS)
	//attack and release are in seconds
	//ratio is expressed as x:1 i.e. 1 for no compression, 60 for brickwall limiting
	//Set kneeWidth to 0 for hard knee, 12 for a very soft knee
	//Wet is used for parallel compression
	void compression(float threshold = -45.0f, float attack = 0.01f, float release = 2.5f, float ratio = 2.0f, float kneeWidth = 6.0f, float wet = 1.0f, bool enable = false);

	//Sets the hard limiter parameters
	//threshold is in dbFS
	//attack & release are in seconds
	void limit(float threshold = EFFECT_DYNAMICS_MAX_DB, float attack = 0.001f, float release = 0.1f, bool enable = false);

	//Enables automatic makeup gain setting
	//headroom is in dbFS
	void autoMakeupGain(float headroom = 3.0f, bool enable = true);

	//Sets a fixed makeup gain value.
	//gain is in dbFS
	void makeupGain(float gain = 0.0f);
  
  float effectiveGain() { return aFinalln * (1.0f / EFFECT_DYNAMICS_DB_LN); }

protected:
	void init();
	float unit2ln(float u);
	float ln2unit(float ln);
	float db2ln(float db, float min = EFFECT_DYNAMICS_MIN_DB, float max = EFFECT_DYNAMICS_MAX_DB);
	float timeToAlpha(float time);
	void computeMakeupGain();

private:
	audio_block_f32_t* inputQueueArray[2];
	
	static constexpr float aMinln = EFFECT_DYNAMICS_MIN_DB * EFFECT_DYNAMICS_DB_LN;
	
	float sample_rate_Hz;
	DetectorTypes aDetector;
	double aRMSLevel;
	float aDetectorLevel;
	float aDetectorDecay;
	float aDcOffset = 0.0f;
	static constexpr float aDcOffsetAlpha = 0.0001f;
	float aVoltageDrop;

	bool aGateEnabled = false;
	float aGateThresholdOpen;
	float aGateThresholdClose;
	float aGateAttack;
	float aGateRelease;
	float aGateAttenuation;
	float aGateln = 0.0f;

	bool aCompEnabled = false;
	float aCompThreshold;
	float aCompAttack;
	float aCompRelease;
	float aCompRatio;
	float aCompHalfKneeWidth;
	float aCompTwoKneeWidth;
	float aCompKneeRatio;
	float aCompLowKnee;
	float aCompHighKnee;
	float aCompWet;
	float aCompln = 0.0f;

	bool aLimitEnabled = false;
	float aLimitThreshold;
	float aLimitAttack;
	float aLimitRelease;
	float aLimitln = 0.0f;

	bool mgAutoEnabled = false;
	float mgHeadroom;
	float mgManual;
	float aMakeupln;

	float aFinalln;

	virtual void update(void);
};
#endif

#endif

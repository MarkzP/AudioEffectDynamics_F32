# AudioEffectDynamics_F32
Dynamics Processor (Gate, Compressor &amp; Limiter) for the Teensy OpenAudio Library https://github.com/chipaudette/OpenAudio_ArduinoLibrary

This effect is lousely based on my recollection of the venerable dBX1086 compressor, minus the de-esser stuff.
Input 0 is the main signal input;
Input 1 is used for side-chaining into the detctor, if so desired. The signal from Input 0 will be used if nothing is connected there.


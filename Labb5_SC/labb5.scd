//Start server
s.boot;
//Stop server
s.quit;
//Start oscilloscope
s.scope;

//Server
(
SynthDef(\firstSynth, { arg inputFrequency, gate = 1;
    //Variables
    var detune = 1; // Detunes with 1Hz
    var pwm = SinOsc.ar(1).range(0,1); // Pulse width
    var lfo = SinOsc.ar(1); // LFO controller

    //Waveforms
    var sinWave = SinOsc.ar(329.63 * 2.21).range(0,5);
    var sinWave2 = SinOsc.ar(329.63 * sinWave);
    var sinMixed = (sinWave + sinWave2)/2;
    var triWave = LFTri.ar(220);
    var squareWave = LFPulse.ar(220,width:pwm);
    var sawWave = LFSaw.ar(inputFrequency);

    //Distortion
    var newSin = SinOsc.ar(220).clip(-0.5, 0.5);
    var newSin2 = SinOsc.ar(220).softclip;

    //Ringmodulation
    var ringM = triWave * lfo;

    //lab5
    var env = Env.adsr(
        attackTime: 0.01,
        decayTime: 0.3,
        sustainLevel: 0.5,
        releaseTime: 1.0,
        peakLevel: 1.0,
        curve: -4.0);

    var envelope = EnvGen.kr(env, gate: gate, doneAction: 0);

    //Outputs
    //Out.ar(0, {sinMixed}!2*0.5);
    //Out.ar(0, [sinMixed+detune, sinMixed-detune]);
    //Out.ar(0, [sinWave, sinWave2]);
    //Out.ar(0, {squareWave}!2);
    Out.ar(0, {sawWave}!2);

}).add;

FreqScope(400, 200, 0, server: s);
)

//Client
(
//twinkle little star
var notes = [72,72,79,79,81,81,79,77,77,76,76,74,74,72,];
var gate = 1;
//
~myFirstSynth = Synth.new(\firstSynth).register;

fork({
    loop({
        0.1.wait;
        for (0, notes.size-1, { arg index;
            ~myFirstSynth.set(\inputFrequency,  notes[index].midicps);
            0.15.wait;
            gate = 1 - gate;
            ~myFirstSynth.set(\gate, gate);
        });
    });
});
)
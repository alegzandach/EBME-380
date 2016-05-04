% list of frequencies used in MIDI
numTrials = 10000;
freqs = [8
9
9
10
10
11
12
12
13
14
15
15
16
16
16
16
16
16
16
16
16
16
16
16
16
17
18
19
21
22
23
25
26
28
29
31
33
35
37
39
41
44
46
49
52
55
58
62
65
69
73
78
82
87
93
98
104
110
117
123
131
139
147
156
165
175
185
196
208
220
233
247
262
277
294
311
330
349
370
392
415
440
466
494
523
554
587
622
659
698
740
784
831
880
932
988
1047
1109
1175
1245
1319
1397
1480
1568
1661
1760
1865
1976
2093
2217
2349
2489
2637
2794
2960
3136
3322
3520
3729
3951
4186
4435
4699
4978
5274
5588
5920
6272
6645
7040
7459
7902];

% setup mapping of arduino


NOTE_C_2 = 8;
NOTE_CS_2 = 9;
NOTE_D_2 = 9;
NOTE_DS_2 = 10;
NOTE_E_2 = 10;
NOTE_F_2 = 11;
NOTE_FS_2 = 12;
NOTE_G_2 = 12;
NOTE_GS_2 = 13;
NOTE_A_2 = 14;
NOTE_AS_2 = 15;
NOTE_B_2 = 15;
NOTE_C_1 = 16;
NOTE_CS_1 = 16;
NOTE_D_1 = 16;
NOTE_DS_1 = 16;
NOTE_E_1 = 16;
NOTE_F_1 = 16;
NOTE_FS_1 = 16;
NOTE_G_1 = 16;
NOTE_GS_1 = 16;
NOTE_A_1 = 16;
NOTE_AS_1 = 16;
NOTE_B_1 = 16;
NOTE_C0 = 16;
NOTE_CS0 = 17;
NOTE_D0 = 18;
NOTE_DS0 = 19;
NOTE_E0 = 21;
NOTE_F0 = 22;
NOTE_FS0 = 23;
NOTE_G0 = 25;
NOTE_GS0 = 26;
NOTE_A0 = 28;
NOTE_AS0 = 29;
NOTE_B0 = 31;
NOTE_C1 = 33;
NOTE_CS1 = 35;
NOTE_D1 = 37;
NOTE_DS1 = 39;
NOTE_E1 = 41;
NOTE_F1 = 44;
NOTE_FS1 = 46;
NOTE_G1 = 49;
NOTE_GS1 = 52;
NOTE_A1 = 55;
NOTE_AS1 = 58;
NOTE_B1 = 62;
NOTE_C2 = 65;
NOTE_CS2 = 69;
NOTE_D2 = 73;
NOTE_DS2 = 78;
NOTE_E2 = 82;
NOTE_F2 = 87;
NOTE_FS2 = 93;
NOTE_G2 = 98;
NOTE_GS2 = 104;
NOTE_A2 = 110;
NOTE_AS2 = 117;
NOTE_B2 = 123;
NOTE_C3 = 131;
NOTE_CS3 = 139;
NOTE_D3 = 147;
NOTE_DS3 = 156;
NOTE_E3 = 165;
NOTE_F3 = 175;
NOTE_FS3 = 185;
NOTE_G3 = 196;
NOTE_GS3 = 208;
NOTE_A3 = 220;
NOTE_AS3 = 233;
NOTE_B3 = 247;
NOTE_C4 = 262;
NOTE_CS4 = 277;
NOTE_D4 = 294;
NOTE_DS4 = 311;
NOTE_E4 = 330;
NOTE_F4 = 349;
NOTE_FS4 = 370;
NOTE_G4 = 392;
NOTE_GS4 = 415;
NOTE_A4 = 440;
NOTE_AS4 = 466;
NOTE_B4 = 494;
NOTE_C5 = 523;
NOTE_CS5 = 554;
NOTE_D5 = 587;
NOTE_DS5 = 622;
NOTE_E5 = 659;
NOTE_F5 = 698;
NOTE_FS5 = 740;
NOTE_G5 = 784;
NOTE_GS5 = 831;
NOTE_A5 = 880;
NOTE_AS5 = 932;
NOTE_B5 = 988;
NOTE_C6 = 1047;
NOTE_CS6 = 1109;
NOTE_D6 = 1175;
NOTE_DS6 = 1245;
NOTE_E6 = 1319;
NOTE_F6 = 1397;
NOTE_FS6 = 1480;
NOTE_G6 = 1568;
NOTE_GS6 = 1661;
NOTE_A6 = 1760;
NOTE_AS6 = 1865;
NOTE_B6 = 1976;
NOTE_C7 = 2093;
NOTE_CS7 = 2217;
NOTE_D7 = 2349;
NOTE_DS7 = 2489;
NOTE_E7 = 2637;
NOTE_F7 = 2794;
NOTE_FS7 = 2960;
NOTE_G7 = 3136;
NOTE_GS7 = 3322;
NOTE_A7 = 3520;
NOTE_AS7 = 3729;
NOTE_B7 = 3951;
NOTE_C8 = 4186;
NOTE_CS8 = 4435;
NOTE_D8 = 4699;
NOTE_DS8 = 4978;
NOTE_E8 = 5274;
NOTE_F8 = 5588;
NOTE_FS8 = 5920;
NOTE_G8 = 6272;
NOTE_GS8 = 6645;
NOTE_A8 = 7040;
NOTE_AS8 = 7459;
NOTE_B8 = 7902;

% set up mapping from arduino
newPitches(1) = NOTE_C_2 / 4;% dividing a frequency by 2 lowers a pitch by an octave; this pitch is C_4
newPitches(2) = NOTE_D_2 / 4;
newPitches(3) = NOTE_E_2 / 4;
newPitches(6) = NOTE_G_2 / 4;
newPitches(8) = NOTE_A_2 / 4;
newPitches(9) = NOTE_C_2 / 2; % C_3
newPitches(12) = NOTE_D_2 / 2;
newPitches(13) = NOTE_E_2 / 2;
newPitches(15) = NOTE_G_2 / 2;
newPitches(17) = NOTE_A_2 / 2;% lower pitches than usually handled in MIDI
newPitches(18) = NOTE_C_2;
newPitches(20) = NOTE_D_2;
newPitches(22) = NOTE_E_2;
newPitches(24) = NOTE_G_2;
newPitches(25) = NOTE_A_2;
newPitches(27) = NOTE_C_1;
newPitches(29) = NOTE_D_1;
newPitches(30) = NOTE_E_1;
newPitches(32) = NOTE_G_1;
newPitches(34) = NOTE_A_1;
newPitches(36) = NOTE_C0;
newPitches(37) = NOTE_D0;
newPitches(39) = NOTE_E0;
newPitches(41) = NOTE_G0;
newPitches(42) = NOTE_A0;
newPitches(44) = NOTE_C1;
newPitches(46) = NOTE_D1;
newPitches(48) = NOTE_E1;
newPitches(49) = NOTE_G1;
newPitches(51) = NOTE_A1;
newPitches(53) = NOTE_C2;
newPitches(54) = NOTE_D2;
newPitches(56) = NOTE_E2;
newPitches(58) = NOTE_G2;
newPitches(60) = NOTE_A2;
newPitches(61) = NOTE_C3;%Middle C is still middle C
newPitches(63) = NOTE_D3;
newPitches(65) = NOTE_E3;
newPitches(66) = NOTE_G3;
newPitches(68) = NOTE_A3;
newPitches(70) = NOTE_C4;
newPitches(72) = NOTE_D4;
newPitches(73) = NOTE_E4;
newPitches(75) = NOTE_G4;
newPitches(77) = NOTE_A4;
newPitches(78) = NOTE_C5;
newPitches(80) = NOTE_D5;
newPitches(82) = NOTE_E5;
newPitches(84) = NOTE_G5;
newPitches(85) = NOTE_A5;
newPitches(87) = NOTE_C6;
newPitches(89) = NOTE_D6;
newPitches(91) = NOTE_E6;
newPitches(92) = NOTE_G6;
newPitches(94) = NOTE_A6;
newPitches(96) = NOTE_C7;
newPitches(97) = NOTE_D7;
newPitches(99) = NOTE_E7;
newPitches(101) = NOTE_G7;
newPitches(102) = NOTE_A7;
newPitches(104) = NOTE_C8;
newPitches(106) = NOTE_D8;
newPitches(108) = NOTE_E8;
newPitches(109) = NOTE_G8;
newPitches(111) = NOTE_A8;
newPitches(113) = NOTE_C8 * 2;% higher than MIDI's usual range, doubling the frequency raises the note by an octave
newPitches(114) = NOTE_D8 * 2;
newPitches(116) = NOTE_E8 * 2;
newPitches(118) = NOTE_G8 * 2;
newPitches(120) = NOTE_A8 * 2;
newPitches(121) = NOTE_C8 * 4; % another octave higher
newPitches(123) = NOTE_D8 * 4;
newPitches(125) = NOTE_E8 * 4;
newPitches(126) = NOTE_G8 * 4;
newPitches(128) = NOTE_A8 * 4;

% black keys
newPitches(2) = NOTE_G_1;
newPitches(4) = NOTE_A_1;
newPitches(7) = NOTE_B_1;
newPitches(9) = NOTE_D_1;
newPitches(11) = NOTE_E_1;
newPitches(14) = NOTE_G_1;
newPitches(16) = NOTE_A_1;
newPitches(19) = NOTE_B_1;
newPitches(21) = NOTE_D0;
newPitches(23) = NOTE_E0;
newPitches(26) = NOTE_G0;
newPitches(28) = NOTE_A0;
newPitches(31) = NOTE_B0;
newPitches(33) = NOTE_D1;
newPitches(35) = NOTE_E1;
newPitches(38) = NOTE_G1;
newPitches(40) = NOTE_A1;
newPitches(43) = NOTE_B1;
newPitches(45) = NOTE_D2;
newPitches(47) = NOTE_E2;
newPitches(50) = NOTE_G2;
newPitches(52) = NOTE_A2;
newPitches(55) = NOTE_B2;
newPitches(57) = NOTE_D3;
newPitches(59) = NOTE_E3;
newPitches(62) = NOTE_G3; % Middle C#
newPitches(64) = NOTE_A3;
newPitches(67) = NOTE_B3;
newPitches(69) = NOTE_D4;
newPitches(71) = NOTE_E4;
newPitches(74) = NOTE_G4;
newPitches(76) = NOTE_A4;
newPitches(79) = NOTE_B4;
newPitches(81) = NOTE_D5;
newPitches(83) = NOTE_E5;
newPitches(86) = NOTE_G5;
newPitches(88) = NOTE_A5;
newPitches(91) = NOTE_B5;
newPitches(93) = NOTE_D6;
newPitches(95) = NOTE_E6;
newPitches(98) = NOTE_G6;
newPitches(100) = NOTE_A6;
newPitches(103) = NOTE_B6;
newPitches(105) = NOTE_D7;
newPitches(107) = NOTE_E7;
newPitches(110) = NOTE_G7;
newPitches(112) = NOTE_A7;
newPitches(115) = NOTE_B7;
newPitches(117) = NOTE_E8;
newPitches(119) = NOTE_D8;
newPitches(122) = NOTE_G8;
newPitches(124) = NOTE_A8;
newPitches(127) = NOTE_B8;

% use 1000 random notes to verify that each note is a valid frequency (is
% one of the notes in either of the used pentatonic scale

validFreqs = [NOTE_C3, NOTE_D3, NOTE_E3, NOTE_G3, NOTE_A3, NOTE_B3];
l = length(validFreqs);
errorCount = 0;
error = 3; % error to account for rounding frequencies to integers

for i = 1:numTrials
    rNote = randi(1,128);
    freq = newPitches(rNote);
    
    % compare to each valid note to one of the pitches.
    isMatch = false;
    for j = 1:l
        testF = validFreqs(j);
        
        for n = -8:8
            diff = abs(freq - testF^n);
            if(diff <= error)
                isMatch = true;
                break;
            end
        end
        if isMatch
            break;
        end
    end
    
    if ~isMatch
        errorCount = errorCount + 1;
        fprintf('ERROR: note %d freq %d\n', rNote, freq);
    end
end

fprintf('All done. There were %d errors.\n', errorCount);
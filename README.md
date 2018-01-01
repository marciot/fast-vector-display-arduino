# fast-vector-display-arduino
An extremely fast vector display for Arduino

My Arduino Nano wishes you a happy new year!

The circuit consists of just two 10uF capacitors and two 150 ohm resistors on pins 11 and 3. Many have suspected that the
Arduino does not have fast enough PWM for a low flicker-free vector display, but I wanted to show otherwise. The secret
sauce here is the software and the novel use of carefully timed pulses of rail-to-rail voltages to charge and discharge
the capacitors way faster than a PWM signal ever could.

The source code consists of:

1) A double-buffered frame buffer that allows the image to be repainted while the next frame is being generated.
2) An interrupt service routine that turns the pins on or off according to precise timing instructions stored in
the frame buffer.
3) An routine that computes the necessary voltage levels and delays to rapidly charge the capacitors to the
desired voltage.
4) A 2D graphics pipeline based on 3x3 matrices and affine transformations.
5) A scissoring algorithm to crop lines that fall outside of the display.

If you enjoy this project, please consider checking out my Patreon page (https://www.patreon.com/marciot).

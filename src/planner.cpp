// Code that converts linear absolute moves (degree/mm) into relative blocks of steps-per-motor, then
// inserts those blocks into the block ring buffer, and optimizes acceleration between the blocks.
#include "configure.h"
#include "lcd.h"
#include "motor.h"

extern Planner planner;

Segment Planner::blockBuffer[MAX_SEGMENTS];
volatile int Planner::block_buffer_head, 
             Planner::block_buffer_nonbusy,
             Planner::block_buffer_planned,
             Planner::block_buffer_tail;
int Planner::first_segment_delay;

float Planner::previous_nominal_speed_sqr;
float Planner::previous_safe_speed;
float Planner::previous_speed[NUM_MUSCLES];
float Planner::prev_unit_vec[NUM_AXIES];
int32_t Planner::position[NUM_MUSCLES];

#ifdef HAS_JUNCTION_DEVIATION
float Planner::junction_deviation = JUNCTION_DEVIATION_UNITS;
#endif

// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if(a < 0) a += (PI * 2.0);
  return a;
}

#if defined(S_CURVE_ACCELERATION)
  #ifdef __AVR__
    /**
     * This routine returns 0x1000000 / d, getting the inverse as fast as possible.
     * A fast-converging iterative Newton-Raphson method can reach full precision in
     * just 1 iteration, and takes 211 cycles (worst case; the mean case is less, up
     * to 30 cycles for small divisors), instead of the 500 cycles a normal division
     * would take.
     *
     * Inspired by the following page:
     *  https://stackoverflow.com/questions/27801397/newton-raphson-division-with-big-integers
     *
     * Suppose we want to calculate  floor(2 ^ k / B)  where B is a positive integer
     * Then, B must be <= 2^k, otherwise, the quotient is 0.
     *
     * The Newton - Raphson iteration for x = B / 2 ^ k yields:
     *  q[n + 1] = q[n] * (2 - q[n] * B / 2 ^ k)
     *
     * This can be rearranged to:
     *  q[n + 1] = q[n] * (2 ^ (k + 1) - q[n] * B) >> k
     *
     * Each iteration requires only integer multiplications and bit shifts.
     * It doesn't necessarily converge to floor(2 ^ k / B) but in the worst case
     * it eventually alternates between floor(2 ^ k / B) and ceil(2 ^ k / B).
     * So it checks for this case and extracts floor(2 ^ k / B).
     *
     * A simple but important optimization for this approach is to truncate
     * multiplications (i.e., calculate only the higher bits of the product) in the
     * early iterations of the Newton - Raphson method. This is done so the results
     * of the early iterations are far from the quotient. Then it doesn't matter if
     * they are done inaccurately.
     * It's important to pick a good starting value for x. Knowing how many
     * digits the divisor has, it can be estimated:
     *
     *   2^k / x = 2 ^ log2(2^k / x)
     *   2^k / x = 2 ^(log2(2^k)-log2(x))
     *   2^k / x = 2 ^(k*log2(2)-log2(x))
     *   2^k / x = 2 ^ (k-log2(x))
     *   2^k / x >= 2 ^ (k-floor(log2(x)))
     *   floor(log2(x)) is simply the index of the most significant bit set.
     *
     * If this estimation can be improved even further the number of iterations can be
     * reduced a lot, saving valuable execution time.
     * The paper "Software Integer Division" by Thomas L.Rodeheffer, Microsoft
     * Research, Silicon Valley,August 26, 2008, available at
     * https://www.microsoft.com/en-us/research/wp-content/uploads/2008/08/tr-2008-141.pdf
     * suggests, for its integer division algorithm, using a table to supply the first
     * 8 bits of precision, then, due to the quadratic convergence nature of the
     * Newton-Raphon iteration, just 2 iterations should be enough to get maximum
     * precision of the division.
     * By precomputing values of inverses for small denominator values, just one
     * Newton-Raphson iteration is enough to reach full precision.
     * This code uses the top 9 bits of the denominator as index.
     *
     * The AVR assembly function implements this C code using the data below:
     *
     *  // For small divisors, it is best to directly retrieve the results
     *  if (d <= 110) return pgm_read_dword(&small_inv_tab[d]);
     *
     *  // Compute initial estimation of 0x1000000/x -
     *  // Get most significant bit set on divider
     *  uint8_t idx = 0;
     *  uint32_t nr = d;
     *  if (!(nr & 0xFF0000)) {
     *    nr <<= 8; idx += 8;
     *    if (!(nr & 0xFF0000)) { nr <<= 8; idx += 8; }
     *  }
     *  if (!(nr & 0xF00000)) { nr <<= 4; idx += 4; }
     *  if (!(nr & 0xC00000)) { nr <<= 2; idx += 2; }
     *  if (!(nr & 0x800000)) { nr <<= 1; idx += 1; }
     *
     *  // Isolate top 9 bits of the denominator, to be used as index into the initial estimation table
     *  uint32_t tidx = nr >> 15,                                       // top 9 bits. bit8 is always set
     *           ie = inv_tab[tidx & 0xFF] + 256,                       // Get the table value. bit9 is always set
     *           x = idx <= 8 ? (ie >> (8 - idx)) : (ie << (idx - 8));  // Position the estimation at the proper place
     *
     *  x = uint32_t((x * uint64_t(_BV(25) - x * d)) >> 24);            // Refine estimation by newton-raphson. 1 iteration is enough
     *  const uint32_t r = _BV(24) - x * d;                             // Estimate remainder
     *  if (r >= d) x++;                                                // Check whether to adjust result
     *  return uint32_t(x);                                             // x holds the proper estimation
     */
    static uint32_t get_period_inverse(uint32_t d) {

      static const uint8_t inv_tab[256] PROGMEM = {
        255,253,252,250,248,246,244,242,240,238,236,234,233,231,229,227,
        225,224,222,220,218,217,215,213,212,210,208,207,205,203,202,200,
        199,197,195,194,192,191,189,188,186,185,183,182,180,179,178,176,
        175,173,172,170,169,168,166,165,164,162,161,160,158,157,156,154,
        153,152,151,149,148,147,146,144,143,142,141,139,138,137,136,135,
        134,132,131,130,129,128,127,126,125,123,122,121,120,119,118,117,
        116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,
        100,99,98,97,96,95,94,93,92,91,90,89,88,88,87,86,
        85,84,83,82,81,80,80,79,78,77,76,75,74,74,73,72,
        71,70,70,69,68,67,66,66,65,64,63,62,62,61,60,59,
        59,58,57,56,56,55,54,53,53,52,51,50,50,49,48,48,
        47,46,46,45,44,43,43,42,41,41,40,39,39,38,37,37,
        36,35,35,34,33,33,32,32,31,30,30,29,28,28,27,27,
        26,25,25,24,24,23,22,22,21,21,20,19,19,18,18,17,
        17,16,15,15,14,14,13,13,12,12,11,10,10,9,9,8,
        8,7,7,6,6,5,5,4,4,3,3,2,2,1,0,0
      };

      // For small denominators, it is cheaper to directly store the result.
      //  For bigger ones, just ONE Newton-Raphson iteration is enough to get
      //  maximum precision we need
      static const uint32_t small_inv_tab[111] PROGMEM = {
        16777216,16777216,8388608,5592405,4194304,3355443,2796202,2396745,2097152,1864135,1677721,1525201,1398101,1290555,1198372,1118481,
        1048576,986895,932067,883011,838860,798915,762600,729444,699050,671088,645277,621378,599186,578524,559240,541200,
        524288,508400,493447,479349,466033,453438,441505,430185,419430,409200,399457,390167,381300,372827,364722,356962,
        349525,342392,335544,328965,322638,316551,310689,305040,299593,294337,289262,284359,279620,275036,270600,266305,
        262144,258111,254200,250406,246723,243148,239674,236298,233016,229824,226719,223696,220752,217885,215092,212369,
        209715,207126,204600,202135,199728,197379,195083,192841,190650,188508,186413,184365,182361,180400,178481,176602,
        174762,172960,171196,169466,167772,166111,164482,162885,161319,159783,158275,156796,155344,153919,152520
      };

      // For small divisors, it is best to directly retrieve the results
      if (d <= 110) return pgm_read_dword(&small_inv_tab[d]);

      uint8_t r8 = d & 0xFF,
              r9 = (d >> 8) & 0xFF,
              r10 = (d >> 16) & 0xFF,
              r2,r3,r4,r5,r6,r7,r11,r12,r13,r14,r15,r16,r17,r18;
      const uint8_t *ptab = inv_tab;

      __asm__ __volatile__(
        // %8:%7:%6 = interval
        // r31:r30: MUST be those registers, and they must point to the inv_tab

        A("clr %13")                       // %13 = 0

        // Now we must compute
        // result = 0xFFFFFF / d
        // %8:%7:%6 = interval
        // %16:%15:%14 = nr
        // %13 = 0

        // A plain division of 24x24 bits should take 388 cycles to complete. We will
        // use Newton-Raphson for the calculation, and will strive to get way less cycles
        // for the same result - Using C division, it takes 500cycles to complete .

        A("clr %3")                       // idx = 0
        A("mov %14,%6")
        A("mov %15,%7")
        A("mov %16,%8")                   // nr = interval
        A("tst %16")                      // nr & 0xFF0000 == 0 ?
        A("brne 2f")                      // No, skip this
        A("mov %16,%15")
        A("mov %15,%14")                  // nr <<= 8, %14 not needed
        A("subi %3,-8")                   // idx += 8
        A("tst %16")                      // nr & 0xFF0000 == 0 ?
        A("brne 2f")                      // No, skip this
        A("mov %16,%15")                  // nr <<= 8, %14 not needed
        A("clr %15")                      // We clear %14
        A("subi %3,-8")                   // idx += 8

        // here %16 != 0 and %16:%15 contains at least 9 MSBits, or both %16:%15 are 0
        L("2")
        A("cpi %16,0x10")                 // (nr & 0xF00000) == 0 ?
        A("brcc 3f")                      // No, skip this
        A("swap %15")                     // Swap nybbles
        A("swap %16")                     // Swap nybbles. Low nybble is 0
        A("mov %14, %15")
        A("andi %14,0x0F")                // Isolate low nybble
        A("andi %15,0xF0")                // Keep proper nybble in %15
        A("or %16, %14")                  // %16:%15 <<= 4
        A("subi %3,-4")                   // idx += 4

        L("3")
        A("cpi %16,0x40")                 // (nr & 0xC00000) == 0 ?
        A("brcc 4f")                      // No, skip this
        A("add %15,%15")
        A("adc %16,%16")
        A("add %15,%15")
        A("adc %16,%16")                  // %16:%15 <<= 2
        A("subi %3,-2")                   // idx += 2

        L("4")
        A("cpi %16,0x80")                 // (nr & 0x800000) == 0 ?
        A("brcc 5f")                      // No, skip this
        A("add %15,%15")
        A("adc %16,%16")                  // %16:%15 <<= 1
        A("inc %3")                       // idx += 1

        // Now %16:%15 contains its MSBit set to 1, or %16:%15 is == 0. We are now absolutely sure
        // we have at least 9 MSBits available to enter the initial estimation table
        L("5")
        A("add %15,%15")
        A("adc %16,%16")                  // %16:%15 = tidx = (nr <<= 1), we lose the top MSBit (always set to 1, %16 is the index into the inverse table)
        A("add r30,%16")                  // Only use top 8 bits
        A("adc r31,%13")                  // r31:r30 = inv_tab + (tidx)
        A("lpm %14, Z")                   // %14 = inv_tab[tidx]
        A("ldi %15, 1")                   // %15 = 1  %15:%14 = inv_tab[tidx] + 256

        // We must scale the approximation to the proper place
        A("clr %16")                      // %16 will always be 0 here
        A("subi %3,8")                    // idx == 8 ?
        A("breq 6f")                      // yes, no need to scale
        A("brcs 7f")                      // If C=1, means idx < 8, result was negative!

        // idx > 8, now %3 = idx - 8. We must perform a left shift. idx range:[1-8]
        A("sbrs %3,0")                    // shift by 1bit position?
        A("rjmp 8f")                      // No
        A("add %14,%14")
        A("adc %15,%15")                  // %15:16 <<= 1
        L("8")
        A("sbrs %3,1")                    // shift by 2bit position?
        A("rjmp 9f")                      // No
        A("add %14,%14")
        A("adc %15,%15")
        A("add %14,%14")
        A("adc %15,%15")                  // %15:16 <<= 1
        L("9")
        A("sbrs %3,2")                    // shift by 4bits position?
        A("rjmp 16f")                     // No
        A("swap %15")                     // Swap nybbles. lo nybble of %15 will always be 0
        A("swap %14")                     // Swap nybbles
        A("mov %12,%14")
        A("andi %12,0x0F")                // isolate low nybble
        A("andi %14,0xF0")                // and clear it
        A("or %15,%12")                   // %15:%16 <<= 4
        L("16")
        A("sbrs %3,3")                    // shift by 8bits position?
        A("rjmp 6f")                      // No, we are done
        A("mov %16,%15")
        A("mov %15,%14")
        A("clr %14")
        A("jmp 6f")

        // idx < 8, now %3 = idx - 8. Get the count of bits
        L("7")
        A("neg %3")                       // %3 = -idx = count of bits to move right. idx range:[1...8]
        A("sbrs %3,0")                    // shift by 1 bit position ?
        A("rjmp 10f")                     // No, skip it
        A("asr %15")                      // (bit7 is always 0 here)
        A("ror %14")
        L("10")
        A("sbrs %3,1")                    // shift by 2 bit position ?
        A("rjmp 11f")                     // No, skip it
        A("asr %15")                      // (bit7 is always 0 here)
        A("ror %14")
        A("asr %15")                      // (bit7 is always 0 here)
        A("ror %14")
        L("11")
        A("sbrs %3,2")                    // shift by 4 bit position ?
        A("rjmp 12f")                     // No, skip it
        A("swap %15")                     // Swap nybbles
        A("andi %14, 0xF0")               // Lose the lowest nybble
        A("swap %14")                     // Swap nybbles. Upper nybble is 0
        A("or %14,%15")                   // Pass nybble from upper byte
        A("andi %15, 0x0F")               // And get rid of that nybble
        L("12")
        A("sbrs %3,3")                    // shift by 8 bit position ?
        A("rjmp 6f")                      // No, skip it
        A("mov %14,%15")
        A("clr %15")
        L("6")                            // %16:%15:%14 = initial estimation of 0x1000000 / d

        // Now, we must refine the estimation present on %16:%15:%14 using 1 iteration
        // of Newton-Raphson. As it has a quadratic convergence, 1 iteration is enough
        // to get more than 18bits of precision (the initial table lookup gives 9 bits of
        // precision to start from). 18bits of precision is all what is needed here for result

        // %8:%7:%6 = d = interval
        // %16:%15:%14 = x = initial estimation of 0x1000000 / d
        // %13 = 0
        // %3:%2:%1:%0 = working accumulator

        // Compute 1<<25 - x*d. Result should never exceed 25 bits and should always be positive
        A("clr %0")
        A("clr %1")
        A("clr %2")
        A("ldi %3,2")                     // %3:%2:%1:%0 = 0x2000000
        A("mul %6,%14")                   // r1:r0 = LO(d) * LO(x)
        A("sub %0,r0")
        A("sbc %1,r1")
        A("sbc %2,%13")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * LO(x)
        A("mul %7,%14")                   // r1:r0 = MI(d) * LO(x)
        A("sub %1,r0")
        A("sbc %2,r1" )
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= MI(d) * LO(x) << 8
        A("mul %8,%14")                   // r1:r0 = HI(d) * LO(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MIL(d) * LO(x) << 16
        A("mul %6,%15")                   // r1:r0 = LO(d) * MI(x)
        A("sub %1,r0")
        A("sbc %2,r1")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * MI(x) << 8
        A("mul %7,%15")                   // r1:r0 = MI(d) * MI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MI(d) * MI(x) << 16
        A("mul %8,%15")                   // r1:r0 = HI(d) * MI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MIL(d) * MI(x) << 24
        A("mul %6,%16")                   // r1:r0 = LO(d) * HI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= LO(d) * HI(x) << 16
        A("mul %7,%16")                   // r1:r0 = MI(d) * HI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MI(d) * HI(x) << 24
        // %3:%2:%1:%0 = (1<<25) - x*d     [169]

        // We need to multiply that result by x, and we are only interested in the top 24bits of that multiply

        // %16:%15:%14 = x = initial estimation of 0x1000000 / d
        // %3:%2:%1:%0 = (1<<25) - x*d = acc
        // %13 = 0

        // result = %11:%10:%9:%5:%4
        A("mul %14,%0")                   // r1:r0 = LO(x) * LO(acc)
        A("mov %4,r1")
        A("clr %5")
        A("clr %9")
        A("clr %10")
        A("clr %11")                      // %11:%10:%9:%5:%4 = LO(x) * LO(acc) >> 8
        A("mul %15,%0")                   // r1:r0 = MI(x) * LO(acc)
        A("add %4,r0")
        A("adc %5,r1")
        A("adc %9,%13")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * LO(acc)
        A("mul %16,%0")                   // r1:r0 = HI(x) * LO(acc)
        A("add %5,r0")
        A("adc %9,r1")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * LO(acc) << 8

        A("mul %14,%1")                   // r1:r0 = LO(x) * MIL(acc)
        A("add %4,r0")
        A("adc %5,r1")
        A("adc %9,%13")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 = LO(x) * MIL(acc)
        A("mul %15,%1")                   // r1:r0 = MI(x) * MIL(acc)
        A("add %5,r0")
        A("adc %9,r1")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * MIL(acc) << 8
        A("mul %16,%1")                   // r1:r0 = HI(x) * MIL(acc)
        A("add %9,r0")
        A("adc %10,r1")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * MIL(acc) << 16

        A("mul %14,%2")                   // r1:r0 = LO(x) * MIH(acc)
        A("add %5,r0")
        A("adc %9,r1")
        A("adc %10,%13")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 = LO(x) * MIH(acc) << 8
        A("mul %15,%2")                   // r1:r0 = MI(x) * MIH(acc)
        A("add %9,r0")
        A("adc %10,r1")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 += MI(x) * MIH(acc) << 16
        A("mul %16,%2")                   // r1:r0 = HI(x) * MIH(acc)
        A("add %10,r0")
        A("adc %11,r1")                   // %11:%10:%9:%5:%4 += MI(x) * MIH(acc) << 24

        A("mul %14,%3")                   // r1:r0 = LO(x) * HI(acc)
        A("add %9,r0")
        A("adc %10,r1")
        A("adc %11,%13")                  // %11:%10:%9:%5:%4 = LO(x) * HI(acc) << 16
        A("mul %15,%3")                   // r1:r0 = MI(x) * HI(acc)
        A("add %10,r0")
        A("adc %11,r1")                   // %11:%10:%9:%5:%4 += MI(x) * HI(acc) << 24
        A("mul %16,%3")                   // r1:r0 = HI(x) * HI(acc)
        A("add %11,r0")                   // %11:%10:%9:%5:%4 += MI(x) * HI(acc) << 32

        // At this point, %11:%10:%9 contains the new estimation of x.

        // Finally, we must correct the result. Estimate remainder as
        // (1<<24) - x*d
        // %11:%10:%9 = x
        // %8:%7:%6 = d = interval" "\n\t"
        A("ldi %3,1")
        A("clr %2")
        A("clr %1")
        A("clr %0")                       // %3:%2:%1:%0 = 0x1000000
        A("mul %6,%9")                    // r1:r0 = LO(d) * LO(x)
        A("sub %0,r0")
        A("sbc %1,r1")
        A("sbc %2,%13")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * LO(x)
        A("mul %7,%9")                    // r1:r0 = MI(d) * LO(x)
        A("sub %1,r0")
        A("sbc %2,r1")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= MI(d) * LO(x) << 8
        A("mul %8,%9")                    // r1:r0 = HI(d) * LO(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MIL(d) * LO(x) << 16
        A("mul %6,%10")                   // r1:r0 = LO(d) * MI(x)
        A("sub %1,r0")
        A("sbc %2,r1")
        A("sbc %3,%13")                   // %3:%2:%1:%0 -= LO(d) * MI(x) << 8
        A("mul %7,%10")                   // r1:r0 = MI(d) * MI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= MI(d) * MI(x) << 16
        A("mul %8,%10")                   // r1:r0 = HI(d) * MI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MIL(d) * MI(x) << 24
        A("mul %6,%11")                   // r1:r0 = LO(d) * HI(x)
        A("sub %2,r0")
        A("sbc %3,r1")                    // %3:%2:%1:%0 -= LO(d) * HI(x) << 16
        A("mul %7,%11")                   // r1:r0 = MI(d) * HI(x)
        A("sub %3,r0")                    // %3:%2:%1:%0 -= MI(d) * HI(x) << 24
        // %3:%2:%1:%0 = r = (1<<24) - x*d
        // %8:%7:%6 = d = interval

        // Perform the final correction
        A("sub %0,%6")
        A("sbc %1,%7")
        A("sbc %2,%8")                    // r -= d
        A("brcs 14f")                     // if ( r >= d)

        // %11:%10:%9 = x
        A("ldi %3,1")
        A("add %9,%3")
        A("adc %10,%13")
        A("adc %11,%13")                  // x++
        L("14")

        // Estimation is done. %11:%10:%9 = x
        A("clr __zero_reg__")              // Make C runtime happy
        // [211 cycles total]
        : "=r" (r2),
          "=r" (r3),
          "=r" (r4),
          "=d" (r5),
          "=r" (r6),
          "=r" (r7),
          "+r" (r8),
          "+r" (r9),
          "+r" (r10),
          "=d" (r11),
          "=r" (r12),
          "=r" (r13),
          "=d" (r14),
          "=d" (r15),
          "=d" (r16),
          "=d" (r17),
          "=d" (r18),
          "+z" (ptab)
        :
        : "r0", "r1", "cc"
      );

      // Return the result
      return r11 | (uint16_t(r12) << 8) | (uint32_t(r13) << 16);
    }
  #else
    // All other 32-bit MPUs can easily do inverse using hardware division,
    // so we don't need to reduce precision or to use assembly language at all.
    // This routine, for all other archs, returns 0x100000000 / d ~= 0xFFFFFFFF / d
    static FORCE_INLINE uint32_t get_period_inverse(const uint32_t d) {
      return d ? 0xFFFFFFFF / d : 0xFFFFFFFF;
    }
  #endif
#endif


void Planner::wait_for_empty_segment_buffer() {
  while (block_buffer_tail != block_buffer_head);
}


void Planner::zeroSpeeds() {
  wait_for_empty_segment_buffer();

  previous_nominal_speed_sqr = 0;
  previous_safe_speed = 0;
  for(ALL_MUSCLES(i)) previous_speed[i] = 0;
  for(ALL_MOTORS(i)) prev_unit_vec[i] = 0;

  block_buffer_tail = 0;
  block_buffer_head = 0;
  first_segment_delay = 0;
}

/**
   Calculate the maximum allowable speed at this point, in order
   to reach 'target_velocity' using 'acceleration' within a given
   'distance'.
   @param acc acceleration
   @param target_velocity
   @param distance
*/
float Planner::max_speed_allowed_sqr(const float &acc, const float &target_velocity_sqr, const float &distance) {
  return target_velocity_sqr - 2 * acc * distance;
}

void Planner::recalculate_reverse_kernel(Segment *const current, const Segment *next) {
  if(current == NULL) return;

  const float entry_speed_max_sqr = current->entry_speed_max_sqr;
  if(current->entry_speed_sqr != entry_speed_max_sqr || (next && TEST(next->flags,BIT_FLAG_RECALCULATE)) ) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    const float new_entry_speed_sqr = TEST(current->flags, BIT_FLAG_NOMINAL)
                                  ? entry_speed_max_sqr
                                  : _MIN( entry_speed_max_sqr, max_speed_allowed_sqr(-current->acceleration, (next ? next->entry_speed_sqr : sq(float(MIN_FEEDRATE))), current->distance) );
    if( current->entry_speed_sqr != new_entry_speed_sqr ) {
      SET_BIT_ON(current->flags, BIT_FLAG_RECALCULATE);
      if( motor.isBlockBusy(current) ) {
        SET_BIT_OFF(current->flags, BIT_FLAG_RECALCULATE);
      } else {
        current->entry_speed_sqr = new_entry_speed_sqr;
      }
    }
  }
}

void Planner::reversePass() {
  int block_index = getPrevBlock(block_buffer_head);

  uint8_t planned_block_index = block_buffer_planned;
  if(planned_block_index == block_buffer_head) return;

  Segment *next = NULL;
  while (block_index != block_buffer_tail) {
    Segment *current = &blockBuffer[block_index];

    recalculate_reverse_kernel(current, next);
    next = current;
    block_index = getPrevBlock(block_index);
    while(planned_block_index != block_buffer_planned) {
      // If we reached the busy block or an already processed block, break the loop now
      if(block_index == planned_block_index) return;
      // Advance the pointer, following the busy block
      planned_block_index = getNextBlock(planned_block_index);
    }
  }
}

void Planner::recalculate_forward_kernel(const Segment *prev, Segment *const current,uint8_t block_index) {
  if(prev == NULL) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if( !TEST(prev->flags,BIT_FLAG_NOMINAL) && prev->entry_speed_sqr < current->entry_speed_sqr ) {
    const float new_entry_speed_sqr = max_speed_allowed_sqr(-prev->acceleration, prev->entry_speed_sqr, prev->distance);
    // Check for junction speed change
    if(new_entry_speed_sqr < current->entry_speed_sqr) {
      SET_BIT_ON(current->flags, BIT_FLAG_RECALCULATE);
      if(motor.isBlockBusy(current)) {
        SET_BIT_OFF(current->flags, BIT_FLAG_RECALCULATE);
      } else {
        current->entry_speed_sqr = new_entry_speed_sqr;
        block_buffer_planned = block_index;
      }
    }
  }
  if(current->entry_speed_sqr == current->entry_speed_max_sqr)
    block_buffer_planned = block_index;
}

void Planner::forwardPass() {
  int block_index = block_buffer_planned;

  Segment *previous = NULL;
  while (block_index != block_buffer_head) {
    Segment *current = &blockBuffer[block_index];
    if(!previous || !motor.isBlockBusy(previous)) {
      recalculate_forward_kernel(previous, current,block_index);
    }
    previous = current;
    block_index = getNextBlock(block_index);
  }
}

float Planner::estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
  if(accel == 0) return 0;
  return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
}

int Planner::intersection_distance(const float &start_rate, const float &end_rate, const float &accel, const float &distance) {
  if(accel == 0) return 0;
  return (2.0 * accel * distance - sq(start_rate) + sq(end_rate)) / (4.0 * accel);
}

void Planner::calculate_trapezoid_for_block(Segment *block, const float &entry_factor, const float &exit_factor) {
  uint32_t initial_rate = ceilf(block->nominal_rate * entry_factor);
  uint32_t final_rate  = ceilf(block->nominal_rate * exit_factor);

  if(initial_rate < MIN_STEP_RATE) initial_rate = MIN_STEP_RATE;
  if(final_rate < MIN_STEP_RATE) final_rate = MIN_STEP_RATE;

#if defined(S_CURVE_ACCELERATION)
  uint32_t cruise_rate = initial_rate;
#endif

  const int32_t accel = block->acceleration_steps_per_s2;
  uint32_t accelerate_steps = ceilf(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel));
  uint32_t decelerate_steps = floorf(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel));
  int32_t plateau_steps = block->steps_total - accelerate_steps - decelerate_steps;
  if(plateau_steps < 0) {
    const float accelerate_steps_float = ceilf(intersection_distance(initial_rate, final_rate, accel, block->steps_total));
    accelerate_steps = _MIN(((uint32_t)_MAX(accelerate_steps_float, 0.0f)), block->steps_total);
    plateau_steps = 0;
  #if defined(S_CURVE_ACCELERATION)
    // We won't reach the cruising rate. Let's calculate the speed we will reach
    cruise_rate = final_speed(initial_rate, accel, accelerate_steps);
  #endif
  }
#if defined(S_CURVE_ACCELERATION)
    else // We have some plateau time, so the cruise rate will be the nominal rate
      cruise_rate = block->nominal_rate;
      
  // Jerk controlled speed requires to express speed versus time, NOT steps
  uint32_t acceleration_time = ((float)(cruise_rate - initial_rate) / accel) * (STEPPER_TIMER_RATE),
            deceleration_time = ((float)(cruise_rate - final_rate) / accel) * (STEPPER_TIMER_RATE),
  // And to offload calculations from the ISR, we also calculate the inverse of those times here
            acceleration_time_inverse = get_period_inverse(acceleration_time),
            deceleration_time_inverse = get_period_inverse(deceleration_time);
#endif
  block->accel_until  = accelerate_steps;
  block->decel_after  = accelerate_steps + plateau_steps;
  block->initial_rate = initial_rate;
  block->final_rate   = final_rate;
#if defined(S_CURVE_ACCELERATION)
  block->acceleration_time = acceleration_time;
  block->deceleration_time = deceleration_time;
  block->acceleration_time_inverse = acceleration_time_inverse;
  block->deceleration_time_inverse = deceleration_time_inverse;
  block->cruise_rate = cruise_rate;
#endif
}

void Planner::recalculate_trapezoids() {
  uint8_t block_index = block_buffer_tail;
  uint8_t head_block_index = block_buffer_head;

  Segment *block = NULL;
  Segment *next = NULL;
  float current_entry_speed = 0, next_entry_speed = 0;

  while (block_index != head_block_index) {
    next = &blockBuffer[block_index];
    next_entry_speed = sqrtf(next->entry_speed_sqr);
    if(block) {
      // Recalculate if current block entry or exit junction speed has changed.
      if( TEST(block->flags,BIT_FLAG_RECALCULATE) || TEST(next->flags,BIT_FLAG_RECALCULATE) ) {
        SET_BIT_ON( block->flags, BIT_FLAG_RECALCULATE );
        if( !Stepper::isBlockBusy(block) ) {
          // NOTE: Entry and exit factors always > 0 by all previous logic operations.
          const float current_nominal_speed = sqrtf(block->nominal_speed_sqr),
                      inom = 1.0f / current_nominal_speed;
          calculate_trapezoid_for_block(block, current_entry_speed * inom, next_entry_speed * inom);
        }
        // Reset current only to ensure next trapezoid is computed
        SET_BIT_OFF(block->flags,BIT_FLAG_RECALCULATE);
      }
    }
    block = next;
    current_entry_speed = next_entry_speed;
    block_index = getNextBlock(block_index);
  }

  // Last/newest block in buffer. Make sure the last block always ends motion.
  if(next) {
    SET_BIT_ON(next->flags,BIT_FLAG_RECALCULATE);
    if( !Stepper::isBlockBusy(next) ) {
      const float current_nominal_speed = sqrtf(next->nominal_speed_sqr),
                  inom = 1.0f / current_nominal_speed;
      calculate_trapezoid_for_block(next, next_entry_speed * inom, float(MINIMUM_PLANNER_SPEED) * inom);
    }
    SET_BIT_OFF(next->flags,BIT_FLAG_RECALCULATE);
  }
}

void Planner::recalculate() {
  if(getPrevBlock(block_buffer_head) != block_buffer_planned) {
    reversePass();
    forwardPass();
  }
  recalculate_trapezoids();
}

void Planner::describeAllSegments() {
  CRITICAL_SECTION_START();
  static uint8_t once = 0;
  if(once == 0) {
    once = 1;
    SERIAL_ECHOLNPGM("A = index");
    SERIAL_ECHOLNPGM("B = distance");
    SERIAL_ECHOLNPGM("C = acceleration");
    SERIAL_ECHOLNPGM("D = acceleration steps s2");
    SERIAL_ECHOLNPGM("E = acceleration rate");

    SERIAL_ECHOLNPGM("F = entry_speed");
    SERIAL_ECHOLNPGM("G = nominal_speed");
    SERIAL_ECHOLNPGM("H = entry_speed_max");

    SERIAL_ECHOLNPGM("I = entry rate");
    SERIAL_ECHOLNPGM("J = nominal rate");
    SERIAL_ECHOLNPGM("K = exit rate");

    SERIAL_ECHOLNPGM("L = accel_until");
    SERIAL_ECHOLNPGM("M = coast steps");
    SERIAL_ECHOLNPGM("N = decel steps");
    SERIAL_ECHOLNPGM("O = total steps");

    SERIAL_ECHOLNPGM("P = nominal?");
    SERIAL_ECHOLNPGM("Q = recalculate?");
    SERIAL_ECHOLNPGM("R = busy?");
    SERIAL_ECHOLNPGM("\nA\tB\tC\tD\tE\tF\tG\tH\tI\tJ\tK\tL\tM\tN\tO\tP\tQ\tR");
  }
  SERIAL_ECHOLNPGM("---------------------------------------------------------------------------------------------------------------------------");

  int s = block_buffer_tail;
  while (s != block_buffer_head) {
    Segment *next = &blockBuffer[s];
    int coast     = next->decel_after - next->accel_until;
    int decel     = next->steps_total - next->decel_after;
    SERIAL_ECHO(s);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->distance);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->acceleration);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->acceleration_steps_per_s2);
#ifndef S_CURVE_ACCELERATION
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->acceleration_rate);
#endif

    SERIAL_CHAR('\t');   SERIAL_ECHO(next->entry_speed_sqr);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->nominal_speed_sqr);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->entry_speed_max_sqr);

    SERIAL_CHAR('\t');   SERIAL_ECHO(next->initial_rate);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->nominal_rate);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->final_rate);

    SERIAL_CHAR('\t');   SERIAL_ECHO(next->accel_until);
    SERIAL_CHAR('\t');   SERIAL_ECHO(coast);
    SERIAL_CHAR('\t');   SERIAL_ECHO(decel);
    SERIAL_CHAR('\t');   SERIAL_ECHO(next->steps_total);
    //SERIAL_CHAR('\t');   SERIAL_ECHO(next->steps_taken);

    SERIAL_CHAR('\t');   SERIAL_ECHO(TEST(next->flags,BIT_FLAG_NOMINAL) != 0 ? 'Y' : 'N');
    SERIAL_CHAR('\t');   SERIAL_ECHO(TEST(next->flags,BIT_FLAG_RECALCULATE) != 0 ? 'Y' : 'N');
    SERIAL_CHAR('\t');   SERIAL_ECHO(motor.isBlockBusy(next) != 0 ? 'Y' : 'N');
    SERIAL_EOL();
    s = getNextBlock(s);
  }
  CRITICAL_SECTION_END();
}

bool Planner::populateBlock(Segment *newBlock,const float *const target, float fr_units_s, float cartesianDistance) {
  // convert from the cartesian position to the motor steps
  int32_t steps[NUM_MUSCLES];
  IK(target, steps);

  // find the number of steps for each motor, the direction, and the absolute steps
  // The axis that has the most steps will control the overall acceleration as per bresenham's algorithm.
  newBlock->steps_total = 0;
  newBlock->dir=0;

  float deltaSteps[NUM_MUSCLES];

  for (ALL_MUSCLES(i)) {
    newBlock->a[i].step_count = steps[i];
    deltaSteps[i] = steps[i] - position[i];
    if(deltaSteps[i] > 0) newBlock->dir |= (1UL<<i);
    newBlock->a[i].absdelta = abs(deltaSteps[i]);
    newBlock->steps_total = _MAX(newBlock->steps_total, newBlock->a[i].absdelta);
#if MACHINE_STYLE == SIXI
    newBlock->a[i].positionStart = axies[i].pos;
    newBlock->a[i].positionEnd   = target_position[i];
#endif
  }

  float oldP[NUM_AXIES];
  float deltaCartesian[NUM_AXIES];
  for(ALL_AXIES(i)) {
    oldP[i] = axies[i].pos;
    deltaCartesian[i] = target[i] - oldP[i];
  }

  // No steps?  No work!  Stop now.
  if(newBlock->steps_total < MIN_STEPS_PER_SEGMENT) return false;

  newBlock->distance = cartesianDistance;

  // record the new target position & feed rate for the next movement.
  float inverseCartesianDistance = 1.0f / cartesianDistance;
  // s = v / d
  // d*s = v
  //
  float inverse_secs = fr_units_s * inverseCartesianDistance;

  int movesQueued = movesPlannedNotBusy();
#ifdef BUFFER_EMPTY_SLOWDOWN
    #ifndef SLOWDOWN_DIVISOR
      #define SLOWDOWN_DIVISOR 2
    #endif
  if(movesQueued>=2 && movesQueued<=(MAX_SEGMENTS / SLOWDOWN_DIVISOR) - 1) {
    uint32_t segment_time_us = lroundf(1000000.0f / inverse_secs);
    const int32_t time_diff = Stepper::min_segment_time_us - segment_time_us;
    if(time_diff > 0) {
      //SERIAL_ECHOPAIR("was ",1.0f/inverse_secs);
      const uint32_t nst = segment_time_us + lroundf(2 * time_diff  / movesQueued);
      inverse_secs       = 1000000.0f / nst;
      //REPORT(" now ",1.0f/inverse_secs);
    }
  }// else REPORT("Q",movesQueued);
#endif

  newBlock->nominal_speed_sqr = sq(cartesianDistance * inverse_secs);
  newBlock->nominal_rate  = ceilf(newBlock->steps_total * inverse_secs);

  // Calculate the the speed limit for each motor.
  // All speeds are connected so if one motor slows, they all have to slow the same amount.
  float current_speed[NUM_MUSCLES], speed_factor = 1.0;
  for (ALL_MUSCLES(i)) {
    current_speed[i] = deltaSteps[i] * inverse_secs;
    const float cs = fabs(current_speed[i]), max_fr = max_step_rate[i];
    if(cs > max_fr) speed_factor = _MIN(speed_factor, max_fr / cs);
  }
  
  // apply the speed limit
  if(speed_factor < 1.0) {
    for (ALL_MUSCLES(i)) {
      current_speed[i] *= speed_factor;
    }
    newBlock->nominal_speed_sqr *= sq(speed_factor);
    newBlock->nominal_rate *= speed_factor;
  }

#if MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)
  float max_acceleration = limitPolargraphAcceleration(target,oldP,desiredAcceleration);
#else
  float max_acceleration = desiredAcceleration;
#endif

  const float steps_per_unit = newBlock->steps_total * inverseCartesianDistance;
  uint32_t accel = ceilf(max_acceleration * steps_per_unit);

  uint32_t highest_rate = 1;
  float max_acceleration_steps_per_s2[NUM_MUSCLES];

  for (ALL_MUSCLES(i)) {
    max_acceleration_steps_per_s2[i] = max_acceleration * motor_spu[i];
    highest_rate = _MAX( highest_rate, max_acceleration_steps_per_s2[i] );
  }
  uint32_t acceleration_long_cutoff = 4294967295UL / highest_rate; // 0xFFFFFFFFUL

  if(newBlock->steps_total <= acceleration_long_cutoff) {
    for (ALL_MUSCLES(i)) {
      if(newBlock->a[i].absdelta && max_acceleration_steps_per_s2[i] < accel) {
        const uint32_t max_possible = max_acceleration_steps_per_s2[i] * newBlock->steps_total / newBlock->a[i].absdelta;
        accel = _MIN( accel, max_possible );
      }
    }
  } else {
    for (ALL_MUSCLES(i)) {
      if(newBlock->a[i].absdelta && max_acceleration_steps_per_s2[i] < accel) {
        const float max_possible = float(max_acceleration_steps_per_s2[i]) * float(newBlock->steps_total) / float(newBlock->a[i].absdelta);
        accel = _MIN( accel, (uint32_t)max_possible );
      }
    }
  }

  newBlock->acceleration_steps_per_s2 = accel;
  newBlock->acceleration              = accel / steps_per_unit;
#ifndef S_CURVE_ACCELERATION
  newBlock->acceleration_rate         = (uint32_t)(accel * (sq(4096.0f) / (STEPPER_TIMER_RATE)));
#endif
  newBlock->steps_taken               = 0;

  // BEGIN JERK LIMITING
  float vmax_junction_sqr;

#ifdef HAS_CLASSIC_JERK
  vmax_junction_sqr = classicJerk(newBlock,current_speed,movesQueued);
#endif
#ifdef HAS_JUNCTION_DEVIATION
  vmax_junction_sqr = junctionDeviation(newBlock,deltaCartesian,movesQueued,inverseCartesianDistance,max_acceleration);
#endif
#ifdef DOT_PRODUCT_JERK
  
  float unit_vec[NUM_MOTORS] = {
    #define COPY_4(NN) (delta[NN]) * inverseCartesianDistance,
    ALL_MOTOR_MACRO(COPY_4)
  };

  float dot = 0
  #define NORM_MOTOR(NN) +(unit_vec[NN]*prev_unit_vec[NN])
  ALL_MOTOR_MACRO(NORM_MOTOR)
  ;
 
  vmax_junction_sqr = newBlock->nominal_speed_sqr * dot * 1.1f;
  vmax_junction_sqr = _MIN(vmax_junction_sqr, newBlock->nominal_speed_sqr);
  vmax_junction_sqr = _MAX(vmax_junction_sqr, MINIMUM_PLANNER_SPEED);
  
  #define COPY_5(NN) prev_unit_vec[NN] = unit_vec[NN];
  ALL_MOTOR_MACRO(COPY_5)
#endif
  // END JERK LIMITING

  newBlock->entry_speed_max_sqr = vmax_junction_sqr;

  float allowable_speed_sqr = max_speed_allowed_sqr(-newBlock->acceleration, sq(float(MINIMUM_PLANNER_SPEED)), newBlock->distance);
  newBlock->entry_speed_sqr = _MIN(vmax_junction_sqr, allowable_speed_sqr);
  SET_BIT( newBlock->flags, BIT_FLAG_NOMINAL, ( newBlock->nominal_speed_sqr <= allowable_speed_sqr ) );
  SET_BIT_ON( newBlock->flags, BIT_FLAG_RECALCULATE );

  previous_nominal_speed_sqr = newBlock->nominal_speed_sqr;
  for(ALL_MUSCLES(i)) {
    previous_speed[i] = current_speed[i];
    position[i] = steps[i];
  }
  for(ALL_AXIES(i)) {
    axies[i].pos = target[i];
  }
  return true;
}

void Planner::segmentReport(Segment &new_seg) {
  uint32_t addr=(long)&new_seg;
  SERIAL_ECHOLNPAIR("seg:",addr);
  SERIAL_ECHOLNPAIR("distance=",new_seg.distance);
  SERIAL_ECHOLNPAIR("nominal_speed=",new_seg.nominal_speed_sqr);
  SERIAL_ECHOLNPAIR("nominal_rate=",new_seg.nominal_rate);
  SERIAL_ECHOLNPAIR("acceleration_steps_per_s2=",new_seg.acceleration_steps_per_s2);
  SERIAL_ECHOLNPAIR("acceleration=",new_seg.acceleration);
  SERIAL_ECHOLNPAIR("nominal_speed=",new_seg.nominal_speed_sqr);
  SERIAL_ECHOLNPAIR("entry_speed_max=",new_seg.entry_speed_max_sqr);
  SERIAL_ECHOLNPAIR("entry_speed=",new_seg.entry_speed_sqr);
}

/**
   Translate the XYZ through the IK to get the number of motor steps and move the motors.
   Uses bresenham's line algorithm to move both motors
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
   @input longest_distance must be >=0
*/
void Planner::addSegment(const float *const target_position, float fr_units_s, float longest_distance) {
  uint8_t next_buffer_head;
  Segment *newBlock = getNextFreeBlock(next_buffer_head);

  if(!populateBlock(newBlock,target_position,fr_units_s,longest_distance)) {
    // move was too short.  bail!
    return;
  }

  if(block_buffer_tail == block_buffer_head) {
    first_segment_delay = BLOCK_DELAY_FOR_1ST_MOVE;
  }
  block_buffer_head = next_buffer_head;

  recalculate();
  // describeAllSegments();

  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
   Split long moves into sub-moves if needed.
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
*/
void Planner::bufferLine(float *pos, float new_feed_rate_units) {
  // Remember the feed rate.  This value will be used whenever no feedrate is given in a command, so it MUST be saved
  // BEFORE the dial adjustment. otherwise the feedrate will slowly fall or climb as new commands are processed.
  desiredFeedRate = new_feed_rate_units;

#ifdef HAS_LCD
  // use LCD to adjust speed while drawing
  new_feed_rate_units *= (float)speed_adjust * 0.01f;
#endif

  // split up long lines to make them straighter
  float deltaCartesian[NUM_AXIES];
  float intermediatePos[NUM_AXIES];
  float lenSquared = 0;

  for (ALL_AXIES(i)) {
    intermediatePos[i] = axies[i].pos;
    deltaCartesian[i] = pos[i] - intermediatePos[i];
    lenSquared += sq(deltaCartesian[i]);
  }

#if MACHINE_STYLE == POLARGRAPH
  if(deltaCartesian[0] == 0 && deltaCartesian[1] == 0) {
    // only moving Z, don't split the line.
    addSegment(pos, new_feed_rate_units, abs(deltaCartesian[2]));
    return;
  }
#endif   

  float totalDistance = sqrtf(lenSquared);
  if(abs(totalDistance) < 0.000001f) return;

#ifndef MIN_SEGMENT_LENGTH
#define MIN_SEGMENT_LENGTH 0.5f
#endif
  const float seconds = totalDistance / new_feed_rate_units;
  int16_t segments   = seconds * SEGMENTS_PER_SECOND;
  segments = _MIN(segments, totalDistance / MIN_SEGMENT_LENGTH);
  segments = _MAX(segments, 1);

#ifdef HAS_GRIPPER
  // if we have a gripper and only gripper is moving, don't split the movement.
  if(lenSquared == sq(deltaCartesian[6])) {
    SERIAL_ECHOLN("only t");
    segments = 1;
    SERIAL_ECHOLNPAIR("seconds=",seconds);
    SERIAL_ECHOLNPAIR("len_units=",len_units);
    SERIAL_ECHOLNPAIR("new_feed_rate_units=",new_feed_rate_units);
  }
#endif

  const float inv_segments = 1.0f / float(segments);
  const float segmentDistance = totalDistance * inv_segments;

  for (ALL_AXIES(i)) deltaCartesian[i] *= inv_segments;

  uint32_t until = millis() + 200UL;
  while (--segments) {
    if(millis()>until) {
      meanwhile();
      until = millis() + 200UL;
    }

    for(ALL_AXIES(i)) intermediatePos[i] += deltaCartesian[i];
    addSegment(intermediatePos, new_feed_rate_units, segmentDistance);
  }

  // guarantee we stop exactly at the destination (no rounding errors).
  addSegment(pos, new_feed_rate_units, segmentDistance);
}

/**
   This method assumes the limits have already been checked.
   This method assumes the start and end radius match.
   This method assumes arcs are not >180 degrees (PI radians)
   @input cx center of circle x value
   @input cy center of circle y value
   @input destination point where movement ends
   @input dir - ARC_CW or ARC_CCW to control direction of arc
   @input new_feed_rate speed to travel along arc
*/
void Planner::bufferArc(float cx, float cy, float *destination, char clockwise, float new_feed_rate_units) {
  // get radius
  float dx = axies[0].pos - cx;
  float dy = axies[1].pos - cy;
  float sr = sqrtf(dx * dx + dy * dy);

  // find angle of arc (sweep)
  float sa = atan3(dy, dx);
  float ea = atan3(destination[1] - cy, destination[0] - cx);
  float er = sqrtf(dx * dx + dy * dy);

  float da = ea - sa;
  if(clockwise == ARC_CW && da < 0)
    ea += 2 * PI;
  else if(clockwise == ARC_CCW && da > 0)
    sa += 2 * PI;
  da       = ea - sa;
  float dr = er - sr;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len1 = abs(da) * sr;
  float totalDistance  = sqrtf(len1 * len1 + dr * dr);  // mm

  const float seconds = totalDistance / new_feed_rate_units;
  int16_t segments   = seconds * SEGMENTS_PER_SECOND;
  segments = _MIN(segments, totalDistance / MIN_SEGMENT_LENGTH);
  segments = _MAX(segments, 1);

  float n[NUM_AXIES], scale;
  float a, r;
#if NUM_AXIES > 2
  float sz = axies[2].pos;
  float z  = destination[2];
#endif

  for(int16_t i = 0; i <= segments; ++i) {
    // interpolate around the arc
    scale = ((float)i) / ((float)segments);

    a = (da * scale) + sa;
    r = (dr * scale) + sr;

    n[0] = cx + cos(a) * r;
    n[1] = cy + sin(a) * r;
#if NUM_AXIES > 2
    n[2] = (z - sz) * scale + sz;
#endif
    // send it to the planner
    bufferLine(n, new_feed_rate_units);
  }
}

void Planner::estop() {
  // clear segment buffer
  block_buffer_nonbusy = block_buffer_planned = block_buffer_head = block_buffer_tail;
}

#if MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)
float Planner::limitPolargraphAcceleration(const float *target_position,const float *oldP,float maxAcceleration) {
  // Adjust the maximum acceleration based on the plotter position to reduce wobble at the bottom of the picture.
  // We only consider the XY plane.
  // Special thanks to https://www.reddit.com/user/zebediah49 for his math help.

  // if T is your target direction unit vector,
  float Tx = target_position[0] - oldP[0];
  float Ty = target_position[1] - oldP[1];
  float Rlen = sq(Tx) + sq(Ty);  // always >=0
  if(Rlen > 0) {
    // only affects XY non-zero movement.  Servo is not touched.
    Rlen = 1.0 / sqrtf(Rlen);
    Tx *= Rlen;
    Ty *= Rlen;

    // normal vectors pointing from plotter to motor
    float R1x = axies[0].limitMin - oldP[0];  // to left
    float R1y = axies[1].limitMax - oldP[1];  // to top
    float Rlen1 = 1.0 / sqrtf(sq(R1x) + sq(R1y));
    R1x *= Rlen1;
    R1y *= Rlen1;

    float R2x = axies[0].limitMax - oldP[0];  // to right
    float R2y = axies[1].limitMax - oldP[1];  // to top
    float Rlen2 = 1.0 / sqrtf(sq(R2x) + sq(R2y));
    R2x *= Rlen2;
    R2y *= Rlen2;

    // solve cT = -gY + k1 R1 for c [and k1]
    // solve cT = -gY + k2 R2 for c [and k2]
    float c1 = -GRAVITYmag * R1x / (Tx * R1y - Ty * R1x);
    float c2 = -GRAVITYmag * R2x / (Tx * R2y - Ty * R2x);

    // If c is negative, that means that that support rope doesn't limit the acceleration; discard that c.
    float cT = -1;
    if(c1 > 0 && c2 > 0) {
      cT = (c1 < c2) ? c1 : c2;
    } else if(c1 > 0) {
      cT = c1;
    } else if(c2 > 0) {
      cT = c2;
    }

    // The maximum acceleration is given by cT if cT>0
    if(cT > 0) {
      maxAcceleration = _MAX(_MIN(maxAcceleration, cT), (float)MIN_ACCELERATION);
    }
  }
  return maxAcceleration;
}
#endif

#ifdef HAS_JUNCTION_DEVIATION
float Planner::junctionDeviation(Segment *newBlock,float *delta,int movesQueued,float inverseCartesianDistance,float max_acceleration) {
  /**
   * Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
   * Let a circle be tangent to both previous and current path line segments, where the junction
   * deviation is defined as the distance from the junction to the closest edge of the circle,
   * colinear with the circle center. The circular segment joining the two paths represents the
   * path of centripetal acceleration. Solve for max velocity based on max acceleration about the
   * radius of the circle, defined indirectly by junction deviation. This may be also viewed as
   * path width or max_jerk in the previous Grbl version. This approach does not actually deviate
   * from path, but used as a robust way to compute cornering speeds, as it takes into account the
   * nonlinearities of both the junction angle and junction velocity.
   *
   * NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path
   * mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
   * stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
   * is exactly the same. Instead of motioning all the way to junction point, the machine will
   * just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
   * a continuous mode path, but ARM-based microcontrollers most certainly do.
   *
   * NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
   * changed dynamically during operation nor can the line move geometry. This must be kept in
   * memory in the event of a feedrate override changing the nominal speeds of blocks, which can
   * change the overall maximum entry speed conditions of all blocks.
   *
   * #######
   * https://github.com/MarlinFirmware/Marlin/issues/10341#issuecomment-388191754
   *
   * hoffbaked: on May 10 2018 tuned and improved the GRBL algorithm for Marlin:
        Okay! It seems to be working good. I somewhat arbitrarily cut it off at 1mm
        on then on anything with less sides than an octagon. With this, and the
        reverse pass actually recalculating things, a corner acceleration value
        of 1000 junction deviation of .05 are pretty reasonable. If the cycles
        can be spared, a better acos could be used. For all I know, it may be
        already calculated in a different place. */

  float unit_vec[NUM_MOTORS] = {
    #define COPY_1(NN) (delta[NN]) * inverseCartesianDistance,
    ALL_MOTOR_MACRO(COPY_1)
  };

  float vmax_junction_sqr;

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if (movesQueued && Planner::previous_nominal_speed_sqr > 1e-6) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    float junction_cos_theta = (-prev_unit_vec[0] * unit_vec[0]) 
                             + (-prev_unit_vec[1] * unit_vec[1])
                             + (-prev_unit_vec[2] * unit_vec[2]);

    // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
    if (junction_cos_theta > 0.999999f) {
      // For a 0 degree acute junction, just set minimum junction speed.
      vmax_junction_sqr = sq(float(MINIMUM_PLANNER_SPEED));
    } else {
      junction_cos_theta = _MAX(junction_cos_theta,-0.999999f); // Check for numerical round-off to avoid divide by zero.

      // Convert delta vector to unit vector
      float junction_unit_vec[NUM_MOTORS] = {
        #define COPY_2(NN) unit_vec[NN] - prev_unit_vec[NN],
        ALL_MOTOR_MACRO(COPY_2)
      };

      float m = normalize_junction_vector(junction_unit_vec);
      if(m==0) {
        vmax_junction_sqr = newBlock->nominal_speed_sqr;
      } else {
        const float junction_acceleration = limit_value_by_axis_maximum(newBlock->acceleration, junction_unit_vec,max_acceleration),
                    sin_theta_d2 = sqrtf(0.5f * (1.0f - junction_cos_theta)); // Trig half angle identity. Always positive.

        vmax_junction_sqr = junction_acceleration * junction_deviation * sin_theta_d2 / (1.0f - sin_theta_d2);

        #if defined(JD_HANDLE_SMALL_SEGMENTS)

          // For small moves with >135° junction (octagon) find speed for approximate arc
          if (newBlock->distance < 1 && junction_cos_theta < -0.7071067812f) {

            #if defined(JD_USE_MATH_ACOS)

              #error "TODO: Inline maths with the MCU / FPU."

            #elif defined(JD_USE_LOOKUP_TABLE)

              // Fast acos approximation (max. error +-0.01 rads)
              // Based on LUT table and linear interpolation

              /**
               *  // Generate the JD Lookup Table
               *  constexpr float c = 1.00751495f; // Correction factor to center error around 0
               *  for (int i = 0; i < jd_lut_count - 1; ++i) {
               *    const float x0 = (sq(i) - 1) / sq(i),
               *                y0 = acos(x0) * (i == 0 ? 1 : c),
               *                x1 = i < jd_lut_count - 1 ?  0.5 * x0 + 0.5 : 0.999999f,
               *                y1 = acos(x1) * (i < jd_lut_count - 1 ? c : 1);
               *    jd_lut_k[i] = (y0 - y1) / (x0 - x1);
               *    jd_lut_b[i] = (y1 * x0 - y0 * x1) / (x0 - x1);
               *  }
               *
               *  // Compute correction factor (Set c to 1.0f first!)
               *  float min = INFINITY, max = -min;
               *  for (float t = 0; t <= 1; t += 0.0003f) {
               *    const float e = acos(t) / approx(t);
               *    if (isfinite(e)) {
               *      if (e < min) min = e;
               *      if (e > max) max = e;
               *    }
               *  }
               *  fprintf(stderr, "%.9gf, ", (min + max) / 2);
               */
              static constexpr int16_t  jd_lut_count = 16;
              static constexpr uint16_t jd_lut_tll   = _BV(jd_lut_count - 1);
              static constexpr int16_t  jd_lut_tll0  = __builtin_clz(jd_lut_tll) + 1; // i.e., 16 - jd_lut_count + 1
              static constexpr float jd_lut_k[jd_lut_count] PROGMEM = {
                -1.03145837f, -1.30760646f, -1.75205851f, -2.41705704f,
                -3.37769222f, -4.74888992f, -6.69649887f, -9.45661736f,
                -13.3640480f, -18.8928222f, -26.7136841f, -37.7754593f,
                -53.4201813f, -75.5458374f, -106.836761f, -218.532821f };
              static constexpr float jd_lut_b[jd_lut_count] PROGMEM = {
                  1.57079637f,  1.70887053f,  2.04220939f,  2.62408352f,
                  3.52467871f,  4.85302639f,  6.77020454f,  9.50875854f,
                  13.4009285f,  18.9188995f,  26.7321243f,  37.7885055f,
                  53.4293975f,  75.5523529f,  106.841369f,  218.534011f };

              const float neg = junction_cos_theta < 0 ? -1 : 1,
                          t = neg * junction_cos_theta;

              const int16_t idx = (t < 0.00000003f) ? 0 : __builtin_clz(uint16_t((1.0f - t) * jd_lut_tll)) - jd_lut_tll0;

              float junction_theta = t * pgm_read_float(&jd_lut_k[idx]) + pgm_read_float(&jd_lut_b[idx]);
              if (neg > 0) junction_theta = RADIANS(180) - junction_theta; // acos(-t)

            #else

              // Fast acos(-t) approximation (max. error +-0.033rad = 1.89°)
              // Based on MinMax polynomial published by W. Randolph Franklin, see
              // https://wrf.ecse.rpi.edu/Research/Short_Notes/arcsin/onlyelem.html
              //  acos( t) = pi / 2 - asin(x)
              //  acos(-t) = pi - acos(t) ... pi / 2 + asin(x)

              const float neg = junction_cos_theta < 0 ? -1 : 1,
                          t = neg * junction_cos_theta,
                          asinx =       0.032843707f
                                + t * (-1.451838349f
                                + t * ( 29.66153956f
                                + t * (-131.1123477f
                                + t * ( 262.8130562f
                                + t * (-242.7199627f
                                + t * ( 84.31466202f ) ))))),
                          junction_theta = RADIANS(90) + neg * asinx; // acos(-t)

              // NOTE: junction_theta bottoms out at 0.033 which avoids divide by 0.

            #endif

          const float limit_sqr = (newBlock->distance * junction_acceleration) / junction_theta;
          vmax_junction_sqr= _MIN(vmax_junction_sqr, limit_sqr);
        }

        #endif // JD_HANDLE_SMALL_SEGMENTS
      }
    }

    // Get the lowest speed
    vmax_junction_sqr = _MIN(vmax_junction_sqr, newBlock->nominal_speed_sqr, previous_nominal_speed_sqr);
  } else {
    // Init entry speed to zero. Assume it starts from rest. Planner will correct this later.
    vmax_junction_sqr = 0;
  }

  #define COPY_3(NN) prev_unit_vec[NN] = unit_vec[NN];
  ALL_MOTOR_MACRO(COPY_3)
  
  return vmax_junction_sqr;
}
#endif

#ifdef HAS_CLASSIC_JERK
float Planner::classicJerk(Segment *newBlock,float *current_speed,int movesQueued) {
  CACHED_SQRT(nominal_speed,newBlock->nominal_speed_sqr);

  float safe_speed = nominal_speed;
  char limited     = 0;
  for (ALL_MUSCLES(i)) {
    const float jerk = fabs(current_speed[i]),
                maxj = max_jerk[i];
    if(jerk > maxj) {  // new current speed too fast?
      if(limited) {
        const float mjerk = maxj * nominal_speed;          // ns*mj
        if(jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;  // ns*mj/cs
      } else {
        safe_speed *= maxj / jerk;  // Initial limit: ns*mj/cs
        ++limited;
      }
    }
  }

  // what is the maximum starting speed for this segment?
  float vmax_junction;
  if(movesQueued > 0 && previous_nominal_speed_sqr > 1e-6) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.0f;
    limited = 0;

    CACHED_SQRT(previous_nominal_speed, previous_nominal_speed_sqr);

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = _MIN(newBlock->nominal_speed_sqr, previous_nominal_speed);
    float smaller_speed_factor = vmax_junction / previous_nominal_speed;
    // Now limit the jerk in all axes.
    for (ALL_MUSCLES(i)) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit  = previous_speed[i] * smaller_speed_factor;
      float v_entry = current_speed[i];
      if(limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ?  //                            coasting             axis reversal
          ((v_entry > 0 || v_exit < 0) ? (v_exit - v_entry) : _MAX(v_exit, -v_entry))
          :  // v_exit <= v_entry          coasting             axis reversal
          ((v_entry < 0 || v_exit > 0) ? (v_entry - v_exit) : _MAX(-v_exit, v_entry));

      if(jerk > max_jerk[i]) {
        v_factor *= max_jerk[i] / jerk;
        ++limited;
      }
    }

    if(limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve
    // faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if(previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      vmax_junction = safe_speed;
    }
  } else {
    vmax_junction = safe_speed;
  }
  
  previous_safe_speed = safe_speed;

  float vmax_junction_sqr = sq(vmax_junction);

  return vmax_junction_sqr;
}
#endif

// Instantly move the virtual plotter position.  Does not check if the move is valid.
// @input cartesianPosition NUM_AXIES number of values.
void Planner::teleport(float *cartesianPosition) {
  planner.zeroSpeeds();

  // remember cartesian position
  for(ALL_AXIES(i)) axies[i].pos = cartesianPosition[i];
  // get step count
  int32_t steps[NUM_MUSCLES];
  IK(cartesianPosition, steps);
  // remember step count
  for(ALL_MUSCLES(i)) position[i] = steps[i];
  motor.set_step_count(steps);
}
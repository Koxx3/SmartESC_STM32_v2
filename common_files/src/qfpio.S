@ Copyright 2015 Mark Owen
@ http://www.quinapalus.com
@ E-mail: qfp@quinapalus.com
@
@ This file is free software: you can redistribute it and/or modify
@ it under the terms of version 2 of the GNU General Public License
@ as published by the Free Software Foundation.
@
@ This file is distributed in the hope that it will be useful,
@ but WITHOUT ANY WARRANTY; without even the implied warranty of
@ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@ GNU General Public License for more details.
@
@ You should have received a copy of the GNU General Public License
@ along with this file.  If not, see <http://www.gnu.org/licenses/> or
@ write to the Free Software Foundation, Inc., 51 Franklin Street,
@ Fifth Floor, Boston, MA  02110-1301, USA.

.syntax unified
.cpu cortex-m0
.thumb

@ exported symbols

.global qfp_float2str
.global qfp_str2float

@ C code in comments is intended to give an idea of the function
@ of the following assembler code. The translation is not exact.

@ // multiply by 128/125: used by conversions in both directions
@ unsigned int div125(unsigned int u) {
@   unsigned int a,b,c,k0=0x4189; // 0x4189~=128/125 Q14
@   a=u>>14;
@   a=a*k0;               // calculate first approximation to answer, good to about 14 bits
@   b=((a>>1)+(a>>2))>>4;
@   b=a-(b>>1)-(b&1);     // find error in approximation
@   c=(u-b)*k0;
@   return a+(c>>14)+1;   // result good to about 28 bits
@   }

div125:
 push {r1-r4,r14}
 ldr r4,=#0x4189  @ k0=0x4189;
 lsrs r1,r0,#14   @ a=u>>14;
 muls r1,r4       @ a=a*k0;
 lsrs r2,r1,#1    @ a>>1
 lsrs r3,r1,#2    @ a>>2
 add r2,r3        @ (a>>1)+(a>>2)
 lsrs r2,#4       @ b=((a>>1)+(a>>2))>>4;
 subs r0,r1       @ u-a
 lsrs r2,#1       @ b>>1
 adcs r0,r2       @ u-a+(b>>1)+(b&1)
 muls r0,r4       @ c=(u-b)*k0;
 lsrs r0,#14      @ c>>14
 add r0,r1        @ a+(c>>14)
 adds r0,#1       @ a+(c>>14)+1
 pop {r1-r4,r15}

.ltorg

opoint: @ output decimal point
 adds r5,#2
 movs r3,#'.'
 b och
ozero: @ output '0'
 movs r3,#0
odig:  @ output one digit from r3
 adds r3,#'0'
och:   @ output one character from r3
 strb r3,[r1]
 adds r1,#1
 bx r14

naninf: @ r4=0 for Inf, otherwise NaN
 ldr r3,=#0x00666e49 @ "fnI"
 cmp r4,#0
 beq 1f
 ldr r3,=#0x004e614e @ "NaN"
1:
 bl och
 lsrs r3,#8
 bne 1b
 b 10f

@ fmt is format control word:
@ b7..b0: number of significant figures
@ b15..b8: -(minimum exponent printable in F format)
@ b23..b16: maximum exponent printable in F format-1
@ b24: output positive mantissas with ' '
@ b25: output positive mantissas with '+'
@ b26: output positive exponents with ' '
@ b27: output positive exponents with '+'
@ b28: suppress traling zeros in fraction
@ b29: fixed-point output: b7..0 give number of decimal places
@ default: 0x18060406
@ Note that if b28 is set (as it is in the default format value) the code will
@ write the trailing decimal point and zeros to the output buffer before truncating
@ the string. Thus it is essential that the output buffer is large enough to accommodate
@ these characters temporarily.
@
@ Overall accuracy is sufficient to print all exactly-representable integers up to 10^8 correctly
@ in 0x18160408 format.
@ 
@ void float2str(float f,char*s,unsigned int fmt) {

qfp_float2str:
 push {r4-r7,r14}

@   if(fmt==0) fmt=0x18060406; // default format

 cmp r2,#0
 bne 1f
 ldr r2,=#0x18060406
1:

@   i=*(int*)&f;
@   if(i&0x80000000) { // output sign of mantissa
@     *p++='-';
@     i&=0x7fffffff;
@   } else {
@     if(fmt&0x01000000) *p++=' ';
@     else if(fmt&0x02000000) *p++='+';
@     }

 movs r3,#'-'
 lsls r0,#1
 bcs 2f
 movs r3,#' '
 lsrs r4,r2,#25
 bcs 2f
 movs r3,#'+'
 lsrs r4,r2,#26
 bcc 3f
2:
 bl och
3:

@   e2=(i>>23)-127; // get binary exponent e2

 movs r4,#0
 lsrs r3,r0,#24
 beq 1f  @ treat zero case specially
 subs r3,#127

@   m=((i&0x7fffff)|0x800000)<<8; // get mantissa, restore implied 1, make Q31

 lsls r4,r0,#8
 cmp r3,#128
 beq naninf @ handle NaN/Inf cases
 adds r4,#1

@   if(e2==-127) {e2=0; m=0;} // flush denormals to zero

 movs r5,#1
 rors r4,r5
1:
 movs r0,r4

@ now binary exponent e2 in r3, mantissa in r0

@   e10=0;  // decimal exponent
@ overall plan is to manipulate m, e2 and e10 so as to take e2 to zero, while maintaining the
@ invariant m * 2^e2 * 10^e10

 movs r4,#0

@   while(e2>0) { // add 3 to e10, take 10 off e2, multiply m by 1024/1000=128/125
@     if(m>=0xf0000000) m>>=1,e2++;
@     m=div125(m);
@     e2-=10;
@     e10+=3;
@     } // now e2<=0

 b 2f
1:
 lsrs r5,r0,#28
 cmp r5,#0x0f
 blo 3f
 lsrs r0,#1
 adds r3,#1
3:
 bl div125
 subs r3,#10
 adds r4,#3
2:
 cmp r3,#0
 bgt 1b

@   while(e2<=-10) { // take 3 off e10, add 10 to e2, multiply m by 1000/1024=125/128
@     m0=(m>>5)+(m>>6);
@     m-=(m0>>1)+(m0&1); // *125/128, more accurate than using multiply instruction
@     e2+=10;
@     e10-=3;
@     } // now -10 < e2 <= 0

 b 2f
1:
 lsrs r5,r0,#5
 lsrs r6,r0,#6
 add r5,r6
 movs r6,#0
 lsrs r5,#1
 adcs r5,r6
 subs r0,r5
 subs r4,#3
2:
 adds r3,#10
 ble 1b
 subs r3,#10

@   m>>=1; // Q30; make sure m will not overflow

 lsrs r0,#1

@   while(e2<=-3) { // take 1 off e10, add 3 to e2, multiply m by 10/8
@     m0=m>>1;
@     m+=(m0>>1)+(m0&1); // *10/8
@     e2+=3;
@     e10--;
@     } // now -3 < e2 <=0

 b 2f
1:
 lsrs r5,r0,#1
 lsrs r5,#1
 adcs r0,r5
 subs r4,#1
2:
 adds r3,#3
 ble 1b
 subs r3,#3

@   while(e2<0) { // add 1 to e2, halve m
@     m>>=1; // *1/2
@     e2++;
@     } // now e2==0

 b 2f
1:
 lsrs r0,#1
2:
 adds r3,#1
 ble 1b
 subs r3,#1

@   if(m>=0x40000000) m>>=2; // convert Q30 to Q28
@   else {
@     m=(m<<1)+(m>>1)+(m&1); // multiply by 10 (maintaining accuracy) if result will not overflow, compensate e10
@     e10--;
@     }

 lsrs r5,r0,#30
 beq 1f
 lsrs r0,#2
 b 2f
1:
 lsls r5,r0,#1
 lsrs r0,#1
 adcs r0,r5
 subs r4,#1
2:

@ now all of binary exponent has been transferred to decimal exponent
@ we have 
@ r0: mantissa m, Q28, 1<=m<10
@ r1: output pointer
@ r2: format
@ r3: 0 (was binary exponent)
@ r4: decimal exponent e10

@   sf=fmt&0xff; // number of significant figures

 uxtb r3,r2 @ e2 is no longer used

@   ff=0; // flag to indicate that output is in "F" format (i.e., will not use "E" notation)

 movs r5,#0

@   d0=e10; // first digit output has significance 10^d0 wrt output '.'
@   d1=d0-sf; // last digit output has significance 10^(d1+1) wrt output '.'

 movs r6,r4
 subs r7,r6,r3

@ r0: mantissa m, Q28, 1<=m<10
@ r1: output pointer
@ r2: format
@ r3: sf
@ r4: decimal exponent e10
@ r5b0: ff
@ r6: d0
@ r7: d1

@   if(fmt&0x20000000) { // forced "F" output format?
@     d1=-(fmt&0xff)-1;
@     sf=d0-d1;
@     ff=1;
@     }

 push {r1,r2}
 lsrs r1,r2,#30
 bcc 1f
 mvns r7,r3
 subs r3,r6,r7
 movs r5,#1
1:

@   m0=0x08000000; // 0.5 Q28
@   for(i=1;i<sf;i++) { // calculate amount to add to m for decimal rounding
@     m0+=m0>>1; // multiply by 0.1
@     m0+=m0>>4;
@     m0+=m0>>8;
@     m0+=m0>>16;
@     m0>>=4;
@     }
@   m+=m0; // rounding

 push {r3}
 movs r1,#8
 lsls r1,#24
2:
 subs r3,#1
 ble 1f
 lsrs r2,r1,#1
 add r1,r2
 lsrs r2,r1,#4
 add r1,r2
 lsrs r2,r1,#8
 add r1,r2
 lsrs r2,r1,#16
 add r1,r2
 lsrs r1,#4
 b 2b
1:
 add r0,r1
 pop {r3}

@   if(m>=0xa0000000) { // has rounding pushed m to 10 (Q28)? if so, set to 1 and increment decimal exponent
@     m=0x10000000;
@     e10++;
@     d0++;
@     if((fmt&0x20000000)==0) d1++;
@     }

 lsrs r1,r0,#28
 cmp r1,#0x0a
 pop {r1,r2}
 blo 1f
 lsrs r0,r2,#30
 bcs 2f
 adds r7,#1
2:
 movs r0,#0x10
 lsls r0,#24
 adds r4,#1
 adds r6,#1
1:

@   if(d0>=-(int)((fmt>>8)&0xff)&&d0<(int)((fmt>>16)&0xff)) ff=1; // in range for F format?

 push {r4}
 lsrs r4,r2,#8
 uxtb r4,r4
 adds r4,r6
 blt 1f
 lsrs r4,r2,#16
 uxtb r4,r4
 cmp r6,r4
 bge 1f
 movs r5,#1
1:

@   if(!ff) d0=0,d1=-sf; // for E format we have one digit before the decimal point

 cmp r5,#0
 bne 1f
 movs r6,#0
 rsbs r7,r3,#0
1:

@ sf (r3) no longer used

@   f0=0; // flag to indicate whether we have we output a '.'

@ f0 in r5b1

@   if(d0<0) *p++='0',*p++='.',f0=1,i=-1; // value <1, so output "0."
@   else i=d0;

 mov r4,r6
 cmp r6,#0
 bge 1f
 bl ozero
 bl opoint
 movs r4,#0
 mvns r4,r4
1:

@   while(i>d0&&i>d1) *p++='0',i--; // output leading zeros before significand as necessary

2:
 cmp r4,r6
 ble 1f
 cmp r4,r7
 ble 1f
 bl ozero
 subs r4,#1
 b 2b
1:

@ d0 (r6) no longer used

@   for(;i>d1;i--) {          // now output digits of significand
@     *p++='0'+(m>>28);       // output integer part of Q28 value
@     m&=0x0fffffff;          // fractional part of Q28 value
@     m=(m<<1)+(m<<3);        // multiply by 10
@     if(i==0) *p++='.',f0=1; // output decimal point as significance goes through 10^0
@     }

2:
 cmp r4,r7
 ble 1f
 lsrs r3,r0,#28
 bl odig
 lsls r0,#4
 lsrs r0,#1
 lsrs r3,r0,#2
 add r0,r3
 subs r4,#1
 bcs 2b
 bl opoint
 b 2b
1:

@ m (r0) no longer used
@ d1 (r7) no longer used

@   for(;i>=0;i--) *p++='0'; // output remaining zeros of integer part

2:
 cmp r4,#0
 blt 1f
 bl ozero
 subs r4,#1
 b 2b
1:

@ i (r4) no longer used

@   if(f0) { // remove trailing zeros and decimal point?
@     if(fmt&0x10000000) while(p[-1]=='0') p--;
@     if(p[-1]=='.') p--;
@     *p=0;
@     }

 lsrs r4,r5,#2
 bcc 1f
 lsrs r4,r2,#29
 bcc 2f
3:
 subs r1,#1
 ldrb r4,[r1]
 cmp r4,#'0'
 beq 3b
 adds r1,#1
2:
 subs r1,#1
 ldrb r4,[r1]
 cmp r4,#'.'
 beq 4f
 adds r1,#1
4:
1:
 pop {r4}

@ now:
@ r0
@ r1: output pointer
@ r2: format
@ r3
@ r4: decimal exponent e10
@ r5b0: ff
@ r6:
@ r7:

@   if(!ff) { // output exponent?

 lsrs r5,#1
 bcs 10f

@     *p++='E';

 movs r3,#'E'
 bl och

@     if(e10<0) *p++='-',e10=-e10; // output exponent sign
@     else {
@            if(fmt&0x04000000) *p++=' ';
@       else if(fmt&0x08000000) *p++='+';
@       }

 cmp r4,#0
 bge 2f
 rsbs r4,#0
 movs r3,#'-'
 b 3f
2:
 movs r3,#' '
 lsrs r6,r2,#27
 bcs 3f
 movs r3,#'+'
 lsrs r6,r2,#28
 bcc 4f
3:
 bl och
4:

@     m=(e10*0xcd)>>11; // tens digit of exponent
@     *p++='0'+m;
@     e10-=m*10;        // units digit of exponent
@     *p++='0'+e10;

 movs r3,#0xcd
 muls r3,r4
 lsrs r3,#11
 movs r0,#10
 muls r0,r3
 bl odig
 subs r3,r4,r0
 bl odig

@     }

10:

@   *p++=0;

 movs r3,#0
 bl och

@   }

 pop {r4-r7,r15}





@ Convert string pointed to by p into float, stored at f. On failure
@ return 1; on success, return 0 and store pointer to first non-converted
@ character at endptr if endptr!=0.

@ #define ISDIG(x) ((x)>='0'&&(x)<='9')

isdig: @ convert ASCII to digit
 subs r2,#'0'
 cmp r2,#10 @ clear carry if digit
 bx r14

@ int str2float(float*f,char*p,char**endptr) {

qfp_str2float:

@   if(*p=='+') p++;
@   else if(*p=='-') sm=0x80000000,p++; // capture mantissa sign

 push {r0,r2,r4-r7,r14}
 movs r7,#0
 ldrb r2,[r1]
 cmp r2,#'+'
 beq 1f
 cmp r2,#'-'
 bne 2f
 movs r7,#1
1:
 adds r1,#1
2:
 movs r0,#0 @ mantissa
 movs r3,#0 @ f0: have we seen a '.'?
 movs r5,#0 @ exponent
 movs r6,#0 @ count of mantissa digits processed

@ r0: m
@ r1: input pointer
@ r3: f0
@ r5: e
@ r6: d
@ r7b0: sm
@ stack: output pointer, end pointer

@   for(;;) {
@     if(f0==0&&*p=='.') {f0=1; p++; continue;}
@     if(!ISDIG(*p)) goto l0; // break out on non-digit
@     if(m<0x10000000) { // accumulate digits (up to about 8 significant figures)
@       m=m*10+*p-'0';
@       if(f0==1) e--; // decrement exponent if we are past the decimal point
@     } else if(f0==0) e++; // just increment exponent after we have captured enough significance in m
@     d++;
@     p++;
@     }
@ l0:

2:
 ldrb r2,[r1]
 cmp r2,#'.'
 bne 1f
 cmp r3,#0
 bne 1f
 movs r3,#1
 b 3f
1:
 bl isdig
 bcs 4f
 lsrs r4,r0,#28
 bne 5f
 movs r4,#10
 muls r0,r4
 add r0,r2
 subs r5,#1
5:
 adds r5,#1
 subs r5,r3
 adds r6,#1
3:
 adds r1,#1
 b 2b
4:

@   if(d==0) return 1; // no digits seen: error

 cmp r6,#0
 bne 1f
 movs r0,#1
 pop {r2-r7,r15}

@ f0 (r3) no longer used
@ d (r6) no longer used

@   e10=0; // decimal exponent

1:
 movs r3,#0

@   if(*p=='e'||*p=='E') { // exponent given?
@     se=0;
@     p++;
@     if(*p=='+') p++; 
@     else if(*p=='-') se=1,p++; // capture exponent sign
@     while(ISDIG(*p)) { // capture exponent digits
@       if(e10<0x01000000) e10=e10*10+*p-'0'; // prevent overflow
@       p++;
@       }
@     if(se) e10=-e10; // apply exponent sign
@     }

 mov r6,r1 @ save r1
 ldrb r2,[r1]
 cmp r2,#'e'
 beq 1f
 cmp r2,#'E'
 bne 2f
1:
 adds r1,#1
 ldrb r2,[r1]
 cmp r2,#'+'
 beq 3f
 cmp r2,#'-'
 bne 4f
 adds r7,#2 @ se in r7b1
3:
 adds r1,#1
 ldrb r2,[r1]
4:
 bl isdig
 bcc 6f
 mov r1,r6 @ E without following digits: restore r1
 b 2f
6:
 lsrs r4,r3,#24
 bne 5f
 movs r4,#10
 muls r3,r4
 add r3,r2
5:
 adds r1,#1
 ldrb r2,[r1]
 bl isdig
 bcc 6b
 cmp r7,#2
 blo 2f
 rsbs r3,#0
2:

@   if(m==0) goto l2; // zero? then we have finished

 movs r2,#0
 cmp r0,#0
 beq 11f

@   e10+=e; // offset e by captured exponent
@   if(e10> 127) e10=127; // clip overflows: 10^127 will be converted later to Inf, 10^-128 to zero
@   if(e10<-128) e10=-128;

 add r3,r5
 lsls r4,r3,#2 @ temporarily set e2 to e10*4: this will cause subsequent conversion to Inf/zero if required
 sxtb r5,r3
 cmp r5,r3
 bne 12f @ not equal to its sign-extended version?

@ e (r5) no longer used

@ r0: m
@ r1: input pointer
@ r3: e10
@ r7b0: sm
@ stack: output pointer, end pointer

@   e2=31; // binary exponent
@ plan is to manipulate m, e2 and e10 so as to take e10 to zero, while maintaining the
@ invariant m * 2^e2 * 10^e10

 movs r4,#31

@   while(m<0x40000000) m+=m,e2--; // normalise so m is now 0x40000000..0xa0000000

2:
 lsrs r2,r0,#30
 bne 1f
 lsls r0,#1
 subs r4,#1
 b 2b
1:

@   while(e10<0) { // add 3 to e10, take 10 off e2 and multiply m by 1024/1000=128/125
@     m=div125(m);
@     e10+=3; e2-=10;
@     if(m>=0x80000000) m>>=1,e2++;
@     } // now e10 >= 0

2:
 cmp r3,#0
 bge 1f
 bl div125
 adds r3,#3
 subs r4,#10
 lsrs r2,r0,#31
 beq 2b
 lsrs r0,#1
 adds r4,#1
 b 2b
1:

@   while(e10>2) { // take 3 off e10, add 10 to e2 and multiply m by 1000/1024=125/128
@     m0=(m>>6)+(m>>5);
@     m-=(m0>>1)+(m0&1); // *125/128
@     e10-=3; e2+=10;
@     } // now 0 <= e10 < 3

2:
 cmp r3,#2
 ble 1f
 lsrs r2,r0,#6
 lsrs r5,r0,#5
 add r2,r5
 movs r5,#0
 lsrs r2,#1
 adcs r2,r5
 subs r0,r2
 subs r3,#3
 adds r4,#10
 b 2b
1:

@   while(e10>0) { // take 1off e10, add 3 to e2 and multiply m by 10/8 = 5/4
@     m0=(m>>1);
@     m+=(m0>>1)+(m0&1); // *5/4
@     e10-=1; e2+=3;
@     } // now e10==0

2:
 cmp r3,#0
 ble 1f
 lsrs r2,r0,#1
 lsrs r2,#1
 adcs r0,r2
 subs r3,#1
 adds r4,#3
 b 2b
1:

@ e10 (r3) no longer used

@   while(m<0x80000000) m+=m,e2--; // renormalise m so MSB is set

 cmp r0,#0
 blt 1f
2:
 subs r4,#1
 adds r0,r0
 bpl 2b
1:

@   m=((m>>7)+1)>>1; // to 24 bits, with rounding

 lsrs r0,#7
 adds r0,#1
 lsrs r0,#1

@   if(m==0x01000000) m>>=1,e2++; // has rounding pushed m to 25 bits? renormalise if so

 lsrs r2,r0,#24
 beq 1f
 lsrs r0,#1
 adds r4,#1
1:

@   e2+=127; // add exponent offset

12:
 movs r2,#0
 movs r3,#0
 adds r4,#127

@   if(e2<=0) {m=0; goto l1;} // too small? flush to zero

 ble 10f

@   if(e2>=255) {m=0x7f800000; goto l1;} // too big? make infinity

 movs r3,#255
 cmp r4,#255
 bge 10f

@   m&=0x007fffff; // remove implied 1

 lsls r2,r0,#9
 lsrs r2,#9
 mov r3,r4

@   m|=e2<<23; // insert exponent bits

10:
 lsls r3,#23
 orrs r2,r3

@   m|=sm; // apply mantissa sign

11:
 lsls r7,#31
 orrs r2,r7

@   *f=*(float*)&m; // write output
@   if(end) *end=p;

 pop {r0,r3}
 str r2,[r0]
 cmp r3,#0
 beq 1f
 str r1,[r3]
1:

@   return 0;

 movs r0,#0
 pop {r4-r7,r15}

@   }

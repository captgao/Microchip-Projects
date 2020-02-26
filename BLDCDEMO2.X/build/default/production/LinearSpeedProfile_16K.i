# 1 "LinearSpeedProfile_16K.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:\\Program Files (x86)\\Microchip\\xc8\\v2.00\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "LinearSpeedProfile_16K.c" 2
# 61 "LinearSpeedProfile_16K.c"
# 1 "./BLDC.h" 1
# 198 "./BLDC.h"
typedef __bit bit;
void __attribute__((picinterrupt(""))) interrupt_handler(void);

void InitSystem(void);
void InitDriver(void);
void Commutate(void);
void ControlStartUp(void);
void StallControl(void);
void WarmUpControl(void);
void TimeBaseManager(void);
void ControlSlowStart(void);
void GetCCPVal(unsigned char speed);
unsigned char FindTableIndex(unsigned int duty_cycle);
unsigned long GetRPM(void);





typedef union {
   int word;
   struct {
      char low;
      char high;
   }bytes;
} doublebyte;
# 62 "LinearSpeedProfile_16K.c" 2
# 92 "LinearSpeedProfile_16K.c"
const int CCP_Values[256]={

    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 21, 23, 25, 27, 29,
    31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61,
    63, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92,
    94, 96, 98, 100,102,104,105,107,109,111,113,115,117,119,121,123,
    125,127,129,131,133,135,137,139,141,143,145,146,148,150,152,154,
    156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,
    188,189,191,193,195,197,199,201,203,205,207,209,211,213,215,217,
    219,221,223,225,227,229,230,232,234,236,238,240,242,244,246,248,
    250,252,254,256,258,260,262,264,266,268,270,271,273,275,277,279,
    281,283,285,287,289,291,293,295,297,299,301,303,305,307,309,311,
    313,314,316,318,320,322,324,326,328,330,332,334,336,338,340,342,
    344,346,348,350,352,354,355,357,359,361,363,365,367,369,371,373,
    375,377,379,381,383,385,387,389,391,393,395,396,398,400,402,404,
    406,408,410,412,414,416,418,420,422,424,426,428,430,432,434,436,
    438,439,441,443,445,447,449,451,453,455,457,459,461,463,465,467,
    469,471,473,475,477,479,480,482,484,486,488,490,492,494,496,500
};

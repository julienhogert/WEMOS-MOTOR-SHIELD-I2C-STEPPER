#define MSTEPS 32
#define MSTEPS4 (MSTEPS*4)
#define MSTEPS2 (MSTEPS/1)
//#define MOTOR_STEPS 64
#define MST_REV  1600000
//static  int wave[]={255,249,243,237,231,225,219,214,208,203,198,192,187,182,177,172,167,162,157,152,147,142,137,132,128,122,117,112,107,102,96,91,85,79,73,66,59,50,41,29,0};//
//{255,250,244,236,225,212,197,180,162,142,120,98,74,50,25,12,0};
//{255,65,61,56,52,47,43,39,34,30,26,21,17,13,8,4,0};
//{255,65,61,52,43,34,26,17,0};
//{255,255,255,255,0,0,0,0,0};
static int wave[]={0,13,25,37,50,62,74,86,98,109,120,131,142,152,162,171,180,189,197,205,212,219,225,231,236,240,244,247,250,252,253,254,255};
//static  int  wave[]={0,50,98,142,180,212,236,250,255};
//static  unsigned int  wave[]={0,180,255};
//static char steps[]={0x1f, 0x3f, 0x2f, 0x6f, 0x4f, 0xcf, 0x8f, 0x9f, 0x1f};

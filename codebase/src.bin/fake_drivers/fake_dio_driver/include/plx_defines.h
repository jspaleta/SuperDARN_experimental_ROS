#define VENDOR_ID 0x494f
//#define DEVICE_ID 0x0c60
#define DEVICE_ID 0x0c78

#define BEAMTABLE_DIR  "/tmp/"
#define RADAR_NAME     "kansas"
#define MAX_BEAM       22
#define BEAMCODES      8192
#define PA_GRP_0                0
#define PB_GRP_0                1
#define PC_GRP_0                2
#define CNTRL_GRP_0             3
#define PA_GRP_1                4
#define PB_GRP_1                5
#define PC_GRP_1                6
#define CNTRL_GRP_1             7
#define PA_GRP_2                8
#define PB_GRP_2                9
#define PC_GRP_2                10
#define CNTRL_GRP_2             11
#define PA_GRP_3                12
#define PB_GRP_3                13
#define PC_GRP_3                14
#define CNTRL_GRP_3             15
#define PA_GRP_4                16
#define PB_GRP_4                17
#define PC_GRP_4                18
#define CNTRL_GRP_4             19

#define SWITCHES 0
#define ATTEN    1
#define READ     0
#define WRITE    1
#define ON       1
#define OFF      0
#define R1_ADDRESS      0
#define R2_ADDRESS      2
#define RXFE_ADDRESS    4

#define ATTENS_OFF      15
#define ATTENS_ON        0
#define STAGE1_ON       16
#define STAGE1_OFF       0
#define IF_ON           32
#define IF_OFF           0
#define STAGE2_ON       64
#define STAGE2_OFF       0
#define STAGE3_ON      128
#define STAGE3_OFF       0

/*struct DIO_TCP_MSG{
        int channel;
        int address;
        int beam;
        int TM;
        int AGC_status;
        int LP_status;
        int status;
        int freq;
};*/


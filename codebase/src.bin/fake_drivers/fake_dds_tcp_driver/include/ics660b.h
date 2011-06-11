
/* Details of the ICS-660B card */
#ifndef ics660_h
#define ics660_h

#define DEBUG 0

#define STATE_TIME 5.e-6
#define SIGLEN 518400
#define GOOD 0
#define BAD -1
#define ENABLE 0x00000001
#define DISABLE 0x00000000
#define INTERNAL 0x00000000
#define EXTERNAL 0x00000001
#define PCI 0x00000000
#define FPDP 0x00000001

#define DEVICE_ID 0x0004
#define VENDOR_ID 0x11b0

#define M2BASE_OFFSET 0x200000
#define M2BASE2_OFFSET 0x300000
#define RESET_VALUE 0xffffffff
#define IMASK_VALUE 0x00000000;
#define REG_BLOCK_SIZE 256
#define ICS660_MEM_SIZE 8388608
#define CLOCK_FREQ 100.0e6

#define BOARD_RESET 0x0000
#define CLEAR_IMASK 0x000a
#define DAC_RESET 0x0004
#define STATUS 0x0008
#define INTERRUPT_MASK 0x000c
#define CONTROL 0x0010
#define TRANS_LENGTH 0x0014
#define BANK_LENGTH 0x0018
#define FPDP_FRAME_LENGTH 0x001c
#define PERSISTENCE 0x0040
#define PERSISTENCE_REG 0x1040
#define BANK_WIDTH 0x0044

#define TRIGGER_SOURCE 0x00000001
#define CLOCK_SELECT 0x00000002
#define MODE 0x0000000c
#define UPCONVERTER_SELECT 0x00000010
#define DATA_SOURCE 0x00000020
#define FPDP_CLOCK_SEL 0x00000040
#define FPDP_TERM 0x00000080
#define SYNC_MASTER 0x00000100
#define SYNC_TERM 0x00000200
#define DAC_ENABLE 0x00000400
#define TRIGGER 0x00000800

#define _MODULE_CONTROL_REG 0x00002001
#define _DC60M_RESET 0x00002002
#define _MISCELANEOUS 0x00002003
#define _SYNC_MODE 0x00002004
#define _INTERPOLATION_MODE 0x00002005
#define _CHANNEL_A_SYNC 0x00002006
#define _CHANNEL_B_SYNC 0x00002007
#define _CHANNEL_C_SYNC 0x00002008
#define _CHANNEL_D_SYNC 0x00002009
#define _INTERPOLATION_GAIN 0x0000200a
#define _INTERPOLATION_BYTE0 0x0000200b
#define _INTERPOLATION_BYTE1 0x0000200c
#define _COUNTER_BYTE0 0x0000200d
#define _COUNTER_BYTE1 0x0000200e
#define _DC_STAT 0x0000100f
#define _SET_PAGE_REG 0x00002010
#define _WRITE_SEQUENCER 0x00002011
#define _CHANNEL_FLUSH_MODE 0x00002012

#define CONTINUOUS 0x0000000c
#define PULSE 0x00000008
#define ONE_SHOT 0x00000004
#define LOOP 0x00000000

#define SET_CHIP 0x000000a0
#define SET_PAGE 0x000000a6
#define LOAD_FREQUENCY 0x000000a1
#define LOAD_PHASE 0x000000a2
#define LOAD_FILTER 0x000000a3
#define SET_DC_READY 0x000000a4
#define RELEASE_RESETS 0x000000a5

#define ICS660_SET_CHIP __DIOT(_DCMD_MISC,SET_CHIP,int)
#define ICS660_SET_PAGE __DIOT(_DCMD_MISC,SET_PAGE,int)
#define ICS660_LOAD_FREQUENCY __DIOT(_DCMD_MISC,LOAD_FREQUENCY,struct ICS660_FREQ)
#define ICS660_LOAD_PHASE __DIOT(_DCMD_MISC,LOAD_PHASE,struct ICS660_PHASE)
#define ICS660_LOAD_FILTER __DIOT(_DCMD_MISC,LOAD_FILTER,struct ICS660_FILTER)
#define ICS660_BANK_WIDTH __DIOT(_DCMD_MISC,BANK_WIDTH,int)
#define ICS660_BANK_LENGTH __DIOT(_DCMD_MISC,BANK_LENGTH,int)
#define ICS660_TRANS_LENGTH __DIOT(_DCMD_MISC,TRANS_LENGTH,int)
#define ICS660_BOARD_RESET __DIOT(_DCMD_MISC,BOARD_RESET,int)
#define ICS660_CLEAR_IMASK __DIOT(_DCMD_MISC,CLEAR_IMASK,int)
#define ICS660_DAC_RESET __DIOT(_DCMD_MISC,DAC_RESET,int)
#define ICS660_STATUS __DIOT(_DCMD_MISC,STATUS,int)
#define ICS660_PERSISTENCE_REG __DIOT(_DCMD_MISC,PERSISTENCE_REG,int)
#define ICS660_SET_DC_READY __DIOT(_DCMD_MISC,SET_DC_READY,int)
#define ICS660_RELEASE_RESETS __DIOT(_DCMD_MISC,RELEASE_RESETS,int)
#define ICS660_TRIGGER_SOURCE __DIOT(_DCMD_MISC,TRIGGER_SOURCE,int)
#define ICS660_CLOCK_SELECT __DIOT(_DCMD_MISC,CLOCK_SELECT,int)
#define ICS660_MODE __DIOT(_DCMD_MISC,MODE,int)
#define ICS660_UPCONVERTER_SELECT __DIOT(_DCMD_MISC,UPCONVERTER_SELECT,int)
#define ICS660_DATA_SOURCE __DIOT(_DCMD_MISC,DATA_SOURCE,int)
#define ICS660_FPDP_CLOCK_SEL __DIOT(_DCMD_MISC,FPDP_CLOCK_SEL,int)
#define ICS660_FPDP_TERM __DIOT(_DCMD_MISC,FPDP_TERM,int)
#define ICS660_SYNC_MASTER __DIOT(_DCMD_MISC,SYNC_MASTER,int)
#define ICS660_SYNC_TERM __DIOT(_DCMD_MISC,SYNC_TERM,int)
#define ICS660_DAC_ENABLE __DIOT(_DCMD_MISC,DAC_ENABLE,int)
#define ICS660_TRIGGER __DIOT(_DCMD_MISC,TRIGGER,int)

#define ICS660_MODULE_CONTROL_REG __DIOT(_DCMD_MISC,_MODULE_CONTROL_REG,uint32_t)
#define ICS660_DC60M_RESET  __DIOT(_DCMD_MISC,_DC60M_RESET,struct dc60m_p_val)
#define ICS660_MISCELANEOUS  __DIOT(_DCMD_MISC,_MISCELANEOUS,struct dc60m_p_val)
#define ICS660_SYNC_MODE  __DIOT(_DCMD_MISC,_SYNC_MODE,struct dc60m_p_val)
#define ICS660_INTERPOLATION_MODE  __DIOT(_DCMD_MISC,_INTERPOLATION_MODE,struct dc60m_p_val)
#define ICS660_CHANNEL_A_SYNC  __DIOT(_DCMD_MISC,_CHANNEL_A_SYNC,struct dc60m_p_val)
#define ICS660_CHANNEL_B_SYNC  __DIOT(_DCMD_MISC,_CHANNEL_B_SYNC,struct dc60m_p_val)
#define ICS660_CHANNEL_C_SYNC  __DIOT(_DCMD_MISC,_CHANNEL_C_SYNC,struct dc60m_p_val)
#define ICS660_CHANNEL_D_SYNC  __DIOT(_DCMD_MISC,_CHANNEL_D_SYNC,struct dc60m_p_val)
#define ICS660_CHANNEL_FLUSH_MODE  __DIOT(_DCMD_MISC,_CHANNEL_FLUSH_MODE,struct dc60m_p_val)
#define ICS660_INTERPOLATION_GAIN  __DIOT(_DCMD_MISC,_INTERPOLATION_GAIN,struct dc60m_p_val)
#define ICS660_INTERPOLATION_BYTE0  __DIOT(_DCMD_MISC,_INTERPOLATION_BYTE0,struct dc60m_p_val)
#define ICS660_INTERPOLATION_BYTE1  __DIOT(_DCMD_MISC,_INTERPOLATION_BYTE1,struct dc60m_p_val)
#define ICS660_COUNTER_BYTE0  __DIOT(_DCMD_MISC,_COUNTER_BYTE0,struct dc60m_p_val)
#define ICS660_COUNTER_BYTE1  __DIOT(_DCMD_MISC,_COUNTER_BYTE1,struct dc60m_p_val)
#define ICS660_DC_STAT  __DIOT(_DCMD_MISC,_DC_STAT,struct dc60m_p_val)
#define ICS660_SET_PAGE_REG  __DIOT(_DCMD_MISC,_SET_PAGE_REG,struct dc60m_pg_reg)
#define ICS660_WRITE_SEQUENCER  __DIOT(_DCMD_MISC,_WRITE_SEQUENCER,struct ICS660_sequencer)


#define SYNC_MODE 0x0000
#define INTERPOLATION_MODE 0x0004
#define INTERPOLATION_GAIN 0x0008
#define INTERPOLATION_BYTE0 0x000c
#define INTERPOLATION_BYTE1 0x0010
#define RESET 0x0014
#define COUNTER_BYTE0 0x0018
#define COUNTER_BYTE1 0x001c
#define CHANNEL_A_SYNC 0x0020
#define CHANNEL_B_SYNC 0x0024
#define CHANNEL_C_SYNC 0x0028
#define CHANNEL_D_SYNC 0x002c
#define CHANNEL_FLUSH_MODE 0x0030
#define MISCELLANEOUS 0x0034
#define STATUS_REG 0x0038
#define PAGE 0x003c
#define INPUT_REGISTER_0 0x0040

#define NTAPS 31

struct ics660_registers
{
  int board_reset;
  int dac_reset;
  int status;
  int interrupt_mask;
  int control;
  int trans_length;
  int bank_length;
  int fpdp_frame_length;
  int persistence;
  int bank_width;
};

struct pci_base
{
  uint32_t io_base;
  uint32_t base0;
  uint32_t base1;
};

struct dc60m_registers
{
  uint32_t sync_mode;
  uint32_t interpolation_mode;
  uint32_t interpolation_gain;
  uint32_t interpolation_byte0;
  uint32_t interpolation_byte1;
  uint32_t reset;
  uint32_t counter_byte0;
  uint32_t counter_byte1;
  uint32_t channel_a_sync;
  uint32_t channel_b_sync;
  uint32_t channel_c_sync;
  uint32_t channel_d_sync;
  uint32_t channel_flush_mode;
  uint32_t miscelaneous;
  uint32_t stat;
  uint32_t page;
  uint32_t input_registers[16];
};

struct dc60m_p_val
{
  uint32_t chip;
  uint32_t value;
};

struct dc60m_pg_reg
{
  uint32_t chip;
  uint32_t page;
  uint32_t reg;
  uint32_t value;
};

struct ics660b
{
  volatile unsigned char *mem0, *mem1;
  uint32_t pci_io;
  struct ics660_registers regs;
  struct dc60m_registers dc60m_regs;
  uint32_t *mod_control_reg;
};

struct ICS660_sequencer
{
  uint32_t offset;
  uint32_t value;
};

struct ICS660_FILTER
{
  int chip;
  int channel;
  int f_corner;
  double state_time;
};

struct ICS660_FREQ
{
  int chip;
  int channel;
  double freq;
};

struct ICS660_PHASE
{
  int chip;
  int channel;
  double phase;
};


typedef union _ics660_devctl_msg {
  int tx;     /*Filled by client on send */
  int rx;     /*Filled by server on reply */
} data_t;

#endif


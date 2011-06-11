/* function to initialize a DC-60-M daughter card */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <sys/mman.h>
#ifdef __QNX__
  #include  <hw/pci.h>
  #include <devctl.h>
  #include <hw/inout.h>
#endif
#include "ics660b.h"

#define round(x) ((double)(x)-floor((double)(x))<(double)0.5?floor(x):ceil(x))

int dc60m_init(FILE *ics660, int interp_val)
{
  uint32_t control_value = 0;
  uint32_t sync_mode_value = 0;
  uint32_t interp0 = 0;
  uint32_t interp1 = 0;
  uint32_t interpolation_gain = 0;
  uint32_t cic_interp = 0;
  uint32_t reset_value = 0;
  uint32_t counter_mode1 = 0xffff;
  uint32_t counter_mode2 = 0xffff;
  uint32_t channel_sync = 0;
  uint32_t channel_flush = 0;
  uint32_t miscellaneous = 0;
  uint32_t status_control = 0;
  uint32_t page = 0;
  uint32_t page_reg_value = 0;
  uint32_t chip = 0;
  int i,status;
  int BIG_SHIFT, SCALE;
  float cicgain;
  struct dc60m_p_val dc60m_p;
  struct dc60m_pg_reg pg_reg;
//  printf("Inside dc60m_init\n");  
//    printf(" interp_val: %d\n",interp_val);  
#ifdef __QNX__
  pdebug("DC60M_INIT - entry\n");
  /* Set module control register */
  control_value = (uint32_t)0x08; //Selects PLL operating range
  if( interp_val < 11) control_value = control_value | (uint32_t)(((uint32_t)(interp_val % 7) & 0x03) << 5); //Sets special case of CIC interpolation when interp =8,9 or 10
  
  pdebug("DC60M_INIT - MODULE_CONTROL_REG %x %x\n",ICS660_MODULE_CONTROL_REG,control_value);
  ics660_set_parameter(ics660, (int)ICS660_MODULE_CONTROL_REG,&control_value,sizeof(control_value));
  
  /* initilize 4116 chips sync modes and interpolation modes */

  page = (uint32_t)0x0;
  ics660_set_parameter(ics660, (int)ICS660_SET_PAGE, &page,sizeof(page));

  for( chip=1; chip<=4; chip++){

    
    dc60m_p.chip = chip;
    ics660_set_parameter(ics660, (int)ICS660_SET_CHIP, &chip,sizeof(chip));

    /* reset chip */
    dc60m_p.value = 0xff;
    ics660_set_parameter(ics660, (int)ICS660_DC60M_RESET,&dc60m_p,sizeof(dc60m_p));

    /* set the miscellaneous register  to have OS_MODE=1*/
    dc60m_p.value = (uint32_t)0x00000080;
    ics660_set_parameter(ics660, (int)ICS660_MISCELANEOUS,&dc60m_p,sizeof(dc60m_p));

    if(chip <= 2 ){
      dc60m_p.value = (uint32_t)0xe5;
    }else{
      dc60m_p.value = (uint32_t) 0x05;
    }
    ics660_set_parameter(ics660, (int)ICS660_SYNC_MODE,&dc60m_p,sizeof(dc60m_p));
  }

  for( chip=1; chip<=4; chip++){


    /* set page to 0 */
    dc60m_p.chip = chip;
    ics660_set_parameter(ics660, (int)ICS660_SET_CHIP, &chip,sizeof(chip));

    page = (uint32_t)0x0;
    ics660_set_parameter(ics660, (int)ICS660_SET_PAGE, &page,sizeof(page));

    dc60m_p.value = (uint32_t) 0xc4; // sets gain sync
    ics660_set_parameter(ics660, (int)ICS660_INTERPOLATION_MODE,&dc60m_p,sizeof(dc60m_p));
    
    dc60m_p.value = (uint32_t) 0x00000055; // sets gain sync
    ics660_set_parameter(ics660, (int)ICS660_CHANNEL_A_SYNC,&dc60m_p,sizeof(dc60m_p));
    ics660_set_parameter(ics660, (int)ICS660_CHANNEL_B_SYNC,&dc60m_p,sizeof(dc60m_p));
    ics660_set_parameter(ics660, (int)ICS660_CHANNEL_C_SYNC,&dc60m_p,sizeof(dc60m_p));
    ics660_set_parameter(ics660, (int)ICS660_CHANNEL_D_SYNC,&dc60m_p,sizeof(dc60m_p));
    ics660_set_parameter(ics660, (int)ICS660_CHANNEL_FLUSH_MODE,&dc60m_p,sizeof(dc60m_p));

    /* calculate and set BIG_SHIFT and SCALE */
    BIG_SHIFT = (uint32_t)2;
    if( interp_val < 1449 ) BIG_SHIFT = (uint32_t)2; 
    if( interp_val < 182 ) BIG_SHIFT = (uint32_t)1; 
    if( interp_val < 23 ) BIG_SHIFT = (uint32_t)0; 
    
//    printf(" chip %d BIG_SHIFT: %d\n",chip,BIG_SHIFT);  
//    printf(" chip %d SCALE: %d\n",chip,SCALE);  
    SCALE = (uint32_t) floor(4.*log((double)interp_val)/log((double) 2.)) - 3. - 12.*BIG_SHIFT + 1;
//    SCALE = (uint32_t) floor(4.*log((double)interp_val)/log((double) 2.)) - 3. - 12.*BIG_SHIFT ;
    if( SCALE > 15 )SCALE=(uint32_t)15;

    cicgain = (float)pow((double)interp_val,(double)4.)*pow((double)2.,(double)-(SCALE+12*BIG_SHIFT+3));
    pdebug("dc60m  interp: %d  BIG_SHIFT:  %x  SCALE: %x  cicgain:  %f\n",interp_val,BIG_SHIFT,SCALE,cicgain);
    
    BIG_SHIFT = (BIG_SHIFT << 4) & 0x00000030;
    pdebug("dc60m  interp: %d  BIG_SHIFT | SCALE:  %x\n",interp_val,BIG_SHIFT | SCALE);
    fflush(stdout);

//    printf(" chip %d cicgain: %lf\n",chip,cicgain);  
    dc60m_p.value = (uint32_t)(BIG_SHIFT | SCALE);
    ics660_set_parameter(ics660, (int)ICS660_INTERPOLATION_GAIN,&dc60m_p,sizeof(dc60m_p));

    /* set the CIC interpolation */
    interp0 = 0x000000ff & (uint32_t)interp_val-1;
    dc60m_p.value = interp0;
    ics660_set_parameter(ics660, (int)ICS660_INTERPOLATION_BYTE0,&dc60m_p,sizeof(dc60m_p));

    interp1 = ((interp_val-1) >> 8) & 0x0000003f;
    dc60m_p.value = interp1;
    ics660_set_parameter(ics660, (int)ICS660_INTERPOLATION_BYTE1,&dc60m_p,sizeof(dc60m_p));
    
    /* set counter mode to ffff */
    dc60m_p.value = (uint32_t)0xffffffff;
    ics660_set_parameter(ics660, (int)ICS660_COUNTER_BYTE0,&dc60m_p,sizeof(dc60m_p));
    ics660_set_parameter(ics660, (int)ICS660_COUNTER_BYTE1,&dc60m_p,sizeof(dc60m_p));    
            
    /* initialize the status register to 00 */
    dc60m_p.value = (uint32_t)0x00;
    pdebug("DC60M_INIT - ICS660_DC_STAT chip %d  value %x\n",dc60m_p.chip,dc60m_p.value);
    ics660_set_parameter(ics660, (int)ICS660_DC_STAT,&dc60m_p,sizeof(dc60m_p));    

    /* set the paged registers */

    pg_reg.chip = chip;
    
    /* initialize gain settings for all channels */
    pg_reg.page = (uint32_t)0x02;
    for( i=0; i<4; i++ ){
       pg_reg.reg = (uint32_t)i;
       pg_reg.value = (uint32_t)(((float)0x50)*cicgain);
//       pg_reg.value = 110;
//       printf ("    chan: %d   input gain: %d\n",i,pg_reg.value);
       pdebug("DC60M_INIT - page %d  reg  %d  value %x  cicgain %f\n",pg_reg.page,pg_reg.reg,pg_reg.value,cicgain);
       ics660_set_parameter(ics660, (int)ICS660_SET_PAGE_REG,&pg_reg,sizeof(pg_reg));
     }      

    /* initialize the I/O control page */
    pg_reg.page = (uint32_t)0x05;
    /* set the channel input mode register to serial packed */
    pg_reg.reg = 0;
    pg_reg.value = (uint32_t)0x05;
    ics660_set_parameter(ics660, (int)ICS660_SET_PAGE_REG,&pg_reg,sizeof(pg_reg));

    /* set the sum I/O mode register */
    pg_reg.reg = 3;
    pg_reg.value = (uint32_t)0xb0;
//    pg_reg.value = (uint32_t)0xb8;
//    printf ("  Sum Scale: %d\n",(pg_reg.value>>3) & 0x7);
    ics660_set_parameter(ics660, (int)ICS660_SET_PAGE_REG,&pg_reg,sizeof(pg_reg));

    /* enable output */
    pg_reg.reg = 8;
    pg_reg.value = (uint32_t)0x55;
    ics660_set_parameter(ics660, (int)ICS660_SET_PAGE_REG,&pg_reg,sizeof(pg_reg));
    

    /* set page back to 0 */
    page = (uint32_t)0x0;
    ics660_set_parameter(ics660, (int)ICS660_SET_PAGE, &page,sizeof(page));

  }
  chip = 0;
  ics660_set_parameter(ics660, (int)ICS660_SET_CHIP, &chip,sizeof(chip));

  return(1);
#else
  return(1);
#endif

}

#include <stdlib.h>
#include "tsg.h"

struct TSGBuf* MakeTimeSeq(struct TSGprm *prm,int index)
{
  struct TSGbuf *buf=NULL;
  int flag,i;
/*  prm.nrang=nrang;         
  prm.frang=frang;         
  prm.rsep=rsep;          
  prm.smsep=smsep;
  prm.txpl=txpl; 
  prm.mpinc=mpinc;
  prm.mppul=mppul; 
  prm.nbaud=nbaud;
  prm.code=pcode;*/
/*  prm->pat=malloc(sizeof(int)*prm->mppul);
  for (i=0;i<prm->mppul;i++) prm->pat[i]=patn[i];*/

  prm->rtoxmin=360;
  prm->stdelay=2;
  prm->gort=1;
  prm->mlag=0;
//  for (i=0;i<prm->mppul;i++) printf("%d %d\n",i,prm->pat[i]);
//  printf("Entering TSGMake %p\n",buf);
  buf=TSGMake(prm,&flag);
  buf->index=index;
//  printf("Leaving TSGMake %p\n",buf);
  return buf;
}

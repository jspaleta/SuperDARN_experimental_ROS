#include "rtypes.h"
#include "iniparser.h"
#include "dictionary.h"
int send_aux_dict(int s,dictionary *aux_dict, int verbose ) { 
    char *secname_in_dict=NULL; // pointer into dict do not free or realloc
    void *dict_buf=NULL; // pointer into dict do not free or realloc
    char *dict_string=NULL; // pointer that should be free'd 
    unsigned int bufsize;
    int32 bytes;
    int32 nsecs;
    int i;
    if(dict_string!=NULL) free(dict_string);
    dict_string=NULL;

    if ( verbose ) printf("AUX Send: Dict\n");
    if ( verbose ) iniparser_dump_ini(aux_dict,stdout);
    dict_string=iniparser_to_string(aux_dict);
    if ( verbose ) printf("AUX Send: String\n");
    if ( verbose ) printf("%s",dict_string);
    bytes=strlen(dict_string)+1;
    send_data(s, &bytes, sizeof(int32));
    send_data(s, dict_string, bytes*sizeof(char));
    nsecs=iniparser_getnsec(aux_dict);
    send_data(s, &nsecs, sizeof(int32));
    for(i=0 ; i<nsecs;i++) {
        secname_in_dict=iniparser_getsecname(aux_dict,i);
        dict_buf=dictionary_getbuf(aux_dict,secname_in_dict,&bufsize);
        bytes=strlen(secname_in_dict)+1;
        send_data(s,&bytes,sizeof(int32));
        send_data(s,secname_in_dict,bytes);
        bytes=bufsize;
        send_data(s,dict_buf,bytes);
    }

    if(dict_string!=NULL) free(dict_string);
    if ( verbose ) printf("AUX Send: Done\n");
    return 0;
}

int recv_aux_dict(int s,dictionary **dict_p,int verbose) { 
    dictionary *aux_dict=NULL;
    char *dict_string=NULL; // pointer into dict do not free or realloc
    char *secname_in_dict=NULL; // pointer into dict do not free or realloc
    void *dict_buf=NULL; // pointer into dict do not free or realloc
    void *temp_buf=NULL; // point that needs to be malloc'd and free'd
    char secname_static[200]; 	
    char entry[200]; 	
    unsigned int bufsize;
    int32 bytes;
    int32 nsecs;
    int i;


    if ( verbose ) printf("AUX Recv: start\n");
    recv_data(s, &bytes, sizeof(int32));
    if ( verbose ) printf("AUX Recv: bytes %d\n",bytes);
    if(dict_string!=NULL) free(dict_string);
    dict_string=malloc(sizeof(char)*(bytes+10));
    if (verbose ) printf("AUX Recv: string malloced\n");
    recv_data(s, dict_string, bytes*sizeof(char));
    if ( verbose ) printf("AUX Recv: got string\n");
    if ( verbose ) printf("AUX Recv: String\n");
    if ( verbose ) printf("%s",dict_string);
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=iniparser_load_from_string(NULL,dict_string);
    nsecs=0;
    recv_data(s,&nsecs,sizeof(int32));
    for(i=0;i<nsecs;i++) {
        recv_data(s,&bytes,sizeof(int32));
        recv_data(s,secname_static,bytes);
        bytes=iniparser_getbufsize(aux_dict,secname_static);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=malloc(bytes);
        recv_data(s,temp_buf,bytes);
        iniparser_setbuf(aux_dict,secname_static,temp_buf,bytes);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=NULL;
    }
    *dict_p=aux_dict;
    if(dict_string!=NULL) free(dict_string);
    dict_string=NULL;
    if ( verbose ) printf("AUX Recv: Dict\n");
    if ( verbose ) iniparser_dump(*dict_p,stdout);
    if ( verbose ) printf("AUX Recv: Dict\n");
    if ( verbose ) iniparser_dump(aux_dict,stdout);
    if ( verbose ) printf("AUX Recv: Done %u %u\n",aux_dict,*dict_p);
    return 0;
}

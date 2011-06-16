int send_aux_dict(int s,dictionary *aux_dict); 
    char *secname_in_dict=NULL // pointer into dict do not free or realloc
    void *dict_buf=NULL // pointer into dict do not free or realloc
    char *dict_string=NULL // pointer that should be free'd 
    unsigned int bufsize;
    int32 bytes;
    int32 nsecs;

    if(dict_string!=NULL) free(dict_string);
    dict_string=NULL:

    dict_string=iniparser_to_string(aux_dict);
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
    printf("AUX Command sent %p\n",aux_dict);

    if(dict_string!=NULL) free(dict_string);
}

int recv_aux_dict(int s,dictionary *aux_dict); 
    char *dict_string=NULL // pointer into dict do not free or realloc
    char *secname_in_dict=NULL // pointer into dict do not free or realloc
    void *dict_buf=NULL // pointer into dict do not free or realloc
    void *temp_buf=NULL // point that needs to be malloc'd and free'd
    char secname_static[200]; 	
    unsigned int bufsize;
    int32 bytes;
    int32 nsecs;

    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;
    if(dict_string!=NULL) free(dict_string);
    dict_string=NULL:
    if(temp_buf!=NULL) free(temp_buf);
    temp_buf=malloc(bytes);

    recv_data(s, &bytes, sizeof(int32));
    dict_string=malloc(sizeof(char)*(bytes+10));
    recv_data(s, dict_string, bytes*sizeof(char));
    aux_dict=iniparser_load_from_string(aux_dict,dict_string);
    nsecs=0;
    recv_data(s,&nsecs,sizeof(int32));
    for(i=0;i<nsecs;i++) {
        recv_data(s,&bytes,sizeof(int32));
        recv_data(s,secname_static,bytes);
        sprintf(entry,"%s:bytes",secname_static);
        bytes=iniparser_getint(aux_dict,entry,0);
        recv_data(s,temp_buf,bytes);
        dictionary_setbuf(aux_dict,secname_static,temp_buf,bytes);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=NULL;
    }
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
    }

    if(dict_string!=NULL) free(dict_string);
    dict_string=NULL:
    if(temp_buf!=NULL) free(temp_buf);
    temp_buf=malloc(bytes);
}


int send_aux_command(int s,dictionary *aux_dict); 
  /* start of helper function code */
    dict_string=iniparser_to_string(aux_dict);
    bytes=strlen(dict_string)+1;
    smsg.type=AUX_COMMAND;
    smsg.status=1;
    send_data(s, &smsg, sizeof(struct ROSMsg));
    recv_data(s, &rmsg, sizeof(struct ROSMsg));
    if(rmsg.status==1) {
      printf("AUX Command is valid\n");
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
      if(aux_dict!=NULL) iniparser_freedict(aux_dict);
      aux_dict=NULL;
      printf("AUX Command send %p\n",aux_dict);
      recv_data(s, &bytes, sizeof(int32));
      if(dict_string!=NULL) free(dict_string);
      dict_string=malloc(sizeof(char)*(bytes+10));
      recv_data(s, dict_string, bytes*sizeof(char));
      if(aux_dict!=NULL) iniparser_freedict(aux_dict);
      aux_dict=NULL;
      aux_dict=iniparser_load_from_string(aux_dict,dict_string);
      free(dict_string);
      nsecs=0;
      recv_data(s,&nsecs,sizeof(int32));
      printf("DIO AUX Command nsecs %d\n",nsecs);
      for(i=0;i<nsecs;i++) {
        recv_data(s,&bytes,sizeof(int32));
        recv_data(s,secname_static,bytes);
        sprintf(entry,"%s:bytes",secname_static);
        bytes=iniparser_getint(aux_dict,entry,0);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=malloc(bytes);
        recv_data(s,temp_buf,bytes);
        dictionary_setbuf(aux_dict,secname_static,temp_buf,bytes);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=NULL;
      }
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
    }

   /* End of helper function code */
}


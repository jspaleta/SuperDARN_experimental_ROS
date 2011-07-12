typedef struct _recv{
  void *IOBASE;
  int pci_index;
} t_recv;

typedef struct _gps{
  void *IOBASE;
  int pci_index;
} t_gps;

typedef struct _dio{
  void *IOBASE;
  int pci_index;
} t_dio;


typedef struct _dds{
  void *IOBASE;
  int pci_index;
  int state_time_usec;
} t_dds;

typedef struct _timing{
  void *IOBASE;
  int pci_index;
  int state_time_usec;
} t_timing;


typedef struct _tcp{
  int port_number;
  char addr[80];
} t_tcp;

typedef union _info 
{
  t_recv recv;
  t_gps gps;
  t_dds dds;
  t_dio dio;
  t_timing timing;
} t_info;

typedef struct _driver
{
  t_info info;
  t_tcp	 tcp;
  char   type[80];
  int    max_seq_length;
  int    max_seqs;
  int    max_pulses;
} t_driver;

#define MIN(a,b)  ((a) < (b) ? (a) : (b))
#define MAX(a,b)  ((a) > (b) ? (a) : (b))

int send_data(int fd,void  *buf,size_t buflen);
int recv_data(int fd,void *buf,size_t buflen);
char* ltoa(int value, char* str, int radix); 


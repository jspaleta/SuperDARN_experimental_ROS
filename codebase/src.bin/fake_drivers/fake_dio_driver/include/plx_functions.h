#include "control_program.h"
int _select_beam(unsigned int base,struct ControlPRM *client);
int _select_card(unsigned int base,struct ControlPRM *client);
int _open_PLX9052(int *pci_handle, unsigned int *mmap_io_ptr, int *interrupt_line, int print);
int tcpsocket(int port);


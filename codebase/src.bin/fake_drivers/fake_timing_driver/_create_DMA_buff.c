#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/mman.h>

int _create_DMA_buff(unsigned long *virtual_addr, unsigned long *physical_addr, size_t length){

	int 		temp;
	size_t		contig_length;
	void		*v_addr;
	//unsigned int	p_addr;
#ifdef __QNX__
	off64_t		p_addr;
	// map and get 'virtual address' (address in calling processes memory space) for a zero filled memory region
	//	of size.  This space can be used for DMA transfer of data.
	//v_addr = mmap( 0, length, PROT_READ|PROT_WRITE|PROT_NOCACHE, MAP_PHYS|MAP_ANON|MAP_NOX64K, NOFD, 0);
	v_addr = mmap( 0, length, PROT_READ|PROT_WRITE|PROT_NOCACHE, MAP_PHYS|MAP_ANON, NOFD, 0);
	//v_addr = mmap( 0, length, PROT_READ|PROT_WRITE|PROT_NOCACHE, MAP_PHYS|MAP_ANON|MAP_BELOW16M, NOFD, 0);
	if (virtual_addr == MAP_FAILED){
		perror("Mapping of DMA buffer failed");
		return -1;
	}
	// now get the phsical address of this memory space to be provided to the DMA bus master (PLX9656) for
	//	DMA transfer of the data from the bus master
	//mem_offset( v_addr, NOFD, length, &p_addr, &contig_length);
	temp=mem_offset64( v_addr, NOFD, 1, &p_addr, 0);
	if (temp == -1){
		perror("Cannot determine physical address of DMA buffer");
		return -1;
	}
	
	*physical_addr=p_addr;
	*virtual_addr=v_addr;
	//printf("PHYSICAL ADDR= %x\n", *physical_addr);
	//printf("VIRTUAL ADDR=  %x\n", *virtual_addr);
	//printf("CONTIG_LENGTH= %d\n", contig_length);
	//printf("LENGTH=	       %d\n", length);
#endif
	return 1;
}

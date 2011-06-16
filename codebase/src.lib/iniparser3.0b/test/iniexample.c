#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "iniparser.h"

void create_example_ini_file(void);
int  parse_ini_file(char * ini_name);

int main(int argc, char * argv[])
{
	int		status ;

	if (argc<2) {
		create_example_ini_file();
		status = parse_ini_file("example.ini");
	} else {
		status = parse_ini_file(argv[1]);
	}
	return status ;
}

void create_example_ini_file(void)
{
	FILE	*	ini ;

	ini = fopen("example.ini", "w");
	fprintf(ini,
    "#\n"
    "# This is an example of ini file\n"
    "#\n"
    "\n"
    "[Pizza]\n"
    "\n"
    "Ham       = yes ; Ham comment\n"
    "Mushrooms = TRUE ;\n"
    "Capres    = 0 ;\n"
    "Cheese    = Non ;\n"
    "\n"
    "\n"
    "[Wine]\n"
    "\n"
    "Grape     = Cabernet Sauvignon ;\n"
    "Year      = 1989 ;\n"
    "Country   = Spain ;\n"
    "Alcohol   = 12.5  ;\n"
    "\n");
	fclose(ini);
}


int parse_ini_file(char * ini_name)
{
	dictionary	*	ini=NULL ;
	dictionary	*	test_dict=NULL;	
	/* Some temporary variables to hold query results */
	int				b ;
	int				i ;
	double			d ;
	char		*	s ;
	char		*out_str=NULL;
        fprintf(stderr,"Loading...\n");
	ini = iniparser_load(ini_name);
	if (ini==NULL) {
		fprintf(stderr, "cannot parse file: %s\n", ini_name);
		return -1 ;
	}
        fprintf(stderr,"Dumping...\n");
	iniparser_dump_ini(ini, stderr);
        fprintf(stdout,"To_stringi...\n");
	out_str=iniparser_to_string(ini);
        fprintf(stdout,"\nstring:\n");
        fprintf(stdout,"%s\n",out_str);
        fprintf(stdout,"From_string\n");
	test_dict=iniparser_load_from_string(test_dict,out_str);
        fprintf(stderr,"Dict from string:\n");
	iniparser_dump_ini(test_dict, stderr);

	/* Get pizza attributes */
	printf("Pizza:\n");

	b = iniparser_getboolean(ini, "pizza:ham", -1);
	printf("Ham:       [%d]\n", b);
	b = iniparser_getboolean(ini, "pizza:mushrooms", -1);
	printf("Mushrooms: [%d]\n", b);
	b = iniparser_getboolean(ini, "pizza:capres", -1);
	printf("Capres:    [%d]\n", b);
	b = iniparser_getboolean(ini, "pizza:cheese", -1);
	printf("Cheese:    [%d]\n", b);

	/* Get wine attributes */
	printf("Wine:\n");
	s = iniparser_getstring(ini, "wine:grape", NULL);
    printf("Grape:     [%s]\n", s ? s : "UNDEF");
	
    i = iniparser_getint(ini, "wine:year", -1);
    printf("Year:      [%d]\n", i);

	s = iniparser_getstring(ini, "wine:country", NULL);
    printf("Country:   [%s]\n", s ? s : "UNDEF");
	
    d = iniparser_getdouble(ini, "wine:alcohol", -1.0);
    printf("Alcohol:   [%g]\n", d);

	iniparser_freedict(ini);
	return 0 ;
}



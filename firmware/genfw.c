/*
 * Firmware generator for s5k4ecgx (5MP Camera) from Samsung
 * a quarter-inch optical format 1.4 micron 5 megapixel (Mp)
 * CMOS image sensor.
 * Copyright (C) 2012, Linaro, Sangwook Lee <sangwook.lee@linaro.org>
 *
 * HOW TO COMPILE
 * $ gcc -o genfw genfw.c -lz
 *
 */

#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <zlib.h>

typedef struct tagCatalogueRecord { 
  unsigned int addr;  
  unsigned short val; 

} __attribute__((packed)) CatalogueRecord; 


unsigned long getFileCRC(FILE *);
unsigned long calcCRC (const unsigned char *, signed long,
		       unsigned long, unsigned long *);
void makeCRCtable(unsigned long *, unsigned long);
int entry_number; /* Total register sets */
#define FW_FILE_NAME	"s5k4ecgx.bin"

unsigned long getFileCRC(FILE *s)
{
	/* File size */
	unsigned char *buf;
	unsigned long crc = ~0;
	size_t len;

	buf = malloc((entry_number+1)*6);
	if(!buf)
		return (unsigned long)buf;

//	crc = crc32(crc, Z_NULL, 0);
	while ( (len = fread(buf, 1, sizeof(buf), s)) != 0 )
		crc = crc32((crc ^ 0xffffffff),buf,len) ^ 0xffffffff;

	free(buf);

	return crc;
}

CatalogueRecord newRecord(int nCopies, int val) 
{ 
	CatalogueRecord record; 
	record.addr = nCopies; 
	record.val = 0; 

	return record; 
} 

/* printRecord: prints a single record */ 
int printRecord(CatalogueRecord record) 
{ 
	return 0; 
} 

#define FW_START_TOKEN                 '{'
#define FW_END_TOKEN                   '}'
/**
+ * parse_line() - parse a line from a s5k4ecgx file
+ *
+ * This function uses tokens to separate the patterns.
+ * The form of the pattern should be as follows:
+ * - use only C-sytle comments
+ * - use one "token address, value token" in a line
+ *       ex) {0xd002020, 0x5a5a}
+ * - address should be a 32bit hex number
+ * - value should be a 16bit hex number
+ */
static int parse_line(char *p1, int *found, CatalogueRecord *reg)
{
       int ret;
       char * p2 = p1;

       *found = 0;
       /* Skip leading white-space characters */
       while (isspace(*p2))
               p2++;

       while (*p2 != '\n') {
               /* Search for start token */
               if (*p2 == FW_START_TOKEN) {
                       p2++;
                       ret = sscanf(p2, "%x,%hx", &reg->addr, &reg->val);
                       if (ret != 2)
                               return -1;
                       /* Fast forward as searching for end token */
                       while (*p2 != FW_END_TOKEN && *p2 != '\n') {
                               p2++;
                       }
                       if (*p2 != FW_END_TOKEN)
                               return -1;
                       *found = 1;
               }
               p2++;
       }

       return 0;
}

static void parsing_regfile(char *file_in, char *file_out)
{
	char buf[120];
	CatalogueRecord reg, head;
	FILE *file_i, *file_o;
	int found, count = 0;

	file_i = fopen(file_in, "r");
	file_o = fopen(file_out, "wb"); 

	/* Make dummy head at first */
	head = newRecord(entry_number, 0);
	fwrite(&head, sizeof(CatalogueRecord), 1, file_o);

	while (fgets(buf, 120, file_i) != NULL) {
		parse_line(buf, &found, &reg);
		if (found) {
			count++;
		//	printf("Parsing: add: %#x val: %#x\n", reg.addr, reg.val);
			fwrite(&reg, sizeof(CatalogueRecord), 1, file_o); 
		}
	}

	/* for CRC memory allocation */ 
	entry_number = count;

	/* Update header */
	rewind(file_o);
	head = newRecord(count, 0); 
	fwrite(&head, sizeof(CatalogueRecord), 1, file_o);
	fclose(file_i);
	fclose(file_o);

	printf("Parsing done:\n"
			"            total %d register sets found\n", count);

}

int main(int argc, char *argv[]) 
{
	unsigned long crc;
	int ret;
	char *fileName = FW_FILE_NAME;
	FILE *file_crc; 

	if (argc != 2) {
		printf("Usage: %s input_file \n", argv[0]);
		return -1;
	}

	printf("Making firmware: %s\n", fileName);
	parsing_regfile(argv[1], fileName);

	file_crc = fopen(fileName, "rb"); 
	crc = getFileCRC(file_crc);
	printf("Got CRC value %#lx (size %ld)\n", crc, sizeof(unsigned long));
	fclose(file_crc); 

	printf("Try to append CRC to %s\n", fileName);
	// Open for append
	file_crc = fopen(fileName, "a+");
	/* Make 4 bytes CRC */ 
	fwrite(&crc, 4, 1, file_crc);
	fclose(file_crc);
	printf("Done\n"); 

	return 0; 
}

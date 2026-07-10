#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

int main (int argc, char *argv[]) 
{
  FILE * in;
  struct stat s;
  int start_address;
  int i;
  char checksum;
  char ch;

  if (argc != 3) {
    fprintf(stderr, "Wrong number of arguments. Program need two parameters. Filename of input file and start address.\n");
    fprintf(stderr, "bin2abs <file name of binary file> <start address>\n");
    fprintf(stderr, "Program will output to stdout.\n");
    exit(1);
  }
  in = fopen (argv[1], "r");
  start_address = strtol(argv[2], NULL, 0);

  fstat(fileno(in), &s);
  
  putchar(0x01);
  checksum += 0x01;
  putchar(0x00);
  putchar((s.st_size+6)&0xff);
  checksum += (s.st_size+6)&0xff;
  putchar(((s.st_size+6)>>8)&0xff);
  checksum += ((s.st_size+6)>>8)&0xff;
  putchar(start_address & 0xff);
  checksum += start_address & 0xff;
  putchar((start_address>>8) & 0xff);
  checksum += (start_address>>8) & 0xff;
  


  for (i=0; i < s.st_size; i++) {
    ch = fgetc(in);
    putchar(ch);
    checksum += ch;
  }

  putchar(-checksum);

  putchar(0x01);
  putchar(0x00);
  putchar(0x06);
  putchar(0x00);
  putchar(0x01);
  putchar(0x00);
  putchar(0xf8);

  return 0;
}

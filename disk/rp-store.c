#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "devtable.h"

void main(int argc, char* argv[]) {
    

    FILE* card;
    FILE* in;

    char devname[100];
    char* buf;
    int bank=0,unit=0;
    char filename[100];
    int id;
    int i;
    uint32_t boffset, doffset, usize, cardoffset, realsize;
    char DD[4];
    char* outbuf;    
    int cyl;
    int sec;
    int adr;
    int pblock=0;

    if (argc < 5) {
        printf("Запись образа RP диска на SD-карту\n\
Формат командной строки:\n\
%s DEV TYPE unit bank [file]\n\n\
DEV   - имя устройства SD-карты, например /dev/sdd\n\
TYPE  - тип диска, RP\n\
unit  - номер устройства - 0\n\
bank  - номер банка дисков\n\
file  - имя файла для записи извлекаемого образа, по умолчанию B#-DD#.DSK\n\n\
 Пример: %s /dev/sdc RP 0 0 BASIC.TAP - запись образа RP: в банк 0\n\n",argv[0],argv[0]);
        return;
    }

    DD[0]=toupper((argv[2][0]));
    DD[1]=toupper((argv[2][1]));
    DD[3]=0;

    // поиск устройства
    for(id=0;;id++) {
        if (devtable[id].name == 0) {
            printf("Неверный тип устройства - %s",DD);
            return;
        }
        if (strncmp(DD,devtable[id].name,2) == 0) {
            doffset=devtable[id].doffset;
            usize=devtable[id].usize;
            realsize=devtable[id].realsize;
            break;
        }     
    }
    unit=atoi(argv[3]);
    bank=atoi(argv[4]);                      
    if (argc>5) strcpy(filename,argv[5]);
    else sprintf(filename,"B%i-%s%i.dsk",bank,devtable[id].name,unit);

    if (unit > devtable[id].maxdev) {
        printf("Недопустимый номер устройства - %i, максимально допустимый = %i",unit, devtable[id].maxdev);
        return;
    }    

    in=fopen(filename,"r");
    if (in == 0) {
        printf("- ошибка открытия входного файла %s\n",filename);
        return;
    }
    card=fopen(argv[1],"r+");
    if (card == 0) {
        printf("- ошибка открытия SD-карты %s\n",argv[1]);
        return;
    }

    buf = malloc(usize*512) ;
    if (buf == 0) {
        printf("- недостаточно памяти под буфер\n");
        return;
    }

    memset(buf, 0, usize*512) ;
    buf[1] = 0xFF ;

    cardoffset = bank * banksize + doffset + unit * usize ;
    fseek(card, (cardoffset)*512, SEEK_SET) ;

    while (fread(buf, 1, 1, in) == 1) {
        fwrite(buf, 1, 2, card) ;
    }

    buf[0] = 0 ;
    buf[1] = 0 ;
    fwrite(buf, 1, 2, card) ;

    fclose(in);
    fclose(card);
    free(buf);

    printf("* Стартовый блок: %xh     Размер: %xh\n",cardoffset,usize);
}

#include "spi_io.h"

unsigned char cal_crc_table(unsigned char *ptr, unsigned char len) 
{
    unsigned char  crc = 0x00;
 
    while (len--)
    {
        crc = crc_table[crc ^ *ptr++];
    }
    return (crc);
}

void spi_open_file(spi_file_t *fd, const char *file)
{
    *fd = open(file, O_RDWR|O_NONBLOCK);
}

void spi_close_file(spi_file_t *fd)
{
    close(*fd);
}

void spi_write(spi_file_t *file, spi_io_write_t *writed)
{
    // dac 
    unsigned char byte_3, byte_4;
    byte_3 = writed->DAC_data >> 8;
    byte_4 = writed->DAC_data & 0xff;

    // // sample
    unsigned char byte_5, byte_6;
    byte_5 = writed->QEI_sample >> 8;
    byte_6 = writed->QEI_sample & 0xff;

    // // value
    if (writed->QEI_value > 4095){
        writed->QEI_value = 0;
        printf("QEI_value over 0-4095!\n");
    }
    unsigned char byte_7, byte_8;
    byte_7 = writed->QEI_value >> 8;
    byte_8 = writed->QEI_value & 0xff;

    // write
    static unsigned char send[9] = {0x00,0x00,0x04,0x18,0x00,0x00,0x01,0x8f,0x09};
    send[0] = writed->gpio_out_1;
    send[1] = writed->gpio_out_2;
    send[2] = byte_3;
    send[3] = byte_4;
    send[4] = byte_5;
    send[5] = byte_6;
    send[6] = byte_7;
    send[7] = byte_8;
    send[8] = cal_crc_table(send,8);
    // for (int i = 0 ; i < 9; i++){
    //     printf("%x ", send[i]);
    // }
    // printf("\n");
    
    char ret = ioctl(*file, TYPE_OUT_WR, &send);
    if (ret < 0){
        printf("ioctl W faill!\n");
    }

}

int spi_read(spi_file_t file, spi_io_read_t *readed)
{
    // read
    unsigned char receive[12];
    char ret = ioctl(file, TYPE_R, receive);
    if (ret < 0){
        printf("ioctl R faill!\n");
    }

    // if (receive[10] != 1 && receive[10] != 0){
    //     // printf("No connect SPI or SPI is damaged!\n");
    //     return 1;
    // }

    readed->gpio_in_1 = receive[0];
    readed->gpio_in_2 = receive[1];
    readed->gpio_out_1 = receive[2];
    readed->gpio_out_2 = receive[3];

    *((unsigned char *)(&readed->EDAC_data)) = receive[5];
    *((unsigned char *)(&readed->EDAC_data) + 1) = receive[4];

    *((unsigned char *)(&readed->QEI_data)) = receive[9];
    *((unsigned char *)(&readed->QEI_data) + 1) = receive[8];
    *((unsigned char *)(&readed->QEI_data) + 2) = receive[7];
    *((unsigned char *)(&readed->QEI_data) + 3) = receive[6];

    readed->QEI_direction = receive[10];
    // for (int i = 0 ; i < 12; i++){
    //     printf("%x ", receive[i]);
    // }
    // printf("\n");

    return 0;
}

void spi_end(spi_file_t file, spi_io_write_t *writed)
{
    writed->gpio_out_1 = 0x00;
    writed->gpio_out_2 = 0x00;
    spi_write(&file, writed);
}

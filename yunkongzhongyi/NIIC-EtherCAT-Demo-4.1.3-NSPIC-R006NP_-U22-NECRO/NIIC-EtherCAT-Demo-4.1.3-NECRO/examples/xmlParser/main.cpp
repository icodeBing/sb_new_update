#include <ecat/master.hpp>
#include <ecat/task.hpp>
#include <iostream>

int main(int argc, const char** argv)  // xml解析器main函数
{
    printf("multi master station support.\n");

    int master_count = ecat::master::master_count();
    for (int i = 0; i < master_count; i++)
    {
        ecat::master m(i, ecat::master::read_write);
        ecat::master_info master_info = m.get_info();
        if (master_info.link_up && master_info.slave_count > 0)
        {
            printf("Master %u network is starting to resolve. Please wait...\n", i);
            ecat::task task(i);
            task.configure(1);
            printf("Master %u network resolution complete.\n", i);
        }
    }

    printf("Master network resolution complete, exit.\n");

    return 0;
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sendData2(char *data, int len)
{
    for(int i = 0; i<len; i++)
    {
        printf("%c", data[i]);
    }
    printf()

}

int main()
{
    char datan[] = "Wynik: \n";

    sendData2(datan, 10);


    return 0;
}

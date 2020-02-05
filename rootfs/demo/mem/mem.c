#include <stdio.h>  
#include <unistd.h>

unsigned char *p = NULL;
void mymalloc(int len)
{
    int i = 0;



    p= (unsigned char *)malloc(len);

    for(i=0;i<len;i++)
    	p[i] = 0x1;

    printf("malloc %d kbytes\n",len/1024);


    return;
}

void myfree(void)
{
    if(p != NULL)
    {
	printf("free memory\n");
	free(p);
	p = NULL;
    }


    return;
}


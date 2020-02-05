#include <stdio.h>  
#include <dlfcn.h>  
#include <stdlib.h>  `
#include <signal.h>
#include <errno.h>
#include <sys/types.h>
#include <string.h>

void catch_signal(int sign)
{
    void *bt[20]; 
    char **strings; 
    size_t sz;
    int i = 0;

    switch (sign)
    {
    case SIGINT:
	 
	 sz = backtrace(bt, 20); 
	 strings = backtrace_symbols(bt, sz); 
	 printf("size : %d\n",sz);
    	 for(i = 0; i < sz; ++i) 
         	fprintf(stderr, "%s/n", strings[i]);
        break;
    }
}

int main(int argc, char **argv) {  
	void *handle;  
	void (*mymalloc)(int);  
	void (*myfree)(void);  
	char *error;  
	
	int intervalSec;
	int sizeKB;

	signal(SIGINT, catch_signal);

	if(argc < 3)
	{
		printf("./a.out param1 param2\n");
		return -1;
	}

	sizeKB = atoi(argv[1]);
	if(sizeKB == 0)
	{
		printf("malloc size param invalid\n");
		return -1;
	}


	intervalSec = atoi(argv[2]);
	intervalSec = intervalSec > 0 ? intervalSec : 3; 


WORK:
	printf("intervalSec : %ds,sizeKB : %dKB\n",intervalSec,sizeKB);
	while(1)
	{
		handle = dlopen ("./libmalloc.so", RTLD_LAZY);  
		if (!handle) {  
			printf (stderr, "%s ", dlerror());  
			exit(1);  
		}  
	  
		mymalloc = (double(*)(double))dlsym(handle, "mymalloc");  
		if ((error = dlerror()) != NULL)  {  
			printf (stderr, "%s ", error);  
			exit(1);  
		}  
	  
		mymalloc(1024*sizeKB);
		sleep(intervalSec);
		#if 1
			myfree = (double(*)(double))dlsym(handle, "myfree");  
			if ((error = dlerror()) != NULL)  {  
				printf (stderr, "%s ", error);  
				exit(1);  
			} 
			myfree();
		#endif
		dlclose(handle);  
	}
	while(1);
	return 0;  
}  


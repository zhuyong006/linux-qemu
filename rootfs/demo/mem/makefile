malloc: 
	/home/jon/opt/FriendlyARM/toolschain/4.4.3/bin/arm-none-linux-gnueabi-gcc -g -fPIC -shared -o libmalloc.so mem.c
test: 
	/home/jon/opt/FriendlyARM/toolschain/4.4.3/bin/arm-none-linux-gnueabi-gcc -g main.c -L. -lmalloc  -fPIC -ldl
segv: 
	/home/jon/opt/FriendlyARM/toolschain/4.4.3/bin/arm-none-linux-gnueabi-gcc -g -ldl -rdynamic segfault.c -o segv

x86:
	gcc -rdynamic main_x86.c -o x86
clean:
	rm -rf *.out
	rm -rf *.so
	rm -rf x86
	rm -rf segv
all: malloc test x86 segv

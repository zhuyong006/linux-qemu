malloc: 
	/home/jon/opt/FriendlyARM/toolschain/4.4.3/bin/arm-none-linux-gnueabi-gcc -g -fPIC -shared -o libmalloc.so mem.c
test: 
	/home/jon/opt/FriendlyARM/toolschain/4.4.3/bin/arm-none-linux-gnueabi-gcc -g main.c -L. -lmalloc  -fPIC -ldl

clean:
	rm -rf *.out
	rm -rf *.so

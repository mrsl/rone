CC=arm-none-eabi-gcc
CFLAGS= -Wall \
		-O0 \
		-g3 \
		-mcpu=cortex-m3 \
		-mthumb

LINKER=arm-none-eabi-ar
LDFLAGS= 
LDLIBS= -l

SYMBOLS= -DGCC_ARMCM3
INCLUDES= -I"./inc"
SRCDIR= src/
OBJDIR= intermediate/

OBJS :=	croutine.o \
		heap_1.o \
		list.o \
		port_gcc.o \
		queue.o \
		tasks.o
		
all: freertos
	
freertos: $(OBJS)
	echo "Linking..."
	$(CC) $(CFLAGS) $(OBJS) -o libfreertos.a
	
$(OBJS): %.o: $(SRCDIR)/%.c
	echo "Compiling $@"
	@$(CC) $< -o $@ -c $(CFLAGS) $(INCLUDES)		
	
clean:
	rm *.o
	
clean_windows:
	del *.o